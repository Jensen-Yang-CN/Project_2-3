#include "BridgeDetectionNode.h"



#include "bridge_detection_worker.h"

#include "SystemFileLogger.h"

#include "common_types_extended.h"

#include "common/pointxyz.h"



#include <QDateTime>

#include <QImage>

#include <QMetaObject>

#include <QThread>

#include <QVector>



#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>



namespace {



QVector<M_PointXYZI> toQtCloud(const std::shared_ptr<const usv::LidarFrame> &src)

{

    QVector<M_PointXYZI> cloud;

    if (!src || src->points.empty())

        return cloud;

    cloud.reserve(static_cast<int>(src->points.size()));

    for (const usv::LidarPoint &p : src->points) {

        M_PointXYZI q;

        q.x = p.x;

        q.y = p.y;

        q.z = p.z;

        q.intensity = static_cast<int>(p.intensity);

        cloud.push_back(q);

    }

    return cloud;

}



QImage toQtImage(const std::shared_ptr<const usv::ImageFrame> &src)

{

    if (!src || src->image.empty())

        return QImage();

    cv::Mat rgb;

    if (src->image.channels() == 3)

        cv::cvtColor(src->image, rgb, cv::COLOR_BGR2RGB);

    else

        rgb = src->image;

    return QImage(rgb.data, rgb.cols, rgb.rows,

                  static_cast<int>(rgb.step), QImage::Format_RGB888)

        .copy();

}

QString formatTraditionalVisionResult(const usv::TraditionalBridgeVisionBusResult &result)
{
    const QString cameraName = result.camera_side == usv::CameraSide::Left
        ? QStringLiteral("左") : QStringLiteral("右");
    QStringList lines;
    lines << QStringLiteral("%1相机：桥墩=%2")
                 .arg(cameraName)
                 .arg(result.pier_count);

    for (const usv::BridgePierObservation &pier : result.piers) {
        lines << QStringLiteral("Pier-%1：方位角=%2°，像素中心=(%3,%4)")
                     .arg(pier.id)
                     .arg(pier.azimuth_deg, 0, 'f', 1)
                     .arg(qRound(pier.pixel_x))
                     .arg(qRound(pier.pixel_y));
    }

    if (result.has_bridge_opening) {
        lines << QStringLiteral("桥孔中心方位角=%1°")
                     .arg(result.bridge_opening_center_azimuth_deg, 0, 'f', 1);
        lines << QStringLiteral("桥墩夹角=%1°")
                     .arg(result.pier_angular_separation_deg, 0, 'f', 1);
    }

    lines << (result.has_deck_line
        ? QStringLiteral("桥面倾角=%1°").arg(result.deck_inclination_deg, 0, 'f', 1)
        : QStringLiteral("桥面倾角=无"));
    return lines.join(QLatin1Char('\n'));
}



} // namespace



BridgeDetectionNode::BridgeDetectionNode(QObject *parent)

    : QObject(parent)

    , comm_(&usv::CommunicationManager::instance())

{

}



BridgeDetectionNode::~BridgeDetectionNode()

{

    stop();

    if (worker_thread_ && worker_thread_->isRunning()) {
        worker_thread_->quit();
        worker_thread_->wait();
    }
    worker_ = nullptr;
    right_worker_ = nullptr;

    if (vision_result_topic_ && vision_result_sub_id_ != 0) {
        vision_result_topic_->unsubscribe(vision_result_sub_id_);
        vision_result_sub_id_ = 0;
    }

}



void BridgeDetectionNode::Init()

{

    sub_bundle_ = comm_->getStateTopic<usv::PreprocessedSensorBundle>(

        "/preprocess/synced_bundle", 16);

    pub_result_ = comm_->getStateTopic<usv::BridgeMeasureResult>(

        "/perception/bridge_result", 16);

    vision_result_topic_ = comm_->getEventTopic<usv::TraditionalBridgeVisionBusResult>(

        "/perception/traditional_bridge_vision", 32);



    sub_id_ = sub_bundle_->subscribe([this](std::shared_ptr<const usv::PreprocessedSensorBundle> msg) {

        if (msg && processing_enabled_.load(std::memory_order_acquire)) {
            sync_queue_.push({
                msg,
                playback_timeline_generation_.load(std::memory_order_acquire)});
        }

    });

    vision_result_sub_id_ = vision_result_topic_->subscribe(
        [this](std::shared_ptr<const usv::TraditionalBridgeVisionBusResult> msg) {
            if (msg
                && msg->timeline_generation
                    == playback_timeline_generation_.load(std::memory_order_acquire)
                && processing_enabled_.load(std::memory_order_acquire))
                postLog(formatTraditionalVisionResult(*msg));
        });

    SystemFileLogger::instance().logLifecycle("BridgeDetection", "ready");
}



void BridgeDetectionNode::postLog(const QString &text)

{

    QMetaObject::invokeMethod(

        this,

        [this, text]() { emit logMessage(text); },

        Qt::QueuedConnection);

}



void BridgeDetectionNode::Start()

{

    bool expected = false;

    if (!is_running_.compare_exchange_strong(expected, true))

        return;



    if (!worker_thread_) {

        worker_thread_ = new QThread(this);

        worker_ = new BridgeDetectionWorker(true);

        worker_->moveToThread(worker_thread_);

        right_worker_ = new BridgeDetectionWorker(false);
        right_worker_->moveToThread(worker_thread_);
        connect(worker_thread_, &QThread::finished,
                worker_, &QObject::deleteLater);
        connect(worker_thread_, &QThread::finished,
                right_worker_, &QObject::deleteLater);

        connect(worker_thread_, &QThread::started, worker_, &BridgeDetectionWorker::initialize);
        connect(worker_thread_, &QThread::started, right_worker_, &BridgeDetectionWorker::initialize);

        connect(worker_, &BridgeDetectionWorker::logMessage,

                this, &BridgeDetectionNode::logMessage, Qt::QueuedConnection);
        connect(right_worker_, &BridgeDetectionWorker::logMessage,
                this, &BridgeDetectionNode::logMessage, Qt::QueuedConnection);
        auto forwardTaskLog = [this](const QString &text, quint64 generation) {
            if (generation
                    == playback_timeline_generation_.load(std::memory_order_acquire)
                && is_running_.load(std::memory_order_acquire)
                && processing_enabled_.load(std::memory_order_acquire)) {
                emit logMessage(text);
            }
        };
        connect(worker_, &BridgeDetectionWorker::taskLogMessage,
                this, forwardTaskLog, Qt::QueuedConnection);
        connect(right_worker_, &BridgeDetectionWorker::taskLogMessage,
                this, forwardTaskLog, Qt::QueuedConnection);

        connect(worker_, &BridgeDetectionWorker::bridgeOverlayReady,
                this, [this](const QImage &overlay, double timestamp,
                             quint64 generation) {
                    if (is_running_.load()
                        && generation == playback_timeline_generation_.load(
                            std::memory_order_acquire)
                        && processing_enabled_.load(std::memory_order_acquire))
                        emit bridgeOverlayReady(overlay, timestamp);
                }, Qt::QueuedConnection);
        connect(right_worker_, &BridgeDetectionWorker::bridgeOverlayReady,
                this, [this](const QImage &overlay, double timestamp,
                             quint64 generation) {
                    if (is_running_.load()
                        && generation == playback_timeline_generation_.load(
                            std::memory_order_acquire)
                        && processing_enabled_.load(std::memory_order_acquire))
                        emit bridgeOverlayReadyRight(overlay, timestamp);
                }, Qt::QueuedConnection);
        connect(worker_, &BridgeDetectionWorker::traditionalDeckLineReady,
                this, [this](const QLineF &line, bool detected, double timestamp,
                             quint64 generation) {
                    if (is_running_.load()
                        && generation == playback_timeline_generation_.load(
                            std::memory_order_acquire)
                        && processing_enabled_.load(std::memory_order_acquire))
                        emit traditionalDeckLineReady(line, detected, timestamp);
                }, Qt::QueuedConnection);
        connect(right_worker_, &BridgeDetectionWorker::traditionalDeckLineReady,
                this, [this](const QLineF &line, bool detected, double timestamp,
                             quint64 generation) {
                    if (is_running_.load()
                        && generation == playback_timeline_generation_.load(
                            std::memory_order_acquire)
                        && processing_enabled_.load(std::memory_order_acquire))
                        emit traditionalDeckLineReadyRight(line, detected, timestamp);
                }, Qt::QueuedConnection);

        worker_thread_->start();

    }

    last_left_vision_submit_ms_.store(0, std::memory_order_relaxed);
    last_right_vision_submit_ms_.store(0, std::memory_order_relaxed);
    left_vision_pending_generation_.store(0, std::memory_order_relaxed);
    right_vision_pending_generation_.store(0, std::memory_order_relaxed);
    pointcloud_pending_generation_.store(0, std::memory_order_relaxed);


    sync_queue_.reopen();

    consumer_ = std::thread(&BridgeDetectionNode::processLoop, this);

    SystemFileLogger::instance().logLifecycle("BridgeDetection", "start");
    postLog(QStringLiteral("[桥洞检测] 检测线程已启动，等待预处理同步包…"));

}

void BridgeDetectionNode::submitCameraFrame(const QImage &image, double timestampSec, bool rightCamera)
{
    BridgeDetectionWorker *target = rightCamera ? right_worker_ : worker_;
    if (!is_running_.load()
        || !processing_enabled_.load(std::memory_order_acquire)
        || !target || image.isNull())
        return;

    // 只提交可被工作线程及时消费的检测帧。此前每秒约 50 帧/路全部排队，
    // 双路在同一线程上执行 OpenCV，队列越积越多，最终反压到 UDP 解码。
    std::atomic<qint64> &lastSubmit = rightCamera ? last_right_vision_submit_ms_
                                                   : last_left_vision_submit_ms_;
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    qint64 previous = lastSubmit.load(std::memory_order_relaxed);
    if (nowMs - previous < kVisionMinIntervalMs)
        return;
    if (!lastSubmit.compare_exchange_strong(previous, nowMs, std::memory_order_relaxed))
        return;

    const quint64 timeline_generation =
        playback_timeline_generation_.load(std::memory_order_acquire);
    std::atomic<quint64> &pendingGeneration = rightCamera
        ? right_vision_pending_generation_
        : left_vision_pending_generation_;
    quint64 expected_pending_generation = 0;
    if (!pendingGeneration.compare_exchange_strong(
            expected_pending_generation, timeline_generation,
            std::memory_order_acq_rel)) {
        return;
    }

    std::atomic<quint64> *pending_generation_ptr = &pendingGeneration;
    const bool queued = QMetaObject::invokeMethod(
        target,
        [this, target, image, timestampSec, pending_generation_ptr,
         timeline_generation]() {
            if (timeline_generation
                    != playback_timeline_generation_.load(std::memory_order_acquire)
                || !processing_enabled_.load(std::memory_order_acquire)) {
                return;
            }
            const QVector<M_PointXYZI> empty_cloud;
            target->runDetection(
                timestampSec, empty_cloud, empty_cloud, image,
                timeline_generation);
            quint64 owned_generation = timeline_generation;
            pending_generation_ptr->compare_exchange_strong(
                owned_generation, 0, std::memory_order_acq_rel);
        },
        Qt::QueuedConnection);
    if (!queued) {
        quint64 owned_generation = timeline_generation;
        pendingGeneration.compare_exchange_strong(
            owned_generation, 0, std::memory_order_acq_rel);
    }
}

void BridgeDetectionNode::setProcessingEnabled(bool enabled)
{
    const bool previous = processing_enabled_.exchange(
        enabled, std::memory_order_acq_rel);
    if (previous == enabled)
        return;

    // Invalidate both queued camera invocations and a point-cloud package that
    // may already have been popped by the consumer. The worker thread remains
    // reusable; standby therefore has no repeated model/thread construction.
    playback_timeline_generation_.fetch_add(1, std::memory_order_acq_rel);
    sync_queue_.clear();
    playback_timeline_reset_requested_.store(true, std::memory_order_release);
    last_left_vision_submit_ms_.store(0, std::memory_order_relaxed);
    last_right_vision_submit_ms_.store(0, std::memory_order_relaxed);
    left_vision_pending_generation_.store(0, std::memory_order_relaxed);
    right_vision_pending_generation_.store(0, std::memory_order_relaxed);
    pointcloud_pending_generation_.store(0, std::memory_order_relaxed);

    postLog(enabled
        ? QStringLiteral("[桥梁检测] 计算节点已恢复")
        : QStringLiteral("[桥梁检测] 计算已暂停"));
}

void BridgeDetectionNode::requestPlaybackTimelineReset()
{
    playback_timeline_generation_.fetch_add(1, std::memory_order_acq_rel);
    sync_queue_.clear();
    playback_timeline_reset_requested_.store(true, std::memory_order_release);
    last_left_vision_submit_ms_.store(0, std::memory_order_relaxed);
    last_right_vision_submit_ms_.store(0, std::memory_order_relaxed);

    if (worker_thread_ && worker_thread_->isRunning()) {
        if (worker_) {
            QMetaObject::invokeMethod(
                worker_, &BridgeDetectionWorker::resetForPlaybackSeek,
                Qt::BlockingQueuedConnection);
        }
        if (right_worker_) {
            QMetaObject::invokeMethod(
                right_worker_, &BridgeDetectionWorker::resetForPlaybackSeek,
                Qt::BlockingQueuedConnection);
        }
    }
    left_vision_pending_generation_.store(0, std::memory_order_relaxed);
    right_vision_pending_generation_.store(0, std::memory_order_relaxed);
    pointcloud_pending_generation_.store(0, std::memory_order_relaxed);
    postLog(QStringLiteral("[桥梁检测] PCAP 时间轴已跳转，视觉与点云跟踪状态已重置"));
}



void BridgeDetectionNode::stop()

{

    bool expected = true;

    if (!is_running_.compare_exchange_strong(expected, false))

        return;



    sync_queue_.shutdown();



    if (consumer_.joinable())

        consumer_.join();

    SystemFileLogger::instance().logLifecycle("BridgeDetection", "stop");

}



void BridgeDetectionNode::processLoop()

{

    while (is_running_) {

        ScheduledBundle scheduled;

        if (!sync_queue_.pop(scheduled) || !scheduled.bundle)

            continue;

        const usv::PreprocessedSensorBundlePtr &task = scheduled.bundle;

        if (!processing_enabled_.load(std::memory_order_acquire))
            continue;

        const quint64 timeline_generation = scheduled.timeline_generation;
        if (timeline_generation
            != playback_timeline_generation_.load(std::memory_order_acquire)) {
            continue;
        }

        if (playback_timeline_reset_requested_.exchange(
                false, std::memory_order_acq_rel)) {
            last_process_tp_ = {};
        }



        const auto now = std::chrono::steady_clock::now();

        if (last_process_tp_ != std::chrono::steady_clock::time_point{}) {

            const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(

                now - last_process_tp_).count();

            if (gap_ms < kProcessIntervalMs)

                continue;

        }

        last_process_tp_ = now;



        const QVector<M_PointXYZI> cloud210 = toQtCloud(task->deskewed_lidar_210);

        const QVector<M_PointXYZI> cloud211 = toQtCloud(task->deskewed_lidar_211);

        const QImage camera = toQtImage(task->synced_image);

        if (cloud210.isEmpty() && cloud211.isEmpty() && camera.isNull())

            continue;

        double ts = task->timestamp;
        if (task->synced_image && task->synced_image->timestamp > 0.0)
            ts = task->synced_image->timestamp;



        if (worker_) {
            // 预处理包可能比桥梁算法更快到达。只保留一个待执行的 Qt
            // 调用；已有任务完成后，下一轮再取最新包，避免事件队列堆积。
            quint64 expected_pending_generation = 0;
            if (!pointcloud_pending_generation_.compare_exchange_strong(
                    expected_pending_generation, timeline_generation,
                    std::memory_order_acq_rel)) {
                continue;
            }

            std::atomic<quint64> *pending_generation_ptr =
                &pointcloud_pending_generation_;
            const bool queued = QMetaObject::invokeMethod(
                worker_,
                [this, worker = worker_, timeline_generation, ts,
                 cloud210, cloud211, camera, pending_generation_ptr]() {
                    if (timeline_generation
                         == playback_timeline_generation_.load(
                            std::memory_order_acquire)
                        && processing_enabled_.load(
                            std::memory_order_acquire)) {
                        worker->runDetection(
                            ts, cloud210, cloud211, camera, timeline_generation);
                    }
                    // 仅清除自己预留的 generation，不能覆盖时间轴切换后
                    // 新任务已经取得的占位。
                    quint64 owned_generation = timeline_generation;
                    pending_generation_ptr->compare_exchange_strong(
                        owned_generation, 0, std::memory_order_acq_rel);
                },
                Qt::QueuedConnection);
            if (!queued) {
                quint64 owned_generation = timeline_generation;
                pointcloud_pending_generation_.compare_exchange_strong(
                    owned_generation, 0, std::memory_order_acq_rel);
            }
        }

    }

}

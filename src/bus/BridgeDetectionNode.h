#pragma once

#include <QObject>
#include <QString>
#include <QImage>
#include <QLineF>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "CommunicationManager_v2.h"
#include "ThreadSafeQueue.h"
#include "common_types_extended.h"

class BridgeDetectionWorker;
class QThread;

/** 订阅 /preprocess/synced_bundle，运行桥洞算法，发布 /perception/bridge_result */
class BridgeDetectionNode : public QObject {
    Q_OBJECT
public:
    explicit BridgeDetectionNode(QObject *parent = nullptr);
    ~BridgeDetectionNode() override;

    void Init();
    void Start();
    void stop();
    void setProcessingEnabled(bool enabled);
    bool processingEnabled() const
    {
        return processing_enabled_.load(std::memory_order_acquire);
    }
    void requestPlaybackTimelineReset();
    void submitCameraFrame(const QImage &image, double timestampSec, bool rightCamera = false);

signals:
    void logMessage(const QString &text);
    void bridgeOverlayReady(const QImage &overlay, double timestampSec);
    void bridgeOverlayReadyRight(const QImage &overlay, double timestampSec);
    void traditionalDeckLineReady(const QLineF &deckLine, bool detected, double timestampSec);
    void traditionalDeckLineReadyRight(const QLineF &deckLine, bool detected, double timestampSec);

private:
    struct ScheduledBundle {
        usv::PreprocessedSensorBundlePtr bundle;
        quint64 timeline_generation = 1;
    };

    void processLoop();
    void postLog(const QString &text);

    usv::CommunicationManager *comm_ = nullptr;
    std::shared_ptr<usv::StateTopic<usv::PreprocessedSensorBundle>> sub_bundle_;
    std::shared_ptr<usv::StateTopic<usv::BridgeMeasureResult>> pub_result_;
    std::shared_ptr<usv::EventTopic<usv::TraditionalBridgeVisionBusResult>> vision_result_topic_;
    uint64_t sub_id_ = 0;
    uint64_t vision_result_sub_id_ = 0;

    usv::ThreadSafeQueue<ScheduledBundle> sync_queue_{1};

    BridgeDetectionWorker *worker_ = nullptr;
    BridgeDetectionWorker *right_worker_ = nullptr;
    QThread *worker_thread_ = nullptr;

    // 视觉检测比视频解码重得多。只取最新的一部分帧，不能让 50 FPS 的相机
    // 在 Qt 事件队列中无限累积检测任务；原始视频仍然按全部帧显示。
    std::atomic<qint64> last_left_vision_submit_ms_{0};
    std::atomic<qint64> last_right_vision_submit_ms_{0};
    // 0 means idle; otherwise the value owns the pending slot for one
    // playback/enable generation. An old task cannot clear a newer owner.
    std::atomic<quint64> left_vision_pending_generation_{0};
    std::atomic<quint64> right_vision_pending_generation_{0};
    // 点云检测任务也通过同一 Qt 工作线程执行；只允许一个完整点云包
    // 在事件队列中等待，避免检测耗时较长时任务和图像数据无限堆积。
    std::atomic<quint64> pointcloud_pending_generation_{0};
    std::atomic<bool> playback_timeline_reset_requested_{false};
    std::atomic<quint64> playback_timeline_generation_{1};
    static constexpr qint64 kVisionMinIntervalMs = 125;

    std::thread consumer_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> processing_enabled_{false};

    static constexpr int kProcessIntervalMs = 500;
    std::chrono::steady_clock::time_point last_process_tp_{};
};

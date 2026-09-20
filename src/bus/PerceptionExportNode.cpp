#include "PerceptionExportNode.h"

#include "BerthProtocolGeometry.h"
#include "BerthUdpPacketBuilder.h"
#include "GnssInsGeoUtils.h"
#include "PerceptionUdpProtocol.h"
#ifdef ENABLE_SLAM
#include "SlamMapGeoUtils.h"
#include "SlamMapBerthGeometry.h"
#endif

#include <QDateTime>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QStringList>
#include <QDebug>
#include <QMetaObject>
#include <QtEndian>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
// 实时回放时同时投递当前 SLAM 关键帧；历史地图仍按原有历史队列投递。
// 泊位 0xFB 的几何与缓存逻辑独立于此开关。
constexpr bool kEnableRealtimeMapExport = true;
constexpr double kHistoricalMapResolutionM = 0.05;
// 实时泊位连续帧的检测抖动通常是米级；35 m 会把相邻泊位错误合并。
constexpr double kRealtimeBerthCacheMatchM = 6.0;

double geodeticDistanceMeters(double lon1, double lat1,
                              double lon2, double lat2)
{
    constexpr double kMetersPerDegree = 111320.0;
    const double meanLat = (lat1 + lat2) * 0.5 * kDegToRad;
    const double east = (lon1 - lon2) * kMetersPerDegree * std::cos(meanLat);
    const double north = (lat1 - lat2) * kMetersPerDegree;
    return std::hypot(east, north);
}

std::uint64_t hashBerthResult(const usv::BerthMeasureResult &result)
{
    // FNV-1a over the fields that affect the outgoing 0xFB geometry.  The
    // result timestamp alone is not enough because a detector may publish a
    // corrected geometry with the same sensor timestamp.
    std::uint64_t hash = 1469598103934665603ULL;
    const auto mixBytes = [&hash](const void *data, std::size_t size) {
        const auto *bytes = static_cast<const unsigned char *>(data);
        for (std::size_t i = 0; i < size; ++i) {
            hash ^= static_cast<std::uint64_t>(bytes[i]);
            hash *= 1099511628211ULL;
        }
    };
    mixBytes(&result.timestamp, sizeof(result.timestamp));
    const std::uint64_t count = result.expanded_berths.size();
    mixBytes(&count, sizeof(count));
    for (const usv::Berth &berth : result.expanded_berths) {
        mixBytes(&berth.cx, sizeof(berth.cx));
        mixBytes(&berth.cy, sizeof(berth.cy));
        mixBytes(&berth.w, sizeof(berth.w));
        mixBytes(&berth.l, sizeof(berth.l));
        mixBytes(&berth.angle, sizeof(berth.angle));
        mixBytes(&berth.opening_edge, sizeof(berth.opening_edge));
        const int kind = static_cast<int>(berth.kind);
        mixBytes(&kind, sizeof(kind));
    }
    return hash;
}

int32_t encodeGeo(double deg)
{
    return static_cast<int32_t>(std::llround(deg * PerceptionUdpConstants::kGeoScale));
}

int16_t encodeAlt(double meters)
{
    return static_cast<int16_t>(std::clamp(
        std::llround(meters * PerceptionUdpConstants::kAltScale),
        static_cast<long long>(INT16_MIN), static_cast<long long>(INT16_MAX)));
}

uint16_t encodeAzimuth(double deg)
{
    double wrapped = std::fmod(deg, 360.0);
    if (wrapped < 0.0)
        wrapped += 360.0;
    return static_cast<uint16_t>(std::llround(
        wrapped * PerceptionUdpConstants::kAzimuthScale));
}

void fillPrefix(uint8_t *prefix)
{
    std::memcpy(prefix, PerceptionUdpConstants::kPrefix, 4);
}

int floorDivLocal(int value, int divisor)
{
    if (divisor <= 0)
        return 0;
    if (value >= 0)
        return value / divisor;
    return (value - divisor + 1) / divisor;
}

}  // namespace

int PerceptionExportNode::floorDiv(int value, int divisor)
{
    return floorDivLocal(value, divisor);
}

void PerceptionExportNode::captureRealtimeMapReference(
    const usv::SlamKeyframe &keyframe)
{
    if (has_realtime_map_reference_
        || !keyframe.geo_reference.valid
        || !keyframe.geo_reference.pose_lidar_enu.matrix().allFinite()
        || !keyframe.pose.matrix().allFinite()) {
        return;
    }

    // keyframe.pose is T_map_lidar, while geo_reference is T_enu_lidar.
    // Their product gives the fixed T_enu_map used for all subsequent 0xFC
    // packets.  The transform is captured once per playback/map session.
    const Eigen::Isometry3d map_to_enu =
        keyframe.geo_reference.pose_lidar_enu * keyframe.pose.inverse();
    if (!map_to_enu.matrix().allFinite())
        return;

    if (!realtime_map_anchor_.valid) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (gnss_enu_state_.initialized) {
            const usv::GnssInsMessage &origin = gnss_enu_state_.origin_gnss;
            realtime_map_anchor_.valid = true;
            realtime_map_anchor_.timestamp = origin.timestamp;
            realtime_map_anchor_.latitude_deg = origin.latitude;
            realtime_map_anchor_.longitude_deg = origin.longitude;
            realtime_map_anchor_.altitude_m = origin.altitude;
        }
    }

    if (!realtime_map_anchor_.valid)
        return;

    realtime_map_to_enu_ = map_to_enu;
    has_realtime_map_reference_ = true;
    qInfo() << "[信息投递] 实时地图固定参考系已建立：SLAM map -> ENU"
            << "anchor=" << realtime_map_anchor_.longitude_deg
            << realtime_map_anchor_.latitude_deg;
}

Eigen::Isometry3d PerceptionExportNode::realtimeMapPose(
    const Eigen::Isometry3d &pose) const
{
    return has_realtime_map_reference_ ? realtime_map_to_enu_ * pose : pose;
}

const usv::SlamGeoAnchor *PerceptionExportNode::realtimeMapAnchor() const
{
    return realtime_map_anchor_.valid ? &realtime_map_anchor_ : nullptr;
}

void PerceptionExportNode::mergeCloudIntoAccumulatedLocked(
    const std::vector<M_PointXYZI> &cloud,
    const Eigen::Isometry3d &pose)
{
    const double resolution = PerceptionUdpConstants::kDefaultResolutionM;
    const double inv_res = 1.0 / resolution;

    for (const M_PointXYZI &p : cloud) {
        const Eigen::Vector3d wp = pose * Eigen::Vector3d(p.x, p.y, p.z);
        const int gx = static_cast<int>(std::floor(wp.x() * inv_res));
        const int gy = static_cast<int>(std::floor(wp.y() * inv_res));
        const int gz = static_cast<int>(std::floor(wp.z() * inv_res));

        ChunkKey key{
            floorDiv(gx, PerceptionUdpConstants::kChunkXLen),
            floorDiv(gy, PerceptionUdpConstants::kChunkYLen),
            floorDiv(gz, PerceptionUdpConstants::kChunkZLen),
        };

        auto &bits = accumulated_chunks_[key];
        if (bits.empty())
            bits.assign(1280, 0);

        const int local_x = gx - key.x * PerceptionUdpConstants::kChunkXLen;
        const int local_y = gy - key.y * PerceptionUdpConstants::kChunkYLen;
        const int local_z = gz - key.z * PerceptionUdpConstants::kChunkZLen;
        if (local_x < 0 || local_x >= PerceptionUdpConstants::kChunkXLen
            || local_y < 0 || local_y >= PerceptionUdpConstants::kChunkYLen
            || local_z < 0 || local_z >= PerceptionUdpConstants::kChunkZLen) {
            continue;
        }

        const int index = local_x + local_y * PerceptionUdpConstants::kChunkXLen
                        + local_z * PerceptionUdpConstants::kChunkXLen
                               * PerceptionUdpConstants::kChunkYLen;
        const int byte_index = index >> 3;
        const int bit_index = index & 7;
        bits[static_cast<size_t>(byte_index)] |= static_cast<uint8_t>(1 << bit_index);
    }
}

void PerceptionExportNode::mergeCloudIntoAccumulated(
    const std::vector<M_PointXYZI> &cloud,
    const Eigen::Isometry3d &pose)
{
    if (cloud.empty())
        return;
    mergeCloudIntoAccumulatedLocked(cloud, pose);
}

void PerceptionExportNode::enqueueFullAccumulatedMapResync()
{
    if (!running_.load() || !config_.send_map || !kEnableRealtimeMapExport
        || accumulated_chunks_.empty())
        return;

    const double ts = has_last_map_keyframe_ ? last_map_keyframe_.timestamp : 0.0;
    enqueueMapPackets(buildAccumulatedMapPackets(), ts,
                      QStringLiteral("全量同步"));
}

void PerceptionExportNode::ingestSlamKeyframes(
    const std::vector<usv::SlamKeyframe> &keyframes)
{
    if (keyframes.empty())
        return;

    if (kEnableRealtimeMapExport) {
        for (const usv::SlamKeyframe &kf : keyframes) {
            captureRealtimeMapReference(kf);
            mergeCloudIntoAccumulated(kf.cloud, realtimeMapPose(kf.pose));
        }
    }

    last_map_keyframe_ = keyframes.back();
    has_last_map_keyframe_ = true;
    enqueueFullAccumulatedMapResync();
}

#ifdef ENABLE_SLAM
bool PerceptionExportNode::loadHistoricalMap(const QString &manifestPath,
                                             QString *error)
{
    history_map_export::HistoricalMapExportSource candidate;
    if (!candidate.openManifest(manifestPath, error))
        return false;

    historical_map_source_ = std::move(candidate);
    historical_map_source_.resetFullMapCursor();
    historical_berths_sent_ = false;
    last_historical_berths_send_ms_ = 0;
    historical_map_packet_queue_.clear();
    historical_initial_stream_complete_ = false;
    realtime_berth_geo_cache_.clear();
    trySendHistoricalBerths();
    if (running_.load() && config_.send_map) {
        pumpHistoricalMapProducer();
        kickMapSendPump();
    }
    return true;
}

void PerceptionExportNode::clearHistoricalMap()
{
    historical_map_source_.close();
    historical_berths_sent_ = false;
    last_historical_berths_send_ms_ = 0;
    historical_map_packet_queue_.clear();
    historical_initial_stream_complete_ = true;
    realtime_berth_geo_cache_.clear();
}
#endif

PerceptionExportNode::PerceptionExportNode(QObject *parent)
    : QObject(parent)
{
    map_drip_timer_.setInterval(kMapDripIntervalMs);
    map_drip_timer_.setSingleShot(false);
    connect(&map_drip_timer_, &QTimer::timeout,
            this, &PerceptionExportNode::processMapSendPump);

    map_resend_timer_.setInterval(static_cast<int>(kMapResendIntervalMs));
    map_resend_timer_.setSingleShot(false);
    connect(&map_resend_timer_, &QTimer::timeout,
            this, &PerceptionExportNode::onMapResendTimer);

    imu_poll_timer_.setInterval(kImuPollIntervalMs);
    imu_poll_timer_.setSingleShot(false);
    connect(&imu_poll_timer_, &QTimer::timeout,
            this, &PerceptionExportNode::pollImuBus);
}

void PerceptionExportNode::loadStaticBerthLibrary()
{
    static_berth_units_.clear();
    static_berth_library_loaded_ = false;
    last_static_berths_send_ms_ = 0;

    QStringList candidates;
    const auto addCandidate = [&candidates](const QString &path) {
        if (path.isEmpty())
            return;
        const QString normalized = QDir::cleanPath(path);
        if (!candidates.contains(normalized))
            candidates.push_back(normalized);
    };
    addCandidate(config_.static_berth_library_path);
    addCandidate(QCoreApplication::applicationDirPath()
                 + QStringLiteral("/泊位检测完整地理信息_20260901.json"));
    addCandidate(QDir::current().filePath(
        QStringLiteral("泊位检测完整地理信息_20260901.json")));

    QString lastError;
    for (const QString &candidate : candidates) {
        if (!QFileInfo::exists(candidate))
            continue;
        static_berth_library::LoadResult loaded;
        QString error;
        if (!static_berth_library::load(candidate, loaded, &error)) {
            lastError = error;
            continue;
        }
        static_berth_units_ = std::move(loaded.units);
        static_berth_library_loaded_ = !static_berth_units_.empty();
        qInfo() << "[固定泊位图层] 已加载" << static_berth_units_.size()
                << "个泊位:" << candidate;
        emit logMessage(QStringLiteral(
            "[固定泊位图层] 已加载 %1 个泊位：%2")
                            .arg(static_cast<qulonglong>(
                                static_berth_units_.size()))
                            .arg(candidate));
        return;
    }

    if (!lastError.isEmpty()) {
        qWarning() << "[固定泊位图层] 加载失败:" << lastError;
        emit logMessage(QStringLiteral("[固定泊位图层] 加载失败：%1")
                            .arg(lastError));
    } else {
        qInfo() << "[固定泊位图层] 未找到泊位 JSON，保留原有泊位投递逻辑";
    }
}

void PerceptionExportNode::sendStaticBerthLibraryIfDue(bool force)
{
    if (!running_.load() || static_berth_units_.empty())
        return;
    const bool reportSend = force || last_static_berths_send_ms_ == 0;
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (!force && last_static_berths_send_ms_ > 0
        && now_ms - last_static_berths_send_ms_
               < kStaticBerthResendIntervalMs) {
        return;
    }

    const std::vector<QByteArray> packets = berth_udp::buildBerthPackets(
        static_cast<int32_t>(QDateTime::currentSecsSinceEpoch()),
        berth_frame_id_++, static_berth_units_);
    bool success = true;
    for (const QByteArray &packet : packets) {
        if (!sendUdpPayload(packet, "0xFB")) {
            success = false;
            break;
        }
    }
    if (!success)
        return;

    last_static_berths_send_ms_ = now_ms;
    qInfo() << "[固定泊位图层] 已发送" << static_berth_units_.size()
            << "个泊位（0xFB）";
    if (reportSend) {
        emit logMessage(QStringLiteral(
            "[固定泊位图层] 已发送 %1 个泊位（0xFB）")
                            .arg(static_cast<qulonglong>(
                                static_berth_units_.size())));
    }
}

void PerceptionExportNode::setConfig(const Config &cfg)
{
    config_ = cfg;
}

PerceptionExportNode::Config PerceptionExportNode::config() const
{
    return config_;
}

void PerceptionExportNode::start()
{
    if (running_.exchange(true))
        return;

    has_last_berth_signature_ = false;
    last_berth_signature_ = 0;
    last_berth_signature_ms_ = 0;
    realtime_berth_geo_cache_.clear();
    loadStaticBerthLibrary();

    if (config_.bind_local && config_.local_host.protocol() == QAbstractSocket::IPv4Protocol) {
        if (!socket_.bind(config_.local_host, config_.local_port,
                          QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
            const QString reason = QStringLiteral("本机 UDP 绑定失败 %1:%2：%3")
                .arg(config_.local_host.toString())
                .arg(config_.local_port)
                .arg(socket_.errorString());
            qWarning() << "[信息投递]" << reason;
            emit logMessage(QStringLiteral("[信息投递] %1").arg(reason));
            running_ = false;
            return;
        }
    }

    emit logMessage(QStringLiteral("[信息投递] UDP 已启动：%1:%2 -> %3:%4")
                        .arg(config_.local_host.toString())
                        .arg(config_.local_port)
                        .arg(config_.remote_host.toString())
                        .arg(config_.remote_port));

    subscribeImuBus();
    subscribeBerthBus();
#ifdef ENABLE_SLAM
    subscribeSlamBus();
#endif

    if (!kEnableRealtimeMapExport) {
        qInfo() << "[信息投递] 实时 SLAM 地图投递已临时关闭，仅投递历史地图与泊位结果";
    } else {
        qInfo() << "[信息投递] 实时 SLAM 地图投递已启用（关键帧/扫描点云）";
    }

    if (config_.send_map) {
        if (kEnableRealtimeMapExport)
            map_resend_timer_.start();
#ifdef ENABLE_SLAM
        historical_berths_sent_ = false;
        last_historical_berths_send_ms_ = 0;
        if (historical_map_source_.isOpen()) {
            historical_map_source_.resetFullMapCursor();
            historical_map_packet_queue_.clear();
            historical_initial_stream_complete_ = false;
            pumpHistoricalMapProducer();
            kickMapSendPump();
        }
#endif
    }
}

void PerceptionExportNode::stop()
{
    if (!running_.exchange(false))
        return;
    unsubscribeImuBus();
    unsubscribeBerthBus();
#ifdef ENABLE_SLAM
    unsubscribeSlamBus();
#endif
    clearMapSendState();
    socket_.close();
    has_last_berth_signature_ = false;
    last_berth_signature_ = 0;
    last_berth_signature_ms_ = 0;
    realtime_berth_geo_cache_.clear();
    static_berth_units_.clear();
    static_berth_library_loaded_ = false;
    last_static_berths_send_ms_ = 0;
}

void PerceptionExportNode::subscribeImuBus()
{
    auto &comm = usv::CommunicationManager::instance();
    imu_topic_ = comm.getRawTopic<usv::GnssInsMessage>("/sensor/imu/raw", 64);
    if (!imu_topic_)
        return;

    imu_subscriber_id_ = imu_topic_->addSubscriber();
    imu_topic_->seekSubscriberToLatest(imu_subscriber_id_);
    if (auto latest = comm.getLatestState<usv::GnssInsMessage>())
        feedGnssIns(*latest);
    imu_poll_timer_.start();
}

void PerceptionExportNode::unsubscribeImuBus()
{
    imu_poll_timer_.stop();
    if (imu_topic_ && imu_subscriber_id_ != 0) {
        imu_topic_->removeSubscriber(imu_subscriber_id_);
        imu_subscriber_id_ = 0;
    }
    imu_topic_.reset();
}

void PerceptionExportNode::subscribeBerthBus()
{
    auto &comm = usv::CommunicationManager::instance();
    berth_topic_ = comm.getStateTopic<usv::BerthMeasureResult>(
        "/perception/berth_result", 16);
    if (!berth_topic_)
        return;

    berth_subscriber_id_ = berth_topic_->subscribe(
        [this](std::shared_ptr<const usv::BerthMeasureResult> message) {
            if (!message)
                return;
            const usv::BerthMeasureResult copy = *message;
            // StateTopic 的回调运行在发布者线程；投递节点对象属于自己的
            // QThread，必须把结果排回该线程，避免并发访问 QUdpSocket。
            QMetaObject::invokeMethod(
                this,
                [this, copy]() { onBerthResult(copy); },
                Qt::QueuedConnection);
        });

    // 启动投递晚于泊位检测时，立即补发当前最新结果，不必等待下一帧。
    if (auto latest = berth_topic_->getLatest()) {
        const usv::BerthMeasureResult copy = *latest;
        QMetaObject::invokeMethod(
            this,
            [this, copy]() { onBerthResult(copy); },
            Qt::QueuedConnection);
    }
    emit logMessage(QStringLiteral(
        "[信息投递] 已订阅 /perception/berth_result（0xFB 泊位不再依赖界面转接）"));
}

void PerceptionExportNode::unsubscribeBerthBus()
{
    if (berth_topic_ && berth_subscriber_id_ != 0) {
        berth_topic_->unsubscribe(berth_subscriber_id_);
        berth_subscriber_id_ = 0;
    }
    berth_topic_.reset();
}

#ifdef ENABLE_SLAM
void PerceptionExportNode::subscribeSlamBus()
{
    auto &comm = usv::CommunicationManager::instance();

    slam_odom_topic_ = comm.getStateTopic<usv::SlamOdometryState>(
        "slam/odometry", 16);
    if (slam_odom_topic_) {
        slam_odom_subscriber_id_ = slam_odom_topic_->subscribe(
            [this](std::shared_ptr<const usv::SlamOdometryState> message) {
                if (!message)
                    return;
                const usv::SlamOdometryState copy = *message;
                QMetaObject::invokeMethod(
                    this,
                    [this, copy]() {
                        if (running_.load())
                            onSlamOdometry(copy);
                    },
                    Qt::QueuedConnection);
            });
    }

    slam_keyframe_topic_ = comm.getStateTopic<usv::SlamKeyframe>(
        "slam/keyframe", 32);
    if (slam_keyframe_topic_) {
        slam_keyframe_subscriber_id_ = slam_keyframe_topic_->subscribe(
            [this](std::shared_ptr<const usv::SlamKeyframe> message) {
                if (!message)
                    return;
                const usv::SlamKeyframe copy = *message;
                QMetaObject::invokeMethod(
                    this,
                    [this, copy]() {
                        if (running_.load())
                            onSlamKeyframe(copy);
                    },
                    Qt::QueuedConnection);
            });
    }

    slam_scan_topic_ = comm.getStateTopic<usv::SlamScanCloudMessage>(
        "slam/scan_cloud", 8);
    if (slam_scan_topic_) {
        slam_scan_subscriber_id_ = slam_scan_topic_->subscribe(
            [this](std::shared_ptr<const usv::SlamScanCloudMessage> message) {
                if (!message || message->points.empty())
                    return;
                QVector<M_PointXYZI> cloud;
                cloud.reserve(static_cast<int>(message->points.size()));
                for (const M_PointXYZI &point : message->points)
                    cloud.append(point);
                QMetaObject::invokeMethod(
                    this,
                    [this, cloud = std::move(cloud)]() mutable {
                        if (running_.load())
                            onSlamScanCloud(std::move(cloud));
                    },
                    Qt::QueuedConnection);
            });
    }

    emit logMessage(QStringLiteral(
        "[信息投递] 已订阅 slam/odometry、slam/keyframe、slam/scan_cloud（实时地图）"));
}

void PerceptionExportNode::unsubscribeSlamBus()
{
    if (slam_odom_topic_ && slam_odom_subscriber_id_ != 0)
        slam_odom_topic_->unsubscribe(slam_odom_subscriber_id_);
    if (slam_keyframe_topic_ && slam_keyframe_subscriber_id_ != 0)
        slam_keyframe_topic_->unsubscribe(slam_keyframe_subscriber_id_);
    if (slam_scan_topic_ && slam_scan_subscriber_id_ != 0)
        slam_scan_topic_->unsubscribe(slam_scan_subscriber_id_);
    slam_odom_subscriber_id_ = 0;
    slam_keyframe_subscriber_id_ = 0;
    slam_scan_subscriber_id_ = 0;
    slam_odom_topic_.reset();
    slam_keyframe_topic_.reset();
    slam_scan_topic_.reset();
}
#endif

void PerceptionExportNode::pollImuBus()
{
    if (!imu_topic_ || imu_subscriber_id_ == 0)
        return;

    while (true) {
        auto opt_msg = imu_topic_->tryConsumeNext(imu_subscriber_id_);
        if (!opt_msg || !(*opt_msg))
            break;
        feedGnssIns(**opt_msg);
    }
}

void PerceptionExportNode::clearMapSendState()
{
    map_drip_timer_.stop();
    map_resend_timer_.stop();
    map_packet_queue_.clear();
#ifdef ENABLE_SLAM
    historical_map_packet_queue_.clear();
    // 投递节点重启后，历史泊位需要和历史 Tile 一样重新发送。
    historical_berths_sent_ = false;
    last_historical_berths_send_ms_ = 0;
#endif
    map_queue_batch_total_ = 0;
    map_queue_batch_sent_ = 0;
    map_queue_batch_failed_ = 0;
    map_queue_batch_tag_.clear();
}

void PerceptionExportNode::onMapResendTimer()
{
    enqueueFullAccumulatedMapResync();
}

void PerceptionExportNode::feedImu(const IMUParsedData &imu)
{
    feedGnssIns(usv::gnss_geo::gnssFromImu(imu));
}

void PerceptionExportNode::feedGnssIns(const usv::GnssInsMessage &gnss)
{
    if (gnss.longitude == 0.0 && gnss.latitude == 0.0)
        return;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_gnss_ins_ = gnss;
        has_imu_ = true;
        recordGnssBodySample(gnss);
        if (!realtime_map_anchor_.valid && gnss_enu_state_.initialized) {
            const usv::GnssInsMessage &origin = gnss_enu_state_.origin_gnss;
            realtime_map_anchor_.valid = true;
            realtime_map_anchor_.timestamp = origin.timestamp;
            realtime_map_anchor_.latitude_deg = origin.latitude;
            realtime_map_anchor_.longitude_deg = origin.longitude;
            realtime_map_anchor_.altitude_m = origin.altitude;
        }
    }
#ifdef ENABLE_SLAM
    // 地图可能先于导航数据加载；每次收到有效导航数据都尝试一次，
    // 首次成功后由标志位抑制重复投递。
    trySendHistoricalBerths();
#endif
}

void PerceptionExportNode::recordGnssBodySample(const usv::GnssInsMessage &gnss)
{
    usv::gnss_geo::updateGnssEnuState(gnss, gnss_enu_state_);
    if (!gnss_enu_state_.initialized)
        return;

    const Eigen::Isometry3d body_pose =
        usv::gnss_geo::bodyPoseInEnu(gnss, gnss_enu_state_);

    if (!gnss_body_history_.empty()
        && std::abs(gnss_body_history_.back().timestamp - gnss.timestamp) < 1e-4) {
        gnss_body_history_.back().timestamp = gnss.timestamp;
        gnss_body_history_.back().body_pose = body_pose;
        gnss_body_history_.back().enu_state = gnss_enu_state_;
        return;
    }

    GnssBodySample sample;
    sample.timestamp = gnss.timestamp;
    sample.body_pose = body_pose;
    sample.enu_state = gnss_enu_state_;
    gnss_body_history_.push_back(sample);
    while (gnss_body_history_.size() > kMaxGnssBodyHistory)
        gnss_body_history_.pop_front();
}

bool PerceptionExportNode::lookupGnssBodyPose(
    double timestamp,
    Eigen::Isometry3d &body_pose,
    usv::gnss_geo::GnssEnuState &enu_state) const
{
    // 泊位结果可能在回放线程先于导航历史进入投递线程。只要当前有一
    // 个有效导航状态，就可以用它完成一次保守的坐标换算，不能因为
    // 历史队列尚未填充而把整批 0xFB 静默丢掉。
    if (gnss_body_history_.empty()) {
        if (!imuValidLocked() || !gnss_enu_state_.initialized)
            return false;
        body_pose = usv::gnss_geo::bodyPoseInEnu(
            latest_gnss_ins_, gnss_enu_state_);
        enu_state = gnss_enu_state_;
        return true;
    }

    double best_dt = 1e9;
    const GnssBodySample *best = nullptr;
    for (const GnssBodySample &sample : gnss_body_history_) {
        const double dt = std::abs(sample.timestamp - timestamp);
        if (dt < best_dt) {
            best_dt = dt;
            best = &sample;
        }
    }

    if (!best)
        return false;

    if (best_dt > kMaxGnssSyncSec) {
        // Offline PCAP replay can publish several sensor frames between two
        // Qt timer turns.  The exact timestamp then falls outside the strict
        // 5 s window even though the newest navigation sample is still the
        // only usable pose.  Allow a bounded fallback instead of silently
        // dropping every 0xFB packet; the caller still gets a diagnostic log.
        constexpr double kMaxGnssReplayFallbackSec = 30.0;
        if (best_dt > kMaxGnssReplayFallbackSec) {
            // 长时间跳播/暂停时，队列中的历史样本可能已经过期；仍然
            // 使用最新有效姿态发送当前泊位，接收端至少能收到几何，
            // 下一帧导航恢复后会自动回到严格匹配。
            if (!imuValidLocked() || !gnss_enu_state_.initialized)
                return false;
            body_pose = usv::gnss_geo::bodyPoseInEnu(
                latest_gnss_ins_, gnss_enu_state_);
            enu_state = gnss_enu_state_;
            const qint64 now = QDateTime::currentMSecsSinceEpoch();
            if (now - last_gnss_fallback_log_ms_ >= 1000) {
                last_gnss_fallback_log_ms_ = now;
                qWarning() << "[信息投递][0xFB] 泊位时间戳与导航相差"
                           << best_dt << "s，使用最新有效姿态发送";
            }
            return true;
        }
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (now - last_gnss_fallback_log_ms_ >= 1000) {
            last_gnss_fallback_log_ms_ = now;
            qWarning() << "[信息投递][0xFB] GNSS 时间差" << best_dt
                       << "s，使用最近导航姿态（回放兜底）";
        }
    }

    body_pose = best->body_pose;
    enu_state = best->enu_state;
    return true;
}

bool PerceptionExportNode::imuValidLocked() const
{
    return has_imu_
        && !(latest_gnss_ins_.longitude == 0.0 && latest_gnss_ins_.latitude == 0.0);
}

#ifdef ENABLE_SLAM
void PerceptionExportNode::trySendHistoricalBerths()
{
    if (!running_.load() || !config_.send_map
        || !historical_map_source_.isOpen()) {
        return;
    }
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (historical_berths_sent_
        && now_ms - last_historical_berths_send_ms_
               < kHistoricalBerthResendIntervalMs) {
        return;
    }

    const QVector<usv::SlamMapBerth> &records =
        historical_map_source_.berths();
    if (records.isEmpty()) {
        sendStaticBerthLibraryIfDue();
        // 没有泊位图层时按重发周期记一次，避免每帧导航数据刷日志。
        historical_berths_sent_ = true;
        last_historical_berths_send_ms_ = now_ms;
        qInfo() << "[历史泊位投递] manifest 未包含持久化泊位记录，"
                << "固定泊位图层已单独处理";
        return;
    }

    // 0xFC 历史点云的坐标原点是 manifest.anchor。泊位也直接在同一个
    // ENU 原点下编码，避免先换到当前导航原点、发送时又换回历史原点，
    // 导致接收端的框与历史点云出现平移。
    const usv::SlamGeoAnchor &historyAnchor =
        historical_map_source_.anchor();
    if (!slam_map_geo::validAnchor(historyAnchor)) {
        qWarning() << "[历史泊位投递] 没有有效历史 ENU 锚点，暂不发送";
        return;
    }
    usv::GnssInsMessage historyGnss;
    historyGnss.timestamp = historyAnchor.timestamp;
    historyGnss.latitude = historyAnchor.latitude_deg;
    historyGnss.longitude = historyAnchor.longitude_deg;
    historyGnss.altitude = historyAnchor.altitude_m;
    historyGnss.yaw = 0.0;
    historyGnss.pitch = 0.0;
    historyGnss.roll = 0.0;
    usv::gnss_geo::GnssEnuState historyState;
    historyState.initialized = true;
    historyState.origin_gnss = historyGnss;
    historyState.last_gnss = historyGnss;
    historyState.accumulated_enu.setZero();
    // 历史泊位已经存储在历史地图 ENU 平面中。不要先把它伪装成当前
    // 雷达二维坐标再经过一次带俯仰/横滚的雷达外参，否则角点会离开
    // 原始 ENU 平面，接收端看到的开口边可能被旋转/重排。
    const bool useStaticLibrary = !static_berth_units_.empty();
    std::vector<BerthUdpUnit> units;
    if (useStaticLibrary) {
        units = static_berth_units_;
    } else {
        units.reserve(std::min<std::size_t>(
            records.size(), PerceptionUdpConstants::kMaxBerthsPerBatch));
    }

    for (const usv::SlamMapBerth &record : records) {
        if (useStaticLibrary)
            break;
        if (!std::isfinite(record.x) || !std::isfinite(record.y)
            || !std::isfinite(record.z)
            || !std::isfinite(record.width) || !std::isfinite(record.length)
            || !std::isfinite(record.angle_deg)
            || record.width <= 0.0 || record.length <= 0.0
            || (record.kind != static_cast<uint8_t>(usv::BerthKind::UShape)
                && record.kind != static_cast<uint8_t>(usv::BerthKind::Line))) {
            continue;
        }

        const double angleRad = record.angle_deg * kDegToRad;
        const double cosA = std::cos(angleRad);
        const double sinA = std::sin(angleRad);
        const double halfW = record.width * 0.5;
        const double halfL = record.length * 0.5;
        const double localX[4] = {-halfW, halfW, halfW, -halfW};
        const double localY[4] = {-halfL, -halfL, halfL, halfL};
        Eigen::Vector3d corners[4];
        for (int c = 0; c < 4; ++c) {
            corners[c] = Eigen::Vector3d(
                record.x + cosA * localX[c] - sinA * localY[c],
                record.y + sinA * localX[c] + cosA * localY[c],
                record.z);
        }

        usv::Berth berth;
        berth.w = record.width;
        berth.l = record.length;
        berth.kind = static_cast<usv::BerthKind>(record.kind);
        berth.source = usv::BerthDetectionSource::CoordinateLibrary;
        berth.opening_edge = static_cast<int>(record.opening_edge);
        if (berth.opening_edge < 0 || berth.opening_edge >= 4)
            berth.opening_edge = 0;

        std::array<berth_udp::BerthProtocolPoint, 4> protocol_points{};
        for (int pi = 0; pi < 4; ++pi) {
            // 协议约定点 1/4 在开口侧，点 2/3 在内部侧。
            const int c = (berth.opening_edge + 1 + pi) % 4;
            double lat = 0.0;
            double lon = 0.0;
            double alt = 0.0;
            usv::gnss_geo::enuToGeodetic(
                corners[c], historyState, lat, lon, alt);
            berth_udp::BerthProtocolPoint &point =
                protocol_points[static_cast<std::size_t>(pi)];
            point.longitude_deg = lon;
            point.latitude_deg = lat;
            point.altitude_m = alt;
            // 0xFB 的本地坐标字段在历史泊位中明确使用历史 ENU，
            // 这样接收端在经纬度不可用时也不会回退到当前船体系。
            point.x_m = corners[c].x();
            point.y_m = corners[c].y();
            point.z_m = corners[c].z();
        }
        units.push_back(berth_udp::makeBerthUnit(
            usv::berth_protocol::type(berth), berth, protocol_points));
    }

    if (units.empty())
        return;

    const std::vector<QByteArray> packets = berth_udp::buildBerthPackets(
        static_cast<int32_t>(std::llround(historyGnss.timestamp)),
        berth_frame_id_++, units);
    for (const QByteArray &packet : packets) {
        if (!sendUdpPayload(packet, "0xFB")) {
            qWarning() << "[历史泊位投递] 0xFB 发送失败，将在下一帧导航数据到来时重试";
            return;
        }
    }

    historical_berths_sent_ = true;
    last_historical_berths_send_ms_ = now_ms;
    if (useStaticLibrary)
        last_static_berths_send_ms_ = now_ms;
    qInfo() << "[历史泊位投递] 已发送" << units.size()
            << "个泊位（0xFB）"
            << (useStaticLibrary ? "来自固定泊位图层" : "来自 slammap");
}
#endif

void PerceptionExportNode::onBerthResult(const usv::BerthMeasureResult &res)
{
    if (!running_.load())
        return;

    if (!res.is_detected || res.expanded_berths.empty())
        return;

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const std::uint64_t signature = hashBerthResult(res);
    // The StateTopic and the compatibility signal can deliver the same frame
    // twice.  Suppress only that immediate duplicate; a stable berth must be
    // resent on later detection frames so a receiver that starts late still
    // gets a current 0xFB update.
    constexpr qint64 kDuplicateBerthWindowMs = 200;
    if (has_last_berth_signature_ && signature == last_berth_signature_
        && now - last_berth_signature_ms_ < kDuplicateBerthWindowMs)
        return;

    QString sendError;
    if (!sendBerth(res, &sendError)) {
        qWarning() << "[信息投递][0xFB] 泊位发送失败:" << sendError
                   << "count=" << res.expanded_berths.size()
                   << "timestamp=" << res.timestamp;
        if (now - last_berth_diag_ms_ >= 1000) {
            last_berth_diag_ms_ = now;
            emit logMessage(QStringLiteral(
                "[信息投递][0xFB] 泊位发送失败：%1（数量=%2，时间戳=%3）")
                                .arg(sendError)
                                .arg(static_cast<qulonglong>(
                                    res.expanded_berths.size()))
                                .arg(res.timestamp, 0, 'f', 3));
        }
        return;
    }

    ++berth_send_success_count_;
    has_last_berth_signature_ = true;
    last_berth_signature_ = signature;
    last_berth_signature_ms_ = now;
    if (berth_send_success_count_ == 1
        || berth_send_success_count_ % 50 == 0) {
        emit logMessage(QStringLiteral(
            "[信息投递][0xFB] 泊位已发送：%1 个（累计批次=%2）")
                            .arg(static_cast<qulonglong>(
                                res.expanded_berths.size()))
                            .arg(static_cast<qulonglong>(
                                berth_send_success_count_)));
    }
}

void PerceptionExportNode::clearSlamPoseCache()
{
    {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        slam_odom_history_.clear();
        has_slam_origin_ = false;
        slam_origin_lon_ = 0.0;
        slam_origin_lat_ = 0.0;
    }
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_gnss_ins_ = {};
        has_imu_ = false;
        gnss_enu_state_ = {};
        gnss_body_history_.clear();
    }
    accumulated_chunks_.clear();
    has_realtime_map_reference_ = false;
    realtime_map_to_enu_ = Eigen::Isometry3d::Identity();
    realtime_map_anchor_ = {};
    has_last_map_keyframe_ = false;
    realtime_berth_geo_cache_.clear();
    last_static_berths_send_ms_ = 0;
    map_packet_queue_.clear();
    map_queue_batch_total_ = 0;
    map_queue_batch_sent_ = 0;
    map_queue_batch_failed_ = 0;
    map_queue_batch_tag_.clear();
#ifdef ENABLE_SLAM
    live_slam_anchor_ = {};
    historical_berths_sent_ = false;
    last_historical_berths_send_ms_ = 0;
    if (!historical_map_packet_queue_.empty()
        || (historical_map_source_.isOpen()
            && historical_map_source_.hasNextFullMapTile())) {
        pumpHistoricalMapProducer();
        kickMapSendPump();
    }
#endif
}

void PerceptionExportNode::addSlamPoseToHistory(const usv::SlamOdometryState &state)
{
    if (!state.pose_lidar.valid)
        return;

    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (!slam_odom_history_.empty()) {
        const double last_ts = slam_odom_history_.back().timestamp;
        if (std::abs(last_ts - state.timestamp) < 1e-4) {
            slam_odom_history_.back() = state;
            ensureSlamOriginLocked();
            return;
        }
    }
    slam_odom_history_.push_back(state);
    while (slam_odom_history_.size() > kMaxSlamOdomHistory)
        slam_odom_history_.pop_front();
    ensureSlamOriginLocked();
}

void PerceptionExportNode::onSlamOdometry(const usv::SlamOdometryState &state)
{
    addSlamPoseToHistory(state);
}

bool PerceptionExportNode::sendUdpPayload(const QByteArray &payload, const char *tag)
{
    const qint64 sent = socket_.writeDatagram(
        payload, config_.remote_host, config_.remote_port);
    if (sent != payload.size()) {
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (std::strcmp(tag, "0xFB") == 0
            || now - last_udp_error_log_ms_ >= 1000) {
            last_udp_error_log_ms_ = now;
            qWarning() << "[信息投递] UDP 发送失败 tag=" << tag
                       << "expected=" << payload.size()
                       << "sent=" << sent
                       << "error=" << socket_.errorString()
                       << "remote=" << config_.remote_host.toString()
                       << config_.remote_port;
        }
        return false;
    }
    return true;
}

void PerceptionExportNode::ensureSlamOriginLocked()
{
    if (has_slam_origin_)
        return;

    usv::GnssInsMessage gnss;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!imuValidLocked())
            return;
        gnss = latest_gnss_ins_;
    }

    slam_origin_lon_ = gnss.longitude;
    slam_origin_lat_ = gnss.latitude;
    has_slam_origin_ = true;
}

bool PerceptionExportNode::lookupSlamPose(double timestamp,
                                          Eigen::Isometry3d &out_pose,
                                          double *out_dt) const
{
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (slam_odom_history_.empty())
        return false;

    double best_dt = 1e9;
    const usv::SlamOdometryState *best = nullptr;
    for (const usv::SlamOdometryState &state : slam_odom_history_) {
        const double dt = std::abs(state.timestamp - timestamp);
        if (dt < best_dt) {
            best_dt = dt;
            best = &state;
        }
    }
    if (!best)
        return false;

    const usv::SlamOdometryState &latest = slam_odom_history_.back();
    const double forward_lag = timestamp - latest.timestamp;

    if (best_dt <= kMaxSlamSyncSec) {
        out_pose = best->pose_lidar.pose;
        if (out_dt)
            *out_dt = best_dt;
        return true;
    }

    // 泊位检测快于 SLAM 处理时，泊位时间戳会略超前于最新里程计
    if (forward_lag >= 0.0 && forward_lag <= kMaxSlamForwardLagSec) {
        out_pose = latest.pose_lidar.pose;
        if (out_dt)
            *out_dt = forward_lag;
        return true;
    }

    if (best_dt <= kMaxSlamFallbackSec) {
        out_pose = best->pose_lidar.pose;
        if (out_dt)
            *out_dt = best_dt;
        return true;
    }

    return false;
}

bool PerceptionExportNode::lookupSlamPoseStrict(double timestamp,
                                                Eigen::Isometry3d &out_pose,
                                                double *out_dt) const
{
    std::lock_guard<std::mutex> lock(slam_mutex_);
    if (slam_odom_history_.empty())
        return false;

    double best_dt = 1e9;
    const usv::SlamOdometryState *best = nullptr;
    for (const usv::SlamOdometryState &state : slam_odom_history_) {
        const double dt = std::abs(state.timestamp - timestamp);
        if (dt < best_dt) {
            best_dt = dt;
            best = &state;
        }
    }
    if (!best || best_dt > kMaxBerthPoseSyncSec)
        return false;

    out_pose = best->pose_lidar.pose;
    if (out_dt)
        *out_dt = best_dt;
    return true;
}

void PerceptionExportNode::onSlamKeyframe(const usv::SlamKeyframe &keyframe)
{
    usv::SlamOdometryState state;
    state.timestamp = keyframe.timestamp;
    state.pose_lidar.pose = keyframe.pose;
    state.pose_lidar.valid = true;
    state.is_keyframe = true;
    addSlamPoseToHistory(state);

    last_map_keyframe_ = keyframe;
    has_last_map_keyframe_ = true;
    captureRealtimeMapReference(keyframe);

    if (!running_.load() || !config_.send_map)
        return;

    if (kEnableRealtimeMapExport) {
        const Eigen::Isometry3d exportPose = realtimeMapPose(keyframe.pose);
        mergeCloudIntoAccumulated(keyframe.cloud, exportPose);
        enqueueMapPackets(
            buildMapPacketsFromCloud(
                keyframe.cloud, exportPose, realtimeMapAnchor()),
            keyframe.timestamp,
            QStringLiteral("关键帧"));
    }

#ifdef ENABLE_SLAM
    if (historical_map_source_.isOpen()) {
        usv::SlamKeyframe virtualKeyframe;
        history_map_export::OverlapStats stats;
        QString error;
        if (historical_map_source_.makeOverlapVirtualKeyframe(
                keyframe, live_slam_anchor_, kHistoricalOverlapPaddingM,
                virtualKeyframe, &stats, &error)) {
            if (!virtualKeyframe.cloud.empty()) {
                enqueueMapPackets(
                    buildMapPacketsFromCloud(
                        virtualKeyframe.cloud,
                        Eigen::Isometry3d::Identity(),
                        &historical_map_source_.anchor(),
                        kHistoricalMapResolutionM),
                    keyframe.timestamp,
                    QStringLiteral("历史重叠虚拟关键帧"));
                qInfo() << "[历史地图投递] 虚拟关键帧"
                        << keyframe.keyframe_id
                        << "候选Tile=" << stats.candidate_tiles
                        << "读取Tile=" << stats.loaded_tiles
                        << "历史点=" << stats.selected_points;
            }
        } else {
            qWarning() << "[历史地图投递] 关键帧" << keyframe.keyframe_id
                       << "重叠区跳过:" << error;
        }
    }
#endif
}

#ifdef ENABLE_SLAM
void PerceptionExportNode::onSlamKeyframeWithAnchor(
    const usv::SlamKeyframe &keyframe,
    const usv::SlamGeoAnchor &liveAnchor)
{
    live_slam_anchor_ = liveAnchor;
    onSlamKeyframe(keyframe);
}
#endif

void PerceptionExportNode::onSlamScanCloud(QVector<M_PointXYZI> worldCloud)
{
    if (!running_.load() || !config_.send_map || !kEnableRealtimeMapExport
        || worldCloud.isEmpty())
        return;

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (now_ms - last_scan_map_enqueue_ms_ < kScanMapMinIntervalMs)
        return;
    last_scan_map_enqueue_ms_ = now_ms;

    double ts = 0.0;
    {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        if (!slam_odom_history_.empty())
            ts = slam_odom_history_.back().timestamp;
    }

    last_map_keyframe_.timestamp = ts;
    last_map_keyframe_.pose = Eigen::Isometry3d::Identity();
    has_last_map_keyframe_ = true;

    std::vector<M_PointXYZI> cloud;
    cloud.reserve(static_cast<size_t>(worldCloud.size()));
    for (const M_PointXYZI &p : worldCloud)
        cloud.push_back(p);

    const Eigen::Isometry3d exportPose = realtimeMapPose(
        Eigen::Isometry3d::Identity());
    mergeCloudIntoAccumulated(cloud, exportPose);
    enqueueMapPackets(
        buildMapPacketsFromCloud(cloud, exportPose, realtimeMapAnchor()), ts,
                      QStringLiteral("扫描"));
}

void PerceptionExportNode::enqueueMapPackets(std::vector<QByteArray> packets,
                                             double ts,
                                             const QString &tag)
{
    if (packets.empty())
        return;

    if (map_packet_queue_.empty()) {
        map_queue_batch_ts_ = ts;
        map_queue_batch_total_ = 0;
        map_queue_batch_sent_ = 0;
        map_queue_batch_failed_ = 0;
        map_queue_batch_tag_ = tag;
    }

    map_queue_batch_total_ += packets.size();
    for (QByteArray &pkt : packets) {
        map_packet_queue_.push_back(std::move(pkt));
    }
    kickMapSendPump();
}

#ifdef ENABLE_SLAM
void PerceptionExportNode::enqueueHistoricalPackets(
    std::vector<QByteArray> packets)
{
    for (QByteArray &packet : packets)
        historical_map_packet_queue_.push_back(std::move(packet));
}

void PerceptionExportNode::pumpHistoricalMapProducer()
{
    if (!running_.load() || !config_.send_map
        || historical_initial_stream_complete_
        || !historical_map_source_.isOpen()) {
        return;
    }
    if (historical_map_packet_queue_.size()
        >= kHistoricalQueueLowWatermark) {
        return;
    }

    while (historical_map_source_.hasNextFullMapTile()
           && historical_map_packet_queue_.size()
                  < kHistoricalQueueLowWatermark) {
        std::vector<M_PointXYZI> tilePoints;
        QString error;
        if (!historical_map_source_.takeNextFullMapTile(tilePoints, &error)) {
            qWarning() << "[历史地图投递] Tile 读取失败:" << error;
            continue;
        }
        enqueueHistoricalPackets(buildMapPacketsFromCloud(
            tilePoints, Eigen::Isometry3d::Identity(),
            &historical_map_source_.anchor(), kHistoricalMapResolutionM));
        // 每轮最多从磁盘读取一个 Tile，避免阻塞投递线程事件循环。
        break;
    }
    const bool wasComplete = historical_initial_stream_complete_;
    historical_initial_stream_complete_ =
        !historical_map_source_.hasNextFullMapTile();
    if (!wasComplete && historical_initial_stream_complete_) {
        qInfo() << "[历史地图投递] 历史整图 Tile 已全部进入发送队列";
    }
}
#endif

void PerceptionExportNode::kickMapSendPump()
{
    if (!running_.load() || !config_.send_map)
        return;
    if (map_packet_queue_.empty()
#ifdef ENABLE_SLAM
        && historical_map_packet_queue_.empty()
        && (historical_initial_stream_complete_
            || !historical_map_source_.isOpen())
#endif
        )
        return;

    if (!map_drip_timer_.isActive())
        map_drip_timer_.start();

    QMetaObject::invokeMethod(this, &PerceptionExportNode::processMapSendPump,
                              Qt::QueuedConnection);
}

void PerceptionExportNode::processMapSendPump()
{
    if (!running_.load() || !config_.send_map) {
        clearMapSendState();
        return;
    }

    if (map_packet_queue_.empty()
#ifdef ENABLE_SLAM
        && historical_map_packet_queue_.empty()
        && (historical_initial_stream_complete_
            || !historical_map_source_.isOpen())
#endif
        ) {
        map_drip_timer_.stop();
        return;
    }

    int sent_this_tick = 0;
    while (sent_this_tick < kMaxUdpPacketsPerTick) {
        QByteArray *next = nullptr;
        if (!map_packet_queue_.empty()) {
            next = &map_packet_queue_.front();
#ifdef ENABLE_SLAM
        } else if (!historical_map_packet_queue_.empty()) {
            next = &historical_map_packet_queue_.front();
#endif
        }
        if (!next)
            break;
        if (sendUdpPayload(*next, "0xFC"))
            ++map_queue_batch_sent_;
        else
            ++map_queue_batch_failed_;
        if (!map_packet_queue_.empty())
            map_packet_queue_.pop_front();
#ifdef ENABLE_SLAM
        else
            historical_map_packet_queue_.pop_front();
#endif
        ++sent_this_tick;
    }

#ifdef ENABLE_SLAM
    pumpHistoricalMapProducer();
#endif

    if (map_packet_queue_.empty() && map_queue_batch_total_ > 0) {
        map_queue_batch_total_ = 0;
        map_queue_batch_sent_ = 0;
        map_queue_batch_failed_ = 0;
        map_queue_batch_tag_.clear();
    }

    if (!map_packet_queue_.empty()
#ifdef ENABLE_SLAM
        || !historical_map_packet_queue_.empty()
        || (!historical_initial_stream_complete_
            && historical_map_source_.isOpen())
#endif
        )
        kickMapSendPump();
    else
        map_drip_timer_.stop();
}

bool PerceptionExportNode::sendBerth(const usv::BerthMeasureResult &res,
                                     QString *error)
{
    if (error)
        error->clear();
    const auto fail = [error](const QString &reason) {
        if (error)
            *error = reason;
        return false;
    };

    if (!res.is_detected || res.expanded_berths.empty())
        return fail(QStringLiteral("泊位结果为空"));

    Eigen::Isometry3d body_pose_in_enu = Eigen::Isometry3d::Identity();
    usv::gnss_geo::GnssEnuState enu_state;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!imuValidLocked())
            return fail(QStringLiteral("GNSS/INS 尚未有效"));
        if (!lookupGnssBodyPose(res.timestamp, body_pose_in_enu, enu_state))
            return fail(QStringLiteral("找不到与泊位时间戳匹配的 GNSS/INS 姿态"));
    }

    return sendBerthWithPose(res, body_pose_in_enu, enu_state, error);
}

bool PerceptionExportNode::sendBerthWithPose(
    const usv::BerthMeasureResult &res,
    const Eigen::Isometry3d &body_pose_in_enu,
    const usv::gnss_geo::GnssEnuState &enu_state,
    QString *error)
{
    if (error)
        error->clear();
    const auto fail = [error](const QString &reason) {
        if (error)
            *error = reason;
        return false;
    };

    if (!res.is_detected || res.expanded_berths.empty())
        return fail(QStringLiteral("泊位结果为空"));
    if (!enu_state.initialized)
        return fail(QStringLiteral("ENU 参考状态尚未初始化"));

    const std::size_t berth_count = std::min(
        res.expanded_berths.size(),
        PerceptionUdpConstants::kMaxBerthsPerBatch);
    if (res.expanded_berths.size() > berth_count) {
        qWarning() << "0xFB berth batch truncated from"
                   << res.expanded_berths.size() << "to" << berth_count;
    }

    std::vector<BerthUdpUnit> units;
    units.reserve(berth_count);
    for (std::size_t i = 0; i < berth_count; ++i) {
        const usv::Berth &b = res.expanded_berths[i];

        const double rad = b.angle * kDegToRad;
        const double cos_a = std::cos(rad);
        const double sin_a = std::sin(rad);
        const double half_l = b.l * 0.5;
        const double half_w = b.w * 0.5;
        // 算法局部系角点 0..3（CCW）：local_x=宽，local_y=长
        const double local_x[4] = {-half_w, half_w, half_w, -half_w};
        const double local_y[4] = {-half_l, -half_l, half_l, half_l};

        struct AlgoCorner {
            Eigen::Vector3d p_lidar;
            double geo_lon = 0.0;
            double geo_lat = 0.0;
            double geo_alt = 0.0;
        };
        AlgoCorner algo_corners[4];

        for (int c = 0; c < 4; ++c) {
            const double bx = b.cx + cos_a * local_x[c] - sin_a * local_y[c];
            const double by = b.cy + sin_a * local_x[c] + cos_a * local_y[c];
            algo_corners[c].p_lidar = Eigen::Vector3d(bx, by, 0.0);
            if (!usv::gnss_geo::cornerLidarToGeodetic(
                algo_corners[c].p_lidar, body_pose_in_enu, enu_state,
                algo_corners[c].geo_lat, algo_corners[c].geo_lon,
                algo_corners[c].geo_alt)) {
                return fail(QStringLiteral("泊位角点经纬度换算失败"));
            }
        }

        const int opening = usv::berth_protocol::openingEdge(
            b, usv::berth_protocol::cornersCounterClockwise(b));
        std::array<berth_udp::BerthProtocolPoint, 4> protocol_points{};
        for (int pi = 0; pi < 4; ++pi) {
            const int c = (opening + 1 + pi) % 4;
            const AlgoCorner &corner = algo_corners[c];
            berth_udp::BerthProtocolPoint &point =
                protocol_points[static_cast<std::size_t>(pi)];
            point.longitude_deg = corner.geo_lon;
            point.latitude_deg = corner.geo_lat;
            point.altitude_m = corner.geo_alt;
            point.x_m = corner.p_lidar.x();
            point.y_m = corner.p_lidar.y();
            point.z_m = corner.p_lidar.z();
        }

        usv::Berth normalized_berth = b;
        normalized_berth.opening_edge = opening;
        const uint8_t protocol_type = usv::berth_protocol::type(b);
        BerthUdpUnit unit = berth_udp::makeBerthUnit(
            protocol_type, normalized_berth, protocol_points);

        // 发送端本地泊位是当前雷达系坐标，接收端却按经纬度绘制。
        // 同一泊位在连续帧中的局部框会有小幅变化；保留首次确认的
        // 地理四角，避免这些变化被接收端表现成泊位随船移动。缓存只
        // 作用于 0xFB 的实时批次，地图、检测器和本地界面均不改动。
        double center_lon = 0.0;
        double center_lat = 0.0;
        for (const berth_udp::BerthProtocolPoint &point : protocol_points) {
            center_lon += point.longitude_deg;
            center_lat += point.latitude_deg;
        }
        center_lon *= 0.25;
        center_lat *= 0.25;

        int matched_cache = -1;
        double best_distance = kRealtimeBerthCacheMatchM;
        for (int cache_index = 0;
             cache_index < static_cast<int>(realtime_berth_geo_cache_.size());
             ++cache_index) {
            const RealtimeBerthGeoCache &cached =
                realtime_berth_geo_cache_[static_cast<std::size_t>(cache_index)];
            if (cached.type != protocol_type
                || std::abs(cached.width_m - b.w) > 3.0
                || std::abs(cached.length_m - b.l) > 3.0) {
                continue;
            }
            const double distance = geodeticDistanceMeters(
                center_lon, center_lat,
                cached.longitude_deg, cached.latitude_deg);
            if (distance < best_distance) {
                best_distance = distance;
                matched_cache = cache_index;
            }
        }

        if (matched_cache >= 0) {
            unit = realtime_berth_geo_cache_[
                static_cast<std::size_t>(matched_cache)].unit;
        } else {
            realtime_berth_geo_cache_.push_back(
                RealtimeBerthGeoCache{
                    protocol_type, center_lon, center_lat, b.w, b.l, unit});
        }
        units.push_back(unit);
    }

    const std::vector<QByteArray> packets = berth_udp::buildBerthPackets(
        static_cast<int32_t>(std::llround(res.timestamp)),
        berth_frame_id_++,
        units);
    for (const QByteArray &packet : packets) {
        if (!sendUdpPayload(packet, "0xFB"))
            return fail(QStringLiteral("UDP 写入失败"));
    }
    return true;
}

std::vector<QByteArray> PerceptionExportNode::buildMapPacketsFromCloud(
    const std::vector<M_PointXYZI> &cloud,
    const Eigen::Isometry3d &pose,
    const usv::SlamGeoAnchor *fixedAnchor,
    double resolution)
{
    std::vector<QByteArray> out;
    if (cloud.empty())
        return out;
    if (!std::isfinite(resolution) || resolution <= 0.0)
        return out;

    double header_lon = 0.0;
    double header_lat = 0.0;
    double header_yaw = 0.0;
    if (fixedAnchor) {
        header_lon = fixedAnchor->longitude_deg;
        header_lat = fixedAnchor->latitude_deg;
    } else {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imuValidLocked()) {
            header_lon = latest_gnss_ins_.longitude;
            header_lat = latest_gnss_ins_.latitude;
            header_yaw = latest_gnss_ins_.yaw;
        }
    }

    const double inv_res = 1.0 / resolution;
    std::unordered_map<ChunkKey, std::vector<uint8_t>, ChunkKeyHash> chunks;

    for (const M_PointXYZI &p : cloud) {
        const Eigen::Vector3d wp = pose * Eigen::Vector3d(p.x, p.y, p.z);
        const int gx = static_cast<int>(std::floor(wp.x() * inv_res));
        const int gy = static_cast<int>(std::floor(wp.y() * inv_res));
        const int gz = static_cast<int>(std::floor(wp.z() * inv_res));

        const ChunkKey key{
            floorDiv(gx, PerceptionUdpConstants::kChunkXLen),
            floorDiv(gy, PerceptionUdpConstants::kChunkYLen),
            floorDiv(gz, PerceptionUdpConstants::kChunkZLen),
        };
        if (key.x < INT16_MIN || key.x > INT16_MAX
            || key.y < INT16_MIN || key.y > INT16_MAX
            || key.z < INT16_MIN || key.z > INT16_MAX) {
            continue;
        }

        auto &bits = chunks[key];
        if (bits.empty())
            bits.assign(1280, 0);

        const int local_x = gx - key.x * PerceptionUdpConstants::kChunkXLen;
        const int local_y = gy - key.y * PerceptionUdpConstants::kChunkYLen;
        const int local_z = gz - key.z * PerceptionUdpConstants::kChunkZLen;
        if (local_x < 0 || local_x >= PerceptionUdpConstants::kChunkXLen
            || local_y < 0 || local_y >= PerceptionUdpConstants::kChunkYLen
            || local_z < 0 || local_z >= PerceptionUdpConstants::kChunkZLen) {
            continue;
        }

        const int index = local_x + local_y * PerceptionUdpConstants::kChunkXLen
                        + local_z * PerceptionUdpConstants::kChunkXLen
                               * PerceptionUdpConstants::kChunkYLen;
        const int byte_index = index >> 3;
        const int bit_index = index & 7;
        bits[static_cast<size_t>(byte_index)] |= static_cast<uint8_t>(1 << bit_index);
    }

    if (chunks.empty())
        return out;

    out.reserve(chunks.size());
    for (const auto &entry : chunks) {
        GridUdpPacket pkt{};
        fillPrefix(pkt.prefix);
        pkt.total_len = qToBigEndian(PerceptionUdpConstants::kGridPacketLen);
        pkt.id = PerceptionUdpConstants::kGridId;
        pkt.reserve1 = 0;
        pkt.sequence_id = qToBigEndian(++map_sequence_);
        pkt.longitude = qToBigEndian(encodeGeo(header_lon));
        pkt.latitude = qToBigEndian(encodeGeo(header_lat));
        pkt.altitude = qToBigEndian(encodeAlt(
            fixedAnchor ? fixedAnchor->altitude_m : 0.0));
        pkt.azimuth = qToBigEndian(encodeAzimuth(header_yaw));
        pkt.chunk_x = qToBigEndian(static_cast<int16_t>(entry.first.x));
        pkt.chunk_y = qToBigEndian(static_cast<int16_t>(entry.first.y));
        pkt.chunk_z = qToBigEndian(static_cast<int16_t>(entry.first.z));
        pkt.resolution = qToBigEndian(static_cast<uint16_t>(
            std::llround(resolution * 100.0)));
        pkt.chunk_x_len = PerceptionUdpConstants::kChunkXLen;
        pkt.chunk_y_len = PerceptionUdpConstants::kChunkYLen;
        pkt.chunk_z_len = PerceptionUdpConstants::kChunkZLen;
        pkt.reserve2 = 0;
        std::memcpy(pkt.grid_bit, entry.second.data(), 1280);

        out.emplace_back(reinterpret_cast<const char *>(&pkt),
                         static_cast<int>(sizeof(pkt)));
    }
    return out;
}

std::vector<QByteArray> PerceptionExportNode::buildAccumulatedMapPackets()
{
    std::vector<QByteArray> out;
    if (accumulated_chunks_.empty())
        return out;

    double header_lon = 0.0;
    double header_lat = 0.0;
    double header_yaw = 0.0;
    if (realtime_map_anchor_.valid) {
        // accumulated_chunks_ is already in the fixed realtime ENU frame.
        // Keep the same anchor and zero azimuth for every resend batch.
        header_lon = realtime_map_anchor_.longitude_deg;
        header_lat = realtime_map_anchor_.latitude_deg;
    } else {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imuValidLocked()) {
            header_lon = latest_gnss_ins_.longitude;
            header_lat = latest_gnss_ins_.latitude;
            header_yaw = latest_gnss_ins_.yaw;
        }
    }

    const double resolution = PerceptionUdpConstants::kDefaultResolutionM;
    out.reserve(accumulated_chunks_.size());
    for (const auto &entry : accumulated_chunks_) {
        GridUdpPacket pkt{};
        fillPrefix(pkt.prefix);
        pkt.total_len = qToBigEndian(PerceptionUdpConstants::kGridPacketLen);
        pkt.id = PerceptionUdpConstants::kGridId;
        pkt.reserve1 = 0;
        pkt.sequence_id = qToBigEndian(++map_sequence_);
        pkt.longitude = qToBigEndian(encodeGeo(header_lon));
        pkt.latitude = qToBigEndian(encodeGeo(header_lat));
        pkt.altitude = qToBigEndian(encodeAlt(0.0));
        pkt.azimuth = qToBigEndian(encodeAzimuth(header_yaw));
        pkt.chunk_x = qToBigEndian(static_cast<int16_t>(entry.first.x));
        pkt.chunk_y = qToBigEndian(static_cast<int16_t>(entry.first.y));
        pkt.chunk_z = qToBigEndian(static_cast<int16_t>(entry.first.z));
        pkt.resolution = qToBigEndian(static_cast<uint16_t>(
            std::llround(resolution * 100.0)));
        pkt.chunk_x_len = PerceptionUdpConstants::kChunkXLen;
        pkt.chunk_y_len = PerceptionUdpConstants::kChunkYLen;
        pkt.chunk_z_len = PerceptionUdpConstants::kChunkZLen;
        pkt.reserve2 = 0;
        std::memcpy(pkt.grid_bit, entry.second.data(), 1280);

        out.emplace_back(reinterpret_cast<const char *>(&pkt),
                         static_cast<int>(sizeof(pkt)));
    }
    return out;
}

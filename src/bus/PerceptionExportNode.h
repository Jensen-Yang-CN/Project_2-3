#pragma once

#include <QByteArray>
#include <QHostAddress>
#include <QObject>
#include <QString>
#include <QUdpSocket>
#include <QTimer>
#include <QVector>

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "GnssInsGeoUtils.h"
#include "NaviProtocol.h"
#include "PerceptionUdpProtocol.h"
#include "StaticBerthLibrary.h"
#include "common/slam_types.h"
#include "common_types_extended.h"
#include "CommunicationManager_v2.h"
#include "bridge_common_types.h"
#ifdef ENABLE_SLAM
#include "HistoricalMapExportSource.h"
#endif

/** 将泊位 / SLAM 地图通过 UDP 投递到对端主机（0xFB / 0xFC） */
class PerceptionExportNode : public QObject {
    Q_OBJECT
public:
    struct Config {
        QHostAddress remote_host = QHostAddress(QStringLiteral("192.168.137.21"));
        quint16 remote_port = 6789;
        QHostAddress local_host = QHostAddress(QStringLiteral("192.168.137.20"));
        quint16 local_port = 6789;
        bool bind_local = true;
        bool send_map = true;
        QString static_berth_library_path = QStringLiteral(
            "D:/xwechat_files/wxid_0cfu1zhr4dz022_ae9a/msg/file/2026-09/"
            "泊位检测完整地理信息_20260901.json");
    };

    explicit PerceptionExportNode(QObject *parent = nullptr);
    ~PerceptionExportNode() override = default;

    void setConfig(const Config &cfg);
    Config config() const;

    bool isRunning() const { return running_.load(); }
    void start();
    void stop();

    /** 兼容入口：内部转为 GnssInsMessage；优先通过总线订阅 */
    void feedImu(const IMUParsedData &imu);
    void feedGnssIns(const usv::GnssInsMessage &gnss);
    void clearSlamPoseCache();
    void ingestSlamKeyframes(const std::vector<usv::SlamKeyframe> &keyframes);
#ifdef ENABLE_SLAM
    bool loadHistoricalMap(const QString &manifestPath,
                           QString *error = nullptr);
    void clearHistoricalMap();
#endif

signals:
    /** 投递诊断信息，连接到主窗口“检测信息”区域。 */
    void logMessage(const QString &text);

public slots:
    void onBerthResult(const usv::BerthMeasureResult &res);
    void onSlamOdometry(const usv::SlamOdometryState &state);
    void onSlamKeyframe(const usv::SlamKeyframe &keyframe);
#ifdef ENABLE_SLAM
    void onSlamKeyframeWithAnchor(const usv::SlamKeyframe &keyframe,
                                  const usv::SlamGeoAnchor &liveAnchor);
#endif
    void onSlamScanCloud(QVector<M_PointXYZI> worldCloud);

private slots:
    void processMapSendPump();
    void onMapResendTimer();
    void pollImuBus();

private:
    void subscribeImuBus();
    void unsubscribeImuBus();
    void subscribeBerthBus();
    void unsubscribeBerthBus();
#ifdef ENABLE_SLAM
    void subscribeSlamBus();
    void unsubscribeSlamBus();
#endif
    bool sendBerth(const usv::BerthMeasureResult &res,
                   QString *error = nullptr);
    /** 使用显式的 ENU 姿态组装并发送 0xFB；历史地图无实时 GNSS 时使用。 */
    bool sendBerthWithPose(
        const usv::BerthMeasureResult &res,
        const Eigen::Isometry3d &body_pose_in_enu,
        const usv::gnss_geo::GnssEnuState &enu_state,
        QString *error = nullptr);
    void loadStaticBerthLibrary();
    void sendStaticBerthLibraryIfDue(bool force = false);
#ifdef ENABLE_SLAM
    /** 将已加载历史地图中的泊位按历史 ENU 锚点投递一次 0xFB。 */
    void trySendHistoricalBerths();
#endif
    void mergeCloudIntoAccumulated(const std::vector<M_PointXYZI> &cloud,
                                   const Eigen::Isometry3d &pose);
    // 实时 SLAM 地图在发送前统一转换到固定的 ENU 参考系，避免每个
    // 0xFC 包随着当前船体 GNSS/航向变化而在接收端旋转或平移。
    void captureRealtimeMapReference(const usv::SlamKeyframe &keyframe);
    Eigen::Isometry3d realtimeMapPose(const Eigen::Isometry3d &pose) const;
    const usv::SlamGeoAnchor *realtimeMapAnchor() const;
    std::vector<QByteArray> buildMapPacketsFromCloud(
        const std::vector<M_PointXYZI> &cloud,
        const Eigen::Isometry3d &pose,
        const usv::SlamGeoAnchor *fixedAnchor = nullptr,
        double resolution_m = PerceptionUdpConstants::kDefaultResolutionM);
    std::vector<QByteArray> buildAccumulatedMapPackets();
    void enqueueMapPackets(std::vector<QByteArray> packets, double ts,
                           const QString &tag);
#ifdef ENABLE_SLAM
    void enqueueHistoricalPackets(std::vector<QByteArray> packets);
    void pumpHistoricalMapProducer();
#endif
    void enqueueFullAccumulatedMapResync();
    void kickMapSendPump();
    bool sendUdpPayload(const QByteArray &payload, const char *tag);
    void clearMapSendState();
    bool imuValidLocked() const;
    void addSlamPoseToHistory(const usv::SlamOdometryState &state);
    bool lookupSlamPose(double timestamp, Eigen::Isometry3d &out_pose,
                         double *out_dt = nullptr) const;
    bool lookupSlamPoseStrict(double timestamp, Eigen::Isometry3d &out_pose,
                              double *out_dt = nullptr) const;
    void ensureSlamOriginLocked();
    void recordGnssBodySample(const usv::GnssInsMessage &gnss);
    bool lookupGnssBodyPose(double timestamp,
                            Eigen::Isometry3d &body_pose,
                            usv::gnss_geo::GnssEnuState &enu_state) const;

    struct ChunkKey {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const ChunkKey &o) const
        {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct ChunkKeyHash {
        std::size_t operator()(const ChunkKey &k) const
        {
            return (static_cast<std::size_t>(k.x) << 20)
                 ^ (static_cast<std::size_t>(k.y) << 10)
                 ^ static_cast<std::size_t>(k.z);
        }
    };
    static int floorDiv(int value, int divisor);
    void mergeCloudIntoAccumulatedLocked(const std::vector<M_PointXYZI> &cloud,
                                         const Eigen::Isometry3d &pose);

    Config config_;
    QUdpSocket socket_;

    struct GnssBodySample {
        double timestamp = 0.0;
        Eigen::Isometry3d body_pose = Eigen::Isometry3d::Identity();
        usv::gnss_geo::GnssEnuState enu_state;
    };

    mutable std::mutex imu_mutex_;
    usv::GnssInsMessage latest_gnss_ins_{};
    bool has_imu_ = false;
    usv::gnss_geo::GnssEnuState gnss_enu_state_;
    std::deque<GnssBodySample> gnss_body_history_;
    static constexpr size_t kMaxGnssBodyHistory = 500;
    static constexpr double kMaxGnssSyncSec = 5.0;
    mutable qint64 last_gnss_fallback_log_ms_ = 0;

    std::shared_ptr<usv::RawTopic<usv::GnssInsMessage>> imu_topic_;
    uint64_t imu_subscriber_id_ = 0;
    QTimer imu_poll_timer_;
    static constexpr int kImuPollIntervalMs = 10;

    // 泊位结果直接订阅总线，避免依赖 MainWindow 的生命周期和连接顺序。
    std::shared_ptr<usv::StateTopic<usv::BerthMeasureResult>> berth_topic_;
    uint64_t berth_subscriber_id_ = 0;
    quint64 berth_send_success_count_ = 0;
    qint64 last_berth_diag_ms_ = 0;
    qint64 last_udp_error_log_ms_ = 0;
    // StateTopic and the compatibility signal can both carry the same frame.
    // Remember the last successfully sent payload to avoid duplicate 0xFB
    // batches while still allowing every new detection frame through.
    bool has_last_berth_signature_ = false;
    std::uint64_t last_berth_signature_ = 0;
    qint64 last_berth_signature_ms_ = 0;
    // 接收端按经纬度绘制泊位。实时检测结果本身在雷达系中会随船体
    // 变化，因此在投递线程保留同一泊位首次确认的地理几何，避免把
    // 传感器局部抖动表现成“泊位跟着船走”。只影响 0xFB 投递，不改
    // 本地检测结果或 SLAM 地图。
    struct RealtimeBerthGeoCache {
        uint8_t type = 0;
        double longitude_deg = 0.0;
        double latitude_deg = 0.0;
        double width_m = 0.0;
        double length_m = 0.0;
        BerthUdpUnit unit{};
    };
    std::vector<RealtimeBerthGeoCache> realtime_berth_geo_cache_;
    std::vector<BerthUdpUnit> static_berth_units_;
    bool static_berth_library_loaded_ = false;
    qint64 last_static_berths_send_ms_ = 0;

#ifdef ENABLE_SLAM
    // 实时 SLAM 结果由投递节点独立订阅，避免依赖界面/OctoMap 的连接顺序。
    std::shared_ptr<usv::StateTopic<usv::SlamOdometryState>> slam_odom_topic_;
    std::shared_ptr<usv::StateTopic<usv::SlamKeyframe>> slam_keyframe_topic_;
    std::shared_ptr<usv::StateTopic<usv::SlamScanCloudMessage>> slam_scan_topic_;
    uint64_t slam_odom_subscriber_id_ = 0;
    uint64_t slam_keyframe_subscriber_id_ = 0;
    uint64_t slam_scan_subscriber_id_ = 0;
#endif

    mutable std::mutex slam_mutex_;
    std::deque<usv::SlamOdometryState> slam_odom_history_;
    bool has_slam_origin_ = false;
    double slam_origin_lon_ = 0.0;
    double slam_origin_lat_ = 0.0;
    bool has_realtime_map_reference_ = false;
    Eigen::Isometry3d realtime_map_to_enu_ = Eigen::Isometry3d::Identity();
    usv::SlamGeoAnchor realtime_map_anchor_;
    static constexpr double kMaxSlamSyncSec = 5.0;
    static constexpr double kMaxSlamForwardLagSec = 20.0;
    static constexpr double kMaxSlamFallbackSec = 30.0;
    static constexpr double kMaxBerthPoseSyncSec = 3.0;
    static constexpr qint64 kMapResendIntervalMs = 3000;
    static constexpr size_t kMaxSlamOdomHistory = 200;
    static constexpr int kMapDripIntervalMs = 2;
    // 历史地图与泊位共用 UDP socket；降低每轮 0xFC 数量，避免接收端
    // 被连续地图报文挤掉 0xFB 泊位报文。
    static constexpr int kMaxUdpPacketsPerTick = 8;
    static constexpr qint64 kScanMapMinIntervalMs = 500;
    static constexpr size_t kMaxMapPacketQueue = 20000;
    static constexpr size_t kHistoricalQueueLowWatermark = 4000;
    static constexpr double kHistoricalOverlapPaddingM = 1.0;
    static constexpr qint64 kHistoricalBerthResendIntervalMs = 2000;
    static constexpr qint64 kStaticBerthResendIntervalMs = 2000;

    usv::SlamKeyframe last_map_keyframe_;
    bool has_last_map_keyframe_ = false;
    qint64 last_scan_map_enqueue_ms_ = 0;

    QTimer map_drip_timer_;
    QTimer map_resend_timer_;
    std::unordered_map<ChunkKey, std::vector<uint8_t>, ChunkKeyHash> accumulated_chunks_;
    /** 实时/重叠区高优先级队列。 */
    std::deque<QByteArray> map_packet_queue_;
#ifdef ENABLE_SLAM
    /** 历史整图后台低优先级队列。 */
    std::deque<QByteArray> historical_map_packet_queue_;
    history_map_export::HistoricalMapExportSource historical_map_source_;
    usv::SlamGeoAnchor live_slam_anchor_;
    bool historical_initial_stream_complete_ = true;
    bool historical_berths_sent_ = false;
    qint64 last_historical_berths_send_ms_ = 0;
#endif
    double map_queue_batch_ts_ = 0.0;
    size_t map_queue_batch_total_ = 0;
    size_t map_queue_batch_sent_ = 0;
    size_t map_queue_batch_failed_ = 0;
    QString map_queue_batch_tag_;

    std::atomic<bool> running_{false};
    quint16 berth_frame_id_ = 0;
    quint32 map_sequence_ = 0;
};

#pragma once

#include <QObject>
#include <QString>

#include <atomic>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "BerthDetection.h"
#include "CommunicationManager_v2.h"
#include "LineBerthDetection.h"
#include "ThreadSafeQueue.h"
#include "common_types_extended.h"

struct IMUParsedData;

namespace usv {
namespace gnss_geo {
struct GnssEnuState;
}
}

/** 订阅 /preprocess/synced_sensors，运行泊位检测，发布 /perception/berth_result */
class BerthDetectionNode : public QObject {
    Q_OBJECT
public:
    explicit BerthDetectionNode(QObject *parent = nullptr);
    ~BerthDetectionNode() override;

    void Init();
    void Start();
    void stop();
    void setDetectionModes(bool u_shape_enabled, bool line_enabled);
    void feedNavigationData(const IMUParsedData &data);
    /** 清空离线回放旧帧、姿态和泊位跟踪状态，避免时间轴切换串帧。 */
    void requestPlaybackTimelineReset(uint64_t sensorTimelineGeneration = 0);
    void resetWorldCoordinateState();

signals:
    void logMessage(const QString &text);
    void berthResultReady(usv::BerthMeasureResult res);

private:
    void processLoop();
    void postLog(const QString &text);
    void postBerthResult(const usv::BerthMeasureResult &res);
    void maybeLogDetectionResult(const usv::BerthMeasureResult &res);
    bool lookupNavigationPose(double timestamp, Eigen::Isometry3d &pose,
                              double *out_dt = nullptr) const;
    std::vector<bool> confirmAndUpdateRealtimeCoordinateLibrary(
        const std::vector<usv::Berth> &berths,
        std::vector<std::string> &debug_messages);
    std::vector<bool> stabilizeRealtimeWorldBerths(std::vector<usv::Berth> &berths,
                                                    std::vector<std::string> &debug_messages);
    void updateWorldAnchorBus(const usv::DebugLine2D &world_bus,
                              std::vector<std::string> &debug_messages);
    bool getWorldAnchorBus(usv::DebugLine2D &world_bus,
                           bool *locked = nullptr) const;
    void updateWorldAnchorBusBerthSide(const std::vector<usv::Berth> &world_berths,
                                       std::vector<std::string> &debug_messages);
    void flipLibraryBerthsOppositeWorldAnchorBus(std::vector<usv::Berth> &world_berths,
                                                 std::vector<std::string> &debug_messages) const;
    void alignCoordinateLibrariesToWorldAnchorBus(
        std::vector<std::string> &debug_messages);
    void resetRealtimeCoordinateConfirmations();
    void markOccupiedBerths(const usv::LidarFrame &frame,
                            usv::BerthMeasureResult &result) const;
    struct LastLogState {
        bool initialized = false;
        bool detected = false;
        std::vector<usv::Berth> berths;
        std::vector<std::string> debug_messages;
    };
    LastLogState last_log_;

    usv::BerthDetector u_shape_algo_;
    usv::LineBerthDetector line_algo_;
    std::atomic<bool> u_shape_enabled_{true};
    std::atomic<bool> line_enabled_{false};
    usv::CommunicationManager *comm_ = nullptr;
    std::shared_ptr<usv::StateTopic<usv::SyncedSensorPackage>> sub_sync_;
    std::shared_ptr<usv::StateTopic<usv::BerthMeasureResult>> pub_result_;
    uint64_t sub_id_ = 0;

    usv::ThreadSafeQueue<usv::SyncedPackagePtr> sync_queue_{1};

    mutable std::mutex map_mutex_;
    struct NavigationPoseState {
        double timestamp = 0.0;
        Eigen::Isometry3d pose_lidar = Eigen::Isometry3d::Identity();
    };
    std::unique_ptr<usv::gnss_geo::GnssEnuState> navigation_enu_state_;
    std::deque<NavigationPoseState> navigation_pose_history_;
    std::vector<usv::Berth> realtime_coordinate_berths_;
    std::vector<usv::Berth> realtime_navigation_coordinate_berths_;
    std::vector<usv::Berth> navigation_world_recovery_rois_;
    struct RealtimeBerthConfirmation {
        usv::Berth berth;
        size_t hit_count = 0;
    };
    struct WorldBerthStabilityState {
        usv::Berth last_accepted_berth;
        std::deque<usv::Berth> accepted_window;
        std::deque<usv::Berth> replacement_window;
        bool has_stable_geometry = false;
    };
    std::vector<RealtimeBerthConfirmation> realtime_navigation_confirmations_;
    std::vector<WorldBerthStabilityState> realtime_navigation_stability_states_;
    struct WorldAnchorBusState {
        bool valid = false;
        bool locked = false;
        usv::DebugLine2D stable_line{};
        bool has_pending = false;
        usv::DebugLine2D pending_line{};
        size_t consecutive_frames = 0;
        size_t stable_consistent_frames = 0;
        int berth_side = 0;
    };
    WorldAnchorBusState navigation_world_anchor_bus_;
    const size_t max_navigation_pose_history_ = 500;
    const size_t realtime_output_confirmation_frames_ = 3;
    const size_t realtime_stability_window_frames_ = 8;
    const size_t realtime_replacement_stability_window_frames_ = 5;
    const double stability_association_distance_m_ = 8.0;
    const double stability_position_l1_gate_m_ = 3.0;
    const double stability_size_gate_m_ = 3.0;
    const double stability_angle_gate_deg_ = 13.0;
    const size_t world_anchor_bus_confirmation_frames_ = 3;
    const size_t world_anchor_bus_lock_frames_ = 6;
    const double world_anchor_bus_angle_gate_deg_ = 8.0;
    const double world_anchor_bus_offset_gate_m_ = 1.0;
    const double world_anchor_bus_pull_distance_m_ = 5.0;
    const double world_anchor_bus_side_edge_gate_m_ = 2.0;
    // GNSS/INS and lidar timestamps must be close enough for ENU projection.
    const double max_navigation_pose_sync_sec_ = 5.0;
    const double occupied_inset_margin_m_ = 0.8;
    const int occupied_min_points_ = 100;
    double last_map_pose_skip_log_timestamp_ = -std::numeric_limits<double>::infinity();

    std::thread consumer_;
    std::atomic<bool> is_running_{false};
    std::atomic<uint64_t> realtime_received_frames_{0};
    std::atomic<uint64_t> realtime_processed_frames_{0};
    uint64_t last_logged_realtime_received_ = 0;
    uint64_t last_logged_realtime_processed_ = 0;
    uint64_t last_logged_realtime_dropped_ = 0;
    std::chrono::steady_clock::time_point last_realtime_performance_log_{};
};

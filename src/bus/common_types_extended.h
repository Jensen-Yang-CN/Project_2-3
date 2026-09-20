#pragma once

#include "bridge_common_types.h"

#include <cstdint>
#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace usv {

struct ImuMessage {
    double timestamp = 0.0;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct GnssMessage {
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double heading = 0.0;
};

/** 预处理输出：210+211 点云与左相机按时间对齐（SLAM 字段预留） */
struct PreprocessedSensorBundle {
    double timestamp = 0.0;
    uint64_t timeline_generation = 1;

    std::shared_ptr<const LidarFrame> raw_lidar_210;
    std::shared_ptr<const LidarFrame> raw_lidar_211;
    std::shared_ptr<const ImageFrame> synced_image;

    std::shared_ptr<const LidarFrame> deskewed_lidar_210;
    std::shared_ptr<const LidarFrame> deskewed_lidar_211;

    std::vector<ImuMessage> imu_segment;
    std::optional<GnssMessage> latest_gnss;

    bool image_found = false;
    bool lidar_210_found = false;
    bool gnss_found = false;
};

using PreprocessedSensorBundlePtr = std::shared_ptr<const PreprocessedSensorBundle>;

/** 传统视觉桥梁检测：每秒发布一次的结构化结果 */
enum class CameraSide {
    Left = 0,
    Right = 1
};

struct BridgePierObservation {
    int id = 0;
    double azimuth_deg = 0.0;
    double pixel_x = 0.0;
    double pixel_y = 0.0;
};

struct TraditionalBridgeVisionBusResult {
    double timestamp = 0.0;
    uint64_t timeline_generation = 1;
    CameraSide camera_side = CameraSide::Left;
    bool bridge_detected = false;
    double confidence = 0.0;
    int pier_count = 0;
    std::vector<BridgePierObservation> piers;
    bool has_bridge_opening = false;
    double bridge_opening_center_azimuth_deg = 0.0;
    double pier_angular_separation_deg = 0.0;
    bool has_deck_line = false;
    double deck_inclination_deg = 0.0;
};

/** 泊位几何 */
struct DebugLine2D {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

enum class BerthKind {
    Unknown = 0,
    UShape = 1,
    Line = 2
};

/** 泊位结果来源：实时检测 / 坐标库回显（交接包 Map 与 CoordinateLibrary 同值） */
enum class BerthDetectionSource {
    Realtime = 0,
    CoordinateLibrary = 1,
    Map = CoordinateLibrary
};

struct Berth {
    double cx = 0.0;
    double cy = 0.0;
    double w = 0.0;
    double l = 0.0;
    double angle = 0.0;
    BerthKind kind = BerthKind::Unknown;
    BerthDetectionSource source = BerthDetectionSource::Realtime;
    /** 内部来源：锚点恢复结果，不写入对外协议 */
    bool is_anchor_recovered = false;
    // Internal occupancy state; not part of the external berth protocol.
    bool has_ship = false;
    int interior_point_count = 0;
    /** 开口边编号 0~3（局部系 x=宽、y=长），-1 表示未知 */
    int opening_edge = -1;
    std::array<int, 4> edge_point_counts{ 0, 0, 0, 0 };
    std::array<double, 4> edge_heights{ 0.0, 0.0, 0.0, 0.0 };
    std::array<int, 4> edge_height_counts{ 0, 0, 0, 0 };
    std::array<bool, 4> edge_height_valid{ false, false, false, false };
    std::array<double, 4> ship_edge_distances{ 0.0, 0.0, 0.0, 0.0 };
    std::array<bool, 4> ship_edge_distance_valid{ false, false, false, false };
    std::array<DebugLine2D, 4> ship_edge_distance_lines{};
    double ship_center_distance = 0.0;
    double ship_opening_normal_angle = 0.0;
    bool has_ship_metrics = false;
    bool has_opening_segment = false;
    DebugLine2D opening_segment{};
    std::vector<DebugLine2D> visible_edges;
};

enum class AnchorRecoveryStatus {
    NotAttempted = 0,
    NoTrackedBerth = 1,
    InvalidTrackedBerthSize = 2,
    NoTopSearchCandidates = 3,
    InvalidAnchorWidth = 4,
    InvalidTrackedLength = 5,
    Success = 6
};

struct AnchorRecoveryDebug {
    AnchorRecoveryStatus status = AnchorRecoveryStatus::NotAttempted;
    bool tracked_size_valid = false;
    bool searched_top_regions = false;
    bool found_top_regions = false;
    int top_search_candidate_count = 0;
    double anchor_distance = 0.0;
    double tracked_w = 0.0;
    double tracked_l = 0.0;
    double recovered_w = 0.0;
    double recovered_l = 0.0;
    double recovered_angle = 0.0;
};

/** 泊位检测结果 */
struct BerthMeasureResult {
    double timestamp = 0.0;
    bool is_detected = false;
    bool is_using_memory = false;
    bool has_debug_roi = false;
    bool has_top_search_roi = false;
    bool is_line_recovered = false;
    /** 当前雷达系下有效锚点总线（界面/调试用，不写入对外协议） */
    bool has_anchor_bus = false;
    DebugLine2D anchor_bus_line{};
    /** 最近一次地图子图检测范围（已换算到当前雷达系；当前 GNSS-ENU 管线默认不填充） */
    bool has_map_detection_range = false;
    double map_detection_center_x = 0.0;
    double map_detection_center_y = 0.0;
    double map_detection_radius_m = 0.0;
    double map_detection_timestamp = 0.0;
    Berth debug_roi{};
    Berth top_search_roi{};
    std::vector<Berth> debug_rois;
    std::vector<Berth> top_search_rois;
    std::vector<DebugLine2D> debug_lines;
    std::vector<Berth> expanded_berths;
    bool has_highest_point = false;
    bool has_second_highest_point = false;
    Eigen::Vector3d highest_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d second_highest_point = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> highest_points;
    std::vector<Eigen::Vector3d> second_highest_points;
    AnchorRecoveryDebug anchor_debug{};
    std::vector<std::string> debug_messages;
};

/** 泊位检测输入：五雷达融合点云（210 坐标系） */
struct SyncedSensorPackage {
    std::shared_ptr<const LidarFrame> lidar;
};

using SyncedPackagePtr = std::shared_ptr<const SyncedSensorPackage>;

// --- 预留：地图 / SLAM（CommunicationManager 仓库类型）---
struct Pose3d {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
};

struct BoundingBox3D {
    double min_x = 0.0, min_y = 0.0, min_z = 0.0;
    double max_x = 0.0, max_y = 0.0, max_z = 0.0;
};

struct BlockKey {
    int x = 0, y = 0, z = 0;
    bool operator==(const BlockKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

struct BlockKeyHasher {
    std::size_t operator()(const BlockKey &k) const
    {
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.z) << 2);
    }
};

struct PointCloudXYZI {
    std::vector<LidarPoint> points;
};

struct SemanticVoxelGrid {
    BoundingBox3D bbox;
    std::vector<uint8_t> labels;
};

struct KeyFrame {
    uint64_t id = 0;
    double timestamp = 0.0;
    Pose3d pose_wb;
    std::shared_ptr<const PointCloudXYZI> cloud;
};

struct MapBlock {
    BlockKey key;
    BoundingBox3D bbox;
    std::shared_ptr<const PointCloudXYZI> cloud;
    std::shared_ptr<const SemanticVoxelGrid> semantic;
    uint64_t version = 0;
    bool dirty = false;
};

struct MapSnapshot {
    uint64_t version = 0;
    double timestamp = 0.0;
    std::vector<std::shared_ptr<const MapBlock>> blocks;
    std::vector<Pose3d> keyframe_poses;
};

} // namespace usv

#pragma once

#include "bridge_common_types.h"

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
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

// --- 预留：地图 / SLAM（CommunicationManager 仓库类型，当前桥洞总线未使用）---
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

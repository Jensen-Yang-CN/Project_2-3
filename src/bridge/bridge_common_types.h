#pragma once
// 桥洞算法专用类型（勿用 src/core/common_types.h，避免与 Qt 侧类型冲突）
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace usv {

struct LidarPoint {
    float x, y, z;
    float intensity;
};

struct LidarFrame {
    double timestamp = 0.0;
    uint64_t timeline_generation = 1;
    uint32_t point_count = 0;
    std::vector<LidarPoint> points;
};

struct ImageFrame {
    double timestamp = 0.0;
    uint64_t timeline_generation = 1;
    cv::Mat image;
};

struct BridgeMeasureResult {
    double timestamp = 0.0;
    bool is_detected = false;
    Eigen::Vector3d average_center_3d = Eigen::Vector3d::Zero();
    double width = 0.0;
    double height = 0.0;
};

/** GNSS/INS 组合导航，供 LidarGnssSlam 转 ENU 里程计 */
struct GnssInsMessage {
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    int navigation_state = 0;
    bool yaw_valid = true;
    bool pitch_valid = true;
    bool roll_valid = true;
};

struct OdomData {
    double timestamp = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct PoseState {
    double timestamp = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    bool valid = false;
};

} // namespace usv

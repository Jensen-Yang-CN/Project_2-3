#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <vector>

// 引用桥洞类型（包含 LidarPoint, LidarFrame, OdomData, PoseState）
#include "bridge_common_types.h"
#include "pointxyz.h"

namespace usv {

//=============================================================================
// SLAM 专用数据类型
//=============================================================================

/**
 * FusedCloudBundle - 融合点云数据包
 * 由 RadarFusionManager 融合多雷达数据后发布到 Topic 总线
 */
struct FusedCloudBundle {
    double timestamp = 0.0;
    uint64_t frame_seq = 0;
    std::vector<M_PointXYZI> points;  // 融合后的点云点

    // 转换为 LidarFrame 用于 SLAM 算法
    std::shared_ptr<LidarFrame> toLidarFrame() const {
        auto frame = std::make_shared<LidarFrame>();
        frame->timestamp = timestamp;
        frame->point_count = static_cast<uint32_t>(points.size());
        frame->points.reserve(points.size());

        for (const auto& p : points) {
            LidarPoint lp;
            lp.x = p.x;
            lp.y = p.y;
            lp.z = p.z;
            lp.intensity = p.intensity;
            frame->points.push_back(lp);
        }

        return frame;
    }
};

/**
 * Pose3D - 带协方差的三维位姿
 */
struct Pose3D {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity();
    bool valid = false;
};

/**
 * SlamOdometryState - SLAM 里程计状态
 * 发布到 "slam/odometry" Topic
 */
struct SlamOdometryState {
    double timestamp = 0.0;

    // 激光雷达在 map 坐标系下的位姿
    Pose3D pose_lidar;

    uint64_t frame_seq = 0;
    uint64_t keyframe_id = 0;

    // GICP 配准得分 (值越小配准越好)
    double registration_score = 0.0;

    // 是否为关键帧
    bool is_keyframe = false;

    // 获取位置信息
    Eigen::Vector3d position() const {
        return pose_lidar.pose.translation();
    }

    // 获取姿态信息
    Eigen::Quaterniond orientation() const {
        return Eigen::Quaterniond(pose_lidar.pose.linear());
    }
};

/**
 * SlamGeoAnchor - SLAM map 坐标系的 WGS84/ENU 地理锚点
 * 当前导航协议没有独立高程时 altitude_m 为 0，但 valid 仍可为 true。
 */
struct SlamGeoAnchor {
    bool valid = false;
    double timestamp = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

/**
 * SlamGeoPoseReference - 与关键帧雷达时间同步的 GNSS/INS ENU 参考位姿。
 * pose_lidar_enu 与当前 SLAM 会话的 SlamGeoAnchor 使用同一 ENU 原点。
 */
struct SlamGeoPoseReference {
    bool valid = false;
    double timestamp = 0.0;
    double sync_error_sec = 0.0;
    Eigen::Isometry3d pose_lidar_enu = Eigen::Isometry3d::Identity();
};

/**
 * SlamKeyframe - SLAM 关键帧
 * 用于地图构建和管理
 */
struct SlamKeyframe {
    uint64_t keyframe_id = 0;
    double timestamp = 0.0;

    // 关键帧位姿 (T_map_lidar)
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    // 原始 GNSS/INS 导航给出的 ENU 雷达参考位姿（可选）。
    SlamGeoPoseReference geo_reference;

    // 关键帧点云
    std::vector<M_PointXYZI> cloud;

    // 转换为 LidarFrame
    std::shared_ptr<LidarFrame> toLidarFrame() const {
        auto frame = std::make_shared<LidarFrame>();
        frame->timestamp = timestamp;
        frame->point_count = static_cast<uint32_t>(cloud.size());
        frame->points.reserve(cloud.size());

        for (const auto& p : cloud) {
            LidarPoint lp;
            lp.x = p.x;
            lp.y = p.y;
            lp.z = p.z;
            lp.intensity = p.intensity;
            frame->points.push_back(lp);
        }

        return frame;
    }
};

/** 当前 SLAM 扫描的世界系点云（供界面/投递订阅使用）。 */
struct SlamScanCloudMessage {
    double timestamp = 0.0;
    std::vector<M_PointXYZI> points;
};

/**
 * SlamMapBerth - 与 SLAM 地图同一 ENU 坐标系下的持久化泊位标注。
 * 这里只保存绘制和拼接所需的稳定几何信息，不保存检测调试缓存。
 */
struct SlamMapBerth {
    uint64_t id = 0;
    double timestamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double width = 0.0;
    double length = 0.0;
    double angle_deg = 0.0;
    uint8_t kind = 0;       // 1=U 型，2=一字型
    int8_t opening_edge = -1;
    uint32_t observation_count = 0;
    double confidence = 0.0;
};

/**
 * DloSlamConfig - DLO SLAM 配置参数
 */
struct DloSlamConfig {
    // 坐标系名称
    std::string map_frame = "map";
    std::string lidar_frame = "lidar";

    // 点云预处理
    double voxel_leaf_size = 0.5;      // 体素降采样叶子大小
    double min_range = 0.0;            // 最小距离
    double max_range = 200.0;          // 最大距离

    // 水面滤波在融合点云进入 SLAM 前统一执行，点云地图和栅格地图共用结果。
    bool water_filter_enabled = true;
    double water_clearance_m = 0.25;
    double water_plane_distance_m = 0.15;
    double water_max_tilt_deg = 20.0;
    int water_min_inliers = 80;
    int water_max_sample_points = 12000;
    int water_ransac_iterations = 160;
    double water_near_surface_band_m = 1.0;
    double water_outlier_radius_m = 0.60;
    int water_outlier_min_neighbors = 2;
    int water_max_fallback_frames = 15;

    // 关键帧策略
    int max_keyframes = 1000;          // 最大关键帧数量
    double max_keyframe_age_sec = 3600.0;  // 关键帧最长时间(秒)
    double keyframe_translation_thresh = 0.5;   // 关键帧平移阈值(m)
    double keyframe_rotation_thresh_deg = 5.0; // 关键帧旋转阈值(度)

    // Odom 融合参数
    int odom_weight_points_low = 300;
    int odom_weight_points_high = 3000;
    double odom_weight_min = 0.0;
    double odom_weight_max = 0.0;

    // GICP 参数
    int gicp_k_correspondences = 20;
    double gicp_max_correspondence_distance = 1.0e6;
    int gicp_max_iterations = 64;
    double gicp_transformation_epsilon = 5e-4;
    /** NanoGICP OpenMP 线程数（默认 4；<=0 表示用满 CPU 逻辑核） */
    int gicp_num_threads = 4;

    /** true：无 GNSS/INS 时跳过点云；false：退化为纯激光里程计 */
    bool require_gnss_ins = true;
    /** 点云与 INS 时间差超过此值(秒)则视为无效 */
    double max_gnss_sync_sec = 0.25;
};

/**
 * MapUpdateEvent - 地图更新事件
 */
struct MapUpdateEvent {
    enum class Type {
        AddKeyframe,
        RemoveOldKeyframes,
        SaveMapRequested,
        SaveMapFinished,
        ClearMap
    };

    Type type = Type::AddKeyframe;
    double timestamp = 0.0;
    uint64_t keyframe_id = 0;
    std::string message;
};

}  // namespace usv

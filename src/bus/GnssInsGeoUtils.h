#pragma once

#include "NaviProtocol.h"
#include "bridge_common_types.h"

#include <Eigen/Geometry>

namespace usv {
namespace gnss_geo {

/** 211 LiDAR → 船身 INS 外参（slamgnss 标定，仅旋转） */
Eigen::Isometry3d lidar211ToBodyExtrinsic();

/** 210 LiDAR → 211 LiDAR 外参（与 radartransform 五路融合一致） */
Eigen::Isometry3d lidar210To211Extrinsic();

/** 210 融合 LiDAR → 船身 INS：T_211→body × T_210→211 */
Eigen::Isometry3d lidarToBodyExtrinsic();

/** SLAM GNSS 累积 ENU 状态（原点为首个有效 GNSS） */
struct GnssEnuState {
    bool initialized = false;
    GnssInsMessage origin_gnss{};
    GnssInsMessage last_gnss{};
    Eigen::Vector3d accumulated_enu = Eigen::Vector3d::Zero();
};

/** 复用 LidarGnssSlam::convertGnssInsToOdom 的 ENU 累积逻辑 */
void updateGnssEnuState(const GnssInsMessage &gnss, GnssEnuState &state);

/** 由当前 GNSS/INS 姿态 + 累积 ENU 平移，得到船身 INS 在 ENU 下的位姿 */
Eigen::Isometry3d bodyPoseInEnu(const GnssInsMessage &gnss,
                                const GnssEnuState &state);

/**
 * ENU 绝对坐标（相对 origin_gnss）反算经纬度高程。
 * 与 LidarGnssSlam 中 dned → dblh 累积过程互逆。
 */
void enuToGeodetic(const Eigen::Vector3d &enu_m,
                   const GnssEnuState &state,
                   double &lat_deg,
                   double &lon_deg,
                   double &alt_m);

/**
 * LiDAR 系角点 → 船身 INS → ENU → 经纬度。
 * @param p_lidar 210 融合 LiDAR 系坐标（米）
 */
bool cornerLidarToGeodetic(const Eigen::Vector3d &p_lidar,
                           const Eigen::Isometry3d &body_pose_in_enu,
                           const GnssEnuState &enu_state,
                           double &lat_deg,
                           double &lon_deg,
                           double &alt_m,
                           Eigen::Vector3d *p_body_out = nullptr,
                           Eigen::Vector3d *p_enu_out = nullptr);

GnssInsMessage gnssFromImu(const IMUParsedData &imu);

}  // namespace gnss_geo
}  // namespace usv

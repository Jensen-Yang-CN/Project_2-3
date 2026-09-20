#pragma once

#include "bridge_common_types.h"
#include "common/slam_types.h"

#include <Eigen/Geometry>

#include <cmath>

namespace usv {
namespace slam_pose {

inline bool isGnssInsUsable(const GnssInsMessage &gnss)
{
    const bool finite =
        std::isfinite(gnss.timestamp)
        && std::isfinite(gnss.latitude)
        && std::isfinite(gnss.longitude)
        && std::isfinite(gnss.altitude)
        && std::isfinite(gnss.yaw)
        && std::isfinite(gnss.pitch)
        && std::isfinite(gnss.roll);
    if (!finite || gnss.timestamp <= 0.0)
        return false;
    if (!gnss.yaw_valid || !gnss.pitch_valid || !gnss.roll_valid)
        return false;
    if (gnss.latitude < -90.0 || gnss.latitude > 90.0
        || gnss.longitude < -180.0 || gnss.longitude > 180.0) {
        return false;
    }
    return gnss.longitude != 0.0 || gnss.latitude != 0.0;
}

inline Eigen::Isometry3d initialLidarPose(const OdomData &matchedOdom)
{
    return matchedOdom.pose;
}

inline bool navigationReferenceMatches(
    const SlamGeoPoseReference &reference,
    double lidarTimestamp,
    double maxSyncErrorSec)
{
    if (!reference.valid
        || !std::isfinite(reference.timestamp)
        || !std::isfinite(reference.sync_error_sec)
        || !std::isfinite(lidarTimestamp)
        || !std::isfinite(maxSyncErrorSec)
        || maxSyncErrorSec < 0.0
        || reference.sync_error_sec < 0.0
        || reference.sync_error_sec > maxSyncErrorSec
        || std::abs(reference.timestamp - lidarTimestamp) > 1.0e-6) {
        return false;
    }
    const Eigen::Matrix4d matrix = reference.pose_lidar_enu.matrix();
    if (!matrix.allFinite())
        return false;
    const Eigen::Quaterniond quaternion(
        reference.pose_lidar_enu.linear());
    return std::isfinite(quaternion.norm())
        && quaternion.norm() > 1.0e-12;
}

inline SlamGeoPoseReference makeNavigationReference(
    double lidarTimestamp,
    const OdomData &matchedOdom,
    double nearestSourceTimeErrorSec,
    bool hasMatchedNavigation)
{
    SlamGeoPoseReference reference;
    if (!hasMatchedNavigation)
        return reference;
    reference.valid = true;
    reference.timestamp = lidarTimestamp;
    reference.sync_error_sec = nearestSourceTimeErrorSec;
    reference.pose_lidar_enu = matchedOdom.pose;
    return reference;
}

inline Eigen::Isometry3d bodyPoseFromLidarPose(
    const Eigen::Isometry3d &mapToLidar,
    const Eigen::Isometry3d &lidarToBody)
{
    return mapToLidar * lidarToBody.inverse();
}

} // namespace slam_pose
} // namespace usv

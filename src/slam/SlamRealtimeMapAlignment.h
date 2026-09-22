#pragma once

#include "common/slam_types.h"

#include <Eigen/Geometry>

namespace slam_realtime_alignment {

/**
 * Build the fixed transform from the SLAM map frame to the ENU frame used by
 * a keyframe's navigation reference.
 */
inline bool computeMapToEnu(const usv::SlamKeyframe &keyframe,
                            Eigen::Isometry3d &mapToEnu)
{
    if (!keyframe.geo_reference.valid
        || !keyframe.geo_reference.pose_lidar_enu.matrix().allFinite()
        || !keyframe.pose.matrix().allFinite())
        return false;

    const Eigen::Isometry3d candidate =
        keyframe.geo_reference.pose_lidar_enu * keyframe.pose.inverse();
    if (!candidate.matrix().allFinite())
        return false;

    mapToEnu = candidate;
    return true;
}

inline void transformPointCloud(std::vector<M_PointXYZI> &cloud,
                                const Eigen::Isometry3d &mapToEnu)
{
    for (M_PointXYZI &point : cloud) {
        const Eigen::Vector3d transformed =
            mapToEnu * Eigen::Vector3d(point.x, point.y, point.z);
        point.x = static_cast<float>(transformed.x());
        point.y = static_cast<float>(transformed.y());
        point.z = static_cast<float>(transformed.z());
    }
}

inline usv::SlamKeyframe transformKeyframe(
    const usv::SlamKeyframe &keyframe,
    const Eigen::Isometry3d &mapToEnu)
{
    usv::SlamKeyframe aligned = keyframe;
    aligned.pose = mapToEnu * keyframe.pose;
    transformPointCloud(aligned.cloud, mapToEnu);
    return aligned;
}

inline Eigen::Isometry3d transformPose(const Eigen::Isometry3d &pose,
                                       const Eigen::Isometry3d &mapToEnu)
{
    return mapToEnu * pose;
}

} // namespace slam_realtime_alignment

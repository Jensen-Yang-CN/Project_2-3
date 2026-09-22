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

inline usv::SlamKeyframe transformKeyframe(
    const usv::SlamKeyframe &keyframe,
    const Eigen::Isometry3d &mapToEnu)
{
    usv::SlamKeyframe aligned = keyframe;
    // The cloud is expressed in the lidar frame.  The map merge path applies
    // the returned pose to each point, so transforming the cloud here would
    // apply the same coordinate change twice.
    aligned.pose = mapToEnu * keyframe.pose;
    return aligned;
}

inline Eigen::Isometry3d transformPose(const Eigen::Isometry3d &pose,
                                       const Eigen::Isometry3d &mapToEnu)
{
    return mapToEnu * pose;
}

} // namespace slam_realtime_alignment

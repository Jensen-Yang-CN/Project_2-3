#pragma once

#include "common/slam_types.h"

#include <Eigen/Geometry>

namespace slam_realtime_alignment {

/**
 * 根据关键帧的导航参考位姿，计算 SLAM map 坐标系到 ENU 坐标系的固定变换。
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
    // 关键帧点云仍然使用雷达坐标系。地图合并流程会用返回的 pose 统一变换
    // 每个点，因此这里不能再次变换 cloud，否则会发生重复变换导致地图偏移。
    aligned.pose = mapToEnu * keyframe.pose;
    return aligned;
}

inline Eigen::Isometry3d transformPose(const Eigen::Isometry3d &pose,
                                       const Eigen::Isometry3d &mapToEnu)
{
    return mapToEnu * pose;
}

} // namespace slam_realtime_alignment

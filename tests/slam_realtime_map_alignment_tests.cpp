#include "SlamRealtimeMapAlignment.h"
#include "common/slam_types.h"

#include <cassert>
#include <cmath>

namespace {

bool closeTo(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-9;
}

} // namespace

int main()
{
    usv::SlamKeyframe keyframe;
    keyframe.pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
    keyframe.geo_reference.valid = true;
    keyframe.geo_reference.pose_lidar_enu.translation() =
        Eigen::Vector3d(11.0, 22.0, 33.0);
    M_PointXYZI point{};
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    keyframe.cloud.push_back(point);

    Eigen::Isometry3d mapToEnu = Eigen::Isometry3d::Identity();
    assert(slam_realtime_alignment::computeMapToEnu(keyframe, mapToEnu));
    const usv::SlamKeyframe aligned =
        slam_realtime_alignment::transformKeyframe(keyframe, mapToEnu);
    assert(closeTo(aligned.pose.translation().x(), 11.0));
    assert(closeTo(aligned.pose.translation().y(), 22.0));
    assert(closeTo(aligned.pose.translation().z(), 33.0));
    // Keyframe clouds stay in the lidar frame.  mergeKeyframeIntoMap applies
    // the aligned pose exactly once when converting them to world points.
    assert(closeTo(aligned.cloud.front().x, 1.0));
    assert(closeTo(aligned.cloud.front().y, 2.0));
    assert(closeTo(aligned.cloud.front().z, 3.0));

    usv::SlamKeyframe withoutReference;
    assert(!slam_realtime_alignment::computeMapToEnu(
        withoutReference, mapToEnu));
    return 0;
}

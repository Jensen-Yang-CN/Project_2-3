#include "SlamPoseUtils.h"
#include "../common/slam_types.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

int g_failures = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        ++g_failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

Eigen::Isometry3d makePlanarPose(double x, double y, double yaw_rad)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    pose.linear() =
        Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return pose;
}

void testInitialMapPoseUsesMatchedNavigationPose()
{
    usv::OdomData matched_odom;
    matched_odom.timestamp = 100.0;
    matched_odom.pose = makePlanarPose(12.0, -3.5, 0.72);

    const Eigen::Isometry3d initial_pose =
        usv::slam_pose::initialLidarPose(matched_odom);
    check(initial_pose.matrix().isApprox(matched_odom.pose.matrix(), 1e-12),
          "the first SLAM pose must preserve the matched ENU lidar pose");
}

void testBodyPoseIsRecoveredFromLidarPose()
{
    const Eigen::Isometry3d map_to_body =
        makePlanarPose(48.0, -17.0, -0.35);
    Eigen::Isometry3d body_to_lidar =
        makePlanarPose(8.0, 0.3, 1.15);
    body_to_lidar.translation().z() = 1.5;
    const Eigen::Isometry3d map_to_lidar =
        map_to_body * body_to_lidar;

    const Eigen::Isometry3d recovered_body =
        usv::slam_pose::bodyPoseFromLidarPose(
            map_to_lidar, body_to_lidar);
    check(recovered_body.matrix().isApprox(map_to_body.matrix(), 1e-12),
          "the displayed vessel pose must remove the lidar extrinsic");
}

usv::GnssInsMessage validGnssIns()
{
    usv::GnssInsMessage gnss;
    gnss.timestamp = 100.0;
    gnss.latitude = 31.25;
    gnss.longitude = 121.5;
    gnss.altitude = 0.0;
    gnss.yaw = 42.0;
    gnss.pitch = 1.0;
    gnss.roll = -2.0;
    gnss.yaw_valid = true;
    gnss.pitch_valid = true;
    gnss.roll_valid = true;
    return gnss;
}

void testGnssInsValidityChecks()
{
    check(usv::slam_pose::isGnssInsUsable(validGnssIns()),
          "a finite, flagged GNSS/INS sample must be accepted");

    auto invalid_flag = validGnssIns();
    invalid_flag.yaw_valid = false;
    check(!usv::slam_pose::isGnssInsUsable(invalid_flag),
          "an invalid attitude flag must reject the GNSS/INS sample");

    auto invalid_range = validGnssIns();
    invalid_range.latitude = 95.0;
    check(!usv::slam_pose::isGnssInsUsable(invalid_range),
          "an out-of-range latitude must reject the GNSS/INS sample");

    auto invalid_number = validGnssIns();
    invalid_number.yaw = std::numeric_limits<double>::quiet_NaN();
    check(!usv::slam_pose::isGnssInsUsable(invalid_number),
          "a non-finite attitude must reject the GNSS/INS sample");

    auto invalid_timestamp = validGnssIns();
    invalid_timestamp.timestamp = 0.0;
    check(!usv::slam_pose::isGnssInsUsable(invalid_timestamp),
          "a non-positive timestamp must reject the GNSS/INS sample");
}

void testNavigationReferenceMatching()
{
    usv::SlamGeoPoseReference reference;
    reference.valid = true;
    reference.timestamp = 100.0;
    reference.sync_error_sec = 0.02;
    reference.pose_lidar_enu = makePlanarPose(2.0, -1.0, 0.1);
    check(usv::slam_pose::navigationReferenceMatches(
              reference, 100.0, 0.25),
          "a matching finite navigation reference must be accepted");

    reference.sync_error_sec = 0.3;
    check(!usv::slam_pose::navigationReferenceMatches(
              reference, 100.0, 0.25),
          "a stale navigation reference must be rejected");

    reference.sync_error_sec = 0.02;
    check(!usv::slam_pose::navigationReferenceMatches(
              reference, 100.01, 0.25),
          "a reference from another lidar frame must be rejected");

    reference.timestamp = 100.01;
    reference.pose_lidar_enu.translation().x() =
        std::numeric_limits<double>::quiet_NaN();
    check(!usv::slam_pose::navigationReferenceMatches(
              reference, 100.01, 0.25),
          "a non-finite navigation pose must be rejected");
}

void testNavigationReferenceConstruction()
{
    usv::OdomData matched;
    matched.timestamp = 100.0;
    matched.pose = makePlanarPose(7.0, 8.0, -0.2);
    const usv::SlamGeoPoseReference reference =
        usv::slam_pose::makeNavigationReference(
            100.0, matched, 0.03, true);
    check(reference.valid
              && reference.timestamp == 100.0
              && reference.sync_error_sec == 0.03
              && reference.pose_lidar_enu.matrix().isApprox(
                  matched.pose.matrix(), 1e-12),
          "a matched navigation odometry sample must become a geo reference");

    const usv::SlamGeoPoseReference missing =
        usv::slam_pose::makeNavigationReference(
            100.0, matched, 0.03, false);
    check(!missing.valid,
          "a fallback odometry pose must not become a geo reference");
}

} // namespace

int main()
{
    testInitialMapPoseUsesMatchedNavigationPose();
    testBodyPoseIsRecoveredFromLidarPose();
    testGnssInsValidityChecks();
    testNavigationReferenceMatching();
    testNavigationReferenceConstruction();
    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All SLAM pose tests passed\n";
    return 0;
}

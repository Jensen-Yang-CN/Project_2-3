#include "GnssInsGeoUtils.h"

#include "CalibrationConfig.h"
#include "NaviProtocol.h"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

Eigen::Matrix3d rotX(double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c;
    return rotation;
}

Eigen::Matrix3d rotY(double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Eigen::Matrix3d rotation;
    rotation << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
    return rotation;
}

Eigen::Matrix3d rotZ(double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Eigen::Matrix3d rotation;
    rotation << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    return rotation;
}

Eigen::Matrix3d eulerFrdToRnb(double roll_rad, double pitch_rad, double yaw_rad)
{
    return rotZ(yaw_rad) * rotY(pitch_rad) * rotX(roll_rad);
}

Eigen::Matrix3d rEnuNed()
{
    Eigen::Matrix3d rotation;
    rotation << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    return rotation;
}

void getDr(const Eigen::Vector3d &blh_rad, double &rm, double &rn)
{
    constexpr double kEarthA = 6378137.0;
    constexpr double kEarthB = 6356752.3142;
    const double e = std::sqrt(1.0 - (kEarthB / kEarthA) * (kEarthB / kEarthA));
    const double e2 = e * e;

    const double lat = blh_rad.x();
    const double height = blh_rad.z();

    rn = kEarthA / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    rm = kEarthA * (1.0 - e2) /
         std::pow(1.0 - e2 * std::sin(lat) * std::sin(lat), 1.5);
    (void)height;
}

}  // namespace

namespace usv {
namespace gnss_geo {

Eigen::Isometry3d lidar211ToBodyExtrinsic()
{
    return CalibrationConfig::instance().matrix211ToBody();
}

Eigen::Isometry3d lidar210To211Extrinsic()
{
    return CalibrationConfig::instance().matrix210To211();
}

Eigen::Isometry3d lidarToBodyExtrinsic()
{
    return CalibrationConfig::instance().matrix210ToBody();
}

GnssInsMessage gnssFromImu(const IMUParsedData &imu)
{
    GnssInsMessage msg;
    msg.timestamp = imu.timestamp;
    msg.latitude = imu.latitude;
    msg.longitude = imu.longitude;
    msg.altitude = 0.0;
    msg.yaw = imu.yaw;
    msg.pitch = imu.pitch;
    msg.roll = imu.roll;
    msg.navigation_state = imu.naviState;
    msg.yaw_valid = imu.yawValid;
    msg.pitch_valid = imu.pitchValid;
    msg.roll_valid = imu.rollValid;
    return msg;
}

void updateGnssEnuState(const GnssInsMessage &gnss, GnssEnuState &state)
{
    if (gnss.longitude == 0.0 && gnss.latitude == 0.0)
        return;

    if (!state.initialized) {
        state.origin_gnss = gnss;
        state.last_gnss = gnss;
        state.accumulated_enu.setZero();
        state.initialized = true;
        return;
    }

    const Eigen::Vector3d current_blh(
        gnss.latitude * kDegToRad, gnss.longitude * kDegToRad, gnss.altitude);
    const Eigen::Vector3d last_blh(
        state.last_gnss.latitude * kDegToRad,
        state.last_gnss.longitude * kDegToRad,
        state.last_gnss.altitude);

    double rm = 0.0;
    double rn = 0.0;
    getDr(current_blh, rm, rn);

    Eigen::Matrix3d dr = -Eigen::Matrix3d::Identity();
    dr(0, 0) = rm + gnss.altitude;
    dr(1, 1) = (rn + gnss.altitude) * std::cos(current_blh.x());

    const Eigen::Vector3d dblh = current_blh - last_blh;
    const Eigen::Vector3d dned = dr * dblh;
    state.accumulated_enu += Eigen::Vector3d(dned.y(), dned.x(), -dned.z());
    state.last_gnss = gnss;
}

Eigen::Isometry3d bodyPoseInEnu(const GnssInsMessage &gnss,
                                const GnssEnuState &state)
{
    const Eigen::Matrix3d rnb = eulerFrdToRnb(
        gnss.roll * kDegToRad, gnss.pitch * kDegToRad, gnss.yaw * kDegToRad);
    const Eigen::Matrix3d reb = rEnuNed() * rnb;

    Eigen::Isometry3d body_pose = Eigen::Isometry3d::Identity();
    body_pose.linear() = reb;
    body_pose.translation() = state.accumulated_enu;
    return body_pose;
}

void enuToGeodetic(const Eigen::Vector3d &enu_m,
                   const GnssEnuState &state,
                   double &lat_deg,
                   double &lon_deg,
                   double &alt_m)
{
    const Eigen::Vector3d origin_blh(
        state.origin_gnss.latitude * kDegToRad,
        state.origin_gnss.longitude * kDegToRad,
        state.origin_gnss.altitude);

    double rm = 0.0;
    double rn = 0.0;
    getDr(origin_blh, rm, rn);

    const double dlat = enu_m.y() / (rm + state.origin_gnss.altitude);
    const double dlon = enu_m.x() /
                        std::max(1e-6, (rn + state.origin_gnss.altitude)
                                             * std::cos(origin_blh.x()));
    const double dalt = -enu_m.z();

    lat_deg = (origin_blh.x() + dlat) * kRadToDeg;
    lon_deg = (origin_blh.y() + dlon) * kRadToDeg;
    alt_m = origin_blh.z() + dalt;
}

bool cornerLidarToGeodetic(const Eigen::Vector3d &p_lidar,
                           const Eigen::Isometry3d &body_pose_in_enu,
                           const GnssEnuState &enu_state,
                           double &lat_deg,
                           double &lon_deg,
                           double &alt_m,
                           Eigen::Vector3d *p_body_out,
                           Eigen::Vector3d *p_enu_out)
{
    if (!enu_state.initialized)
        return false;

    const Eigen::Vector3d p_body = lidarToBodyExtrinsic() * p_lidar;
    const Eigen::Vector3d p_enu = body_pose_in_enu * p_body;
    enuToGeodetic(p_enu, enu_state, lat_deg, lon_deg, alt_m);

    if (p_body_out)
        *p_body_out = p_body;
    if (p_enu_out)
        *p_enu_out = p_enu;
    return true;
}

}  // namespace gnss_geo
}  // namespace usv

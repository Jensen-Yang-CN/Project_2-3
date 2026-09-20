#include "SlamMapGeoUtils.h"

#include <cmath>

namespace slam_map_geo {

namespace {

constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kPi = 3.14159265358979323846;

bool fail(QString *error, const QString &message)
{
    if (error)
        *error = message;
    return false;
}

double radians(double degrees)
{
    return degrees * kPi / 180.0;
}

Eigen::Vector3d toEcef(const usv::SlamGeoAnchor &anchor)
{
    const double latitude = radians(anchor.latitude_deg);
    const double longitude = radians(anchor.longitude_deg);
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double sinLongitude = std::sin(longitude);
    const double cosLongitude = std::cos(longitude);
    const double eccentricitySquared =
        kWgs84Flattening * (2.0 - kWgs84Flattening);
    const double primeVerticalRadius = kWgs84A
        / std::sqrt(1.0 - eccentricitySquared
                              * sinLatitude * sinLatitude);
    return Eigen::Vector3d(
        (primeVerticalRadius + anchor.altitude_m)
            * cosLatitude * cosLongitude,
        (primeVerticalRadius + anchor.altitude_m)
            * cosLatitude * sinLongitude,
        (primeVerticalRadius * (1.0 - eccentricitySquared)
             + anchor.altitude_m)
            * sinLatitude);
}

}  // namespace

bool validAnchor(const usv::SlamGeoAnchor &anchor)
{
    return anchor.valid
        && std::isfinite(anchor.timestamp)
        && std::isfinite(anchor.latitude_deg)
        && std::isfinite(anchor.longitude_deg)
        && std::isfinite(anchor.altitude_m)
        && anchor.latitude_deg >= -90.0
        && anchor.latitude_deg <= 90.0
        && anchor.longitude_deg >= -180.0
        && anchor.longitude_deg <= 180.0;
}

bool anchorOffsetEnu(const usv::SlamGeoAnchor &base,
                     const usv::SlamGeoAnchor &source,
                     Eigen::Vector3d &offset,
                     QString *error)
{
    if (!validAnchor(base) || !validAnchor(source))
        return fail(error, QStringLiteral("地理锚点无效，无法进行地图粗对齐"));

    const Eigen::Vector3d delta = toEcef(source) - toEcef(base);
    const double latitude = radians(base.latitude_deg);
    const double longitude = radians(base.longitude_deg);
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double sinLongitude = std::sin(longitude);
    const double cosLongitude = std::cos(longitude);

    Eigen::Vector3d candidate;
    candidate.x() = -sinLongitude * delta.x()
                  + cosLongitude * delta.y();
    candidate.y() = -sinLatitude * cosLongitude * delta.x()
                  - sinLatitude * sinLongitude * delta.y()
                  + cosLatitude * delta.z();
    candidate.z() = cosLatitude * cosLongitude * delta.x()
                  + cosLatitude * sinLongitude * delta.y()
                  + sinLatitude * delta.z();
    if (!candidate.allFinite())
        return fail(error, QStringLiteral("地理锚点换算产生了非法数值"));

    offset = candidate;
    return true;
}

}  // namespace slam_map_geo

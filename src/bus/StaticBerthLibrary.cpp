#include "StaticBerthLibrary.h"

#include "BerthUdpPacketBuilder.h"
#include "common_types_extended.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

#include <cmath>
#include <array>
#include <limits>

#include <Eigen/Geometry>

namespace static_berth_library {
namespace {

constexpr double kDedupDistanceM = 5.0;

struct ParsedRecord {
    uint8_t type = 0;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_latitude = 0.0;
    double center_longitude = 0.0;
    double center_altitude = 0.0;
    bool has_geodetic_center = false;
    BerthUdpUnit unit{};
    usv::Berth display_berth;
};

bool finite(double value)
{
    return std::isfinite(value);
}

bool validAnchor(const usv::SlamGeoAnchor &anchor)
{
    return anchor.valid
        && finite(anchor.latitude_deg)
        && finite(anchor.longitude_deg)
        && finite(anchor.altitude_m)
        && anchor.latitude_deg >= -90.0
        && anchor.latitude_deg <= 90.0
        && anchor.longitude_deg >= -180.0
        && anchor.longitude_deg <= 180.0;
}

double radians(double degrees)
{
    return degrees * 3.14159265358979323846 / 180.0;
}

Eigen::Vector3d ecef(double latitude_deg, double longitude_deg,
                     double altitude_m)
{
    constexpr double kWgs84A = 6378137.0;
    constexpr double kWgs84Flattening = 1.0 / 298.257223563;
    const double latitude = radians(latitude_deg);
    const double longitude = radians(longitude_deg);
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
        (primeVerticalRadius + altitude_m)
            * cosLatitude * cosLongitude,
        (primeVerticalRadius + altitude_m)
            * cosLatitude * sinLongitude,
        (primeVerticalRadius * (1.0 - eccentricitySquared)
             + altitude_m)
            * sinLatitude);
}

Eigen::Vector3d geodeticToEnu(double latitude_deg, double longitude_deg,
                              double altitude_m,
                              const usv::SlamGeoAnchor &anchor)
{
    const Eigen::Vector3d delta = ecef(latitude_deg, longitude_deg, altitude_m)
        - ecef(anchor.latitude_deg, anchor.longitude_deg, anchor.altitude_m);
    const double latitude = radians(anchor.latitude_deg);
    const double longitude = radians(anchor.longitude_deg);
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double sinLongitude = std::sin(longitude);
    const double cosLongitude = std::cos(longitude);
    return Eigen::Vector3d(
        -sinLongitude * delta.x() + cosLongitude * delta.y(),
        -sinLatitude * cosLongitude * delta.x()
            - sinLatitude * sinLongitude * delta.y()
            + cosLatitude * delta.z(),
        cosLatitude * cosLongitude * delta.x()
            + cosLatitude * sinLongitude * delta.y()
            + sinLatitude * delta.z());
}

bool number(const QJsonObject &object, const char *name, double &out)
{
    const QJsonValue value = object.value(QLatin1String(name));
    if (!value.isDouble())
        return false;
    out = value.toDouble();
    return finite(out);
}

bool parsePoint(const QJsonObject &object,
                berth_udp::BerthProtocolPoint &point)
{
    double latitude = 0.0;
    double longitude = 0.0;
    double east = 0.0;
    double north = 0.0;
    if (!number(object, "latitude_deg", latitude)
        || !number(object, "longitude_deg", longitude)
        || !number(object, "east_m", east)
        || !number(object, "north_m", north)) {
        return false;
    }

    double altitude = 0.0;
    if (object.contains(QLatin1String("up_m"))) {
        if (!number(object, "up_m", altitude))
            return false;
    } else if (object.contains(QLatin1String("z_m"))) {
        if (!number(object, "z_m", altitude))
            return false;
    }

    point.latitude_deg = latitude;
    point.longitude_deg = longitude;
    point.altitude_m = altitude;
    point.x_m = east;
    point.y_m = north;
    point.z_m = altitude;
    return true;
}

bool parseRecord(const QJsonObject &object, ParsedRecord &out)
{
    const QJsonObject berthType =
        object.value(QLatin1String("berth_type")).toObject();
    const int type = berthType.value(QLatin1String("code")).toInt(0);
    if (type != BerthProtocolType::kLine
        && type != BerthProtocolType::kGroove) {
        return false;
    }

    double width = 0.0;
    double length = 0.0;
    double angle = 0.0;
    if (!number(object, "width_m", width)
        || !number(object, "length_m", length)
        || !number(object, "angle_deg", angle)
        || width <= 0.0 || length <= 0.0) {
        return false;
    }

    const QJsonArray vertices =
        object.value(QLatin1String("vertices")).toArray();
    if (vertices.size() < 4)
        return false;
    std::array<berth_udp::BerthProtocolPoint, 4> points{};
    for (int index = 0; index < 4; ++index) {
        if (!parsePoint(vertices.at(index).toObject(), points[index]))
            return false;
    }

    double centerX = 0.0;
    double centerY = 0.0;
    double centerLatitude = 0.0;
    double centerLongitude = 0.0;
    double centerAltitude = 0.0;
    bool hasGeodeticCenter = false;
    const QJsonObject center =
        object.value(QLatin1String("center")).toObject();
    hasGeodeticCenter = number(center, "latitude_deg", centerLatitude)
        && number(center, "longitude_deg", centerLongitude);
    if (center.contains(QLatin1String("up_m")))
        hasGeodeticCenter = hasGeodeticCenter
            && number(center, "up_m", centerAltitude);
    else if (center.contains(QLatin1String("z_m")))
        hasGeodeticCenter = hasGeodeticCenter
            && number(center, "z_m", centerAltitude);
    if (!number(center, "east_m", centerX)
        || !number(center, "north_m", centerY)) {
        centerX = 0.0;
        centerY = 0.0;
        for (const auto &point : points) {
            centerX += point.x_m;
            centerY += point.y_m;
        }
        centerX *= 0.25;
        centerY *= 0.25;
    }
    if (!hasGeodeticCenter) {
        centerLatitude = 0.0;
        centerLongitude = 0.0;
        centerAltitude = 0.0;
        for (const auto &point : points) {
            centerLatitude += point.latitude_deg;
            centerLongitude += point.longitude_deg;
            centerAltitude += point.altitude_m;
        }
        centerLatitude *= 0.25;
        centerLongitude *= 0.25;
        centerAltitude *= 0.25;
        hasGeodeticCenter = finite(centerLatitude)
            && finite(centerLongitude) && finite(centerAltitude);
    }

    usv::Berth berth;
    berth.w = width;
    berth.l = length;
    berth.cx = centerX;
    berth.cy = centerY;
    berth.angle = angle;
    berth.kind = type == BerthProtocolType::kLine
        ? usv::BerthKind::Line : usv::BerthKind::UShape;
    berth.source = usv::BerthDetectionSource::CoordinateLibrary;
    berth.opening_edge = 0;
    const QJsonObject opening =
        object.value(QLatin1String("opening_edge")).toObject();
    if (opening.contains(QLatin1String(
            "internal_geometric_edge_index_zero_based"))) {
        berth.opening_edge = opening.value(QLatin1String(
            "internal_geometric_edge_index_zero_based")).toInt(0);
    }
    if (berth.opening_edge < 0 || berth.opening_edge >= 4)
        berth.opening_edge = 0;

    out.type = static_cast<uint8_t>(type);
    out.center_x = centerX;
    out.center_y = centerY;
    out.center_latitude = centerLatitude;
    out.center_longitude = centerLongitude;
    out.center_altitude = centerAltitude;
    out.has_geodetic_center = hasGeodeticCenter;
    out.unit = berth_udp::makeBerthUnit(out.type, berth, points);
    out.display_berth = berth;
    return true;
}

bool selectedLibraryRecord(const QJsonObject &object)
{
    return object.value(QLatin1String("record_class")).toString()
               == QLatin1String("berth_library")
        && object.value(QLatin1String("is_final_output")).toBool(false);
}

bool selectedFallbackRecord(const QJsonObject &object)
{
    return object.value(QLatin1String("is_final_output")).toBool(false)
        && object.value(QLatin1String("record_class")).toString()
               != QLatin1String("realtime_detection");
}

bool fail(QString *error, const QString &message)
{
    if (error)
        *error = message;
    return false;
}

}  // namespace

bool load(const QString &path, LoadResult &result, QString *error)
{
    result.units.clear();
    if (error)
        error->clear();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法打开泊位库：%1")
                              .arg(file.errorString()));

    QJsonParseError parseError;
    const QJsonDocument document =
        QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError
        || !document.isObject()) {
        return fail(error, QStringLiteral("泊位库 JSON 无效：%1")
                              .arg(parseError.errorString()));
    }

    const QJsonArray frames =
        document.object().value(QLatin1String("frames")).toArray();
    if (frames.isEmpty())
        return fail(error, QStringLiteral("泊位库没有 frames 数据"));

    std::vector<ParsedRecord> records;
    for (const QJsonValue &frameValue : frames) {
        const QJsonObject frame = frameValue.toObject();
        const QJsonArray detections =
            frame.value(QLatin1String("detections")).toArray();
        for (const QJsonValue &detectionValue : detections) {
            const QJsonObject detection = detectionValue.toObject();
            if (!selectedLibraryRecord(detection))
                continue;
            ParsedRecord record;
            if (parseRecord(detection, record))
                records.push_back(record);
        }
    }

    if (records.empty()) {
        for (const QJsonValue &frameValue : frames) {
            const QJsonObject frame = frameValue.toObject();
            const QJsonArray detections =
                frame.value(QLatin1String("detections")).toArray();
            for (const QJsonValue &detectionValue : detections) {
                const QJsonObject detection = detectionValue.toObject();
                if (!selectedFallbackRecord(detection))
                    continue;
                ParsedRecord record;
                if (parseRecord(detection, record))
                    records.push_back(record);
            }
        }
    }

    if (records.empty())
        return fail(error, QStringLiteral("泊位库没有有效的最终泊位记录"));

    struct Cluster {
        uint8_t type = 0;
        double x = 0.0;
        double y = 0.0;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        bool has_geodetic_center = false;
        BerthUdpUnit unit{};
        usv::Berth display_berth;
    };
    std::vector<Cluster> clusters;
    clusters.reserve(records.size());
    for (const ParsedRecord &record : records) {
        int match = -1;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (int index = 0; index < static_cast<int>(clusters.size()); ++index) {
            const Cluster &cluster = clusters[static_cast<std::size_t>(index)];
            if (cluster.type != record.type)
                continue;
            const double distance = std::hypot(
                cluster.x - record.center_x,
                cluster.y - record.center_y);
            if (distance <= kDedupDistanceM && distance < bestDistance) {
                bestDistance = distance;
                match = index;
            }
        }
        if (match < 0) {
            clusters.push_back(Cluster{
                record.type, record.center_x, record.center_y,
                record.center_latitude, record.center_longitude,
                record.center_altitude, record.has_geodetic_center,
                record.unit, record.display_berth});
        }
    }

    result.units.reserve(clusters.size());
    result.display_berths.reserve(clusters.size());
    for (const Cluster &cluster : clusters) {
        result.units.push_back(cluster.unit);
        result.display_berths.push_back(cluster.display_berth);
        if (!result.display_reference.valid && cluster.has_geodetic_center) {
            result.display_reference.valid = true;
            result.display_reference.latitude_deg = cluster.latitude;
            result.display_reference.longitude_deg = cluster.longitude;
            result.display_reference.altitude_m = cluster.altitude;
            result.display_reference.projected_east_m = cluster.x;
            result.display_reference.projected_north_m = cluster.y;
        }
    }
    return true;
}

bool rebaseDisplayBerthsToAnchor(
    LoadResult &result,
    const usv::SlamGeoAnchor &target_anchor,
    QString *error)
{
    if (error)
        error->clear();
    if (result.display_berths_rebased)
        return true;
    if (!validAnchor(target_anchor))
        return fail(error, QStringLiteral("目标 ENU 地理锚点无效"));
    if (!result.display_reference.valid
        || !finite(result.display_reference.latitude_deg)
        || !finite(result.display_reference.longitude_deg)
        || !finite(result.display_reference.altitude_m)
        || !finite(result.display_reference.projected_east_m)
        || !finite(result.display_reference.projected_north_m)) {
        return fail(error, QStringLiteral("泊位库缺少可用于换算的经纬度参考点"));
    }

    const Eigen::Vector3d targetEnu = geodeticToEnu(
        result.display_reference.latitude_deg,
        result.display_reference.longitude_deg,
        result.display_reference.altitude_m,
        target_anchor);
    const double offsetX = targetEnu.x()
        - result.display_reference.projected_east_m;
    const double offsetY = targetEnu.y()
        - result.display_reference.projected_north_m;
    if (!finite(offsetX) || !finite(offsetY))
        return fail(error, QStringLiteral("泊位库坐标换算产生了非法平移"));

    for (usv::Berth &berth : result.display_berths) {
        berth.cx += offsetX;
        berth.cy += offsetY;
    }
    result.display_berths_rebased = true;
    return true;
}

}  // namespace static_berth_library

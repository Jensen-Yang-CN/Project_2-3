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

namespace static_berth_library {
namespace {

constexpr double kDedupDistanceM = 5.0;

struct ParsedRecord {
    uint8_t type = 0;
    double center_x = 0.0;
    double center_y = 0.0;
    BerthUdpUnit unit{};
    usv::Berth display_berth;
};

bool finite(double value)
{
    return std::isfinite(value);
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
    const QJsonObject center =
        object.value(QLatin1String("center")).toObject();
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
                record.type, record.center_x, record.center_y, record.unit,
                record.display_berth});
        }
    }

    result.units.reserve(clusters.size());
    result.display_berths.reserve(clusters.size());
    for (const Cluster &cluster : clusters) {
        result.units.push_back(cluster.unit);
        result.display_berths.push_back(cluster.display_berth);
    }
    return true;
}

}  // namespace static_berth_library

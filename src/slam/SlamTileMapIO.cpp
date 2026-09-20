#include "SlamTileMapIO.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace slam_tile {

namespace {

const QByteArray kManifestMagic("USV_TILE_MAP");
const QByteArray kTileMagic("USVTILE1");
const QByteArray kTrajectoryMagic("USVTRAJ1");
constexpr quint16 kPointRecordSize = 13;
constexpr quint64 kMaxTilePoints = 20000000;
constexpr quint64 kMaxTrajectoryPoses = 10000000;
constexpr int kMaxManifestBerths = 10000;

bool fail(QString *error, const QString &message)
{
    if (error)
        *error = message;
    return false;
}

void configureStream(QDataStream &stream)
{
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setVersion(QDataStream::Qt_5_12);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
}

bool finite(double value)
{
    return std::isfinite(value);
}

quint64 unsignedFromJson(const QJsonValue &value)
{
    if (value.isString())
        return value.toString().toULongLong();
    if (!value.isDouble())
        return 0;
    const double number = value.toDouble(-1.0);
    if (!finite(number) || number < 0.0
        || number > static_cast<double>(std::numeric_limits<quint64>::max())) {
        return 0;
    }
    return static_cast<quint64>(number);
}

bool validAnchor(const usv::SlamGeoAnchor &anchor)
{
    return anchor.valid && finite(anchor.timestamp)
        && finite(anchor.latitude_deg) && finite(anchor.longitude_deg)
        && finite(anchor.altitude_m)
        && anchor.latitude_deg >= -90.0 && anchor.latitude_deg <= 90.0
        && anchor.longitude_deg >= -180.0 && anchor.longitude_deg <= 180.0
        && (anchor.latitude_deg != 0.0 || anchor.longitude_deg != 0.0);
}

bool validLods(const QVector<LodLevel> &lods)
{
    if (lods.isEmpty())
        return false;
    QSet<int> levels;
    for (const LodLevel &lod : lods) {
        if (lod.level < 0 || !finite(lod.voxel_size_m)
            || lod.voxel_size_m <= 0.0 || levels.contains(lod.level)) {
            return false;
        }
        levels.insert(lod.level);
    }
    return true;
}

bool validBerth(const usv::SlamMapBerth &berth)
{
    return finite(berth.timestamp) && finite(berth.x)
        && finite(berth.y) && finite(berth.z)
        && finite(berth.width) && finite(berth.length)
        && finite(berth.angle_deg) && finite(berth.confidence)
        && berth.width > 1.0e-3 && berth.length > 1.0e-3
        && berth.kind <= 2 && berth.opening_edge >= -1
        && berth.opening_edge < 4 && berth.confidence >= 0.0
        && berth.confidence <= 1.0;
}

bool validManifest(const Manifest &manifest, QString *error)
{
    if (manifest.format_major != kTileMapFormatMajor
        || manifest.format_minor > kTileMapFormatMinor) {
        return fail(error, QStringLiteral("不支持的分块地图版本"));
    }
    if (!validAnchor(manifest.anchor))
        return fail(error, QStringLiteral("分块地图地理锚点无效"));
    if (!manifest.bounds.valid())
        return fail(error, QStringLiteral("分块地图边界无效"));
    if (!finite(manifest.grid_origin_x)
        || !finite(manifest.grid_origin_y)
        || !finite(manifest.tile_size_m)
        || manifest.tile_size_m <= 0.0) {
        return fail(error, QStringLiteral("分块地图网格参数无效"));
    }
    if (!validLods(manifest.lods))
        return fail(error, QStringLiteral("分块地图 LOD 参数无效"));
    if (manifest.sources.isEmpty())
        return fail(error, QStringLiteral("分块地图缺少来源信息"));
    if (manifest.berths.size() > kMaxManifestBerths)
        return fail(error, QStringLiteral("分块地图泊位数量超限"));
    for (const usv::SlamMapBerth &berth : manifest.berths) {
        if (!validBerth(berth))
            return fail(error, QStringLiteral("分块地图泊位记录无效"));
    }
    if (manifest.tiles.isEmpty())
        return fail(error, QStringLiteral("分块地图不包含 Tile"));
    for (const SourceFingerprint &source : manifest.sources) {
        if (source.canonical_path.isEmpty())
            return fail(error, QStringLiteral("分块地图来源路径为空"));
    }
    QSet<TileId> ids;
    for (const TileMeta &tile : manifest.tiles) {
        if (tile.id.lod < 0 || tile.relative_path.isEmpty()
            || !tile.bounds.valid() || tile.point_count == 0
            || tile.point_count > kMaxTilePoints || tile.byte_size == 0
            || tile.checksum_sha256.size() != 32
            || ids.contains(tile.id)) {
            return fail(error, QStringLiteral("分块地图 Tile 元数据无效"));
        }
        ids.insert(tile.id);
    }
    return true;
}

QJsonObject boundsToJson(const Bounds3d &bounds)
{
    QJsonObject object;
    object.insert(QStringLiteral("min_x"), bounds.min_x);
    object.insert(QStringLiteral("min_y"), bounds.min_y);
    object.insert(QStringLiteral("min_z"), bounds.min_z);
    object.insert(QStringLiteral("max_x"), bounds.max_x);
    object.insert(QStringLiteral("max_y"), bounds.max_y);
    object.insert(QStringLiteral("max_z"), bounds.max_z);
    return object;
}

Bounds3d boundsFromJson(const QJsonObject &object)
{
    return {object.value(QStringLiteral("min_x")).toDouble(
                std::numeric_limits<double>::quiet_NaN()),
            object.value(QStringLiteral("min_y")).toDouble(
                std::numeric_limits<double>::quiet_NaN()),
            object.value(QStringLiteral("min_z")).toDouble(
                std::numeric_limits<double>::quiet_NaN()),
            object.value(QStringLiteral("max_x")).toDouble(
                std::numeric_limits<double>::quiet_NaN()),
            object.value(QStringLiteral("max_y")).toDouble(
                std::numeric_limits<double>::quiet_NaN()),
            object.value(QStringLiteral("max_z")).toDouble(
                std::numeric_limits<double>::quiet_NaN())};
}

QByteArray sha256ForFile(const QString &path, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        fail(error, QStringLiteral("无法读取文件用于校验: %1").arg(path));
        return {};
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) {
        fail(error, QStringLiteral("计算文件校验值失败: %1").arg(path));
        return {};
    }
    return hash.result();
}

bool createParentDirectory(const QString &path, QString *error)
{
    const QString parent = QFileInfo(path).absolutePath();
    if (!QDir().mkpath(parent))
        return fail(error, QStringLiteral("无法创建目录: %1").arg(parent));
    return true;
}

}  // namespace

QString tileRelativePath(const TileId &id)
{
    return QStringLiteral("L%1/tile_%2_%3.tilebin")
        .arg(id.lod).arg(id.x).arg(id.y);
}

bool saveManifest(const QString &path, const Manifest &manifest,
                  QString *error)
{
    if (!validManifest(manifest, error) || !createParentDirectory(path, error))
        return false;

    QJsonObject root;
    root.insert(QStringLiteral("format_magic"),
                QString::fromLatin1(kManifestMagic));
    root.insert(QStringLiteral("format_major"), manifest.format_major);
    root.insert(QStringLiteral("format_minor"), manifest.format_minor);

    QJsonObject anchor;
    anchor.insert(QStringLiteral("valid"), manifest.anchor.valid);
    anchor.insert(QStringLiteral("timestamp"), manifest.anchor.timestamp);
    anchor.insert(QStringLiteral("latitude_deg"),
                  manifest.anchor.latitude_deg);
    anchor.insert(QStringLiteral("longitude_deg"),
                  manifest.anchor.longitude_deg);
    anchor.insert(QStringLiteral("altitude_m"), manifest.anchor.altitude_m);
    root.insert(QStringLiteral("anchor"), anchor);
    root.insert(QStringLiteral("bounds"), boundsToJson(manifest.bounds));
    root.insert(QStringLiteral("grid_origin_x"), manifest.grid_origin_x);
    root.insert(QStringLiteral("grid_origin_y"), manifest.grid_origin_y);
    root.insert(QStringLiteral("tile_size_m"), manifest.tile_size_m);
    root.insert(QStringLiteral("trajectory"),
                manifest.trajectory_relative_path);

    QJsonArray lods;
    for (const LodLevel &lod : manifest.lods) {
        QJsonObject object;
        object.insert(QStringLiteral("level"), lod.level);
        object.insert(QStringLiteral("voxel_size_m"), lod.voxel_size_m);
        lods.append(object);
    }
    root.insert(QStringLiteral("lods"), lods);

    QJsonArray sources;
    for (const SourceFingerprint &source : manifest.sources) {
        QJsonObject object;
        object.insert(QStringLiteral("path"), source.canonical_path);
        object.insert(QStringLiteral("size_bytes"),
                      QString::number(source.size_bytes));
        object.insert(QStringLiteral("modified_msecs_utc"),
                      QString::number(source.modified_msecs_utc));
        sources.append(object);
    }
    root.insert(QStringLiteral("sources"), sources);

    QJsonArray tiles;
    for (const TileMeta &tile : manifest.tiles) {
        QJsonObject object;
        object.insert(QStringLiteral("x"), tile.id.x);
        object.insert(QStringLiteral("y"), tile.id.y);
        object.insert(QStringLiteral("lod"), tile.id.lod);
        object.insert(QStringLiteral("path"), tile.relative_path);
        object.insert(QStringLiteral("bounds"), boundsToJson(tile.bounds));
        object.insert(QStringLiteral("point_count"),
                      QString::number(tile.point_count));
        object.insert(QStringLiteral("byte_size"),
                      QString::number(tile.byte_size));
        object.insert(QStringLiteral("sha256"),
                      QString::fromLatin1(tile.checksum_sha256.toHex()));
        tiles.append(object);
    }
    root.insert(QStringLiteral("tiles"), tiles);

    QJsonArray berths;
    for (const usv::SlamMapBerth &berth : manifest.berths) {
        QJsonObject object;
        object.insert(QStringLiteral("id"), QString::number(berth.id));
        object.insert(QStringLiteral("timestamp"), berth.timestamp);
        object.insert(QStringLiteral("x"), berth.x);
        object.insert(QStringLiteral("y"), berth.y);
        object.insert(QStringLiteral("z"), berth.z);
        object.insert(QStringLiteral("width"), berth.width);
        object.insert(QStringLiteral("length"), berth.length);
        object.insert(QStringLiteral("angle_deg"), berth.angle_deg);
        object.insert(QStringLiteral("kind"), static_cast<int>(berth.kind));
        object.insert(QStringLiteral("opening_edge"),
                      static_cast<int>(berth.opening_edge));
        object.insert(QStringLiteral("observation_count"),
                      QString::number(berth.observation_count));
        object.insert(QStringLiteral("confidence"), berth.confidence);
        berths.append(object);
    }
    root.insert(QStringLiteral("berths"), berths);

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return fail(error, QStringLiteral("无法写入 Manifest: %1").arg(path));
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size() || !file.commit())
        return fail(error, QStringLiteral("提交 Manifest 失败: %1").arg(path));
    return true;
}

bool loadManifest(const QString &path, Manifest &manifest, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法读取 Manifest: %1").arg(path));
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(
        file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
        return fail(error, QStringLiteral("Manifest JSON 无效"));
    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("format_magic")).toString()
        != QString::fromLatin1(kManifestMagic)) {
        return fail(error, QStringLiteral("Manifest 魔数无效"));
    }

    Manifest candidate;
    candidate.format_major = root.value(QStringLiteral("format_major")).toInt(-1);
    candidate.format_minor = root.value(QStringLiteral("format_minor")).toInt(-1);
    const QJsonObject anchor = root.value(QStringLiteral("anchor")).toObject();
    candidate.anchor.valid = anchor.value(QStringLiteral("valid")).toBool(false);
    candidate.anchor.timestamp = anchor.value(QStringLiteral("timestamp")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    candidate.anchor.latitude_deg =
        anchor.value(QStringLiteral("latitude_deg")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
    candidate.anchor.longitude_deg =
        anchor.value(QStringLiteral("longitude_deg")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
    candidate.anchor.altitude_m =
        anchor.value(QStringLiteral("altitude_m")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
    candidate.bounds = boundsFromJson(root.value(QStringLiteral("bounds")).toObject());
    candidate.grid_origin_x = root.value(QStringLiteral("grid_origin_x")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    candidate.grid_origin_y = root.value(QStringLiteral("grid_origin_y")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    candidate.tile_size_m = root.value(QStringLiteral("tile_size_m")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    candidate.trajectory_relative_path =
        root.value(QStringLiteral("trajectory")).toString();

    for (const QJsonValue &value : root.value(QStringLiteral("lods")).toArray()) {
        const QJsonObject object = value.toObject();
        candidate.lods.append({object.value(QStringLiteral("level")).toInt(-1),
                               object.value(QStringLiteral("voxel_size_m")).toDouble(
                                   std::numeric_limits<double>::quiet_NaN())});
    }
    for (const QJsonValue &value : root.value(QStringLiteral("sources")).toArray()) {
        const QJsonObject object = value.toObject();
        candidate.sources.append({object.value(QStringLiteral("path")).toString(),
            object.value(QStringLiteral("size_bytes")).toString().toULongLong(),
            object.value(QStringLiteral("modified_msecs_utc")).toString().toLongLong()});
    }
    for (const QJsonValue &value : root.value(QStringLiteral("tiles")).toArray()) {
        const QJsonObject object = value.toObject();
        TileMeta tile;
        tile.id = {object.value(QStringLiteral("x")).toInt(),
                   object.value(QStringLiteral("y")).toInt(),
                   object.value(QStringLiteral("lod")).toInt(-1)};
        tile.relative_path = object.value(QStringLiteral("path")).toString();
        tile.bounds = boundsFromJson(object.value(QStringLiteral("bounds")).toObject());
        tile.point_count = object.value(QStringLiteral("point_count")).toString().toULongLong();
        tile.byte_size = object.value(QStringLiteral("byte_size")).toString().toULongLong();
        tile.checksum_sha256 = QByteArray::fromHex(
            object.value(QStringLiteral("sha256")).toString().toLatin1());
        candidate.tiles.append(tile);
    }
    for (const QJsonValue &value : root.value(QStringLiteral("berths")).toArray()) {
        const QJsonObject object = value.toObject();
        usv::SlamMapBerth berth;
        berth.id = unsignedFromJson(object.value(QStringLiteral("id")));
        berth.timestamp = object.value(QStringLiteral("timestamp")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.x = object.value(QStringLiteral("x")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.y = object.value(QStringLiteral("y")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.z = object.value(QStringLiteral("z")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.width = object.value(QStringLiteral("width")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.length = object.value(QStringLiteral("length")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.angle_deg = object.value(QStringLiteral("angle_deg")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        berth.kind = static_cast<uint8_t>(
            object.value(QStringLiteral("kind")).toInt(0));
        berth.opening_edge = static_cast<int8_t>(
            object.value(QStringLiteral("opening_edge")).toInt(-1));
        const quint64 observationCount = unsignedFromJson(
            object.value(QStringLiteral("observation_count")));
        berth.observation_count = observationCount > std::numeric_limits<uint32_t>::max()
            ? std::numeric_limits<uint32_t>::max()
            : static_cast<uint32_t>(observationCount);
        berth.confidence = object.value(QStringLiteral("confidence")).toDouble(
            std::numeric_limits<double>::quiet_NaN());
        candidate.berths.append(berth);
    }
    if (!validManifest(candidate, error))
        return false;
    manifest = std::move(candidate);
    return true;
}

bool saveTile(const QString &path, const TileData &tile,
              QByteArray *checksumSha256, QString *error)
{
    if (tile.points.isEmpty() || tile.points.size() > static_cast<int>(kMaxTilePoints)
        || !finite(tile.tile_origin_x) || !finite(tile.tile_origin_y)
        || !tile.meta.bounds.valid()) {
        return fail(error, QStringLiteral("Tile 数据无效"));
    }
    if (!createParentDirectory(path, error))
        return false;

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return fail(error, QStringLiteral("无法写入 Tile: %1").arg(path));
    QDataStream stream(&file);
    configureStream(stream);
    if (stream.writeRawData(kTileMagic.constData(), kTileMagic.size())
        != kTileMagic.size()) {
        return fail(error, QStringLiteral("写入 Tile 魔数失败"));
    }
    stream << static_cast<quint16>(kTileMapFormatMajor)
           << static_cast<quint16>(kTileMapFormatMinor)
           << static_cast<qint32>(tile.meta.id.x)
           << static_cast<qint32>(tile.meta.id.y)
           << static_cast<qint32>(tile.meta.id.lod)
           << tile.tile_origin_x << tile.tile_origin_y
           << tile.meta.bounds.min_x << tile.meta.bounds.min_y
           << tile.meta.bounds.min_z << tile.meta.bounds.max_x
           << tile.meta.bounds.max_y << tile.meta.bounds.max_z
           << static_cast<quint64>(tile.points.size())
           << kPointRecordSize;
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (const M_PointXYZI &point : tile.points) {
        const float localX = point.x - static_cast<float>(tile.tile_origin_x);
        const float localY = point.y - static_cast<float>(tile.tile_origin_y);
        stream << localX << localY << point.z
               << static_cast<quint8>(point.intensity);
    }
    if (stream.status() != QDataStream::Ok || !file.commit())
        return fail(error, QStringLiteral("提交 Tile 失败: %1").arg(path));
    const QByteArray checksum = sha256ForFile(path, error);
    if (checksum.size() != 32)
        return false;
    if (checksumSha256)
        *checksumSha256 = checksum;
    return true;
}

bool loadTile(const QString &path, const QByteArray &expectedChecksumSha256,
              TileData &tile, QString *error)
{
    if (expectedChecksumSha256.size() == 32) {
        const QByteArray actual = sha256ForFile(path, error);
        if (actual.size() != 32)
            return false;
        if (actual != expectedChecksumSha256)
            return fail(error, QStringLiteral("Tile 校验值不匹配: %1").arg(path));
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法读取 Tile: %1").arg(path));
    QDataStream stream(&file);
    configureStream(stream);
    QByteArray magic(kTileMagic.size(), Qt::Uninitialized);
    if (stream.readRawData(magic.data(), magic.size()) != magic.size()
        || magic != kTileMagic) {
        return fail(error, QStringLiteral("Tile 魔数无效"));
    }
    quint16 major = 0;
    quint16 minor = 0;
    qint32 x = 0;
    qint32 y = 0;
    qint32 lod = 0;
    quint64 count = 0;
    quint16 recordSize = 0;
    TileData candidate;
    stream >> major >> minor >> x >> y >> lod
           >> candidate.tile_origin_x >> candidate.tile_origin_y
           >> candidate.meta.bounds.min_x >> candidate.meta.bounds.min_y
           >> candidate.meta.bounds.min_z >> candidate.meta.bounds.max_x
           >> candidate.meta.bounds.max_y >> candidate.meta.bounds.max_z
           >> count >> recordSize;
    if (stream.status() != QDataStream::Ok
        || major != kTileMapFormatMajor || minor > kTileMapFormatMinor
        || lod < 0 || count == 0 || count > kMaxTilePoints
        || recordSize != kPointRecordSize
        || !finite(candidate.tile_origin_x)
        || !finite(candidate.tile_origin_y)
        || !candidate.meta.bounds.valid()) {
        return fail(error, QStringLiteral("Tile 文件头无效"));
    }
    candidate.meta.id = {x, y, lod};
    candidate.meta.point_count = count;
    candidate.meta.byte_size = static_cast<quint64>(file.size());
    candidate.meta.checksum_sha256 = expectedChecksumSha256;
    candidate.points.reserve(static_cast<int>(count));
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (quint64 index = 0; index < count; ++index) {
        float localX = 0.0f;
        float localY = 0.0f;
        float z = 0.0f;
        quint8 intensity = 0;
        stream >> localX >> localY >> z >> intensity;
        if (stream.status() != QDataStream::Ok
            || !std::isfinite(localX) || !std::isfinite(localY)
            || !std::isfinite(z)) {
            return fail(error, QStringLiteral("Tile 点数据不完整或无效"));
        }
        M_PointXYZI point{};
        point.x = localX + static_cast<float>(candidate.tile_origin_x);
        point.y = localY + static_cast<float>(candidate.tile_origin_y);
        point.z = z;
        point.intensity = intensity;
        candidate.points.append(point);
    }
    if (!file.atEnd())
        return fail(error, QStringLiteral("Tile 包含多余数据"));
    tile = std::move(candidate);
    return true;
}

bool saveTrajectory(const QString &path,
                    const QVector<TrajectoryPose> &trajectory,
                    QString *error)
{
    if (trajectory.isEmpty() || !createParentDirectory(path, error))
        return fail(error, QStringLiteral("轨迹为空或目录无效"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return fail(error, QStringLiteral("无法写入轨迹: %1").arg(path));
    QDataStream stream(&file);
    configureStream(stream);
    stream.writeRawData(kTrajectoryMagic.constData(), kTrajectoryMagic.size());
    stream << static_cast<quint16>(kTileMapFormatMajor)
           << static_cast<quint16>(kTileMapFormatMinor)
           << static_cast<quint64>(trajectory.size());
    for (const TrajectoryPose &pose : trajectory) {
        const Eigen::Quaterniond quaternion(pose.pose_lidar_enu.linear());
        const Eigen::Vector3d translation = pose.pose_lidar_enu.translation();
        if (!finite(pose.timestamp) || !pose.pose_lidar_enu.matrix().allFinite()
            || quaternion.norm() < 1e-12) {
            return fail(error, QStringLiteral("轨迹位姿无效"));
        }
        const Eigen::Quaterniond normalized = quaternion.normalized();
        stream << pose.keyframe_id << pose.timestamp
               << translation.x() << translation.y() << translation.z()
               << normalized.x() << normalized.y() << normalized.z()
               << normalized.w();
    }
    if (stream.status() != QDataStream::Ok || !file.commit())
        return fail(error, QStringLiteral("提交轨迹失败"));
    return true;
}

bool loadTrajectory(const QString &path,
                    QVector<TrajectoryPose> &trajectory,
                    QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法读取轨迹: %1").arg(path));
    QDataStream stream(&file);
    configureStream(stream);
    QByteArray magic(kTrajectoryMagic.size(), Qt::Uninitialized);
    if (stream.readRawData(magic.data(), magic.size()) != magic.size()
        || magic != kTrajectoryMagic) {
        return fail(error, QStringLiteral("轨迹魔数无效"));
    }
    quint16 major = 0;
    quint16 minor = 0;
    quint64 count = 0;
    stream >> major >> minor >> count;
    if (stream.status() != QDataStream::Ok
        || major != kTileMapFormatMajor || minor > kTileMapFormatMinor
        || count == 0 || count > kMaxTrajectoryPoses) {
        return fail(error, QStringLiteral("轨迹文件头无效"));
    }
    QVector<TrajectoryPose> candidate;
    candidate.reserve(static_cast<int>(count));
    for (quint64 index = 0; index < count; ++index) {
        TrajectoryPose pose;
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        stream >> pose.keyframe_id >> pose.timestamp
               >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        if (stream.status() != QDataStream::Ok || !finite(pose.timestamp)
            || !finite(tx) || !finite(ty) || !finite(tz)
            || !finite(qx) || !finite(qy) || !finite(qz) || !finite(qw)
            || quaternion.norm() < 1e-12) {
            return fail(error, QStringLiteral("轨迹位姿无效"));
        }
        quaternion.normalize();
        pose.pose_lidar_enu = Eigen::Isometry3d::Identity();
        pose.pose_lidar_enu.translation() = Eigen::Vector3d(tx, ty, tz);
        pose.pose_lidar_enu.linear() = quaternion.toRotationMatrix();
        candidate.append(pose);
    }
    if (!file.atEnd())
        return fail(error, QStringLiteral("轨迹包含多余数据"));
    trajectory = std::move(candidate);
    return true;
}

SourceFingerprint sourceFingerprint(const QString &path)
{
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    return {QDir::cleanPath(canonical.isEmpty()
                                ? info.absoluteFilePath() : canonical),
            info.exists() ? static_cast<quint64>(info.size()) : 0,
            info.exists() ? info.lastModified().toMSecsSinceEpoch() : 0};
}

QString cacheKey(const QVector<SourceFingerprint> &sources,
                 double tileSizeM,
                 const QVector<LodLevel> &lods)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    configureStream(stream);
    stream << static_cast<quint16>(kTileMapFormatMajor)
           << static_cast<quint16>(kTileMapFormatMinor)
           << tileSizeM << static_cast<quint32>(sources.size());
    for (const SourceFingerprint &source : sources) {
        stream << QDir::cleanPath(source.canonical_path)
               << source.size_bytes << source.modified_msecs_utc;
    }
    stream << static_cast<quint32>(lods.size());
    for (const LodLevel &lod : lods)
        stream << static_cast<qint32>(lod.level) << lod.voxel_size_m;
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

bool manifestMatches(const Manifest &manifest,
                     const QVector<SourceFingerprint> &sources,
                     double tileSizeM,
                     const QVector<LodLevel> &lods)
{
    if (manifest.format_major != kTileMapFormatMajor
        || manifest.format_minor != kTileMapFormatMinor
        || manifest.sources != sources
        || manifest.lods.size() != lods.size()
        || std::abs(manifest.tile_size_m - tileSizeM) > 1e-9) {
        return false;
    }
    for (int index = 0; index < lods.size(); ++index) {
        if (manifest.lods[index].level != lods[index].level
            || std::abs(manifest.lods[index].voxel_size_m
                        - lods[index].voxel_size_m) > 1e-9) {
            return false;
        }
    }
    return true;
}

bool cacheFilesPresent(const QString &manifestPath,
                       const Manifest &manifest,
                       QString *error)
{
    const QDir root = QFileInfo(manifestPath).absoluteDir();
    const QString rootPrefix = QDir::fromNativeSeparators(
        QDir::cleanPath(root.absolutePath())) + QLatin1Char('/');
    auto checkedPath = [&root, &rootPrefix](const QString &relative,
                                            QString &absolute) {
        if (relative.isEmpty() || QDir::isAbsolutePath(relative))
            return false;
        absolute = QDir::fromNativeSeparators(
            QDir::cleanPath(root.absoluteFilePath(relative)));
        return absolute.startsWith(rootPrefix, Qt::CaseInsensitive);
    };

    if (!manifest.trajectory_relative_path.isEmpty()) {
        QString trajectoryPath;
        if (!checkedPath(manifest.trajectory_relative_path, trajectoryPath)
            || !QFileInfo(trajectoryPath).isFile()
            || QFileInfo(trajectoryPath).size() <= 0) {
            return fail(error, QStringLiteral("分块地图轨迹文件缺失"));
        }
    }
    for (const TileMeta &tile : manifest.tiles) {
        QString tilePath;
        if (!checkedPath(tile.relative_path, tilePath))
            return fail(error, QStringLiteral("分块地图 Tile 路径越界"));
        const QFileInfo info(tilePath);
        if (!info.isFile()
            || static_cast<quint64>(info.size()) != tile.byte_size) {
            return fail(error, QStringLiteral("分块地图 Tile 缺失或大小不符: %1")
                                   .arg(tile.relative_path));
        }
    }
    return true;
}

}  // namespace slam_tile

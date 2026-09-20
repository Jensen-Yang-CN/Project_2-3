#include "SlamTileMapBuilder.h"

#include "SlamTileMapIO.h"

#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QUuid>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_set>

namespace slam_tile {

namespace {

struct VoxelKey {
    qint64 x = 0;
    qint64 y = 0;
    qint64 z = 0;

    bool operator==(const VoxelKey &other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const noexcept
    {
        std::size_t seed = std::hash<qint64>{}(key.x);
        seed ^= std::hash<qint64>{}(key.y)
            + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<qint64>{}(key.z)
            + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct Accumulator {
    QVector<M_PointXYZI> points;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    Bounds3d bounds;
    bool has_bounds = false;
};

bool finitePoint(const Eigen::Vector3d &point)
{
    return point.allFinite();
}

void addToBounds(Bounds3d &bounds, bool &hasBounds,
                 double x, double y, double z)
{
    if (!hasBounds) {
        bounds = {x, y, z, x, y, z};
        hasBounds = true;
        return;
    }
    bounds.min_x = std::min(bounds.min_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.min_z = std::min(bounds.min_z, z);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.max_y = std::max(bounds.max_y, y);
    bounds.max_z = std::max(bounds.max_z, z);
}

QString failBuild(BuildResult &result, const QString &message)
{
    result.error = message;
    return message;
}

bool validOptions(const BuildOptions &options)
{
    if (!std::isfinite(options.tile_size_m) || options.tile_size_m <= 0.0
        || options.lods.isEmpty()) {
        return false;
    }
    QSet<int> levels;
    for (const LodLevel &lod : options.lods) {
        if (lod.level < 0 || levels.contains(lod.level)
            || !std::isfinite(lod.voxel_size_m)
            || lod.voxel_size_m <= 0.0) {
            return false;
        }
        levels.insert(lod.level);
    }
    return true;
}

}  // namespace

BuildResult buildTileCache(
    const slam_map_io::MapArchive &archive,
    const QVector<SourceFingerprint> &sources,
    const QString &cacheRoot,
    const QString &cacheKeyValue,
    const BuildOptions &options)
{
    BuildResult result;
    if (!archive.anchor.valid || archive.keyframes.empty()) {
        failBuild(result, QStringLiteral("统一 ENU 地图缺少有效锚点或关键帧"));
        return result;
    }
    if (sources.isEmpty() || cacheRoot.isEmpty() || cacheKeyValue.isEmpty()
        || !validOptions(options)) {
        failBuild(result, QStringLiteral("分块地图构建参数无效"));
        return result;
    }

    QDir root(cacheRoot);
    if (!root.mkpath(QStringLiteral("."))) {
        failBuild(result, QStringLiteral("无法创建分块缓存根目录"));
        return result;
    }
    const QString finalDirectory = root.absoluteFilePath(cacheKeyValue);
    const QString finalManifest =
        QDir(finalDirectory).absoluteFilePath(QStringLiteral("manifest.json"));
    Manifest cached;
    QString ioError;
    if (loadManifest(finalManifest, cached, &ioError)
        && manifestMatches(cached, sources, options.tile_size_m, options.lods)
        && cacheFilesPresent(finalManifest, cached, &ioError)) {
        result.success = true;
        result.cache_hit = true;
        result.cache_directory = finalDirectory;
        result.manifest_path = finalManifest;
        result.manifest = std::move(cached);
        return result;
    }

    const QString temporaryName = QStringLiteral(".%1.tmp-%2")
        .arg(cacheKeyValue,
             QUuid::createUuid().toString(QUuid::WithoutBraces));
    const QString temporaryDirectory = root.absoluteFilePath(temporaryName);
    QDir temporary(temporaryDirectory);
    if (!QDir().mkpath(temporaryDirectory)) {
        failBuild(result, QStringLiteral("无法创建分块缓存临时目录"));
        return result;
    }
    auto cleanupTemporary = [&temporary]() {
        if (temporary.exists())
            temporary.removeRecursively();
    };

    Manifest manifest;
    manifest.format_major = kTileMapFormatMajor;
    manifest.format_minor = kTileMapFormatMinor;
    manifest.anchor = archive.anchor;
    manifest.grid_origin_x = 0.0;
    manifest.grid_origin_y = 0.0;
    manifest.tile_size_m = options.tile_size_m;
    manifest.lods = options.lods;
    manifest.sources = sources;
    manifest.berths.reserve(static_cast<int>(archive.berths.size()));
    for (const usv::SlamMapBerth &berth : archive.berths)
        manifest.berths.append(berth);
    manifest.trajectory_relative_path = QStringLiteral("trajectory.bin");

    QVector<TrajectoryPose> trajectory;
    trajectory.reserve(static_cast<int>(archive.keyframes.size()));
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        if (!keyframe.pose.matrix().allFinite()) {
            cleanupTemporary();
            failBuild(result, QStringLiteral("关键帧位姿包含非法数值"));
            return result;
        }
        trajectory.append({keyframe.keyframe_id, keyframe.timestamp,
                           keyframe.pose});
        result.stats.source_points += keyframe.cloud.size();
    }
    if (result.stats.source_points == 0) {
        cleanupTemporary();
        failBuild(result, QStringLiteral("地图关键帧不包含点"));
        return result;
    }
    if (!saveTrajectory(
            temporary.absoluteFilePath(manifest.trajectory_relative_path),
            trajectory, &ioError)) {
        cleanupTemporary();
        failBuild(result, ioError);
        return result;
    }

    bool hasGlobalBounds = false;
    result.stats.lod_points.reserve(options.lods.size());
    for (const LodLevel &lod : options.lods) {
        std::map<TileId, Accumulator> accumulators;
        for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
            for (const M_PointXYZI &localPoint : keyframe.cloud) {
                const Eigen::Vector3d world = keyframe.pose
                    * Eigen::Vector3d(localPoint.x, localPoint.y, localPoint.z);
                if (!finitePoint(world))
                    continue;
                if (lod.level == options.lods.first().level) {
                    addToBounds(manifest.bounds, hasGlobalBounds,
                                world.x(), world.y(), world.z());
                }
                const TileId id{tileIndex(world.x(), manifest.grid_origin_x,
                                          options.tile_size_m),
                                tileIndex(world.y(), manifest.grid_origin_y,
                                          options.tile_size_m),
                                lod.level};
                Accumulator &accumulator = accumulators[id];
                const VoxelKey voxel{
                    static_cast<qint64>(std::floor(
                        (world.x() - manifest.grid_origin_x)
                        / lod.voxel_size_m)),
                    static_cast<qint64>(std::floor(
                        (world.y() - manifest.grid_origin_y)
                        / lod.voxel_size_m)),
                    static_cast<qint64>(std::floor(world.z()
                                                  / lod.voxel_size_m))};
                if (!accumulator.occupied.insert(voxel).second)
                    continue;
                M_PointXYZI output{};
                output.x = static_cast<float>(world.x());
                output.y = static_cast<float>(world.y());
                output.z = static_cast<float>(world.z());
                output.intensity = options.display_intensity;
                accumulator.points.append(output);
                addToBounds(accumulator.bounds, accumulator.has_bounds,
                            world.x(), world.y(), world.z());
            }
        }

        quint64 lodPointCount = 0;
        for (auto &entry : accumulators) {
            const TileId id = entry.first;
            Accumulator &accumulator = entry.second;
            if (accumulator.points.isEmpty() || !accumulator.has_bounds)
                continue;
            TileData tile;
            tile.meta.id = id;
            tile.meta.relative_path = tileRelativePath(id);
            tile.meta.bounds = accumulator.bounds;
            tile.tile_origin_x = manifest.grid_origin_x
                + static_cast<double>(id.x) * options.tile_size_m;
            tile.tile_origin_y = manifest.grid_origin_y
                + static_cast<double>(id.y) * options.tile_size_m;
            tile.points = std::move(accumulator.points);
            QByteArray checksum;
            const QString tilePath = temporary.absoluteFilePath(
                tile.meta.relative_path);
            if (!saveTile(tilePath, tile, &checksum, &ioError)) {
                cleanupTemporary();
                failBuild(result, ioError);
                return result;
            }
            tile.meta.point_count = static_cast<quint64>(tile.points.size());
            tile.meta.byte_size = static_cast<quint64>(QFileInfo(tilePath).size());
            tile.meta.checksum_sha256 = checksum;
            manifest.tiles.append(tile.meta);
            lodPointCount += tile.meta.point_count;
            ++result.stats.tile_files;
        }
        result.stats.lod_points.append(lodPointCount);
    }

    if (!hasGlobalBounds || manifest.tiles.isEmpty()) {
        cleanupTemporary();
        failBuild(result, QStringLiteral("地图未生成有效 Tile"));
        return result;
    }
    const QString temporaryManifest = temporary.absoluteFilePath(
        QStringLiteral("manifest.json"));
    if (!saveManifest(temporaryManifest, manifest, &ioError)) {
        cleanupTemporary();
        failBuild(result, ioError);
        return result;
    }

    QDir existing(finalDirectory);
    if (existing.exists() && !existing.removeRecursively()) {
        cleanupTemporary();
        failBuild(result, QStringLiteral("无法替换失效的分块地图缓存"));
        return result;
    }
    if (!root.rename(temporaryName, cacheKeyValue)) {
        cleanupTemporary();
        failBuild(result, QStringLiteral("无法发布分块地图缓存"));
        return result;
    }

    result.success = true;
    result.cache_directory = finalDirectory;
    result.manifest_path = finalManifest;
    result.manifest = std::move(manifest);
    return result;
}

}  // namespace slam_tile

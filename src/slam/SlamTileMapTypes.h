#ifndef SLAMTILEMAPTYPES_H
#define SLAMTILEMAPTYPES_H

#include "common/pointxyz.h"
#include "common/slam_types.h"

#include <QSet>
#include <QString>
#include <QVector>
#include <QMetaType>
#include <QtGlobal>

#include <cmath>
#include <limits>
#include <memory>

namespace slam_tile {

struct TileId {
    int x = 0;
    int y = 0;
    int lod = 0;

    bool operator==(const TileId &other) const noexcept
    {
        return x == other.x && y == other.y && lod == other.lod;
    }

    bool operator!=(const TileId &other) const noexcept
    {
        return !(*this == other);
    }

    bool operator<(const TileId &other) const noexcept
    {
        if (lod != other.lod)
            return lod < other.lod;
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
};

inline uint qHash(const TileId &id, uint seed = 0) noexcept
{
    seed = ::qHash(id.x, seed);
    seed = ::qHash(id.y, seed ^ 0x9e3779b9U);
    return ::qHash(id.lod, seed ^ 0x85ebca6bU);
}

struct Bounds2d {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;

    bool valid() const noexcept
    {
        return std::isfinite(min_x) && std::isfinite(min_y)
            && std::isfinite(max_x) && std::isfinite(max_y)
            && min_x < max_x && min_y < max_y;
    }
};

struct Bounds3d {
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;

    bool valid() const noexcept
    {
        return std::isfinite(min_x) && std::isfinite(min_y)
            && std::isfinite(min_z) && std::isfinite(max_x)
            && std::isfinite(max_y) && std::isfinite(max_z)
            && min_x <= max_x && min_y <= max_y && min_z <= max_z;
    }
};

struct SourceFingerprint {
    QString canonical_path;
    quint64 size_bytes = 0;
    qint64 modified_msecs_utc = 0;

    bool operator==(const SourceFingerprint &other) const noexcept
    {
        return canonical_path == other.canonical_path
            && size_bytes == other.size_bytes
            && modified_msecs_utc == other.modified_msecs_utc;
    }

    bool operator!=(const SourceFingerprint &other) const noexcept
    {
        return !(*this == other);
    }
};

struct LodLevel {
    int level = 0;
    double voxel_size_m = 0.0;
};

struct TileMeta {
    TileId id;
    QString relative_path;
    Bounds3d bounds;
    quint64 point_count = 0;
    quint64 byte_size = 0;
    QByteArray checksum_sha256;
};

struct Manifest {
    int format_major = 1;
    int format_minor = 0;
    usv::SlamGeoAnchor anchor;
    Bounds3d bounds;
    double grid_origin_x = 0.0;
    double grid_origin_y = 0.0;
    double tile_size_m = 50.0;
    QVector<LodLevel> lods;
    QVector<SourceFingerprint> sources;
    QVector<TileMeta> tiles;
    // 与 Tile 点云相同 ENU 坐标系下的持久化泊位语义图层。
    QVector<usv::SlamMapBerth> berths;
    QString trajectory_relative_path;
};

struct TileData {
    TileMeta meta;
    double tile_origin_x = 0.0;
    double tile_origin_y = 0.0;
    QVector<M_PointXYZI> points;

    quint64 memoryBytes() const noexcept
    {
        return static_cast<quint64>(points.size())
            * static_cast<quint64>(sizeof(M_PointXYZI));
    }
};

using TileDataPtr = std::shared_ptr<const TileData>;

struct TrajectoryPose {
    quint64 keyframe_id = 0;
    double timestamp = 0.0;
    Eigen::Isometry3d pose_lidar_enu = Eigen::Isometry3d::Identity();
};

inline int tileIndex(double coordinate, double origin, double tileSize)
{
    if (!std::isfinite(coordinate) || !std::isfinite(origin)
        || !std::isfinite(tileSize) || tileSize <= 0.0) {
        return 0;
    }
    return static_cast<int>(std::floor((coordinate - origin) / tileSize));
}

inline QSet<TileId> tilesForBounds(const Bounds2d &bounds,
                                   double originX,
                                   double originY,
                                   double tileSize,
                                   int lod)
{
    QSet<TileId> result;
    if (!bounds.valid() || !std::isfinite(originX)
        || !std::isfinite(originY) || !std::isfinite(tileSize)
        || tileSize <= 0.0 || lod < 0) {
        return result;
    }

    const double maxX = std::nextafter(bounds.max_x, bounds.min_x);
    const double maxY = std::nextafter(bounds.max_y, bounds.min_y);
    const int minTileX = tileIndex(bounds.min_x, originX, tileSize);
    const int maxTileX = tileIndex(maxX, originX, tileSize);
    const int minTileY = tileIndex(bounds.min_y, originY, tileSize);
    const int maxTileY = tileIndex(maxY, originY, tileSize);
    for (int x = minTileX; x <= maxTileX; ++x) {
        for (int y = minTileY; y <= maxTileY; ++y)
            result.insert({x, y, lod});
    }
    return result;
}

inline QSet<TileId> expandTiles(const QSet<TileId> &tiles, int rings)
{
    if (tiles.isEmpty() || rings <= 0)
        return tiles;

    QSet<TileId> result = tiles;
    for (const TileId &id : tiles) {
        for (int dx = -rings; dx <= rings; ++dx) {
            for (int dy = -rings; dy <= rings; ++dy)
                result.insert({id.x + dx, id.y + dy, id.lod});
        }
    }
    return result;
}

inline bool requestResultIsCurrent(quint64 resultMapGeneration,
                                   quint64 resultRequestGeneration,
                                   quint64 currentMapGeneration,
                                   quint64 currentRequestGeneration,
                                   bool stillRequired) noexcept
{
    return stillRequired
        && resultMapGeneration == currentMapGeneration
        && resultRequestGeneration == currentRequestGeneration;
}

inline Bounds2d viewportBounds(double centerX, double centerY,
                               double halfHeight, double aspectRatio)
{
    if (!std::isfinite(centerX) || !std::isfinite(centerY)
        || !std::isfinite(halfHeight) || halfHeight <= 0.0
        || !std::isfinite(aspectRatio) || aspectRatio <= 0.0) {
        return {};
    }
    const double halfWidth = halfHeight * aspectRatio;
    return {centerX - halfWidth, centerY - halfHeight,
            centerX + halfWidth, centerY + halfHeight};
}

inline int selectLodForView(const QVector<LodLevel> &lods,
                            double halfHeight, int viewportHeightPixels)
{
    if (lods.isEmpty())
        return 0;

    const double metresPerPixel = (std::isfinite(halfHeight)
                                   && halfHeight > 0.0
                                   && viewportHeightPixels > 0)
        ? (2.0 * halfHeight / static_cast<double>(viewportHeightPixels))
        : 0.0;
    const double targetVoxel = std::max(0.02, 2.0 * metresPerPixel);

    const LodLevel *finest = &lods.first();
    const LodLevel *selected = nullptr;
    for (const LodLevel &lod : lods) {
        if (lod.voxel_size_m < finest->voxel_size_m)
            finest = &lod;
        if (lod.voxel_size_m <= targetVoxel
            && (!selected
                || lod.voxel_size_m > selected->voxel_size_m)) {
            selected = &lod;
        }
    }
    return selected ? selected->level : finest->level;
}

}  // namespace slam_tile

Q_DECLARE_METATYPE(slam_tile::TileId)
Q_DECLARE_METATYPE(slam_tile::TileDataPtr)
Q_DECLARE_METATYPE(slam_tile::Bounds2d)

#endif  // SLAMTILEMAPTYPES_H

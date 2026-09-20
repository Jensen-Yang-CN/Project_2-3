#include "HistoricalMapExportSource.h"

#include "SlamMapGeoUtils.h"
#include "SlamTileMapIO.h"

#include <QDir>
#include <QFileInfo>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace history_map_export {

namespace {

bool fail(QString *error, const QString &message)
{
    if (error)
        *error = message;
    return false;
}

bool finitePoint(const M_PointXYZI &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

} // namespace

bool HistoricalMapExportSource::openManifest(const QString &manifestPath,
                                             QString *error)
{
    slam_tile::Manifest candidate;
    if (!slam_tile::loadManifest(manifestPath, candidate, error))
        return false;
    if (!slam_map_geo::validAnchor(candidate.anchor))
        return fail(error, QStringLiteral("历史分块地图缺少有效 ENU 地理锚点"));
    if (candidate.lods.isEmpty() || candidate.tiles.isEmpty())
        return fail(error, QStringLiteral("历史分块地图没有可投递的 LOD/Tile"));

    const slam_tile::LodLevel *finest = &candidate.lods.first();
    for (const slam_tile::LodLevel &lod : candidate.lods) {
        if (lod.voxel_size_m < finest->voxel_size_m)
            finest = &lod;
    }
    const int finestLod = finest->level;

    QVector<slam_tile::TileMeta> finestTiles;
    for (const slam_tile::TileMeta &meta : candidate.tiles) {
        if (meta.id.lod == finestLod)
            finestTiles.append(meta);
    }
    if (finestTiles.isEmpty())
        return fail(error, QStringLiteral("历史分块地图最细 LOD 没有 Tile"));
    std::sort(finestTiles.begin(), finestTiles.end(),
              [](const slam_tile::TileMeta &a,
                 const slam_tile::TileMeta &b) {
                  return a.id < b.id;
              });

    close();
    manifest_path_ = QFileInfo(manifestPath).absoluteFilePath();
    manifest_dir_ = QFileInfo(manifest_path_).absolutePath();
    manifest_ = std::move(candidate);
    finest_lod_ = finestLod;
    finest_tiles_ = std::move(finestTiles);
    full_map_cursor_ = 0;
    open_ = true;
    return true;
}

void HistoricalMapExportSource::close()
{
    open_ = false;
    manifest_path_.clear();
    manifest_dir_.clear();
    manifest_ = {};
    finest_lod_ = -1;
    finest_tiles_.clear();
    full_map_cursor_ = 0;
    tile_cache_.clear();
    cache_lru_.clear();
}

bool HistoricalMapExportSource::hasNextFullMapTile() const noexcept
{
    return open_ && full_map_cursor_ >= 0
        && full_map_cursor_ < finest_tiles_.size();
}

bool HistoricalMapExportSource::takeNextFullMapTile(
    std::vector<M_PointXYZI> &points, QString *error)
{
    points.clear();
    if (!hasNextFullMapTile())
        return fail(error, QStringLiteral("历史整图 Tile 已读取完毕"));

    const slam_tile::TileMeta meta = finest_tiles_.at(full_map_cursor_++);
    slam_tile::TileDataPtr tile;
    if (!loadTile(meta, tile, error))
        return false;
    points.reserve(static_cast<std::size_t>(tile->points.size()));
    for (const M_PointXYZI &p : tile->points) {
        if (finitePoint(p))
            points.push_back(p);
    }
    return true;
}

bool HistoricalMapExportSource::makeOverlapVirtualKeyframe(
    const usv::SlamKeyframe &liveKeyframe,
    const usv::SlamGeoAnchor &liveAnchor,
    double paddingM,
    usv::SlamKeyframe &virtualKeyframe,
    OverlapStats *stats,
    QString *error)
{
    virtualKeyframe = {};
    if (stats)
        *stats = {};
    if (!open_)
        return fail(error, QStringLiteral("历史分块地图尚未加载"));
    if (!slam_map_geo::validAnchor(liveAnchor))
        return fail(error, QStringLiteral("实时 SLAM 地理锚点无效"));
    if (liveKeyframe.cloud.empty())
        return fail(error, QStringLiteral("实时关键帧点云为空"));

    Eigen::Vector3d liveOriginInHistory;
    if (!slam_map_geo::anchorOffsetEnu(
            manifest_.anchor, liveAnchor, liveOriginInHistory, error)) {
        return false;
    }

    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const M_PointXYZI &p : liveKeyframe.cloud) {
        if (!finitePoint(p))
            continue;
        const Eigen::Vector3d liveWorld = liveKeyframe.pose
            * Eigen::Vector3d(p.x, p.y, p.z);
        const Eigen::Vector3d historyWorld = liveOriginInHistory + liveWorld;
        minX = std::min(minX, historyWorld.x());
        minY = std::min(minY, historyWorld.y());
        maxX = std::max(maxX, historyWorld.x());
        maxY = std::max(maxY, historyWorld.y());
    }
    if (!std::isfinite(minX) || !std::isfinite(minY)
        || !std::isfinite(maxX) || !std::isfinite(maxY)) {
        return fail(error, QStringLiteral("实时关键帧没有有效点"));
    }

    const double padding = std::max(0.0, paddingM);
    const double minimumSpan = 0.01;
    slam_tile::Bounds2d bounds{
        minX - padding, minY - padding,
        maxX + padding, maxY + padding};
    if (bounds.max_x - bounds.min_x < minimumSpan) {
        bounds.min_x -= minimumSpan * 0.5;
        bounds.max_x += minimumSpan * 0.5;
    }
    if (bounds.max_y - bounds.min_y < minimumSpan) {
        bounds.min_y -= minimumSpan * 0.5;
        bounds.max_y += minimumSpan * 0.5;
    }

    const QSet<slam_tile::TileId> wanted = slam_tile::tilesForBounds(
        bounds, manifest_.grid_origin_x, manifest_.grid_origin_y,
        manifest_.tile_size_m, finest_lod_);

    virtualKeyframe.keyframe_id = liveKeyframe.keyframe_id;
    virtualKeyframe.timestamp = liveKeyframe.timestamp;
    virtualKeyframe.pose = Eigen::Isometry3d::Identity();
    if (stats) {
        stats->candidate_tiles = wanted.size();
        stats->bounds = bounds;
    }

    for (const slam_tile::TileId &id : wanted) {
        const slam_tile::TileMeta *meta = findFinestTile(id);
        if (!meta)
            continue;
        slam_tile::TileDataPtr tile;
        QString tileError;
        if (!loadTile(*meta, tile, &tileError)) {
            if (error && error->isEmpty())
                *error = tileError;
            continue;
        }
        if (stats)
            ++stats->loaded_tiles;
        for (const M_PointXYZI &p : tile->points) {
            if (!finitePoint(p))
                continue;
            if (p.x < bounds.min_x || p.x > bounds.max_x
                || p.y < bounds.min_y || p.y > bounds.max_y) {
                continue;
            }
            virtualKeyframe.cloud.push_back(p);
        }
    }
    if (stats)
        stats->selected_points = virtualKeyframe.cloud.size();
    return true;
}

bool HistoricalMapExportSource::loadTile(const slam_tile::TileMeta &meta,
                                         slam_tile::TileDataPtr &tile,
                                         QString *error)
{
    const auto found = tile_cache_.constFind(meta.id);
    if (found != tile_cache_.constEnd()) {
        tile = found.value();
        touchCache(meta.id);
        return true;
    }

    slam_tile::TileData loaded;
    const QString path = QDir(manifest_dir_).absoluteFilePath(meta.relative_path);
    if (!slam_tile::loadTile(path, meta.checksum_sha256, loaded, error))
        return false;
    if (loaded.meta.id != meta.id)
        return fail(error, QStringLiteral("历史 Tile 标识与 manifest 不一致"));

    tile = std::make_shared<const slam_tile::TileData>(std::move(loaded));
    tile_cache_.insert(meta.id, tile);
    touchCache(meta.id);
    while (cache_lru_.size() > kMaxCachedTiles) {
        const slam_tile::TileId evicted = cache_lru_.takeFirst();
        tile_cache_.remove(evicted);
    }
    return true;
}

const slam_tile::TileMeta *HistoricalMapExportSource::findFinestTile(
    const slam_tile::TileId &id) const
{
    for (const slam_tile::TileMeta &meta : finest_tiles_) {
        if (meta.id == id)
            return &meta;
    }
    return nullptr;
}

void HistoricalMapExportSource::touchCache(const slam_tile::TileId &id)
{
    cache_lru_.removeAll(id);
    cache_lru_.append(id);
}

} // namespace history_map_export

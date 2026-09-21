#include "HistoricalMapExportSource.h"

#include "SlamMapGeoUtils.h"
#include "SlamTileMapIO.h"

#include <QDir>
#include <QFileInfo>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

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

constexpr double kBerthRoiPaddingM = 2.0;

bool boundsIntersect(const slam_tile::Bounds2d &lhs,
                     const slam_tile::Bounds2d &rhs)
{
    return lhs.valid() && rhs.valid()
        && lhs.min_x <= rhs.max_x && lhs.max_x >= rhs.min_x
        && lhs.min_y <= rhs.max_y && lhs.max_y >= rhs.min_y;
}

slam_tile::Bounds2d tileBounds(const slam_tile::Manifest &manifest,
                               const slam_tile::TileId &id)
{
    const double minX = manifest.grid_origin_x
        + static_cast<double>(id.x) * manifest.tile_size_m;
    const double minY = manifest.grid_origin_y
        + static_cast<double>(id.y) * manifest.tile_size_m;
    return {minX, minY, minX + manifest.tile_size_m,
            minY + manifest.tile_size_m};
}

bool berthRoi(const usv::SlamMapBerth &berth,
              slam_tile::Bounds2d &bounds)
{
    if (!std::isfinite(berth.x) || !std::isfinite(berth.y)
        || !std::isfinite(berth.width) || !std::isfinite(berth.length)
        || berth.width <= 0.0 || berth.length <= 0.0) {
        return false;
    }

    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double halfLength = 0.5 * std::abs(berth.length);
    const double halfWidth = 0.5 * std::abs(berth.width);
    const double angle = berth.angle_deg * kDegToRad;
    const double c = std::abs(std::cos(angle));
    const double s = std::abs(std::sin(angle));
    const double halfExtentX = c * halfLength + s * halfWidth
        + kBerthRoiPaddingM;
    const double halfExtentY = s * halfLength + c * halfWidth
        + kBerthRoiPaddingM;
    bounds = {berth.x - halfExtentX, berth.y - halfExtentY,
              berth.x + halfExtentX, berth.y + halfExtentY};
    return bounds.valid();
}

bool overlapsAnyBerth(const slam_tile::Manifest &manifest,
                       const slam_tile::TileId &id)
{
    const slam_tile::Bounds2d bounds = tileBounds(manifest, id);
    for (const usv::SlamMapBerth &berth : manifest.berths) {
        slam_tile::Bounds2d roi;
        if (berthRoi(berth, roi) && boundsIntersect(bounds, roi))
            return true;
    }
    return false;
}

const slam_tile::TileMeta *findTileForCoordinate(
    const std::map<std::pair<int, int>, slam_tile::TileMeta> &tiles,
    int x, int y)
{
    const auto found = tiles.find({x, y});
    return found == tiles.end() ? nullptr : &found->second;
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

    QVector<slam_tile::LodLevel> lods = candidate.lods;
    std::sort(lods.begin(), lods.end(),
              [](const slam_tile::LodLevel &a,
                 const slam_tile::LodLevel &b) {
                  if (a.voxel_size_m != b.voxel_size_m)
                      return a.voxel_size_m < b.voxel_size_m;
                  return a.level < b.level;
              });
    const slam_tile::LodLevel berthLod = lods.first();
    const slam_tile::LodLevel backgroundLod = lods.size() > 1
        ? lods.at(1)
        : berthLod;

    std::map<std::pair<int, int>, slam_tile::TileMeta> berthTiles;
    std::map<std::pair<int, int>, slam_tile::TileMeta> backgroundTiles;
    for (const slam_tile::TileMeta &meta : candidate.tiles) {
        if (meta.id.lod == berthLod.level)
            berthTiles[{meta.id.x, meta.id.y}] = meta;
        if (meta.id.lod == backgroundLod.level)
            backgroundTiles[{meta.id.x, meta.id.y}] = meta;
    }

    std::set<std::pair<int, int>> coordinates;
    for (const auto &entry : berthTiles)
        coordinates.insert(entry.first);
    for (const auto &entry : backgroundTiles)
        coordinates.insert(entry.first);

    QVector<slam_tile::TileMeta> selectedTiles;
    int berthTileCount = 0;
    int backgroundTileCount = 0;
    for (const auto &coordinate : coordinates) {
        const bool berthRegion = overlapsAnyBerth(
            candidate,
            {coordinate.first, coordinate.second, berthLod.level});
        const slam_tile::TileMeta *selected = berthRegion
            ? findTileForCoordinate(berthTiles,
                                    coordinate.first, coordinate.second)
            : findTileForCoordinate(backgroundTiles,
                                    coordinate.first, coordinate.second);
        if (!selected) {
            selected = berthRegion
                ? findTileForCoordinate(backgroundTiles,
                                        coordinate.first, coordinate.second)
                : findTileForCoordinate(berthTiles,
                                        coordinate.first, coordinate.second);
        }
        if (!selected)
            continue;
        selectedTiles.append(*selected);
        if (selected->id.lod == berthLod.level)
            ++berthTileCount;
        else if (selected->id.lod == backgroundLod.level)
            ++backgroundTileCount;
    }
    if (selectedTiles.isEmpty())
        return fail(error, QStringLiteral("历史分块地图没有可投递的 Tile"));
    std::sort(selectedTiles.begin(), selectedTiles.end(),
              [](const slam_tile::TileMeta &a,
                 const slam_tile::TileMeta &b) {
                  if (a.id.x != b.id.x)
                      return a.id.x < b.id.x;
                  if (a.id.y != b.id.y)
                      return a.id.y < b.id.y;
                  return a.id.lod < b.id.lod;
              });

    close();
    manifest_path_ = QFileInfo(manifestPath).absoluteFilePath();
    manifest_dir_ = QFileInfo(manifest_path_).absolutePath();
    manifest_ = std::move(candidate);
    berth_lod_ = berthLod.level;
    background_lod_ = backgroundLod.level;
    berth_lod_voxel_size_m_ = berthLod.voxel_size_m;
    background_lod_voxel_size_m_ = backgroundLod.voxel_size_m;
    berth_tile_count_ = berthTileCount;
    background_tile_count_ = backgroundTileCount;
    selected_tiles_ = std::move(selectedTiles);
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
    berth_lod_ = -1;
    background_lod_ = -1;
    berth_lod_voxel_size_m_ = 0.0;
    background_lod_voxel_size_m_ = 0.0;
    berth_tile_count_ = 0;
    background_tile_count_ = 0;
    selected_tiles_.clear();
    full_map_cursor_ = 0;
    tile_cache_.clear();
    cache_lru_.clear();
}

bool HistoricalMapExportSource::hasNextFullMapTile() const noexcept
{
    return open_ && full_map_cursor_ >= 0
        && full_map_cursor_ < selected_tiles_.size();
}

bool HistoricalMapExportSource::takeNextFullMapTile(
    std::vector<M_PointXYZI> &points, QString *error)
{
    points.clear();
    if (!hasNextFullMapTile())
        return fail(error, QStringLiteral("历史整图 Tile 已读取完毕"));

    const slam_tile::TileMeta meta = selected_tiles_.at(full_map_cursor_++);
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

    virtualKeyframe.keyframe_id = liveKeyframe.keyframe_id;
    virtualKeyframe.timestamp = liveKeyframe.timestamp;
    virtualKeyframe.pose = Eigen::Isometry3d::Identity();
    if (stats) {
        stats->bounds = bounds;
    }

    for (const slam_tile::TileMeta &meta : selected_tiles_) {
        if (!boundsIntersect(tileBounds(manifest_, meta.id), bounds))
            continue;
        if (stats)
            ++stats->candidate_tiles;
        slam_tile::TileDataPtr tile;
        QString tileError;
        if (!loadTile(meta, tile, &tileError)) {
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

void HistoricalMapExportSource::touchCache(const slam_tile::TileId &id)
{
    cache_lru_.removeAll(id);
    cache_lru_.append(id);
}

} // namespace history_map_export

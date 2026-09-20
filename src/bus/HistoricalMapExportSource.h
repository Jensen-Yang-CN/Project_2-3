#pragma once

#include "common/slam_types.h"
#include "SlamTileMapTypes.h"

#include <QHash>
#include <QList>
#include <QString>
#include <QVector>

#include <memory>
#include <vector>

namespace history_map_export {

struct OverlapStats {
    int candidate_tiles = 0;
    int loaded_tiles = 0;
    std::size_t selected_points = 0;
    slam_tile::Bounds2d bounds;
};

/**
 * 只读历史分块地图数据源。
 *
 * Tile 中的点已经位于 manifest.anchor 对应的 ENU 世界坐标系；本类只为
 * UDP 投递提供数据，不持有、也不修改实时 SLAM 节点。
 */
class HistoricalMapExportSource {
public:
    bool openManifest(const QString &manifestPath, QString *error = nullptr);
    void close();

    bool isOpen() const noexcept { return open_; }
    const usv::SlamGeoAnchor &anchor() const noexcept { return manifest_.anchor; }
    /**
     * 返回 manifest 中持久化的泊位语义图层。
     *
     * 泊位记录与 Tile 点云共用同一 ENU 锚点；导出端可以只读访问这些
     * 记录并按现有 0xFB 协议投递，不会修改历史地图或实时 SLAM 状态。
     */
    const QVector<usv::SlamMapBerth> &berths() const noexcept
    {
        return manifest_.berths;
    }
    int finestLod() const noexcept { return finest_lod_; }
    int fullMapTileCount() const noexcept { return finest_tiles_.size(); }

    void resetFullMapCursor() noexcept { full_map_cursor_ = 0; }
    bool hasNextFullMapTile() const noexcept;
    bool takeNextFullMapTile(std::vector<M_PointXYZI> &points,
                             QString *error = nullptr);

    bool makeOverlapVirtualKeyframe(
        const usv::SlamKeyframe &liveKeyframe,
        const usv::SlamGeoAnchor &liveAnchor,
        double paddingM,
        usv::SlamKeyframe &virtualKeyframe,
        OverlapStats *stats = nullptr,
        QString *error = nullptr);

private:
    bool loadTile(const slam_tile::TileMeta &meta,
                  slam_tile::TileDataPtr &tile,
                  QString *error);
    const slam_tile::TileMeta *findFinestTile(
        const slam_tile::TileId &id) const;
    void touchCache(const slam_tile::TileId &id);

    bool open_ = false;
    QString manifest_path_;
    QString manifest_dir_;
    slam_tile::Manifest manifest_;
    int finest_lod_ = -1;
    QVector<slam_tile::TileMeta> finest_tiles_;
    int full_map_cursor_ = 0;

    QHash<slam_tile::TileId, slam_tile::TileDataPtr> tile_cache_;
    QList<slam_tile::TileId> cache_lru_;
    static constexpr int kMaxCachedTiles = 12;
};

} // namespace history_map_export

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
    int finestLod() const noexcept { return berth_lod_; }
    int backgroundLod() const noexcept { return background_lod_; }
    double berthVoxelSize() const noexcept { return berth_lod_voxel_size_m_; }
    double backgroundVoxelSize() const noexcept
    {
        return background_lod_voxel_size_m_;
    }
    int berthTileCount() const noexcept { return berth_tile_count_; }
    int backgroundTileCount() const noexcept
    {
        return background_tile_count_;
    }
    int fullMapTileCount() const noexcept { return selected_tiles_.size(); }

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
    void touchCache(const slam_tile::TileId &id);

    bool open_ = false;
    QString manifest_path_;
    QString manifest_dir_;
    slam_tile::Manifest manifest_;
    int berth_lod_ = -1;
    int background_lod_ = -1;
    double berth_lod_voxel_size_m_ = 0.0;
    double background_lod_voxel_size_m_ = 0.0;
    int berth_tile_count_ = 0;
    int background_tile_count_ = 0;
    QVector<slam_tile::TileMeta> selected_tiles_;
    int full_map_cursor_ = 0;

    QHash<slam_tile::TileId, slam_tile::TileDataPtr> tile_cache_;
    QList<slam_tile::TileId> cache_lru_;
    static constexpr int kMaxCachedTiles = 12;
};

} // namespace history_map_export

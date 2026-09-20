#ifndef SLAMTILEMAPMANAGER_H
#define SLAMTILEMAPMANAGER_H

#include "SlamTileMapTypes.h"

#include <QFutureWatcher>
#include <QHash>
#include <QObject>
#include <QRunnable>
#include <QSet>
#include <QThreadPool>
#include <QTimer>

namespace slam_tile {

class TileMemoryCache {
public:
    explicit TileMemoryCache(quint64 budgetBytes = 512ULL * 1024ULL * 1024ULL);

    QSet<TileId> setBudgetBytes(quint64 budgetBytes);
    QSet<TileId> setPinned(const QSet<TileId> &pinned);
    QSet<TileId> insert(TileDataPtr tile);
    void touch(const TileId &id);
    QSet<TileId> removeOutside(const QSet<TileId> &retain);
    bool contains(const TileId &id) const;
    TileDataPtr value(const TileId &id) const;
    QVector<TileId> ids() const;
    quint64 memoryBytes() const noexcept;
    quint64 budgetBytes() const noexcept;
    void clear();

private:
    struct Entry {
        TileDataPtr data;
        quint64 last_use = 0;
    };

    QSet<TileId> trim();

    QHash<TileId, Entry> entries_;
    QSet<TileId> pinned_;
    quint64 budget_bytes_ = 0;
    quint64 memory_bytes_ = 0;
    quint64 use_counter_ = 0;
};

struct ManagerSettings {
    int prefetch_ring_tiles = 1;
    int unload_ring_tiles = 2;
    int view_update_delay_ms = 120;
    quint64 ram_budget_bytes = 512ULL * 1024ULL * 1024ULL;
    int max_concurrent_loads = 2;
};

struct ManagerStats {
    int active_tiles = 0;
    int prefetch_tiles = 0;
    int resident_tiles = 0;
    int inflight_tiles = 0;
    quint64 resident_bytes = 0;
    quint64 cache_hits = 0;
    quint64 cache_misses = 0;
    int lod = 0;
};

class SlamTileMapManager : public QObject {
    Q_OBJECT

public:
    explicit SlamTileMapManager(const ManagerSettings &settings = {},
                                QObject *parent = nullptr);
    ~SlamTileMapManager() override;

    bool openManifest(const QString &manifestPath, QString *error = nullptr);
    void closeMap();
    void updateView(const Bounds2d &viewBounds, int lod);
    void updateViewNow(const Bounds2d &viewBounds, int lod);

    bool isOpen() const noexcept;
    const Manifest &manifest() const noexcept;
    const QVector<TrajectoryPose> &trajectory() const noexcept;
    const ManagerStats &stats() const noexcept;
    quint64 mapGeneration() const noexcept;
    quint64 requestGeneration() const noexcept;

signals:
    void mapCleared();
    void tileAvailable(slam_tile::TileId id, slam_tile::TileDataPtr tile);
    void tileRemoved(slam_tile::TileId id);
    void statisticsChanged(const slam_tile::ManagerStats &stats);
    void loadWarning(const QString &message);

private:
    struct TileLoadResult {
        TileId id;
        TileDataPtr data;
        quint64 map_generation = 0;
        quint64 request_generation = 0;
        QString error;
    };

    void rebuildIndex();
    void scheduleTile(const TileMeta &meta, quint64 requestGeneration);
    void publishStats();
    QSet<TileId> existingTiles(const QSet<TileId> &candidates) const;

    ManagerSettings settings_;
    Manifest manifest_;
    QVector<TrajectoryPose> trajectory_;
    QString manifest_path_;
    QString cache_directory_;
    QHash<TileId, TileMeta> index_;
    TileMemoryCache cache_;
    QSet<TileId> active_;
    QSet<TileId> prefetch_;
    QSet<TileId> desired_;
    QSet<TileId> inflight_;
    QTimer update_timer_;
    Bounds2d pending_bounds_;
    int pending_lod_ = 0;
    bool has_pending_view_ = false;
    bool open_ = false;
    bool closing_ = false;
    quint64 map_generation_ = 0;
    quint64 request_generation_ = 0;
    ManagerStats stats_;
    QThreadPool loader_pool_;
};

}  // namespace slam_tile

Q_DECLARE_METATYPE(slam_tile::ManagerStats)

#endif  // SLAMTILEMAPMANAGER_H

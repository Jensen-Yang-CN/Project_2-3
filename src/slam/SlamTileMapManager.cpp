#include "SlamTileMapManager.h"

#include "SlamTileMapIO.h"

#include <QtConcurrent/QtConcurrentRun>

#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <limits>

namespace slam_tile {

TileMemoryCache::TileMemoryCache(quint64 budgetBytes)
    : budget_bytes_(budgetBytes)
{
}

QSet<TileId> TileMemoryCache::setBudgetBytes(quint64 budgetBytes)
{
    budget_bytes_ = budgetBytes;
    return trim();
}

QSet<TileId> TileMemoryCache::setPinned(const QSet<TileId> &pinned)
{
    pinned_ = pinned;
    return trim();
}

QSet<TileId> TileMemoryCache::insert(TileDataPtr tile)
{
    QSet<TileId> removed;
    if (!tile)
        return removed;
    const TileId id = tile->meta.id;
    auto existing = entries_.find(id);
    if (existing != entries_.end()) {
        memory_bytes_ -= existing->data ? existing->data->memoryBytes() : 0;
        entries_.erase(existing);
    }
    memory_bytes_ += tile->memoryBytes();
    entries_.insert(id, {std::move(tile), ++use_counter_});
    return trim();
}

void TileMemoryCache::touch(const TileId &id)
{
    auto it = entries_.find(id);
    if (it != entries_.end())
        it->last_use = ++use_counter_;
}

QSet<TileId> TileMemoryCache::removeOutside(const QSet<TileId> &retain)
{
    QSet<TileId> removed;
    for (auto it = entries_.begin(); it != entries_.end();) {
        if (retain.contains(it.key()) || pinned_.contains(it.key())) {
            ++it;
            continue;
        }
        memory_bytes_ -= it->data ? it->data->memoryBytes() : 0;
        removed.insert(it.key());
        it = entries_.erase(it);
    }
    return removed;
}

bool TileMemoryCache::contains(const TileId &id) const
{
    return entries_.contains(id);
}

TileDataPtr TileMemoryCache::value(const TileId &id) const
{
    const auto it = entries_.constFind(id);
    return it == entries_.constEnd() ? TileDataPtr{} : it->data;
}

QVector<TileId> TileMemoryCache::ids() const
{
    return entries_.keys().toVector();
}

quint64 TileMemoryCache::memoryBytes() const noexcept
{
    return memory_bytes_;
}

quint64 TileMemoryCache::budgetBytes() const noexcept
{
    return budget_bytes_;
}

void TileMemoryCache::clear()
{
    entries_.clear();
    pinned_.clear();
    memory_bytes_ = 0;
}

QSet<TileId> TileMemoryCache::trim()
{
    QSet<TileId> removed;
    while (memory_bytes_ > budget_bytes_) {
        auto victim = entries_.end();
        quint64 oldest = std::numeric_limits<quint64>::max();
        for (auto it = entries_.begin(); it != entries_.end(); ++it) {
            if (!pinned_.contains(it.key()) && it->last_use < oldest) {
                oldest = it->last_use;
                victim = it;
            }
        }
        if (victim == entries_.end())
            break;
        memory_bytes_ -= victim->data ? victim->data->memoryBytes() : 0;
        removed.insert(victim.key());
        entries_.erase(victim);
    }
    return removed;
}

SlamTileMapManager::SlamTileMapManager(const ManagerSettings &settings,
                                       QObject *parent)
    : QObject(parent)
    , settings_(settings)
    , cache_(settings.ram_budget_bytes)
{
    qRegisterMetaType<slam_tile::TileId>("slam_tile::TileId");
    qRegisterMetaType<slam_tile::TileDataPtr>("slam_tile::TileDataPtr");
    qRegisterMetaType<slam_tile::ManagerStats>("slam_tile::ManagerStats");
    loader_pool_.setMaxThreadCount(std::max(1, settings_.max_concurrent_loads));
    update_timer_.setSingleShot(true);
    update_timer_.setInterval(std::max(0, settings_.view_update_delay_ms));
    connect(&update_timer_, &QTimer::timeout, this, [this]() {
        if (has_pending_view_)
            updateViewNow(pending_bounds_, pending_lod_);
    });
}

SlamTileMapManager::~SlamTileMapManager()
{
    closing_ = true;
    closeMap();
    loader_pool_.clear();
    loader_pool_.waitForDone();
}

bool SlamTileMapManager::openManifest(const QString &manifestPath,
                                      QString *error)
{
    Manifest candidate;
    if (!loadManifest(manifestPath, candidate, error))
        return false;
    QVector<TrajectoryPose> trajectory;
    if (!candidate.trajectory_relative_path.isEmpty()) {
        const QString trajectoryPath = QFileInfo(manifestPath).absoluteDir()
            .absoluteFilePath(candidate.trajectory_relative_path);
        if (!loadTrajectory(trajectoryPath, trajectory, error))
            return false;
    }

    closeMap();
    closing_ = false;
    manifest_ = std::move(candidate);
    trajectory_ = std::move(trajectory);
    manifest_path_ = QFileInfo(manifestPath).absoluteFilePath();
    cache_directory_ = QFileInfo(manifestPath).absolutePath();
    open_ = true;
    rebuildIndex();
    publishStats();
    return true;
}

void SlamTileMapManager::closeMap()
{
    update_timer_.stop();
    has_pending_view_ = false;
    ++map_generation_;
    ++request_generation_;
    open_ = false;
    active_.clear();
    prefetch_.clear();
    desired_.clear();
    inflight_.clear();
    index_.clear();
    cache_.clear();
    manifest_ = Manifest{};
    trajectory_.clear();
    manifest_path_.clear();
    cache_directory_.clear();
    stats_ = ManagerStats{};
    emit mapCleared();
}

void SlamTileMapManager::updateView(const Bounds2d &viewBounds, int lod)
{
    if (!open_ || !viewBounds.valid())
        return;
    pending_bounds_ = viewBounds;
    pending_lod_ = lod;
    has_pending_view_ = true;
    if (settings_.view_update_delay_ms <= 0)
        updateViewNow(viewBounds, lod);
    else
        update_timer_.start();
}

void SlamTileMapManager::updateViewNow(const Bounds2d &viewBounds, int lod)
{
    if (!open_ || !viewBounds.valid())
        return;
    has_pending_view_ = false;
    update_timer_.stop();
    ++request_generation_;

    active_ = existingTiles(tilesForBounds(
        viewBounds, manifest_.grid_origin_x, manifest_.grid_origin_y,
        manifest_.tile_size_m, lod));
    const QSet<TileId> expanded = existingTiles(
        expandTiles(active_, std::max(0, settings_.prefetch_ring_tiles)));
    prefetch_ = expanded;
    for (const TileId &id : active_)
        prefetch_.remove(id);
    desired_ = active_;
    desired_.unite(prefetch_);

    const QSet<TileId> retain = existingTiles(
        expandTiles(active_, std::max(settings_.prefetch_ring_tiles,
                                      settings_.unload_ring_tiles)));
    const QSet<TileId> budgetRemoved = cache_.setPinned(active_);
    for (const TileId &id : budgetRemoved)
        emit tileRemoved(id);
    const QSet<TileId> removed = cache_.removeOutside(retain);
    for (const TileId &id : removed)
        emit tileRemoved(id);

    auto requestSet = [this](const QSet<TileId> &ids) {
        QVector<TileId> ordered = ids.values().toVector();
        std::sort(ordered.begin(), ordered.end());
        for (const TileId &id : ordered) {
            if (cache_.contains(id)) {
                ++stats_.cache_hits;
                cache_.touch(id);
                emit tileAvailable(id, cache_.value(id));
                continue;
            }
            if (!inflight_.contains(id)) {
                ++stats_.cache_misses;
                scheduleTile(index_.value(id), request_generation_);
            }
        }
    };
    requestSet(active_);
    requestSet(prefetch_);
    stats_.lod = lod;
    publishStats();
}

bool SlamTileMapManager::isOpen() const noexcept
{
    return open_;
}

const Manifest &SlamTileMapManager::manifest() const noexcept
{
    return manifest_;
}

const QVector<TrajectoryPose> &SlamTileMapManager::trajectory() const noexcept
{
    return trajectory_;
}

const ManagerStats &SlamTileMapManager::stats() const noexcept
{
    return stats_;
}

quint64 SlamTileMapManager::mapGeneration() const noexcept
{
    return map_generation_;
}

quint64 SlamTileMapManager::requestGeneration() const noexcept
{
    return request_generation_;
}

void SlamTileMapManager::rebuildIndex()
{
    index_.clear();
    for (const TileMeta &tile : manifest_.tiles)
        index_.insert(tile.id, tile);
}

void SlamTileMapManager::scheduleTile(const TileMeta &meta,
                                      quint64 requestGeneration)
{
    if (!open_ || meta.relative_path.isEmpty())
        return;
    const TileId id = meta.id;
    const quint64 mapGeneration = map_generation_;
    const QString path = QDir(cache_directory_).absoluteFilePath(
        meta.relative_path);
    const QByteArray checksum = meta.checksum_sha256;
    inflight_.insert(id);
    auto *watcher = new QFutureWatcher<TileLoadResult>(this);
    connect(watcher, &QFutureWatcher<TileLoadResult>::finished,
            this, [this, watcher, id]() {
        const TileLoadResult result = watcher->result();
        watcher->deleteLater();
        // 旧地图的同名 Tile 可能晚于新地图返回，不能误删新一代请求。
        if (result.map_generation == map_generation_)
            inflight_.remove(id);
        if (closing_ || !open_)
            return;
        const bool current = requestResultIsCurrent(
            result.map_generation, result.request_generation,
            map_generation_, request_generation_, desired_.contains(id));
        if (!current) {
            if (desired_.contains(id) && index_.contains(id)
                && !inflight_.contains(id) && !cache_.contains(id)) {
                scheduleTile(index_.value(id), request_generation_);
            }
            publishStats();
            return;
        }
        if (!result.error.isEmpty() || !result.data) {
            emit loadWarning(result.error.isEmpty()
                ? QStringLiteral("Tile 加载失败") : result.error);
            publishStats();
            return;
        }
        const QSet<TileId> evicted = cache_.insert(result.data);
        for (const TileId &removed : evicted)
            emit tileRemoved(removed);
        // 预取 Tile 可能因预算不足在插入时立即被 LRU 淘汰。
        if (cache_.contains(id))
            emit tileAvailable(id, result.data);
        publishStats();
    });
    watcher->setFuture(QtConcurrent::run(
        &loader_pool_, [path, checksum, id, mapGeneration,
                        requestGeneration]() {
            TileLoadResult result;
            result.id = id;
            result.map_generation = mapGeneration;
            result.request_generation = requestGeneration;
            TileData loaded;
            if (!loadTile(path, checksum, loaded, &result.error))
                return result;
            result.data = std::make_shared<const TileData>(std::move(loaded));
            return result;
        }));
}

void SlamTileMapManager::publishStats()
{
    stats_.active_tiles = active_.size();
    stats_.prefetch_tiles = prefetch_.size();
    stats_.resident_tiles = cache_.ids().size();
    stats_.inflight_tiles = inflight_.size();
    stats_.resident_bytes = cache_.memoryBytes();
    emit statisticsChanged(stats_);
}

QSet<TileId> SlamTileMapManager::existingTiles(
    const QSet<TileId> &candidates) const
{
    QSet<TileId> result;
    for (const TileId &id : candidates) {
        if (index_.contains(id))
            result.insert(id);
    }
    return result;
}

}  // namespace slam_tile

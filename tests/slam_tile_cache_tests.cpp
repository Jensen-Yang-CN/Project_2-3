#include "SlamTileMapTypes.h"
#include "SlamTileMapManager.h"
#include "SlamTileMapIO.h"

#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QSet>
#include <QTemporaryDir>
#include <QTimer>

#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void testTileIndexUsesHalfOpenIntervals(int &failures)
{
    check(slam_tile::tileIndex(-0.01, 0.0, 50.0) == -1,
          "a negative coordinate immediately below the origin belongs to tile -1",
          failures);
    check(slam_tile::tileIndex(0.0, 0.0, 50.0) == 0,
          "the grid origin belongs to tile 0", failures);
    check(slam_tile::tileIndex(49.999, 0.0, 50.0) == 0,
          "the upper open boundary remains in tile 0", failures);
    check(slam_tile::tileIndex(50.0, 0.0, 50.0) == 1,
          "the exact positive boundary belongs to the next tile", failures);
}

void testViewportAndPrefetchSets(int &failures)
{
    const slam_tile::Bounds2d view{-10.0, -5.0, 60.0, 40.0};
    const QSet<slam_tile::TileId> active =
        slam_tile::tilesForBounds(view, 0.0, 0.0, 50.0, 1);
    check(active.size() == 6,
          "the viewport must cover two x columns and three y columns",
          failures);
    check(active.contains({-1, -1, 1})
              && active.contains({1, 0, 1}),
          "the active set must include the negative and positive edge tiles",
          failures);

    const QSet<slam_tile::TileId> expanded =
        slam_tile::expandTiles(active, 1);
    check(expanded.size() == 20,
          "one prefetch ring around a 3 by 2 tile rectangle should contain 20 tiles",
          failures);
    for (const slam_tile::TileId &id : active) {
        check(expanded.contains(id),
              "the expanded set must retain every active tile", failures);
    }
}

void testRequestGenerationGuard(int &failures)
{
    check(slam_tile::requestResultIsCurrent(4, 8, 4, 8, true),
          "a current still-required tile result should be accepted", failures);
    check(!slam_tile::requestResultIsCurrent(3, 8, 4, 8, true),
          "a result from an old map must be rejected", failures);
    check(!slam_tile::requestResultIsCurrent(4, 7, 4, 8, true),
          "a result from an old viewport request must be rejected", failures);
    check(!slam_tile::requestResultIsCurrent(4, 8, 4, 8, false),
          "a tile no longer requested must be rejected", failures);
}

void testViewportBoundsAndLodSelection(int &failures)
{
    const slam_tile::Bounds2d bounds = slam_tile::viewportBounds(
        10.0, -5.0, 60.0, 2.0);
    check(bounds.min_x == -110.0 && bounds.max_x == 130.0
              && bounds.min_y == -65.0 && bounds.max_y == 55.0,
          "BEV pan, range and aspect ratio must produce ENU view bounds",
          failures);

    const QVector<slam_tile::LodLevel> lods{
        {0, 1.0}, {1, 0.3}, {2, 0.1}};
    check(slam_tile::selectLodForView(lods, 10.0, 800) == 2,
          "a close view should select the finest available LOD", failures);
    check(slam_tile::selectLodForView(lods, 60.0, 800) == 1,
          "a medium view should select the 0.3 metre LOD", failures);
    check(slam_tile::selectLodForView(lods, 300.0, 800) == 0,
          "a wide view should select the coarse one metre LOD", failures);
}

std::shared_ptr<const slam_tile::TileData> tileData(
    const slam_tile::TileId &id, int pointCount)
{
    auto tile = std::make_shared<slam_tile::TileData>();
    tile->meta.id = id;
    tile->meta.point_count = static_cast<quint64>(pointCount);
    tile->points.resize(pointCount);
    return tile;
}

void testByteBudgetUsesLruAndPinsActiveTiles(int &failures)
{
    const quint64 tileBytes = 4 * sizeof(M_PointXYZI);
    slam_tile::TileMemoryCache cache(tileBytes * 2);
    const slam_tile::TileId a{0, 0, 1};
    const slam_tile::TileId b{1, 0, 1};
    const slam_tile::TileId c{2, 0, 1};
    cache.insert(tileData(a, 4));
    cache.insert(tileData(b, 4));
    check(cache.contains(a) && cache.contains(b),
          "two tiles should fit within the byte budget", failures);
    cache.touch(a);
    cache.insert(tileData(c, 4));
    check(cache.contains(a) && cache.contains(c) && !cache.contains(b),
          "inserting a third tile should evict the least recently used tile",
          failures);

    cache.setPinned({a});
    cache.setBudgetBytes(tileBytes);
    check(cache.contains(a) && !cache.contains(c),
          "a pinned active tile must survive budget trimming", failures);
    check(cache.memoryBytes() <= tileBytes,
          "unpinned tiles should be removed until the byte budget is met",
          failures);
}

void testUnloadRemovesTilesOutsideRetainSet(int &failures)
{
    slam_tile::TileMemoryCache cache(1024 * 1024);
    const slam_tile::TileId keep{0, 0, 0};
    const slam_tile::TileId remove{8, 8, 0};
    cache.insert(tileData(keep, 1));
    cache.insert(tileData(remove, 1));
    const QSet<slam_tile::TileId> removed = cache.removeOutside({keep});
    check(cache.contains(keep) && !cache.contains(remove)
              && removed.contains(remove),
          "tiles beyond the unload region must leave the RAM cache",
          failures);
}

void testManagerLoadsOnlyRequestedTilesAndUnloadsOldRegion(int &failures)
{
    QTemporaryDir directory;
    if (!directory.isValid()) {
        ++failures;
        return;
    }

    slam_tile::Manifest manifest;
    manifest.anchor.valid = true;
    manifest.anchor.timestamp = 1.0;
    manifest.anchor.latitude_deg = 35.4;
    manifest.anchor.longitude_deg = 119.5;
    manifest.bounds = {0.0, 0.0, 0.0, 110.0, 10.0, 1.0};
    manifest.tile_size_m = 50.0;
    manifest.lods = {{0, 1.0}};
    manifest.sources = {{QStringLiteral("fixture.slammap"), 1, 1}};

    for (const slam_tile::TileId id :
         {slam_tile::TileId{0, 0, 0}, slam_tile::TileId{2, 0, 0}}) {
        slam_tile::TileData tile;
        tile.meta.id = id;
        tile.meta.bounds = {id.x * 50.0, 0.0, 0.0,
                            id.x * 50.0 + 10.0, 10.0, 1.0};
        tile.tile_origin_x = id.x * 50.0;
        tile.points = tileData(id, 3)->points;
        tile.meta.relative_path = slam_tile::tileRelativePath(id);
        QByteArray checksum;
        QString error;
        const QString path = QDir(directory.path()).absoluteFilePath(
            tile.meta.relative_path);
        check(slam_tile::saveTile(path, tile, &checksum, &error),
              "manager fixture tile should save", failures);
        tile.meta.point_count = tile.points.size();
        tile.meta.byte_size = QFileInfo(path).size();
        tile.meta.checksum_sha256 = checksum;
        manifest.tiles.append(tile.meta);
    }
    const QString manifestPath = directory.filePath(QStringLiteral("manifest.json"));
    QString error;
    check(slam_tile::saveManifest(manifestPath, manifest, &error),
          "manager fixture manifest should save", failures);

    slam_tile::ManagerSettings settings;
    settings.view_update_delay_ms = 0;
    settings.prefetch_ring_tiles = 0;
    settings.unload_ring_tiles = 0;
    slam_tile::SlamTileMapManager manager(settings);
    check(manager.openManifest(manifestPath, &error),
          "manager should open a valid manifest", failures);

    QVector<slam_tile::TileId> loaded;
    QVector<slam_tile::TileId> removed;
    QObject::connect(&manager, &slam_tile::SlamTileMapManager::tileAvailable,
                     [&loaded](slam_tile::TileId id, slam_tile::TileDataPtr) {
        loaded.append(id);
    });
    QObject::connect(&manager, &slam_tile::SlamTileMapManager::tileRemoved,
                     [&removed](slam_tile::TileId id) { removed.append(id); });

    auto waitForIdle = [&manager]() {
        QEventLoop loop;
        QTimer poll;
        poll.setInterval(10);
        QObject::connect(&poll, &QTimer::timeout, &loop, [&]() {
            if (manager.stats().inflight_tiles == 0)
                loop.quit();
        });
        QTimer::singleShot(3000, &loop, &QEventLoop::quit);
        poll.start();
        loop.exec();
    };

    manager.updateViewNow({1.0, 1.0, 9.0, 9.0}, 0);
    waitForIdle();
    check(loaded.contains({0, 0, 0}) && !loaded.contains({2, 0, 0}),
          "manager should asynchronously load only the requested region",
          failures);

    manager.updateViewNow({101.0, 1.0, 109.0, 9.0}, 0);
    waitForIdle();
    check(loaded.contains({2, 0, 0}) && removed.contains({0, 0, 0}),
          "panning should load the new tile and unload the old region",
          failures);
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    testTileIndexUsesHalfOpenIntervals(failures);
    testViewportAndPrefetchSets(failures);
    testRequestGenerationGuard(failures);
    testViewportBoundsAndLodSelection(failures);
    testByteBudgetUsesLruAndPinsActiveTiles(failures);
    testUnloadRemovesTilesOutsideRetainSet(failures);
    testManagerLoadsOnlyRequestedTilesAndUnloadsOldRegion(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM tile cache tests passed\n";
    return 0;
}

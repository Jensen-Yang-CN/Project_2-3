#include "HistoricalMapExportSource.h"

#include "SlamTileMapIO.h"

#include <QCoreApplication>
#include <QDir>
#include <QTemporaryDir>

#include <cmath>
#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

M_PointXYZI point(float x, float y, float z, quint8 intensity = 80)
{
    M_PointXYZI value{};
    value.x = x;
    value.y = y;
    value.z = z;
    value.intensity = intensity;
    return value;
}

bool writeTile(const QString &root, slam_tile::Manifest &manifest,
               const slam_tile::TileId &id, double voxelSize,
               const QVector<M_PointXYZI> &points, QString *error)
{
    slam_tile::TileData tile;
    tile.meta.id = id;
    tile.meta.relative_path = slam_tile::tileRelativePath(id);
    tile.tile_origin_x = manifest.grid_origin_x
        + static_cast<double>(id.x) * manifest.tile_size_m;
    tile.tile_origin_y = manifest.grid_origin_y
        + static_cast<double>(id.y) * manifest.tile_size_m;
    tile.points = points;
    tile.meta.bounds = {points.first().x, points.first().y, points.first().z,
                        points.first().x, points.first().y, points.first().z};
    for (const M_PointXYZI &p : points) {
        tile.meta.bounds.min_x = std::min(tile.meta.bounds.min_x,
                                         static_cast<double>(p.x));
        tile.meta.bounds.min_y = std::min(tile.meta.bounds.min_y,
                                         static_cast<double>(p.y));
        tile.meta.bounds.min_z = std::min(tile.meta.bounds.min_z,
                                         static_cast<double>(p.z));
        tile.meta.bounds.max_x = std::max(tile.meta.bounds.max_x,
                                         static_cast<double>(p.x));
        tile.meta.bounds.max_y = std::max(tile.meta.bounds.max_y,
                                         static_cast<double>(p.y));
        tile.meta.bounds.max_z = std::max(tile.meta.bounds.max_z,
                                         static_cast<double>(p.z));
    }
    const QString absolute = QDir(root).absoluteFilePath(tile.meta.relative_path);
    QByteArray checksum;
    if (!slam_tile::saveTile(absolute, tile, &checksum, error))
        return false;
    tile.meta.point_count = static_cast<quint64>(points.size());
    tile.meta.byte_size = static_cast<quint64>(QFileInfo(absolute).size());
    tile.meta.checksum_sha256 = checksum;
    manifest.tiles.append(tile.meta);
    if (manifest.lods.isEmpty())
        manifest.lods.append({id.lod, voxelSize});
    return true;
}

QString createFixture(QTemporaryDir &dir, QString *error,
                      bool includeBerth = true)
{
    slam_tile::Manifest manifest;
    manifest.anchor.valid = true;
    manifest.anchor.timestamp = 10.0;
    manifest.anchor.latitude_deg = 0.1;
    manifest.anchor.longitude_deg = 0.0;
    manifest.anchor.altitude_m = 2.0;
    manifest.grid_origin_x = 0.0;
    manifest.grid_origin_y = 0.0;
    manifest.tile_size_m = 50.0;
    manifest.bounds = {0.0, 0.0, 0.0, 100.0, 50.0, 3.0};
    manifest.lods = {{0, 0.3}, {1, 0.1}};
    manifest.sources = {{QStringLiteral("C:/fixtures/history.slammap"),
                         1024, 123456}};
    usv::SlamMapBerth berth;
    berth.id = 42;
    berth.timestamp = 10.0;
    berth.x = 12.0;
    berth.y = 6.0;
    berth.z = 0.0;
    berth.width = 4.0;
    berth.length = 8.0;
    berth.angle_deg = 15.0;
    berth.kind = 1;
    berth.opening_edge = 2;
    berth.observation_count = 3;
    berth.confidence = 0.9;
    if (includeBerth)
        manifest.berths.append(berth);

    if (!writeTile(dir.path(), manifest, {0, 0, 1}, 0.1,
                   {point(11.2f, 0.2f, 0.0f),
                    point(11.8f, 0.8f, 0.0f),
                    point(13.2f, 0.2f, 0.0f),
                    point(13.8f, 0.8f, 0.0f),
                    point(20.0f, 20.0f, 0.0f)}, error)) {
        return {};
    }
    if (!writeTile(dir.path(), manifest, {1, 0, 0}, 0.3,
                   {point(55.0f, 5.0f, 0.0f)}, error)) {
        return {};
    }
    const QString path = dir.filePath(QStringLiteral("manifest.json"));
    return slam_tile::saveManifest(path, manifest, error) ? path : QString{};
}

void testOpenAndInitialStreamUsesBerthAndBackgroundLods(int &failures)
{
    QTemporaryDir dir;
    QString error;
    const QString manifestPath = createFixture(dir, &error);
    if (manifestPath.isEmpty())
        std::cerr << "fixture error: " << error.toStdString() << '\n';
    check(!manifestPath.isEmpty(), "fixture should be created", failures);

    history_map_export::HistoricalMapExportSource source;
    check(source.openManifest(manifestPath, &error),
          "history manifest should open", failures);
    check(source.berths().size() == 1
              && source.berths().front().id == 42
              && std::abs(source.berths().front().x - 12.0) < 1e-9,
          "historical manifest should expose persisted berth records",
          failures);
    check(source.finestLod() == 1 && source.backgroundLod() == 0,
          "berth and background LODs should be selected", failures);
    check(source.berthTileCount() == 1
              && source.backgroundTileCount() == 1
              && source.fullMapTileCount() == 2,
          "one berth tile and one background tile should be streamed",
          failures);
    std::size_t totalPoints = 0;
    bool sawBerthPoints = false;
    bool sawBackgroundPoints = false;
    while (source.hasNextFullMapTile()) {
        std::vector<M_PointXYZI> points;
        check(source.takeNextFullMapTile(points, &error),
              "initial stream should read every selected tile", failures);
        totalPoints += points.size();
        for (const M_PointXYZI &p : points) {
            sawBerthPoints = sawBerthPoints || p.x < 20.0f;
            sawBackgroundPoints = sawBackgroundPoints || p.x > 50.0f;
        }
    }
    check(totalPoints == 6 && sawBerthPoints && sawBackgroundPoints,
          "initial stream should contain berth and background points",
          failures);
    check(!source.hasNextFullMapTile(),
          "initial tile cursor should become exhausted", failures);
}

void testEveryKeyframeBuildsHistoryEnuOverlap(int &failures)
{
    QTemporaryDir dir;
    QString error;
    history_map_export::HistoricalMapExportSource source;
    check(source.openManifest(createFixture(dir, &error), &error),
          "history fixture should open", failures);

    usv::SlamGeoAnchor liveAnchor = source.anchor();
    liveAnchor.longitude_deg = 0.0001; // about +11.13 m east at equator

    usv::SlamKeyframe live;
    live.keyframe_id = 77;
    live.timestamp = 123.5;
    live.pose = Eigen::Isometry3d::Identity();
    live.pose.translation().x() = 2.0;
    live.cloud = {point(0.0f, 0.0f, 0.0f), point(1.0f, 1.0f, 0.0f)};

    usv::SlamKeyframe first;
    history_map_export::OverlapStats firstStats;
    check(source.makeOverlapVirtualKeyframe(
              live, liveAnchor, 0.5, first, &firstStats, &error),
          "valid anchors should produce a virtual keyframe", failures);
    check(first.keyframe_id == live.keyframe_id
              && first.timestamp == live.timestamp
              && first.pose.isApprox(Eigen::Isometry3d::Identity())
              && first.cloud.size() == 2,
          "virtual keyframe should contain only overlapping history points",
          failures);

    usv::SlamKeyframe second;
    history_map_export::OverlapStats secondStats;
    check(source.makeOverlapVirtualKeyframe(
              live, liveAnchor, 0.5, second, &secondStats, &error)
              && second.cloud.size() == first.cloud.size()
              && secondStats.selected_points == firstStats.selected_points,
          "the same overlap must be regenerated for every realtime keyframe",
          failures);
}

void testMapsWithoutBerthsUseBackgroundLod(int &failures)
{
    QTemporaryDir dir;
    QString error;
    history_map_export::HistoricalMapExportSource source;
    check(source.openManifest(createFixture(dir, &error, false), &error),
          "a historical map without berths should still open", failures);
    check(source.berths().isEmpty()
              && source.backgroundLod() == 0
              && source.berthTileCount() == 1
              && source.backgroundTileCount() == 1
              && source.fullMapTileCount() == 2,
          "maps without berth records should prefer the background LOD",
          failures);
}

void testInvalidLiveAnchorDoesNotCreateVirtualFrame(int &failures)
{
    QTemporaryDir dir;
    QString error;
    history_map_export::HistoricalMapExportSource source;
    check(source.openManifest(createFixture(dir, &error), &error),
          "history fixture should open", failures);
    usv::SlamKeyframe live;
    live.cloud = {point(0.0f, 0.0f, 0.0f), point(1.0f, 1.0f, 0.0f)};
    usv::SlamKeyframe output;
    check(!source.makeOverlapVirtualKeyframe(
              live, usv::SlamGeoAnchor{}, 0.5, output, nullptr, &error),
          "invalid live anchor must be rejected", failures);
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    testOpenAndInitialStreamUsesBerthAndBackgroundLods(failures);
    testEveryKeyframeBuildsHistoryEnuOverlap(failures);
    testMapsWithoutBerthsUseBackgroundLod(failures);
    testInvalidLiveAnchorDoesNotCreateVirtualFrame(failures);
    if (failures == 0)
        std::cout << "historical_map_export_tests: PASS\n";
    return failures == 0 ? 0 : 1;
}

#include "SlamTileMapBuilder.h"
#include "SlamTileMapIO.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>

#include <Eigen/Geometry>

#include <algorithm>
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

bool close(double actual, double expected, double epsilon = 1e-5)
{
    return std::abs(actual - expected) <= epsilon;
}

M_PointXYZI point(float x, float y, float z, quint8 intensity)
{
    M_PointXYZI value{};
    value.x = x;
    value.y = y;
    value.z = z;
    value.intensity = intensity;
    return value;
}

usv::SlamKeyframe keyframe(quint64 id, double tx, double ty,
                           std::initializer_list<M_PointXYZI> points)
{
    usv::SlamKeyframe value;
    value.keyframe_id = id;
    value.timestamp = 100.0 + static_cast<double>(id);
    value.pose = Eigen::Isometry3d::Identity();
    value.pose.translation() = Eigen::Vector3d(tx, ty, 0.0);
    value.cloud.assign(points.begin(), points.end());
    return value;
}

slam_tile::Manifest sampleManifest()
{
    slam_tile::Manifest manifest;
    manifest.anchor.valid = true;
    manifest.anchor.timestamp = 12.0;
    manifest.anchor.latitude_deg = 35.4;
    manifest.anchor.longitude_deg = 119.5;
    manifest.anchor.altitude_m = 3.2;
    manifest.bounds = {-50.0, 0.0, -2.0, 100.0, 50.0, 8.0};
    manifest.grid_origin_x = 0.0;
    manifest.grid_origin_y = 0.0;
    manifest.tile_size_m = 50.0;
    manifest.lods = {{0, 1.0}, {1, 0.3}, {2, 0.1}};
    manifest.sources = {{QStringLiteral("C:/maps/a.slammap"), 1234, 4567},
                        {QStringLiteral("C:/maps/b.slammap"), 2222, 8888}};
    slam_tile::TileMeta meta;
    meta.id = {-1, 0, 1};
    meta.relative_path = QStringLiteral("L1/tile_-1_0.tilebin");
    meta.bounds = {-50.0, 0.0, -1.0, 0.0, 50.0, 4.0};
    meta.point_count = 2;
    meta.byte_size = 99;
    meta.checksum_sha256 = QByteArray::fromHex(
        "0123456789abcdef0123456789abcdef"
        "0123456789abcdef0123456789abcdef");
    manifest.tiles.append(meta);
    manifest.trajectory_relative_path = QStringLiteral("trajectory.bin");
    return manifest;
}

void testManifestRoundTrip(int &failures)
{
    QTemporaryDir dir;
    check(dir.isValid(), "temporary directory must be available", failures);
    if (!dir.isValid())
        return;

    const slam_tile::Manifest source = sampleManifest();
    const QString path = dir.filePath(QStringLiteral("manifest.json"));
    QString error;
    check(slam_tile::saveManifest(path, source, &error),
          "manifest should save", failures);

    slam_tile::Manifest loaded;
    check(slam_tile::loadManifest(path, loaded, &error),
          "manifest should load", failures);
    check(loaded.anchor.valid
              && close(loaded.anchor.latitude_deg, source.anchor.latitude_deg)
              && close(loaded.anchor.longitude_deg, source.anchor.longitude_deg),
          "manifest must preserve its geographic anchor", failures);
    check(loaded.sources.size() == 2
              && loaded.sources[0].canonical_path.endsWith("a.slammap")
              && loaded.sources[1].canonical_path.endsWith("b.slammap"),
          "manifest must preserve source order", failures);
    check(loaded.lods.size() == 3
              && close(loaded.lods[1].voxel_size_m, 0.3)
              && loaded.tiles.size() == 1
              && loaded.tiles[0].id == source.tiles[0].id,
          "manifest must preserve LOD and tile metadata", failures);

    slam_tile::Manifest before = loaded;
    QFile truncated(path);
    check(truncated.open(QIODevice::WriteOnly | QIODevice::Truncate),
          "manifest fixture should reopen", failures);
    truncated.write("{\"format_magic\":\"BAD\"}");
    truncated.close();
    check(!slam_tile::loadManifest(path, loaded, &error),
          "invalid manifest must be rejected", failures);
    check(loaded.tiles.size() == before.tiles.size()
              && loaded.anchor.latitude_deg == before.anchor.latitude_deg,
          "failed manifest load must leave the output unchanged", failures);
}

void testTileRoundTripAndChecksum(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    slam_tile::TileData source;
    source.meta.id = {-1, 2, 1};
    source.meta.bounds = {-50.0, 100.0, -2.0, 0.0, 150.0, 6.0};
    source.tile_origin_x = -50.0;
    source.tile_origin_y = 100.0;
    source.points = {point(-49.5f, 100.25f, -1.0f, 17),
                     point(-0.1f, 149.9f, 5.0f, 91)};

    const QString path = dir.filePath(QStringLiteral("tile.tilebin"));
    QByteArray checksum;
    QString error;
    check(slam_tile::saveTile(path, source, &checksum, &error)
              && checksum.size() == 32,
          "tile should save with a SHA-256 checksum", failures);

    slam_tile::TileData loaded;
    check(slam_tile::loadTile(path, checksum, loaded, &error),
          "tile should load when its checksum matches", failures);
    check(loaded.meta.id == source.meta.id && loaded.points.size() == 2,
          "tile identity and point count must round-trip", failures);
    if (loaded.points.size() == 2) {
        check(close(loaded.points[0].x, -49.5)
                  && close(loaded.points[0].y, 100.25)
                  && loaded.points[1].intensity == 91,
              "tile-local storage must restore world ENU coordinates",
              failures);
    }

    QFile corrupt(path);
    check(corrupt.open(QIODevice::Append),
          "tile fixture should open for corruption", failures);
    corrupt.write("x", 1);
    corrupt.close();
    slam_tile::TileData before = loaded;
    check(!slam_tile::loadTile(path, checksum, loaded, &error),
          "a checksum mismatch must be rejected", failures);
    check(loaded.points.size() == before.points.size(),
          "failed tile load must leave the output unchanged", failures);
}

void testCacheKeyTracksSourcesAndParameters(int &failures)
{
    const QVector<slam_tile::SourceFingerprint> sources{
        {QStringLiteral("C:/maps/a.slammap"), 100, 200},
        {QStringLiteral("C:/maps/b.slammap"), 300, 400}};
    const QVector<slam_tile::LodLevel> lods{{0, 1.0}, {1, 0.3}};
    const QString base = slam_tile::cacheKey(sources, 50.0, lods);
    QVector<slam_tile::SourceFingerprint> reversed = sources;
    std::reverse(reversed.begin(), reversed.end());
    check(base != slam_tile::cacheKey(reversed, 50.0, lods),
          "cache key must preserve source order", failures);
    check(base != slam_tile::cacheKey(sources, 100.0, lods),
          "cache key must include tile size", failures);
    QVector<slam_tile::LodLevel> changed = lods;
    changed[1].voxel_size_m = 0.5;
    check(base != slam_tile::cacheKey(sources, 50.0, changed),
          "cache key must include LOD parameters", failures);
}

void testBuilderProjectsAndDownsamples(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    slam_map_io::MapArchive archive;
    archive.anchor.valid = true;
    archive.anchor.latitude_deg = 35.4;
    archive.anchor.longitude_deg = 119.5;
    archive.keyframes.push_back(keyframe(
        1, -49.0, 10.0,
        {point(0.0f, 0.0f, 0.0f, 20),
         point(0.04f, 0.04f, 0.0f, 21),
         point(49.0f, 0.0f, 1.0f, 22)}));
    archive.keyframes.push_back(keyframe(
        2, 51.0, 10.0,
        {point(0.0f, 0.0f, 0.0f, 30),
         point(0.15f, 0.0f, 0.0f, 31)}));

    slam_tile::BuildOptions options;
    options.tile_size_m = 50.0;
    options.lods = {{0, 1.0}, {1, 0.3}, {2, 0.1}};
    options.display_intensity = 90;
    const QVector<slam_tile::SourceFingerprint> sources{
        {QStringLiteral("C:/maps/a.slammap"), 100, 200}};
    const QString key = slam_tile::cacheKey(
        sources, options.tile_size_m, options.lods);
    const slam_tile::BuildResult result = slam_tile::buildTileCache(
        archive, sources, dir.path(), key, options);
    check(result.success, "a valid ENU archive should build a tile cache",
          failures);
    if (!result.success)
        return;
    check(result.manifest.anchor.valid
              && close(result.manifest.anchor.latitude_deg, 35.4),
          "the tile cache must preserve the base ENU anchor", failures);

    quint64 counts[3] = {0, 0, 0};
    bool sawNegativeTile = false;
    bool sawPositiveTile = false;
    for (const slam_tile::TileMeta &meta : result.manifest.tiles) {
        if (meta.id.lod >= 0 && meta.id.lod < 3)
            counts[meta.id.lod] += meta.point_count;
        sawNegativeTile = sawNegativeTile || meta.id.x == -1;
        sawPositiveTile = sawPositiveTile || meta.id.x == 1;
    }
    check(sawNegativeTile && sawPositiveTile,
          "world ENU projection must partition negative and positive coordinates",
          failures);
    check(counts[0] <= counts[1] && counts[1] <= counts[2],
          "coarser LODs must not contain more points than finer LODs",
          failures);
    check(counts[0] < 5,
          "coarse global voxel downsampling must remove nearby duplicates",
          failures);

    slam_tile::Manifest loaded;
    QString error;
    check(slam_tile::loadManifest(result.manifest_path, loaded, &error)
              && loaded.tiles.size() == result.manifest.tiles.size(),
          "the published cache manifest must be readable", failures);

    slam_map_io::MapArchive empty;
    const slam_tile::BuildResult failed = slam_tile::buildTileCache(
        empty, sources, dir.path(), QStringLiteral("empty"), options);
    check(!failed.success
              && !QFile::exists(dir.filePath(QStringLiteral(
                  "empty/manifest.json"))),
          "an invalid archive must not publish a manifest", failures);
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    testManifestRoundTrip(failures);
    testTileRoundTripAndChecksum(failures);
    testCacheKeyTracksSourcesAndParameters(failures);
    testBuilderProjectsAndDownsamples(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM tile map tests passed\n";
    return 0;
}

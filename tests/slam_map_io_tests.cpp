#include "SlamMapBinaryIO.h"

#include <QCoreApplication>
#include <QDataStream>
#include <QDateTime>
#include <QFile>
#include <QTemporaryDir>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
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

M_PointXYZI makePoint(float x, float y, float z, uint8_t intensity)
{
    M_PointXYZI point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

usv::SlamKeyframe makeKeyframe(uint64_t id, double timestamp,
                               double tx, double ty)
{
    usv::SlamKeyframe keyframe;
    keyframe.keyframe_id = id;
    keyframe.timestamp = timestamp;
    keyframe.pose = Eigen::Isometry3d::Identity();
    keyframe.pose.translation() = Eigen::Vector3d(tx, ty, 0.5);
    keyframe.pose.linear() =
        Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    keyframe.cloud.push_back(makePoint(1.0f, 2.0f, 3.0f, 17));
    keyframe.cloud.push_back(makePoint(-2.0f, 0.5f, -1.0f, 29));
    return keyframe;
}

usv::SlamMapBerth makeMapBerth(uint64_t id)
{
    usv::SlamMapBerth berth;
    berth.id = id;
    berth.timestamp = 12.75;
    berth.x = 15.5;
    berth.y = -7.25;
    berth.z = 0.4;
    berth.width = 8.5;
    berth.length = 18.0;
    berth.angle_deg = 32.0;
    berth.kind = 1;
    berth.opening_edge = 2;
    berth.observation_count = 12;
    berth.confidence = 0.93;
    return berth;
}

bool writeVersion20Archive(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return false;
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setVersion(QDataStream::Qt_5_12);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    stream.writeRawData("USVSLM2", 7);
    stream << static_cast<quint16>(2)
           << static_cast<quint16>(0)
           << static_cast<quint8>(1)
           << static_cast<quint8>(1)
           << 20.0 << 31.0 << 121.0 << 2.0
           << static_cast<quint32>(1)
           << static_cast<quint64>(9) << 20.0
           << 1.0 << 2.0 << 0.5
           << 0.0 << 0.0 << 0.0 << 1.0
           << static_cast<quint32>(1);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    stream << 3.0f << 4.0f << 5.0f << static_cast<quint8>(42);
    return stream.status() == QDataStream::Ok;
}

void testArchiveRoundTrip(int &failures)
{
    QTemporaryDir dir;
    check(dir.isValid(), "temporary directory should be available", failures);
    if (!dir.isValid())
        return;

    slam_map_io::MapArchive source;
    source.anchor.valid = true;
    source.anchor.timestamp = 12.5;
    source.anchor.latitude_deg = 31.23456789;
    source.anchor.longitude_deg = 121.45678901;
    source.anchor.altitude_m = 4.2;
    source.keyframes.push_back(makeKeyframe(7, 12.5, 1.0, 2.0));
    source.keyframes.push_back(makeKeyframe(8, 13.0, 3.0, 4.0));
    source.berths.push_back(makeMapBerth(41));
    source.keyframes[0].geo_reference.valid = true;
    source.keyframes[0].geo_reference.timestamp = 12.5;
    source.keyframes[0].geo_reference.sync_error_sec = 0.02;
    source.keyframes[0].geo_reference.pose_lidar_enu =
        Eigen::Isometry3d::Identity();
    source.keyframes[0].geo_reference.pose_lidar_enu.translation() =
        Eigen::Vector3d(4.0, -3.0, 1.0);
    source.keyframes[0].geo_reference.pose_lidar_enu.linear() =
        Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    const QString path = dir.filePath(QStringLiteral("roundtrip.slammap"));
    QString error;
    check(slam_map_io::saveArchive(path, source, &error),
          "versioned archive should save", failures);

    slam_map_io::MapArchive loaded;
    check(slam_map_io::loadArchive(path, loaded, &error),
          "versioned archive should load", failures);
    check(loaded.anchor.valid
              && close(loaded.anchor.timestamp, 12.5)
              && close(loaded.anchor.latitude_deg, 31.23456789)
              && close(loaded.anchor.longitude_deg, 121.45678901)
              && close(loaded.anchor.altitude_m, 4.2),
          "geographic anchor must round-trip", failures);
    check(loaded.keyframes.size() == 2
              && loaded.keyframes[0].keyframe_id == 7
              && loaded.keyframes[1].cloud.size() == 2,
          "keyframe metadata and local clouds must round-trip", failures);
    check(loaded.berths.size() == 1
              && loaded.berths[0].id == 41
              && close(loaded.berths[0].x, 15.5)
              && close(loaded.berths[0].y, -7.25)
              && close(loaded.berths[0].width, 8.5)
              && close(loaded.berths[0].length, 18.0)
              && loaded.berths[0].kind == 1
              && loaded.berths[0].opening_edge == 2
              && loaded.berths[0].observation_count == 12
              && close(loaded.berths[0].confidence, 0.93),
          "saved berth layer must round-trip", failures);
    if (loaded.keyframes.size() == 2) {
        check(close(loaded.keyframes[1].pose.translation().x(), 3.0)
                  && close(loaded.keyframes[1].pose.translation().y(), 4.0),
              "keyframe pose must round-trip", failures);
        check(loaded.keyframes[0].geo_reference.valid
                  && close(loaded.keyframes[0].geo_reference.timestamp,
                           12.5)
                  && close(
                      loaded.keyframes[0].geo_reference.sync_error_sec,
                      0.02)
                  && close(loaded.keyframes[0]
                               .geo_reference.pose_lidar_enu.translation().x(),
                           4.0)
                  && close(loaded.keyframes[0]
                               .geo_reference.pose_lidar_enu.translation().y(),
                           -3.0),
              "v2.1 keyframe geographic reference must round-trip",
              failures);
        check(!loaded.keyframes[1].geo_reference.valid,
              "an invalid geographic reference must remain invalid",
              failures);
    }

    const QVector<M_PointXYZI> world =
        slam_map_io::rebuildWorldCloud(loaded.keyframes, 100, 90);
    check(world.size() == 4,
          "all restored keyframe points should be projected", failures);
    if (world.size() == 4) {
        const Eigen::Vector3d expected =
            source.keyframes[1].pose * Eigen::Vector3d(1.0, 2.0, 3.0);
        check(close(world[2].x, expected.x())
                  && close(world[2].y, expected.y())
                  && close(world[2].z, expected.z())
                  && world[2].intensity == 90,
              "local points must be transformed by T_map_lidar", failures);
    }
}

void testInvalidAnchorStillRoundTrips(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    slam_map_io::MapArchive source;
    source.keyframes.push_back(makeKeyframe(1, 2.0, 0.0, 0.0));
    QString error;
    const QString path = dir.filePath(QStringLiteral("local_only.slammap"));
    check(slam_map_io::saveArchive(path, source, &error),
          "local map without geographic anchor should save", failures);

    slam_map_io::MapArchive loaded;
    check(slam_map_io::loadArchive(path, loaded, &error)
              && !loaded.anchor.valid
              && loaded.keyframes.size() == 1,
          "invalid anchor marker should round-trip without losing keyframes",
          failures);
}

void testNonArchiveAndCorruptFiles(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    const QString legacyPath = dir.filePath(QStringLiteral("legacy.pcd"));
    QFile legacyFile(legacyPath);
    check(legacyFile.open(QIODevice::WriteOnly),
          "non-archive fixture should open", failures);
    const uint32_t count = 1;
    const M_PointXYZI point = makePoint(1.0f, 2.0f, 3.0f, 4);
    legacyFile.write(reinterpret_cast<const char *>(&count), sizeof(count));
    legacyFile.write(reinterpret_cast<const char *>(&point), sizeof(point));
    legacyFile.close();

    slam_map_io::MapArchive loaded;
    QString error;
    check(!slam_map_io::loadArchive(legacyPath, loaded, &error)
              && error.contains(QStringLiteral("魔数")),
          "non-archive point-only map must be rejected", failures);

    loaded.anchor.valid = true;
    loaded.anchor.latitude_deg = 30.0;
    loaded.anchor.longitude_deg = 120.0;
    loaded.keyframes.push_back(makeKeyframe(99, 9.0, 1.0, 2.0));

    QFile truncated(dir.filePath(QStringLiteral("truncated.slammap")));
    check(truncated.open(QIODevice::WriteOnly),
          "truncated fixture should open", failures);
    truncated.write("USVSLM2", 7);
    truncated.close();
    const slam_map_io::MapArchive before = loaded;
    check(!slam_map_io::loadArchive(truncated.fileName(), loaded, &error),
          "truncated versioned map must be rejected", failures);
    check(loaded.anchor.valid
              && loaded.anchor.latitude_deg == before.anchor.latitude_deg
              && loaded.keyframes.size() == before.keyframes.size()
              && loaded.keyframes[0].keyframe_id
                     == before.keyframes[0].keyframe_id,
          "failed load must leave output archive unchanged", failures);
}

void testLegacyCloudCompatibility(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    QVector<M_PointXYZI> source;
    source.append(makePoint(1.0f, 2.0f, 3.0f, 11));
    source.append(makePoint(-4.0f, 5.0f, -6.0f, 22));

    const QString path = dir.filePath(QStringLiteral("legacy_map.pcd"));
    QString error;
    check(slam_map_io::save(path, source, &error),
          "legacy point-cloud map should save", failures);

    QVector<M_PointXYZI> loaded;
    check(slam_map_io::load(path, loaded, &error),
          "legacy point-cloud map should load", failures);
    check(loaded.size() == source.size()
              && loaded[0].x == source[0].x
              && loaded[0].y == source[0].y
              && loaded[0].z == source[0].z
              && loaded[0].intensity == source[0].intensity
              && loaded[1].x == source[1].x
              && loaded[1].intensity == source[1].intensity,
          "legacy point-cloud fields must round-trip", failures);

    QFile truncated(path);
    check(truncated.open(QIODevice::WriteOnly | QIODevice::Truncate),
          "legacy corruption fixture should open", failures);
    const quint32 declaredCount = 2;
    truncated.write(reinterpret_cast<const char *>(&declaredCount),
                    sizeof(declaredCount));
    const M_PointXYZI onlyPoint = makePoint(7.0f, 8.0f, 9.0f, 33);
    truncated.write(reinterpret_cast<const char *>(&onlyPoint),
                    sizeof(onlyPoint));
    truncated.close();

    const QVector<M_PointXYZI> before = loaded;
    check(!slam_map_io::load(path, loaded, &error),
          "truncated legacy map must be rejected", failures);
    check(loaded.size() == before.size()
              && loaded[0].x == before[0].x
              && loaded[1].intensity == before[1].intensity,
          "failed legacy load must preserve output cloud", failures);
}

void testMinorVersionCompatibility(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    const QString legacyPath =
        dir.filePath(QStringLiteral("version_2_0.slammap"));
    check(writeVersion20Archive(legacyPath),
          "v2.0 fixture should be written", failures);

    slam_map_io::MapArchive loaded;
    QString error;
    check(slam_map_io::loadArchive(legacyPath, loaded, &error)
              && loaded.keyframes.size() == 1
              && loaded.source_format_major == 2
              && loaded.source_format_minor == 0
              && !loaded.keyframes[0].geo_reference.valid
              && loaded.berths.empty(),
          "v2.0 archive must load with an invalid geographic reference",
          failures);

    QFile legacy(legacyPath);
    check(legacy.open(QIODevice::ReadOnly),
          "v2.0 fixture should reopen", failures);
    QByteArray unsupportedBytes = legacy.readAll();
    legacy.close();
    if (unsupportedBytes.size() >= 11) {
        unsupportedBytes[9] = 3;
        unsupportedBytes[10] = 0;
    }
    const QString unsupportedPath =
        dir.filePath(QStringLiteral("version_2_3.slammap"));
    QFile unsupported(unsupportedPath);
    check(unsupported.open(QIODevice::WriteOnly)
              && unsupported.write(unsupportedBytes)
                     == unsupportedBytes.size(),
          "unsupported minor-version fixture should be written", failures);
    unsupported.close();

    check(!slam_map_io::loadArchive(unsupportedPath, loaded, &error)
              && error.contains(QStringLiteral("次版本")),
          "a newer unsupported minor version must be rejected", failures);
}

void testAutomaticRestoreCandidates(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }

    slam_map_io::MapArchive archive;
    archive.keyframes.push_back(makeKeyframe(1, 2.0, 0.0, 0.0));
    QString error;
    const QString older = dir.filePath(QStringLiteral("older.slammap"));
    const QString newer = dir.filePath(QStringLiteral("newer.slammap"));
    check(slam_map_io::saveArchive(older, archive, &error)
              && slam_map_io::saveArchive(newer, archive, &error),
          "restore candidate fixtures should save", failures);

    QFile olderFile(older);
    QFile newerFile(newer);
    olderFile.open(QIODevice::ReadOnly);
    newerFile.open(QIODevice::ReadOnly);
    olderFile.setFileTime(QDateTime::fromSecsSinceEpoch(1000),
                          QFileDevice::FileModificationTime);
    newerFile.setFileTime(QDateTime::fromSecsSinceEpoch(2000),
                          QFileDevice::FileModificationTime);
    olderFile.close();
    newerFile.close();

    check(slam_map_io::rememberLastMap(older, dir.path(), &error),
          "preferred startup map should be remembered", failures);
    QStringList candidates =
        slam_map_io::automaticRestoreCandidates(dir.path());
    check(candidates.size() == 2
              && QFileInfo(candidates[0]).canonicalFilePath()
                     == QFileInfo(older).canonicalFilePath()
              && QFileInfo(candidates[1]).canonicalFilePath()
                     == QFileInfo(newer).canonicalFilePath(),
          "remembered map must have priority over the newest map", failures);

    QFile::remove(older);
    candidates = slam_map_io::automaticRestoreCandidates(dir.path());
    check(candidates.size() == 1
              && QFileInfo(candidates[0]).canonicalFilePath()
                     == QFileInfo(newer).canonicalFilePath(),
          "missing remembered map must fall back to newest existing map",
          failures);
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    testArchiveRoundTrip(failures);
    testInvalidAnchorStillRoundTrips(failures);
    testNonArchiveAndCorruptFiles(failures);
    testLegacyCloudCompatibility(failures);
    testMinorVersionCompatibility(failures);
    testAutomaticRestoreCandidates(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM map I/O tests passed\n";
    return 0;
}

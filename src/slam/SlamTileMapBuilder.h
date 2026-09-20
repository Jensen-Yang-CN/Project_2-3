#ifndef SLAMTILEMAPBUILDER_H
#define SLAMTILEMAPBUILDER_H

#include "SlamMapBinaryIO.h"
#include "SlamTileMapTypes.h"

#include <QString>
#include <QVector>

namespace slam_tile {

struct BuildOptions {
    double tile_size_m = 50.0;
    QVector<LodLevel> lods{{0, 1.0}, {1, 0.3}, {2, 0.1}};
    quint8 display_intensity = 90;
};

struct BuildStats {
    quint64 source_points = 0;
    QVector<quint64> lod_points;
    int tile_files = 0;
};

struct BuildResult {
    bool success = false;
    bool cache_hit = false;
    QString cache_directory;
    QString manifest_path;
    Manifest manifest;
    BuildStats stats;
    QString error;
};

BuildResult buildTileCache(
    const slam_map_io::MapArchive &archive,
    const QVector<SourceFingerprint> &sources,
    const QString &cacheRoot,
    const QString &cacheKeyValue,
    const BuildOptions &options = {});

}  // namespace slam_tile

#endif  // SLAMTILEMAPBUILDER_H

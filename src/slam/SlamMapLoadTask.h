#ifndef SLAMMAPLOADTASK_H
#define SLAMMAPLOADTASK_H

#include "SlamMapGeoCorrector.h"
#include "SlamMapStitcher.h"
#include "SlamTileMapBuilder.h"

#include <QString>
#include <QStringList>

namespace slam_map_load {

struct GeoCorrectionRecord {
    QString source_path;
    uint16_t format_major = slam_map_io::kFormatMajor;
    uint16_t format_minor = slam_map_io::kFormatMinor;
    slam_map_geo_correct::Diagnostic diagnostic;
};

struct Result {
    bool success = false;
    QString error;
    slam_map_stitch::Result stitched;
    QVector<GeoCorrectionRecord> geo_corrections;
    slam_tile::BuildResult tile_build;
};

Result loadAndStitch(
    const QStringList &paths,
    const slam_map_stitch::Options &options = slam_map_stitch::Options{});

Result loadAndPrepareTiles(
    const QStringList &paths,
    const QString &cacheRoot,
    const slam_map_stitch::Options &stitchOptions =
        slam_map_stitch::Options{},
    const slam_tile::BuildOptions &tileOptions = slam_tile::BuildOptions{});

}  // namespace slam_map_load

#endif  // SLAMMAPLOADTASK_H

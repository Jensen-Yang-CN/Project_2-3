#ifndef SLAMMAPBINARYIO_H
#define SLAMMAPBINARYIO_H

#include "common/pointxyz.h"
#include "common/slam_types.h"

#include <QVector>
#include <QString>
#include <QStringList>
#include <cstdint>
#include <vector>

namespace slam_map_io {

constexpr uint16_t kFormatMajor = 2;
constexpr uint16_t kFormatMinor = 2;
constexpr uint32_t kMaxKeyframeCount = 10000;
constexpr uint32_t kMaxPointsPerKeyframe = 5000000;
constexpr uint64_t kMaxTotalKeyframePoints = 100000000;
constexpr uint32_t kMaxMapBerthCount = 10000;
// Legacy format used by the source archive: [uint32 count][M_PointXYZI[]].
constexpr uint32_t kLegacyMaxPointCount = 5000000;

struct MapArchive {
    uint16_t source_format_major = kFormatMajor;
    uint16_t source_format_minor = kFormatMinor;
    usv::SlamGeoAnchor anchor;
    std::vector<usv::SlamKeyframe> keyframes;
    std::vector<usv::SlamMapBerth> berths;
};

bool saveArchive(const QString &path, const MapArchive &archive,
                 QString *error = nullptr);
bool loadArchive(const QString &path, MapArchive &archive,
                 QString *error = nullptr);

// Compatibility API for the archive's old flat .pcd map format. New maps
// must continue to use saveArchive()/loadArchive().
bool save(const QString &path, const QVector<M_PointXYZI> &cloud,
          QString *error = nullptr);
bool load(const QString &path, QVector<M_PointXYZI> &cloud,
          QString *error = nullptr);

QVector<M_PointXYZI> rebuildWorldCloud(
    const std::vector<usv::SlamKeyframe> &keyframes,
    int max_points,
    uint8_t display_intensity = 90);

bool rememberLastMap(const QString &mapPath, QString *error = nullptr);
bool rememberLastMap(const QString &mapPath, const QString &mapsDirectory,
                     QString *error = nullptr);
QStringList automaticRestoreCandidates();
QStringList automaticRestoreCandidates(const QString &mapsDirectory);

QString defaultMapsDirectory();

}  // namespace slam_map_io

#endif  // SLAMMAPBINARYIO_H

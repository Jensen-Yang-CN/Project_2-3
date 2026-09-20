#ifndef SLAMTILEMAPIO_H
#define SLAMTILEMAPIO_H

#include "SlamTileMapTypes.h"

#include <QString>

namespace slam_tile {

constexpr int kTileMapFormatMajor = 1;
constexpr int kTileMapFormatMinor = 1;

bool saveManifest(const QString &path, const Manifest &manifest,
                  QString *error = nullptr);
bool loadManifest(const QString &path, Manifest &manifest,
                  QString *error = nullptr);

bool saveTile(const QString &path, const TileData &tile,
              QByteArray *checksumSha256 = nullptr,
              QString *error = nullptr);
bool loadTile(const QString &path, const QByteArray &expectedChecksumSha256,
              TileData &tile, QString *error = nullptr);

bool saveTrajectory(const QString &path,
                    const QVector<TrajectoryPose> &trajectory,
                    QString *error = nullptr);
bool loadTrajectory(const QString &path,
                    QVector<TrajectoryPose> &trajectory,
                    QString *error = nullptr);

SourceFingerprint sourceFingerprint(const QString &path);
QString cacheKey(const QVector<SourceFingerprint> &sources,
                 double tileSizeM,
                 const QVector<LodLevel> &lods);

bool manifestMatches(const Manifest &manifest,
                     const QVector<SourceFingerprint> &sources,
                     double tileSizeM,
                     const QVector<LodLevel> &lods);

bool cacheFilesPresent(const QString &manifestPath,
                       const Manifest &manifest,
                       QString *error = nullptr);

QString tileRelativePath(const TileId &id);

}  // namespace slam_tile

#endif  // SLAMTILEMAPIO_H

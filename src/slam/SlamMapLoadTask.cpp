#include "SlamMapLoadTask.h"

#include "SlamMapBinaryIO.h"
#include "SlamTileMapIO.h"

#include <QDir>
#include <QFileInfo>

namespace slam_map_load {

namespace {

bool loadCompatibleArchive(const QString &path,
                           slam_map_io::MapArchive &archive,
                           QString *error)
{
    const QFileInfo info(path);
    if (info.suffix().compare(QStringLiteral("slammap"),
                              Qt::CaseInsensitive) == 0) {
        return slam_map_io::loadArchive(path, archive, error);
    }

    if (info.suffix().compare(QStringLiteral("pcd"),
                              Qt::CaseInsensitive) != 0) {
        if (error)
            *error = QStringLiteral(
                "仅支持 .slammap 或压缩包兼容的 .pcd 地图文件");
        return false;
    }

    QVector<M_PointXYZI> cloud;
    if (!slam_map_io::load(path, cloud, error))
        return false;

    usv::SlamKeyframe keyframe;
    keyframe.keyframe_id = 0;
    keyframe.timestamp = 0.0;
    keyframe.pose = Eigen::Isometry3d::Identity();
    keyframe.cloud.reserve(static_cast<std::size_t>(cloud.size()));
    for (const M_PointXYZI &point : cloud)
        keyframe.cloud.push_back(point);

    archive = slam_map_io::MapArchive{};
    archive.source_format_major = 1;
    archive.source_format_minor = 0;
    archive.keyframes.push_back(std::move(keyframe));
    return true;
}

}  // namespace

Result loadAndStitch(const QStringList &paths,
                     const slam_map_stitch::Options &options)
{
    Result result;
    if (paths.isEmpty()) {
        result.error = QStringLiteral("未选择要加载的 SLAM 地图");
        return result;
    }

    QVector<slam_map_stitch::Input> inputs;
    inputs.reserve(paths.size());
    for (const QString &path : paths) {
        const QFileInfo info(path);
        slam_map_io::MapArchive archive;
        QString loadError;
        if (!loadCompatibleArchive(path, archive, &loadError)) {
            result.error = QStringLiteral("加载地图失败: %1 (%2)")
                               .arg(path, loadError);
            return result;
        }
        slam_map_geo_correct::Result corrected =
            slam_map_geo_correct::correct(archive);
        result.geo_corrections.append(
            {info.absoluteFilePath(), archive.source_format_major,
             archive.source_format_minor, corrected.diagnostic});
        archive = std::move(corrected.archive);
        inputs.append({info.absoluteFilePath(), std::move(archive)});
    }

    result.stitched = slam_map_stitch::stitch(inputs, options);
    if (!result.stitched.success) {
        result.error = result.stitched.error;
        return result;
    }
    result.success = true;
    return result;
}

Result loadAndPrepareTiles(
    const QStringList &paths,
    const QString &cacheRoot,
    const slam_map_stitch::Options &stitchOptions,
    const slam_tile::BuildOptions &tileOptions)
{
    Result result;
    if (paths.isEmpty()) {
        result.error = QStringLiteral("未选择要加载的 SLAM 地图");
        return result;
    }
    if (cacheRoot.isEmpty()) {
        result.error = QStringLiteral("分块地图缓存目录为空");
        return result;
    }

    QVector<slam_tile::SourceFingerprint> sources;
    sources.reserve(paths.size());
    for (const QString &path : paths) {
        const QFileInfo info(path);
        if (info.suffix().compare(QStringLiteral("slammap"),
                                  Qt::CaseInsensitive) != 0
            && info.suffix().compare(QStringLiteral("pcd"),
                                     Qt::CaseInsensitive) != 0) {
            result.error = QStringLiteral(
                "仅支持 .slammap 或压缩包兼容的 .pcd 地图文件: %1")
                               .arg(path);
            return result;
        }
        if (!info.exists() || !info.isFile()) {
            result.error = QStringLiteral("地图文件不存在: %1").arg(path);
            return result;
        }
        sources.append(slam_tile::sourceFingerprint(path));
    }

    const QString key = slam_tile::cacheKey(
        sources, tileOptions.tile_size_m, tileOptions.lods);
    const QString cacheDirectory = QDir(cacheRoot).absoluteFilePath(key);
    const QString manifestPath = QDir(cacheDirectory).absoluteFilePath(
        QStringLiteral("manifest.json"));
    slam_tile::Manifest manifest;
    QString manifestError;
    if (slam_tile::loadManifest(manifestPath, manifest, &manifestError)
        && slam_tile::manifestMatches(manifest, sources,
                                      tileOptions.tile_size_m,
                                      tileOptions.lods)
        && slam_tile::cacheFilesPresent(
            manifestPath, manifest, &manifestError)) {
        result.success = true;
        result.tile_build.success = true;
        result.tile_build.cache_hit = true;
        result.tile_build.cache_directory = cacheDirectory;
        result.tile_build.manifest_path = manifestPath;
        result.tile_build.manifest = std::move(manifest);
        return result;
    }

    result = loadAndStitch(paths, stitchOptions);
    if (!result.success)
        return result;

    result.tile_build = slam_tile::buildTileCache(
        result.stitched.archive, sources, cacheRoot, key, tileOptions);
    if (!result.tile_build.success) {
        result.success = false;
        result.error = QStringLiteral("生成只读分块地图失败: %1")
                           .arg(result.tile_build.error);
    }
    return result;
}

}  // namespace slam_map_load

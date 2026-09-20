#include "AppConfig.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

AppConfig &AppConfig::instance()
{
    static AppConfig cfg;
    return cfg;
}

QString AppConfig::resolvePath(const QString &path)
{
    if (path.isEmpty())
        return QString();
    if (QDir::isAbsolutePath(path))
        return QDir::cleanPath(path);
    return QDir::cleanPath(QCoreApplication::applicationDirPath() + QLatin1Char('/') + path);
}

bool AppConfig::load(const QString &filePath)
{
    const QString path = filePath.isEmpty()
        ? resolvePath(QStringLiteral("config.json"))
        : resolvePath(filePath);
    m_configFilePath = path;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "[AppConfig] 未找到配置文件，使用默认值:" << path;
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        qWarning() << "[AppConfig] 配置文件格式无效:" << path;
        return false;
    }

    const QJsonObject root = doc.object();
    if (root.contains(QStringLiteral("maps_dir")))
        m_mapsDir = root.value(QStringLiteral("maps_dir")).toString(m_mapsDir);
    if (root.contains(QStringLiteral("devices_file")))
        m_devicesFile = root.value(QStringLiteral("devices_file")).toString(m_devicesFile);
    if (root.contains(QStringLiteral("calibration_file")))
        m_calibrationFile = root.value(QStringLiteral("calibration_file")).toString(m_calibrationFile);

    const QJsonObject pcapPlayback =
        root.value(QStringLiteral("pcap_playback")).toObject();
    if (pcapPlayback.contains(QStringLiteral("default_speed"))) {
        m_pcapPlayback.default_speed =
            pcapPlayback.value(QStringLiteral("default_speed"))
                .toDouble(m_pcapPlayback.default_speed);
    }
    m_pcapPlayback.default_speed =
        std::clamp(m_pcapPlayback.default_speed, 0.5, 4.0);

    const QJsonObject nodeScheduler =
        root.value(QStringLiteral("compute_node_scheduler")).toObject();
    if (nodeScheduler.contains(QStringLiteral(
            "stable_berth_timestamps_to_stop_bridge"))) {
        m_computeNodeScheduler.stable_berth_timestamps_to_stop_bridge =
            nodeScheduler.value(QStringLiteral(
                "stable_berth_timestamps_to_stop_bridge"))
                .toInt(m_computeNodeScheduler
                    .stable_berth_timestamps_to_stop_bridge);
    }
    if (nodeScheduler.contains(QStringLiteral(
            "missing_berth_timestamps_to_start_bridge"))) {
        m_computeNodeScheduler.missing_berth_timestamps_to_start_bridge =
            nodeScheduler.value(QStringLiteral(
                "missing_berth_timestamps_to_start_bridge"))
                .toInt(m_computeNodeScheduler
                    .missing_berth_timestamps_to_start_bridge);
    }
    m_computeNodeScheduler.stable_berth_timestamps_to_stop_bridge =
        std::clamp(
            m_computeNodeScheduler.stable_berth_timestamps_to_stop_bridge,
            1, 10000);
    m_computeNodeScheduler.missing_berth_timestamps_to_start_bridge =
        std::clamp(
            m_computeNodeScheduler.missing_berth_timestamps_to_start_bridge,
            1, 10000);

    const QJsonObject slam = root.value(QStringLiteral("slam")).toObject();
    if (slam.contains(QStringLiteral("voxel_leaf_size")))
        m_slam.voxel_leaf_size = slam.value(QStringLiteral("voxel_leaf_size")).toDouble(m_slam.voxel_leaf_size);
    if (slam.contains(QStringLiteral("max_gnss_sync_sec")))
    m_slam.max_gnss_sync_sec = slam.value(QStringLiteral("max_gnss_sync_sec")).toDouble(m_slam.max_gnss_sync_sec);

    const QJsonObject tileMap =
        slam.value(QStringLiteral("offline_tile_map")).toObject();
    SlamTileMapAppConfig &tiles = m_slam.offline_tile_map;
    if (tileMap.contains(QStringLiteral("enabled")))
        tiles.enabled = tileMap.value(QStringLiteral("enabled")).toBool(tiles.enabled);
    if (tileMap.contains(QStringLiteral("tile_size_m")))
        tiles.tile_size_m = tileMap.value(QStringLiteral("tile_size_m")).toDouble(tiles.tile_size_m);
    const QJsonArray lodSizes =
        tileMap.value(QStringLiteral("lod_voxel_sizes_m")).toArray();
    if (!lodSizes.isEmpty()) {
        QVector<double> parsed;
        parsed.reserve(lodSizes.size());
        for (const QJsonValue &value : lodSizes) {
            const double size = value.toDouble(-1.0);
            if (std::isfinite(size) && size > 0.0)
                parsed.append(size);
        }
        if (!parsed.isEmpty())
            tiles.lod_voxel_sizes_m = std::move(parsed);
    }
    if (tileMap.contains(QStringLiteral("prefetch_ring_tiles")))
        tiles.prefetch_ring_tiles = tileMap.value(QStringLiteral("prefetch_ring_tiles")).toInt(tiles.prefetch_ring_tiles);
    if (tileMap.contains(QStringLiteral("unload_ring_tiles")))
        tiles.unload_ring_tiles = tileMap.value(QStringLiteral("unload_ring_tiles")).toInt(tiles.unload_ring_tiles);
    if (tileMap.contains(QStringLiteral("ram_budget_mb")))
        tiles.ram_budget_mb = tileMap.value(QStringLiteral("ram_budget_mb")).toInt(tiles.ram_budget_mb);
    if (tileMap.contains(QStringLiteral("gpu_point_budget")))
        tiles.gpu_point_budget = tileMap.value(QStringLiteral("gpu_point_budget")).toInt(tiles.gpu_point_budget);
    if (tileMap.contains(QStringLiteral("view_update_delay_ms")))
        tiles.view_update_delay_ms = tileMap.value(QStringLiteral("view_update_delay_ms")).toInt(tiles.view_update_delay_ms);
    if (tileMap.contains(QStringLiteral("max_concurrent_loads")))
        tiles.max_concurrent_loads = tileMap.value(QStringLiteral("max_concurrent_loads")).toInt(tiles.max_concurrent_loads);

    tiles.tile_size_m = std::clamp(tiles.tile_size_m, 10.0, 500.0);
    for (double &size : tiles.lod_voxel_sizes_m)
        size = std::clamp(size, 0.02, 10.0);
    tiles.prefetch_ring_tiles = std::clamp(tiles.prefetch_ring_tiles, 0, 8);
    tiles.unload_ring_tiles = std::max(
        tiles.prefetch_ring_tiles,
        std::clamp(tiles.unload_ring_tiles, 0, 16));
    tiles.ram_budget_mb = std::clamp(tiles.ram_budget_mb, 64, 8192);
    tiles.gpu_point_budget = std::clamp(
        tiles.gpu_point_budget, 100000, 20000000);
    tiles.view_update_delay_ms = std::clamp(
        tiles.view_update_delay_ms, 0, 2000);
    tiles.max_concurrent_loads = std::clamp(
        tiles.max_concurrent_loads, 1, 8);

    const QJsonObject waterFilter =
        slam.value(QStringLiteral("water_filter")).toObject();
    WaterFilterAppConfig &water = m_slam.water_filter;
    if (waterFilter.contains(QStringLiteral("enabled")))
        water.enabled = waterFilter.value(QStringLiteral("enabled")).toBool(water.enabled);
    if (waterFilter.contains(QStringLiteral("clearance_m")))
        water.clearance_m = waterFilter.value(QStringLiteral("clearance_m")).toDouble(water.clearance_m);
    if (waterFilter.contains(QStringLiteral("plane_distance_m")))
        water.plane_distance_m = waterFilter.value(QStringLiteral("plane_distance_m")).toDouble(water.plane_distance_m);
    if (waterFilter.contains(QStringLiteral("max_tilt_deg")))
        water.max_tilt_deg = waterFilter.value(QStringLiteral("max_tilt_deg")).toDouble(water.max_tilt_deg);
    if (waterFilter.contains(QStringLiteral("min_inliers")))
        water.min_inliers = waterFilter.value(QStringLiteral("min_inliers")).toInt(water.min_inliers);
    if (waterFilter.contains(QStringLiteral("max_sample_points")))
        water.max_sample_points = waterFilter.value(QStringLiteral("max_sample_points")).toInt(water.max_sample_points);
    if (waterFilter.contains(QStringLiteral("ransac_iterations")))
        water.ransac_iterations = waterFilter.value(QStringLiteral("ransac_iterations")).toInt(water.ransac_iterations);
    if (waterFilter.contains(QStringLiteral("near_surface_band_m")))
        water.near_surface_band_m = waterFilter.value(QStringLiteral("near_surface_band_m")).toDouble(water.near_surface_band_m);
    if (waterFilter.contains(QStringLiteral("outlier_radius_m")))
        water.outlier_radius_m = waterFilter.value(QStringLiteral("outlier_radius_m")).toDouble(water.outlier_radius_m);
    if (waterFilter.contains(QStringLiteral("outlier_min_neighbors")))
        water.outlier_min_neighbors = waterFilter.value(QStringLiteral("outlier_min_neighbors")).toInt(water.outlier_min_neighbors);
    if (waterFilter.contains(QStringLiteral("max_fallback_frames")))
        water.max_fallback_frames = waterFilter.value(QStringLiteral("max_fallback_frames")).toInt(water.max_fallback_frames);

    // 配置允许现场调节，但限制范围以避免整帧误删或 RANSAC 计算失控。
    water.clearance_m = std::clamp(water.clearance_m, -5.0, 5.0);
    water.plane_distance_m = std::clamp(water.plane_distance_m, 0.01, 1.0);
    water.max_tilt_deg = std::clamp(water.max_tilt_deg, 1.0, 45.0);
    water.min_inliers = std::clamp(water.min_inliers, 3, 100000);
    water.max_sample_points =
        std::clamp(water.max_sample_points, water.min_inliers, 100000);
    water.ransac_iterations = std::clamp(water.ransac_iterations, 10, 2000);
    water.near_surface_band_m =
        std::clamp(water.near_surface_band_m, 0.0, 5.0);
    water.outlier_radius_m = std::clamp(water.outlier_radius_m, 0.05, 5.0);
    water.outlier_min_neighbors =
        std::clamp(water.outlier_min_neighbors, 0, 100);
    water.max_fallback_frames =
        std::clamp(water.max_fallback_frames, 0, 1000);

    const QJsonObject network = root.value(QStringLiteral("network")).toObject();
    if (network.contains(QStringLiteral("receive_mode")))
        m_network.receive_mode = network.value(QStringLiteral("receive_mode")).toString(m_network.receive_mode);
    const QJsonObject localIps = network.value(QStringLiteral("local_ips")).toObject();
    if (localIps.contains(QStringLiteral("lidar")))
        m_network.lidar_local_ip = localIps.value(QStringLiteral("lidar")).toString(m_network.lidar_local_ip);
    if (localIps.contains(QStringLiteral("video")))
        m_network.video_local_ip = localIps.value(QStringLiteral("video")).toString(m_network.video_local_ip);
    if (localIps.contains(QStringLiteral("navigation")))
        m_network.navigation_local_ip = localIps.value(QStringLiteral("navigation")).toString(m_network.navigation_local_ip);

    const QJsonObject systemLog = root.value(QStringLiteral("system_log")).toObject();
    if (systemLog.contains(QStringLiteral("enabled")))
        m_systemLog.enabled = systemLog.value(QStringLiteral("enabled")).toBool(m_systemLog.enabled);
    if (systemLog.contains(QStringLiteral("dir")))
        m_systemLog.dir = systemLog.value(QStringLiteral("dir")).toString(m_systemLog.dir);
    if (systemLog.contains(QStringLiteral("retain_days")))
        m_systemLog.retain_days = systemLog.value(QStringLiteral("retain_days")).toInt(m_systemLog.retain_days);
    if (systemLog.contains(QStringLiteral("min_free_gb")))
        m_systemLog.min_free_gb = systemLog.value(QStringLiteral("min_free_gb")).toDouble(m_systemLog.min_free_gb);
    if (systemLog.contains(QStringLiteral("cleanup_on_startup")))
        m_systemLog.cleanup_on_startup = systemLog.value(QStringLiteral("cleanup_on_startup")).toBool(m_systemLog.cleanup_on_startup);
    if (systemLog.contains(QStringLiteral("sensor_fps_interval_sec")))
        m_systemLog.sensor_fps_interval_sec = systemLog.value(QStringLiteral("sensor_fps_interval_sec")).toInt(m_systemLog.sensor_fps_interval_sec);
    if (systemLog.contains(QStringLiteral("disk_check_interval_sec")))
        m_systemLog.disk_check_interval_sec = systemLog.value(QStringLiteral("disk_check_interval_sec")).toInt(m_systemLog.disk_check_interval_sec);

    qInfo() << "[AppConfig] 已加载" << path
            << "maps_dir=" << mapsDirectory()
             << "voxel=" << m_slam.voxel_leaf_size
             << "gnss_sync=" << m_slam.max_gnss_sync_sec
             << "tile_map=" << m_slam.offline_tile_map.enabled
             << "tile_size=" << m_slam.offline_tile_map.tile_size_m
             << "tile_ram_mb=" << m_slam.offline_tile_map.ram_budget_mb
             << "pcap_default_speed=" << m_pcapPlayback.default_speed
             << "bridge_stop_stable_timestamps="
             << m_computeNodeScheduler.stable_berth_timestamps_to_stop_bridge
             << "bridge_start_missing_timestamps="
             << m_computeNodeScheduler.missing_berth_timestamps_to_start_bridge
             << "water_filter=" << m_slam.water_filter.enabled
            << "water_clearance=" << m_slam.water_filter.clearance_m
            << "network=" << m_network.receive_mode
            << "lidar_ip=" << m_network.lidar_local_ip
            << "system_log.enabled=" << m_systemLog.enabled
            << "dir=" << resolvePath(m_systemLog.dir)
            << "retain_days=" << m_systemLog.retain_days
            << "min_free_gb=" << m_systemLog.min_free_gb
            << "cleanup_on_startup=" << m_systemLog.cleanup_on_startup
            << "sensor_fps_interval_sec=" << m_systemLog.sensor_fps_interval_sec
            << "disk_check_interval_sec=" << m_systemLog.disk_check_interval_sec;
    return true;
}

QString AppConfig::mapsDirectory() const
{
    const QString dir = resolvePath(m_mapsDir);
    QDir().mkpath(dir);
    return dir;
}

QString AppConfig::devicesFilePath() const
{
    return resolvePath(m_devicesFile);
}

QString AppConfig::calibrationFilePath() const
{
    return resolvePath(m_calibrationFile);
}

float AppConfig::slamMapVoxelSize() const
{
    return static_cast<float>(m_slam.voxel_leaf_size);
}

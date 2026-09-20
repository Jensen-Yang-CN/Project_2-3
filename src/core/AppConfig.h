#ifndef APPCONFIG_H
#define APPCONFIG_H

#include "NetworkReceiveConfig.h"
#include "SystemLogConfig.h"

#include <QString>
#include <QVector>

struct WaterFilterAppConfig {
    bool enabled = true;
    double clearance_m = 0.25;
    double plane_distance_m = 0.15;
    double max_tilt_deg = 20.0;
    int min_inliers = 80;
    int max_sample_points = 12000;
    int ransac_iterations = 160;
    double near_surface_band_m = 1.0;
    double outlier_radius_m = 0.60;
    int outlier_min_neighbors = 2;
    int max_fallback_frames = 15;
};

struct SlamTileMapAppConfig {
    bool enabled = true;
    double tile_size_m = 50.0;
    QVector<double> lod_voxel_sizes_m{1.0, 0.3, 0.05};
    int prefetch_ring_tiles = 1;
    int unload_ring_tiles = 2;
    int ram_budget_mb = 512;
    int gpu_point_budget = 1500000;
    int view_update_delay_ms = 120;
    int max_concurrent_loads = 2;
};

struct SlamAppConfig {
    double voxel_leaf_size = 0.5;
    double max_gnss_sync_sec = 0.25;
    WaterFilterAppConfig water_filter;
    SlamTileMapAppConfig offline_tile_map;
};

struct PcapPlaybackConfig {
    double default_speed = 1.0;
};

struct ComputeNodeSchedulerAppConfig {
    int stable_berth_timestamps_to_stop_bridge = 10;
    int missing_berth_timestamps_to_start_bridge = 10;
};

class AppConfig
{
public:
    static AppConfig &instance();

    /** 从程序目录下 config.json 加载；失败则保留内置默认值 */
    bool load(const QString &filePath = QString());

    QString mapsDirectory() const;
    QString devicesFilePath() const;
    QString calibrationFilePath() const;
    float slamMapVoxelSize() const;

    const SlamAppConfig &slam() const { return m_slam; }
    const PcapPlaybackConfig &pcapPlayback() const { return m_pcapPlayback; }
    const ComputeNodeSchedulerAppConfig &computeNodeScheduler() const
    {
        return m_computeNodeScheduler;
    }
    const NetworkReceiveConfig &network() const { return m_network; }
    const SystemLogConfig &systemLog() const { return m_systemLog; }

    static QString resolvePath(const QString &path);

private:
    AppConfig() = default;

    QString m_configFilePath;
    QString m_mapsDir = QStringLiteral("maps");
    QString m_devicesFile = QStringLiteral("devices.json");
    QString m_calibrationFile = QStringLiteral("calibration.json");
    SlamAppConfig m_slam;
    PcapPlaybackConfig m_pcapPlayback;
    ComputeNodeSchedulerAppConfig m_computeNodeScheduler;
    NetworkReceiveConfig m_network;
    SystemLogConfig m_systemLog;
};

#endif  // APPCONFIG_H

#ifndef SLAMMAPDISPLAYBUILDER_H
#define SLAMMAPDISPLAYBUILDER_H

#include "SlamMapStitcher.h"

#include <QVector>

namespace slam_map_display {

struct BuildStats {
    quint64 source_points = 0;
    quint64 sampled_points = 0;
    quint64 duplicate_points_removed = 0;
    quint64 output_points = 0;
};

struct PointCountStats {
    quint64 source_points = 0;
    quint64 valid_world_points = 0;
    quint64 resolution_points = 0;
};

struct StableRegion {
    quint64 stable_cells = 0;
    quint64 point_count = 0;
    quint64 observed_keyframes = 0;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    double estimated_area_m2 = 0.0;
    double average_observations = 0.0;
    int min_observations = 0;
    int max_observations = 0;
};

struct StableRegionAnalysis {
    double cell_size_m = 0.0;
    int min_observations = 0;
    int min_region_cells = 0;
    quint64 candidate_cells = 0;
    quint64 stable_cells = 0;
    quint64 discarded_small_regions = 0;
    QVector<StableRegion> regions;
};

// 统计完整地图在指定 XYZ 体素分辨率下的点数，不受界面显示抽样上限影响。
PointCountStats countPointsAtResolution(
    const slam_map_io::MapArchive &archive,
    double resolution_m);

// 只读分析跨关键帧反复出现的 XY 网格，并把相邻网格聚合为稳定区域。
// 该函数不修改归档内容，也不参与地图拼接或 ICP。
StableRegionAnalysis analyzeStableRegions(
    const slam_map_io::MapArchive &archive,
    double cell_size_m = 1.0,
    int min_observations = 2,
    int min_region_cells = 5);

QVector<M_PointXYZI> rebuild(
    const slam_map_io::MapArchive &archive,
    const QVector<slam_map_stitch::SourceRange> &ranges,
    int max_points,
    double voxel_size_m = 0.3,
    BuildStats *stats = nullptr);

}  // namespace slam_map_display

#endif  // SLAMMAPDISPLAYBUILDER_H

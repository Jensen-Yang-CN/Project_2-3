#ifndef SLAMMAPSTITCHER_H
#define SLAMMAPSTITCHER_H

#include "SlamMapBinaryIO.h"

#include <QString>
#include <QVector>

#include <Eigen/Core>

#include <cstdint>

namespace slam_map_stitch {

enum class AlignmentMode {
    Base,
    AnchorOnly,
    AnchorAndIcp,
    TemporalSeam,
    DuplicateSkipped
};

struct Options {
    // 地理锚点先把独立地图放到统一 ENU；无时间重叠时，再通过严格
    // 质量门控的 ICP 消除各次独立 SLAM 的剩余平移误差。ENU 轴向
    // 默认不可改变，刚性偏航修正只用于显式诊断。
    bool enable_icp = true;
    bool allow_yaw_correction = false;
    double voxel_size_m = 0.5;
    double coarse_voxel_size_m = 1.5;
    double overlap_padding_m = 3.0;
    int min_overlap_points = 200;
    int max_registration_points = 50000;
    // 优先使用被不同关键帧重复观测的二维网格，降低水面、运动物体和
    // 单帧离群点对配准的干扰；稳定点不足时自动回退到全部地图点。
    double stable_voxel_size_m = 1.0;
    int min_stable_observations = 2;
    int min_stable_points = 200;
    // ICP 之前围绕 ENU 结果搜索一个受限二维初值。最终修正仍由下方
    // 平移/偏航上限和双向质量指标共同约束。
    int max_initial_search_points = 2000;
    int max_initial_search_finalists = 24;
    int max_initial_search_refinements = 8;
    double initial_search_translation_step_m = 1.0;
    double initial_search_yaw_step_deg = 3.0;
    double initial_search_max_yaw_deg = 6.0;
    double initial_search_max_distance_m = 5.0;
    int max_iterations = 40;
    int coarse_max_iterations = 30;
    double max_correspondence_m = 1.5;
    double coarse_max_correspondence_m = 4.0;
    double max_correction_translation_m = 3.0;
    double correction_boundary_margin_m = 0.5;
    double max_correction_yaw_deg = 8.0;
    double quality_max_distance_m = 2.0;
    double min_bidirectional_match_ratio = 0.20;
    double max_robust_rmse_m = 1.25;
    double min_absolute_quality_improvement = 0.10;
    double min_relative_quality_improvement = 0.10;
    // 时间重叠默认只用于删除重复关键帧，不再估计额外刚体修正。
    bool deduplicate_temporal_overlap = true;
    bool enable_temporal_seam = false;
    double min_temporal_overlap_s = 5.0;
    double temporal_seam_window_s = 15.0;
    int min_temporal_pose_matches = 5;
    double max_temporal_median_error_m = 0.75;
    double max_temporal_p90_error_m = 1.5;
    double max_temporal_correction_translation_m = 30.0;
    double max_temporal_correction_yaw_deg = 10.0;
};

struct Input {
    QString source_path;
    slam_map_io::MapArchive archive;
};

struct SourceRange {
    int first_keyframe = 0;
    int keyframe_count = 0;
    uint8_t display_intensity = 90;
};

struct Diagnostic {
    QString source_path;
    AlignmentMode mode = AlignmentMode::Base;
    Eigen::Vector3d anchor_offset_enu = Eigen::Vector3d::Zero();
    int source_overlap_points = 0;
    int target_overlap_points = 0;
    int source_registration_points = 0;
    int target_registration_points = 0;
    bool source_used_stable_points = false;
    bool target_used_stable_points = false;
    bool icp_converged = false;
    double fitness_m2 = 0.0;
    double correction_translation_m = 0.0;
    double correction_yaw_deg = 0.0;
    double actual_overlap_area_m2 = 0.0;
    double anchor_robust_rmse_m = 0.0;
    double final_robust_rmse_m = 0.0;
    double anchor_quality_score = 0.0;
    double final_quality_score = 0.0;
    double source_match_ratio = 0.0;
    double target_match_ratio = 0.0;
    double quality_improvement_ratio = 0.0;
    double temporal_overlap_s = 0.0;
    int temporal_pose_matches = 0;
    double temporal_median_error_m = 0.0;
    double temporal_p90_error_m = 0.0;
    int trimmed_keyframes = 0;
    int appended_keyframes = 0;
    QString message;
};

struct Result {
    bool success = false;
    QString error;
    slam_map_io::MapArchive archive;
    QVector<SourceRange> source_ranges;
    QVector<Diagnostic> diagnostics;
};

Result stitch(const QVector<Input> &inputs,
              const Options &options = Options{});

}  // namespace slam_map_stitch

#endif  // SLAMMAPSTITCHER_H

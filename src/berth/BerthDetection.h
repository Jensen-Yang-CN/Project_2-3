#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "common_types_extended.h"
#include "BerthOpeningRules.h"
#include "BerthRectGeometry.h"
#include "BerthRecoveryRules.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <deque>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

// ====== 鍔犱笂杩欎竴娈甸槻鑼?MSVC 鎵句笉鍒?M_PI 鐨勯棶棰?======
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// =====================================================


namespace usv {

    // =========================================================
    // 娉婁綅妫€娴嬫牳蹇冪畻娉曞疄鐜?(BerthDetector)
    // =========================================================
    class BerthDetector {
    private:
        // 绠楁硶閰嶇疆鍙傛暟
        double z_min_ = -4.0;
        double z_max_ = 10.0;
        double roi_x_min_ = -80.0;
        double roi_x_max_ = 80.0;
        double roi_y_min_ = -80.0;
        double roi_y_max_ = 80.0;
        double bev_res_ = 0.2;
        double berth_width_min_ = 7.0;
        double berth_width_max_ = 12.0;
        double berth_length_min_ = 8.0;
        double berth_length_max_ = 25.0;
        double berth_area_min_ = 56.0;
        double line_roi_margin_ = 4.0;
        double line_roi_bev_res_ = 0.2;
        int line_roi_min_points_ = 25;
        int line_roi_hough_threshold_ = 16;
        double line_roi_min_line_length_ = 2.5;
        double line_roi_max_line_gap_ = 1.2;
        double line_roi_axis_tol_deg_ = 22.0;
        double line_roi_center_shift_max_ = 5.0;
        double opening_edge_count_band_m_ = 0.8;
        double edge_count_strip_length_ratio_ = 0.7;
        double opening_ray_length_m_ = 20.0;
        double opening_ray_half_width_m_ = 1.0;
        int opening_ray_min_hits_ = 1;
        int opening_edge_lock_frames_ = 3;
        double opening_edge_direction_stable_deg_ = 20.0;
        double edge_height_bin_m_ = 0.5;
        double edge_height_outlier_m_ = 0.6;
        int edge_height_min_bins_ = 3;
        double top_region_radius_ = 0.7;
        double top_region_min_separation_ = 1.5;
        double top_region_top_band_ = 0.35;
        double anchor_min_height_m_ = 2.5;
        double top_region_jump_threshold_ = 3.0;
        int top_region_jump_confirm_frames_ = 3;
        double anchor_bus_angle_tol_deg_ = 8.0;
        double anchor_bus_distance_tol_m_ = 1.5;
        double anchor_bus_projection_gap_m_ = 2.0;
        double anchor_recovery_local_window_ = 4.0;
        double anchor_recovery_roi_extra_margin_ = 1.0;
        double anchor_recovery_min_size_ = 6.0;
        // 锚点恢复后的首个正常候选需保持方向连续，避免形态学候选单帧转向。
        double recovered_to_detect_angle_jump_deg_ = 5.0;
        double recovered_to_detect_size_jump_m_ = 3.5;
        double detected_width_update_alpha_ = 0.2;
        double detected_length_update_alpha_ = 0.1;
        double detected_size_update_max_jump_m_ = 2.0;
        double tracked_size_lock_min_m_ = 9.0;
        double tracked_size_lock_max_m_ = 11.0;
        double recovered_expand_step_m_ = 0.2;
        double recovered_expand_max_m_ = 1.2;
        int recovered_expand_min_points_ = 5;
        double expand_area_jump_max_ratio_ = 1.2;
        double recovered_expand_area_jump_max_ratio_ = 1.2;
        double expand_history_area_jump_max_ratio_ = 1.2;
        double pre_expand_length_shrink_trigger_m_ = 11.0;
        double pre_expand_length_shrink_target_m_ = 9.0;
        int size_smooth_history_frames_ = 10;
        double size_smooth_max_ratio_ = 1.1;
        int no_berth_reset_frames_ = 10;
        int max_berth_candidates_ = 12;
        double multi_candidate_min_center_sep_ = 1.5;
        double duplicate_strict_center_sep_ = 1.0;
        double duplicate_iou_threshold_ = 0.35;
        double duplicate_angle_threshold_deg_ = 20.0;
        double abnormal_near_history_center_m_ = 8.0;
        double abnormal_near_history_iou_ = 0.2;
        cv::Mat last_morph_connected_debug_;
        cv::Mat last_morph_full_debug_;

        // 璺熻釜涓庤蹇嗙姸鎬?
        struct TopRegionState {
            Eigen::Vector3d stable_a{ Eigen::Vector3d::Zero() };
            Eigen::Vector3d stable_b{ Eigen::Vector3d::Zero() };
            Eigen::Vector3d pending_a{ Eigen::Vector3d::Zero() };
            Eigen::Vector3d pending_b{ Eigen::Vector3d::Zero() };
            bool has_stable = false;
            bool has_pending = false;
            int pending_count = 0;
        };

        struct AnchorBusState {
            bool valid = false;
            Eigen::Vector3d start{ Eigen::Vector3d::Zero() };
            Eigen::Vector3d end{ Eigen::Vector3d::Zero() };
            double ux = 1.0;
            double uy = 0.0;
        };

        struct OpeningEdgeState {
            int candidate_edge = -1;
            int consecutive_count = 0;
            int locked_edge = -1;
            Eigen::Vector2d candidate_direction{ Eigen::Vector2d::Zero() };
            Eigen::Vector2d locked_direction{ Eigen::Vector2d::Zero() };
            bool has_candidate_direction = false;
            bool locked = false;
        };

        std::vector<Berth> tracked_berths_;
        std::vector<bool> tracked_was_recovered_;
        std::vector<Berth> stable_history_berths_;
        double stable_history_anchor_margin_m_ =
            berth_recovery::kStableHistoryAnchorMarginM;
        std::vector<TopRegionState> top_region_states_;
        std::vector<OpeningEdgeState> opening_edge_states_;
        AnchorBusState anchor_bus_;
        bool anchor_bus_from_world_ = false;
        std::deque<std::vector<Berth>> output_size_history_;
        Berth roi_berth_;
        std::vector<DebugLine2D> last_debug_lines_;
        bool is_tracked_ = false;
        bool has_line_roi_ = false;
        bool is_anchor_recovered_ = false;
        int consecutive_no_berth_frames_ = 0;

        void clear_detection_memory() {
            is_tracked_ = false;
            has_line_roi_ = false;
            is_anchor_recovered_ = false;
            tracked_berths_.clear();
            tracked_was_recovered_.clear();
            top_region_states_.clear();
            opening_edge_states_.clear();
            anchor_bus_ = AnchorBusState{};
            anchor_bus_from_world_ = false;
            output_size_history_.clear();
        }

    public:
        void init() {
            clear_detection_memory();
            consecutive_no_berth_frames_ = 0;
        }

        void setWorldAnchorBusConstraint(const DebugLine2D& line) {
            const double dx = line.x2 - line.x1;
            const double dy = line.y2 - line.y1;
            const double length = std::hypot(dx, dy);
            if (!std::isfinite(length) || length <= 1e-3) {
                clearWorldAnchorBusConstraint();
                return;
            }
            anchor_bus_.valid = true;
            anchor_bus_.start = Eigen::Vector3d(line.x1, line.y1, 0.0);
            anchor_bus_.end = Eigen::Vector3d(line.x2, line.y2, 0.0);
            anchor_bus_.ux = dx / length;
            anchor_bus_.uy = dy / length;
            anchor_bus_from_world_ = true;
        }

        void setStableHistoricalBerths(const std::vector<Berth>& berths) {
            stable_history_berths_ = berths;
        }

        void clearStableHistoricalBerths() {
            stable_history_berths_.clear();
        }

        void setWorldProjectedRecoveryRois(const std::vector<Berth>& berths) {
            if (berths.empty()) {
                return;
            }

            std::vector<TopRegionState> next_top_states(berths.size());
            std::vector<OpeningEdgeState> next_opening_states(berths.size());
            std::vector<bool> next_recovered(berths.size(), false);
            std::vector<bool> used_tracks(tracked_berths_.size(), false);

            auto transform_point_between_rois = [](const Eigen::Vector3d& point,
                                                    const Berth& from,
                                                    const Berth& to) {
                const double from_rad = from.angle * M_PI / 180.0;
                const double to_rad = to.angle * M_PI / 180.0;
                const double dx = point.x() - from.cx;
                const double dy = point.y() - from.cy;
                const double local_x = dx * std::cos(from_rad)
                    + dy * std::sin(from_rad);
                const double local_y = -dx * std::sin(from_rad)
                    + dy * std::cos(from_rad);
                return Eigen::Vector3d(
                    local_x * std::cos(to_rad)
                        - local_y * std::sin(to_rad) + to.cx,
                    local_x * std::sin(to_rad)
                        + local_y * std::cos(to_rad) + to.cy,
                    point.z());
            };

            for (size_t i = 0; i < berths.size(); ++i) {
                size_t best_idx = tracked_berths_.size();
                double best_score = std::numeric_limits<double>::infinity();
                for (size_t j = 0; j < tracked_berths_.size(); ++j) {
                    if (used_tracks[j]) {
                        continue;
                    }
                    const double distance = std::hypot(
                        berths[i].cx - tracked_berths_[j].cx,
                        berths[i].cy - tracked_berths_[j].cy);
                    const double angle_diff = angle_distance_180(
                        berths[i].angle, tracked_berths_[j].angle);
                    const double size_diff =
                        std::abs(berths[i].w - tracked_berths_[j].w)
                        + std::abs(berths[i].l - tracked_berths_[j].l);
                    const double score = distance + 0.05 * angle_diff
                        + 0.25 * size_diff;
                    if (distance < 20.0 && angle_diff < 45.0
                        && score < best_score) {
                        best_score = score;
                        best_idx = j;
                    }
                }

                if (best_idx >= tracked_berths_.size()) {
                    continue;
                }
                used_tracks[best_idx] = true;
                if (best_idx < tracked_was_recovered_.size()) {
                    next_recovered[i] = tracked_was_recovered_[best_idx];
                }
                if (best_idx < top_region_states_.size()) {
                    next_top_states[i] = top_region_states_[best_idx];
                    TopRegionState& state = next_top_states[i];
                    const Berth& old_roi = tracked_berths_[best_idx];
                    if (state.has_stable) {
                        state.stable_a = transform_point_between_rois(
                            state.stable_a, old_roi, berths[i]);
                        state.stable_b = transform_point_between_rois(
                            state.stable_b, old_roi, berths[i]);
                    }
                    if (state.has_pending) {
                        state.pending_a = transform_point_between_rois(
                            state.pending_a, old_roi, berths[i]);
                        state.pending_b = transform_point_between_rois(
                            state.pending_b, old_roi, berths[i]);
                    }
                }
                if (best_idx < opening_edge_states_.size()) {
                    next_opening_states[i] = opening_edge_states_[best_idx];
                    double delta_deg =
                        berths[i].angle - tracked_berths_[best_idx].angle;
                    while (delta_deg > 90.0) delta_deg -= 180.0;
                    while (delta_deg < -90.0) delta_deg += 180.0;
                    const double delta = delta_deg * M_PI / 180.0;
                    const Eigen::Rotation2Dd rotation(delta);
                    next_opening_states[i].candidate_direction =
                        rotation
                        * next_opening_states[i].candidate_direction;
                    next_opening_states[i].locked_direction =
                        rotation * next_opening_states[i].locked_direction;
                }
            }

            tracked_berths_ = berths;
            tracked_was_recovered_ = std::move(next_recovered);
            top_region_states_ = std::move(next_top_states);
            opening_edge_states_ = std::move(next_opening_states);
            roi_berth_ = tracked_berths_.front();
            has_line_roi_ = true;
            is_tracked_ = true;
        }

        bool hasRecoveryRoiMemory() const {
            return has_line_roi_ || !tracked_berths_.empty();
        }

        void clearWorldAnchorBusConstraint() {
            if (anchor_bus_from_world_) {
                anchor_bus_ = AnchorBusState{};
            }
            anchor_bus_from_world_ = false;
        }

        void setLineRoiMargin(double margin) {
            if (std::isfinite(margin) && margin >= 0.0) {
                line_roi_margin_ = margin;
            }
        }

        BerthMeasureResult process(const LidarFrame& lidar) {
            BerthMeasureResult result;
            result.timestamp = lidar.timestamp;
            result.is_detected = false;
            result.is_using_memory = false;
            result.has_debug_roi = false;
            result.is_line_recovered = false;
            result.debug_lines.clear();
            result.anchor_debug = AnchorRecoveryDebug{};
            last_debug_lines_.clear();

            std::vector<Eigen::Vector3d> fused_points;
            fused_points.reserve(lidar.points.size());
            for (const auto& pt : lidar.points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    fused_points.emplace_back(pt.x, pt.y, pt.z);
                }
            }

            std::vector<Berth> detected_berths = process_marina_berths(fused_points);
            bool found = !detected_berths.empty();

            std::vector<Berth> final_berths;
            std::vector<bool> final_is_recovered;
            std::vector<bool> final_use_normal_expand;
            bool is_line_recovered = false;
            is_anchor_recovered_ = false;

            auto append_single_anchor_recovery = [&](const Berth& size_reference,
                size_t track_index, const char* context,
                const Berth* search_roi_reference = nullptr) {
                Berth recovered;
                Eigen::Vector3d single_anchor = Eigen::Vector3d::Zero();
                int anchor_candidate_count = 0;
                std::string rejection_reason;
                if (!recover_detected_berth_from_single_anchor(
                        fused_points, size_reference, recovered, single_anchor,
                        &anchor_candidate_count, search_roi_reference,
                        &rejection_reason)) {
                    if (!rejection_reason.empty()) {
                        result.debug_messages.push_back(
                            std::string("[BerthDebug] reject continuous single-anchor recovery context=")
                            + context
                            + " track=#" + std::to_string(track_index + 1)
                            + " reason=" + rejection_reason);
                    }
                    return false;
                }

                final_berths.push_back(recovered);
                final_is_recovered.push_back(true);
                final_use_normal_expand.push_back(true);
                is_anchor_recovered_ = true;
                is_line_recovered = true;
                result.highest_points.push_back(single_anchor);
                result.debug_messages.push_back(
                    std::string("[BerthDebug] continuous single-anchor recovery context=")
                    + context
                    + " track=#" + std::to_string(track_index + 1)
                    + " anchor=" + format_point_xy(single_anchor)
                    + " raw_candidates=" + std::to_string(anchor_candidate_count)
                    + " reference{" + format_berth(size_reference) + "}"
                    + " recovered{" + format_berth(recovered) + "}");
                return true;
            };

            if (found) {
                is_tracked_ = true;
                final_berths = std::move(detected_berths);
                final_is_recovered.assign(final_berths.size(), false);
                final_use_normal_expand.assign(final_berths.size(), false);

                if (!tracked_berths_.empty()) {
                    if (top_region_states_.size() < tracked_berths_.size()) {
                        top_region_states_.resize(tracked_berths_.size());
                    }

                    for (size_t i = 0; i < tracked_berths_.size(); ++i) {
                        const size_t matched_idx = find_matching_berth_index(
                            tracked_berths_[i], final_berths);
                        if (matched_idx < final_berths.size()) {
                            const bool previous_was_recovered =
                                i < tracked_was_recovered_.size() && tracked_was_recovered_[i];
                            if (!previous_was_recovered
                                || !normal_detection_jumps_after_recovery(
                                    tracked_berths_[i], final_berths[matched_idx])) {
                                continue;
                            }

                            const Berth detected_before_recovery = final_berths[matched_idx];
                            Berth recovered;
                            Eigen::Vector3d anchor_a = Eigen::Vector3d::Zero();
                            Eigen::Vector3d anchor_b = Eigen::Vector3d::Zero();
                            AnchorRecoveryDebug debug;
                            if (recover_berth_from_highest_anchors(
                                    fused_points, tracked_berths_[i], top_region_states_[i],
                                    recovered, anchor_a, anchor_b, debug)) {
                                is_anchor_recovered_ = true;
                                is_line_recovered = true;
                                final_berths[matched_idx] = recovered;
                                final_is_recovered[matched_idx] = true;
                                result.highest_points.push_back(anchor_a);
                                result.second_highest_points.push_back(anchor_b);
                                result.debug_messages.push_back(
                                    "[泊位调试] 上帧为锚点恢复，本帧正常检测尺寸/方向跳变，改用锚点恢复 track=#"
                                    + std::to_string(i + 1)
                                    + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                    + " detected{" + format_berth(detected_before_recovery) + "}"
                                    + " recovered{" + format_berth(recovered) + "}"
                                    + " anchorA=" + format_point_xy(anchor_a)
                                    + " anchorB=" + format_point_xy(anchor_b)
                                    + " " + format_recovery_debug(debug));
                                if (result.anchor_debug.status != AnchorRecoveryStatus::Success) {
                                    result.anchor_debug = debug;
                                }
                            } else {
                                result.debug_messages.push_back(
                                    "[泊位调试] 上帧为锚点恢复，本帧正常检测尺寸/方向跳变，但锚点恢复失败，保留正常检测 track=#"
                                    + std::to_string(i + 1)
                                    + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                    + " detected{" + format_berth(detected_before_recovery) + "}"
                                    + " " + format_recovery_debug(debug));
                            }
                            continue;
                        }

                        Berth recovered;
                        Eigen::Vector3d anchor_a = Eigen::Vector3d::Zero();
                        Eigen::Vector3d anchor_b = Eigen::Vector3d::Zero();
                        AnchorRecoveryDebug debug;
                        if (recover_berth_from_highest_anchors(
                                fused_points, tracked_berths_[i], top_region_states_[i],
                                recovered, anchor_a, anchor_b, debug)) {
                            is_anchor_recovered_ = true;
                            is_line_recovered = true;
                            final_berths.push_back(recovered);
                            final_is_recovered.push_back(true);
                            final_use_normal_expand.push_back(false);
                            result.highest_points.push_back(anchor_a);
                            result.second_highest_points.push_back(anchor_b);
                            result.debug_messages.push_back(
                                "[泊位调试] 历史泊位未被本帧候选匹配，使用锚点恢复成功 track=#"
                                + std::to_string(i + 1)
                                + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                + " recovered{" + format_berth(recovered) + "}"
                                + " anchorA=" + format_point_xy(anchor_a)
                                + " anchorB=" + format_point_xy(anchor_b)
                                + " " + format_recovery_debug(debug));
                            if (result.anchor_debug.status != AnchorRecoveryStatus::Success) {
                                result.anchor_debug = debug;
                            }
                        } else if (!append_single_anchor_recovery(
                                       tracked_berths_[i], i, "unmatched-history")) {
                            result.debug_messages.push_back(
                                "[泊位调试] 历史泊位未被本帧候选匹配，锚点恢复失败 track=#"
                                + std::to_string(i + 1)
                                + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                + " " + format_recovery_debug(debug));
                        }
                    }
                }
            }
            else {
                if (tracked_berths_.empty()) {
                    result.anchor_debug.status = AnchorRecoveryStatus::NoTrackedBerth;
                }
                else {
                    if (top_region_states_.size() < tracked_berths_.size()) {
                        top_region_states_.resize(tracked_berths_.size());
                    }

                    for (size_t i = 0; i < tracked_berths_.size(); ++i) {
                        Berth recovered;
                        Eigen::Vector3d anchor_a = Eigen::Vector3d::Zero();
                        Eigen::Vector3d anchor_b = Eigen::Vector3d::Zero();
                        AnchorRecoveryDebug debug;
                        if (recover_berth_from_highest_anchors(
                                fused_points, tracked_berths_[i], top_region_states_[i],
                                recovered, anchor_a, anchor_b, debug)) {
                            is_anchor_recovered_ = true;
                            is_line_recovered = true;
                            final_berths.push_back(recovered);
                            final_is_recovered.push_back(true);
                            final_use_normal_expand.push_back(false);
                            result.highest_points.push_back(anchor_a);
                            result.second_highest_points.push_back(anchor_b);
                            result.debug_messages.push_back(
                                "[泊位调试] 本帧无形态学泊位候选，使用历史泊位锚点恢复成功 track=#"
                                + std::to_string(i + 1)
                                + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                + " recovered{" + format_berth(recovered) + "}"
                                + " anchorA=" + format_point_xy(anchor_a)
                                + " anchorB=" + format_point_xy(anchor_b)
                                + " " + format_recovery_debug(debug));
                            if (result.anchor_debug.status != AnchorRecoveryStatus::Success) {
                                result.anchor_debug = debug;
                            }
                        } else if (append_single_anchor_recovery(
                                       tracked_berths_[i], i, "no-normal-candidate")) {
                            continue;
                        } else if (result.anchor_debug.status == AnchorRecoveryStatus::NotAttempted) {
                            result.anchor_debug = debug;
                            result.debug_messages.push_back(
                                "[泊位调试] 本帧无形态学泊位候选，历史泊位锚点恢复失败 track=#"
                                + std::to_string(i + 1)
                                + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                + " " + format_recovery_debug(debug));
                        } else {
                            result.debug_messages.push_back(
                                "[泊位调试] 本帧无形态学泊位候选，历史泊位锚点恢复失败 track=#"
                                + std::to_string(i + 1)
                                + " tracked{" + format_berth(tracked_berths_[i]) + "}"
                                + " " + format_recovery_debug(debug));
                        }
                    }
                }
            }

            for (size_t i = 0; i < final_berths.size(); ++i) {
                if (i < final_is_recovered.size() && final_is_recovered[i]) {
                    continue;
                }
                const Berth detected_before_recovery = final_berths[i];
                Berth recovered;
                Eigen::Vector3d single_anchor = Eigen::Vector3d::Zero();
                int anchor_candidate_count = 0;
                std::string rejection_reason;
                if (!recover_detected_berth_from_single_anchor(
                        fused_points, detected_before_recovery, recovered,
                        single_anchor, &anchor_candidate_count, nullptr,
                        &rejection_reason)) {
                    if (!rejection_reason.empty()) {
                        result.debug_messages.push_back(
                            "[BerthDebug] reject single-anchor recovery candidate=#"
                            + std::to_string(i + 1)
                            + " reason=" + rejection_reason);
                    }
                    continue;
                }

                final_berths[i] = recovered;
                final_is_recovered[i] = true;
                final_use_normal_expand[i] = true;
                is_anchor_recovered_ = true;
                is_line_recovered = true;
                result.highest_points.push_back(single_anchor);
                result.debug_messages.push_back(
                    "[BerthDebug] single-anchor recovery candidate=#"
                    + std::to_string(i + 1)
                    + " anchor=" + format_point_xy(single_anchor)
                    + " raw_candidates=" + std::to_string(anchor_candidate_count)
                    + " detected{" + format_berth(detected_before_recovery) + "}"
                    + " recovered{" + format_berth(recovered) + "}");
            }

            suppress_abnormal_near_history_candidates(
                final_berths, final_is_recovered, result.debug_messages,
                &final_use_normal_expand);

            std::vector<Berth> expanded_berths;
            std::vector<bool> expanded_is_recovered;
            for (size_t i = 0; i < final_berths.size(); ++i) {
                Berth b = final_berths[i];
                const bool was_recovered =
                    i < final_is_recovered.size() && final_is_recovered[i];
                const bool use_normal_expand =
                    i < final_use_normal_expand.size() && final_use_normal_expand[i];
                if (std::isfinite(b.l) &&
                    pre_expand_length_shrink_trigger_m_ > pre_expand_length_shrink_target_m_ &&
                    pre_expand_length_shrink_target_m_ >= berth_length_min_ &&
                    b.l > pre_expand_length_shrink_trigger_m_) {
                    const Berth before_shrink = b;
                    b.l = pre_expand_length_shrink_target_m_;
                    result.debug_messages.push_back(
                        "[BerthDebug] berth length larger than pre-expand limit, shrink before expand candidate=#"
                        + std::to_string(i + 1)
                        + " recovered=" + std::string(was_recovered ? "true" : "false")
                        + " before{" + format_berth(before_shrink) + "}"
                        + " after{" + format_berth(b) + "}");
                }
                if (was_recovered) {
                    const Berth before_expand = b;
                    if (use_normal_expand) {
                        b = expand_berth_to_points(b, fused_points);
                    } else {
                        b = expand_berth_to_points(
                            b,
                            fused_points,
                            recovered_expand_step_m_,
                            recovered_expand_max_m_,
                            recovered_expand_min_points_);
                    }
                    if (!is_valid_berth_size(b)) {
                        result.debug_messages.push_back(
                            "[BerthDebug] recovered berth invalid after expand, keep historical-size recovery candidate=#"
                            + std::to_string(i + 1)
                            + " before{" + format_berth(before_expand) + "}"
                            + " after{" + format_berth(b) + "}");
                        b = before_expand;
                    }
                    else {
                        const Berth before_align = b;
                        b = align_recovered_expansion_to_anchor_edge(before_expand, b);
                        if (std::abs(b.cx - before_align.cx) > 1e-3 ||
                            std::abs(b.cy - before_align.cy) > 1e-3 ||
                            std::abs(angle_distance_180(b.angle, before_align.angle)) > 1e-3) {
                            result.debug_messages.push_back(
                                "[BerthDebug] align recovered expanded berth to anchor edge before{"
                                + format_berth(before_align)
                                + "} after{" + format_berth(b)
                                + "} anchor_edge_ref{" + format_berth(before_expand) + "}");
                        }
                        std::string reason;
                        if (is_expansion_area_jump(
                                before_expand, b, !use_normal_expand, reason)) {
                            result.debug_messages.push_back(
                                "[BerthDebug] recovered berth expand area jump, keep before-expand candidate=#"
                                + std::to_string(i + 1)
                                + " reason=" + reason
                                + " before{" + format_berth(before_expand) + "}"
                                + " after{" + format_berth(b) + "}");
                            b = before_expand;
                        }
                    }
                    expanded_berths.push_back(b);
                    expanded_is_recovered.push_back(true);
                }
                else {
                    const Berth before_expand = b;
                    b = expand_berth_to_points(b, fused_points);
                    if (is_valid_berth_size(b)) {
                        std::string reason;
                        if (is_expansion_area_jump(before_expand, b, false, reason)) {
                            result.debug_messages.push_back(
                                "[BerthDebug] detected berth expand area jump, keep before-expand candidate=#"
                                + std::to_string(i + 1)
                                + " reason=" + reason
                                + " before{" + format_berth(before_expand) + "}"
                                + " after{" + format_berth(b) + "}");
                            b = before_expand;
                        }
                        expanded_berths.push_back(b);
                        expanded_is_recovered.push_back(false);
                    } else {
                        result.debug_messages.push_back(
                            "[泊位调试] 候选扩边后尺寸不合法，丢弃 candidate=#"
                            + std::to_string(i + 1)
                            + " before{" + format_berth(before_expand) + "}"
                            + " after{" + format_berth(b) + "}");
                    }
                }
            }
            apply_final_berth_nms(expanded_berths, expanded_is_recovered, result.debug_messages);
            stabilize_output_berth_sizes(expanded_berths, expanded_is_recovered, result.debug_messages);
            smooth_output_berth_sizes_by_history(
                expanded_berths, expanded_is_recovered, result.debug_messages);
            reorder_berths_by_tracked_order(
                expanded_berths, expanded_is_recovered, result.debug_messages);
            std::vector<int> opening_edges_before_post_process(
                expanded_berths.size(), -1);
            for (size_t i = 0; i < expanded_berths.size(); ++i) {
                Berth& berth = expanded_berths[i];
                berth.edge_point_counts = count_edge_points(berth, fused_points);
                const int old_opening_edge = berth.opening_edge;
                const bool was_recovered =
                    i < expanded_is_recovered.size() && expanded_is_recovered[i];
                const std::array<int, 4> ray_hits =
                    count_opening_ray_hits(berth, fused_points);
                berth.opening_edge = berth_opening::selectOpeningEdge(
                    ray_hits, berth.edge_point_counts, was_recovered,
                    old_opening_edge, find_nearest_opening_edge(berth),
                    opening_ray_min_hits_);
                if (was_recovered
                    && (old_opening_edge < 0 || old_opening_edge >= 4)) {
                    // 锚点恢复的连接边就是开口；仅在恢复未提供有效边时回退。
                    berth.opening_edge = find_nearest_opening_edge(berth);
                }
                opening_edges_before_post_process[i] = berth.opening_edge;
                int missing_count = 0;
                for (int edge = 0; edge < 4; ++edge) {
                    if (ray_hits[edge]
                        < std::max(1, opening_ray_min_hits_)) {
                        ++missing_count;
                    }
                }
                const std::string opening_source = !was_recovered
                    ? (missing_count > 0
                           ? "normal-ray-miss"
                           : "normal-edge-count-fallback")
                    : (missing_count == 1
                           ? "recovered-unique-ray-miss"
                           : ((old_opening_edge < 0
                               || old_opening_edge >= 4)
                                  ? "recovered-invalid-anchor-nearest-edge"
                                  : "recovered-anchor-edge"));
                result.debug_messages.push_back(
                    "[BerthOpening] initial berth=" + std::to_string(i + 1)
                    + " recovered=" + std::to_string(was_recovered ? 1 : 0)
                    + " source=" + opening_source
                    + " old_edge=" + std::to_string(old_opening_edge)
                    + " selected_edge=" + std::to_string(berth.opening_edge)
                    + " missing_count=" + std::to_string(missing_count)
                    + " rayRegion=20x2-BEV"
                    + " rayHits=e0:" + std::to_string(ray_hits[0])
                    + " e1:" + std::to_string(ray_hits[1])
                    + " e2:" + std::to_string(ray_hits[2])
                    + " e3:" + std::to_string(ray_hits[3]));
                if (old_opening_edge != berth.opening_edge) {
                    result.debug_messages.push_back(
                        std::string(was_recovered
                            ? "[BerthDebug] recovered opening changed by unique ray miss from "
                            : "[BerthDebug] final opening edge updated from ")
                        + std::to_string(old_opening_edge)
                        + " to " + std::to_string(berth.opening_edge)
                        + " rayHits=e0:" + std::to_string(ray_hits[0])
                        + " e1:" + std::to_string(ray_hits[1])
                        + " e2:" + std::to_string(ray_hits[2])
                        + " e3:" + std::to_string(ray_hits[3])
                        + " counts=e0:" + std::to_string(berth.edge_point_counts[0])
                        + " e1:" + std::to_string(berth.edge_point_counts[1])
                        + " e2:" + std::to_string(berth.edge_point_counts[2])
                        + " e3:" + std::to_string(berth.edge_point_counts[3]));
                }
            }
            stabilize_output_opening_edges(expanded_berths, result.debug_messages);
            apply_anchor_bus_opening_edges(expanded_berths, result.debug_messages);
            for (size_t i = 0; i < expanded_berths.size(); ++i) {
                Berth& berth = expanded_berths[i];
                result.debug_messages.push_back(
                    "[BerthOpening] final berth=" + std::to_string(i + 1)
                    + " initial_edge="
                    + std::to_string(opening_edges_before_post_process[i])
                    + " final_edge=" + std::to_string(berth.opening_edge));
                compute_ship_relative_metrics(berth);
                compute_edge_heights(berth, fused_points);
                update_berth_display_edges(berth);
            }

            if (expanded_berths.empty() && !final_berths.empty()) {
                result.debug_messages.push_back(
                    "[泊位调试] 本帧有候选/恢复结果，但全部在扩边或尺寸检查后被丢弃 final_count="
                    + std::to_string(final_berths.size()));
            }

            if (!expanded_berths.empty()) {
                // 先收集本帧锚点证据，再发布正常候选。
                for (size_t i = 0; i < expanded_berths.size(); ++i) {
                    const Berth& berth = expanded_berths[i];
                    Berth debug_roi = expanded_line_roi(berth);
                    const Berth& top_search_roi = debug_roi;
                    result.debug_rois.push_back(debug_roi);
                    result.top_search_rois.push_back(top_search_roi);

                    Eigen::Vector3d raw_top_a = Eigen::Vector3d::Zero();
                    Eigen::Vector3d raw_top_b = Eigen::Vector3d::Zero();
                    const int top_region_count = find_highest_regions_in_roi(
                        top_search_roi, fused_points, raw_top_a, raw_top_b);
                    if (top_region_count > 0) {
                        const bool was_recovered =
                            i < expanded_is_recovered.size() && expanded_is_recovered[i];
                        if (!was_recovered) {
                            result.highest_points.push_back(raw_top_a);
                            if (top_region_count > 1) {
                                result.second_highest_points.push_back(raw_top_b);
                            }
                        }
                    }
                }

                if (!result.debug_rois.empty()) {
                    result.has_debug_roi = true;
                    result.debug_roi = result.debug_rois.front();
                }
                if (!result.top_search_rois.empty()) {
                    result.has_top_search_roi = true;
                    result.top_search_roi = result.top_search_rois.front();
                }
                if (!result.highest_points.empty()) {
                    result.has_highest_point = true;
                    result.has_second_highest_point = !result.second_highest_points.empty();
                    result.highest_point = result.highest_points.front();
                    if (!result.second_highest_points.empty())
                        result.second_highest_point = result.second_highest_points.front();
                }

                update_anchor_bus_from_pairs(
                    result.highest_points, result.second_highest_points,
                    result.debug_messages);

                // 本帧锚点对可能刚建立总线，立即重新约束本帧开口。
                apply_anchor_bus_opening_edges(
                    expanded_berths, result.debug_messages);
                for (Berth& berth : expanded_berths) {
                    compute_ship_relative_metrics(berth);
                    update_berth_display_edges(berth);
                }

                if (!expanded_berths.empty()) {
                    consecutive_no_berth_frames_ = 0;
                    result.is_detected = true;
                    result.is_using_memory = false;
                    result.is_line_recovered = is_line_recovered;
                    for (size_t i = 0; i < expanded_berths.size(); ++i) {
                        expanded_berths[i].is_anchor_recovered =
                            i < expanded_is_recovered.size()
                            && expanded_is_recovered[i];
                    }
                    result.expanded_berths = expanded_berths;
                    update_tracked_berths(
                        expanded_berths, expanded_is_recovered);
                    append_output_size_history(expanded_berths);
                    update_line_roi(expanded_berths.front());
                }
            }
            if (expanded_berths.empty()) {
                ++consecutive_no_berth_frames_;
                if (consecutive_no_berth_frames_ >= std::max(1, no_berth_reset_frames_)) {
                    result.debug_messages.push_back(
                        "[BerthDebug] no berth output for "
                        + std::to_string(consecutive_no_berth_frames_)
                        + " consecutive frames, clear tracked berths, ROI, anchor bus and size history");
                    clear_detection_memory();
                    consecutive_no_berth_frames_ = 0;
                }
            }

            if (!result.has_debug_roi && has_line_roi_) {
                result.has_debug_roi = true;
                result.debug_roi = expanded_line_roi(roi_berth_);
                result.debug_rois.push_back(result.debug_roi);
                result.has_top_search_roi = true;
                result.top_search_roi = result.debug_roi;
                result.top_search_rois.push_back(result.top_search_roi);
            }
            if (anchor_bus_.valid) {
                result.has_anchor_bus = true;
                result.anchor_bus_line = DebugLine2D{
                    anchor_bus_.start.x(), anchor_bus_.start.y(),
                    anchor_bus_.end.x(), anchor_bus_.end.y()};
            }
            result.debug_lines = last_debug_lines_;

            return result;
        }

    private:
        bool is_valid_berth_size(const Berth& b) const {
            if (!std::isfinite(b.w) || !std::isfinite(b.l)) return false;
            const double width = std::min(b.w, b.l);
            const double length = std::max(b.w, b.l);
            return width >= berth_width_min_ && width <= berth_width_max_ &&
                length >= berth_length_min_ && length <= berth_length_max_ &&
                width * length >= berth_area_min_;
        }

        const char* anchor_status_name(AnchorRecoveryStatus status) const {
            switch (status) {
            case AnchorRecoveryStatus::NotAttempted: return "NotAttempted";
            case AnchorRecoveryStatus::NoTrackedBerth: return "NoTrackedBerth";
            case AnchorRecoveryStatus::InvalidTrackedBerthSize: return "InvalidTrackedBerthSize";
            case AnchorRecoveryStatus::NoTopSearchCandidates: return "NoTopSearchCandidates";
            case AnchorRecoveryStatus::InvalidAnchorWidth: return "InvalidAnchorWidth";
            case AnchorRecoveryStatus::InvalidTrackedLength: return "InvalidTrackedLength";
            case AnchorRecoveryStatus::Success: return "Success";
            }
            return "Unknown";
        }

        std::string format_berth(const Berth& b) const {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "center=(" << b.cx << "," << b.cy << ")"
                << " size=" << b.w << "x" << b.l
                << " angle=" << std::setprecision(1) << b.angle
                << " edge=" << b.opening_edge;
            return oss.str();
        }

        std::string format_point_xy(const Eigen::Vector3d& p) const {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "(" << p.x() << "," << p.y() << "," << p.z() << ")";
            return oss.str();
        }

        std::string format_recovery_debug(const AnchorRecoveryDebug& debug) const {
            std::ostringstream oss;
            oss << "status=" << anchor_status_name(debug.status)
                << " candidates=" << debug.top_search_candidate_count
                << std::fixed << std::setprecision(2)
                << " anchor_dist=" << debug.anchor_distance
                << " tracked_size=" << debug.tracked_w << "x" << debug.tracked_l
                << " recovered_size=" << debug.recovered_w << "x" << debug.recovered_l
                << " recovered_angle=" << std::setprecision(1) << debug.recovered_angle;
            return oss.str();
        }

        DebugLine2D berth_edge_segment_world(const Berth& b, int edge) const {
            DebugLine2D line;
            if (!is_valid_berth_size(b) || edge < 0 || edge >= 4) {
                return line;
            }

            const double rad = b.angle * M_PI / 180.0;
            const double cos_a = std::cos(rad);
            const double sin_a = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;
            const double local_x[4] = {-half_w, half_w, half_w, -half_w};
            const double local_y[4] = {-half_l, -half_l, half_l, half_l};

            const int next = (edge + 1) % 4;
            line.x1 = b.cx + local_x[edge] * cos_a - local_y[edge] * sin_a;
            line.y1 = b.cy + local_x[edge] * sin_a + local_y[edge] * cos_a;
            line.x2 = b.cx + local_x[next] * cos_a - local_y[next] * sin_a;
            line.y2 = b.cy + local_x[next] * sin_a + local_y[next] * cos_a;
            return line;
        }

        int opening_edge_nearest_anchor_bus(const Berth& berth) const {
            if (!anchor_bus_.valid || !is_valid_berth_size(berth)) {
                return -1;
            }

            int nearest_edge = -1;
            double nearest_distance = std::numeric_limits<double>::infinity();
            for (int edge = 0; edge < 4; ++edge) {
                const DebugLine2D segment =
                    berth_edge_segment_world(berth, edge);
                const double midpoint_x = 0.5 * (segment.x1 + segment.x2);
                const double midpoint_y = 0.5 * (segment.y1 + segment.y2);
                const double dx = midpoint_x - anchor_bus_.start.x();
                const double dy = midpoint_y - anchor_bus_.start.y();
                const double distance = std::abs(
                    dx * (-anchor_bus_.uy) + dy * anchor_bus_.ux);
                if (distance < nearest_distance) {
                    nearest_distance = distance;
                    nearest_edge = edge;
                }
            }
            return nearest_edge;
        }

        void apply_anchor_bus_opening_edges(
            std::vector<Berth>& berths,
            std::vector<std::string>& debug_messages) const {
            if (!anchor_bus_.valid) {
                return;
            }

            for (Berth& berth : berths) {
                // The detector already decided the physical opening from the
                // BEV ray evidence. The anchor bus is only a position/heading
                // reference; replacing a valid edge with the nearest bus edge
                // flips every U-shape when the bus is the fixed wall side.
                const int opening_edge =
                    (berth.opening_edge >= 0 && berth.opening_edge < 4)
                    ? berth.opening_edge
                    : opening_edge_nearest_anchor_bus(berth);
                if (opening_edge < 0) {
                    continue;
                }
                if (berth.opening_edge == opening_edge)
                    continue;
                debug_messages.push_back(
                    "[BerthDebug] anchor bus opening edge="
                    + std::to_string(opening_edge)
                    + " updates opening edge from "
                    + std::to_string(berth.opening_edge)
                    + " to " + std::to_string(opening_edge));
                berth.opening_edge = opening_edge;
            }
        }

        void update_berth_display_edges(Berth& b) const {
            b.visible_edges.clear();
            b.has_opening_segment = false;
            if (!is_valid_berth_size(b) || b.opening_edge < 0 || b.opening_edge >= 4) {
                return;
            }

            b.opening_segment = berth_edge_segment_world(b, b.opening_edge);
            b.has_opening_segment = true;
            b.visible_edges.reserve(3);
            for (int edge = 0; edge < 4; ++edge) {
                if (edge == b.opening_edge) {
                    continue;
                }
                b.visible_edges.push_back(berth_edge_segment_world(b, edge));
            }
        }

        cv::Point bev_pixel(double x, double y) const {
            const int px = static_cast<int>(std::lround((x - roi_x_min_) / bev_res_));
            const int py = static_cast<int>(std::lround((y - roi_y_min_) / bev_res_));
            return cv::Point(px, py);
        }

        cv::Point debug_bev_pixel(double x, double y, double view_x_min, double view_y_min) const {
            const int px = static_cast<int>(std::lround((x - view_x_min) / bev_res_));
            const int py = static_cast<int>(std::lround((y - view_y_min) / bev_res_));
            return cv::Point(px, py);
        }

        void draw_berth_rect(cv::Mat& image, const Berth& b,
            const cv::Scalar& color, int thickness, bool skip_opening_edge) const {
            if (skip_opening_edge && !b.visible_edges.empty()) {
                for (const DebugLine2D& edge : b.visible_edges) {
                    cv::line(image,
                        bev_pixel(edge.x1, edge.y1),
                        bev_pixel(edge.x2, edge.y2),
                        color, thickness, cv::LINE_AA);
                }
                return;
            }

            const double rad = b.angle * M_PI / 180.0;
            const double cos_a = std::cos(rad);
            const double sin_a = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;
            const double local_x[4] = {-half_w, half_w, half_w, -half_w};
            const double local_y[4] = {-half_l, -half_l, half_l, half_l};
            cv::Point corners[4];

            for (int i = 0; i < 4; ++i) {
                const double x = b.cx + local_x[i] * cos_a - local_y[i] * sin_a;
                const double y = b.cy + local_x[i] * sin_a + local_y[i] * cos_a;
                corners[i] = bev_pixel(x, y);
            }

            int skip_edge = -1;
            if (skip_opening_edge) {
                double best_dist2 = std::numeric_limits<double>::infinity();
                for (int edge = 0; edge < 4; ++edge) {
                    const double mx = 0.5 * (corners[edge].x + corners[(edge + 1) % 4].x);
                    const double my = 0.5 * (corners[edge].y + corners[(edge + 1) % 4].y);
                    const double wx = mx * bev_res_ + roi_x_min_;
                    const double wy = my * bev_res_ + roi_y_min_;
                    const double dist2 = wx * wx + wy * wy;
                    if (dist2 < best_dist2) {
                        best_dist2 = dist2;
                        skip_edge = edge;
                    }
                }
            }

            for (int edge = 0; edge < 4; ++edge) {
                if (edge == skip_edge) continue;
                cv::line(image, corners[edge], corners[(edge + 1) % 4], color, thickness, cv::LINE_AA);
            }
        }

        void draw_anchor_marker(cv::Mat& image, const Eigen::Vector3d& p,
            const cv::Scalar& color, int radius) const {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y())) return;
            const cv::Point c = bev_pixel(p.x(), p.y());
            cv::line(image, cv::Point(c.x - radius, c.y), cv::Point(c.x + radius, c.y), color, 2, cv::LINE_AA);
            cv::line(image, cv::Point(c.x, c.y - radius), cv::Point(c.x, c.y + radius), color, 2, cv::LINE_AA);
            cv::circle(image, c, radius, color, 1, cv::LINE_AA);
        }

        void draw_debug_berth_rect(cv::Mat& image, const Berth& b,
            const cv::Scalar& color, int thickness, bool skip_opening_edge,
            double view_x_min, double view_y_min) const {
            if (skip_opening_edge && !b.visible_edges.empty()) {
                for (const DebugLine2D& edge : b.visible_edges) {
                    cv::line(image,
                        debug_bev_pixel(edge.x1, edge.y1, view_x_min, view_y_min),
                        debug_bev_pixel(edge.x2, edge.y2, view_x_min, view_y_min),
                        color, thickness, cv::LINE_AA);
                }
                return;
            }

            const double rad = b.angle * M_PI / 180.0;
            const double cos_a = std::cos(rad);
            const double sin_a = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;
            const double local_x[4] = {-half_w, half_w, half_w, -half_w};
            const double local_y[4] = {-half_l, -half_l, half_l, half_l};
            cv::Point corners[4];
            for (int i = 0; i < 4; ++i) {
                const double x = b.cx + local_x[i] * cos_a - local_y[i] * sin_a;
                const double y = b.cy + local_x[i] * sin_a + local_y[i] * cos_a;
                corners[i] = debug_bev_pixel(x, y, view_x_min, view_y_min);
            }

            int skip_edge = -1;
            if (skip_opening_edge) {
                double best_dist2 = std::numeric_limits<double>::infinity();
                for (int edge = 0; edge < 4; ++edge) {
                    const double mx = 0.5 * (corners[edge].x + corners[(edge + 1) % 4].x);
                    const double my = 0.5 * (corners[edge].y + corners[(edge + 1) % 4].y);
                    const double wx = mx * bev_res_ + view_x_min;
                    const double wy = my * bev_res_ + view_y_min;
                    const double dist2 = wx * wx + wy * wy;
                    if (dist2 < best_dist2) {
                        best_dist2 = dist2;
                        skip_edge = edge;
                    }
                }
            }

            for (int edge = 0; edge < 4; ++edge) {
                if (edge == skip_edge) continue;
                cv::line(image, corners[edge], corners[(edge + 1) % 4], color, thickness, cv::LINE_AA);
            }
        }

        void draw_debug_anchor_marker(cv::Mat& image, const Eigen::Vector3d& p,
            const cv::Scalar& color, int radius,
            double view_x_min, double view_y_min) const {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y())) return;
            const cv::Point c = debug_bev_pixel(p.x(), p.y(), view_x_min, view_y_min);
            cv::line(image, cv::Point(c.x - radius, c.y), cv::Point(c.x + radius, c.y), color, 2, cv::LINE_AA);
            cv::line(image, cv::Point(c.x, c.y - radius), cv::Point(c.x, c.y + radius), color, 2, cv::LINE_AA);
            cv::circle(image, c, radius, color, 1, cv::LINE_AA);
        }

        cv::Mat build_morph_connected_debug_image(const cv::Mat& bev_img,
            const cv::Mat& isolated,
            const std::vector<std::vector<cv::Point>>& contours) const {
            cv::Mat debug;
            cv::cvtColor(bev_img, debug, cv::COLOR_GRAY2BGR);
            debug *= 0.35;

            cv::Mat labels;
            const int component_count = cv::connectedComponents(isolated, labels, 8, CV_32S);
            for (int y = 0; y < labels.rows; ++y) {
                for (int x = 0; x < labels.cols; ++x) {
                    const int label = labels.at<int>(y, x);
                    if (label <= 0) {
                        continue;
                    }
                    const int hue = (label * 47) % 180;
                    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 200, 230));
                    cv::Mat bgr;
                    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
                    debug.at<cv::Vec3b>(y, x) = bgr.at<cv::Vec3b>(0, 0);
                }
            }

            cv::drawContours(debug, contours, -1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            cv::putText(debug,
                "U-shape morphology connected regions",
                cv::Point(8, 18),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
            cv::putText(debug,
                "color=connected component green=valid red=invalid",
                cv::Point(8, 36),
                cv::FONT_HERSHEY_SIMPLEX,
                0.42,
                cv::Scalar(230, 230, 230),
                1,
                cv::LINE_AA);
            (void)component_count;
            return debug;
        }

        cv::Mat build_morph_full_debug_image(const cv::Mat& bev_img,
            const cv::Mat& filtered_bev,
            const cv::Mat& dilated_bev,
            const cv::Mat& closed_comb,
            const cv::Mat& isolated) const {
            auto to_bgr = [](const cv::Mat& src, const cv::Scalar& tint) {
                cv::Mat gray;
                if (src.channels() == 1) {
                    gray = src;
                }
                else {
                    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
                }

                cv::Mat bgr(gray.size(), CV_8UC3, cv::Scalar(18, 18, 18));
                for (int y = 0; y < gray.rows; ++y) {
                    for (int x = 0; x < gray.cols; ++x) {
                        const uchar v = gray.at<uchar>(y, x);
                        if (v == 0) {
                            continue;
                        }
                        bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(
                            static_cast<uchar>(std::min(255.0, tint[0] * v / 255.0)),
                            static_cast<uchar>(std::min(255.0, tint[1] * v / 255.0)),
                            static_cast<uchar>(std::min(255.0, tint[2] * v / 255.0)));
                    }
                }
                return bgr;
            };

            auto label = [](cv::Mat& image, const std::string& text) {
                cv::rectangle(image, cv::Rect(0, 0, image.cols, 24), cv::Scalar(20, 20, 20), cv::FILLED);
                cv::putText(image, text, cv::Point(8, 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            };

            std::vector<cv::Mat> panels;
            panels.push_back(to_bgr(bev_img, cv::Scalar(180, 180, 180)));
            panels.push_back(to_bgr(filtered_bev, cv::Scalar(80, 180, 255)));
            panels.push_back(to_bgr(dilated_bev, cv::Scalar(80, 255, 120)));
            panels.push_back(to_bgr(closed_comb, cv::Scalar(255, 180, 80)));
            panels.push_back(to_bgr(isolated, cv::Scalar(255, 80, 255)));

            const char* names[] = {
                "raw BEV",
                "morph open",
                "dilated",
                "closed h/v combined",
                "isolated = closed - dilated"
            };
            for (size_t i = 0; i < panels.size(); ++i) {
                label(panels[i], names[i]);
            }

            cv::Mat top;
            cv::hconcat(std::vector<cv::Mat>{ panels[0], panels[1], panels[2] }, top);

            cv::Mat blank = cv::Mat::zeros(panels[3].size(), CV_8UC3);
            cv::Mat bottom;
            cv::hconcat(std::vector<cv::Mat>{ panels[3], panels[4], blank }, bottom);

            cv::Mat full;
            cv::vconcat(std::vector<cv::Mat>{ top, bottom }, full);
            return full;
        }

        void draw_rotated_rect_on_image(cv::Mat& image,
            const cv::RotatedRect& rect,
            const cv::Scalar& color,
            int thickness) const {
            if (image.empty()) {
                return;
            }
            cv::Point2f corners[4];
            rect.points(corners);
            for (int i = 0; i < 4; ++i) {
                cv::line(image,
                    cv::Point(static_cast<int>(std::lround(corners[i].x)),
                        static_cast<int>(std::lround(corners[i].y))),
                    cv::Point(static_cast<int>(std::lround(corners[(i + 1) % 4].x)),
                        static_cast<int>(std::lround(corners[(i + 1) % 4].y))),
                    color,
                    thickness,
                    cv::LINE_AA);
            }
        }


        struct LineCandidate {
            double angle = 0.0;
            double length = 0.0;
            double mid_x = 0.0;
            double mid_y = 0.0;
            double x1 = 0.0;
            double y1 = 0.0;
            double x2 = 0.0;
            double y2 = 0.0;

        };

        double normalize_angle_180(double angle) const {
            angle = std::fmod(angle, 180.0);
            if (angle < 0.0) angle += 180.0;
            return angle;
        }

        double angle_distance_180(double a, double b) const {
            double diff = std::abs(normalize_angle_180(a) - normalize_angle_180(b));
            return std::min(diff, 180.0 - diff);
        }

        void update_line_roi(const Berth& b) {
            if (!is_valid_berth_size(b)) return;
            roi_berth_ = b;
            has_line_roi_ = true;
        }

        Berth expanded_line_roi(const Berth& b) const {
            Berth roi = b;
            roi.w = b.w + 2.0 * line_roi_margin_;
            roi.l = b.l + 2.0 * line_roi_margin_;
            return roi;
        }

        Eigen::Vector3d clamp_point_to_berth_roi(const Berth& b, const Eigen::Vector3d& pt) const {
            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);
            double dx = pt.x() - b.cx;
            double dy = pt.y() - b.cy;
            double rx = dx * cos_a + dy * sin_a;
            double ry = -dx * sin_a + dy * cos_a;
            double half_w = b.w / 2.0;
            double half_l = b.l / 2.0;

            rx = std::max(-half_w, std::min(half_w, rx));
            ry = std::max(-half_l, std::min(half_l, ry));

            return Eigen::Vector3d(
                rx * cos_a - ry * sin_a + b.cx,
                rx * sin_a + ry * cos_a + b.cy,
                pt.z());
        }

        DebugLine2D local_line_to_world(double x1, double y1, double x2, double y2) const {
            double rad = roi_berth_.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);
            DebugLine2D line;
            line.x1 = x1 * cos_a - y1 * sin_a + roi_berth_.cx;
            line.y1 = x1 * sin_a + y1 * cos_a + roi_berth_.cy;
            line.x2 = x2 * cos_a - y2 * sin_a + roi_berth_.cx;
            line.y2 = x2 * sin_a + y2 * cos_a + roi_berth_.cy;
            return line;
        }

        bool point_in_expanded_line_roi(const Eigen::Vector3d& pt,
            const Berth& b,
            double extra_margin = 0.0) const {
            if (pt.z() <= z_min_ || pt.z() >= z_max_) return false;

            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);
            double dx = pt.x() - b.cx;
            double dy = pt.y() - b.cy;
            double rx = dx * cos_a + dy * sin_a;
            double ry = -dx * sin_a + dy * cos_a;

            return std::abs(rx) <= (b.w / 2.0 + line_roi_margin_ + extra_margin) &&
                std::abs(ry) <= (b.l / 2.0 + line_roi_margin_ + extra_margin);
        }

        bool single_anchor_has_stable_history(const Eigen::Vector3d& anchor,
            size_t* stable_index = nullptr) const {
            const Eigen::Vector2d anchor_xy(anchor.x(), anchor.y());
            for (size_t i = 0; i < stable_history_berths_.size(); ++i) {
                if (berth_recovery::anchorOverlapsStableBerth(
                        anchor_xy, stable_history_berths_[i],
                        stable_history_anchor_margin_m_)) {
                    if (stable_index) {
                        *stable_index = i;
                    }
                    return true;
                }
            }
            return false;
        }

        int find_nearest_opening_edge(const Berth& b) const {
            if (!is_valid_berth_size(b)) return -1;
            return find_nearest_edge_to_point(b, 0.0, 0.0);
        }

        int find_nearest_edge_to_point(const Berth& b, double px, double py) const {
            if (!is_valid_berth_size(b)) return -1;

            const double rad = b.angle * M_PI / 180.0;
            const double ca = std::cos(rad);
            const double sa = std::sin(rad);
            const double half_w = b.w / 2.0;
            const double half_l = b.l / 2.0;

            const Eigen::Vector2d local_centers[4] = {
                Eigen::Vector2d(0.0, -half_l),
                Eigen::Vector2d(half_w, 0.0),
                Eigen::Vector2d(0.0, half_l),
                Eigen::Vector2d(-half_w, 0.0)
            };

            int opening_edge = -1;
            double best_dist2 = std::numeric_limits<double>::infinity();
            for (int i = 0; i < 4; ++i) {
                const double x = local_centers[i].x() * ca - local_centers[i].y() * sa + b.cx;
                const double y = local_centers[i].x() * sa + local_centers[i].y() * ca + b.cy;
                const double dx = x - px;
                const double dy = y - py;
                const double dist2 = dx * dx + dy * dy;
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    opening_edge = i;
                }
            }
            return opening_edge;
        }

        std::array<int, 4> count_edge_points(const Berth& b,
            const std::vector<Eigen::Vector3d>& points) const {
            std::array<int, 4> counts{0, 0, 0, 0};
            if (!is_valid_berth_size(b)) return counts;

            const double rad = b.angle * M_PI / 180.0;
            const double ca = std::cos(rad);
            const double sa = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;
            const double band = std::max(0.05, opening_edge_count_band_m_);
            const double length_ratio = std::clamp(edge_count_strip_length_ratio_, 0.05, 1.0);
            const double half_count_w = half_w * length_ratio;
            const double half_count_l = half_l * length_ratio;

            for (const auto& pt : points) {
                if (!std::isfinite(pt.x()) ||
                    !std::isfinite(pt.y()) ||
                    !std::isfinite(pt.z())) {
                    continue;
                }

                const double dx = pt.x() - b.cx;
                const double dy = pt.y() - b.cy;
                const double rx = dx * ca + dy * sa;
                const double ry = -dx * sa + dy * ca;
                if (std::abs(rx) > half_w + band ||
                    std::abs(ry) > half_l + band) {
                    continue;
                }

                if (std::abs(ry + half_l) <= band && std::abs(rx) <= half_count_w) ++counts[0];
                if (std::abs(rx - half_w) <= band && std::abs(ry) <= half_count_l) ++counts[1];
                if (std::abs(ry - half_l) <= band && std::abs(rx) <= half_count_w) ++counts[2];
                if (std::abs(rx + half_w) <= band && std::abs(ry) <= half_count_l) ++counts[3];
            }

            return counts;
        }

        void compute_edge_heights(Berth& b,
            const std::vector<Eigen::Vector3d>& points) const {
            b.edge_heights = {0.0, 0.0, 0.0, 0.0};
            b.edge_height_counts = {0, 0, 0, 0};
            b.edge_height_valid = {false, false, false, false};
            if (!is_valid_berth_size(b)) return;

            const double rad = b.angle * M_PI / 180.0;
            const double ca = std::cos(rad);
            const double sa = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;
            const double band = std::max(0.05, opening_edge_count_band_m_);
            const double length_ratio = std::clamp(edge_count_strip_length_ratio_, 0.05, 1.0);
            const double half_count_w = half_w * length_ratio;
            const double half_count_l = half_l * length_ratio;
            const double bin_m = std::max(0.1, edge_height_bin_m_);
            const int bins_w = std::max(1, static_cast<int>(std::ceil((2.0 * half_count_w) / bin_m)));
            const int bins_l = std::max(1, static_cast<int>(std::ceil((2.0 * half_count_l) / bin_m)));

            std::array<std::vector<double>, 4> max_z;
            max_z[0].assign(bins_w, -std::numeric_limits<double>::infinity());
            max_z[1].assign(bins_l, -std::numeric_limits<double>::infinity());
            max_z[2].assign(bins_w, -std::numeric_limits<double>::infinity());
            max_z[3].assign(bins_l, -std::numeric_limits<double>::infinity());

            auto update_bin = [&](int edge, double along, double half_len, int bins, double z) {
                if (edge < 0 || edge >= 4 || bins <= 0 || half_len <= 1e-6) {
                    return;
                }
                double t = (along + half_len) / (2.0 * half_len);
                t = std::clamp(t, 0.0, 1.0);
                int bin = static_cast<int>(std::floor(t * bins));
                if (bin >= bins) {
                    bin = bins - 1;
                }
                max_z[edge][bin] = std::max(max_z[edge][bin], z);
            };

            for (const auto& pt : points) {
                if (!std::isfinite(pt.x()) ||
                    !std::isfinite(pt.y()) ||
                    !std::isfinite(pt.z()) ||
                    pt.z() <= z_min_ ||
                    pt.z() >= z_max_) {
                    continue;
                }

                const double dx = pt.x() - b.cx;
                const double dy = pt.y() - b.cy;
                const double rx = dx * ca + dy * sa;
                const double ry = -dx * sa + dy * ca;
                if (std::abs(rx) > half_w + band ||
                    std::abs(ry) > half_l + band) {
                    continue;
                }

                if (std::abs(ry + half_l) <= band && std::abs(rx) <= half_count_w) {
                    update_bin(0, rx, half_count_w, bins_w, pt.z());
                }
                if (std::abs(rx - half_w) <= band && std::abs(ry) <= half_count_l) {
                    update_bin(1, ry, half_count_l, bins_l, pt.z());
                }
                if (std::abs(ry - half_l) <= band && std::abs(rx) <= half_count_w) {
                    update_bin(2, rx, half_count_w, bins_w, pt.z());
                }
                if (std::abs(rx + half_w) <= band && std::abs(ry) <= half_count_l) {
                    update_bin(3, ry, half_count_l, bins_l, pt.z());
                }
            }

            auto median_of = [](std::vector<double> values) {
                const size_t mid = values.size() / 2;
                std::nth_element(values.begin(), values.begin() + mid, values.end());
                double median = values[mid];
                if (values.size() % 2 == 0 && mid > 0) {
                    std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
                    median = 0.5 * (median + values[mid - 1]);
                }
                return median;
            };

            for (int edge = 0; edge < 4; ++edge) {
                if (edge == b.opening_edge) {
                    continue;
                }

                std::vector<double> tops;
                tops.reserve(max_z[edge].size());
                for (double z : max_z[edge]) {
                    if (std::isfinite(z)) {
                        tops.push_back(z);
                    }
                }
                if (static_cast<int>(tops.size()) < edge_height_min_bins_) {
                    continue;
                }

                const double median = median_of(tops);
                const double outlier_m = std::max(0.05, edge_height_outlier_m_);
                double sum = 0.0;
                int count = 0;
                for (double z : tops) {
                    if (std::abs(z - median) <= outlier_m) {
                        sum += z;
                        ++count;
                    }
                }
                if (count < edge_height_min_bins_) {
                    continue;
                }

                b.edge_heights[edge] = sum / count;
                b.edge_height_counts[edge] = count;
                b.edge_height_valid[edge] = true;
            }
        }

        void compute_ship_relative_metrics(Berth& b) const {
            b.ship_edge_distances = {0.0, 0.0, 0.0, 0.0};
            b.ship_edge_distance_valid = {false, false, false, false};
            b.ship_edge_distance_lines = {};
            b.ship_center_distance = std::sqrt(b.cx * b.cx + b.cy * b.cy);
            b.ship_opening_normal_angle = 0.0;
            b.has_ship_metrics = false;
            if (!is_valid_berth_size(b) || b.opening_edge < 0 || b.opening_edge >= 4) {
                return;
            }

            const double rad = b.angle * M_PI / 180.0;
            const double ca = std::cos(rad);
            const double sa = std::sin(rad);
            const double half_w = b.w * 0.5;
            const double half_l = b.l * 0.5;

            const double ship_dx = -b.cx;
            const double ship_dy = -b.cy;
            const double ship_rx = ship_dx * ca + ship_dy * sa;
            const double ship_ry = -ship_dx * sa + ship_dy * ca;

            auto local_to_world_line_from_ship = [&](double foot_x, double foot_y) {
                DebugLine2D line;
                line.x1 = 0.0;
                line.y1 = 0.0;
                line.x2 = b.cx + foot_x * ca - foot_y * sa;
                line.y2 = b.cy + foot_x * sa + foot_y * ca;
                return line;
            };

            for (int edge = 0; edge < 4; ++edge) {
                if (edge == b.opening_edge) {
                    continue;
                }

                double foot_x = ship_rx;
                double foot_y = ship_ry;
                double distance = 0.0;
                switch (edge) {
                case 0:
                    foot_y = -half_l;
                    distance = std::abs(ship_ry + half_l);
                    break;
                case 1:
                    foot_x = half_w;
                    distance = std::abs(ship_rx - half_w);
                    break;
                case 2:
                    foot_y = half_l;
                    distance = std::abs(ship_ry - half_l);
                    break;
                case 3:
                    foot_x = -half_w;
                    distance = std::abs(ship_rx + half_w);
                    break;
                default:
                    continue;
                }

                b.ship_edge_distances[edge] = distance;
                b.ship_edge_distance_valid[edge] = true;
                b.ship_edge_distance_lines[edge] =
                    local_to_world_line_from_ship(foot_x, foot_y);
            }

            const Eigen::Vector2d opening_dirs[4] = {
                Eigen::Vector2d(0.0, -1.0),
                Eigen::Vector2d(1.0, 0.0),
                Eigen::Vector2d(0.0, 1.0),
                Eigen::Vector2d(-1.0, 0.0)
            };
            const Eigen::Vector2d local_open = opening_dirs[b.opening_edge];
            const Eigen::Vector2d open_world(
                local_open.x() * ca - local_open.y() * sa,
                local_open.x() * sa + local_open.y() * ca);
            const Eigen::Vector2d center_to_ship(-b.cx, -b.cy);
            const double denom = open_world.norm() * center_to_ship.norm();
            if (denom > 1e-6) {
                const double cos_angle = std::clamp(
                    open_world.dot(center_to_ship) / denom, -1.0, 1.0);
                b.ship_opening_normal_angle = std::acos(cos_angle) * 180.0 / M_PI;
            }
            b.has_ship_metrics = true;
        }

        int find_sparse_opening_edge(const Berth& b,
            const std::vector<Eigen::Vector3d>& points) const {
            if (!is_valid_berth_size(b)) return -1;

            const std::array<int, 4> counts = count_edge_points(b, points);
            const std::array<int, 4> ray_hits = count_opening_ray_hits(b, points);
            return select_opening_edge_from_rays(ray_hits, counts);
        }

        std::array<int, 4> count_opening_ray_hits(const Berth& b,
            const std::vector<Eigen::Vector3d>& points) const {
            return berth_opening::countOpeningRayHits(
                b, points, opening_ray_length_m_,
                opening_ray_half_width_m_);
        }

        int select_opening_edge_from_rays(const std::array<int, 4>& ray_hits,
            const std::array<int, 4>& edge_counts) const {
            const int min_hits = std::max(1, opening_ray_min_hits_);
            int opening_edge = -1;
            for (int edge = 0; edge < 4; ++edge) {
                if (ray_hits[edge] >= min_hits) {
                    continue;
                }
                if (opening_edge < 0 ||
                    edge_counts[edge] < edge_counts[opening_edge] ||
                    (edge_counts[edge] == edge_counts[opening_edge] &&
                        ray_hits[edge] < ray_hits[opening_edge])) {
                    opening_edge = edge;
                }
            }
            if (opening_edge >= 0) {
                return opening_edge;
            }
            return select_opening_edge_from_counts(edge_counts);
        }

        int select_opening_edge_from_counts(const std::array<int, 4>& counts) const {
            int opening_edge = 0;
            for (int i = 1; i < 4; ++i) {
                if (counts[i] < counts[opening_edge]) {
                    opening_edge = i;
                }
            }
            return opening_edge;
        }

        double point_distance_xy(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
            const double dx = a.x() - b.x();
            const double dy = a.y() - b.y();
            return std::sqrt(dx * dx + dy * dy);
        }

        double top_pair_jump_distance(const Eigen::Vector3d& a,
            const Eigen::Vector3d& b,
            const Eigen::Vector3d& ref_a,
            const Eigen::Vector3d& ref_b) const {
            const double direct = std::max(point_distance_xy(a, ref_a), point_distance_xy(b, ref_b));
            const double swapped = std::max(point_distance_xy(a, ref_b), point_distance_xy(b, ref_a));
            return std::min(direct, swapped);
        }

        double point_to_anchor_bus_distance(const Eigen::Vector3d& pt) const {
            if (!anchor_bus_.valid) {
                return 0.0;
            }
            const double dx = pt.x() - anchor_bus_.start.x();
            const double dy = pt.y() - anchor_bus_.start.y();
            return std::abs(dx * (-anchor_bus_.uy) + dy * anchor_bus_.ux);
        }

        bool point_is_near_anchor_bus(const Eigen::Vector3d& pt) const {
            if (!anchor_bus_.valid) {
                return true;
            }
            return point_to_anchor_bus_distance(pt) <= anchor_bus_distance_tol_m_;
        }

        bool build_anchor_bus_from_two_pairs(const Eigen::Vector3d& a0,
            const Eigen::Vector3d& a1,
            const Eigen::Vector3d& b0,
            const Eigen::Vector3d& b1,
            AnchorBusState& out_bus) const {
            const double ax = a1.x() - a0.x();
            const double ay = a1.y() - a0.y();
            const double bx = b1.x() - b0.x();
            const double by = b1.y() - b0.y();
            const double len_a = std::sqrt(ax * ax + ay * ay);
            const double len_b = std::sqrt(bx * bx + by * by);
            if (!berth_recovery::validAnchorPairDistance(len_a)
                || !berth_recovery::validAnchorPairDistance(len_b)) {
                return false;
            }

            const double au = ax / len_a;
            const double av = ay / len_a;
            const double bu = bx / len_b;
            const double bv = by / len_b;
            const double abs_dot = std::abs(std::clamp(au * bu + av * bv, -1.0, 1.0));
            const double angle_diff = std::acos(abs_dot) * 180.0 / M_PI;
            if (angle_diff > anchor_bus_angle_tol_deg_) {
                return false;
            }

            auto signed_perp = [&](const Eigen::Vector3d& p) {
                const double dx = p.x() - a0.x();
                const double dy = p.y() - a0.y();
                return dx * (-av) + dy * au;
            };
            const double dist_b0 = std::abs(signed_perp(b0));
            const double dist_b1 = std::abs(signed_perp(b1));
            if (std::max(dist_b0, dist_b1) > anchor_bus_distance_tol_m_) {
                return false;
            }

            std::array<double, 4> proj = {
                0.0,
                len_a,
                (b0.x() - a0.x()) * au + (b0.y() - a0.y()) * av,
                (b1.x() - a0.x()) * au + (b1.y() - a0.y()) * av
            };
            const double min_a = std::min(proj[0], proj[1]);
            const double max_a = std::max(proj[0], proj[1]);
            const double min_b = std::min(proj[2], proj[3]);
            const double max_b = std::max(proj[2], proj[3]);
            double gap = 0.0;
            if (max_a < min_b) {
                gap = min_b - max_a;
            }
            else if (max_b < min_a) {
                gap = min_a - max_b;
            }
            if (gap > anchor_bus_projection_gap_m_) {
                return false;
            }

            const double min_p = *std::min_element(proj.begin(), proj.end());
            const double max_p = *std::max_element(proj.begin(), proj.end());
            out_bus.valid = true;
            out_bus.ux = au;
            out_bus.uy = av;
            out_bus.start = Eigen::Vector3d(a0.x() + au * min_p, a0.y() + av * min_p, 0.0);
            out_bus.end = Eigen::Vector3d(a0.x() + au * max_p, a0.y() + av * max_p, 0.0);
            return true;
        }

        void update_anchor_bus_from_pairs(const std::vector<Eigen::Vector3d>& first_points,
            const std::vector<Eigen::Vector3d>& second_points,
            std::vector<std::string>& debug_messages) {
            // The confirmed ENU bus is the persistent constraint for this frame.
            // Raw frame-local anchor pairs must not move or replace it.
            if (anchor_bus_from_world_) {
                if (anchor_bus_.valid) {
                    last_debug_lines_.push_back(DebugLine2D{
                        anchor_bus_.start.x(), anchor_bus_.start.y(),
                        anchor_bus_.end.x(), anchor_bus_.end.y()});
                }
                return;
            }
            if (first_points.size() != second_points.size()) {
                // A lidar-frame bus cannot persist while the vessel moves.
                // Only the ENU-projected bus is allowed to survive missing pairs.
                anchor_bus_ = AnchorBusState{};
                return;
            }
            const size_t pair_count = std::min(first_points.size(), second_points.size());
            if (pair_count < 2) {
                anchor_bus_ = AnchorBusState{};
                return;
            }

            AnchorBusState best_bus;
            double best_len = -1.0;
            for (size_t i = 0; i < pair_count; ++i) {
                for (size_t j = i + 1; j < pair_count; ++j) {
                    AnchorBusState candidate;
                    if (!build_anchor_bus_from_two_pairs(
                            first_points[i], second_points[i],
                            first_points[j], second_points[j],
                            candidate)) {
                        continue;
                    }
                    const double dx = candidate.end.x() - candidate.start.x();
                    const double dy = candidate.end.y() - candidate.start.y();
                    const double len = std::sqrt(dx * dx + dy * dy);
                    if (len > best_len) {
                        best_len = len;
                        best_bus = candidate;
                    }
                }
            }

            if (best_bus.valid) {
                const bool was_valid = anchor_bus_.valid;
                anchor_bus_ = best_bus;
                last_debug_lines_.push_back(DebugLine2D{
                    anchor_bus_.start.x(), anchor_bus_.start.y(),
                    anchor_bus_.end.x(), anchor_bus_.end.y()});
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[BerthDebug] anchor bus "
                    << (was_valid ? "updated" : "created")
                    << " start=(" << anchor_bus_.start.x() << "," << anchor_bus_.start.y() << ")"
                    << " end=(" << anchor_bus_.end.x() << "," << anchor_bus_.end.y() << ")"
                    << " length=" << best_len
                    << " pairs=" << pair_count;
                debug_messages.push_back(oss.str());
            }
            else if (anchor_bus_.valid) {
                anchor_bus_ = AnchorBusState{};
            }
        }

        bool has_matching_berth(const Berth& berth, const std::vector<Berth>& candidates) const {
            return find_matching_berth_index(berth, candidates) < candidates.size();
        }

        size_t find_matching_berth_index(const Berth& berth,
            const std::vector<Berth>& candidates) const {
            double best_score = std::numeric_limits<double>::infinity();
            size_t best_idx = candidates.size();
            for (const Berth& candidate : candidates) {
                const double dx = berth.cx - candidate.cx;
                const double dy = berth.cy - candidate.cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                const double angle_diff = angle_distance_180(berth.angle, candidate.angle);
                const double size_diff = std::abs(berth.w - candidate.w)
                    + std::abs(berth.l - candidate.l);
                if (dist < 6.0 && angle_diff < 30.0 && size_diff < 6.0) {
                    const double score = dist + 0.05 * angle_diff + 0.25 * size_diff;
                    if (score < best_score) {
                        best_score = score;
                        best_idx = static_cast<size_t>(&candidate - candidates.data());
                    }
                }
            }
            return best_idx;
        }

        bool normal_detection_jumps_after_recovery(const Berth& tracked,
            const Berth& detected) const {
            const double angle_diff = angle_distance_180(tracked.angle, detected.angle);
            const double size_diff = std::abs(tracked.w - detected.w)
                + std::abs(tracked.l - detected.l);
            return angle_diff > recovered_to_detect_angle_jump_deg_ ||
                size_diff > recovered_to_detect_size_jump_m_;
        }

        Eigen::Vector2d opening_edge_world_direction(
            const Berth& berth, int edge) const {
            if (edge < 0 || edge >= 4) {
                return Eigen::Vector2d::Zero();
            }
            const Eigen::Vector2d local_directions[4] = {
                Eigen::Vector2d(0.0, -1.0),
                Eigen::Vector2d(1.0, 0.0),
                Eigen::Vector2d(0.0, 1.0),
                Eigen::Vector2d(-1.0, 0.0)
            };
            const double rad = berth.angle * M_PI / 180.0;
            const double ca = std::cos(rad);
            const double sa = std::sin(rad);
            const Eigen::Vector2d& local = local_directions[edge];
            return Eigen::Vector2d(
                local.x() * ca - local.y() * sa,
                local.x() * sa + local.y() * ca);
        }

        int opening_edge_nearest_world_direction(
            const Berth& berth,
            const Eigen::Vector2d& direction) const {
            if (direction.squaredNorm() <= 1e-8) {
                return -1;
            }
            const Eigen::Vector2d normalized = direction.normalized();
            int best_edge = -1;
            double best_dot = -std::numeric_limits<double>::infinity();
            for (int edge = 0; edge < 4; ++edge) {
                const Eigen::Vector2d edge_direction =
                    opening_edge_world_direction(berth, edge);
                const double dot = normalized.dot(edge_direction);
                if (dot > best_dot) {
                    best_dot = dot;
                    best_edge = edge;
                }
            }
            return best_edge;
        }

        void stabilize_output_opening_edges(
            std::vector<Berth>& berths,
            std::vector<std::string>& debug_messages) {
            std::vector<OpeningEdgeState> next_states(berths.size());
            std::vector<bool> used_tracks(tracked_berths_.size(), false);
            const int required_frames =
                std::max(1, opening_edge_lock_frames_);

            for (size_t i = 0; i < berths.size(); ++i) {
                Berth& berth = berths[i];
                size_t matched_track = tracked_berths_.size();
                double best_score = std::numeric_limits<double>::infinity();
                for (size_t j = 0; j < tracked_berths_.size(); ++j) {
                    if (used_tracks[j]) {
                        continue;
                    }
                    const double dx = berth.cx - tracked_berths_[j].cx;
                    const double dy = berth.cy - tracked_berths_[j].cy;
                    const double center_dist = std::hypot(dx, dy);
                    const double angle_diff = angle_distance_180(
                        berth.angle, tracked_berths_[j].angle);
                    if (center_dist >= 8.0 || angle_diff >= 35.0) {
                        continue;
                    }
                    const double score =
                        center_dist + 0.05 * angle_diff;
                    if (score < best_score) {
                        best_score = score;
                        matched_track = j;
                    }
                }

                OpeningEdgeState state;
                if (matched_track < opening_edge_states_.size()) {
                    state = opening_edge_states_[matched_track];
                    used_tracks[matched_track] = true;
                }

                const int detected_edge = berth.opening_edge;
                const Eigen::Vector2d detected_direction =
                    opening_edge_world_direction(berth, detected_edge);
                const std::string track_label =
                    matched_track < tracked_berths_.size()
                    ? std::to_string(matched_track + 1)
                    : "new";
                if (state.locked
                    && state.locked_direction.squaredNorm() > 1e-8) {
                    const int mapped_edge =
                        opening_edge_nearest_world_direction(
                            berth, state.locked_direction);
                    if (mapped_edge >= 0
                        && detected_edge != mapped_edge) {
                        debug_messages.push_back(
                            "[BerthDebug] keep locked U-shape opening direction track=#"
                            + track_label
                            + " mapped_edge=" + std::to_string(mapped_edge)
                            + " detected=" + std::to_string(detected_edge));
                        berth.opening_edge = mapped_edge;
                    }
                    state.locked_edge = mapped_edge;
                } else if (detected_direction.squaredNorm() > 1e-8) {
                    const double direction_cosine = std::cos(
                        opening_edge_direction_stable_deg_
                        * M_PI / 180.0);
                    if (state.has_candidate_direction
                        && state.candidate_direction.dot(detected_direction)
                            >= direction_cosine) {
                        ++state.consecutive_count;
                        const Eigen::Vector2d averaged =
                            state.candidate_direction
                                * static_cast<double>(
                                    state.consecutive_count - 1)
                            + detected_direction;
                        state.candidate_direction = averaged.normalized();
                    } else {
                        state.candidate_edge = detected_edge;
                        state.consecutive_count = 1;
                        state.candidate_direction = detected_direction;
                        state.has_candidate_direction = true;
                    }
                    if (state.consecutive_count >= required_frames) {
                        state.locked = true;
                        state.locked_edge = detected_edge;
                        state.locked_direction =
                            state.candidate_direction;
                        debug_messages.push_back(
                            "[BerthDebug] lock U-shape opening direction track=#"
                            + track_label
                            + " edge=" + std::to_string(state.locked_edge)
                            + " stable_frames="
                            + std::to_string(state.consecutive_count));
                    }
                }
                next_states[i] = state;
            }
            opening_edge_states_ = std::move(next_states);
        }

        void update_tracked_berths(const std::vector<Berth>& berths,
            const std::vector<bool>& recovery_flags) {
            std::vector<TopRegionState> next_states(berths.size());
            std::vector<bool> next_recovered_flags(berths.size(), false);
            std::vector<bool> used_states(top_region_states_.size(), false);

            for (size_t i = 0; i < berths.size(); ++i) {
                next_recovered_flags[i] =
                    i < recovery_flags.size() && recovery_flags[i];
                double best_score = std::numeric_limits<double>::infinity();
                size_t best_idx = top_region_states_.size();
                for (size_t j = 0; j < tracked_berths_.size() && j < top_region_states_.size(); ++j) {
                    if (used_states[j]) {
                        continue;
                    }
                    const double dx = berths[i].cx - tracked_berths_[j].cx;
                    const double dy = berths[i].cy - tracked_berths_[j].cy;
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    const double angle_diff = angle_distance_180(berths[i].angle, tracked_berths_[j].angle);
                    const double size_diff = std::abs(berths[i].w - tracked_berths_[j].w)
                        + std::abs(berths[i].l - tracked_berths_[j].l);
                    if (dist < 8.0 && angle_diff < 35.0) {
                        const double score = dist + 0.05 * angle_diff + 0.25 * size_diff;
                        if (score < best_score) {
                            best_score = score;
                            best_idx = j;
                        }
                    }
                }
                if (best_idx < top_region_states_.size()) {
                    next_states[i] = top_region_states_[best_idx];
                    used_states[best_idx] = true;
                }
            }

            tracked_berths_ = berths;
            tracked_was_recovered_ = std::move(next_recovered_flags);
            top_region_states_ = std::move(next_states);
            is_tracked_ = !tracked_berths_.empty();
        }

        void accept_stable_top_regions(TopRegionState& state,
            const Eigen::Vector3d& a,
            const Eigen::Vector3d& b) {
            state.stable_a = a;
            state.stable_b = b;
            state.has_stable = true;
            state.has_pending = false;
            state.pending_count = 0;
        }

        void stabilize_top_regions(TopRegionState& state,
            const Eigen::Vector3d& raw_a,
            const Eigen::Vector3d& raw_b,
            Eigen::Vector3d& out_a,
            Eigen::Vector3d& out_b) {
            if (!state.has_stable) {
                accept_stable_top_regions(state, raw_a, raw_b);
                out_a = state.stable_a;
                out_b = state.stable_b;
                return;
            }

            const double jump = top_pair_jump_distance(raw_a, raw_b, state.stable_a, state.stable_b);
            if (jump <= top_region_jump_threshold_) {
                accept_stable_top_regions(state, raw_a, raw_b);
                out_a = state.stable_a;
                out_b = state.stable_b;
                return;
            }

            if (!state.has_pending ||
                top_pair_jump_distance(raw_a, raw_b, state.pending_a, state.pending_b) > top_region_jump_threshold_) {
                state.pending_a = raw_a;
                state.pending_b = raw_b;
                state.has_pending = true;
                state.pending_count = 1;
            }
            else {
                ++state.pending_count;
            }

            if (state.pending_count >= top_region_jump_confirm_frames_) {
                accept_stable_top_regions(state, state.pending_a, state.pending_b);
            }

            out_a = state.stable_a;
            out_b = state.stable_b;
        }

        bool find_highest_region_near_anchor(const Eigen::Vector3d& center,
            const std::vector<Eigen::Vector3d>& points,
            Eigen::Vector3d& out_point,
            const Berth* search_roi = nullptr,
            int* candidate_count = nullptr) const {
            if (candidate_count) *candidate_count = 0;
            if (!std::isfinite(center.x()) || !std::isfinite(center.y())) {
                return false;
            }

            const double half = anchor_recovery_local_window_ * 0.5;
            struct CandidatePoint {
                Eigen::Vector3d world;
                double z = 0.0;
            };

            std::vector<CandidatePoint> candidates;
            candidates.reserve(64);
            for (const auto& pt : points) {
                if (!std::isfinite(pt.x()) ||
                    !std::isfinite(pt.y()) ||
                    !std::isfinite(pt.z())) {
                    continue;
                }
                if (pt.z() < z_min_ || pt.z() > z_max_) {
                    continue;
                }
                if (pt.z() <= anchor_min_height_m_) {
                    continue;
                }
                if (!point_is_near_anchor_bus(pt)) {
                    continue;
                }
                if (search_roi && !point_in_expanded_line_roi(
                    pt, *search_roi, anchor_recovery_roi_extra_margin_)) {
                    continue;
                }
                if (std::abs(pt.x() - center.x()) <= half &&
                    std::abs(pt.y() - center.y()) <= half) {
                    candidates.push_back({ pt, pt.z() });
                }
            }
            if (candidate_count) *candidate_count = static_cast<int>(candidates.size());
            if (candidates.empty()) {
                return false;
            }

            const auto best_it = std::max_element(candidates.begin(), candidates.end(),
                [](const CandidatePoint& a, const CandidatePoint& b) {
                    return a.z < b.z;
                });

            const Eigen::Vector3d seed = best_it->world;
            const double max_z = best_it->z;
            double sum_x = 0.0;
            double sum_y = 0.0;
            int count = 0;
            for (const auto& c : candidates) {
                const double dx = c.world.x() - seed.x();
                const double dy = c.world.y() - seed.y();
                if (dx * dx + dy * dy <= top_region_radius_ * top_region_radius_ &&
                    c.z >= max_z - top_region_top_band_) {
                    sum_x += c.world.x();
                    sum_y += c.world.y();
                    ++count;
                }
            }

            if (count == 0) {
                out_point = seed;
            } else {
                out_point = Eigen::Vector3d(sum_x / count, sum_y / count, max_z);
            }
            out_point.x() = std::max(center.x() - half, std::min(center.x() + half, out_point.x()));
            out_point.y() = std::max(center.y() - half, std::min(center.y() + half, out_point.y()));
            return true;
        }

        bool recover_berth_from_highest_anchors(const std::vector<Eigen::Vector3d>& points,
            const Berth& tracked_berth,
            TopRegionState& top_state,
            Berth& out_berth,
            Eigen::Vector3d& anchor_a,
            Eigen::Vector3d& anchor_b,
            AnchorRecoveryDebug& debug,
            const Berth* search_roi_reference = nullptr) {
            debug = AnchorRecoveryDebug{};
            debug.tracked_w = tracked_berth.w;
            debug.tracked_l = tracked_berth.l;
            debug.tracked_size_valid =
                std::isfinite(tracked_berth.w) &&
                std::isfinite(tracked_berth.l) &&
                tracked_berth.w > 1e-6 &&
                tracked_berth.l > 1e-6;
            if (!debug.tracked_size_valid) {
                debug.status = AnchorRecoveryStatus::InvalidTrackedBerthSize;
                return false;
            }

            const Berth& roi_reference =
                search_roi_reference && is_valid_berth_size(*search_roi_reference)
                ? *search_roi_reference
                : tracked_berth;
            Berth search_roi = expanded_line_roi(roi_reference);
            debug.searched_top_regions = true;
            int top_candidate_count = 0;
            if (top_state.has_stable) {
                int count_a = 0;
                int count_b = 0;
                const bool found_a = find_highest_region_near_anchor(
                    top_state.stable_a, points, anchor_a, &search_roi, &count_a);
                const bool found_b = find_highest_region_near_anchor(
                    top_state.stable_b, points, anchor_b, &search_roi, &count_b);
                top_candidate_count = count_a + count_b;
                if (!found_a || !found_b) {
                    debug.top_search_candidate_count = top_candidate_count;
                    debug.status = AnchorRecoveryStatus::NoTopSearchCandidates;
                    return false;
                }
            } else {
                if (find_highest_regions_in_roi(
                        search_roi, points, anchor_a, anchor_b, true,
                        &top_candidate_count) < 2) {
                    debug.top_search_candidate_count = top_candidate_count;
                    debug.status = AnchorRecoveryStatus::NoTopSearchCandidates;
                    return false;
                }
            }
            debug.top_search_candidate_count = top_candidate_count;
            debug.found_top_regions = true;
            const TopRegionState previous_top_state = top_state;
            stabilize_top_regions(top_state, anchor_a, anchor_b, anchor_a, anchor_b);

            const double dx = anchor_b.x() - anchor_a.x();
            const double dy = anchor_b.y() - anchor_a.y();
            const double anchor_width = std::sqrt(dx * dx + dy * dy);
            debug.anchor_distance = anchor_width;
            debug.recovered_w = anchor_width;
            if (!berth_recovery::validAnchorPairDistance(anchor_width)) {
                top_state = previous_top_state;
                debug.status = AnchorRecoveryStatus::InvalidAnchorWidth;
                return false;
            }

            const double width = tracked_berth.w;
            debug.recovered_w = width;
            if (!std::isfinite(width) || width <= anchor_recovery_min_size_) {
                top_state = previous_top_state;
                debug.status = AnchorRecoveryStatus::InvalidTrackedBerthSize;
                return false;
            }

            const double length = tracked_berth.l;
            debug.recovered_l = length;
            if (!std::isfinite(length) || length <= anchor_recovery_min_size_) {
                top_state = previous_top_state;
                debug.status = AnchorRecoveryStatus::InvalidTrackedLength;
                return false;
            }

            const double width_angle = std::atan2(dy, dx);
            const double ux = std::cos(width_angle);
            const double uy = std::sin(width_angle);
            const double vx = -uy;
            const double vy = ux;
            const double mid_x = 0.5 * (anchor_a.x() + anchor_b.x());
            const double mid_y = 0.5 * (anchor_a.y() + anchor_b.y());

            const double prev_dx = tracked_berth.cx - mid_x;
            const double prev_dy = tracked_berth.cy - mid_y;
            const double side = (prev_dx * vx + prev_dy * vy) >= 0.0 ? 1.0 : -1.0;

            Berth recovered;
            recovered.cx = mid_x + side * vx * length * 0.5;
            recovered.cy = mid_y + side * vy * length * 0.5;
            recovered.w = width;
            recovered.l = length;
            recovered.angle = width_angle * 180.0 / M_PI;
            while (recovered.angle < 0.0) recovered.angle += 180.0;
            while (recovered.angle >= 180.0) recovered.angle -= 180.0;
            // Keep the detector's physical edge convention.  The recovered
            // anchor pair identifies the same edge used by the UI/protocol.
            recovered.opening_edge = find_nearest_edge_to_point(
                recovered, mid_x, mid_y);

            out_berth = recovered;
            debug.recovered_w = recovered.w;
            debug.recovered_l = recovered.l;
            debug.recovered_angle = recovered.angle;
            debug.status = AnchorRecoveryStatus::Success;
            return true;
        }

        bool recover_detected_berth_from_single_anchor(
            const std::vector<Eigen::Vector3d>& points,
            const Berth& detected_berth,
            Berth& out_berth,
            Eigen::Vector3d& out_anchor,
            int* candidate_count = nullptr,
            const Berth* search_roi_reference = nullptr,
            std::string* rejection_reason = nullptr) const {
            if (candidate_count) *candidate_count = 0;
            if (rejection_reason) rejection_reason->clear();
            if (!anchor_bus_.valid || !is_valid_berth_size(detected_berth)) {
                return false;
            }

            const Berth& roi_reference =
                search_roi_reference && is_valid_berth_size(*search_roi_reference)
                ? *search_roi_reference
                : detected_berth;
            Berth search_roi = expanded_line_roi(roi_reference);
            Eigen::Vector3d unused_second = Eigen::Vector3d::Zero();
            const int region_count = find_highest_regions_in_roi(
                search_roi, points, out_anchor, unused_second, true, candidate_count);
            if (region_count != 1 || out_anchor.z() <= anchor_min_height_m_
                || !point_is_near_anchor_bus(out_anchor)) {
                return false;
            }

            size_t stable_index = stable_history_berths_.size();
            if (single_anchor_has_stable_history(out_anchor, &stable_index)) {
                if (rejection_reason) {
                    *rejection_reason =
                        "anchor-overlaps-stable-history stable=#"
                        + std::to_string(stable_index + 1)
                        + " anchor=" + format_point_xy(out_anchor)
                        + " margin="
                        + std::to_string(stable_history_anchor_margin_m_)
                        + "m";
                }
                return false;
            }

            const double ux = anchor_bus_.ux;
            const double uy = anchor_bus_.uy;
            const double vx = -uy;
            const double vy = ux;
            const double center_dx = detected_berth.cx - anchor_bus_.start.x();
            const double center_dy = detected_berth.cy - anchor_bus_.start.y();
            const double center_projection = center_dx * ux + center_dy * uy;
            const double edge_mid_x = anchor_bus_.start.x() + center_projection * ux;
            const double edge_mid_y = anchor_bus_.start.y() + center_projection * uy;
            const double side_projection =
                (detected_berth.cx - edge_mid_x) * vx
                + (detected_berth.cy - edge_mid_y) * vy;
            const double side = side_projection >= 0.0 ? 1.0 : -1.0;

            Berth recovered = detected_berth;
            recovered.angle = std::atan2(uy, ux) * 180.0 / M_PI;
            while (recovered.angle < 0.0) recovered.angle += 180.0;
            while (recovered.angle >= 180.0) recovered.angle -= 180.0;
            recovered.cx = edge_mid_x + side * vx * recovered.l * 0.5;
            recovered.cy = edge_mid_y + side * vy * recovered.l * 0.5;
            recovered.opening_edge = find_nearest_edge_to_point(
                recovered, edge_mid_x, edge_mid_y);
            if (!is_valid_berth_size(recovered)) {
                return false;
            }

            out_berth = recovered;
            return true;
        }

        bool recover_berth_from_line_roi(const std::vector<Eigen::Vector3d>& points, Berth& out_berth) {
            if (!has_line_roi_ || !is_valid_berth_size(roi_berth_)) return false;

            const double half_w = roi_berth_.w / 2.0 + line_roi_margin_;
            const double half_l = roi_berth_.l / 2.0 + line_roi_margin_;
            const int img_w = std::max(10, int((half_w * 2.0) / line_roi_bev_res_) + 1);
            const int img_h = std::max(10, int((half_l * 2.0) / line_roi_bev_res_) + 1);

            cv::Mat bev_img = cv::Mat::zeros(img_h, img_w, CV_8UC1);
            double rad = roi_berth_.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);
            int roi_point_count = 0;

            for (const auto& pt : points) {
                if (!point_in_expanded_line_roi(pt, roi_berth_)) continue;

                double dx = pt.x() - roi_berth_.cx;
                double dy = pt.y() - roi_berth_.cy;
                double rx = dx * cos_a + dy * sin_a;
                double ry = -dx * sin_a + dy * cos_a;
                int px = int((rx + half_w) / line_roi_bev_res_);
                int py = int((ry + half_l) / line_roi_bev_res_);
                if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                    bev_img.at<uchar>(py, px) = 255;
                    roi_point_count++;
                }
            }

            if (roi_point_count < line_roi_min_points_) return false;

            cv::Mat line_img;
            cv::dilate(bev_img, line_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

            std::vector<cv::Vec4i> raw_lines;
            cv::HoughLinesP(line_img, raw_lines, 1.0, CV_PI / 180.0, line_roi_hough_threshold_,
                line_roi_min_line_length_ / line_roi_bev_res_, line_roi_max_line_gap_ / line_roi_bev_res_);
            if (raw_lines.empty()) return false;

            std::vector<LineCandidate> width_axis_lines;
            std::vector<LineCandidate> length_axis_lines;
            for (const auto& l : raw_lines) {
                double x1 = l[0] * line_roi_bev_res_ - half_w;
                double y1 = l[1] * line_roi_bev_res_ - half_l;
                double x2 = l[2] * line_roi_bev_res_ - half_w;
                double y2 = l[3] * line_roi_bev_res_ - half_l;
                double dx = x2 - x1;
                double dy = y2 - y1;
                double length = std::sqrt(dx * dx + dy * dy);
                if (length < line_roi_min_line_length_) continue;

                double angle = normalize_angle_180(std::atan2(dy, dx) * 180.0 / M_PI);
                LineCandidate candidate;
                candidate.angle = angle;
                candidate.length = length;
                candidate.mid_x = (x1 + x2) / 2.0;
                candidate.mid_y = (y1 + y2) / 2.0;
                candidate.x1 = x1;
                candidate.y1 = y1;
                candidate.x2 = x2;
                candidate.y2 = y2;

                if (angle_distance_180(angle, 0.0) <= line_roi_axis_tol_deg_) {
                    width_axis_lines.push_back(candidate);
                }
                else if (angle_distance_180(angle, 90.0) <= line_roi_axis_tol_deg_) {
                    length_axis_lines.push_back(candidate);
                }
            }

            if (width_axis_lines.empty() || length_axis_lines.empty()) return false;

            auto nearest_to_roi_center = [](const std::vector<LineCandidate>& lines, bool use_mid_x) {
                return *std::min_element(lines.begin(), lines.end(),
                    [use_mid_x](const LineCandidate& a, const LineCandidate& b) {
                        const double da = std::abs(use_mid_x ? a.mid_x : a.mid_y);
                        const double db = std::abs(use_mid_x ? b.mid_x : b.mid_y);
                        if (std::abs(da - db) > 1e-6) return da < db;
                        return a.length > b.length;
                    });
            };

            const LineCandidate width_edge = nearest_to_roi_center(width_axis_lines, false);
            const LineCandidate length_edge = nearest_to_roi_center(length_axis_lines, true);

            last_debug_lines_.clear();
            last_debug_lines_.push_back(local_line_to_world(width_edge.x1, width_edge.y1,
                width_edge.x2, width_edge.y2));
            last_debug_lines_.push_back(local_line_to_world(length_edge.x1, length_edge.y1,
                length_edge.x2, length_edge.y2));

            double local_cx = 0.0;
            if (std::abs(length_edge.mid_x - roi_berth_.w / 2.0) <
                std::abs(length_edge.mid_x + roi_berth_.w / 2.0)) {
                local_cx = length_edge.mid_x - roi_berth_.w / 2.0;
            }
            else {
                local_cx = length_edge.mid_x + roi_berth_.w / 2.0;
            }

            double local_cy = 0.0;
            if (std::abs(width_edge.mid_y - roi_berth_.l / 2.0) <
                std::abs(width_edge.mid_y + roi_berth_.l / 2.0)) {
                local_cy = width_edge.mid_y - roi_berth_.l / 2.0;
            }
            else {
                local_cy = width_edge.mid_y + roi_berth_.l / 2.0;
            }

            if (std::abs(local_cx) > line_roi_center_shift_max_ ||
                std::abs(local_cy) > line_roi_center_shift_max_) {
                return false;
            }

            Berth recovered;
            recovered.cx = local_cx * cos_a - local_cy * sin_a + roi_berth_.cx;
            recovered.cy = local_cx * sin_a + local_cy * cos_a + roi_berth_.cy;
            recovered.w = roi_berth_.w;
            recovered.l = roi_berth_.l;
            recovered.angle = roi_berth_.angle;
            recovered.opening_edge = roi_berth_.opening_edge;

            if (!is_valid_berth_size(recovered)) return false;
            out_berth = recovered;
            return true;
        }

        int find_highest_regions_in_roi(const Berth& b,
            const std::vector<Eigen::Vector3d>& points,
            Eigen::Vector3d& first_point,
            Eigen::Vector3d& second_point,
            bool use_height_limit = true,
            int* candidate_count = nullptr,
            bool require_anchor_bus = true) const {
            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);
            double half_w = b.w / 2.0 + anchor_recovery_roi_extra_margin_;
            double half_l = b.l / 2.0 + anchor_recovery_roi_extra_margin_;
            if (candidate_count) *candidate_count = 0;

            struct CandidatePoint {
                Eigen::Vector3d world;
                double z = 0.0;
            };

            std::vector<CandidatePoint> candidates;
            candidates.reserve(points.size());
            for (const auto& pt : points) {
                if (!std::isfinite(pt.z())) continue;
                if (use_height_limit && (pt.z() < z_min_ || pt.z() > z_max_)) continue;
                if (pt.z() <= anchor_min_height_m_) continue;
                if (require_anchor_bus && !point_is_near_anchor_bus(pt)) continue;
                double dx = pt.x() - b.cx;
                double dy = pt.y() - b.cy;
                double rx = dx * cos_a + dy * sin_a;
                double ry = -dx * sin_a + dy * cos_a;
                if (std::abs(rx) <= half_w && std::abs(ry) <= half_l) {
                    candidates.push_back({ pt, pt.z() });
                }
            }
            if (candidate_count) *candidate_count = static_cast<int>(candidates.size());

            if (candidates.empty()) return 0;

            std::vector<size_t> order(candidates.size());
            for (size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b_idx) {
                return candidates[a].z > candidates[b_idx].z;
            });

            auto build_region = [&](size_t seed_idx) {
                const auto& seed = candidates[seed_idx];
                double max_z = seed.z;
                double sum_x = 0.0;
                double sum_y = 0.0;
                int count = 0;

                for (const auto& c : candidates) {
                    double dx = c.world.x() - seed.world.x();
                    double dy = c.world.y() - seed.world.y();
                    if (dx * dx + dy * dy <= top_region_radius_ * top_region_radius_ &&
                        c.z >= max_z - top_region_top_band_) {
                        sum_x += c.world.x();
                        sum_y += c.world.y();
                        ++count;
                    }
                }

                if (count == 0) {
                    return clamp_point_to_berth_roi(b, seed.world);
                }
                return clamp_point_to_berth_roi(b, Eigen::Vector3d(sum_x / count, sum_y / count, max_z));
            };

            size_t first_idx = order.front();
            first_point = build_region(first_idx);

            bool found_separated = false;
            size_t second_idx = first_idx;
            for (size_t idx : order) {
                double dx = candidates[idx].world.x() - first_point.x();
                double dy = candidates[idx].world.y() - first_point.y();
                if (dx * dx + dy * dy >= top_region_min_separation_ * top_region_min_separation_) {
                    second_idx = idx;
                    found_separated = true;
                    break;
                }
            }

            first_point = clamp_point_to_berth_roi(b, first_point);
            if (!found_separated) {
                second_point = Eigen::Vector3d::Zero();
                return 1;
            }
            second_point = clamp_point_to_berth_roi(b, build_region(second_idx));
            return 2;
        }

        double berth_rotated_iou(const Berth& a, const Berth& b) const {
            if (!is_valid_berth_size(a) || !is_valid_berth_size(b)) {
                return 0.0;
            }

            const double area_a = a.w * a.l;
            const double area_b = b.w * b.l;
            if (area_a <= 1e-6 || area_b <= 1e-6) {
                return 0.0;
            }

            const cv::RotatedRect rect_a(
                cv::Point2f(static_cast<float>(a.cx), static_cast<float>(a.cy)),
                cv::Size2f(static_cast<float>(a.w), static_cast<float>(a.l)),
                static_cast<float>(a.angle));
            const cv::RotatedRect rect_b(
                cv::Point2f(static_cast<float>(b.cx), static_cast<float>(b.cy)),
                cv::Size2f(static_cast<float>(b.w), static_cast<float>(b.l)),
                static_cast<float>(b.angle));

            std::vector<cv::Point2f> intersection;
            const int intersect_type = cv::rotatedRectangleIntersection(rect_a, rect_b, intersection);
            if (intersect_type == cv::INTERSECT_NONE) {
                return 0.0;
            }

            double intersection_area = 0.0;
            if (intersect_type == cv::INTERSECT_FULL) {
                intersection_area = std::min(area_a, area_b);
            }
            else if (intersection.size() >= 3) {
                intersection_area = std::abs(cv::contourArea(intersection));
            }

            const double union_area = area_a + area_b - intersection_area;
            if (union_area <= 1e-6) {
                return 0.0;
            }
            return intersection_area / union_area;
        }

        bool is_duplicate_candidate(const Berth& candidate,
            const std::vector<Berth>& accepted) const {
            for (const Berth& b : accepted) {
                const double dx = candidate.cx - b.cx;
                const double dy = candidate.cy - b.cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < duplicate_strict_center_sep_) {
                    return true;
                }

                if (berth_rotated_iou(candidate, b) > duplicate_iou_threshold_) {
                    return true;
                }

                const double size_sep = 0.18 * std::min(
                    std::max(candidate.w, candidate.l),
                    std::max(b.w, b.l));
                const double min_sep = std::max(multi_candidate_min_center_sep_, size_sep);
                if (dist < min_sep &&
                    angle_distance_180(candidate.angle, b.angle) < duplicate_angle_threshold_deg_) {
                    return true;
                }
            }
            return false;
        }

        bool is_abnormal_near_history_candidate(const Berth& candidate,
            const Berth** matched_history,
            double* matched_center_dist,
            double* matched_iou) const {
            if (matched_history) *matched_history = nullptr;
            if (matched_center_dist) *matched_center_dist = std::numeric_limits<double>::infinity();
            if (matched_iou) *matched_iou = 0.0;
            if (tracked_berths_.empty()) {
                return false;
            }

            bool abnormal = false;
            double best_score = std::numeric_limits<double>::infinity();
            for (const Berth& tracked : tracked_berths_) {
                const double dx = candidate.cx - tracked.cx;
                const double dy = candidate.cy - tracked.cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                const double iou = berth_rotated_iou(candidate, tracked);
                if (dist >= abnormal_near_history_center_m_ &&
                    iou <= abnormal_near_history_iou_) {
                    continue;
                }

                abnormal = true;
                const double score = dist - 10.0 * iou;
                if (score < best_score) {
                    best_score = score;
                    if (matched_history) *matched_history = &tracked;
                    if (matched_center_dist) *matched_center_dist = dist;
                    if (matched_iou) *matched_iou = iou;
                }
            }

            return abnormal;
        }

        void suppress_abnormal_near_history_candidates(std::vector<Berth>& berths,
            std::vector<bool>& recovery_flags,
            std::vector<std::string>& debug_messages,
            std::vector<bool>* normal_expand_flags = nullptr) const {
            if (berths.empty() || tracked_berths_.empty()) {
                return;
            }

            std::vector<Berth> kept;
            std::vector<bool> kept_flags;
            std::vector<bool> kept_normal_expand_flags;
            kept.reserve(berths.size());
            kept_flags.reserve(recovery_flags.size());
            if (normal_expand_flags) {
                kept_normal_expand_flags.reserve(normal_expand_flags->size());
            }

            for (size_t i = 0; i < berths.size(); ++i) {
                const bool recovered = i < recovery_flags.size() && recovery_flags[i];
                if (recovered || has_matching_berth(berths[i], tracked_berths_)) {
                    kept.push_back(berths[i]);
                    kept_flags.push_back(recovered);
                    if (normal_expand_flags) {
                        kept_normal_expand_flags.push_back(
                            i < normal_expand_flags->size() && (*normal_expand_flags)[i]);
                    }
                    continue;
                }

                const Berth* history = nullptr;
                double center_dist = 0.0;
                double iou = 0.0;
                if (is_abnormal_near_history_candidate(
                        berths[i], &history, &center_dist, &iou)) {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(2)
                        << "[BerthDebug] suppress abnormal candidate near tracked berth #"
                        << (i + 1)
                        << " center_dist=" << center_dist
                        << " iou=" << iou
                        << " center_limit=" << abnormal_near_history_center_m_
                        << " iou_limit=" << abnormal_near_history_iou_
                        << " candidate{" << format_berth(berths[i]) << "}";
                    if (history) {
                        oss << " tracked{" << format_berth(*history) << "}";
                    }
                    debug_messages.push_back(oss.str());
                    continue;
                }

                kept.push_back(berths[i]);
                kept_flags.push_back(recovered);
                if (normal_expand_flags) {
                    kept_normal_expand_flags.push_back(
                        i < normal_expand_flags->size() && (*normal_expand_flags)[i]);
                }
            }

            berths = std::move(kept);
            recovery_flags = std::move(kept_flags);
            if (normal_expand_flags) {
                *normal_expand_flags = std::move(kept_normal_expand_flags);
            }
        }

        Berth align_recovered_expansion_to_anchor_edge(const Berth& anchor_ref,
            Berth expanded) const {
            if (!is_valid_berth_size(anchor_ref) || !is_valid_berth_size(expanded)) {
                return expanded;
            }

            int edge = anchor_ref.opening_edge;
            if (edge != 0 && edge != 2) {
                edge = find_nearest_opening_edge(anchor_ref);
            }
            if (edge != 0 && edge != 2) {
                return expanded;
            }

            const double rad = anchor_ref.angle * M_PI / 180.0;
            const double vx = -std::sin(rad);
            const double vy = std::cos(rad);
            const double edge_sign = edge == 0 ? -1.0 : 1.0;

            const double anchor_edge_cx = anchor_ref.cx + edge_sign * vx * anchor_ref.l * 0.5;
            const double anchor_edge_cy = anchor_ref.cy + edge_sign * vy * anchor_ref.l * 0.5;

            expanded.angle = anchor_ref.angle;
            expanded.opening_edge = edge;
            expanded.cx = anchor_edge_cx - edge_sign * vx * expanded.l * 0.5;
            expanded.cy = anchor_edge_cy - edge_sign * vy * expanded.l * 0.5;
            return expanded;
        }

        bool is_expansion_area_jump(const Berth& before,
            const Berth& after,
            bool recovered,
            std::string& reason) const {
            reason.clear();
            const double before_area = before.w * before.l;
            const double after_area = after.w * after.l;
            if (!std::isfinite(before_area) || !std::isfinite(after_area) ||
                before_area <= 1e-6 || after_area <= 1e-6) {
                return false;
            }

            const double allowed_ratio = recovered
                ? recovered_expand_area_jump_max_ratio_
                : expand_area_jump_max_ratio_;
            const double expand_ratio = after_area / before_area;
            if (expand_ratio > allowed_ratio) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "expand_area_ratio=" << expand_ratio
                    << " limit=" << allowed_ratio;
                reason = oss.str();
                return true;
            }

            double best_score = std::numeric_limits<double>::infinity();
            const Berth* matched_track = nullptr;
            for (const Berth& tracked : tracked_berths_) {
                const double dx = before.cx - tracked.cx;
                const double dy = before.cy - tracked.cy;
                const double dist = std::sqrt(dx * dx + dy * dy);
                const double angle_diff = angle_distance_180(before.angle, tracked.angle);
                if (dist < 8.0 && angle_diff < 35.0) {
                    const double score = dist + 0.05 * angle_diff;
                    if (score < best_score) {
                        best_score = score;
                        matched_track = &tracked;
                    }
                }
            }

            if (matched_track) {
                const double tracked_area = matched_track->w * matched_track->l;
                if (std::isfinite(tracked_area) && tracked_area > 1e-6) {
                    const double history_ratio = after_area / tracked_area;
                    if (history_ratio > expand_history_area_jump_max_ratio_) {
                        std::ostringstream oss;
                        oss << std::fixed << std::setprecision(2)
                            << "history_area_ratio=" << history_ratio
                            << " limit=" << expand_history_area_jump_max_ratio_;
                        reason = oss.str();
                        return true;
                    }
                }
            }

            return false;
        }

        void apply_final_berth_nms(std::vector<Berth>& berths,
            std::vector<bool>& recovery_flags,
            std::vector<std::string>& debug_messages) const {
            if (berths.size() <= 1) {
                return;
            }

            std::vector<size_t> order;
            order.reserve(berths.size());
            for (size_t i = 0; i < berths.size(); ++i) {
                if (i < recovery_flags.size() && recovery_flags[i]) {
                    order.push_back(i);
                }
            }
            for (size_t i = 0; i < berths.size(); ++i) {
                if (!(i < recovery_flags.size() && recovery_flags[i])) {
                    order.push_back(i);
                }
            }

            std::vector<Berth> kept;
            std::vector<bool> kept_flags;
            kept.reserve(berths.size());
            kept_flags.reserve(berths.size());
            for (size_t idx : order) {
                if (is_duplicate_candidate(berths[idx], kept)) {
                    debug_messages.push_back(
                        "[BerthDebug] final NMS suppressed berth #"
                        + std::to_string(idx + 1)
                        + " berth{" + format_berth(berths[idx]) + "}");
                    continue;
                }
                kept.push_back(berths[idx]);
                kept_flags.push_back(idx < recovery_flags.size() && recovery_flags[idx]);
            }

            berths = std::move(kept);
            recovery_flags = std::move(kept_flags);
        }

        void reorder_berths_by_tracked_order(std::vector<Berth>& berths,
            std::vector<bool>& recovery_flags,
            std::vector<std::string>& debug_messages) const {
            if (berths.size() <= 1 || tracked_berths_.empty()) {
                return;
            }

            std::vector<Berth> ordered;
            std::vector<bool> ordered_flags;
            std::vector<bool> used(berths.size(), false);
            std::vector<int> old_indices;
            ordered.reserve(berths.size());
            ordered_flags.reserve(berths.size());
            old_indices.reserve(berths.size());

            for (const Berth& tracked : tracked_berths_) {
                double best_score = std::numeric_limits<double>::infinity();
                size_t best_idx = berths.size();
                for (size_t i = 0; i < berths.size(); ++i) {
                    if (used[i]) {
                        continue;
                    }
                    const double dx = berths[i].cx - tracked.cx;
                    const double dy = berths[i].cy - tracked.cy;
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    const double angle_diff = angle_distance_180(berths[i].angle, tracked.angle);
                    const double size_diff = std::abs(berths[i].w - tracked.w)
                        + std::abs(berths[i].l - tracked.l);
                    if (dist < 8.0 && angle_diff < 35.0 && size_diff < 8.0) {
                        const double score = dist + 0.05 * angle_diff + 0.25 * size_diff;
                        if (score < best_score) {
                            best_score = score;
                            best_idx = i;
                        }
                    }
                }

                if (best_idx < berths.size()) {
                    used[best_idx] = true;
                    ordered.push_back(berths[best_idx]);
                    ordered_flags.push_back(
                        best_idx < recovery_flags.size() && recovery_flags[best_idx]);
                    old_indices.push_back(static_cast<int>(best_idx));
                }
            }

            for (size_t i = 0; i < berths.size(); ++i) {
                if (used[i]) {
                    continue;
                }
                ordered.push_back(berths[i]);
                ordered_flags.push_back(i < recovery_flags.size() && recovery_flags[i]);
                old_indices.push_back(static_cast<int>(i));
            }

            bool changed = ordered.size() == berths.size();
            if (changed) {
                for (size_t i = 0; i < old_indices.size(); ++i) {
                    if (old_indices[i] != static_cast<int>(i)) {
                        changed = true;
                        break;
                    }
                    changed = false;
                }
            }
            if (changed) {
                std::ostringstream oss;
                oss << "[BerthDebug] reorder berths by tracked order old_indices=";
                for (size_t i = 0; i < old_indices.size(); ++i) {
                    if (i > 0) {
                        oss << ",";
                    }
                    oss << (old_indices[i] + 1);
                }
                debug_messages.push_back(oss.str());
            }

            berths = std::move(ordered);
            recovery_flags = std::move(ordered_flags);
        }

        void stabilize_output_berth_sizes(std::vector<Berth>& berths,
            const std::vector<bool>& recovery_flags,
            std::vector<std::string>& debug_messages) const {
            if (berths.empty() || tracked_berths_.empty()) {
                return;
            }

            std::vector<bool> used_tracks(tracked_berths_.size(), false);
            for (size_t i = 0; i < berths.size(); ++i) {
                double best_score = std::numeric_limits<double>::infinity();
                size_t best_idx = tracked_berths_.size();
                for (size_t j = 0; j < tracked_berths_.size(); ++j) {
                    if (used_tracks[j]) {
                        continue;
                    }
                    const double dx = berths[i].cx - tracked_berths_[j].cx;
                    const double dy = berths[i].cy - tracked_berths_[j].cy;
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    const double angle_diff = angle_distance_180(berths[i].angle, tracked_berths_[j].angle);
                    if (dist < 8.0 && angle_diff < 35.0) {
                        const double score = dist + 0.05 * angle_diff;
                        if (score < best_score) {
                            best_score = score;
                            best_idx = j;
                        }
                    }
                }

                if (best_idx >= tracked_berths_.size()) {
                    continue;
                }
                used_tracks[best_idx] = true;

                const Berth before = berths[i];
                const Berth& tracked = tracked_berths_[best_idx];
                if (i < recovery_flags.size() && recovery_flags[i]) {
                    continue;
                }
                else {
                    const bool tracked_size_can_lock =
                        tracked.w >= tracked_size_lock_min_m_ &&
                        tracked.w <= tracked_size_lock_max_m_ &&
                        tracked.l >= tracked_size_lock_min_m_ &&
                        tracked.l <= tracked_size_lock_max_m_;
                    if (std::isfinite(tracked.w) && tracked.w > 1e-6) {
                        if (std::abs(berths[i].w - tracked.w) <= detected_size_update_max_jump_m_) {
                            berths[i].w = (1.0 - detected_width_update_alpha_) * tracked.w
                                + detected_width_update_alpha_ * berths[i].w;
                        }
                        else if (tracked_size_can_lock) {
                            berths[i].w = tracked.w;
                        }
                    }
                    if (std::isfinite(tracked.l) && tracked.l > 1e-6) {
                        if (std::abs(berths[i].l - tracked.l) <= detected_size_update_max_jump_m_) {
                            berths[i].l = (1.0 - detected_length_update_alpha_) * tracked.l
                                + detected_length_update_alpha_ * berths[i].l;
                        }
                        else if (tracked_size_can_lock) {
                            berths[i].l = tracked.l;
                        }
                    }
                }

                if (std::abs(berths[i].w - before.w) > 1e-3 ||
                    std::abs(berths[i].l - before.l) > 1e-3) {
                    debug_messages.push_back(
                        "[BerthDebug] stabilized berth size before{"
                        + format_berth(before)
                        + "} after{" + format_berth(berths[i])
                        + "} ref{" + format_berth(tracked) + "}");
                }
            }
        }

        void smooth_output_berth_sizes_by_history(std::vector<Berth>& berths,
            const std::vector<bool>& recovery_flags,
            std::vector<std::string>& debug_messages) const {
            const size_t required_frames = static_cast<size_t>(
                std::max(1, size_smooth_history_frames_));
            if (berths.empty() || output_size_history_.size() < required_frames) {
                return;
            }

            const double max_ratio = std::max(1.0, size_smooth_max_ratio_);
            for (size_t i = 0; i < berths.size(); ++i) {
                double sum_w = 0.0;
                double sum_l = 0.0;
                int match_count = 0;

                for (const auto& frame_berths : output_size_history_) {
                    const size_t matched_idx = find_matching_berth_index(berths[i], frame_berths);
                    if (matched_idx >= frame_berths.size()) {
                        continue;
                    }

                    const Berth& history = frame_berths[matched_idx];
                    if (!is_valid_berth_size(history)) {
                        continue;
                    }
                    sum_w += history.w;
                    sum_l += history.l;
                    ++match_count;
                }

                if (match_count <= 0) {
                    continue;
                }

                const double avg_w = sum_w / match_count;
                const double avg_l = sum_l / match_count;
                if (!std::isfinite(avg_w) || !std::isfinite(avg_l) ||
                    avg_w <= 1e-6 || avg_l <= 1e-6) {
                    continue;
                }

                const Berth before = berths[i];
                bool changed = false;
                if (berths[i].w > avg_w * max_ratio) {
                    berths[i].w = avg_w;
                    changed = true;
                }
                if (berths[i].l > avg_l * max_ratio) {
                    berths[i].l = avg_l;
                    changed = true;
                }

                if (changed) {
                    const bool is_recovered =
                        i < recovery_flags.size() && recovery_flags[i];
                    if (is_recovered) {
                        berths[i] = align_recovered_expansion_to_anchor_edge(before, berths[i]);
                    }

                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(2)
                        << "[BerthDebug] multi-frame size smoothing berth #"
                        << (i + 1)
                        << " history_frames=" << output_size_history_.size()
                        << " matched_frames=" << match_count
                        << " avg_size=" << avg_w << "x" << avg_l
                        << " max_ratio=" << max_ratio
                        << " recovered=" << (is_recovered ? 1 : 0)
                        << " before{" << format_berth(before)
                        << "} after{" << format_berth(berths[i]) << "}";
                    debug_messages.push_back(oss.str());
                }
            }
        }

        void append_output_size_history(const std::vector<Berth>& berths) {
            if (berths.empty()) {
                return;
            }

            output_size_history_.push_back(berths);
            const size_t max_frames = static_cast<size_t>(
                std::max(1, size_smooth_history_frames_));
            while (output_size_history_.size() > max_frames) {
                output_size_history_.pop_front();
            }
        }

        std::vector<Berth> process_marina_berths(const std::vector<Eigen::Vector3d>& points) {
            last_morph_connected_debug_.release();
            last_morph_full_debug_.release();
            int img_w = (roi_x_max_ - roi_x_min_) / bev_res_;
            int img_h = (roi_y_max_ - roi_y_min_) / bev_res_;
            cv::Mat bev_img = cv::Mat::zeros(img_h, img_w, CV_8UC1);

            for (const auto& pt : points) {
                if (pt.z() > z_min_ && pt.z() < z_max_ &&
                    pt.x() > roi_x_min_ && pt.x() < roi_x_max_ &&
                    pt.y() > roi_y_min_ && pt.y() < roi_y_max_) {
                    int px = (pt.x() - roi_x_min_) / bev_res_;
                    int py = (pt.y() - roi_y_min_) / bev_res_;
                    if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
                        bev_img.at<uchar>(py, px) = 255;
                    }
                }
            }

            cv::Mat filtered_bev, dilated_bev;
            cv::morphologyEx(bev_img, filtered_bev, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
            cv::dilate(filtered_bev, dilated_bev, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

            int bridge_width = berth_width_max_ / bev_res_ + 1;
            cv::Mat closed_h, closed_v, closed_comb;
            cv::morphologyEx(dilated_bev, closed_h, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(bridge_width, 1)));
            cv::morphologyEx(dilated_bev, closed_v, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, bridge_width)));
            cv::bitwise_or(closed_h, closed_v, closed_comb);

            cv::Mat isolated;
            cv::subtract(closed_comb, dilated_bev, isolated);
            cv::morphologyEx(isolated, isolated, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21)));

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(isolated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            last_morph_full_debug_ =
                build_morph_full_debug_image(bev_img, filtered_bev, dilated_bev, closed_comb, isolated);
            last_morph_connected_debug_ =
                build_morph_connected_debug_image(bev_img, isolated, contours);

            struct Candidate {
                Berth berth;
                double area = 0.0;
            };
            std::vector<Candidate> candidates;
            candidates.reserve(contours.size());

            for (const auto& contour : contours) {
                const double area = cv::contourArea(contour);
                if (area <= 5.0) {
                    cv::RotatedRect small_rect = cv::minAreaRect(contour);
                    draw_rotated_rect_on_image(
                        last_morph_connected_debug_, small_rect, cv::Scalar(0, 0, 180), 1);
                    continue;
                }

                cv::RotatedRect rect = cv::minAreaRect(contour);
                double w_px = rect.size.width;
                double h_px = rect.size.height;
                const double angle =
                    berth_geometry::commonWidthAxisAngle(
                        w_px, h_px, rect.angle);

                double w_3d = w_px * bev_res_;
                double h_3d = h_px * bev_res_;
                const double berth_w = std::min(w_3d, h_3d);
                const double berth_l = std::max(w_3d, h_3d);
                Berth berth{
                    rect.center.x * bev_res_ + roi_x_min_,
                    rect.center.y * bev_res_ + roi_y_min_,
                    berth_w,
                    berth_l,
                    angle
                };

                if (is_valid_berth_size(berth)) {
                    berth.opening_edge = find_sparse_opening_edge(berth, points);
                    candidates.push_back({berth, area});
                    draw_rotated_rect_on_image(
                        last_morph_connected_debug_, rect, cv::Scalar(0, 255, 0), 2);
                }
                else {
                    draw_rotated_rect_on_image(
                        last_morph_connected_debug_, rect, cv::Scalar(0, 0, 255), 1);
                }
            }

            std::sort(candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) {
                    return a.area > b.area;
                });

            std::vector<Berth> berths;
            berths.reserve(std::min<int>(max_berth_candidates_, static_cast<int>(candidates.size())));
            for (const Candidate& candidate : candidates) {
                if (static_cast<int>(berths.size()) >= max_berth_candidates_) {
                    break;
                }
                if (is_duplicate_candidate(candidate.berth, berths)) {
                    continue;
                }
                berths.push_back(candidate.berth);
            }

            return berths;
        }

        Berth expand_berth_to_points(Berth b, const std::vector<Eigen::Vector3d>& points, double step = 0.2, double max_exp = 5.0, int threshold = 5) const {
            std::vector<Eigen::Vector2d> pts_2d;
            for (const auto& pt : points) {
                if (pt.z() > z_min_ && pt.z() < z_max_) pts_2d.push_back({ pt.x(), pt.y() });
            }
            if (pts_2d.empty()) return b;

            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);

            double bound_x_pos = b.w / 2.0, bound_x_neg = -b.w / 2.0;
            double bound_y_pos = b.l / 2.0, bound_y_neg = -b.l / 2.0;
            const bool width_can_expand =
                berth_recovery::canExpandDimension(b.w, step);
            const bool length_can_expand =
                berth_recovery::canExpandDimension(b.l, step);
            bool exp_x_pos = width_can_expand, exp_x_neg = width_can_expand;
            bool exp_y_pos = length_can_expand, exp_y_neg = length_can_expand;
            bool hit_x_pos = false, hit_x_neg = false, hit_y_pos = false, hit_y_neg = false;

            for (int i = 0; i < int(max_exp / step); ++i) {
                if (!exp_x_pos && !exp_x_neg && !exp_y_pos && !exp_y_neg) break;

                int c_xp = 0, c_xn = 0, c_yp = 0, c_yn = 0;
                for (const auto& p : pts_2d) {
                    double rx = (p.x() - b.cx) * cos_a + (p.y() - b.cy) * sin_a;
                    double ry = -(p.x() - b.cx) * sin_a + (p.y() - b.cy) * cos_a;

                    if (exp_x_pos && rx > bound_x_pos && rx <= bound_x_pos + step && ry >= bound_y_neg && ry <= bound_y_pos) c_xp++;
                    if (exp_x_neg && rx < bound_x_neg && rx >= bound_x_neg - step && ry >= bound_y_neg && ry <= bound_y_pos) c_xn++;
                    if (exp_y_pos && ry > bound_y_pos && ry <= bound_y_pos + step && rx >= bound_x_neg && rx <= bound_x_pos) c_yp++;
                    if (exp_y_neg && ry < bound_y_neg && ry >= bound_y_neg - step && rx >= bound_x_neg && rx <= bound_x_pos) c_yn++;
                }

                if (exp_x_pos) {
                    if (c_xp >= threshold) {
                        exp_x_pos = false;
                        hit_x_pos = true;
                    } else if (berth_recovery::canExpandDimension(
                                   bound_x_pos - bound_x_neg, step)) {
                        bound_x_pos += step;
                    } else {
                        exp_x_pos = false;
                    }
                }
                if (exp_x_neg) {
                    if (c_xn >= threshold) {
                        exp_x_neg = false;
                        hit_x_neg = true;
                    } else if (berth_recovery::canExpandDimension(
                                   bound_x_pos - bound_x_neg, step)) {
                        bound_x_neg -= step;
                    } else {
                        exp_x_neg = false;
                    }
                }
                if (exp_y_pos) {
                    if (c_yp >= threshold) {
                        exp_y_pos = false;
                        hit_y_pos = true;
                    } else if (berth_recovery::canExpandDimension(
                                   bound_y_pos - bound_y_neg, step)) {
                        bound_y_pos += step;
                    } else {
                        exp_y_pos = false;
                    }
                }
                if (exp_y_neg) {
                    if (c_yn >= threshold) {
                        exp_y_neg = false;
                        hit_y_neg = true;
                    } else if (berth_recovery::canExpandDimension(
                                   bound_y_pos - bound_y_neg, step)) {
                        bound_y_neg -= step;
                    } else {
                        exp_y_neg = false;
                    }
                }
            }

            if (!hit_x_pos) bound_x_pos = b.w / 2.0;
            if (!hit_x_neg) bound_x_neg = -b.w / 2.0;
            if (!hit_y_pos) bound_y_pos = b.l / 2.0;
            if (!hit_y_neg) bound_y_neg = -b.l / 2.0;

            double local_cx = (bound_x_pos + bound_x_neg) / 2.0;
            double local_cy = (bound_y_pos + bound_y_neg) / 2.0;

            Berth new_b;
            new_b.cx = local_cx * cos_a - local_cy * sin_a + b.cx;
            new_b.cy = local_cx * sin_a + local_cy * cos_a + b.cy;
            new_b.w = bound_x_pos - bound_x_neg;
            new_b.l = bound_y_pos - bound_y_neg;
            new_b.angle = b.angle;
            new_b.opening_edge = b.opening_edge;
            return new_b;
        }

        Berth enforce_free_space(Berth b, const std::vector<Eigen::Vector3d>& points, int threshold = 20, double push_step = 0.1) {
            std::vector<Eigen::Vector2d> pts_2d;
            for (const auto& pt : points) {
                if (pt.z() > z_min_ && pt.z() < z_max_) pts_2d.push_back({ pt.x(), pt.y() });
            }
            if (pts_2d.empty()) return b;

            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);

            for (int iter = 0; iter < 10; ++iter) {
                std::vector<Eigen::Vector2d> inside;
                for (const auto& p : pts_2d) {
                    double rx = (p.x() - b.cx) * cos_a + (p.y() - b.cy) * sin_a;
                    double ry = -(p.x() - b.cx) * sin_a + (p.y() - b.cy) * cos_a;
                    if (std::abs(rx) < (b.w / 2.0 - 0.1) && std::abs(ry) < (b.l / 2.0 - 0.1)) {
                        inside.push_back({ rx, ry });
                    }
                }

                if (inside.size() < threshold) break;

                double sum_x = 0, sum_y = 0;
                for (auto& p : inside) { sum_x += p.x(); sum_y += p.y(); }
                double mean_x = sum_x / inside.size();
                double mean_y = sum_y / inside.size();

                double push_x = (std::abs(mean_x) > 0.05) ? ((mean_x > 0 ? -1 : 1) * push_step) : 0;
                double push_y = (std::abs(mean_y) > 0.05) ? ((mean_y > 0 ? -1 : 1) * push_step) : 0;

                b.cx += push_x * cos_a - push_y * sin_a;
                b.cy += push_x * sin_a + push_y * cos_a;
            }
            return b;
        }
    };
} // namespace usv

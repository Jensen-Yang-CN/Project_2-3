#pragma once

#define _USE_MATH_DEFINES

#include "common_types_extended.h"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace usv {

struct LineEdge {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    double length = 0.0;
    double angle = 0.0;
    double distance_to_sensor = 0.0;
    double support_z_mean = std::numeric_limits<double>::quiet_NaN();
    double support_region_extent_m = 0.0;
    int support_points = 0;
};

struct LineBerth {
    double cx = 0.0;
    double cy = 0.0;
    double length = 0.0;
    double width = 3.5;
    double angle = 0.0;
    int inner_point_count = 0;
    int berth_side_point_count = 0;
    LineEdge edge;
};

struct LineBerthMeasureResult {
    double timestamp = 0.0;
    bool is_detected = false;
    bool is_using_memory = false;
    int point_count = 0;
    LineBerth berth;
    std::vector<LineBerth> berths;
    cv::Mat debug_view;
    cv::Mat debug_stages;
};

class LineBerthDetector {
public:
    void init() {
        known_dam_edges_.clear();
    }

    LineBerthMeasureResult process(const LidarFrame& lidar) {
        LineBerthMeasureResult result;
        result.timestamp = lidar.timestamp;
        result.point_count = static_cast<int>(lidar.points.size());

        std::vector<Eigen::Vector3d> points;
        points.reserve(lidar.points.size());
        for (const auto& pt : lidar.points) {
            points.emplace_back(pt.x, pt.y, pt.z);
        }

        std::vector<LineBerth> detected_berths;
        bool found = detect_line_berths(points, detected_berths, result.debug_view,
                                        result.debug_stages);

        if (found) {
            result.is_detected = true;
            result.is_using_memory = false;
            result.berths = detected_berths;
            if (!result.berths.empty()) {
                result.berth = result.berths.front();
            }
            for (const LineBerth& berth : result.berths) {
                draw_berth_overlay(result.debug_view, berth, cv::Scalar(0, 80, 255));
            }
        }

        draw_status(result);
        return result;
    }

private:
    struct Candidate {
        LineEdge edge;
        cv::Vec4i line_px;
        double quality_score = 0.0;
        double score = 0.0;
    };

    double z_min_ = -2.0;
    double z_max_ = 2.0;
    double roi_x_min_ = -105.0;
    double roi_x_max_ = 105.0;
    double roi_y_min_ = -105.0;
    double roi_y_max_ = 105.0;
    double bev_res_ = 0.2;
    double fixed_berth_width_m_ = 3.5;
    double length_extend_step_m_ = 0.2;
    double length_extend_max_m_ = 10.0;
    double length_endpoint_perp_half_m_ = 1.0;
    double length_endpoint_along_band_m_ = 0.25;
    int length_endpoint_min_hits_ = 1;
    double min_line_length_m_ = 12.0;
    double max_line_gap_m_ = 1.4;
    double berth_distance_min_m_ = 8.0;
    double berth_distance_max_m_ = 35.0;
    double self_exclusion_x_min_ = -22.0;
    double self_exclusion_x_max_ = 12.0;
    double self_exclusion_y_min_ = -7.0;
    double self_exclusion_y_max_ = 7.0;
    double line_support_band_m_ = 0.45;
    double max_support_z_mean_m_ = 0.0;
    double outer_line_distance_weight_ = 1.6;
    double outer_parallel_angle_tol_deg_ = 8.0;
    double outer_parallel_lateral_window_m_ = 6.0;
    double outer_parallel_min_overlap_ = 0.35;
    double dam_min_region_extent_m_ = 50.0;
    double dam_memory_lateral_window_m_ = 5.0;
    double dam_memory_min_overlap_ = 0.15;
    double min_component_display_area_m2_ = 3.0;
    int component_close_kernel_px_ = 13;
    int component_dilate_kernel_px_ = 9;
    int component_dilate_iterations_ = 2;
    int line_preclose_kernel_px_ = 5;
    int line_predilate_kernel_px_ = 3;
    int line_predilate_iterations_ = 1;
    int line_outer_edge_erode_kernel_px_ = 3;
    int min_support_points_ = 12;
    int max_known_dam_edges_ = 12;
    int max_line_berth_outputs_ = 2;
    double line_berth_nms_center_m_ = 4.0;
    double line_berth_nms_iou_ = 0.20;
    double berth_side_check_depth_m_ = 3.5;
    double berth_side_check_margin_m_ = 0.2;
    int berth_side_max_points_ = 80;
    std::vector<LineEdge> known_dam_edges_;

    int image_width() const {
        return static_cast<int>(std::round((roi_x_max_ - roi_x_min_) / bev_res_));
    }

    int image_height() const {
        return static_cast<int>(std::round((roi_y_max_ - roi_y_min_) / bev_res_));
    }

    bool world_to_pixel(double x, double y, cv::Point& out) const {
        const int col = static_cast<int>(std::round((x - roi_x_min_) / bev_res_));
        const int row = image_height() - 1 -
                        static_cast<int>(std::round((y - roi_y_min_) / bev_res_));
        if (col < 0 || col >= image_width() || row < 0 || row >= image_height()) {
            return false;
        }
        out = cv::Point(col, row);
        return true;
    }

    Eigen::Vector2d pixel_to_world(int col, int row) const {
        const double x = roi_x_min_ + col * bev_res_;
        const double y = roi_y_min_ + (image_height() - 1 - row) * bev_res_;
        return {x, y};
    }

    static double normalize_angle(double angle_deg) {
        angle_deg = std::fmod(angle_deg, 180.0);
        if (angle_deg < 0.0) {
            angle_deg += 180.0;
        }
        return angle_deg;
    }

    bool is_self_vessel_area(double x, double y) const {
        return x >= self_exclusion_x_min_ && x <= self_exclusion_x_max_ &&
               y >= self_exclusion_y_min_ && y <= self_exclusion_y_max_;
    }

    bool is_valid_detection_point(const Eigen::Vector3d& pt) const {
        return pt.z() >= z_min_ && pt.z() <= z_max_ &&
               !is_self_vessel_area(pt.x(), pt.y());
    }

    void draw_bev_grid(cv::Mat& image) const {
        for (double x = std::ceil(roi_x_min_ / 10.0) * 10.0; x <= roi_x_max_; x += 10.0) {
            cv::Point p1, p2;
            if (world_to_pixel(x, roi_y_min_, p1) && world_to_pixel(x, roi_y_max_, p2)) {
                cv::line(image, p1, p2, cv::Scalar(35, 35, 35), 1, cv::LINE_AA);
            }
        }
        for (double y = std::ceil(roi_y_min_ / 10.0) * 10.0; y <= roi_y_max_; y += 10.0) {
            cv::Point p1, p2;
            if (world_to_pixel(roi_x_min_, y, p1) && world_to_pixel(roi_x_max_, y, p2)) {
                cv::line(image, p1, p2, cv::Scalar(35, 35, 35), 1, cv::LINE_AA);
            }
        }

        cv::Point origin;
        if (world_to_pixel(0.0, 0.0, origin)) {
            cv::drawMarker(image, origin, cv::Scalar(255, 80, 80), cv::MARKER_CROSS,
                           16, 2, cv::LINE_AA);
            cv::putText(image, "radar", origin + cv::Point(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 120, 120),
                        1, cv::LINE_AA);
        }
    }

    void draw_self_exclusion(cv::Mat& image) const {
        cv::Point p1, p2;
        if (!world_to_pixel(self_exclusion_x_min_, self_exclusion_y_min_, p1) ||
            !world_to_pixel(self_exclusion_x_max_, self_exclusion_y_max_, p2)) {
            return;
        }

        cv::Rect rect(cv::Point(std::min(p1.x, p2.x), std::min(p1.y, p2.y)),
                      cv::Point(std::max(p1.x, p2.x), std::max(p1.y, p2.y)));
        cv::rectangle(image, rect, cv::Scalar(70, 40, 40), 1, cv::LINE_AA);
        cv::putText(image, "ignored self-vessel zone", rect.tl() + cv::Point(6, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(100, 100, 180), 1,
                    cv::LINE_AA);
    }

    void draw_connected_components(cv::Mat& image, const cv::Mat& stats) const {
        if (image.empty() || stats.empty()) {
            return;
        }

        for (int label = 1; label < stats.rows; ++label) {
            const int left = stats.at<int>(label, cv::CC_STAT_LEFT);
            const int top = stats.at<int>(label, cv::CC_STAT_TOP);
            const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
            const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
            const int area_px = stats.at<int>(label, cv::CC_STAT_AREA);

            const double area_m2 = area_px * bev_res_ * bev_res_;
            if (area_m2 < min_component_display_area_m2_) {
                continue;
            }

            const double extent_m = std::hypot(width * bev_res_, height * bev_res_);
            const bool dam_like = extent_m >= dam_min_region_extent_m_;
            const cv::Scalar color = dam_like ? cv::Scalar(0, 180, 255)
                                              : cv::Scalar(180, 140, 70);
            const int thickness = dam_like ? 2 : 1;

            cv::Rect rect(left, top, width, height);
            rect &= cv::Rect(0, 0, image.cols, image.rows);
            if (rect.empty()) {
                continue;
            }

            cv::rectangle(image, rect, color, thickness, cv::LINE_AA);
            std::ostringstream label_text;
            label_text << "cc " << std::fixed << std::setprecision(1) << extent_m << "m";
            if (dam_like) {
                label_text << " dam";
            }

            const cv::Point text_pos(rect.x + 3, std::max(14, rect.y + 14));
            cv::putText(image, label_text.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX,
                        0.38, color, 1, cv::LINE_AA);
        }
    }

    cv::Mat make_stage_panel(const cv::Mat& source, const std::string& label) const {
        constexpr int panel_size = 420;
        constexpr int label_height = 28;

        cv::Mat color;
        if (source.empty()) {
            color = cv::Mat::zeros(panel_size, panel_size, CV_8UC3);
        } else if (source.channels() == 1) {
            cv::cvtColor(source, color, cv::COLOR_GRAY2BGR);
        } else {
            source.copyTo(color);
        }

        cv::Mat resized;
        cv::resize(color, resized, cv::Size(panel_size, panel_size), 0.0, 0.0,
                   cv::INTER_AREA);
        cv::copyMakeBorder(resized, resized, label_height, 0, 0, 0,
                           cv::BORDER_CONSTANT, cv::Scalar(18, 18, 18));
        cv::putText(resized, label, cv::Point(8, 19), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
        return resized;
    }

    cv::Mat build_stage_debug_mosaic(const cv::Mat& bev,
                                     const cv::Mat& opened,
                                     const cv::Mat& closed,
                                     const cv::Mat& line_mask,
                                     const cv::Mat& component_mask,
                                     const cv::Mat& edge_mask,
                                     const cv::Mat& annotated) const {
        std::vector<cv::Mat> panels;
        panels.push_back(make_stage_panel(bev, "raw bev"));
        panels.push_back(make_stage_panel(opened, "morph open"));
        panels.push_back(make_stage_panel(closed, "morph close"));
        panels.push_back(make_stage_panel(line_mask, "line mask"));
        panels.push_back(make_stage_panel(component_mask, "component mask"));
        panels.push_back(make_stage_panel(edge_mask, "outer edge mask"));
        panels.push_back(make_stage_panel(annotated, "components + lines + berth"));

        while (panels.size() % 3 != 0) {
            panels.push_back(cv::Mat::zeros(panels.front().size(), panels.front().type()));
        }

        std::vector<cv::Mat> rows;
        for (std::size_t i = 0; i < panels.size(); i += 3) {
            cv::Mat row;
            cv::hconcat(std::vector<cv::Mat>{panels[i], panels[i + 1], panels[i + 2]}, row);
            rows.push_back(row);
        }

        cv::Mat mosaic;
        cv::vconcat(rows, mosaic);
        return mosaic;
    }


    cv::Mat build_bev(const std::vector<Eigen::Vector3d>& points) const {
        cv::Mat bev = cv::Mat::zeros(image_height(), image_width(), CV_8UC1);
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }
            cv::Point px;
            if (world_to_pixel(pt.x(), pt.y(), px)) {
                bev.at<uchar>(px) = 255;
            }
        }
        return bev;
    }

    bool detect_line_berths(const std::vector<Eigen::Vector3d>& points,
                            std::vector<LineBerth>& out_berths,
                            cv::Mat& debug_view,
                            cv::Mat& debug_stages) {
        out_berths.clear();
        cv::Mat bev = build_bev(points);

        cv::Mat opened;
        cv::morphologyEx(bev, opened, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

        cv::Mat closed;
        cv::morphologyEx(opened, closed, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::Mat line_mask;
        cv::morphologyEx(
            closed, line_mask, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT,
                                      cv::Size(line_preclose_kernel_px_,
                                               line_preclose_kernel_px_)));
        cv::dilate(line_mask, line_mask,
                   cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(line_predilate_kernel_px_,
                                                      line_predilate_kernel_px_)),
                   cv::Point(-1, -1), line_predilate_iterations_);
        cv::Mat component_mask;
        cv::morphologyEx(
            closed, component_mask, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT,
                                      cv::Size(component_close_kernel_px_,
                                               component_close_kernel_px_)));
        cv::dilate(component_mask, component_mask,
                   cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(component_dilate_kernel_px_,
                                                      component_dilate_kernel_px_)),
                   cv::Point(-1, -1), component_dilate_iterations_);
        cv::Mat component_labels;
        cv::Mat component_stats;
        cv::Mat component_centroids;
        cv::connectedComponentsWithStats(component_mask, component_labels, component_stats,
                                         component_centroids, 8, CV_32S);

        cv::Mat edge_mask;
        cv::Mat eroded_line_mask;
        cv::erode(line_mask, eroded_line_mask,
                  cv::getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(line_outer_edge_erode_kernel_px_,
                                                     line_outer_edge_erode_kernel_px_)));
        cv::subtract(line_mask, eroded_line_mask, edge_mask);

        cv::cvtColor(bev, debug_view, cv::COLOR_GRAY2BGR);
        debug_view.setTo(cv::Scalar(0, 130, 255), edge_mask);
        draw_connected_components(debug_view, component_stats);
        draw_bev_grid(debug_view);
        draw_self_exclusion(debug_view);
        auto update_debug_stages = [&]() {
            debug_stages = build_stage_debug_mosaic(
                bev, opened, closed, line_mask, component_mask, edge_mask, debug_view);
        };

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edge_mask, lines, 1.0, CV_PI / 180.0, 24,
                        min_line_length_m_ / bev_res_,
                        max_line_gap_m_ / bev_res_);

        std::vector<Candidate> candidates;
        candidates.reserve(lines.size());
        for (const auto& line : lines) {
            auto p1 = pixel_to_world(line[0], line[1]);
            auto p2 = pixel_to_world(line[2], line[3]);
            const double dx = p2.x() - p1.x();
            const double dy = p2.y() - p1.y();
            const double length = std::hypot(dx, dy);
            if (length < min_line_length_m_) {
                continue;
            }

            const double dist = distance_to_segment(Eigen::Vector2d::Zero(), p1, p2);
            const Eigen::Vector2d mid = 0.5 * (p1 + p2);
            if (is_self_vessel_area(p1.x(), p1.y()) ||
                is_self_vessel_area(p2.x(), p2.y()) ||
                is_self_vessel_area(mid.x(), mid.y())) {
                continue;
            }

            const int support = count_line_support(points, p1, p2);
            if (support < min_support_points_) {
                continue;
            }
            const double support_z_mean = mean_line_support_z(points, p1, p2);

            LineEdge edge;
            edge.x1 = p1.x();
            edge.y1 = p1.y();
            edge.x2 = p2.x();
            edge.y2 = p2.y();
            edge.length = length;
            edge.angle = normalize_angle(std::atan2(dy, dx) * 180.0 / M_PI);
            edge.distance_to_sensor = dist;
            edge.support_points = support;
            edge.support_z_mean = support_z_mean;
            edge.support_region_extent_m =
                connected_region_extent_m(line, component_labels, component_stats);

            if (is_in_known_dam_region(edge)) {
                cv::line(debug_view, cv::Point(line[0], line[1]),
                         cv::Point(line[2], line[3]), cv::Scalar(255, 120, 0), 2,
                         cv::LINE_AA);
                continue;
            }
            if (is_dam_line(edge)) {
                remember_dam_edge(edge);
                cv::line(debug_view, cv::Point(line[0], line[1]),
                         cv::Point(line[2], line[3]), cv::Scalar(255, 180, 0), 2,
                         cv::LINE_AA);
                continue;
            }

            Candidate candidate;
            candidate.edge = edge;
            candidate.line_px = line;
            candidate.quality_score = length * 1.2 + support * 0.08;
            candidate.score = candidate.quality_score + dist * outer_line_distance_weight_;
            candidates.push_back(candidate);

            cv::line(debug_view, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                     cv::Scalar(50, 180, 50), 1, cv::LINE_AA);
        }

        if (candidates.empty()) {
            update_debug_stages();
            return false;
        }

        const std::vector<std::size_t> outer_indices = select_outer_candidates_by_side(candidates);
        if (outer_indices.empty()) {
            update_debug_stages();
            return false;
        }

        std::vector<std::size_t> height_filtered;
        height_filtered.reserve(outer_indices.size());
        for (const std::size_t index : outer_indices) {
            const double zmean = candidates[index].edge.support_z_mean;
            if (std::isfinite(zmean) && zmean <= max_support_z_mean_m_) {
                height_filtered.push_back(index);
            }
        }
        if (height_filtered.empty()) {
            update_debug_stages();
            return false;
        }

        std::sort(height_filtered.begin(), height_filtered.end(),
                  [&candidates](std::size_t a, std::size_t b) {
                      return candidates[a].score > candidates[b].score;
                  });

        const int max_outputs = std::max(1, max_line_berth_outputs_);
        for (const std::size_t index : height_filtered) {
            if (static_cast<int>(out_berths.size()) >= max_outputs) {
                break;
            }

            LineBerth berth = make_berth_from_edge(candidates[index].edge, points);
            berth.inner_point_count = count_points_inside_berth(points, berth);
            berth.berth_side_point_count = count_points_on_berth_side(points, berth);
            if (berth.berth_side_point_count > berth_side_max_points_) {
                cv::line(debug_view,
                         cv::Point(candidates[index].line_px[0],
                                   candidates[index].line_px[1]),
                         cv::Point(candidates[index].line_px[2],
                                   candidates[index].line_px[3]),
                         cv::Scalar(180, 80, 220), 2, cv::LINE_AA);
                continue;
            }
            if (is_duplicate_line_berth(berth, out_berths)) {
                cv::line(debug_view,
                         cv::Point(candidates[index].line_px[0],
                                   candidates[index].line_px[1]),
                         cv::Point(candidates[index].line_px[2],
                                   candidates[index].line_px[3]),
                         cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
                continue;
            }
            out_berths.push_back(berth);

            cv::Point edge_p1, edge_p2;
            if (world_to_pixel(berth.edge.x1, berth.edge.y1, edge_p1) &&
                world_to_pixel(berth.edge.x2, berth.edge.y2, edge_p2)) {
                cv::line(debug_view, edge_p1, edge_p2, cv::Scalar(0, 0, 255), 3,
                         cv::LINE_AA);
            }
        }
        update_debug_stages();
        return !out_berths.empty();
    }

    std::vector<std::size_t> select_outer_candidates_by_side(
        const std::vector<Candidate>& candidates) const {
        std::vector<std::size_t> outer_indices;
        select_outer_candidate_for_side(candidates, 1, outer_indices);
        select_outer_candidate_for_side(candidates, -1, outer_indices);
        return outer_indices;
    }

    void select_outer_candidate_for_side(const std::vector<Candidate>& candidates,
                                         int side,
                                         std::vector<std::size_t>& outer_indices) const {
        std::size_t base_index = candidates.size();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (candidate_side(candidates[i]) != side) {
                continue;
            }
            if (base_index == candidates.size() ||
                candidates[i].quality_score > candidates[base_index].quality_score) {
                base_index = i;
            }
        }
        if (base_index == candidates.size()) {
            return;
        }

        const Candidate& base = candidates[base_index];
        std::size_t outer_index = base_index;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const Candidate& candidate = candidates[i];
            if (candidate_side(candidate) != side) {
                continue;
            }
            if (!is_same_line_family(base.edge, candidate.edge)) {
                continue;
            }
            if (candidate.edge.distance_to_sensor > candidates[outer_index].edge.distance_to_sensor + 0.15 ||
                (std::abs(candidate.edge.distance_to_sensor -
                          candidates[outer_index].edge.distance_to_sensor) <= 0.15 &&
                 candidate.quality_score > candidates[outer_index].quality_score)) {
                outer_index = i;
            }
        }

        outer_indices.push_back(outer_index);
    }

    static int candidate_side(const Candidate& candidate) {
        const double mid_y = 0.5 * (candidate.edge.y1 + candidate.edge.y2);
        return mid_y >= 0.0 ? 1 : -1;
    }

    bool is_dam_line(const LineEdge& edge) const {
        return edge.support_region_extent_m >= dam_min_region_extent_m_;
    }

    void remember_dam_edge(const LineEdge& edge) {
        for (const auto& known : known_dam_edges_) {
            if (is_same_dam_region(known, edge)) {
                return;
            }
        }
        known_dam_edges_.push_back(edge);
        if (known_dam_edges_.size() > static_cast<std::size_t>(max_known_dam_edges_)) {
            known_dam_edges_.erase(known_dam_edges_.begin());
        }
    }

    bool is_in_known_dam_region(const LineEdge& edge) const {
        return std::any_of(known_dam_edges_.begin(), known_dam_edges_.end(),
                           [&](const LineEdge& known) {
                               return is_same_dam_region(known, edge);
                           });
    }

    bool is_same_dam_region(const LineEdge& known, const LineEdge& edge) const {
        if (angle_difference_deg(known.angle, edge.angle) > outer_parallel_angle_tol_deg_) {
            return false;
        }

        const Eigen::Vector2d k1(known.x1, known.y1);
        const Eigen::Vector2d k2(known.x2, known.y2);
        Eigen::Vector2d dir = k2 - k1;
        const double len = dir.norm();
        if (len < 1e-6) {
            return false;
        }
        dir /= len;
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        const Eigen::Vector2d known_mid = 0.5 * (k1 + k2);
        const Eigen::Vector2d edge_mid(0.5 * (edge.x1 + edge.x2),
                                       0.5 * (edge.y1 + edge.y2));

        const double lateral_gap = std::abs((edge_mid - known_mid).dot(normal));
        if (lateral_gap > dam_memory_lateral_window_m_) {
            return false;
        }

        return axis_overlap_ratio(known, edge, dir) >= dam_memory_min_overlap_;
    }

    bool is_same_line_family(const LineEdge& a, const LineEdge& b) const {
        if (angle_difference_deg(a.angle, b.angle) > outer_parallel_angle_tol_deg_) {
            return false;
        }

        const Eigen::Vector2d a1(a.x1, a.y1);
        const Eigen::Vector2d a2(a.x2, a.y2);
        Eigen::Vector2d dir = a2 - a1;
        const double len = dir.norm();
        if (len < 1e-6) {
            return false;
        }
        dir /= len;
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        const Eigen::Vector2d a_mid = 0.5 * (a1 + a2);
        const Eigen::Vector2d b_mid(0.5 * (b.x1 + b.x2), 0.5 * (b.y1 + b.y2));

        const double lateral_gap = std::abs((b_mid - a_mid).dot(normal));
        if (lateral_gap > outer_parallel_lateral_window_m_) {
            return false;
        }

        return axis_overlap_ratio(a, b, dir) >= outer_parallel_min_overlap_;
    }

    double line_berth_iou(const LineBerth& a, const LineBerth& b) const {
        if (a.length <= 1e-6 || a.width <= 1e-6 ||
            b.length <= 1e-6 || b.width <= 1e-6) {
            return 0.0;
        }

        const cv::RotatedRect rect_a(
            cv::Point2f(static_cast<float>(a.cx), static_cast<float>(a.cy)),
            cv::Size2f(static_cast<float>(a.length), static_cast<float>(a.width)),
            static_cast<float>(a.angle));
        const cv::RotatedRect rect_b(
            cv::Point2f(static_cast<float>(b.cx), static_cast<float>(b.cy)),
            cv::Size2f(static_cast<float>(b.length), static_cast<float>(b.width)),
            static_cast<float>(b.angle));

        std::vector<cv::Point2f> intersection;
        const int status = cv::rotatedRectangleIntersection(rect_a, rect_b, intersection);
        if (status == cv::INTERSECT_NONE || intersection.empty()) {
            return 0.0;
        }

        const double intersection_area =
            std::abs(cv::contourArea(intersection));
        const double union_area =
            a.length * a.width + b.length * b.width - intersection_area;
        if (union_area <= 1e-6) {
            return 0.0;
        }
        return intersection_area / union_area;
    }

    bool is_duplicate_line_berth(const LineBerth& candidate,
                                 const std::vector<LineBerth>& selected) const {
        for (const LineBerth& existing : selected) {
            const double center_dist =
                std::hypot(candidate.cx - existing.cx, candidate.cy - existing.cy);
            if (center_dist < line_berth_nms_center_m_) {
                return true;
            }
            if (line_berth_iou(candidate, existing) > line_berth_nms_iou_) {
                return true;
            }
            if (is_same_line_family(candidate.edge, existing.edge)) {
                return true;
            }
        }
        return false;
    }

    static double angle_difference_deg(double a, double b) {
        double diff = std::abs(normalize_angle(a) - normalize_angle(b));
        if (diff > 90.0) {
            diff = 180.0 - diff;
        }
        return diff;
    }

    static double axis_overlap_ratio(const LineEdge& a,
                                     const LineEdge& b,
                                     const Eigen::Vector2d& axis) {
        const Eigen::Vector2d a1(a.x1, a.y1);
        const Eigen::Vector2d a2(a.x2, a.y2);
        const Eigen::Vector2d b1(b.x1, b.y1);
        const Eigen::Vector2d b2(b.x2, b.y2);

        const double a_min = std::min(a1.dot(axis), a2.dot(axis));
        const double a_max = std::max(a1.dot(axis), a2.dot(axis));
        const double b_min = std::min(b1.dot(axis), b2.dot(axis));
        const double b_max = std::max(b1.dot(axis), b2.dot(axis));
        const double overlap = std::max(0.0, std::min(a_max, b_max) - std::max(a_min, b_min));
        const double shorter = std::max(1e-6, std::min(a_max - a_min, b_max - b_min));
        return overlap / shorter;
    }

    int count_line_support(const std::vector<Eigen::Vector3d>& points,
                           const Eigen::Vector2d& a,
                           const Eigen::Vector2d& b) const {
        const Eigen::Vector2d ab = b - a;
        const double length = ab.norm();
        if (length < 1e-6) {
            return 0;
        }

        const Eigen::Vector2d dir = ab / length;
        int support = 0;
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }

            const Eigen::Vector2d p(pt.x(), pt.y());
            const Eigen::Vector2d rel = p - a;
            const double along = rel.dot(dir);
            if (along < 0.0 || along > length) {
                continue;
            }

            const double lateral = std::abs(rel.x() * dir.y() - rel.y() * dir.x());
            if (lateral <= line_support_band_m_) {
                ++support;
            }
        }
        return support;
    }

    double connected_region_extent_m(const cv::Vec4i& line,
                                     const cv::Mat& labels,
                                     const cv::Mat& stats) const {
        int label = label_at_line(labels, line);
        if (label <= 0 || label >= stats.rows) {
            return 0.0;
        }

        const double width_m = stats.at<int>(label, cv::CC_STAT_WIDTH) * bev_res_;
        const double height_m = stats.at<int>(label, cv::CC_STAT_HEIGHT) * bev_res_;
        return std::hypot(width_m, height_m);
    }

    static int label_at_line(const cv::Mat& labels, const cv::Vec4i& line) {
        const std::array<cv::Point, 5> probes = {
            cv::Point(line[0], line[1]),
            cv::Point(line[2], line[3]),
            cv::Point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2),
            cv::Point((line[0] * 3 + line[2]) / 4, (line[1] * 3 + line[3]) / 4),
            cv::Point((line[0] + line[2] * 3) / 4, (line[1] + line[3] * 3) / 4),
        };

        for (const auto& point : probes) {
            if (point.x < 0 || point.x >= labels.cols ||
                point.y < 0 || point.y >= labels.rows) {
                continue;
            }
            const int label = labels.at<int>(point.y, point.x);
            if (label > 0) {
                return label;
            }
        }
        return 0;
    }

    double mean_line_support_z(const std::vector<Eigen::Vector3d>& points,
                               const Eigen::Vector2d& a,
                               const Eigen::Vector2d& b) const {
        const Eigen::Vector2d ab = b - a;
        const double length = ab.norm();
        if (length < 1e-6) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const Eigen::Vector2d dir = ab / length;
        double z_sum = 0.0;
        int support = 0;
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }

            const Eigen::Vector2d p(pt.x(), pt.y());
            const Eigen::Vector2d rel = p - a;
            const double along = rel.dot(dir);
            if (along < 0.0 || along > length) {
                continue;
            }

            const double lateral = std::abs(rel.x() * dir.y() - rel.y() * dir.x());
            if (lateral <= line_support_band_m_) {
                z_sum += pt.z();
                ++support;
            }
        }

        if (support == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return z_sum / support;
    }

    static double distance_to_segment(const Eigen::Vector2d& p,
                                      const Eigen::Vector2d& a,
                                      const Eigen::Vector2d& b) {
        const Eigen::Vector2d ab = b - a;
        const double denom = ab.squaredNorm();
        if (denom < 1e-9) {
            return (p - a).norm();
        }
        const double t = std::clamp((p - a).dot(ab) / denom, 0.0, 1.0);
        return (p - (a + t * ab)).norm();
    }

    bool endpoint_perpendicular_hits_points(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector2d& endpoint,
        const Eigen::Vector2d& dir,
        const Eigen::Vector2d& normal) const {
        const double along_band = std::max(0.05, length_endpoint_along_band_m_);
        const double half_perp = std::max(0.05, length_endpoint_perp_half_m_);
        const int min_hits = std::max(1, length_endpoint_min_hits_);

        int hits = 0;
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }

            const Eigen::Vector2d rel(pt.x() - endpoint.x(), pt.y() - endpoint.y());
            if (std::abs(rel.dot(dir)) <= along_band &&
                std::abs(rel.dot(normal)) <= half_perp) {
                ++hits;
                if (hits >= min_hits) {
                    return true;
                }
            }
        }
        return false;
    }

    LineEdge extend_line_edge_by_endpoint_hits(
        const LineEdge& edge,
        const std::vector<Eigen::Vector3d>& points) const {
        Eigen::Vector2d p1(edge.x1, edge.y1);
        Eigen::Vector2d p2(edge.x2, edge.y2);
        Eigen::Vector2d dir = p2 - p1;
        const double original_len = dir.norm();
        if (original_len < 1e-6) {
            return edge;
        }
        dir /= original_len;
        const Eigen::Vector2d normal(-dir.y(), dir.x());

        const double step = std::max(0.05, length_extend_step_m_);
        const double max_ext = std::max(0.0, length_extend_max_m_);

        double extended_neg = 0.0;
        while (extended_neg + step <= max_ext) {
            const Eigen::Vector2d next = p1 - dir * step;
            if (!endpoint_perpendicular_hits_points(points, next, dir, normal)) {
                break;
            }
            p1 = next;
            extended_neg += step;
        }

        double extended_pos = 0.0;
        while (extended_pos + step <= max_ext) {
            const Eigen::Vector2d next = p2 + dir * step;
            if (!endpoint_perpendicular_hits_points(points, next, dir, normal)) {
                break;
            }
            p2 = next;
            extended_pos += step;
        }

        LineEdge extended = edge;
        extended.x1 = p1.x();
        extended.y1 = p1.y();
        extended.x2 = p2.x();
        extended.y2 = p2.y();
        extended.length = (p2 - p1).norm();
        extended.angle = normalize_angle(std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) *
                                         180.0 / M_PI);
        extended.distance_to_sensor = distance_to_segment(Eigen::Vector2d::Zero(), p1, p2);
        extended.support_points = count_line_support(points, p1, p2);
        extended.support_z_mean = mean_line_support_z(points, p1, p2);
        return extended;
    }

    LineBerth make_berth_from_edge(const LineEdge& edge,
                                   const std::vector<Eigen::Vector3d>& points) const {
        const LineEdge extended_edge = extend_line_edge_by_endpoint_hits(edge, points);
        const Eigen::Vector2d p1(extended_edge.x1, extended_edge.y1);
        const Eigen::Vector2d p2(extended_edge.x2, extended_edge.y2);
        const Eigen::Vector2d mid = 0.5 * (p1 + p2);
        Eigen::Vector2d dir = p2 - p1;
        const double len = std::max(1e-6, dir.norm());
        dir /= len;
        Eigen::Vector2d normal(-dir.y(), dir.x());

        const int side = choose_ship_side(mid, normal);
        const Eigen::Vector2d center = mid + side * normal * (fixed_berth_width_m_ * 0.5);

        LineBerth berth;
        berth.cx = center.x();
        berth.cy = center.y();
        berth.length = std::max(min_line_length_m_, extended_edge.length);
        berth.width = fixed_berth_width_m_;
        berth.angle = extended_edge.angle;
        berth.edge = extended_edge;
        return berth;
    }

    int count_points_inside_berth(const std::vector<Eigen::Vector3d>& points,
                                  const LineBerth& berth) const {
        const double rad = berth.angle * M_PI / 180.0;
        const Eigen::Vector2d dir(std::cos(rad), std::sin(rad));
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        const Eigen::Vector2d center(berth.cx, berth.cy);
        const double half_length = berth.length * 0.5;
        const double half_width = berth.width * 0.5;

        int count = 0;
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }

            const Eigen::Vector2d rel = Eigen::Vector2d(pt.x(), pt.y()) - center;
            if (std::abs(rel.dot(dir)) <= half_length &&
                std::abs(rel.dot(normal)) <= half_width) {
                ++count;
            }
        }
        return count;
    }

    int count_points_on_berth_side(const std::vector<Eigen::Vector3d>& points,
                                   const LineBerth& berth) const {
        const Eigen::Vector2d p1(berth.edge.x1, berth.edge.y1);
        const Eigen::Vector2d p2(berth.edge.x2, berth.edge.y2);
        const Eigen::Vector2d edge_mid = 0.5 * (p1 + p2);
        Eigen::Vector2d dir = p2 - p1;
        const double edge_len = dir.norm();
        if (edge_len < 1e-6) {
            return 0;
        }
        dir /= edge_len;

        Eigen::Vector2d berth_side =
            Eigen::Vector2d(berth.cx, berth.cy) - edge_mid;
        const double side_norm = berth_side.norm();
        if (side_norm < 1e-6) {
            return 0;
        }
        berth_side /= side_norm;

        const double half_length = berth.length * 0.5;
        const double margin = std::max(0.0, berth_side_check_margin_m_);
        const double depth = std::max(margin, berth_side_check_depth_m_);

        int count = 0;
        for (const auto& pt : points) {
            if (!is_valid_detection_point(pt)) {
                continue;
            }

            const Eigen::Vector2d rel =
                Eigen::Vector2d(pt.x(), pt.y()) - edge_mid;
            const double along = rel.dot(dir);
            if (std::abs(along) > half_length) {
                continue;
            }

            const double side_distance = rel.dot(berth_side);
            if (side_distance > margin && side_distance <= depth) {
                ++count;
            }
        }
        return count;
    }

    int choose_ship_side(const Eigen::Vector2d& mid,
                         const Eigen::Vector2d& normal) const {
        const Eigen::Vector2d center_positive = mid + normal * (fixed_berth_width_m_ * 0.5);
        const Eigen::Vector2d center_negative = mid - normal * (fixed_berth_width_m_ * 0.5);
        return center_positive.norm() <= center_negative.norm() ? 1 : -1;
    }

    void draw_berth_overlay(cv::Mat& image,
                            const LineBerth& berth,
                            const cv::Scalar& color) const {
        if (image.empty()) {
            return;
        }

        const double rad = berth.angle * M_PI / 180.0;
        const Eigen::Vector2d dir(std::cos(rad), std::sin(rad));
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        const Eigen::Vector2d center(berth.cx, berth.cy);

        std::array<Eigen::Vector2d, 4> corners = {
            center + dir * (berth.length * 0.5) + normal * (berth.width * 0.5),
            center - dir * (berth.length * 0.5) + normal * (berth.width * 0.5),
            center - dir * (berth.length * 0.5) - normal * (berth.width * 0.5),
            center + dir * (berth.length * 0.5) - normal * (berth.width * 0.5),
        };

        std::vector<cv::Point> poly;
        for (const auto& corner : corners) {
            cv::Point px;
            if (world_to_pixel(corner.x(), corner.y(), px)) {
                poly.push_back(px);
            }
        }
        if (poly.size() == 4) {
            cv::polylines(image, poly, true, color, 2, cv::LINE_AA);
        }

        cv::Point e1, e2;
        if (world_to_pixel(berth.edge.x1, berth.edge.y1, e1) &&
            world_to_pixel(berth.edge.x2, berth.edge.y2, e2)) {
            cv::line(image, e1, e2, color, 3, cv::LINE_AA);
        }
    }

    void draw_status(LineBerthMeasureResult& result) const {
        if (result.debug_view.empty()) {
            return;
        }

        std::ostringstream text;
        text << "t=" << std::fixed << std::setprecision(3) << result.timestamp
             << " pts=" << result.point_count;
        if (result.is_detected) {
            text << (result.is_using_memory ? " MEMORY" : " DETECT")
                 << " count=" << result.berths.size()
                 << " len=" << std::setprecision(1) << result.berth.length
                 << "m dist=" << result.berth.edge.distance_to_sensor
                 << "m support=" << result.berth.edge.support_points;
            if (std::isfinite(result.berth.edge.support_z_mean)) {
                text << " zmean=" << std::setprecision(2)
                     << result.berth.edge.support_z_mean << "m";
            } else {
                text << " zmean=nan";
            }
            text << " inside=" << result.berth.inner_point_count
                 << " sidePts=" << result.berth.berth_side_point_count
                 << " angle=" << std::setprecision(1) << result.berth.angle;
        } else {
            text << " no line berth";
        }

        cv::rectangle(result.debug_view, cv::Rect(0, 0, result.debug_view.cols, 30),
                      cv::Scalar(20, 20, 20), cv::FILLED);
        cv::putText(result.debug_view, text.str(), cv::Point(10, 21),
                    cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);

        cv::putText(result.debug_view, "BEV centered on radar; detecting both-side berth candidates",
                    cv::Point(10, result.debug_view.rows - 12), cv::FONT_HERSHEY_SIMPLEX,
                    0.48, cv::Scalar(190, 190, 190), 1, cv::LINE_AA);
    }
};

}  // namespace usv

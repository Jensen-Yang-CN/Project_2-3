#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "common_types_extended.h"
#include "../berth/BerthRectGeometry.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

// ====== 加上这一段防范 MSVC 找不到 M_PI 的问题 ======
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// =====================================================


namespace usv::predetect {

/** TestSubscribPublish 泊位预检测（BEV 形态学） */
class BerthDetector {
    private:
        // 算法配置参数
        double z_min_ = -4.0;
        double z_max_ = 3.0;
        double roi_x_min_ = -40.0;
        double roi_x_max_ = 40.0;
        double roi_y_min_ = -40.0;
        double roi_y_max_ = 40.0;
        double bev_res_ = 0.2;
        double berth_width_max_ = 12.0;

        // 跟踪与记忆状态
        Berth tracked_berth_;
        bool is_tracked_ = false;
        int lost_frames_ = 0;
        const int MAX_LOST_FRAMES = 1000;

    public:
        void init() {
            is_tracked_ = false;
            lost_frames_ = 0;
        }

        BerthMeasureResult process(const LidarFrame& lidar) {
            BerthMeasureResult result;
            result.timestamp = lidar.timestamp;
            result.is_detected = false;
            result.is_using_memory = false;

            std::vector<Eigen::Vector3d> fused_points;
            fused_points.reserve(lidar.points.size());
            for (const auto& pt : lidar.points) {
                fused_points.emplace_back(pt.x, pt.y, pt.z);
            }

            Berth detected;
            bool found = process_marina_berths(fused_points, detected);

            std::vector<Berth> final_berths;
            bool is_using_memory = false;

            if (found) {
                tracked_berth_ = detected;
                is_tracked_ = true;
                lost_frames_ = 0;
                final_berths.push_back(detected);
            }
            else if (is_tracked_ && lost_frames_ < MAX_LOST_FRAMES) {
                final_berths.push_back(tracked_berth_);
                lost_frames_++;
                is_using_memory = true;
            }
            else {
                is_tracked_ = false;
            }

            std::vector<Berth> expanded_berths;
            for (auto& b : final_berths) {
                if (is_using_memory) {
                    b = enforce_free_space(b, fused_points);
                    expanded_berths.push_back(b);
                }
                else {
                    b = expand_berth_to_points(b, fused_points);
                    expanded_berths.push_back(b);
                    tracked_berth_ = b;
                }
            }

            if (!expanded_berths.empty()) {
                result.is_detected = true;
                result.is_using_memory = is_using_memory;
                result.expanded_berths = expanded_berths;
            }

            return result;
        }

    private:
        bool process_marina_berths(const std::vector<Eigen::Vector3d>& points, Berth& out_berth) {
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

            if (!contours.empty()) {
                auto max_cnt = *std::max_element(contours.begin(), contours.end(),
                    [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

                if (cv::contourArea(max_cnt) > 5) {
                    cv::RotatedRect rect = cv::minAreaRect(max_cnt);
                    double w_px = rect.size.width, h_px = rect.size.height;
                    const double angle = usv::berth_geometry::commonWidthAxisAngle(
                        w_px, h_px, rect.angle);

                    double w_3d = w_px * bev_res_, h_3d = h_px * bev_res_;
                    if (w_3d >= 2.0 && w_3d <= 10.0 && h_3d >= 5.0 && h_3d <= 20.0) {
                        out_berth = { rect.center.x * bev_res_ + roi_x_min_, rect.center.y * bev_res_ + roi_y_min_, w_3d, h_3d, angle };
                        return true;
                    }
                }
            }
            return false;
        }

        Berth expand_berth_to_points(Berth b, const std::vector<Eigen::Vector3d>& points, double step = 0.2, double max_exp = 5.0, int threshold = 5) {
            std::vector<Eigen::Vector2d> pts_2d;
            for (const auto& pt : points) {
                if (pt.z() > z_min_ && pt.z() < z_max_) pts_2d.push_back({ pt.x(), pt.y() });
            }
            if (pts_2d.empty()) return b;

            double rad = b.angle * M_PI / 180.0;
            double cos_a = std::cos(rad), sin_a = std::sin(rad);

            double bound_x_pos = b.w / 2.0, bound_x_neg = -b.w / 2.0;
            double bound_y_pos = b.l / 2.0, bound_y_neg = -b.l / 2.0;
            bool exp_x_pos = true, exp_x_neg = true, exp_y_pos = true, exp_y_neg = true;
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

                if (exp_x_pos) { if (c_xp >= threshold) { exp_x_pos = false; hit_x_pos = true; } else bound_x_pos += step; }
                if (exp_x_neg) { if (c_xn >= threshold) { exp_x_neg = false; hit_x_neg = true; } else bound_x_neg -= step; }
                if (exp_y_pos) { if (c_yp >= threshold) { exp_y_pos = false; hit_y_pos = true; } else bound_y_pos += step; }
                if (exp_y_neg) { if (c_yn >= threshold) { exp_y_neg = false; hit_y_neg = true; } else bound_y_neg -= step; }
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
} // namespace usv::predetect

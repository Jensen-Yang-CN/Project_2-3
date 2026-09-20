#pragma once

#include "BerthGeometryTransform.h"
#include "common_types_extended.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

namespace usv {

// Stores confirmed realtime detections in a stable external coordinate frame.
// The caller owns localization and supplies T_world_lidar for each frame.
class RealtimeBerthCoordinateLibrary {
public:
    struct Config {
        size_t confirmation_frames = 5;
        double duplicate_center_m = 5.0;
        double duplicate_iou = 0.20;
    };

    struct MergeResult {
        std::vector<Berth> berths;
        size_t recalled_count = 0;
    };

    RealtimeBerthCoordinateLibrary()
        : RealtimeBerthCoordinateLibrary(Config{})
    {
    }

    explicit RealtimeBerthCoordinateLibrary(Config config)
        : config_(std::move(config))
    {
        config_.confirmation_frames = std::max<size_t>(1, config_.confirmation_frames);
        config_.duplicate_center_m = std::max(0.0, config_.duplicate_center_m);
        config_.duplicate_iou = std::clamp(config_.duplicate_iou, 0.0, 1.0);
    }

    // Call only with actual current-frame detections, before recalled berths are
    // appended to the displayed result. Hit counts are cumulative, not consecutive.
    void updateFromRealtime(const std::vector<Berth> &lidar_berths,
                            const Eigen::Isometry3d &world_from_lidar)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<bool> used(pending_.size(), false);
        for (const Berth &lidar_berth : lidar_berths) {
            const Berth world_berth = transformBerth(
                lidar_berth, world_from_lidar, BerthDetectionSource::CoordinateLibrary);

            Pending *pending = nullptr;
            for (size_t i = 0; i < pending_.size(); ++i) {
                if (!used[i] && sameCandidate(pending_[i].berth, world_berth)) {
                    pending = &pending_[i];
                    used[i] = true;
                    break;
                }
            }
            if (pending == nullptr) {
                pending_.push_back(Pending{world_berth, 1});
                used.push_back(true);
                pending = &pending_.back();
            } else {
                pending->berth = world_berth;
                ++pending->hit_count;
            }

            if (pending->hit_count < config_.confirmation_frames) {
                continue;
            }

            auto stored = std::find_if(
                stored_.begin(), stored_.end(),
                [&world_berth, this](const Berth &known) {
                    return sameCandidate(known, world_berth);
                });
            if (stored == stored_.end()) {
                stored_.push_back(world_berth);
            } else {
                updateStoredBerth(*stored, world_berth);
            }
        }
    }

    // Transforms stored world-coordinate berths back to the current lidar frame
    // and appends only those not already covered by realtime detections.
    MergeResult mergeIntoCurrentFrame(const std::vector<Berth> &realtime_berths,
                                      const Eigen::Isometry3d &world_from_lidar) const
    {
        MergeResult result;
        result.berths = realtime_berths;
        const Eigen::Isometry3d lidar_from_world = world_from_lidar.inverse();

        std::lock_guard<std::mutex> lock(mutex_);
        for (const Berth &world_berth : stored_) {
            const Berth lidar_berth = transformBerth(
                world_berth, lidar_from_world, BerthDetectionSource::CoordinateLibrary);
            auto duplicated = std::find_if(
                result.berths.begin(), result.berths.end(),
                [&lidar_berth, this](const Berth &existing) {
                    return sameCandidate(existing, lidar_berth);
                });
            if (duplicated == result.berths.end()) {
                result.berths.push_back(lidar_berth);
                ++result.recalled_count;
            } else if (shouldReplaceShortLine(*duplicated, lidar_berth)) {
                *duplicated = lidar_berth;
                ++result.recalled_count;
            }
        }
        return result;
    }

    std::vector<Berth> snapshotWorldBerths() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stored_;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.clear();
        stored_.clear();
    }

    static Berth transformGeometry(
        const Berth &input,
        const Eigen::Isometry3d &transform,
        BerthDetectionSource source)
    {
        return transformBerth(input, transform, source);
    }

    static DebugLine2D transformDebugLine(
        const DebugLine2D &line,
        const Eigen::Isometry3d &transform)
    {
        return transformLine(line, transform);
    }

    bool matches(const Berth &a, const Berth &b) const
    {
        return sameCandidate(a, b);
    }

private:
    struct Pending {
        Berth berth;
        size_t hit_count = 0;
    };

    static constexpr double kLineBerthShortLengthRatio = 2.0 / 3.0;

    static void updateStoredBerth(Berth &stored, const Berth &incoming)
    {
        const double locked_line_length =
            stored.kind == BerthKind::Line && stored.l > 1e-6
            ? stored.l
            : 0.0;
        stored = incoming;
        if (locked_line_length > 0.0 && stored.kind == BerthKind::Line) {
            stored.l = locked_line_length;
        }
    }

    static bool shouldReplaceShortLine(const Berth &realtime,
                                       const Berth &stored)
    {
        return realtime.kind == BerthKind::Line
            && stored.kind == BerthKind::Line
            && stored.l > 1e-6
            && realtime.l < kLineBerthShortLengthRatio * stored.l;
    }

    static double normalizeAngle180(double angle)
    {
        return berth_geometry::normalizeAngle180(angle);
    }

    static double poseYawDeg(const Eigen::Isometry3d &pose)
    {
        return berth_geometry::poseYawDeg(pose);
    }

    static DebugLine2D transformLine(const DebugLine2D &line,
                                     const Eigen::Isometry3d &transform)
    {
        return berth_geometry::transformLine(line, transform);
    }

    static Berth transformBerth(const Berth &input,
                                const Eigen::Isometry3d &transform,
                                BerthDetectionSource source)
    {
        return berth_geometry::transformBerth(input, transform, source);
    }

    bool sameCandidate(const Berth &a, const Berth &b) const
    {
        if (a.kind != b.kind) {
            return false;
        }
        if (std::hypot(a.cx - b.cx, a.cy - b.cy) < config_.duplicate_center_m) {
            return true;
        }

        const double area_a = a.w * a.l;
        const double area_b = b.w * b.l;
        if (!(area_a > 1e-6) || !(area_b > 1e-6)) {
            return false;
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
        const int type = cv::rotatedRectangleIntersection(rect_a, rect_b, intersection);
        if (type == cv::INTERSECT_NONE) {
            return false;
        }
        const double intersection_area = type == cv::INTERSECT_FULL
            ? std::min(area_a, area_b)
            : (intersection.size() >= 3 ? std::abs(cv::contourArea(intersection)) : 0.0);
        const double union_area = area_a + area_b - intersection_area;
        return union_area > 1e-6
            && intersection_area / union_area >= config_.duplicate_iou;
    }

    Config config_;
    mutable std::mutex mutex_;
    std::vector<Pending> pending_;
    std::vector<Berth> stored_;
};

}  // namespace usv

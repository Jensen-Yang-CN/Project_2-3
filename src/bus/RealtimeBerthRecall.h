#pragma once

#include "RealtimeBerthCoordinateLibrary.h"
#include "RealtimeBerthStability.h"
#include "RealtimeWorldAnchorBus.h"
#include "bridge_common_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace usv {

namespace berth_recall_detail {

inline cv::RotatedRect rectangle(const Berth &berth)
{
    return cv::RotatedRect(
        cv::Point2f(
            static_cast<float>(berth.cx),
            static_cast<float>(berth.cy)),
        cv::Size2f(
            static_cast<float>(berth.w),
            static_cast<float>(berth.l)),
        static_cast<float>(berth.angle));
}

using BerthCorners = std::array<Eigen::Vector2d, 4>;

inline BerthCorners corners(const Berth &berth)
{
    constexpr double kPi = 3.14159265358979323846;
    const double rad = berth.angle * kPi / 180.0;
    const double cos_a = std::cos(rad);
    const double sin_a = std::sin(rad);
    const double half_w = berth.w * 0.5;
    const double half_l = berth.l * 0.5;
    const double local[4][2] = {
        {-half_w, -half_l},
        { half_w, -half_l},
        { half_w,  half_l},
        {-half_w,  half_l},
    };
    BerthCorners result;
    for (int i = 0; i < 4; ++i) {
        result[i] = Eigen::Vector2d(
            berth.cx + cos_a * local[i][0] - sin_a * local[i][1],
            berth.cy + sin_a * local[i][0] + cos_a * local[i][1]);
    }
    return result;
}

inline double cross2d(const Eigen::Vector2d &a,
                      const Eigen::Vector2d &b,
                      const Eigen::Vector2d &point)
{
    return (b.x() - a.x()) * (point.y() - a.y())
        - (b.y() - a.y()) * (point.x() - a.x());
}

inline bool pointOnSegment(const Eigen::Vector2d &a,
                           const Eigen::Vector2d &b,
                           const Eigen::Vector2d &point)
{
    constexpr double epsilon = 1e-6;
    return std::abs(cross2d(a, b, point)) <= epsilon
        && point.x() >= std::min(a.x(), b.x()) - epsilon
        && point.x() <= std::max(a.x(), b.x()) + epsilon
        && point.y() >= std::min(a.y(), b.y()) - epsilon
        && point.y() <= std::max(a.y(), b.y()) + epsilon;
}

inline bool segmentsIntersect(const Eigen::Vector2d &a0,
                              const Eigen::Vector2d &a1,
                              const Eigen::Vector2d &b0,
                              const Eigen::Vector2d &b1)
{
    constexpr double epsilon = 1e-6;
    const double c1 = cross2d(a0, a1, b0);
    const double c2 = cross2d(a0, a1, b1);
    const double c3 = cross2d(b0, b1, a0);
    const double c4 = cross2d(b0, b1, a1);
    if (((c1 > epsilon && c2 < -epsilon)
         || (c1 < -epsilon && c2 > epsilon))
        && ((c3 > epsilon && c4 < -epsilon)
            || (c3 < -epsilon && c4 > epsilon))) {
        return true;
    }
    return pointOnSegment(a0, a1, b0)
        || pointOnSegment(a0, a1, b1)
        || pointOnSegment(b0, b1, a0)
        || pointOnSegment(b0, b1, a1);
}

inline bool uShapeEdgesIntersect(const Berth &a, const Berth &b)
{
    if (a.kind != BerthKind::UShape || b.kind != BerthKind::UShape)
        return false;
    const BerthCorners corners_a = corners(a);
    const BerthCorners corners_b = corners(b);
    for (int edge_a = 0; edge_a < 4; ++edge_a) {
        for (int edge_b = 0; edge_b < 4; ++edge_b) {
            if (segmentsIntersect(
                    corners_a[edge_a],
                    corners_a[(edge_a + 1) % 4],
                    corners_b[edge_b],
                    corners_b[(edge_b + 1) % 4])) {
                return true;
            }
        }
    }
    return false;
}

inline double outputConfidence(const Berth &berth)
{
    double point_support = 0.0;
    int valid_height_edges = 0;
    for (int edge = 0; edge < 4; ++edge) {
        if (edge == berth.opening_edge)
            continue;
        point_support += std::max(0, berth.edge_point_counts[edge]);
        valid_height_edges += berth.edge_height_valid[edge] ? 1 : 0;
    }
    return point_support
        + 100.0 * static_cast<double>(valid_height_edges);
}

inline bool firstHasHigherPriority(const Berth &first,
                                   const Berth &second)
{
    const double first_support = outputConfidence(first);
    const double second_support = outputConfidence(second);
    if (first_support != second_support)
        return first_support > second_support;
    return first.w * first.l >= second.w * second.l;
}

} // namespace berth_recall_detail

/**
 * Output-only arbitration for overlapping U-shape candidates in one frame.
 * Detector tracking/ROI state remains untouched because only the result copy
 * is filtered.
 */
inline void suppressIntersectingUShapeOutputBerths(
    BerthMeasureResult &result)
{
    std::vector<bool> keep(result.expanded_berths.size(), true);
    for (size_t i = 0; i < result.expanded_berths.size(); ++i) {
        if (!keep[i]
            || result.expanded_berths[i].kind != BerthKind::UShape) {
            continue;
        }
        for (size_t j = i + 1;
             j < result.expanded_berths.size(); ++j) {
            if (!keep[j]
                || result.expanded_berths[j].kind
                       != BerthKind::UShape
                || !berth_recall_detail::uShapeEdgesIntersect(
                       result.expanded_berths[i],
                       result.expanded_berths[j])) {
                continue;
            }
            if (berth_recall_detail::firstHasHigherPriority(
                    result.expanded_berths[i],
                    result.expanded_berths[j])) {
                keep[j] = false;
            } else {
                keep[i] = false;
                break;
            }
        }
    }

    std::vector<Berth> filtered;
    filtered.reserve(result.expanded_berths.size());
    for (size_t i = 0; i < result.expanded_berths.size(); ++i) {
        if (keep[i])
            filtered.push_back(std::move(result.expanded_berths[i]));
    }
    result.expanded_berths = std::move(filtered);
    result.is_detected = !result.expanded_berths.empty();
}

/**
 * Applies the handoff package's coordinate-library rules to a realtime result.
 *
 * The caller owns localization and supplies T_world_lidar. Keeping pose
 * acquisition outside this class makes it impossible for recalled berths to be
 * mistaken for fresh detector output.
 */
class RealtimeBerthRecall {
public:
    struct Config {
        RealtimeBerthCoordinateLibrary::Config library{};
        double max_pose_age_sec = 0.25;
    };

    struct MergeStats {
        size_t realtime_count = 0;
        size_t recalled_count = 0;
    };

    struct UShapeDetectionContext {
        bool has_world_anchor_bus = false;
        DebugLine2D world_anchor_bus{};
        std::vector<Berth> stable_historical_berths;
        std::vector<Berth> recovery_rois;
    };

    RealtimeBerthRecall()
        : RealtimeBerthRecall(Config{})
    {
    }

    explicit RealtimeBerthRecall(Config config)
        : config_(std::move(config))
        , line_library_(config_.library)
    {
        config_.max_pose_age_sec = std::max(0.0, config_.max_pose_age_sec);
    }

    static bool isGnssUsable(const GnssInsMessage &gnss,
                             double lidar_timestamp,
                             double max_pose_age_sec)
    {
        const bool finite =
            std::isfinite(gnss.timestamp) &&
            std::isfinite(gnss.latitude) &&
            std::isfinite(gnss.longitude) &&
            std::isfinite(gnss.altitude) &&
            std::isfinite(gnss.yaw) &&
            std::isfinite(gnss.pitch) &&
            std::isfinite(gnss.roll) &&
            std::isfinite(lidar_timestamp);
        if (!finite || !gnss.yaw_valid || !gnss.pitch_valid
            || !gnss.roll_valid || gnss.timestamp <= 0.0
            || lidar_timestamp <= 0.0) {
            return false;
        }
        if (gnss.latitude < -90.0 || gnss.latitude > 90.0 ||
            gnss.longitude < -180.0 || gnss.longitude > 180.0) {
            return false;
        }
        if (gnss.latitude == 0.0 && gnss.longitude == 0.0)
            return false;
        return std::abs(gnss.timestamp - lidar_timestamp) <=
               std::max(0.0, max_pose_age_sec);
    }

    bool isGnssUsable(const GnssInsMessage &gnss,
                      double lidar_timestamp) const
    {
        return isGnssUsable(gnss, lidar_timestamp,
                            config_.max_pose_age_sec);
    }

    UShapeDetectionContext uShapeDetectionContext(
        const Eigen::Isometry3d &world_from_lidar) const
    {
        UShapeDetectionContext context;
        const Eigen::Isometry3d lidar_from_world =
            world_from_lidar.inverse();

        if (world_anchor_bus_.valid()) {
            context.has_world_anchor_bus = true;
            context.world_anchor_bus =
                RealtimeBerthCoordinateLibrary::transformDebugLine(
                    world_anchor_bus_.line(), lidar_from_world);
        }

        const std::vector<Berth> stable_world =
            u_shape_stability_.stableLibrary();
        context.stable_historical_berths.reserve(stable_world.size());
        for (const Berth &world_berth : stable_world) {
            context.stable_historical_berths.push_back(
                RealtimeBerthCoordinateLibrary::transformGeometry(
                    world_berth, lidar_from_world,
                    BerthDetectionSource::CoordinateLibrary));
        }

        context.recovery_rois.reserve(
            u_shape_recovery_rois_world_.size());
        for (const Berth &world_roi : u_shape_recovery_rois_world_) {
            context.recovery_rois.push_back(
                RealtimeBerthCoordinateLibrary::transformGeometry(
                    world_roi, lidar_from_world,
                    BerthDetectionSource::Realtime));
        }
        return context;
    }

    void updateUShapeRecoveryRois(
        const std::vector<Berth> &local_rois,
        const Eigen::Isometry3d &world_from_lidar,
        bool detector_has_memory)
    {
        if (!local_rois.empty()) {
            u_shape_recovery_rois_world_.clear();
            u_shape_recovery_rois_world_.reserve(local_rois.size());
            for (const Berth &local_roi : local_rois) {
                if (local_roi.kind != BerthKind::UShape)
                    continue;
                u_shape_recovery_rois_world_.push_back(
                    RealtimeBerthCoordinateLibrary::transformGeometry(
                        local_roi, world_from_lidar,
                        BerthDetectionSource::Realtime));
            }
            return;
        }

        if (!detector_has_memory)
            u_shape_recovery_rois_world_.clear();
    }

    void clearUShapeRecoveryRois()
    {
        u_shape_recovery_rois_world_.clear();
    }

    MergeStats mergeWithPose(BerthMeasureResult &result,
                             const Eigen::Isometry3d &world_from_lidar,
                             bool u_shape_enabled = true,
                             bool line_enabled = true)
    {
        const auto kind_enabled = [u_shape_enabled, line_enabled](BerthKind kind) {
            if (kind == BerthKind::UShape)
                return u_shape_enabled;
            if (kind == BerthKind::Line)
                return line_enabled;
            return u_shape_enabled || line_enabled;
        };

        std::vector<Berth> realtime_u_lidar;
        std::vector<Berth> realtime_lines;
        std::vector<Berth> realtime_other;
        for (const Berth &berth : result.expanded_berths) {
            if (berth.source != BerthDetectionSource::Realtime
                || !kind_enabled(berth.kind))
                continue;
            if (berth.kind == BerthKind::UShape)
                realtime_u_lidar.push_back(berth);
            else if (berth.kind == BerthKind::Line)
                realtime_lines.push_back(berth);
            else
                realtime_other.push_back(berth);
        }

        const bool has_raw_anchor_bus =
            u_shape_enabled && result.has_anchor_bus;
        const DebugLine2D raw_anchor_bus = result.anchor_bus_line;
        result.has_anchor_bus = false;
        result.anchor_bus_line = {};
        if (u_shape_enabled) {
            world_anchor_bus_.update(
                has_raw_anchor_bus
                    ? RealtimeBerthCoordinateLibrary::transformDebugLine(
                          raw_anchor_bus, world_from_lidar)
                    : DebugLine2D{});
        }

        std::vector<Berth> realtime_u_world;
        realtime_u_world.reserve(realtime_u_lidar.size());
        for (const Berth &berth : realtime_u_lidar) {
            realtime_u_world.push_back(
                RealtimeBerthCoordinateLibrary::transformGeometry(
                    berth, world_from_lidar,
                    BerthDetectionSource::Realtime));
        }
        world_anchor_bus_.updateBerthRowSide(realtime_u_world);
        if (world_anchor_bus_.valid()) {
            for (Berth &berth : realtime_u_world)
                world_anchor_bus_.alignBerth(berth);
        }

        const std::vector<bool> accepted =
            u_shape_stability_.update(realtime_u_world);
        std::vector<Berth> accepted_realtime_u;
        for (size_t i = 0; i < realtime_u_lidar.size(); ++i) {
            if (i < accepted.size() && accepted[i])
                accepted_realtime_u.push_back(realtime_u_lidar[i]);
        }

        if (world_anchor_bus_.valid()) {
            u_shape_stability_.correctAll(
                [this](Berth &berth) {
                    world_anchor_bus_.alignBerth(berth);
                });
        }

        line_library_.updateFromRealtime(
            realtime_lines, world_from_lidar);
        const RealtimeBerthCoordinateLibrary::MergeResult merged_lines =
            line_library_.mergeIntoCurrentFrame(
                realtime_lines, world_from_lidar);

        result.expanded_berths.clear();
        result.expanded_berths.reserve(
            accepted_realtime_u.size()
            + merged_lines.berths.size()
            + realtime_other.size()
            + u_shape_stability_.stableLibrary().size());
        result.expanded_berths.insert(
            result.expanded_berths.end(),
            accepted_realtime_u.begin(), accepted_realtime_u.end());
        result.expanded_berths.insert(
            result.expanded_berths.end(),
            merged_lines.berths.begin(), merged_lines.berths.end());
        result.expanded_berths.insert(
            result.expanded_berths.end(),
            realtime_other.begin(), realtime_other.end());

        size_t recalled_count = 0;
        for (const Berth &world_berth :
             u_shape_stability_.stableLibrary()) {
            if (!u_shape_enabled)
                continue;
            Berth recalled =
                RealtimeBerthCoordinateLibrary::transformGeometry(
                    world_berth, world_from_lidar.inverse(),
                    BerthDetectionSource::CoordinateLibrary);
            const bool duplicated = std::any_of(
                result.expanded_berths.begin(),
                result.expanded_berths.end(),
                [&recalled](const Berth &existing) {
                    return sameStableGeometry(existing, recalled);
                });
            const bool conflicts_with_realtime = std::any_of(
                accepted_realtime_u.begin(),
                accepted_realtime_u.end(),
                [&recalled](const Berth &realtime) {
                    return berth_recall_detail::uShapeEdgesIntersect(
                        realtime, recalled);
                });
            if (!duplicated && !conflicts_with_realtime) {
                result.expanded_berths.push_back(recalled);
                ++recalled_count;
            }
        }
        for (const Berth &berth : merged_lines.berths) {
            if (berth.source == BerthDetectionSource::CoordinateLibrary)
                ++recalled_count;
        }

        if (u_shape_enabled && world_anchor_bus_.valid()) {
            result.has_anchor_bus = true;
            result.anchor_bus_line =
                RealtimeBerthCoordinateLibrary::transformDebugLine(
                    world_anchor_bus_.line(),
                    world_from_lidar.inverse());
        }
        result.is_detected = !result.expanded_berths.empty();
        result.is_using_memory = recalled_count > 0;

        return MergeStats{
            accepted_realtime_u.size()
                + realtime_lines.size()
                + realtime_other.size(),
            recalled_count};
    }

    void reset()
    {
        line_library_.clear();
        u_shape_stability_.clear();
        world_anchor_bus_.clear();
        u_shape_recovery_rois_world_.clear();
    }

private:
    static double angleDifference(double a, double b)
    {
        double difference = std::fmod(std::abs(a - b), 180.0);
        if (difference < 0.0)
            difference += 180.0;
        return std::min(difference, 180.0 - difference);
    }

    static cv::RotatedRect rectangle(const Berth &berth)
    {
        return cv::RotatedRect(
            cv::Point2f(
                static_cast<float>(berth.cx),
                static_cast<float>(berth.cy)),
            cv::Size2f(
                static_cast<float>(berth.w),
                static_cast<float>(berth.l)),
            static_cast<float>(berth.angle));
    }

    static double intersectionOverUnion(const Berth &a, const Berth &b)
    {
        if (!(a.w > 1e-6) || !(a.l > 1e-6)
            || !(b.w > 1e-6) || !(b.l > 1e-6)) {
            return 0.0;
        }
        std::vector<cv::Point2f> intersection;
        const int type = cv::rotatedRectangleIntersection(
            rectangle(a), rectangle(b), intersection);
        if (type == cv::INTERSECT_NONE)
            return 0.0;
        const double area_a = a.w * a.l;
        const double area_b = b.w * b.l;
        const double intersection_area =
            type == cv::INTERSECT_FULL
            ? std::min(area_a, area_b)
            : (intersection.size() >= 3
                   ? std::abs(cv::contourArea(intersection))
                   : 0.0);
        const double union_area =
            area_a + area_b - intersection_area;
        return union_area > 1e-6
            ? intersection_area / union_area
            : 0.0;
    }

    static bool sameStableGeometry(const Berth &a, const Berth &b)
    {
        if (a.kind != BerthKind::UShape
            || b.kind != BerthKind::UShape) {
            return false;
        }
        const bool direct_axes_match =
            std::abs(a.w - b.w) < 1.5
            && std::abs(a.l - b.l) < 1.5
            && angleDifference(a.angle, b.angle) < 8.0;
        const bool swapped_axes_match =
            std::abs(a.l - b.w) < 1.5
            && std::abs(a.w - b.l) < 1.5
            && angleDifference(a.angle + 90.0, b.angle) < 8.0;
        return std::hypot(a.cx - b.cx, a.cy - b.cy) < 1.5
            && (direct_axes_match || swapped_axes_match)
            && intersectionOverUnion(a, b) >= 0.55;
    }

    Config config_;
    RealtimeBerthCoordinateLibrary line_library_;
    berth_realtime::RealtimeBerthStability u_shape_stability_;
    berth_realtime::RealtimeWorldAnchorBus world_anchor_bus_;
    std::vector<Berth> u_shape_recovery_rois_world_;
};

}  // namespace usv

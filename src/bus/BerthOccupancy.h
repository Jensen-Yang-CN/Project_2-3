#pragma once

#include "common_types_extended.h"

#include <cmath>

namespace usv::berth_occupancy {

inline constexpr double kDefaultInsetMarginM = 0.8;
inline constexpr int kDefaultOccupiedMinPoints = 100;
inline constexpr double kPi = 3.14159265358979323846;

/**
 * Count points in the inset rectangle of a berth and update its internal
 * occupancy annotation. The annotation is intentionally kept out of the
 * external berth protocol and slammap persistence format.
 */
inline int countInteriorPoints(const usv::LidarFrame &frame,
                               const usv::Berth &berth,
                               double inset_margin_m = kDefaultInsetMarginM)
{
    const double half_width = berth.w * 0.5 - inset_margin_m;
    const double half_length = berth.l * 0.5 - inset_margin_m;
    if (!std::isfinite(berth.cx) || !std::isfinite(berth.cy)
        || !std::isfinite(berth.angle) || half_width <= 0.0
        || half_length <= 0.0) {
        return 0;
    }

    const double radians = berth.angle * kPi / 180.0;
    const double cos_angle = std::cos(radians);
    const double sin_angle = std::sin(radians);
    // Reject points outside the rotated rectangle's axis-aligned envelope
    // before evaluating the full coordinate transform. This keeps the
    // per-berth scan inexpensive for multi-million-point fused frames while
    // preserving the exact inset-rectangle result.
    const double envelope_x = std::abs(cos_angle) * half_width
        + std::abs(sin_angle) * half_length;
    const double envelope_y = std::abs(sin_angle) * half_width
        + std::abs(cos_angle) * half_length;
    int count = 0;
    for (const usv::LidarPoint &point : frame.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            continue;
        const double dx = static_cast<double>(point.x) - berth.cx;
        const double dy = static_cast<double>(point.y) - berth.cy;
        if (std::abs(dx) > envelope_x || std::abs(dy) > envelope_y)
            continue;
        const double local_x = cos_angle * dx + sin_angle * dy;
        const double local_y = -sin_angle * dx + cos_angle * dy;
        if (std::abs(local_x) <= half_width
            && std::abs(local_y) <= half_length) {
            ++count;
        }
    }
    return count;
}

inline void annotate(usv::LidarFrame const &frame,
                     usv::Berth &berth,
                     double inset_margin_m = kDefaultInsetMarginM,
                     int occupied_min_points = kDefaultOccupiedMinPoints)
{
    berth.interior_point_count = countInteriorPoints(
        frame, berth, inset_margin_m);
    berth.has_ship = berth.interior_point_count >= occupied_min_points;
}

} // namespace usv::berth_occupancy

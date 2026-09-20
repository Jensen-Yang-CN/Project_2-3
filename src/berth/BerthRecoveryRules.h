#pragma once

#include "common_types_extended.h"

#include <Eigen/Dense>

#include <cmath>

namespace usv::berth_recovery {

inline constexpr double kAnchorPairMinDistanceM = 9.0;
inline constexpr double kExpansionDimensionLimitM = 10.0;
inline constexpr double kStableHistoryAnchorMarginM = 1.5;

inline bool validAnchorPairDistance(double distance_m)
{
    return std::isfinite(distance_m)
        && distance_m > kAnchorPairMinDistanceM;
}

inline bool canExpandDimension(double current_m, double step_m)
{
    return std::isfinite(current_m) && std::isfinite(step_m)
        && current_m > 0.0 && step_m > 0.0
        && current_m + step_m
               <= kExpansionDimensionLimitM + 1e-9;
}

inline bool anchorOverlapsStableBerth(
    const Eigen::Vector2d &anchor,
    const Berth &stable,
    double margin_m = kStableHistoryAnchorMarginM)
{
    if (!anchor.allFinite()
        || !std::isfinite(stable.cx) || !std::isfinite(stable.cy)
        || !std::isfinite(stable.w) || !std::isfinite(stable.l)
        || !std::isfinite(stable.angle)
        || stable.w <= 1e-6 || stable.l <= 1e-6
        || !std::isfinite(margin_m) || margin_m < 0.0) {
        return false;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double radians = stable.angle * kPi / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    const double dx = anchor.x() - stable.cx;
    const double dy = anchor.y() - stable.cy;
    const double local_x = dx * cosine + dy * sine;
    const double local_y = -dx * sine + dy * cosine;

    return std::abs(local_x) <= stable.w * 0.5 + margin_m
        && std::abs(local_y) <= stable.l * 0.5 + margin_m;
}

}  // namespace usv::berth_recovery


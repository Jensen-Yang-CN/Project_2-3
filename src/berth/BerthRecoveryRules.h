#pragma once

#include "common_types_extended.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

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

inline Berth makeDefaultExpansionFallback(const Berth &original)
{
    Berth fallback = original;
    fallback.w = kExpansionDimensionLimitM;
    fallback.l = kExpansionDimensionLimitM;
    fallback.used_default_expansion_size = true;
    return fallback;
}

inline bool hasReliableExpansionSize(const Berth &berth)
{
    return !berth.used_default_expansion_size;
}

inline bool canStabilizeSizeFromTrackedBerth(
    const Berth &candidate,
    const Berth &tracked)
{
    return hasReliableExpansionSize(candidate)
        && hasReliableExpansionSize(tracked);
}

inline void appendExpansionSizeHistoryFrame(
    std::deque<std::vector<Berth>> &history,
    const std::vector<Berth> &berths,
    std::size_t max_frames)
{
    std::vector<Berth> reliable_berths;
    reliable_berths.reserve(berths.size());
    for (const Berth &berth : berths) {
        if (hasReliableExpansionSize(berth)) {
            reliable_berths.push_back(berth);
        }
    }

    history.push_back(std::move(reliable_berths));
    const std::size_t capacity = std::max<std::size_t>(1, max_frames);
    while (history.size() > capacity) {
        history.pop_front();
    }
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

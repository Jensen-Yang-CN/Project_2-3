#pragma once

#include "common_types_extended.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace usv::berth_opening {

inline bool validSize(const Berth &berth)
{
    return std::isfinite(berth.cx) && std::isfinite(berth.cy)
        && std::isfinite(berth.w) && std::isfinite(berth.l)
        && std::isfinite(berth.angle)
        && berth.w > 1e-6 && berth.l > 1e-6;
}

inline std::array<int, 4> countOpeningRayHits(
    const Berth &berth,
    const std::vector<Eigen::Vector3d> &points,
    double ray_length_m = 20.0,
    double ray_half_width_m = 1.0)
{
    std::array<int, 4> hits{0, 0, 0, 0};
    if (!validSize(berth))
        return hits;

    constexpr double kPi = 3.14159265358979323846;
    const double rad = berth.angle * kPi / 180.0;
    const double cos_a = std::cos(rad);
    const double sin_a = std::sin(rad);
    const double half_w = berth.w * 0.5;
    const double half_l = berth.l * 0.5;
    const double ray_length = std::max(0.1, ray_length_m);
    const double half_width = std::max(0.01, ray_half_width_m);

    for (const Eigen::Vector3d &point : points) {
        // Opening occupancy is a BEV rule. Height must not hide a blocking
        // object above or below the detector's morphology Z band.
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()))
            continue;

        const double dx = point.x() - berth.cx;
        const double dy = point.y() - berth.cy;
        const double local_x = dx * cos_a + dy * sin_a;
        const double local_y = -dx * sin_a + dy * cos_a;

        const double outward_0 = -local_y - half_l;
        const double outward_1 = local_x - half_w;
        const double outward_2 = local_y - half_l;
        const double outward_3 = -local_x - half_w;
        if (outward_0 >= 0.0 && outward_0 <= ray_length
            && std::abs(local_x) <= half_width) {
            ++hits[0];
        }
        if (outward_1 >= 0.0 && outward_1 <= ray_length
            && std::abs(local_y) <= half_width) {
            ++hits[1];
        }
        if (outward_2 >= 0.0 && outward_2 <= ray_length
            && std::abs(local_x) <= half_width) {
            ++hits[2];
        }
        if (outward_3 >= 0.0 && outward_3 <= ray_length
            && std::abs(local_y) <= half_width) {
            ++hits[3];
        }
    }
    return hits;
}

inline int sparsestEdge(const std::array<int, 4> &edge_counts)
{
    int edge = 0;
    for (int candidate = 1; candidate < 4; ++candidate) {
        if (edge_counts[candidate] < edge_counts[edge])
            edge = candidate;
    }
    return edge;
}

inline int selectMissingRayEdge(const std::array<int, 4> &ray_hits,
                                const std::array<int, 4> &edge_counts,
                                int min_hits)
{
    int selected = -1;
    for (int edge = 0; edge < 4; ++edge) {
        if (ray_hits[edge] >= min_hits)
            continue;
        if (selected < 0
            || edge_counts[edge] < edge_counts[selected]
            || (edge_counts[edge] == edge_counts[selected]
                && ray_hits[edge] < ray_hits[selected])) {
            selected = edge;
        }
    }
    return selected;
}

inline int selectOpeningEdge(const std::array<int, 4> &ray_hits,
                             const std::array<int, 4> &edge_counts,
                             bool anchor_recovered,
                             int anchor_edge,
                             int geometry_fallback_edge,
                             int min_hits = 1)
{
    min_hits = std::max(1, min_hits);
    int missing_edge = -1;
    int missing_count = 0;
    for (int edge = 0; edge < 4; ++edge) {
        if (ray_hits[edge] < min_hits) {
            missing_edge = edge;
            ++missing_count;
        }
    }

    if (!anchor_recovered) {
        const int selected =
            selectMissingRayEdge(ray_hits, edge_counts, min_hits);
        return selected >= 0 ? selected : sparsestEdge(edge_counts);
    }

    if (missing_count == 1)
        return missing_edge;
    if (anchor_edge >= 0 && anchor_edge < 4)
        return anchor_edge;
    return geometry_fallback_edge >= 0 && geometry_fallback_edge < 4
        ? geometry_fallback_edge
        : sparsestEdge(edge_counts);
}

}  // namespace usv::berth_opening

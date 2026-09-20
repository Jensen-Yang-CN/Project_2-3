#pragma once

#include "common_types_extended.h"

#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

namespace usv::berth_protocol {

using BerthCorners = std::array<Eigen::Vector2d, 4>;

inline BerthCorners cornersCounterClockwise(const Berth &berth)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double rad = berth.angle * kDegToRad;
    const double cos_a = std::cos(rad);
    const double sin_a = std::sin(rad);
    const double half_w = berth.w * 0.5;
    const double half_l = berth.l * 0.5;
    const double local_corners[4][2] = {
        {-half_w, -half_l},
        { half_w, -half_l},
        { half_w,  half_l},
        {-half_w,  half_l},
    };

    BerthCorners corners;
    for (int i = 0; i < 4; ++i) {
        const double local_x = local_corners[i][0];
        const double local_y = local_corners[i][1];
        corners[i] = Eigen::Vector2d(
            berth.cx + cos_a * local_x - sin_a * local_y,
            berth.cy + sin_a * local_x + cos_a * local_y);
    }
    return corners;
}

inline double edgeMidpointDistanceSquared(
    const BerthCorners &corners, int edge)
{
    const Eigen::Vector2d midpoint =
        0.5 * (corners[edge] + corners[(edge + 1) % 4]);
    return midpoint.squaredNorm();
}

inline int openingEdge(const Berth &berth, const BerthCorners &corners)
{
    if (berth.kind == BerthKind::UShape
        && berth.opening_edge >= 0 && berth.opening_edge < 4) {
        return berth.opening_edge;
    }

    if (berth.kind == BerthKind::Line) {
        // The detected berth line is opposite the opening long edge.
        return edgeMidpointDistanceSquared(corners, 1)
                   <= edgeMidpointDistanceSquared(corners, 3)
            ? 1
            : 3;
    }

    int nearest_edge = 0;
    for (int edge = 1; edge < 4; ++edge) {
        if (edgeMidpointDistanceSquared(corners, edge)
            < edgeMidpointDistanceSquared(corners, nearest_edge)) {
            nearest_edge = edge;
        }
    }
    return nearest_edge;
}

inline BerthCorners orderedCorners(const Berth &berth)
{
    const BerthCorners geometric_corners = cornersCounterClockwise(berth);
    const int opening_edge = openingEdge(berth, geometric_corners);

    BerthCorners ordered;
    for (int i = 0; i < 4; ++i) {
        ordered[i] = geometric_corners[(opening_edge + 1 + i) % 4];
    }
    return ordered;
}

inline uint8_t type(const Berth &berth)
{
    return berth.kind == BerthKind::Line ? 1 : 2;
}

}  // namespace usv::berth_protocol

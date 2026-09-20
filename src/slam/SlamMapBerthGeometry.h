#pragma once

#include <cmath>

namespace slam_map_berth {

/**
 * Normalize a rectangle axis to [-90, 90) while keeping the opening edge
 * attached to the same physical edge.  Angles that differ by 180 degrees
 * describe the same unoriented axis, but the local rectangle axes are
 * reversed and edge indices 0..3 must move to their opposite edges.
 */
inline double normalizeAngleAndOpeningEdge(double angle_deg, int &opening_edge)
{
    const double original_angle = angle_deg;
    while (angle_deg >= 90.0)
        angle_deg -= 180.0;
    while (angle_deg < -90.0)
        angle_deg += 180.0;

    if (opening_edge >= 0 && opening_edge < 4) {
        constexpr double kPi = 3.14159265358979323846;
        const double delta_rad =
            (original_angle - angle_deg) * kPi / 180.0;
        if (std::cos(delta_rad) < 0.0)
            opening_edge = (opening_edge + 2) % 4;
    }
    return angle_deg;
}

} // namespace slam_map_berth

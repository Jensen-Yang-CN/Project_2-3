#pragma once

#include <cmath>

namespace usv::berth_geometry {

/**
 * Convert cv::RotatedRect's angle to the common berth convention.
 *
 * The common Berth::angle is the direction of the short (width) axis;
 * OpenCV's angle describes the side represented by rect.size.width.  When
 * OpenCV reports width > height that side is the long axis, so the short
 * axis is 90 degrees away.  When width <= height the reported side already
 * is the short axis and must not be rotated again.
 */
inline double commonWidthAxisAngle(double rect_width_px,
                                   double rect_height_px,
                                   double rect_angle_deg)
{
    double angle = rect_angle_deg;
    if (rect_width_px > rect_height_px)
        angle += 90.0;

    angle = std::fmod(angle, 180.0);
    if (angle < 0.0)
        angle += 180.0;
    return angle;
}

}  // namespace usv::berth_geometry

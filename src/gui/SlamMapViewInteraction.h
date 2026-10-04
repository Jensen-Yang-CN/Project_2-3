#pragma once

#include <cmath>

namespace slam_map_view {

inline void panOrthographic(float &centerX, float &centerY,
                            int deltaX, int deltaY,
                            float verticalRange, float aspectRatio,
                            int viewportWidth, int viewportHeight)
{
    if (verticalRange <= 0.0f || aspectRatio <= 0.0f
        || viewportWidth <= 0 || viewportHeight <= 0)
        return;

    const float unitsPerPixelX =
        (2.0f * verticalRange * aspectRatio) / viewportWidth;
    const float unitsPerPixelY =
        (2.0f * verticalRange) / viewportHeight;
    centerX -= static_cast<float>(deltaX) * unitsPerPixelX;
    centerY += static_cast<float>(deltaY) * unitsPerPixelY;
}

inline void panPerspective(float &offsetX, float &offsetY,
                           int deltaX, int deltaY,
                           float cameraDistance, int viewportHeight)
{
    if (viewportHeight <= 0)
        return;

    constexpr float kVerticalFieldOfViewDeg = 45.0f;
    constexpr float kPi = 3.14159265358979323846f;
    const float distance = std::fmax(1.0f, std::abs(cameraDistance));
    const float halfFovRad =
        0.5f * kVerticalFieldOfViewDeg * kPi / 180.0f;
    const float unitsPerPixel =
        2.0f * distance * std::tan(halfFovRad) / viewportHeight;
    offsetX += static_cast<float>(deltaX) * unitsPerPixel;
    offsetY -= static_cast<float>(deltaY) * unitsPerPixel;
}

}  // namespace slam_map_view

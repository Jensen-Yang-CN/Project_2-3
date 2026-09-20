#pragma once

#include <cmath>
#include <cstddef>

namespace usv_direction {

struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

template <typename Path>
bool movementYawDeg(const Path &path, float minDistanceM, float &yawDeg)
{
    if (path.size() < 2)
        return false;

    const auto &current = path[path.size() - 1];
    const float minDistance2 = minDistanceM * minDistanceM;
    for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(path.size()) - 2;
         i >= 0; --i) {
        const float dx = current.x - path[static_cast<std::size_t>(i)].x;
        const float dy = current.y - path[static_cast<std::size_t>(i)].y;
        const float distance2 = dx * dx + dy * dy;
        if (distance2 < minDistance2 || distance2 < 1.0e-6f)
            continue;

        yawDeg = static_cast<float>(
            std::atan2(static_cast<double>(dy), static_cast<double>(dx))
            * 180.0 / 3.14159265358979323846);
        return true;
    }
    return false;
}

inline float compassHeadingToOpenGlYawDeg(float headingDeg)
{
    float yawDeg = std::fmod(90.0f - headingDeg, 360.0f);
    if (yawDeg <= -180.0f)
        yawDeg += 360.0f;
    else if (yawDeg > 180.0f)
        yawDeg -= 360.0f;
    return yawDeg;
}

} // namespace usv_direction

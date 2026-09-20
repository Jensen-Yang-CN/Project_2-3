#include "SlamDisplayPolicy.h"
#include "UsvDirectionUtils.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool closeAngle(float actual, float expected)
{
    float diff = std::fmod(actual - expected + 540.0f, 360.0f) - 180.0f;
    return std::abs(diff) < 0.01f;
}

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void testMovementDirection(int &failures)
{
    float yaw = -999.0f;
    std::vector<usv_direction::Point2f> east{
        {0.0f, 0.0f}, {0.3f, 0.0f}, {2.0f, 0.0f},
    };
    check(usv_direction::movementYawDeg(east, 1.0f, yaw),
          "a sufficiently long trajectory should produce a direction",
          failures);
    check(closeAngle(yaw, 0.0f),
          "eastward movement should have OpenGL yaw 0 degrees", failures);

    std::vector<usv_direction::Point2f> north{
        {0.0f, 0.0f}, {0.0f, 2.0f},
    };
    check(usv_direction::movementYawDeg(north, 1.0f, yaw)
              && closeAngle(yaw, 90.0f),
          "northward movement should have OpenGL yaw 90 degrees", failures);

    std::vector<usv_direction::Point2f> reverse{
        {5.0f, 0.0f}, {3.0f, 0.0f}, {0.0f, 0.0f},
    };
    check(usv_direction::movementYawDeg(reverse, 1.0f, yaw)
              && closeAngle(yaw, 180.0f),
          "the arrow should follow reverse movement instead of bow heading",
          failures);

    std::vector<usv_direction::Point2f> jitter{
        {0.0f, 0.0f}, {0.1f, -0.1f}, {0.2f, 0.1f},
    };
    check(!usv_direction::movementYawDeg(jitter, 1.0f, yaw),
          "sub-threshold position jitter must not change direction", failures);
}

void testCompassConversion(int &failures)
{
    check(closeAngle(usv_direction::compassHeadingToOpenGlYawDeg(0.0f), 90.0f),
          "north compass heading should map to OpenGL +Y", failures);
    check(closeAngle(usv_direction::compassHeadingToOpenGlYawDeg(90.0f), 0.0f),
          "east compass heading should map to OpenGL +X", failures);
    check(closeAngle(usv_direction::compassHeadingToOpenGlYawDeg(270.0f), 180.0f),
          "west compass heading should map to OpenGL -X", failures);
}

} // namespace

int main()
{
    int failures = 0;
    if (!slam_display::showAccumulatedKeyframeLayer()) {
        ++failures;
        std::cerr << "FAIL: the growing accumulated SLAM keyframe layer must be visible\n";
    }
    if (slam_display::showLiveScanLayer()) {
        ++failures;
        std::cerr << "FAIL: the current live SLAM scan layer must be hidden\n";
    }
    testMovementDirection(failures);
    testCompassConversion(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM display tests passed\n";
    return 0;
}

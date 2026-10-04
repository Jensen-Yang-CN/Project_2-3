#include "SlamDisplayPolicy.h"
#include "SlamMapViewInteraction.h"
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

void testMapViewPan(int &failures)
{
    float centerX = 0.0f;
    float centerY = 0.0f;
    slam_map_view::panOrthographic(centerX, centerY,
                                   10, 5, 50.0f, 2.0f, 1000, 500);
    check(std::abs(centerX + 2.0f) < 0.001f
              && std::abs(centerY - 1.0f) < 0.001f,
          "top-down drag should move the map with the pointer at map scale",
          failures);

    float perspectiveX = 0.0f;
    float perspectiveY = 0.0f;
    slam_map_view::panPerspective(perspectiveX, perspectiveY,
                                  10, 5, 50.0f, 500);
    const float unitsPerPixel =
        2.0f * 50.0f * std::tan(22.5f * 3.14159265358979323846f / 180.0f)
        / 500.0f;
    check(std::abs(perspectiveX - 10.0f * unitsPerPixel) < 0.001f
              && std::abs(perspectiveY + 5.0f * unitsPerPixel) < 0.001f,
          "3D drag should pan in screen directions using view distance",
          failures);

    float fartherX = 0.0f;
    float fartherY = 0.0f;
    slam_map_view::panPerspective(fartherX, fartherY,
                                  10, 5, 100.0f, 500);
    check(std::abs(fartherX - 2.0f * perspectiveX) < 0.001f
              && std::abs(fartherY - 2.0f * perspectiveY) < 0.001f,
          "3D pan should scale with camera distance", failures);
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
    testMapViewPan(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM display tests passed\n";
    return 0;
}

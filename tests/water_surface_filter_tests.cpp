#include "WaterSurfaceFilter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        ++g_failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

M_PointXYZI point(float x, float y, float z)
{
    M_PointXYZI p{};
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 100;
    return p;
}

std::vector<M_PointXYZI> horizontalWaterScene()
{
    std::vector<M_PointXYZI> cloud;
    for (int ix = -10; ix <= 10; ++ix) {
        for (int iy = -10; iy <= 10; ++iy) {
            const float noise = static_cast<float>(((ix + iy) % 3) - 1) * 0.01f;
            cloud.push_back(point(ix * 0.5f, iy * 0.5f, noise));
        }
    }

    // 水下反射点必须连同水面保护带一起被删除。
    for (int i = 0; i < 20; ++i)
        cloud.push_back(point(-4.0f + i * 0.4f, 7.0f, -0.6f));

    // 高出水面 0.25 m 的密集目标应保留。
    for (int ix = 0; ix < 4; ++ix) {
        for (int iy = 0; iy < 4; ++iy)
            cloud.push_back(point(8.0f + ix * 0.1f, 8.0f + iy * 0.1f, 0.65f));
    }
    return cloud;
}

usv::WaterSurfaceFilterConfig testConfig()
{
    usv::WaterSurfaceFilterConfig cfg;
    cfg.clearance_m = 0.25;
    cfg.plane_distance_m = 0.08;
    cfg.max_tilt_deg = 20.0;
    cfg.min_inliers = 80;
    cfg.max_sample_points = 5000;
    cfg.ransac_iterations = 200;
    cfg.near_surface_band_m = 1.0;
    cfg.outlier_radius_m = 0.35;
    cfg.outlier_min_neighbors = 2;
    cfg.max_fallback_frames = 2;
    return cfg;
}

void testRemovesHorizontalWaterAndKeepsObstacle()
{
    usv::WaterSurfaceFilter filter(testConfig());
    const usv::WaterSurfaceFilterResult result =
        filter.filter(horizontalWaterScene());

    check(result.plane_detected, "水平水面应被识别");
    check(result.removed_surface_or_below >= 440,
          "水面及水下反射点应被删除");
    check(result.points.size() == 16,
          "仅高出水面的密集障碍物应保留");
    check(std::all_of(result.points.begin(), result.points.end(),
                      [](const M_PointXYZI &p) { return p.z > 0.25f; }),
          "输出中不应残留水面保护带内的点");
}

void testRecognizesTiltedWater()
{
    std::vector<M_PointXYZI> cloud;
    for (int ix = -10; ix <= 10; ++ix) {
        for (int iy = -10; iy <= 10; ++iy) {
            const float x = ix * 0.5f;
            const float y = iy * 0.5f;
            const float water_z = 0.08f * x - 0.04f * y;
            cloud.push_back(point(x, y, water_z));
        }
    }
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            const float x = 7.0f + ix * 0.1f;
            const float y = 7.0f + iy * 0.1f;
            const float water_z = 0.08f * x - 0.04f * y;
            cloud.push_back(point(x, y, water_z + 0.70f));
        }
    }

    usv::WaterSurfaceFilter filter(testConfig());
    const auto result = filter.filter(cloud);
    check(result.plane_detected, "允许倾角内的水面应被识别");
    check(result.points.size() == 9, "倾斜水面上的障碍物应保留");
}

void testDoesNotDeleteWithoutTrustedPlane()
{
    std::vector<M_PointXYZI> sparse;
    for (int i = 0; i < 20; ++i)
        sparse.push_back(point(i * 0.7f, std::sin(i * 0.4f), 1.0f + i * 0.11f));

    usv::WaterSurfaceFilter filter(testConfig());
    const auto result = filter.filter(sparse);
    check(!result.plane_detected, "稀疏非平面点不应被误判为水面");
    check(result.points.size() == sparse.size(),
          "首次无可信水面时必须原样放行");
}

void testUsesRecentPlaneAsShortFallback()
{
    usv::WaterSurfaceFilter filter(testConfig());
    const auto warmup = filter.filter(horizontalWaterScene());
    check(warmup.plane_detected, "回退测试需要先建立可信水面");

    std::vector<M_PointXYZI> sparse{
        point(0.0f, 0.0f, -0.4f),
        point(2.0f, 2.0f, 1.5f),
    };
    const auto result = filter.filter(sparse);
    check(result.used_fallback_plane, "短时缺少平面时应沿用最近可信水面");
    check(result.points.size() == 1 && result.points.front().z > 1.0f,
          "历史水面应继续删除水下反射并保留高目标");
}

void testRemovesOnlyIsolatedNearSurfacePoints()
{
    std::vector<M_PointXYZI> cloud = horizontalWaterScene();
    cloud.push_back(point(-9.0f, -9.0f, 0.55f));  // 孤立水面飞点
    for (int i = 0; i < 4; ++i)
        cloud.push_back(point(4.0f + i * 0.1f, -7.0f, 0.55f));  // 小型真实目标簇

    usv::WaterSurfaceFilter filter(testConfig());
    const auto result = filter.filter(cloud);
    const bool isolated_kept = std::any_of(
        result.points.begin(), result.points.end(),
        [](const M_PointXYZI &p) { return p.x < -8.5f && p.y < -8.5f; });
    const int cluster_count = static_cast<int>(std::count_if(
        result.points.begin(), result.points.end(),
        [](const M_PointXYZI &p) { return p.x >= 4.0f && p.x < 5.0f && p.y < -6.5f; }));

    check(!isolated_kept, "水面附近孤立飞点应被删除");
    check(cluster_count == 4, "水面附近成簇真实目标应保留");
}

void testKeepsWaterSurfaceAndIsolatedPointsButRemovesUnderwater()
{
    std::vector<M_PointXYZI> cloud = horizontalWaterScene();
    cloud.push_back(point(-9.0f, -9.0f, 0.55f));

    usv::WaterSurfaceFilterConfig cfg = testConfig();
    cfg.clearance_m = -0.15;
    cfg.outlier_min_neighbors = 0;

    usv::WaterSurfaceFilter filter(cfg);
    const auto result = filter.filter(cloud);

    const int water_surface_count = static_cast<int>(std::count_if(
        result.points.begin(), result.points.end(),
        [](const M_PointXYZI &p) { return std::abs(p.z) <= 0.05f; }));
    const int underwater_count = static_cast<int>(std::count_if(
        result.points.begin(), result.points.end(),
        [](const M_PointXYZI &p) { return p.z < -0.30f; }));
    const bool isolated_kept = std::any_of(
        result.points.begin(), result.points.end(),
        [](const M_PointXYZI &p) { return p.x < -8.5f && p.y < -8.5f; });

    check(result.plane_detected, "water plane should still be detected");
    check(water_surface_count >= 400, "water-surface points should be kept");
    check(underwater_count == 0, "underwater points should still be removed");
    check(isolated_kept, "isolated near-surface points should be kept");
    check(result.removed_near_surface_outliers == 0,
          "near-surface outlier removal should be disabled");
}

}  // namespace

int main()
{
    testRemovesHorizontalWaterAndKeepsObstacle();
    testRecognizesTiltedWater();
    testDoesNotDeleteWithoutTrustedPlane();
    testUsesRecentPlaneAsShortFallback();
    testRemovesOnlyIsolatedNearSurfacePoints();
    testKeepsWaterSurfaceAndIsolatedPointsButRemovesUnderwater();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All water surface filter tests passed\n";
    return 0;
}

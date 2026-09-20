#include "pcl_cluster_preview.h"

#include <cmath>
#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

QVector<M_PointXYZI> makeBlock(float minX, float maxX,
                               float minY, float maxY,
                               float minZ, float maxZ,
                               float spacing)
{
    QVector<M_PointXYZI> cloud;
    for (float z = minZ; z <= maxZ + 1.0e-4f; z += spacing) {
        for (float y = minY; y <= maxY + 1.0e-4f; y += spacing) {
            for (float x = minX; x <= maxX + 1.0e-4f; x += spacing) {
                M_PointXYZI point{};
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = 1;
                cloud.append(point);
            }
        }
    }
    return cloud;
}

} // namespace

int main()
{
    int failures = 0;
    const PclClusterPreview::Config defaults;
    check(std::abs(defaults.shipHalfExtentX - 6.0f) < 1.0e-6f
              && std::abs(defaults.shipHalfExtentY - 2.5f) < 1.0e-6f,
          "ship exclusion must match the current +X-forward rendered hull",
          failures);

    PclClusterPreview preview;
    const PclClusterPreviewResult empty = preview.process({}, 10.0);
    check(!empty.classificationReady && empty.obstacles.isEmpty(),
          "empty input must produce an empty result", failures);

    const QVector<M_PointXYZI> outside =
        makeBlock(8.0f, 10.0f, -1.0f, 1.0f, 0.0f, 1.0f, 0.2f);
    const PclClusterPreviewResult obstacle = preview.process(outside, 10.1);
    check(obstacle.classificationReady && obstacle.nearestObstacleDetected
              && !obstacle.obstacles.isEmpty(),
          "a dense block beyond the bow must be classified as an obstacle",
          failures);
    if (obstacle.nearestObstacleDetected) {
        check(obstacle.nearestObstacleClearanceMeters >= 1.5f
                  && obstacle.nearestObstacleClearanceMeters <= 2.5f,
              "obstacle clearance must be measured from the bow boundary",
              failures);
    }

    PclClusterPreview insidePreview;
    const QVector<M_PointXYZI> inside =
        makeBlock(-2.0f, 2.0f, -1.0f, 1.0f, 0.0f, 1.0f, 0.2f);
    const PclClusterPreviewResult insideResult =
        insidePreview.process(inside, 20.0);
    check(!insideResult.nearestObstacleDetected,
          "points inside the rendered ship hull must be excluded", failures);

    if (failures != 0)
        return 1;
    std::cout << "All PCL cluster preview tests passed\n";
    return 0;
}

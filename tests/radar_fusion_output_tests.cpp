#include "radarfusionmanager.h"

#include <QCoreApplication>
#include <QEventLoop>
#include <QSet>
#include <QTimer>

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

QVector<M_PointXYZI> onePoint(float x)
{
    return {M_PointXYZI{x, 0.0F, 0.0F, 1, {0, 0, 0}}};
}

void waitForFusion()
{
    QEventLoop loop;
    QTimer::singleShot(40, &loop, &QEventLoop::quit);
    loop.exec();
}

QSet<int> intensities(const QVector<M_PointXYZI> &cloud)
{
    QSet<int> result;
    for (const M_PointXYZI &point : cloud)
        result.insert(point.intensity);
    return result;
}

void testRearRs32OnlyEntersDisplayOutput()
{
    RadarFusionManager manager;
    QVector<M_PointXYZI> display;
    QVector<M_PointXYZI> algorithm;
    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        display = cloud;
    });
    QObject::connect(&manager, &RadarFusionManager::algorithmCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        algorithm = cloud;
    });

    manager.handleRadarCloud("201", onePoint(1.0F), 100.0);
    manager.handleRadarCloud("211", onePoint(2.0F), 100.0);
    manager.handleRadarCloud("213", onePoint(3.0F), 100.0);
    manager.handleRadarCloud("214", onePoint(4.0F), 100.0);
    manager.handleRadarCloud("210", onePoint(5.0F), 100.0);
    waitForFusion();

    check(display.size() == 5,
          "display output must contain 201/210/211/213/214");
    check(algorithm.size() == 2,
          "algorithm output must contain only 210/211");
    check(intensities(display) == QSet<int>({50, 100, 150, 200, 250}),
          "display output must include rear 201 with blue display intensity");
    check(intensities(algorithm) == QSet<int>({150, 250}),
          "algorithm output must exclude 201/213/214");
}

void testStaleRearFramesAreSkippedWithoutBlockingFrontLidars()
{
    RadarFusionManager manager;
    QVector<M_PointXYZI> display;
    QVector<M_PointXYZI> algorithm;
    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        display = cloud;
    });
    QObject::connect(&manager, &RadarFusionManager::algorithmCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double) {
        algorithm = cloud;
    });

    manager.handleRadarCloud("213", onePoint(3.0F), 99.8);
    manager.handleRadarCloud("214", onePoint(4.0F), 99.8);
    manager.handleRadarCloud("211", onePoint(2.0F), 100.0);
    manager.handleRadarCloud("210", onePoint(5.0F), 100.0);
    waitForFusion();

    check(display.size() == 2 && algorithm.size() == 2,
          "stale RS32 frames must be skipped without blocking 210/211");
}

void testRadar211OnlyTriggersFusionOutputs()
{
    RadarFusionManager manager;
    int displaySignalCount = 0;
    int algorithmSignalCount = 0;
    QVector<M_PointXYZI> display;
    QVector<M_PointXYZI> algorithm;
    double displayTimestamp = -1.0;
    double algorithmTimestamp = -1.0;

    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double timestamp) {
        ++displaySignalCount;
        display = cloud;
        displayTimestamp = timestamp;
    });
    QObject::connect(&manager, &RadarFusionManager::algorithmCloudReady,
                     [&](const QVector<M_PointXYZI> &cloud, double timestamp) {
        ++algorithmSignalCount;
        algorithm = cloud;
        algorithmTimestamp = timestamp;
    });

    manager.handleRadarCloud("211", onePoint(2.0F), 200.0);
    waitForFusion();

    check(displaySignalCount == 1 && display.size() == 1,
          "211-only input must produce one display cloud");
    check(algorithmSignalCount == 1 && algorithm.size() == 1,
          "211-only input must produce one algorithm cloud");
    check(std::abs(displayTimestamp - 200.0) < 1e-9
              && std::abs(algorithmTimestamp - 200.0) < 1e-9,
          "211-only outputs must use the 211 capture timestamp");
}

void testRadar211FallbackDoesNotDuplicateActive210AndRecovers()
{
    RadarFusionManager manager;
    QVector<double> displayTimestamps;
    QObject::connect(&manager, &RadarFusionManager::displayCloudReady,
                     [&](const QVector<M_PointXYZI> &, double timestamp) {
        displayTimestamps.append(timestamp);
    });

    manager.handleRadarCloud("210", onePoint(1.0F), 300.0);
    waitForFusion();
    check(displayTimestamps.size() == 1,
          "210 must remain the normal fusion trigger");

    manager.handleRadarCloud("211", onePoint(2.0F), 300.2);
    waitForFusion();
    check(displayTimestamps.size() == 1,
          "recent 210 activity must suppress a duplicate 211 trigger");

    manager.handleRadarCloud("211", onePoint(3.0F), 300.6);
    waitForFusion();
    check(displayTimestamps.size() == 2
              && std::abs(displayTimestamps.constLast() - 300.6) < 1e-9,
          "211 must take over after the 210 capture timestamp becomes stale");

    manager.handleRadarCloud("210", onePoint(4.0F), 300.7);
    waitForFusion();
    check(displayTimestamps.size() == 3
              && std::abs(displayTimestamps.constLast() - 300.7) < 1e-9,
          "210 must immediately resume as the fusion trigger");
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testRearRs32OnlyEntersDisplayOutput();
    testStaleRearFramesAreSkippedWithoutBlockingFrontLidars();
    testRadar211OnlyTriggersFusionOutputs();
    testRadar211FallbackDoesNotDuplicateActive210AndRecovers();
    if (failures != 0) {
        std::cerr << failures << " radar fusion output assertion(s) failed\n";
        return 1;
    }
    std::cout << "All radar fusion output tests passed\n";
    return 0;
}

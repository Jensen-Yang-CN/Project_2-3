#include "PauseSnapshot.h"

#include <QCoreApplication>
#include <QFile>
#include <QImage>
#include <QTemporaryDir>

#include <cmath>
#include <cstring>
#include <iostream>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

QVector<M_PointXYZI> cloud(float marker)
{
    QVector<M_PointXYZI> result(2);
    result[0] = M_PointXYZI{marker, 2.0F, 3.0F, 4, {0, 0, 0}};
    result[1] = M_PointXYZI{5.0F, 6.0F, 7.0F, 8, {0, 0, 0}};
    return result;
}

QImage image(Qt::GlobalColor color)
{
    QImage result(4, 3, QImage::Format_RGB888);
    result.fill(color);
    return result;
}

void testCollectorChoosesClosestNonFutureFrame()
{
    pause_snapshot::Collector collector(0.5);
    collector.updateLidar210(99.70, cloud(1.0F));
    check(collector.begin(100.0), "valid pause timestamp must start collection");
    collector.updateLidar210(99.90, cloud(2.0F));
    collector.updateLidar210(99.90, cloud(4.0F));
    collector.updateLidar210(100.10, cloud(3.0F));

    const pause_snapshot::Snapshot snapshot = collector.finish();
    check(snapshot.lidar210.valid,
          "collector must retain a valid non-future lidar frame");
    check(std::abs(snapshot.lidar210.timestamp - 99.90) < 1e-9,
          "collector must choose the closest frame at or before pause time");
    check(snapshot.lidar210.data.first().x == 4.0F,
          "the latest arrival must win when synchronized timestamps are equal");
}

void testCompletenessAndTolerance()
{
    pause_snapshot::Collector completeCollector(0.5);
    completeCollector.updateLidar210(10.0, cloud(1.0F));
    completeCollector.updateLidar211(10.1, cloud(2.0F));
    completeCollector.updateCameraLeft(10.2, image(Qt::red));
    completeCollector.updateCameraRight(10.3, image(Qt::green));
    check(completeCollector.begin(10.4), "complete snapshot must begin");
    const pause_snapshot::Snapshot complete = completeCollector.finish();
    check(complete.complete(), "four fresh sources must form a complete snapshot");

    pause_snapshot::Collector missingCollector(0.5);
    missingCollector.updateLidar210(20.0, cloud(1.0F));
    missingCollector.updateLidar211(20.0, cloud(2.0F));
    missingCollector.updateCameraLeft(20.0, image(Qt::red));
    missingCollector.begin(20.1);
    const pause_snapshot::Snapshot missing = missingCollector.finish();
    check(!missing.complete() && !missing.problems.isEmpty(),
          "missing right camera must be reported as incomplete");

    pause_snapshot::Collector staleCollector(0.5);
    staleCollector.updateLidar210(29.0, cloud(1.0F));
    staleCollector.updateLidar211(30.0, cloud(2.0F));
    staleCollector.updateCameraLeft(30.0, image(Qt::red));
    staleCollector.updateCameraRight(30.0, image(Qt::green));
    staleCollector.begin(30.0);
    const pause_snapshot::Snapshot stale = staleCollector.finish();
    check(!stale.complete(), "a source older than 0.5 seconds must be incomplete");
}

void testCollectorIgnoresEmptyUpdatesAndResetsBetweenPlaybacks()
{
    pause_snapshot::Collector collector(0.5);
    collector.updateLidar210(40.0, cloud(7.0F));
    collector.updateLidar210(40.1, {});
    collector.begin(40.2);
    pause_snapshot::Snapshot snapshot = collector.finish();
    check(snapshot.lidar210.valid
              && snapshot.lidar210.data.first().x == 7.0F,
          "an empty worker update must not erase the latest complete frame");

    collector.reset();
    collector.begin(40.2);
    snapshot = collector.finish();
    check(!snapshot.lidar210.valid,
          "reset must prevent frames leaking across playback sessions");
}

void testWriterProducesBinaryPcdAndPngFiles()
{
    pause_snapshot::Collector collector(0.5);
    collector.updateLidar210(1710000000.100, cloud(1.0F));
    collector.updateLidar211(1710000000.100, cloud(2.0F));
    collector.updateCameraLeft(1710000000.100, image(Qt::red));
    collector.updateCameraRight(1710000000.100, image(Qt::green));
    collector.begin(1710000000.125);

    QTemporaryDir dir;
    const pause_snapshot::SaveResult result = pause_snapshot::saveSnapshot(
        collector.finish(), dir.path(), QTimeZone("Asia/Shanghai"));
    check(result.complete, "complete snapshot must save successfully");
    check(result.written_files.size() == 4,
          "snapshot writer must report all four files");

    const QString pcdPath = dir.filePath("210/20240310_000000_125.pcd");
    QFile pcd(pcdPath);
    check(pcd.open(QIODevice::ReadOnly), "210 PCD file must exist");
    const QByteArray pcdBytes = pcd.readAll();
    const QByteArray marker("DATA binary\n");
    const int payloadOffset = pcdBytes.indexOf(marker) + marker.size();
    check(payloadOffset >= marker.size(), "PCD must contain a binary data marker");
    check(pcdBytes.contains("FIELDS x y z intensity\n"),
          "PCD fields must describe XYZ and uint8 intensity");
    check(pcdBytes.contains("SIZE 4 4 4 1\n"),
          "PCD sizes must describe a packed 13-byte point");
    check(pcdBytes.size() - payloadOffset == 26,
          "two PCD points must use exactly 26 payload bytes without padding");

    QImage left(dir.filePath("camera_left/20240310_000000_125.png"));
    QImage right(dir.filePath("camera_right/20240310_000000_125.png"));
    check(left.size() == QSize(4, 3), "left camera PNG must be readable");
    check(right.size() == QSize(4, 3), "right camera PNG must be readable");
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testCollectorChoosesClosestNonFutureFrame();
    testCompletenessAndTolerance();
    testCollectorIgnoresEmptyUpdatesAndResetsBetweenPlaybacks();
    testWriterProducesBinaryPcdAndPngFiles();

    if (failures != 0) {
        std::cerr << failures << " pause snapshot test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All pause snapshot tests passed\n";
    return 0;
}

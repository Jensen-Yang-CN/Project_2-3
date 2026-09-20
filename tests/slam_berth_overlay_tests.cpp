#include "BerthGeometryTransform.h"
#include "SlamBerthOverlaySynchronizer.h"
#include "SlamMapBerthStore.h"

#include <Eigen/Geometry>

#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool close(double actual, double expected, double tolerance = 1e-6)
{
    return std::abs(actual - expected) <= tolerance;
}

usv::BerthMeasureResult makeLocalResult(
    double timestamp,
    usv::BerthDetectionSource source = usv::BerthDetectionSource::Realtime)
{
    usv::Berth berth;
    berth.cx = 1.0;
    berth.cy = 2.0;
    berth.w = 4.0;
    berth.l = 10.0;
    berth.angle = 0.0;
    berth.kind = usv::BerthKind::UShape;
    berth.source = source;
    berth.opening_edge = 2;
    berth.has_opening_segment = true;
    berth.opening_segment = {0.0, 2.0, 2.0, 2.0};
    berth.visible_edges = {
        {0.0, 0.0, 1.0, 0.0},
        {1.0, 0.0, 1.0, 1.0},
        {1.0, 1.0, 0.0, 1.0},
    };

    usv::BerthMeasureResult result;
    result.timestamp = timestamp;
    result.is_detected = true;
    result.expanded_berths.push_back(berth);
    result.has_highest_point = true;
    result.highest_point = Eigen::Vector3d(1.0, 2.0, 0.0);
    result.highest_points.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
    result.has_second_highest_point = true;
    result.second_highest_point = Eigen::Vector3d(-1.0, 0.0, 0.0);
    result.has_anchor_bus = true;
    result.anchor_bus_line = {-2.0, 0.0, 2.0, 0.0};
    result.debug_lines.push_back({0.0, -1.0, 0.0, 1.0});
    result.has_debug_roi = true;
    result.debug_roi = berth;
    result.debug_rois.push_back(berth);
    return result;
}

Eigen::Isometry3d mapFromLidar()
{
    constexpr double kPi = 3.14159265358979323846;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(
        0.5 * kPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(10.0, 20.0, 0.0);
    return pose;
}

void testCompleteGeometryTransform()
{
    const usv::BerthMeasureResult local = makeLocalResult(
        100.0, usv::BerthDetectionSource::CoordinateLibrary);
    const usv::BerthMeasureResult world =
        usv::berth_geometry::transformResult(local, mapFromLidar());

    check(world.expanded_berths.size() == 1,
          "transformed result must keep berth count");
    const usv::Berth &berth = world.expanded_berths.front();
    check(close(berth.cx, 8.0) && close(berth.cy, 21.0),
          "berth center must be transformed into SLAM map coordinates");
    check(close(berth.angle, 90.0),
          "berth angle must include SLAM pose yaw");
    check(berth.opening_edge == 2,
          "opening edge must keep its physical meaning after a 90 degree transform");
    check(berth.source == usv::BerthDetectionSource::CoordinateLibrary,
          "blind-zone recall source must be preserved for blue rendering");
    check(berth.visible_edges.size() == 3,
          "U-shaped berth must keep exactly three visible solid edges");
    check(close(berth.visible_edges[0].x1, 10.0)
              && close(berth.visible_edges[0].y1, 20.0)
              && close(berth.visible_edges[0].x2, 10.0)
              && close(berth.visible_edges[0].y2, 21.0),
          "visible berth edges must be transformed into map coordinates");
    check(close(world.highest_point.x(), 8.0)
              && close(world.highest_point.y(), 21.0),
          "single berth anchor must be transformed");
    check(world.highest_points.size() == 1
              && close(world.highest_points[0].x(), 10.0)
              && close(world.highest_points[0].y(), 22.0),
          "berth anchor arrays must be transformed");
    check(close(world.anchor_bus_line.x1, 10.0)
              && close(world.anchor_bus_line.y1, 18.0)
              && close(world.anchor_bus_line.x2, 10.0)
              && close(world.anchor_bus_line.y2, 22.0),
          "anchor bus must be transformed");
    check(world.debug_rois.size() == 1
              && close(world.debug_rois[0].cx, 8.0)
              && close(world.debug_rois[0].cy, 21.0),
          "debug ROI geometry must be transformed with the berth result");
}

void testPoseArrivingAfterDetectionProjectsPendingResult()
{
    SlamBerthOverlaySynchronizer sync;
    sync.addBerthResult(makeLocalResult(200.0));
    check(!sync.worldResult().has_value(),
          "berth result must wait for a matching SLAM pose");

    sync.addSlamPose(200.0, mapFromLidar());
    const auto world = sync.worldResult();
    check(world.has_value(),
          "a later SLAM pose must project a pending berth result");
    check(world && close(world->expanded_berths[0].cx, 8.0),
          "late pose projection must use map-from-lidar geometry");
}

void testDetectionArrivingAfterPoseProjectsImmediately()
{
    SlamBerthOverlaySynchronizer sync;
    sync.addSlamPose(300.0, mapFromLidar());
    sync.addBerthResult(makeLocalResult(300.1));
    check(sync.worldResult().has_value(),
          "a berth result must use an already buffered matching pose");
    check(sync.visibleAt(308.1),
          "an overlay at the eight-second boundary must remain visible");
    check(!sync.visibleAt(308.11),
          "an overlay older than eight seconds must be hidden");
}

void testUnsynchronizedPoseIsRejected()
{
    SlamBerthOverlaySynchronizer sync;
    sync.addSlamPose(400.0, mapFromLidar());
    sync.addBerthResult(makeLocalResult(400.251));
    check(!sync.worldResult().has_value(),
          "a pose farther than 250 ms must not move a berth frame");
}

void testEmptyResultAndResetClearWorldOverlay()
{
    SlamBerthOverlaySynchronizer sync;
    sync.addSlamPose(500.0, mapFromLidar());
    sync.addBerthResult(makeLocalResult(500.0));
    check(sync.worldResult().has_value(),
          "setup must create a projected berth result");

    usv::BerthMeasureResult empty;
    empty.timestamp = 500.1;
    sync.addBerthResult(empty);
    check(!sync.worldResult().has_value(),
          "an empty realtime result must clear the old SLAM overlay immediately");

    sync.addBerthResult(makeLocalResult(500.2));
    check(sync.worldResult().has_value(),
          "a later valid result may reuse retained pose history");
    sync.resetAll();
    check(!sync.worldResult().has_value(),
          "timeline reset must clear the projected overlay");
    sync.addBerthResult(makeLocalResult(500.2));
    check(!sync.worldResult().has_value(),
          "timeline reset must also remove old poses");
}

void testWidgetConsumesProjectedOverlayInSlamMode()
{
    const std::filesystem::path sourceRoot =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(sourceRoot / "src" / "gui" / "MultiLidarWidget.cpp");
    const std::string source((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
    std::string compact;
    compact.reserve(source.size());
    for (const unsigned char ch : source) {
        if (!std::isspace(ch))
            compact.push_back(static_cast<char>(ch));
    }

    check(compact.find("m_slamBerthOverlay.addBerthResult(res);")
              != std::string::npos,
          "the widget must feed every berth result into the SLAM synchronizer");
    check(compact.find(
              "m_slamBerthOverlay.addSlamPose(state.timestamp,state.pose_lidar.pose);")
              != std::string::npos,
          "the widget must feed raw T_map_lidar poses into the synchronizer");
    check(compact.find(
              "drawBerthOverlays(matrix,*slamBerthResult,false);")
              != std::string::npos,
          "SLAM mode must draw the synchronized world-coordinate berth result");
    check(compact.find("m_slamMapBerthStore.observe(*worldResult);")
              != std::string::npos,
          "synchronized world berths must be persisted for map saving");
    check(compact.find("m_slamMapBerthStore.setRecords(archive.berths);")
              != std::string::npos,
          "loading an archive must restore its persistent berth layer");
    check(compact.find(
              "drawBerthOverlays(matrix,savedBerthResult,false);")
              != std::string::npos,
          "SLAM mode must draw saved berth annotations");
    check(compact.find(
              "drawBerthOverlays(matrix,m_berthResult,true);")
              != std::string::npos,
          "point-cloud mode must keep drawing the local-coordinate berth result");
    check(compact.find(
              "if(!m_showSlamMap&&berthAlignedForPaint())drawBerthOverlays(matrix);")
              == std::string::npos,
          "the old condition that disabled berth drawing in SLAM mode must be removed");
}

void testHistoricalAndRealtimeMapLayersUseDistinctColors()
{
    const std::filesystem::path sourceRoot =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(sourceRoot / "src" / "gui" / "MultiLidarWidget.cpp");
    const std::string source((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
    std::ifstream headerInput(sourceRoot / "src" / "gui" / "MultiLidarWidget.h");
    const std::string header((std::istreambuf_iterator<char>(headerInput)),
                             std::istreambuf_iterator<char>());
    check(header.find("kSlamHistoricalMapIntensity = 220")
              != std::string::npos,
          "historical SLAM map must use a dedicated light-yellow intensity");
    check(source.find("m_slamRealtimeMapCloud") != std::string::npos,
          "realtime keyframes must be kept in a separate render layer");
    check(header.find("kSlamLiveScanIntensity = 90")
              != std::string::npos,
          "realtime SLAM data must keep the green intensity");
    check(source.find("slam_map_geo_correct::correct(archive)")
              != std::string::npos,
          "local historical map loading must use the same geo correction as export");
}

void testPersistentBerthStoreMergesRepeatedWorldObservations()
{
    SlamMapBerthStore store;
    usv::BerthMeasureResult first;
    first.timestamp = 10.0;
    first.is_detected = true;
    usv::Berth berth;
    berth.cx = 100.0;
    berth.cy = 25.0;
    berth.w = 8.0;
    berth.l = 18.0;
    berth.angle = 30.0;
    berth.kind = usv::BerthKind::UShape;
    berth.opening_edge = 2;
    first.expanded_berths.push_back(berth);

    store.observe(first);
    check(store.records().size() == 1,
          "first world berth observation must create one saved record");

    usv::BerthMeasureResult second = first;
    second.timestamp = 11.0;
    second.expanded_berths[0].cx = 100.4;
    second.expanded_berths[0].cy = 24.8;
    second.expanded_berths[0].angle = 32.0;
    store.observe(second);
    check(store.records().size() == 1,
          "nearby repeated berth observations must be merged");
    check(store.records().front().observation_count == 2,
          "merged berth must retain observation count");
    check(close(store.records().front().x, 100.2, 1e-6)
              && close(store.records().front().y, 24.9, 1e-6),
          "merged berth geometry must use a running average");

    usv::BerthMeasureResult distant = first;
    distant.timestamp = 12.0;
    distant.expanded_berths[0].cx = 130.0;
    store.observe(distant);
    check(store.records().size() == 2,
          "a distant berth must remain a separate saved record");
}

} // namespace

int main()
{
    testCompleteGeometryTransform();
    testPoseArrivingAfterDetectionProjectsPendingResult();
    testDetectionArrivingAfterPoseProjectsImmediately();
    testUnsynchronizedPoseIsRejected();
    testEmptyResultAndResetClearWorldOverlay();
    testWidgetConsumesProjectedOverlayInSlamMode();
    testHistoricalAndRealtimeMapLayersUseDistinctColors();
    testPersistentBerthStoreMergesRepeatedWorldObservations();

    if (failures != 0) {
        std::cerr << failures << " SLAM berth overlay assertion(s) failed\n";
        return 1;
    }
    std::cout << "All SLAM berth overlay tests passed\n";
    return 0;
}

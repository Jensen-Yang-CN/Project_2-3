#include "SlamMapGeoUtils.h"
#include "SlamMapGeoCorrector.h"
#include "SlamMapLoadTask.h"
#include "SlamMapDisplayBuilder.h"
#include "SlamMapStitcher.h"

#include <QCoreApplication>
#include <QTemporaryDir>

#include <Eigen/Geometry>

#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool close(double actual, double expected, double epsilon)
{
    return std::abs(actual - expected) <= epsilon;
}

M_PointXYZI makePoint(float x, float y, float z)
{
    M_PointXYZI point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1;
    return point;
}

slam_map_io::MapArchive makeArchive(const usv::SlamGeoAnchor &anchor,
                                    double x, double y)
{
    slam_map_io::MapArchive archive;
    archive.anchor = anchor;
    usv::SlamKeyframe keyframe;
    keyframe.timestamp = anchor.timestamp;
    keyframe.pose = Eigen::Isometry3d::Identity();
    keyframe.pose.translation() = Eigen::Vector3d(x, y, 0.0);
    keyframe.cloud.push_back(makePoint(0.0f, 0.0f, 0.0f));
    archive.keyframes.push_back(std::move(keyframe));
    return archive;
}

usv::SlamGeoAnchor makeAnchor()
{
    usv::SlamGeoAnchor anchor;
    anchor.valid = true;
    anchor.timestamp = 1.0;
    anchor.latitude_deg = 35.0;
    anchor.longitude_deg = 119.0;
    anchor.altitude_m = 2.0;
    return anchor;
}

std::vector<M_PointXYZI> makeAsymmetricPoints(int count = 1600)
{
    std::vector<M_PointXYZI> points;
    points.reserve(static_cast<std::size_t>(count));
    uint32_t state = 0x6d2b79f5u;
    auto randomUnit = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<double>(state & 0x00ffffffu)
             / static_cast<double>(0x01000000u);
    };
    for (int index = 0; index < count; ++index) {
        double x = 0.0;
        double y = 0.0;
        if (index % 3 != 0) {
            x = 80.0 * randomUnit();
            y = 0.08 * x + 4.0 * std::sin(0.11 * x)
              + 12.0 * (randomUnit() - 0.5);
        } else {
            y = 65.0 * randomUnit();
            x = 8.0 + 2.0 * std::sin(0.09 * y)
              + 10.0 * (randomUnit() - 0.5);
        }
        const float z = static_cast<float>(
            0.35 * std::sin(0.17 * x)
            + 0.22 * std::cos(0.13 * y)
            + 0.08 * (randomUnit() - 0.5));
        points.push_back(makePoint(x, y, z));
    }
    return points;
}

std::vector<M_PointXYZI> makeDenseGrid(double minX, double maxX,
                                       double minY, double maxY,
                                       double spacing = 0.5)
{
    std::vector<M_PointXYZI> points;
    for (double y = minY; y <= maxY; y += spacing) {
        for (double x = minX; x <= maxX; x += spacing)
            points.push_back(makePoint(static_cast<float>(x),
                                       static_cast<float>(y), 0.0f));
    }
    return points;
}

std::vector<M_PointXYZI> makeStructuredHarbourPoints()
{
    std::vector<M_PointXYZI> points;
    auto addLine = [&points](double x0, double y0,
                             double x1, double y1, int count) {
        for (int index = 0; index < count; ++index) {
            const double ratio = count > 1
                ? static_cast<double>(index)
                      / static_cast<double>(count - 1)
                : 0.0;
            const double x = x0 + ratio * (x1 - x0);
            const double y = y0 + ratio * (y1 - y0);
            points.push_back(makePoint(
                static_cast<float>(x), static_cast<float>(y),
                static_cast<float>(0.2 * std::sin(0.13 * x + 0.07 * y))));
        }
    };
    addLine(0.0, 0.0, 105.0, 0.0, 700);
    addLine(0.0, 0.0, 0.0, 82.0, 550);
    addLine(14.0, 12.0, 92.0, 66.0, 600);
    addLine(22.0, 58.0, 54.0, 58.0, 260);
    addLine(54.0, 58.0, 54.0, 76.0, 150);
    addLine(68.0, 18.0, 86.0, 18.0, 180);
    addLine(86.0, 18.0, 86.0, 43.0, 200);
    for (int index = 0; index < 420; ++index) {
        constexpr double pi = 3.14159265358979323846;
        const double angle = 1.6 * pi
            * static_cast<double>(index) / 419.0;
        points.push_back(makePoint(
            static_cast<float>(72.0 + 8.0 * std::cos(angle)),
            static_cast<float>(28.0 + 5.0 * std::sin(angle)),
            static_cast<float>(0.15 * std::cos(2.0 * angle))));
    }
    return points;
}

Eigen::Isometry3d planarTransform(double x, double y, double yawDegrees)
{
    constexpr double pi = 3.14159265358979323846;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, y, 0.0);
    transform.linear() = Eigen::AngleAxisd(
        yawDegrees * pi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return transform;
}

std::vector<M_PointXYZI> transformPoints(
    const std::vector<M_PointXYZI> &points,
    const Eigen::Isometry3d &transform)
{
    std::vector<M_PointXYZI> transformed;
    transformed.reserve(points.size());
    for (const M_PointXYZI &point : points) {
        const Eigen::Vector3d position = transform
            * Eigen::Vector3d(point.x, point.y, point.z);
        transformed.push_back(makePoint(static_cast<float>(position.x()),
                                        static_cast<float>(position.y()),
                                        static_cast<float>(position.z())));
    }
    return transformed;
}

slam_map_io::MapArchive makeSyntheticArchive(
    const usv::SlamGeoAnchor &anchor,
    std::vector<M_PointXYZI> points)
{
    slam_map_io::MapArchive archive;
    archive.anchor = anchor;
    usv::SlamKeyframe keyframe;
    keyframe.timestamp = anchor.timestamp;
    keyframe.pose = Eigen::Isometry3d::Identity();
    keyframe.cloud = std::move(points);
    archive.keyframes.push_back(std::move(keyframe));
    return archive;
}

slam_map_io::MapArchive makeRepeatedObservationArchive(
    usv::SlamGeoAnchor anchor,
    double startTimestamp,
    const std::vector<M_PointXYZI> &stablePoints,
    double dynamicOffset)
{
    slam_map_io::MapArchive archive;
    anchor.timestamp = startTimestamp;
    archive.anchor = anchor;
    for (int frameIndex = 0; frameIndex < 4; ++frameIndex) {
        usv::SlamKeyframe keyframe;
        keyframe.timestamp = startTimestamp + frameIndex;
        keyframe.pose = Eigen::Isometry3d::Identity();
        keyframe.cloud = stablePoints;
        for (int pointIndex = 0; pointIndex < 600; ++pointIndex) {
            const double x = dynamicOffset + 25.0 * frameIndex
                + 0.05 * (pointIndex % 30);
            const double y = -30.0 + 0.05 * (pointIndex / 30);
            keyframe.cloud.push_back(makePoint(
                static_cast<float>(x), static_cast<float>(y), 0.0f));
        }
        archive.keyframes.push_back(std::move(keyframe));
    }
    return archive;
}

slam_map_io::MapArchive makeTrajectoryArchive(
    const usv::SlamGeoAnchor &anchor,
    double startTimestamp,
    int keyframeCount,
    double referenceTimestamp,
    const Eigen::Isometry3d &archiveFromReference)
{
    slam_map_io::MapArchive archive;
    archive.anchor = anchor;
    archive.keyframes.reserve(static_cast<std::size_t>(keyframeCount));
    for (int index = 0; index < keyframeCount; ++index) {
        usv::SlamKeyframe keyframe;
        keyframe.timestamp = startTimestamp + static_cast<double>(index);
        const double x = keyframe.timestamp - referenceTimestamp;
        const double y = 0.03 * x * x + 0.4 * std::sin(0.2 * x);
        Eigen::Isometry3d referencePose = Eigen::Isometry3d::Identity();
        referencePose.translation() = Eigen::Vector3d(x, y, 0.0);
        keyframe.pose = archiveFromReference * referencePose;
        keyframe.cloud.push_back(makePoint(0.0f, 0.0f, 0.0f));
        archive.keyframes.push_back(std::move(keyframe));
    }
    return archive;
}

slam_map_io::MapArchive makeGeoreferencedDriftArchive(
    int keyframeCount, int spikeIndex = -1)
{
    slam_map_io::MapArchive archive;
    archive.anchor = makeAnchor();
    archive.keyframes.reserve(static_cast<std::size_t>(keyframeCount));
    for (int index = 0; index < keyframeCount; ++index) {
        const double timestamp = 100.0 + static_cast<double>(index);
        const double trueX = 2.0 * static_cast<double>(index);
        const double driftX = trueX + 0.20 * static_cast<double>(index);
        const double driftY = 0.08 * static_cast<double>(index);

        usv::SlamKeyframe keyframe;
        keyframe.timestamp = timestamp;
        keyframe.pose = planarTransform(driftX, driftY, 0.0);
        keyframe.cloud.push_back(makePoint(0.0f, 0.0f, 0.0f));
        keyframe.geo_reference.valid = true;
        keyframe.geo_reference.timestamp = timestamp;
        keyframe.geo_reference.sync_error_sec = 0.01;
        keyframe.geo_reference.pose_lidar_enu = planarTransform(
            trueX + (index == spikeIndex ? 30.0 : 0.0), 0.0, 0.0);
        archive.keyframes.push_back(std::move(keyframe));
    }
    return archive;
}

void testGeoDriftCorrection(int &failures)
{
    auto source = makeGeoreferencedDriftArchive(12);
    usv::SlamMapBerth berth;
    berth.id = 5;
    berth.timestamp = 106.0;
    berth.x = 5.0;
    berth.y = 1.0;
    berth.width = 8.0;
    berth.length = 18.0;
    berth.angle_deg = 135.0;
    berth.kind = 1;
    berth.opening_edge = 1;
    berth.confidence = 1.0;
    source.berths.push_back(berth);
    const auto result = slam_map_geo_correct::correct(source);
    check(result.diagnostic.applied,
          "three or more GNSS references should enable drift correction",
          failures);
    check(result.diagnostic.valid_references == 12,
          "all clean GNSS references should be retained", failures);
    check(close(result.archive.keyframes.back().pose.translation().x(),
                22.0, 0.20)
              && close(result.archive.keyframes.back().pose.translation().y(),
                       0.0, 0.20),
          "linear SLAM drift should be corrected to the ENU trajectory",
          failures);
    check(result.archive.berths.size() == 1
              && std::isfinite(result.archive.berths[0].x)
              && !close(result.archive.berths[0].x, berth.x, 1.0e-6),
          "berth annotations should receive the same timestamped geo correction",
          failures);
    check(result.archive.berths.size() == 1
              && close(result.archive.berths[0].angle_deg, -45.0, 1.0e-6)
              && result.archive.berths[0].opening_edge == 3,
          "geo correction must keep the physical berth opening edge",
          failures);
}

void testStitchingKeepsBerthOpeningEdge(int &failures)
{
    const usv::SlamGeoAnchor anchor = makeAnchor();
    auto archive = makeArchive(anchor, 0.0, 0.0);
    usv::SlamMapBerth berth;
    berth.id = 1;
    berth.timestamp = anchor.timestamp;
    berth.x = 2.0;
    berth.y = 3.0;
    berth.width = 8.0;
    berth.length = 18.0;
    berth.angle_deg = 135.0;
    berth.kind = 1;
    berth.opening_edge = 1;
    berth.observation_count = 1;
    berth.confidence = 1.0;
    archive.berths.push_back(berth);

    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("berth.slammap"), archive}});
    check(result.success && result.archive.berths.size() == 1
              && close(result.archive.berths[0].angle_deg, -45.0, 1.0e-6)
              && result.archive.berths[0].opening_edge == 3,
          "stitching must keep the physical berth opening edge",
          failures);
}

void testGeoDriftCorrectionRejectsSpike(int &failures)
{
    const auto source = makeGeoreferencedDriftArchive(12, 6);
    const auto result = slam_map_geo_correct::correct(source);
    check(result.diagnostic.applied,
          "one GNSS spike must not disable otherwise valid correction",
          failures);
    check(result.diagnostic.rejected_references >= 1,
          "GNSS position spike should be rejected", failures);
    check(close(result.archive.keyframes[6].pose.translation().x(),
                12.0, 0.35),
          "rejected GNSS spike should be filled from neighbouring references",
          failures);
}

void testGeoDriftCorrectionRejectsStaleReference(int &failures)
{
    auto source = makeGeoreferencedDriftArchive(12);
    source.keyframes[5].geo_reference.sync_error_sec = 0.30;
    const auto result = slam_map_geo_correct::correct(source);
    check(result.diagnostic.applied
              && result.diagnostic.rejected_references >= 1,
          "stale navigation reference should be rejected from correction",
          failures);
    check(close(result.archive.keyframes[5].pose.translation().x(),
                10.0, 0.35),
          "stale reference should be bridged by neighbouring corrections",
          failures);
}

void testGeoDriftCorrectionNeedsThreeReferences(int &failures)
{
    const auto source = makeGeoreferencedDriftArchive(2);
    const auto result = slam_map_geo_correct::correct(source);
    check(!result.diagnostic.applied,
          "fewer than three GNSS references should leave map unchanged",
          failures);
    check(result.archive.keyframes[1].pose.matrix().isApprox(
              source.keyframes[1].pose.matrix(), 1.0e-12),
          "insufficient GNSS references must not alter keyframe poses",
          failures);
}

void testAnchorOffsetAndCoarseMerge(int &failures)
{
    usv::SlamGeoAnchor base;
    base.valid = true;
    base.timestamp = 1.0;
    base.latitude_deg = 0.0;
    base.longitude_deg = 0.0;
    base.altitude_m = 3.0;
    usv::SlamGeoAnchor east = base;
    east.timestamp = 2.0;
    east.longitude_deg = 0.0001;
    east.altitude_m = 6.0;

    Eigen::Vector3d offset;
    QString error;
    check(slam_map_geo::anchorOffsetEnu(base, east, offset, &error),
          "valid anchors should convert", failures);
    check(close(offset.x(), 11.131949, 0.02)
              && close(offset.y(), 0.0, 0.02)
              && close(offset.z(), 3.0, 0.02),
          "WGS84 offset should be east/north/up", failures);

    slam_map_stitch::Input a{
        QStringLiteral("a.slammap"), makeArchive(base, 5.0, 0.0)};
    slam_map_stitch::Input b{
        QStringLiteral("b.slammap"), makeArchive(east, 2.0, 1.0)};
    usv::SlamMapBerth baseBerth;
    baseBerth.id = 7;
    baseBerth.timestamp = 1.0;
    baseBerth.x = 1.0;
    baseBerth.y = 2.0;
    baseBerth.width = 8.0;
    baseBerth.length = 18.0;
    baseBerth.kind = 1;
    baseBerth.confidence = 1.0;
    a.archive.berths.push_back(baseBerth);
    usv::SlamMapBerth eastBerth = baseBerth;
    eastBerth.id = 8;
    eastBerth.timestamp = 2.0;
    eastBerth.x = 1.0;
    eastBerth.y = 2.0;
    b.archive.berths.push_back(eastBerth);
    slam_map_stitch::Options options;
    options.enable_icp = false;
    const auto result = slam_map_stitch::stitch({a, b}, options);
    check(result.success && result.archive.keyframes.size() == 2,
          "coarse stitching should merge both archives", failures);
    if (!result.success || result.archive.keyframes.size() != 2)
        return;
    check(result.archive.keyframes[0].keyframe_id == 0
              && result.archive.keyframes[1].keyframe_id == 1,
          "merged keyframes should receive unique sequential ids", failures);
    check(close(result.archive.keyframes[1].pose.translation().x(),
                13.131949, 0.03),
          "source keyframe pose should move into base ENU", failures);
    check(result.archive.berths.size() == 2
              && close(result.archive.berths[1].x, 12.131949, 0.03),
          "source berth annotations should receive the same ENU stitch offset",
          failures);
}

void testBoundedIcpCorrection(int &failures)
{
    const Eigen::Isometry3d expected = planarTransform(1.2, -0.6, 2.0);
    const std::vector<M_PointXYZI> points = makeAsymmetricPoints();
    const auto base = makeSyntheticArchive(makeAnchor(), points);
    const auto source = makeSyntheticArchive(
        makeAnchor(), transformPoints(points, expected.inverse()));
    slam_map_stitch::Options options;
    options.enable_icp = true;
    options.allow_yaw_correction = true;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("source.slammap"), source}}, options);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorAndIcp,
          "reliable overlap should use ICP", failures);
    if (!result.success || result.diagnostics.size() != 2)
        return;
    if (!close(result.diagnostics[1].correction_translation_m,
               std::hypot(1.2, 0.6), 0.15)
        || !close(result.diagnostics[1].correction_yaw_deg, 2.0, 0.2)) {
        std::cerr << "ICP diagnostic: translation="
                  << result.diagnostics[1].correction_translation_m
                  << ", yaw=" << result.diagnostics[1].correction_yaw_deg
                  << ", fitness=" << result.diagnostics[1].fitness_m2
                  << '\n';
    }
    check(close(result.diagnostics[1].correction_translation_m,
                std::hypot(1.2, 0.6), 0.15)
              && close(result.diagnostics[1].correction_yaw_deg, 2.0, 0.2),
          "ICP should recover the bounded planar error", failures);
}

void testLargeAnchorResidualUsesGuardedIcpByDefault(int &failures)
{
    const Eigen::Isometry3d expected = planarTransform(12.0, -4.0, 0.0);
    const std::vector<M_PointXYZI> points = makeStructuredHarbourPoints();
    const auto base = makeSyntheticArchive(makeAnchor(), points);
    const auto source = makeSyntheticArchive(
        makeAnchor(), transformPoints(points, expected.inverse()));

    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("source.slammap"), source}});
    const slam_map_stitch::Options defaults;
    check(close(defaults.max_correction_translation_m, 3.0, 1.0e-9),
          "automatic ICP correction must be bounded to three metres",
          failures);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "default stitching must reject a large correction that could create a false match",
          failures);
    if (!result.success || result.diagnostics.size() != 2)
        return;
    check(result.diagnostics[1].correction_translation_m > 3.0
              && close(result.diagnostics[1].correction_yaw_deg,
                       0.0, 1.0e-6),
          "diagnostics should expose the rejected large translation without applying it",
          failures);
}

void testDefaultRegistrationNeverChangesEnuYaw(int &failures)
{
    const slam_map_stitch::Options defaults;
    check(!defaults.allow_yaw_correction,
          "automatic map stitching must preserve the ENU yaw by default",
          failures);

    const Eigen::Isometry3d misleadingRigidMatch =
        planarTransform(8.0, -3.0, 6.0);
    const std::vector<M_PointXYZI> points = makeStructuredHarbourPoints();
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"),
         makeSyntheticArchive(makeAnchor(), points)},
        {QStringLiteral("repetitive_source.slammap"),
         makeSyntheticArchive(
             makeAnchor(),
             transformPoints(points, misleadingRigidMatch.inverse()))}});
    check(result.success && result.diagnostics.size() == 2,
          "default registration should finish safely on repetitive structures",
          failures);
    if (!result.success || result.diagnostics.size() != 2)
        return;
    check(close(result.diagnostics[1].correction_yaw_deg, 0.0, 1.0e-6),
          "default registration must never rotate a map away from its ENU axes",
          failures);
    if (result.archive.keyframes.size() >= 2) {
        check(result.archive.keyframes[1].pose.linear().isApprox(
                  Eigen::Matrix3d::Identity(), 1.0e-6),
              "the appended default map pose must retain identity ENU rotation",
              failures);
    }
}

void testStableObservationsFilterSingleFrameClutter(int &failures)
{
    const Eigen::Isometry3d expected = planarTransform(4.0, -2.0, 0.0);
    const std::vector<M_PointXYZI> stable = makeStructuredHarbourPoints();
    const auto base = makeRepeatedObservationArchive(
        makeAnchor(), 1.0, stable, 150.0);
    const auto source = makeRepeatedObservationArchive(
        makeAnchor(), 20.0, transformPoints(stable, expected.inverse()),
        -180.0);
    slam_map_stitch::Options options;
    options.max_correction_translation_m = 6.0;
    options.correction_boundary_margin_m = 0.5;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("source.slammap"), source}}, options);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorAndIcp,
          "stable repeated structures should drive registration despite single-frame clutter",
          failures);
    if (!result.success || result.diagnostics.size() != 2)
        return;
    const slam_map_stitch::Diagnostic &diagnostic = result.diagnostics[1];
    check(diagnostic.target_used_stable_points
              && diagnostic.source_used_stable_points
              && diagnostic.target_registration_points > 0
              && diagnostic.source_registration_points > 0,
          "multi-keyframe registration should select stable observation cells",
          failures);
    check(close(diagnostic.correction_translation_m,
                std::hypot(4.0, 2.0), 0.6)
              && close(diagnostic.correction_yaw_deg, 0.0, 1.0e-6),
          "single-frame clutter must not steer the accepted correction",
          failures);
}

void testIcpFallbacks(int &failures)
{
    auto sparse = makeSyntheticArchive(makeAnchor(), makeAsymmetricPoints(100));
    slam_map_stitch::Options sparseOptions;
    sparseOptions.enable_icp = true;
    auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), sparse},
        {QStringLiteral("sparse.slammap"), sparse}}, sparseOptions);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "insufficient overlap should keep anchor alignment", failures);

    slam_map_stitch::Options options;
    options.enable_icp = true;
    options.max_correspondence_m = 30.0;
    const std::vector<M_PointXYZI> points = makeAsymmetricPoints();
    auto shifted = makeSyntheticArchive(
        makeAnchor(), transformPoints(
                          points, planarTransform(28.0, 0.0, 0.0).inverse()));
    result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"),
         makeSyntheticArchive(makeAnchor(), points)},
        {QStringLiteral("far.slammap"), shifted}}, options);
    if (result.success && result.diagnostics.size() == 2
        && result.diagnostics[1].mode
               != slam_map_stitch::AlignmentMode::AnchorOnly) {
        std::cerr << "Far ICP diagnostic: translation="
                  << result.diagnostics[1].correction_translation_m
                  << ", yaw=" << result.diagnostics[1].correction_yaw_deg
                  << ", fitness=" << result.diagnostics[1].fitness_m2
                  << ", message="
                  << result.diagnostics[1].message.toLocal8Bit().constData()
                  << '\n';
    }
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "correction beyond the guarded search envelope should be rejected",
          failures);
}

void testNearSearchBoundaryIsRejected(int &failures)
{
    const std::vector<M_PointXYZI> points = makeStructuredHarbourPoints();
    const auto source = makeSyntheticArchive(
        makeAnchor(), transformPoints(
                          points,
                          planarTransform(19.0, 0.0, 0.0).inverse()));
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"),
         makeSyntheticArchive(makeAnchor(), points)},
        {QStringLiteral("near_boundary.slammap"), source}});
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "a correction close to the search boundary must fall back to ENU",
          failures);
}

void testTemporalSeamTrimsRepeatedKeyframes(int &failures)
{
    usv::SlamGeoAnchor anchor = makeAnchor();
    anchor.timestamp = 100.0;
    const auto base = makeTrajectoryArchive(
        anchor, 100.0, 21, 100.0, Eigen::Isometry3d::Identity());
    const Eigen::Isometry3d expectedCorrection =
        planarTransform(3.0, -1.5, 3.0);
    const auto source = makeTrajectoryArchive(
        anchor, 110.0, 21, 100.0, expectedCorrection.inverse());

    slam_map_stitch::Options options;
    options.enable_temporal_seam = true;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("continued.slammap"), source}}, options);
    check(result.success && result.archive.keyframes.size() == 31,
          "temporal seam should append only ten new keyframes", failures);
    check(result.diagnostics.size() == 2
              && result.diagnostics[0].appended_keyframes == 21,
          "base diagnostics should report its appended keyframes", failures);
    check(result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::TemporalSeam,
          "partially overlapping maps should use a temporal seam", failures);
    if (!result.success || result.diagnostics.size() != 2)
        return;
    check(result.diagnostics[1].trimmed_keyframes == 11
              && result.diagnostics[1].appended_keyframes == 10,
          "temporal seam diagnostics should report trimmed and appended frames",
          failures);
    bool sequentialIds = true;
    for (std::size_t index = 0; index < result.archive.keyframes.size(); ++index) {
        if (result.archive.keyframes[index].keyframe_id != index) {
            sequentialIds = false;
            break;
        }
    }
    check(sequentialIds,
          "temporal seam output should keep sequential keyframe ids", failures);
    check(close(result.archive.keyframes.back().pose.translation().x(),
                30.0, 0.15),
          "new keyframes should be transformed into the target trajectory",
          failures);
}

void testFullyCoveredMapIsSkipped(int &failures)
{
    usv::SlamGeoAnchor anchor = makeAnchor();
    anchor.timestamp = 100.0;
    const auto base = makeTrajectoryArchive(
        anchor, 100.0, 21, 100.0, Eigen::Isometry3d::Identity());
    const auto duplicate = makeTrajectoryArchive(
        anchor, 105.0, 11, 100.0,
        planarTransform(2.0, 1.0, -2.0).inverse());

    slam_map_stitch::Options options;
    options.enable_temporal_seam = true;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("duplicate.slammap"), duplicate}}, options);
    check(result.success && result.archive.keyframes.size() == 21,
          "a fully covered map should not duplicate keyframes", failures);
    check(result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::DuplicateSkipped
              && result.diagnostics[1].trimmed_keyframes == 11
              && result.diagnostics[1].appended_keyframes == 0,
          "fully covered map diagnostics should report a duplicate skip",
          failures);
}

void testPaddingCannotCreateSpatialOverlap(int &failures)
{
    const auto base = makeSyntheticArchive(
        makeAnchor(), makeDenseGrid(0.0, 20.0, 0.0, 100.0));
    const auto separated = makeSyntheticArchive(
        makeAnchor(), makeDenseGrid(24.0, 44.0, 0.0, 100.0));
    slam_map_stitch::Options options;
    options.enable_icp = true;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("separated.slammap"), separated}}, options);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "padding must not manufacture spatial overlap for ICP", failures);
}

void testIcpMustImproveAnchorBaseline(int &failures)
{
    const std::vector<M_PointXYZI> points = makeAsymmetricPoints();
    const auto base = makeSyntheticArchive(makeAnchor(), points);
    const auto alreadyAligned = makeSyntheticArchive(makeAnchor(), points);
    slam_map_stitch::Options options;
    options.enable_icp = true;
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("already_aligned.slammap"), alreadyAligned}}, options);
    check(result.success && result.diagnostics.size() == 2
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "ICP must be rejected when it does not improve anchor alignment",
          failures);
}

void testDefaultUsesDirectEnuAndOnlyTrimsDuplicateTime(int &failures)
{
    const slam_map_stitch::Options defaults;
    check(defaults.enable_icp && !defaults.enable_temporal_seam,
          "default stitching should use guarded spatial ICP but no temporal rigid correction",
          failures);

    usv::SlamGeoAnchor anchor = makeAnchor();
    anchor.timestamp = 100.0;
    const auto base = makeTrajectoryArchive(
        anchor, 100.0, 21, 100.0, Eigen::Isometry3d::Identity());
    const Eigen::Isometry3d sourceFromEnu =
        planarTransform(3.0, -1.5, 3.0).inverse();
    const auto source = makeTrajectoryArchive(
        anchor, 110.0, 21, 100.0, sourceFromEnu);

    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("continued.slammap"), source}});
    check(result.success && result.archive.keyframes.size() == 31,
          "default ENU stitching should trim duplicate timestamps and append the new tail",
          failures);
    if (!result.success || result.diagnostics.size() != 2
        || result.archive.keyframes.empty()) {
        return;
    }

    const slam_map_stitch::Diagnostic &diagnostic = result.diagnostics[1];
    check(diagnostic.mode == slam_map_stitch::AlignmentMode::AnchorOnly
              && diagnostic.trimmed_keyframes == 11
              && diagnostic.appended_keyframes == 10,
          "default overlap diagnostics should report anchor-only temporal deduplication",
          failures);
    check(close(diagnostic.correction_translation_m, 0.0, 1.0e-9)
              && close(diagnostic.correction_yaw_deg, 0.0, 1.0e-9),
          "default ENU stitching must not report a secondary rigid correction",
          failures);
    check(result.archive.keyframes.back().pose.matrix().isApprox(
              source.keyframes.back().pose.matrix(), 1.0e-9),
          "new tail poses must retain the source ENU pose after anchor translation only",
          failures);
}

void testLoadTaskAndSourceColours(int &failures)
{
    QTemporaryDir directory;
    check(directory.isValid(), "temporary map directory should exist",
          failures);
    if (!directory.isValid())
        return;

    const usv::SlamGeoAnchor anchor = makeAnchor();
    const slam_map_io::MapArchive first = makeArchive(anchor, 0.0, 0.0);
    const slam_map_io::MapArchive second = makeArchive(anchor, 5.0, 0.0);
    const QString firstPath = directory.filePath(QStringLiteral("first.slammap"));
    const QString secondPath = directory.filePath(QStringLiteral("second.slammap"));
    QString error;
    check(slam_map_io::saveArchive(firstPath, first, &error)
              && slam_map_io::saveArchive(secondPath, second, &error),
          "map task fixtures should save", failures);

    slam_map_stitch::Options options;
    options.enable_icp = false;
    const slam_map_load::Result loaded = slam_map_load::loadAndStitch(
        {firstPath, secondPath}, options);
    check(loaded.success
              && loaded.stitched.archive.keyframes.size() == 2
              && loaded.stitched.source_ranges.size() == 2,
          "file task should load and stitch both archives", failures);
    if (!loaded.success)
        return;

    const QVector<M_PointXYZI> display = slam_map_display::rebuild(
        loaded.stitched.archive, loaded.stitched.source_ranges, 100);
    check(display.size() == 2
              && display[0].intensity == 90
              && display[1].intensity == 130,
          "display cloud should preserve source colours", failures);

    const QVector<M_PointXYZI> cappedDisplay = slam_map_display::rebuild(
        loaded.stitched.archive, loaded.stitched.source_ranges, 1);
    check(cappedDisplay.size() <= 1,
          "display cloud must obey its point limit", failures);

    const QString mergedPath =
        directory.filePath(QStringLiteral("merged.slammap"));
    check(slam_map_io::saveArchive(
              mergedPath, loaded.stitched.archive, &error),
          "stitched archive should save", failures);
    const slam_map_load::Result reloaded = slam_map_load::loadAndStitch(
        {mergedPath}, options);
    check(reloaded.success
              && reloaded.stitched.archive.keyframes.size() == 2
              && reloaded.stitched.source_ranges.size() == 1,
          "saved stitch should reload as one base map", failures);
    if (reloaded.success) {
        const QVector<M_PointXYZI> unified = slam_map_display::rebuild(
            reloaded.stitched.archive, reloaded.stitched.source_ranges, 100);
        check(unified.size() == 2
                  && unified[0].intensity == 90
                  && unified[1].intensity == 90,
              "a reloaded stitch should use the single base colour", failures);
    }

    const QString thirdPath = directory.filePath(QStringLiteral("third.slammap"));
    check(slam_map_io::saveArchive(
              thirdPath, makeArchive(anchor, 10.0, 0.0), &error),
          "continued stitch fixture should save", failures);
    const slam_map_load::Result continued = slam_map_load::loadAndStitch(
        {mergedPath, thirdPath}, options);
    check(continued.success
              && continued.stitched.archive.keyframes.size() == 3
              && continued.stitched.archive.keyframes[0].keyframe_id == 0
              && continued.stitched.archive.keyframes[2].keyframe_id == 2,
          "a saved stitch should accept another map and re-id all keyframes",
          failures);

    const QString legacyPath =
        directory.filePath(QStringLiteral("legacy.pcd"));
    QVector<M_PointXYZI> legacyCloud;
    legacyCloud.append(makePoint(1.0f, 2.0f, 3.0f));
    check(slam_map_io::save(legacyPath, legacyCloud, &error),
          "legacy point-cloud fixture should save", failures);
    const slam_map_load::Result legacyLoaded =
        slam_map_load::loadAndStitch({legacyPath}, options);
    check(legacyLoaded.success
              && legacyLoaded.stitched.archive.keyframes.size() == 1
              && legacyLoaded.stitched.archive.keyframes[0].cloud.size() == 1,
          "load task should wrap a legacy .pcd as one local keyframe",
          failures);
}

void testLoadTaskBuildsAndReusesReadOnlyTileCache(int &failures)
{
    QTemporaryDir directory;
    check(directory.isValid(), "tile load task temp directory should exist",
          failures);
    if (!directory.isValid())
        return;

    const usv::SlamGeoAnchor anchor = makeAnchor();
    const QString firstPath = directory.filePath(QStringLiteral("first.slammap"));
    const QString secondPath = directory.filePath(QStringLiteral("second.slammap"));
    QString error;
    slam_map_io::MapArchive firstArchive = makeArchive(anchor, 0.0, 0.0);
    usv::SlamMapBerth berth;
    berth.id = 3;
    berth.timestamp = 1.0;
    berth.x = 2.0;
    berth.y = 3.0;
    berth.width = 8.0;
    berth.length = 18.0;
    berth.kind = 1;
    berth.confidence = 1.0;
    firstArchive.berths.push_back(berth);
    check(slam_map_io::saveArchive(firstPath, firstArchive, &error)
              && slam_map_io::saveArchive(secondPath, makeArchive(anchor, 55.0, 0.0), &error),
          "tile load task fixtures should save", failures);

    slam_map_stitch::Options stitchOptions;
    stitchOptions.enable_icp = false;
    slam_tile::BuildOptions tileOptions;
    const QString cacheRoot = directory.filePath(QStringLiteral("tiled_cache"));
    const slam_map_load::Result first = slam_map_load::loadAndPrepareTiles(
        {firstPath, secondPath}, cacheRoot, stitchOptions, tileOptions);
    check(first.success && first.tile_build.success
              && !first.tile_build.cache_hit
              && QFile::exists(first.tile_build.manifest_path)
              && first.tile_build.manifest.berths.size() == 1,
          "first tiled load should stitch in ENU and publish a manifest",
          failures);

    const slam_map_load::Result second = slam_map_load::loadAndPrepareTiles(
        {firstPath, secondPath}, cacheRoot, stitchOptions, tileOptions);
    check(second.success && second.tile_build.success
              && second.tile_build.cache_hit
              && second.stitched.archive.keyframes.empty()
              && second.tile_build.manifest.berths.size() == 1,
          "second tiled load should reuse the read-only cache without loading archives",
          failures);

    if (!second.tile_build.manifest.tiles.isEmpty()) {
        const QString missingTile = QDir(second.tile_build.cache_directory)
            .absoluteFilePath(
                second.tile_build.manifest.tiles.first().relative_path);
        check(QFile::remove(missingTile),
              "tile cache fixture should remove one tile", failures);
        const slam_map_load::Result repaired =
            slam_map_load::loadAndPrepareTiles(
                {firstPath, secondPath}, cacheRoot,
                stitchOptions, tileOptions);
        check(repaired.success && repaired.tile_build.success
                  && !repaired.tile_build.cache_hit
                  && QFile::exists(missingTile),
              "a cache with a missing tile must rebuild from source maps",
              failures);
    }
}

void testLoadTaskAppliesGeoCorrection(int &failures)
{
    QTemporaryDir directory;
    check(directory.isValid(), "geo correction temp directory should exist",
          failures);
    if (!directory.isValid())
        return;
    const QString path = directory.filePath(QStringLiteral("drift.slammap"));
    QString error;
    check(slam_map_io::saveArchive(
              path, makeGeoreferencedDriftArchive(12), &error),
          "georeferenced drift fixture should save", failures);
    slam_map_stitch::Options options;
    options.enable_icp = false;
    const slam_map_load::Result loaded =
        slam_map_load::loadAndStitch({path}, options);
    check(loaded.success && loaded.geo_corrections.size() == 1
              && loaded.geo_corrections[0].format_major == 2
              && loaded.geo_corrections[0].format_minor == 1
              && loaded.geo_corrections[0].diagnostic.applied,
          "load task should apply and report per-keyframe ENU correction",
          failures);
    if (loaded.success && !loaded.stitched.archive.keyframes.empty()) {
        check(close(loaded.stitched.archive.keyframes.back()
                        .pose.translation().x(),
                    22.0, 0.20),
              "loaded archive should contain corrected keyframe poses",
              failures);
    }
}

void testDisplayVoxelDeduplication(int &failures)
{
    slam_map_io::MapArchive archive;
    archive.anchor = makeAnchor();
    for (int index = 0; index < 3; ++index) {
        usv::SlamKeyframe keyframe;
        keyframe.timestamp = 1.0 + index;
        keyframe.pose = Eigen::Isometry3d::Identity();
        keyframe.cloud.push_back(makePoint(
            static_cast<float>(0.02 * index),
            static_cast<float>(0.01 * index), 0.0f));
        archive.keyframes.push_back(std::move(keyframe));
    }
    usv::SlamKeyframe elevated;
    elevated.timestamp = 4.0;
    elevated.pose = Eigen::Isometry3d::Identity();
    elevated.cloud.push_back(makePoint(0.01f, 0.01f, 1.0f));
    archive.keyframes.push_back(std::move(elevated));
    slam_map_display::BuildStats stats;
    const QVector<M_PointXYZI> display = slam_map_display::rebuild(
        archive, {}, 100, 0.3, &stats);
    check(display.size() == 2 && stats.duplicate_points_removed == 2,
          "0.3 metre XYZ voxels should remove repeated display points",
          failures);
    check(close(display.back().z, 1.0, 1.0e-6),
          "points at different heights must survive XYZ deduplication",
          failures);
}

void testPointCloudMapCountAtHalfMetreResolution(int &failures)
{
    slam_map_io::MapArchive archive;
    archive.anchor = makeAnchor();

    usv::SlamKeyframe first;
    first.pose = Eigen::Isometry3d::Identity();
    first.cloud.push_back(makePoint(0.10f, 0.10f, 0.10f));
    first.cloud.push_back(makePoint(0.49f, 0.49f, 0.49f));
    archive.keyframes.push_back(std::move(first));

    usv::SlamKeyframe second;
    second.pose = Eigen::Isometry3d::Identity();
    second.pose.translation().x() = 0.5;
    second.cloud.push_back(makePoint(0.10f, 0.10f, 0.10f));
    second.cloud.push_back(makePoint(0.10f, 0.10f, 0.60f));
    archive.keyframes.push_back(std::move(second));

    const slam_map_display::PointCountStats stats =
        slam_map_display::countPointsAtResolution(archive, 0.5);
    check(stats.source_points == 4 && stats.valid_world_points == 4,
          "point count must inspect every source map point", failures);
    check(stats.resolution_points == 3,
          "0.5 metre XYZ point count must retain one point per occupied voxel",
          failures);
}

void testStableRegionAnalysisUsesDistinctKeyframesAndClusters(int &failures)
{
    slam_map_io::MapArchive archive;
    archive.anchor = makeAnchor();

    usv::SlamKeyframe first;
    first.pose = Eigen::Isometry3d::Identity();
    first.cloud.push_back(makePoint(0.10f, 0.10f, 0.0f));
    first.cloud.push_back(makePoint(0.20f, 0.20f, 0.5f));
    first.cloud.push_back(makePoint(1.10f, 0.10f, 1.0f));
    first.cloud.push_back(makePoint(10.10f, 0.10f, 2.0f));
    first.cloud.push_back(makePoint(20.10f, 0.10f, 3.0f));
    archive.keyframes.push_back(std::move(first));

    usv::SlamKeyframe second;
    second.pose = Eigen::Isometry3d::Identity();
    second.cloud.push_back(makePoint(0.15f, 0.15f, 1.0f));
    second.cloud.push_back(makePoint(1.15f, 0.15f, 2.0f));
    archive.keyframes.push_back(std::move(second));

    usv::SlamKeyframe third;
    third.pose = Eigen::Isometry3d::Identity();
    third.cloud.push_back(makePoint(10.15f, 0.15f, 4.0f));
    archive.keyframes.push_back(std::move(third));

    const slam_map_display::StableRegionAnalysis analysis =
        slam_map_display::analyzeStableRegions(archive, 1.0, 2, 1);
    check(analysis.candidate_cells == 4 && analysis.stable_cells == 3,
          "stable analysis must count observations once per keyframe",
          failures);
    check(analysis.regions.size() == 2,
          "adjacent stable cells should merge while separated cells remain separate",
          failures);
    if (analysis.regions.size() == 2) {
        const slam_map_display::StableRegion &largest =
            analysis.regions.front();
        check(largest.stable_cells == 2 && largest.point_count == 5,
              "the largest stable region should aggregate adjacent cells and points",
              failures);
        check(largest.observed_keyframes == 2,
              "region keyframe count should use distinct keyframes",
              failures);
        check(close(largest.min_x, 0.10, 1.0e-5)
                  && close(largest.max_x, 1.15, 1.0e-5)
                  && close(largest.min_z, 0.0, 1.0e-5)
                  && close(largest.max_z, 2.0, 1.0e-5),
              "stable region bounds should describe its world points",
              failures);
    }

    const slam_map_display::StableRegionAnalysis filtered =
        slam_map_display::analyzeStableRegions(archive, 1.0, 2, 2);
    check(filtered.regions.size() == 1
              && filtered.discarded_small_regions == 1,
          "small isolated stable regions should be discarded",
          failures);
}

void testWidgetArchiveDisplayWiring(int &failures)
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

    check(compact.find(
              "slam_map_display::rebuild(archive,sourceRanges,kMaxSlamMapPoints,0.3,&localStats)")
              != std::string::npos,
          "widget should rebuild an archive with source colours", failures);
    check(compact.find("m_viewPanX=0.5f*(minX+maxX);")
              != std::string::npos,
          "stitched maps should initially centre on their bounds", failures);
    check(compact.find(std::string("archive.") + "legacy_" + "format")
              == std::string::npos,
          "widget must not retain legacy PCD handling", failures);
}

void testMainWindowMapFlowWiring(int &failures)
{
    const std::filesystem::path sourceRoot =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(sourceRoot / "src" / "gui" / "mainwindow.cpp");
    const std::string source((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
    std::string compact;
    compact.reserve(source.size());
    for (const unsigned char ch : source) {
        if (!std::isspace(ch))
            compact.push_back(static_cast<char>(ch));
    }

    check(compact.find(
              "QTimer::singleShot(0,this,&MainWindow::restoreLastSlamMap);")
              == std::string::npos,
          "startup must not restore a SLAM map automatically", failures);
    check(compact.find("QFileDialog::getOpenFileNames(")
              != std::string::npos,
          "map loading should allow multiple archives", failures);
    check(compact.find("slam_map_load::loadAndStitch(filePaths)")
              != std::string::npos,
          "selected archives should use the background load task", failures);
    check(compact.find("m_offlineSlamArchive=result.stitched.archive;")
              != std::string::npos,
          "a successful offline stitch should become saveable", failures);
    check(compact.find("archive.berths=ui->openGLWidget->slamMapBerths();")
              != std::string::npos,
          "saving a map should include the current persistent berth layer",
          failures);
    check(compact.find("record.diagnostic.message") != std::string::npos
              && compact.find("record.format_minor") != std::string::npos,
          "map loading should log format and geographic correction diagnostics",
          failures);
    check(compact.find("displayStats.duplicate_points_removed")
              != std::string::npos,
          "map loading should log XYZ voxel deduplication statistics",
          failures);
    check(compact.find(
              "slam_map_display::analyzeStableRegions(archive,1.0,2,5)")
              != std::string::npos,
          "map saving should analyze repeated stable regions after saving",
          failures);
    check(compact.find(std::string("*") + ".slammap") != std::string::npos
              && compact.find(std::string("*") + ".pcd") != std::string::npos,
          "the map file dialog must advertise both new and legacy formats",
          failures);
}

int diagnoseRealMaps(int argc, char **argv)
{
    QStringList paths;
    for (int index = 2; index < argc; ++index)
        paths.append(QString::fromLocal8Bit(argv[index]));
    const slam_map_load::Result loaded =
        slam_map_load::loadAndStitch(paths);
    if (!loaded.success) {
        std::cerr << "REAL_MAP_ERROR "
                  << loaded.error.toLocal8Bit().constData() << '\n';
        return 2;
    }
    std::cout << "REAL_MAP_KEYFRAMES "
              << loaded.stitched.archive.keyframes.size() << '\n';
    for (const slam_map_stitch::Diagnostic &diagnostic
         : loaded.stitched.diagnostics) {
        std::cout << "REAL_MAP_DIAGNOSTIC mode="
                  << static_cast<int>(diagnostic.mode)
                  << " path="
                  << diagnostic.source_path.toLocal8Bit().constData()
                  << " overlap_s=" << diagnostic.temporal_overlap_s
                  << " trimmed=" << diagnostic.trimmed_keyframes
                  << " appended=" << diagnostic.appended_keyframes
                  << " anchor_rmse=" << diagnostic.anchor_robust_rmse_m
                  << " final_rmse=" << diagnostic.final_robust_rmse_m
                  << " source_match=" << diagnostic.source_match_ratio
                  << " target_match=" << diagnostic.target_match_ratio
                  << " improvement=" << diagnostic.quality_improvement_ratio
                  << " translation=" << diagnostic.correction_translation_m
                  << " yaw=" << diagnostic.correction_yaw_deg
                  << " message="
                  << diagnostic.message.toLocal8Bit().constData()
                  << '\n';
    }

    QTemporaryDir directory;
    if (!directory.isValid()) {
        std::cerr << "REAL_MAP_ERROR temporary directory unavailable\n";
        return 3;
    }
    const QString mergedPath =
        directory.filePath(QStringLiteral("real_merged.slammap"));
    QString error;
    if (!slam_map_io::saveArchive(
            mergedPath, loaded.stitched.archive, &error)) {
        std::cerr << "REAL_MAP_ERROR save "
                  << error.toLocal8Bit().constData() << '\n';
        return 4;
    }
    slam_map_io::MapArchive reloaded;
    if (!slam_map_io::loadArchive(mergedPath, reloaded, &error)
        || reloaded.keyframes.size()
               != loaded.stitched.archive.keyframes.size()) {
        std::cerr << "REAL_MAP_ERROR reload "
                  << error.toLocal8Bit().constData() << '\n';
        return 5;
    }
    std::cout << "REAL_MAP_RELOAD_KEYFRAMES "
              << reloaded.keyframes.size() << '\n';
    return 0;
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (argc > 2
        && std::string(argv[1]) == "--diagnose-real-maps") {
        return diagnoseRealMaps(argc, argv);
    }
    int failures = 0;
    testAnchorOffsetAndCoarseMerge(failures);
    testGeoDriftCorrection(failures);
    testStitchingKeepsBerthOpeningEdge(failures);
    testGeoDriftCorrectionRejectsSpike(failures);
    testGeoDriftCorrectionRejectsStaleReference(failures);
    testGeoDriftCorrectionNeedsThreeReferences(failures);
    testBoundedIcpCorrection(failures);
    testLargeAnchorResidualUsesGuardedIcpByDefault(failures);
    testDefaultRegistrationNeverChangesEnuYaw(failures);
    testStableObservationsFilterSingleFrameClutter(failures);
    testIcpFallbacks(failures);
    testNearSearchBoundaryIsRejected(failures);
    testTemporalSeamTrimsRepeatedKeyframes(failures);
    testFullyCoveredMapIsSkipped(failures);
    testPaddingCannotCreateSpatialOverlap(failures);
    testIcpMustImproveAnchorBaseline(failures);
    testDefaultUsesDirectEnuAndOnlyTrimsDuplicateTime(failures);
    testLoadTaskAndSourceColours(failures);
    testLoadTaskBuildsAndReusesReadOnlyTileCache(failures);
    testLoadTaskAppliesGeoCorrection(failures);
    testDisplayVoxelDeduplication(failures);
    testPointCloudMapCountAtHalfMetreResolution(failures);
    testStableRegionAnalysisUsesDistinctKeyframesAndClusters(failures);
    testWidgetArchiveDisplayWiring(failures);
    testMainWindowMapFlowWiring(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM map stitching tests passed\n";
    return 0;
}

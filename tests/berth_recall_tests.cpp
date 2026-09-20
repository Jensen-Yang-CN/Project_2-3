#include "BerthOverlayStyle.h"
#include "BerthPoseBuffer.h"
#include "BerthProtocolGeometry.h"
#include "RealtimeBerthRecall.h"
#include "ThreadSafeQueue.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        ++g_failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool nearlyEqual(double a, double b, double epsilon = 1e-6)
{
    return std::abs(a - b) <= epsilon;
}

void testTimelineResetCanClearPendingDetectionTasks()
{
    usv::ThreadSafeQueue<int> queue(4);
    queue.push(10);
    queue.push(20);
    queue.clear();

    int value = 0;
    check(!queue.try_pop(value),
          "playback timeline reset must remove pending detection tasks");
    queue.push(30);
    check(queue.try_pop(value) && value == 30,
          "queue must remain usable after a playback timeline reset");
}

usv::Berth realtimeBerth()
{
    usv::Berth berth;
    berth.cx = 12.0;
    berth.cy = -4.0;
    berth.w = 8.0;
    berth.l = 16.0;
    berth.angle = 25.0;
    berth.kind = usv::BerthKind::UShape;
    berth.source = usv::BerthDetectionSource::Realtime;
    berth.opening_edge = 2;
    return berth;
}

usv::Berth realtimeLineBerth(double length)
{
    usv::Berth berth = realtimeBerth();
    berth.w = 3.5;
    berth.l = length;
    berth.kind = usv::BerthKind::Line;
    berth.opening_edge = -1;
    return berth;
}

usv::BerthMeasureResult resultWithRealtimeBerth(double timestamp)
{
    usv::BerthMeasureResult result;
    result.timestamp = timestamp;
    result.is_detected = true;
    result.expanded_berths.push_back(realtimeBerth());
    return result;
}

usv::BerthMeasureResult resultWithRealtimeLineBerth(double timestamp,
                                                    double length)
{
    usv::BerthMeasureResult result;
    result.timestamp = timestamp;
    result.is_detected = true;
    result.expanded_berths.push_back(realtimeLineBerth(length));
    return result;
}

usv::GnssInsMessage validGnss(double timestamp)
{
    usv::GnssInsMessage gnss;
    gnss.timestamp = timestamp;
    gnss.latitude = 31.2304;
    gnss.longitude = 121.4737;
    gnss.altitude = 3.0;
    gnss.yaw = 45.0;
    gnss.pitch = 0.0;
    gnss.roll = 0.0;
    return gnss;
}

void testRecallsConfirmedBerthWhenRealtimeMisses()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    for (int i = 0; i < 5; ++i) {
        auto result = resultWithRealtimeBerth(100.0 + i);
        const auto stats = recall.mergeWithPose(result, pose);
        check(stats.recalled_count == 0,
              "确认阶段的实时泊位不应被重复回显");
    }

    usv::BerthMeasureResult too_early;
    too_early.timestamp = 105.0;
    recall.mergeWithPose(too_early, pose);
    check(too_early.expanded_berths.empty(),
          "U 型泊位只有 5 帧时不得提前进入世界稳定库");

    for (int i = 5; i < 8; ++i) {
        auto result = resultWithRealtimeBerth(100.0 + i);
        recall.mergeWithPose(result, pose);
    }

    usv::BerthMeasureResult missed;
    missed.timestamp = 109.0;
    const auto stats = recall.mergeWithPose(missed, pose);

    check(stats.recalled_count == 1, "累计稳定 8 帧后漏检应回显一个泊位");
    check(missed.is_detected, "存在回显泊位时结果应标记为已检测");
    check(missed.is_using_memory, "存在回显泊位时应标记使用历史");
    check(missed.expanded_berths.size() == 1,
          "漏检帧应只包含一个去重后的回显泊位");
    if (!missed.expanded_berths.empty()) {
        const usv::Berth &berth = missed.expanded_berths.front();
        check(berth.source == usv::BerthDetectionSource::CoordinateLibrary,
              "回显泊位来源必须是坐标库");
        check(nearlyEqual(berth.cx, 12.0) && nearlyEqual(berth.cy, -4.0),
              "静止位姿下回显泊位坐标必须保持不变");
    }
}

void testRealtimeResultWinsOverCoordinateLibrary()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 8; ++i) {
        auto warmup = resultWithRealtimeBerth(200.0 + i);
        recall.mergeWithPose(warmup, pose);
    }

    auto current = resultWithRealtimeBerth(206.0);
    const auto stats = recall.mergeWithPose(current, pose);

    check(stats.recalled_count == 0,
          "实时结果覆盖同一坐标库泊位时不应追加回显");
    check(current.expanded_berths.size() == 1,
          "实时结果与坐标库结果重复时只能保留一个");
    check(current.expanded_berths.front().source ==
              usv::BerthDetectionSource::Realtime,
          "去重后必须保留实时来源");
    check(!current.is_using_memory,
          "只有实时结果时不应标记使用历史");
}

void testRecallMovesWithCurrentLidarPose()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 8; ++i) {
        auto warmup = resultWithRealtimeBerth(220.0 + i);
        recall.mergeWithPose(warmup, initial_pose);
    }

    Eigen::Isometry3d moved_pose = Eigen::Isometry3d::Identity();
    moved_pose.translation().x() = 5.0;
    usv::BerthMeasureResult missed;
    missed.timestamp = 226.0;
    recall.mergeWithPose(missed, moved_pose);

    check(missed.expanded_berths.size() == 1,
          "船体移动后的漏检帧仍应回显已确认泊位");
    if (!missed.expanded_berths.empty()) {
        check(nearlyEqual(missed.expanded_berths.front().cx, 7.0),
              "船体向世界 X 正向移动 5m 后，回显泊位应在雷达系后移 5m");
    }
}

void testDisabledModeDoesNotRecallStoredBerth()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 8; ++i) {
        auto warmup = resultWithRealtimeBerth(250.0 + i);
        recall.mergeWithPose(warmup, pose, true, true);
    }

    usv::BerthMeasureResult disabled;
    disabled.timestamp = 256.0;
    const auto stats = recall.mergeWithPose(disabled, pose, false, true);

    check(stats.recalled_count == 0,
          "关闭 U 型模式后不得回显坐标库中的 U 型泊位");
    check(disabled.expanded_berths.empty(),
          "关闭对应检测模式时结果中不得残留该类型泊位");
}

void testRejectsStaleOrInvalidGnss()
{
    const usv::GnssInsMessage gnss = validGnss(300.0);
    check(usv::RealtimeBerthRecall::isGnssUsable(gnss, 301.0, 2.0),
          "时间差在阈值内的有效 GNSS/INS 应可用");
    check(!usv::RealtimeBerthRecall::isGnssUsable(gnss, 303.0, 2.0),
          "超过时间阈值的 GNSS/INS 必须拒绝");

    usv::GnssInsMessage zero = gnss;
    zero.latitude = 0.0;
    zero.longitude = 0.0;
    check(!usv::RealtimeBerthRecall::isGnssUsable(zero, 300.0, 2.0),
          "经纬度同时为零的 GNSS/INS 必须拒绝");

    usv::GnssInsMessage invalid = gnss;
    invalid.latitude = 100.0;
    check(!usv::RealtimeBerthRecall::isGnssUsable(invalid, 300.0, 2.0),
          "越界经纬度必须拒绝");

    usv::GnssInsMessage invalid_attitude = gnss;
    invalid_attitude.yaw_valid = false;
    check(!usv::RealtimeBerthRecall::isGnssUsable(
              invalid_attitude, 300.0, 2.0),
          "航向质量无效时不得写世界库或回显");
}

void testPoseBufferMatchesLidarTimestampAndResetsOnRollback()
{
    usv::BerthPoseBuffer buffer(8);

    Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
    pose_a.translation().x() = 1.0;
    Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
    pose_b.translation().x() = 2.0;

    check(buffer.push(10.0, pose_a) ==
              usv::BerthPoseBuffer::PushResult::Accepted,
          "首个有效位姿必须写入时间缓存");
    check(buffer.push(10.2, pose_b) ==
              usv::BerthPoseBuffer::PushResult::Accepted,
          "递增时间戳位姿必须写入时间缓存");

    Eigen::Isometry3d matched = Eigen::Isometry3d::Identity();
    check(buffer.lookupNearest(10.16, 0.10, matched),
          "点云必须能匹配时间邻近的世界位姿");
    check(nearlyEqual(matched.translation().x(), 2.0),
          "点云必须选择最近时间戳位姿，而不是总线最新状态");
    check(!buffer.lookupNearest(10.50, 0.10, matched),
          "超过时间阈值的位姿不得用于世界回显");

    Eigen::Isometry3d reset_pose = Eigen::Isometry3d::Identity();
    reset_pose.translation().x() = 9.0;
    check(buffer.push(9.0, reset_pose) ==
              usv::BerthPoseBuffer::PushResult::TimelineReset,
          "导航时间回退必须触发坐标时间轴重置");
    check(buffer.size() == 1,
          "时间回退后必须清除旧坐标系的全部缓存位姿");
    check(!buffer.lookupNearest(10.2, 0.10, matched),
          "时间回退后不得再命中旧时间轴位姿");
}

void testUsesRequestedRealtimeAndRecallColors()
{
    const BerthOverlayColor realtime =
        berthOverlayColor(usv::BerthDetectionSource::Realtime);
    const BerthOverlayColor recalled =
        berthOverlayColor(usv::BerthDetectionSource::CoordinateLibrary);

    check(realtime.g > realtime.r && realtime.r > realtime.b,
          "实时泊位框必须是黄绿色");
    check(recalled.b > recalled.g && recalled.g > recalled.r,
          "盲区回显框必须是蓝色");
    check(nearlyEqual(realtime.r, 0.68) &&
              nearlyEqual(realtime.g, 1.00) &&
              nearlyEqual(realtime.b, 0.12),
          "实时泊位框必须使用约定的黄绿色 RGB");
    check(nearlyEqual(recalled.r, 0.10) &&
              nearlyEqual(recalled.g, 0.45) &&
              nearlyEqual(recalled.b, 1.00),
          "盲区回显框必须使用约定的蓝色 RGB");
}

void testEmptyResultDoesNotKeepOldOverlayVisible()
{
    const usv::BerthMeasureResult empty;
    check(!berthResultHasOverlay(empty),
          "空泊位结果必须清除上一帧框，不能形成静态假回显");

    usv::BerthMeasureResult recalled;
    usv::Berth berth = realtimeBerth();
    berth.source = usv::BerthDetectionSource::CoordinateLibrary;
    recalled.expanded_berths.push_back(berth);
    check(berthResultHasOverlay(recalled),
          "坐标库回显泊位必须被视为可见 overlay");

    usv::BerthMeasureResult bus_only;
    bus_only.has_anchor_bus = true;
    check(!berthResultHasOverlay(bus_only),
          "锚点总线只用于内部校正，不能作为棕黄色实体线覆盖泊位开口");
}

void testUShapeKeepsItsEntranceOpen()
{
    usv::Berth u_shape = realtimeBerth();
    check(berthOutlineMode(u_shape) == BerthOutlineMode::OpenEntrance,
          "U 型泊位必须保留入口，不能把 opening_segment 画成实体边");

    usv::Berth line_shape = u_shape;
    line_shape.kind = usv::BerthKind::Line;
    check(berthOutlineMode(line_shape) == BerthOutlineMode::Closed,
          "一字型泊位仍按完整轮廓显示");
}

void testProtocolP1P4RemainTheOpeningSide()
{
    const usv::Berth berth = realtimeBerth();
    const auto geometric =
        usv::berth_protocol::cornersCounterClockwise(berth);
    const auto ordered = usv::berth_protocol::orderedCorners(berth);
    const int opening = berth.opening_edge;

    check((ordered[0] - geometric[(opening + 1) % 4]).norm()
              < 1e-9
          && (ordered[3] - geometric[opening]).norm() < 1e-9,
          "协议 P1/P4 必须始终构成 U 型实际开口边");
    check(usv::berth_protocol::type(berth) == 2,
          "U 型泊位协议类型必须保持为 2");
}

void testRejectedUShapeJumpFallsBackToStableLibrary()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 8; ++i) {
        auto warmup = resultWithRealtimeBerth(330.0 + i);
        recall.mergeWithPose(warmup, pose);
    }

    auto jumped = resultWithRealtimeBerth(339.0);
    jumped.expanded_berths.front().cx += 4.0;
    const auto stats = recall.mergeWithPose(jumped, pose);

    check(jumped.expanded_berths.size() == 1,
          "跳变 U 型候选被拒绝后必须回显旧稳定泊位");
    if (!jumped.expanded_berths.empty()) {
        check(jumped.expanded_berths.front().source ==
                  usv::BerthDetectionSource::CoordinateLibrary,
              "被拒绝的跳变候选不得伪装成实时结果");
        check(nearlyEqual(jumped.expanded_berths.front().cx, 12.0),
              "跳变候选不得覆盖世界坐标稳定库");
    }
    check(stats.recalled_count == 1 && jumped.is_using_memory,
          "跳变回退必须计为蓝色历史回显");
}

void testCurrentFrameIntersectionKeepsHigherConfidenceUShape()
{
    usv::BerthMeasureResult frame;
    usv::Berth recovered = realtimeBerth();
    recovered.is_anchor_recovered = true;
    recovered.cx = 12.0;
    recovered.edge_point_counts = {80, 80, 80, 0};

    usv::Berth fresh = realtimeBerth();
    fresh.is_anchor_recovered = false;
    fresh.cx = 15.5;
    fresh.edge_point_counts = {20, 20, 20, 0};

    frame.expanded_berths = {recovered, fresh};
    usv::suppressIntersectingUShapeOutputBerths(frame);

    check(frame.expanded_berths.size() == 1,
          "当前帧相交的两个 U 型候选只能输出高置信候选");
    if (!frame.expanded_berths.empty()) {
        check(frame.expanded_berths.front().is_anchor_recovered,
              "实时完整检测必须优先于相交的锚点恢复候选");
        check(nearlyEqual(frame.expanded_berths.front().cx, 12.0),
              "相交仲裁不能误保留低优先级恢复框");
    }
}

void testContainedUShapesAreNotTreatedAsCrossingEdges()
{
    usv::BerthMeasureResult frame;
    usv::Berth outer = realtimeBerth();
    outer.cx = 0.0;
    outer.cy = 0.0;
    outer.w = 20.0;
    outer.l = 30.0;
    outer.angle = 0.0;

    usv::Berth inner = outer;
    inner.w = 4.0;
    inner.l = 8.0;
    inner.edge_point_counts = {100, 100, 100, 0};

    frame.expanded_berths = {outer, inner};
    usv::suppressIntersectingUShapeOutputBerths(frame);
    check(frame.expanded_berths.size() == 2,
          "containment without edge contact must not trigger crossing suppression");
}

void testConfirmedWorldBusIsReturnedInCurrentLidarFrame()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 3; ++i) {
        auto frame = resultWithRealtimeBerth(350.0 + i);
        frame.has_anchor_bus = true;
        frame.anchor_bus_line = usv::DebugLine2D{-20.0, 0.0, 20.0, 0.0};
        recall.mergeWithPose(frame, pose);
        if (i < 2) {
            check(!frame.has_anchor_bus,
                  "未满 3 帧的原始锚点总线不得直接交给 UI");
        } else {
            check(frame.has_anchor_bus,
                  "连续 3 帧后必须回显确认的世界锚点总线");
            check(nearlyEqual(frame.anchor_bus_line.y1, 0.0)
                      && nearlyEqual(frame.anchor_bus_line.y2, 0.0),
                  "确认总线必须变换回当前雷达坐标");
        }
    }
}

void testLineLibraryKeepsFirstConfirmedLength()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 5; ++i) {
        auto warmup = resultWithRealtimeLineBerth(400.0 + i, 15.0);
        recall.mergeWithPose(warmup, pose);
    }

    auto shorter = resultWithRealtimeLineBerth(406.0, 12.0);
    recall.mergeWithPose(shorter, pose);

    usv::BerthMeasureResult missed;
    missed.timestamp = 407.0;
    recall.mergeWithPose(missed, pose);

    check(missed.expanded_berths.size() == 1,
          "一字泊位后续漏检应回显已确认泊位");
    if (!missed.expanded_berths.empty()) {
        check(nearlyEqual(missed.expanded_berths.front().l, 15.0),
              "一字泊位首次正式入库长度不得被后续较短检测覆盖");
    }
}

void testLineAtTwoThirdsKeepsRealtimeResult()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 5; ++i) {
        auto warmup = resultWithRealtimeLineBerth(420.0 + i, 15.0);
        recall.mergeWithPose(warmup, pose);
    }

    auto current = resultWithRealtimeLineBerth(426.0, 10.0);
    const auto stats = recall.mergeWithPose(current, pose);

    check(current.expanded_berths.size() == 1,
          "一字泊位阈值边界只能输出一个结果");
    if (!current.expanded_berths.empty()) {
        check(nearlyEqual(current.expanded_berths.front().l, 10.0),
              "等于参考长度三分之二时应继续输出实时长度");
        check(current.expanded_berths.front().source ==
                  usv::BerthDetectionSource::Realtime,
              "等于阈值时应保留实时来源");
    }
    check(stats.recalled_count == 0 && !current.is_using_memory,
          "阈值边界未使用历史替换时不应标记使用历史");
}

void testShortLineUsesCompleteLibraryGeometry()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 5; ++i) {
        auto warmup = resultWithRealtimeLineBerth(440.0 + i, 15.0);
        recall.mergeWithPose(warmup, pose);
    }

    auto current = resultWithRealtimeLineBerth(446.0, 9.0);
    const auto stats = recall.mergeWithPose(current, pose);

    check(current.expanded_berths.size() == 1,
          "严重缩短的一字泊位只能输出一个历史校正结果");
    if (!current.expanded_berths.empty()) {
        const usv::Berth &berth = current.expanded_berths.front();
        check(nearlyEqual(berth.l, 15.0),
              "短于参考长度三分之二时应恢复完整历史长度");
        check(berth.source == usv::BerthDetectionSource::CoordinateLibrary,
              "历史校正的一字泊位必须标记为坐标库来源");
    }
    check(stats.recalled_count == 1,
          "历史替换应计入坐标库回显数量");
    check(current.is_using_memory,
          "历史替换后结果必须标记使用历史");
}

void testWorldRecoveryRoiProjectsIntoCurrentLidarFrame()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    recall.updateUShapeRecoveryRois(
        {realtimeBerth()}, initial_pose, true);

    Eigen::Isometry3d moved_pose = Eigen::Isometry3d::Identity();
    moved_pose.translation().x() = 5.0;
    const auto moved_context = recall.uShapeDetectionContext(moved_pose);
    check(moved_context.recovery_rois.size() == 1,
          "saved world recovery ROI must be available before detection");
    if (!moved_context.recovery_rois.empty()) {
        check(nearlyEqual(moved_context.recovery_rois.front().cx, 7.0),
              "world recovery ROI must move back 5 m in the current lidar frame");
        check(moved_context.recovery_rois.front().source ==
                  usv::BerthDetectionSource::Realtime,
              "projected recovery ROI must remain a realtime search ROI");
    }

    recall.updateUShapeRecoveryRois({}, moved_pose, true);
    check(recall.uShapeDetectionContext(moved_pose).recovery_rois.size() == 1,
          "empty output must retain world ROI while detector memory is alive");

    recall.updateUShapeRecoveryRois({}, moved_pose, false);
    check(recall.uShapeDetectionContext(moved_pose).recovery_rois.empty(),
          "detector memory reset must clear the world recovery ROI");

    recall.updateUShapeRecoveryRois(
        {realtimeBerth()}, initial_pose, true);
    recall.clearUShapeRecoveryRois();
    check(recall.uShapeDetectionContext(initial_pose).recovery_rois.empty(),
          "an unprojectable local ROI must explicitly clear stale world recovery ROIs");
}

void testDetectionContextIncludesStableLibraryAndConfirmedBus()
{
    usv::RealtimeBerthRecall recall;
    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 8; ++i) {
        auto frame = resultWithRealtimeBerth(500.0 + i);
        recall.mergeWithPose(frame, pose);
    }
    for (int i = 0; i < 3; ++i) {
        auto frame = resultWithRealtimeBerth(508.0 + i);
        frame.has_anchor_bus = true;
        frame.anchor_bus_line =
            usv::DebugLine2D{-20.0, 0.0, 20.0, 0.0};
        recall.mergeWithPose(frame, pose);
    }

    Eigen::Isometry3d moved_pose = Eigen::Isometry3d::Identity();
    moved_pose.translation().x() = 5.0;
    const auto initial_context = recall.uShapeDetectionContext(pose);
    const auto context = recall.uShapeDetectionContext(moved_pose);
    check(context.has_world_anchor_bus,
          "confirmed world bus must be supplied before U-shape detection");
    if (context.has_world_anchor_bus) {
        check(initial_context.has_world_anchor_bus
                  && nearlyEqual(context.world_anchor_bus.x1,
                                 initial_context.world_anchor_bus.x1 - 5.0)
                  && nearlyEqual(context.world_anchor_bus.x2,
                                 initial_context.world_anchor_bus.x2 - 5.0),
              "confirmed world bus must project into the current lidar frame");
    }
    check(context.stable_historical_berths.size() == 1,
          "stable U-shape library must be supplied before detection");
    if (!context.stable_historical_berths.empty()) {
        check(!initial_context.stable_historical_berths.empty()
                  && nearlyEqual(
                      context.stable_historical_berths.front().cx,
                      initial_context.stable_historical_berths.front().cx
                          - 5.0),
              "stable historical berth must project into the current lidar frame");
        check(context.stable_historical_berths.front().source ==
                  usv::BerthDetectionSource::CoordinateLibrary,
              "stable historical context must preserve library provenance");
    }

    recall.reset();
    const auto reset_context = recall.uShapeDetectionContext(moved_pose);
    check(!reset_context.has_world_anchor_bus
              && reset_context.stable_historical_berths.empty()
              && reset_context.recovery_rois.empty(),
          "timeline reset must clear all U-shape world detection context");
}

}  // namespace

int main()
{
    testTimelineResetCanClearPendingDetectionTasks();
    testRecallsConfirmedBerthWhenRealtimeMisses();
    testRealtimeResultWinsOverCoordinateLibrary();
    testRecallMovesWithCurrentLidarPose();
    testDisabledModeDoesNotRecallStoredBerth();
    testRejectsStaleOrInvalidGnss();
    testPoseBufferMatchesLidarTimestampAndResetsOnRollback();
    testUsesRequestedRealtimeAndRecallColors();
    testEmptyResultDoesNotKeepOldOverlayVisible();
    testUShapeKeepsItsEntranceOpen();
    testProtocolP1P4RemainTheOpeningSide();
    testRejectedUShapeJumpFallsBackToStableLibrary();
    testCurrentFrameIntersectionKeepsHigherConfidenceUShape();
    testContainedUShapesAreNotTreatedAsCrossingEdges();
    testConfirmedWorldBusIsReturnedInCurrentLidarFrame();
    testLineLibraryKeepsFirstConfirmedLength();
    testLineAtTwoThirdsKeepsRealtimeResult();
    testShortLineUsesCompleteLibraryGeometry();
    testWorldRecoveryRoiProjectsIntoCurrentLidarFrame();
    testDetectionContextIncludesStableLibraryAndConfirmedBus();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All berth recall tests passed\n";
    return 0;
}

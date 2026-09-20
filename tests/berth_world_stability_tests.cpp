#include "RealtimeBerthStability.h"
#include "RealtimeWorldAnchorBus.h"

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

bool nearlyEqual(double a, double b, double epsilon = 1e-6)
{
    return std::abs(a - b) <= epsilon;
}

usv::Berth uShape(double cx = 0.0, double cy = 0.0)
{
    usv::Berth berth;
    berth.cx = cx;
    berth.cy = cy;
    berth.w = 8.0;
    berth.l = 16.0;
    berth.angle = 0.0;
    berth.kind = usv::BerthKind::UShape;
    berth.source = usv::BerthDetectionSource::Realtime;
    berth.opening_edge = 0;
    berth.edge_point_counts = {10, 20, 30, 40};
    berth.edge_heights = {1.0, 2.0, 3.0, 4.0};
    berth.edge_height_counts = {1, 2, 3, 4};
    berth.edge_height_valid = {true, true, true, true};
    return berth;
}

std::array<Eigen::Vector2d, 4> corners(const usv::Berth &berth)
{
    constexpr double kPi = 3.14159265358979323846;
    const double rad = berth.angle * kPi / 180.0;
    const double ca = std::cos(rad);
    const double sa = std::sin(rad);
    const double hw = berth.w * 0.5;
    const double hl = berth.l * 0.5;
    const double local[4][2] = {
        {-hw, -hl}, {hw, -hl}, {hw, hl}, {-hw, hl}};
    std::array<Eigen::Vector2d, 4> result;
    for (int i = 0; i < 4; ++i) {
        result[i] = Eigen::Vector2d(
            berth.cx + ca * local[i][0] - sa * local[i][1],
            berth.cy + sa * local[i][0] + ca * local[i][1]);
    }
    return result;
}

double edgeMidpointY(const usv::Berth &berth, int edge)
{
    const auto c = corners(berth);
    return 0.5 * (c[edge].y() + c[(edge + 1) % 4].y());
}

void testStableLibraryRequiresEightFrames()
{
    usv::berth_realtime::RealtimeBerthStability stability;
    for (int i = 0; i < 7; ++i) {
        std::vector<usv::Berth> frame{uShape(0.1 * i, 0.0)};
        const auto accepted = stability.update(frame);
        check(accepted.size() == 1 && accepted.front(),
              "稳定窗口形成前的正常实时 U 型观测应继续输出");
        check(stability.stableLibrary().empty(),
              "U 型世界坐标库必须累计满 8 帧后才正式稳定");
    }

    std::vector<usv::Berth> eighth{uShape(0.7, 0.0)};
    stability.update(eighth);
    check(stability.stableLibrary().size() == 1,
          "第 8 个稳定观测必须生成一个世界坐标稳定泊位");
}

void testFiveFrameReplacementAndAxisSwap()
{
    usv::berth_realtime::RealtimeBerthStability stability;
    for (int i = 0; i < 8; ++i) {
        usv::Berth sample = uShape();
        if (i == 4) {
            sample.w = 16.0;
            sample.l = 8.0;
            sample.angle = 90.0;
            sample.opening_edge = 3;
        }
        std::vector<usv::Berth> frame{sample};
        const auto accepted = stability.update(frame);
        check(accepted.front(), "长宽交换加 90° 的同一泊位不应被当成跳变");
    }

    for (int i = 0; i < 4; ++i) {
        std::vector<usv::Berth> replacement{uShape(4.0, 0.0)};
        const auto accepted = stability.update(replacement);
        check(!accepted.front(), "不足 5 帧的新几何簇必须拒绝输出和入库");
    }
    std::vector<usv::Berth> fifth{uShape(4.0, 0.0)};
    const auto accepted = stability.update(fifth);
    check(accepted.front(), "连续 5 帧稳定的新几何簇必须替换旧结果");
    const auto library = stability.stableLibrary();
    check(library.size() == 1 && nearlyEqual(library.front().cx, 4.0),
          "替换后稳定库必须使用新五帧簇的几何均值");
}

void testAxisSwapAcross180RemapsEveryEdgeField()
{
    usv::berth_realtime::RealtimeBerthStability stability;
    usv::Berth reference = uShape();
    reference.angle = 10.0;
    std::vector<usv::Berth> first{reference};
    stability.update(first);

    usv::Berth swapped = uShape();
    swapped.w = 16.0;
    swapped.l = 8.0;
    swapped.angle = 100.0;
    swapped.opening_edge = 0;
    swapped.edge_point_counts = {10, 20, 30, 40};
    swapped.ship_edge_distances = {1.0, 2.0, 3.0, 4.0};
    swapped.ship_edge_distance_valid = {true, false, true, false};
    std::vector<usv::Berth> second{swapped};
    const auto accepted = stability.update(second);

    check(accepted.size() == 1 && accepted.front(),
          "equivalent rectangle across 180 degrees must remain accepted");
    check(second.front().opening_edge == 1,
          "axis swap crossing 180 degrees must include opposite-edge remap");
    check(second.front().edge_point_counts
              == std::array<int, 4>{40, 10, 20, 30},
          "edge point counts must follow the same axis remap");
    check(second.front().ship_edge_distances
              == std::array<double, 4>{4.0, 1.0, 2.0, 3.0},
          "ship edge distances must follow the same axis remap");
    check(second.front().ship_edge_distance_valid
              == std::array<bool, 4>{false, true, false, true},
          "ship edge distance validity must follow the same axis remap");
}

void testWorldBusConfirmsLocksAndKeepsFirstReference()
{
    usv::berth_realtime::RealtimeWorldAnchorBus bus;
    const usv::DebugLine2D first{-10.0, 0.0, 10.0, 0.0};
    bus.update(first);
    bus.update(usv::DebugLine2D{-10.0, 0.2, 10.0, 0.2});
    check(!bus.valid(), "世界锚点总线不足 3 帧不能确认");
    bus.update(usv::DebugLine2D{-10.0, 0.4, 10.0, 0.4});
    check(bus.valid(), "世界锚点总线连续 3 帧必须确认");
    check(nearlyEqual(bus.line().y1, first.y1),
          "确认窗口必须保持第一帧参考，不能逐帧向外漂移");

    bus.update(first);
    bus.update(first);
    bus.update(first);
    check(bus.locked(), "确认后累计到 6 帧必须锁定世界锚点总线");
}

void testBusAlignmentRefreshesOpeningAndVisibleEdges()
{
    usv::berth_realtime::RealtimeWorldAnchorBus bus;
    const usv::DebugLine2D line{-30.0, 0.0, 30.0, 0.0};
    bus.update(line);
    bus.update(line);
    bus.update(line);

    usv::Berth berth = uShape(0.0, 8.5);
    berth.angle = 17.0;
    berth.visible_edges = {
        usv::DebugLine2D{100.0, 100.0, 101.0, 101.0}};
    std::vector<usv::Berth> library{berth};
    bus.alignLibrary(library);

    check(library.size() == 1, "总线贴线不得删除稳定泊位");
    if (library.empty())
        return;
    const usv::Berth &aligned = library.front();
    check(aligned.opening_edge >= 0 && aligned.opening_edge < 4,
          "总线贴线后必须给出有效开口边");
    check(std::abs(edgeMidpointY(aligned, aligned.opening_edge)) < 1e-6,
          "贴线后的开口边中点必须落在世界锚点总线上");
    check(aligned.has_opening_segment,
          "贴线后必须重建 opening_segment");
    check(aligned.visible_edges.size() == 3,
          "贴线后必须重建三条实体 visible_edges");
    bool contains_stale_edge = false;
    for (const auto &edge : aligned.visible_edges) {
        contains_stale_edge =
            contains_stale_edge || (edge.x1 > 90.0 || edge.y1 > 90.0);
    }
    check(!contains_stale_edge,
          "总线贴线后不得保留修正前的旧 visible_edges");
}

void testOppositeSideIsMirroredAndEdgeMetadataRemapped()
{
    usv::berth_realtime::RealtimeWorldAnchorBus bus;
    const usv::DebugLine2D line{-30.0, 0.0, 30.0, 0.0};
    bus.update(line);
    bus.update(line);
    bus.update(line);

    bus.updateBerthRowSide(std::vector<usv::Berth>{uShape(0.0, 9.0)});
    check(bus.berthRowSide() == 1,
          "总线上方的实时候选必须确定泊位排正侧");

    usv::Berth wrong_side = uShape(0.0, -9.0);
    wrong_side.opening_edge = 2;
    std::vector<usv::Berth> library{wrong_side};
    bus.alignLibrary(library);
    check(!library.empty() && library.front().cy > 0.0,
          "总线错误侧的历史泊位必须镜像到泊位排所在侧");
    if (!library.empty()) {
        check(library.front().edge_point_counts[2] == 10,
              "镜像后边点数必须映射到物理对边");
        check(library.front().visible_edges.size() == 3,
              "镜像后仍必须保持 U 型三条实体边");
    }
}

void testCorrectionUpdatesStabilityInternals()
{
    usv::berth_realtime::RealtimeBerthStability stability;
    for (int i = 0; i < 8; ++i) {
        std::vector<usv::Berth> frame{uShape(0.0, 8.5)};
        stability.update(frame);
    }

    usv::berth_realtime::RealtimeWorldAnchorBus bus;
    const usv::DebugLine2D line{-30.0, 0.0, 30.0, 0.0};
    bus.update(line);
    bus.update(line);
    bus.update(line);
    stability.correctAll(
        [&bus](usv::Berth &berth) { bus.alignBerth(berth); });

    const auto corrected = stability.stableLibrary();
    check(corrected.size() == 1,
          "校正稳定器内部状态后稳定库数量必须保持不变");
    if (corrected.empty())
        return;
    const int corrected_wall_edge = corrected.front().opening_edge;
    check(std::abs(edgeMidpointY(
              corrected.front(), corrected_wall_edge)) < 1e-6,
          "世界总线校正必须写回稳定器内部 last/window/replacement");
}

void testResultCarriesRealtimeAnchorBusAndRecoverySource()
{
    usv::Berth berth = uShape();
    berth.is_anchor_recovered = true;
    check(berth.is_anchor_recovered,
          "泊位结果必须保留锚点恢复来源供节点内部仲裁");

    usv::BerthMeasureResult result;
    result.has_anchor_bus = true;
    result.anchor_bus_line = usv::DebugLine2D{-1.0, 0.0, 1.0, 0.0};
    check(result.has_anchor_bus
              && nearlyEqual(result.anchor_bus_line.x2, 1.0),
          "检测结果必须携带当前雷达坐标系锚点总线");
}

}  // namespace

int main()
{
    testStableLibraryRequiresEightFrames();
    testFiveFrameReplacementAndAxisSwap();
    testAxisSwapAcross180RemapsEveryEdgeField();
    testWorldBusConfirmsLocksAndKeepsFirstReference();
    testBusAlignmentRefreshesOpeningAndVisibleEdges();
    testOppositeSideIsMirroredAndEdgeMetadataRemapped();
    testCorrectionUpdatesStabilityInternals();
    testResultCarriesRealtimeAnchorBusAndRecoverySource();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All berth world stability tests passed\n";
    return 0;
}

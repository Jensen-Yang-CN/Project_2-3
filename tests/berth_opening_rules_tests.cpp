#include "BerthOpeningRules.h"

#include <Eigen/Dense>

#include <array>
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

usv::Berth sampleBerth()
{
    usv::Berth berth;
    berth.w = 8.0;
    berth.l = 16.0;
    berth.angle = 0.0;
    berth.kind = usv::BerthKind::UShape;
    berth.edge_point_counts = {40, 30, 20, 10};
    return berth;
}

void testRayRegionStartsAtEdgeAndIgnoresHeight()
{
    const usv::Berth berth = sampleBerth();
    const std::vector<Eigen::Vector3d> points{
        Eigen::Vector3d(0.0, -9.0, 100.0),
        Eigen::Vector3d(0.9, -27.9, -100.0),
        Eigen::Vector3d(0.0, -1.0, 0.0),
        Eigen::Vector3d(1.1, -10.0, 0.0),
    };

    const auto hits =
        usv::berth_opening::countOpeningRayHits(berth, points, 20.0, 1.0);
    check(hits[0] == 2,
          "20x2 BEV ray must start at edge midpoint and ignore point height");
    check(hits[1] == 0 && hits[2] == 0 && hits[3] == 0,
          "points outside the edge ray must not affect other directions");
}

void testNormalOpeningUsesMissingRayThenEdgeCountFallback()
{
    const std::array<int, 4> edge_counts{40, 30, 20, 10};
    check(usv::berth_opening::selectOpeningEdge(
              {2, 0, 3, 4}, edge_counts, false, -1, 3, 1) == 1,
          "normal berth must choose the missing ray direction");
    check(usv::berth_opening::selectOpeningEdge(
              {2, 2, 3, 4}, edge_counts, false, -1, 3, 1) == 3,
          "when every ray hits, normal berth must use the sparsest edge");
}

void testRecoveredOpeningChangesOnlyForUniqueMissingRay()
{
    const std::array<int, 4> edge_counts{40, 30, 20, 10};
    check(usv::berth_opening::selectOpeningEdge(
              {2, 0, 3, 4}, edge_counts, true, 3, 2, 1) == 1,
          "recovered berth must follow one unambiguous missing ray");
    check(usv::berth_opening::selectOpeningEdge(
              {0, 0, 3, 4}, edge_counts, true, 3, 2, 1) == 3,
          "recovered berth must keep anchor edge when multiple rays are missing");
    check(usv::berth_opening::selectOpeningEdge(
              {2, 2, 3, 4}, edge_counts, true, 3, 2, 1) == 3,
          "recovered berth must keep anchor edge when every ray hits");
    check(usv::berth_opening::selectOpeningEdge(
              {0, 0, 3, 4}, edge_counts, true, -1, 2, 1) == 2,
          "invalid recovered anchor edge must use geometry fallback");
}

}  // namespace

int main()
{
    testRayRegionStartsAtEdgeAndIgnoresHeight();
    testNormalOpeningUsesMissingRayThenEdgeCountFallback();
    testRecoveredOpeningChangesOnlyForUniqueMissingRay();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All berth opening rule tests passed\n";
    return 0;
}

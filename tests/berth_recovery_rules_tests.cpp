#include "BerthRecoveryRules.h"
#include "BerthDetection.h"

#include <Eigen/Dense>

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

void testAnchorPairDistanceUsesNineMeterThreshold()
{
    check(!usv::berth_recovery::validAnchorPairDistance(8.99),
          "anchor pair shorter than 9 m must be rejected");
    check(!usv::berth_recovery::validAnchorPairDistance(9.0),
          "anchor pair exactly 9 m must be rejected");
    check(usv::berth_recovery::validAnchorPairDistance(9.01),
          "anchor pair longer than 9 m must be accepted");
}

void testExpansionStopsAtTenMetersPerDimension()
{
    check(!usv::berth_recovery::canExpandDimension(10.0, 0.2),
          "a 10 m dimension must not expand further");
    check(usv::berth_recovery::canExpandDimension(9.8, 0.2),
          "a dimension may expand exactly to 10 m");
    check(!usv::berth_recovery::canExpandDimension(9.9, 0.2),
          "an expansion step must not cross 10 m");
}

void testDefaultExpansionFallbackDoesNotPolluteSizeHistory()
{
    usv::Berth original;
    original.cx = 12.0;
    original.cy = -7.0;
    original.w = 2.0;
    original.l = 4.0;
    original.angle = 37.0;
    original.opening_edge = 2;

    const usv::Berth fallback =
        usv::berth_recovery::makeDefaultExpansionFallback(original);
    check(std::abs(fallback.w - 10.0) < 1e-9
              && std::abs(fallback.l - 10.0) < 1e-9,
          "a berth with no valid expansion must receive a 10 x 10 m fallback");
    check(std::abs(fallback.cx - original.cx) < 1e-9
              && std::abs(fallback.cy - original.cy) < 1e-9
              && std::abs(fallback.angle - original.angle) < 1e-9
              && fallback.opening_edge == original.opening_edge,
          "fallback size must preserve the detected berth position and orientation");
    check(fallback.used_default_expansion_size,
          "fallback berth must be tagged as using a default expansion size");
    check(!usv::berth_recovery::hasReliableExpansionSize(fallback),
          "fallback size must not be stored as historical smoothing data");
    check(usv::berth_recovery::hasReliableExpansionSize(original),
          "normal detected size must remain eligible for historical smoothing");
    check(!usv::berth_recovery::canStabilizeSizeFromTrackedBerth(
              fallback, original),
          "fallback candidate must not occupy a size-stabilization match");
    check(!usv::berth_recovery::canStabilizeSizeFromTrackedBerth(
              original, fallback),
          "fallback track must not occupy a size-stabilization match");
    check(usv::berth_recovery::canStabilizeSizeFromTrackedBerth(
              original, original),
          "measured candidate and track remain eligible for size stabilization");
}

void testFallbackFramesAdvanceAndExpireSizeHistory()
{
    usv::Berth measured;
    measured.w = 5.0;
    measured.l = 15.0;
    const usv::Berth fallback =
        usv::berth_recovery::makeDefaultExpansionFallback(measured);
    std::deque<std::vector<usv::Berth>> history;

    usv::berth_recovery::appendExpansionSizeHistoryFrame(
        history, {measured}, 2);
    usv::berth_recovery::appendExpansionSizeHistoryFrame(
        history, {fallback}, 2);
    check(history.size() == 2 && history.back().empty(),
          "a fallback-only frame must advance history without storing fallback size");

    usv::berth_recovery::appendExpansionSizeHistoryFrame(
        history, {fallback}, 2);
    check(history.size() == 2 && history.front().empty(),
          "fallback frames must evict stale measured sizes from the bounded history");
}

void testSingleAnchorOverlappingStableBerthIsRejected()
{
    usv::Berth stable;
    stable.cx = 10.0;
    stable.cy = -4.0;
    stable.w = 8.0;
    stable.l = 16.0;
    stable.angle = 90.0;
    stable.kind = usv::BerthKind::UShape;

    check(usv::berth_recovery::anchorOverlapsStableBerth(
              Eigen::Vector2d(10.0, 1.4), stable, 1.5),
          "single anchor inside the stable berth margin must be rejected");
    check(!usv::berth_recovery::anchorOverlapsStableBerth(
              Eigen::Vector2d(10.0, 1.6), stable, 1.5),
          "single anchor beyond the stable berth margin must remain eligible");
}

void testDetectorAcceptsProjectedWorldRecoveryContext()
{
    usv::BerthDetector detector;
    detector.init();

    usv::DebugLine2D bus;
    bus.x1 = -4.0;
    bus.y1 = 2.0;
    bus.x2 = 20.0;
    bus.y2 = 2.0;
    detector.setWorldAnchorBusConstraint(bus);

    usv::Berth stable;
    stable.cx = 8.0;
    stable.cy = 8.0;
    stable.w = 8.0;
    stable.l = 16.0;
    stable.angle = 0.0;
    stable.kind = usv::BerthKind::UShape;
    detector.setStableHistoricalBerths({stable});

    usv::Berth recovery_roi = stable;
    recovery_roi.cy = 2.0;
    detector.setWorldProjectedRecoveryRois({recovery_roi});
    check(detector.hasRecoveryRoiMemory(),
          "projected world ROI must become detector recovery memory");

    detector.clearWorldAnchorBusConstraint();
    detector.clearStableHistoricalBerths();
}

}  // namespace

int main()
{
    testAnchorPairDistanceUsesNineMeterThreshold();
    testExpansionStopsAtTenMetersPerDimension();
    testDefaultExpansionFallbackDoesNotPolluteSizeHistory();
    testFallbackFramesAdvanceAndExpireSizeHistory();
    testSingleAnchorOverlappingStableBerthIsRejected();
    testDetectorAcceptsProjectedWorldRecoveryContext();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All berth recovery rule tests passed\n";
    return 0;
}

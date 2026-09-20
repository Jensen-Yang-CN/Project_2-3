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
    testSingleAnchorOverlappingStableBerthIsRejected();
    testDetectorAcceptsProjectedWorldRecoveryContext();

    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All berth recovery rule tests passed\n";
    return 0;
}

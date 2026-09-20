#include "DetectionNodeScheduler.h"

#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void observeRange(usv::scheduling::DetectionNodeScheduler &scheduler,
                  double firstTimestamp, int count, bool stable)
{
    for (int index = 0; index < count; ++index)
        scheduler.observe(firstTimestamp + index * 0.1, stable);
}

} // namespace

int main()
{
    using usv::scheduling::BridgeScheduleState;
    using usv::scheduling::DetectionNodeScheduler;

    int failures = 0;
    DetectionNodeScheduler scheduler({10, 10});

    check(scheduler.state() == BridgeScheduleState::ForcedOff,
          "bridge must be forced off before manual permission", failures);
    scheduler.setBridgePermission(true);
    check(scheduler.bridgeShouldRun(),
          "granting permission must start bridge in safe initial mode", failures);

    observeRange(scheduler, 1.0, 9, true);
    check(scheduler.bridgeShouldRun() && scheduler.stableCount() == 9,
          "nine stable timestamps must not suspend bridge", failures);
    const auto tenthStable = scheduler.observe(1.9, true);
    check(tenthStable.state_changed && !tenthStable.bridge_should_run
              && scheduler.state() == BridgeScheduleState::AutomaticStandby,
          "tenth stable timestamp must suspend bridge", failures);

    observeRange(scheduler, 2.0, 9, false);
    check(!scheduler.bridgeShouldRun() && scheduler.missingCount() == 9,
          "nine missing timestamps must not resume bridge", failures);
    const auto tenthMissing = scheduler.observe(2.9, false);
    check(tenthMissing.state_changed && tenthMissing.bridge_should_run
              && scheduler.state() == BridgeScheduleState::AutomaticRunning,
          "tenth missing timestamp must resume bridge", failures);

    scheduler.resetObservationWindow();
    for (int index = 0; index < 20; ++index)
        scheduler.observe(4.0 + index * 0.1, (index % 2) == 0);
    check(scheduler.bridgeShouldRun()
              && scheduler.stableCount() <= 1
              && scheduler.missingCount() <= 1,
          "alternating observations must not trigger standby", failures);

    scheduler.resetObservationWindow();
    const auto first = scheduler.observe(7.0, true);
    const auto duplicate = scheduler.observe(7.0, true);
    check(!first.state_changed && !duplicate.observation_counted
              && scheduler.stableCount() == 1,
          "duplicate timestamp must not be counted twice", failures);

    observeRange(scheduler, 7.1, 9, true);
    check(!scheduler.bridgeShouldRun(),
          "scheduler should be in standby before rollback test", failures);
    const auto rollback = scheduler.observe(3.0, false);
    check(rollback.timeline_reset && rollback.state_changed
              && rollback.bridge_should_run
              && scheduler.missingCount() == 1,
          "timestamp rollback must reset the window and safely resume bridge",
          failures);

    scheduler.setBridgePermission(false);
    observeRange(scheduler, 10.0, 20, false);
    check(!scheduler.bridgeShouldRun()
              && scheduler.state() == BridgeScheduleState::ForcedOff,
          "automatic observations must never override manual forced-off",
          failures);
    scheduler.setBridgePermission(true);
    check(scheduler.bridgeShouldRun() && scheduler.stableCount() == 0
              && scheduler.missingCount() == 0,
          "re-enabling permission must start with a clean running state",
          failures);

    DetectionNodeScheduler clamped({0, -2});
    clamped.setBridgePermission(true);
    const auto immediate = clamped.observe(20.0, true);
    check(immediate.state_changed && !immediate.bridge_should_run,
          "invalid thresholds must clamp to one timestamp", failures);

    if (failures != 0)
        return 1;
    std::cout << "All detection node scheduler tests passed\n";
    return 0;
}


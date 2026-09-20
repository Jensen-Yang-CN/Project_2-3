#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace usv::scheduling {

enum class BridgeScheduleState {
    ForcedOff,
    AutomaticRunning,
    AutomaticStandby
};

struct DetectionNodeSchedulerConfig {
    int stable_timestamps_to_standby = 10;
    int missing_timestamps_to_resume = 10;
};

struct DetectionNodeScheduleUpdate {
    bool observation_counted = false;
    bool state_changed = false;
    bool timeline_reset = false;
    bool bridge_should_run = false;
    int stable_count = 0;
    int missing_count = 0;
};

/**
 * Pure state machine for the berth-driven bridge compute policy.
 *
 * The manual permission is a hard upper bound: observations may put an
 * allowed bridge into automatic standby or bring it back, but may never
 * override a user-forced shutdown.
 */
class DetectionNodeScheduler {
public:
    explicit DetectionNodeScheduler(
        DetectionNodeSchedulerConfig config = {})
        : config_(sanitize(config))
    {
    }

    DetectionNodeScheduleUpdate setBridgePermission(bool allowed)
    {
        const BridgeScheduleState before = state_;
        bridge_permission_ = allowed;
        resetCounters();
        last_timestamp_.reset();
        state_ = allowed ? BridgeScheduleState::AutomaticRunning
                         : BridgeScheduleState::ForcedOff;
        return snapshot(state_ != before);
    }

    void resetObservationWindow()
    {
        resetCounters();
        last_timestamp_.reset();
        state_ = bridge_permission_
            ? BridgeScheduleState::AutomaticRunning
            : BridgeScheduleState::ForcedOff;
    }

    DetectionNodeScheduleUpdate observe(double timestamp,
                                         bool stableRealtimeBerth)
    {
        DetectionNodeScheduleUpdate update = snapshot(false);
        if (!bridge_permission_ || !std::isfinite(timestamp))
            return update;

        bool resetByRollback = false;
        if (last_timestamp_) {
            constexpr double kTimestampEpsilonSec = 1e-6;
            if (timestamp < *last_timestamp_ - kTimestampEpsilonSec) {
                const BridgeScheduleState before = state_;
                resetObservationWindow();
                resetByRollback = true;
                update.timeline_reset = true;
                update.state_changed = state_ != before;
            } else if (std::abs(timestamp - *last_timestamp_)
                       <= kTimestampEpsilonSec) {
                return update;
            }
        }
        last_timestamp_ = timestamp;

        if (stableRealtimeBerth) {
            missing_count_ = 0;
            stable_count_ = (std::min)(
                stable_count_ + 1,
                config_.stable_timestamps_to_standby);
            if (stable_count_ >= config_.stable_timestamps_to_standby
                && state_ == BridgeScheduleState::AutomaticRunning) {
                state_ = BridgeScheduleState::AutomaticStandby;
                update.state_changed = true;
            }
        } else {
            stable_count_ = 0;
            missing_count_ = (std::min)(
                missing_count_ + 1,
                config_.missing_timestamps_to_resume);
            if (missing_count_ >= config_.missing_timestamps_to_resume
                && state_ == BridgeScheduleState::AutomaticStandby) {
                state_ = BridgeScheduleState::AutomaticRunning;
                update.state_changed = true;
            }
        }

        update.observation_counted = true;
        update.timeline_reset = resetByRollback;
        update.bridge_should_run = bridgeShouldRun();
        update.stable_count = stable_count_;
        update.missing_count = missing_count_;
        return update;
    }

    BridgeScheduleState state() const { return state_; }
    bool bridgePermission() const { return bridge_permission_; }
    bool bridgeShouldRun() const
    {
        return state_ == BridgeScheduleState::AutomaticRunning;
    }
    int stableCount() const { return stable_count_; }
    int missingCount() const { return missing_count_; }
    const DetectionNodeSchedulerConfig &config() const { return config_; }

private:
    static DetectionNodeSchedulerConfig sanitize(
        DetectionNodeSchedulerConfig config)
    {
        config.stable_timestamps_to_standby = (std::clamp)(
            config.stable_timestamps_to_standby, 1, 10000);
        config.missing_timestamps_to_resume = (std::clamp)(
            config.missing_timestamps_to_resume, 1, 10000);
        return config;
    }

    void resetCounters()
    {
        stable_count_ = 0;
        missing_count_ = 0;
    }

    DetectionNodeScheduleUpdate snapshot(bool changed) const
    {
        DetectionNodeScheduleUpdate update;
        update.state_changed = changed;
        update.bridge_should_run = bridgeShouldRun();
        update.stable_count = stable_count_;
        update.missing_count = missing_count_;
        return update;
    }

    DetectionNodeSchedulerConfig config_;
    bool bridge_permission_ = false;
    BridgeScheduleState state_ = BridgeScheduleState::ForcedOff;
    int stable_count_ = 0;
    int missing_count_ = 0;
    std::optional<double> last_timestamp_;
};

} // namespace usv::scheduling


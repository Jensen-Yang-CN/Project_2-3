#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace usv {

/**
 * Small timestamped world-pose history used by berth replay.
 *
 * A navigation timestamp rollback means the producer started a new time/world
 * epoch. In that case old poses are discarded before accepting the new sample.
 */
class BerthPoseBuffer {
public:
    enum class PushResult {
        Accepted,
        Rejected,
        TimelineReset,
    };

    explicit BerthPoseBuffer(size_t capacity = 128)
        : capacity_(std::max<size_t>(1, capacity))
    {
    }

    PushResult push(double timestamp,
                    const Eigen::Isometry3d &world_from_lidar)
    {
        if (!std::isfinite(timestamp) || timestamp <= 0.0
            || !world_from_lidar.matrix().allFinite()) {
            return PushResult::Rejected;
        }

        PushResult result = PushResult::Accepted;
        if (!samples_.empty()
            && timestamp + kTimestampEpsilon
                   < samples_.back().timestamp) {
            samples_.clear();
            result = PushResult::TimelineReset;
        }

        if (!samples_.empty()
            && std::abs(timestamp - samples_.back().timestamp)
                   <= kTimestampEpsilon) {
            samples_.back() = Sample{timestamp, world_from_lidar};
            return result;
        }

        samples_.push_back(Sample{timestamp, world_from_lidar});
        while (samples_.size() > capacity_)
            samples_.pop_front();
        return result;
    }

    bool lookupNearest(double timestamp,
                       double max_age_sec,
                       Eigen::Isometry3d &world_from_lidar) const
    {
        if (samples_.empty() || !std::isfinite(timestamp)
            || timestamp <= 0.0) {
            return false;
        }

        const double allowed_age = std::max(0.0, max_age_sec);
        const Sample *nearest = nullptr;
        double nearest_age = std::numeric_limits<double>::infinity();
        for (const Sample &sample : samples_) {
            const double age = std::abs(sample.timestamp - timestamp);
            if (age < nearest_age) {
                nearest = &sample;
                nearest_age = age;
            }
        }
        if (!nearest || nearest_age > allowed_age)
            return false;

        world_from_lidar = nearest->world_from_lidar;
        return true;
    }

    void clear()
    {
        samples_.clear();
    }

    size_t size() const
    {
        return samples_.size();
    }

private:
    struct Sample {
        double timestamp = 0.0;
        Eigen::Isometry3d world_from_lidar =
            Eigen::Isometry3d::Identity();
    };

    static constexpr double kTimestampEpsilon = 1e-6;
    size_t capacity_ = 128;
    std::deque<Sample> samples_;
};

} // namespace usv

#pragma once

#include "BerthGeometryTransform.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>

#include <Eigen/Geometry>

class SlamBerthOverlaySynchronizer {
public:
    struct Config {
        double max_sync_sec = 0.25;
        double max_stale_sec = 8.0;
        std::size_t max_pose_history = 200;
        std::size_t max_result_history = 100;
    };

    SlamBerthOverlaySynchronizer()
        : SlamBerthOverlaySynchronizer(Config{})
    {
    }

    explicit SlamBerthOverlaySynchronizer(const Config &config)
        : config_(config)
    {
    }

    void addSlamPose(double timestamp,
                     const Eigen::Isometry3d &worldFromLidar)
    {
        if (!std::isfinite(timestamp)
            || !worldFromLidar.matrix().allFinite()) {
            return;
        }

        upsertPose(timestamp, worldFromLidar);
        latest_slam_timestamp_ = has_slam_timestamp_
            ? std::max(latest_slam_timestamp_, timestamp)
            : timestamp;
        has_slam_timestamp_ = true;
        trimHistory(poses_, config_.max_pose_history);
        tryProjectLatestMatch();
    }

    void addBerthResult(const usv::BerthMeasureResult &result)
    {
        if (!hasOverlay(result)) {
            clearResults();
            return;
        }
        if (!std::isfinite(result.timestamp))
            return;

        upsertResult(result);
        trimHistory(results_, config_.max_result_history);
        tryProjectLatestMatch();
    }

    const std::optional<usv::BerthMeasureResult> &worldResult() const
    {
        return world_result_;
    }

    bool visibleAt(double slamTimestamp) const
    {
        if (!world_result_ || !std::isfinite(slamTimestamp))
            return false;
        const double age = slamTimestamp - world_result_->timestamp;
        constexpr double kEpsilon = 1e-9;
        return age >= -config_.max_sync_sec - kEpsilon
            && age <= config_.max_stale_sec + kEpsilon;
    }

    bool visibleAtLatestSlamPose() const
    {
        return has_slam_timestamp_ && visibleAt(latest_slam_timestamp_);
    }

    void clearResults()
    {
        results_.clear();
        world_result_.reset();
        projected_timestamp_ = -std::numeric_limits<double>::infinity();
    }

    void resetAll()
    {
        clearResults();
        poses_.clear();
        latest_slam_timestamp_ = 0.0;
        has_slam_timestamp_ = false;
    }

private:
    struct PoseSample {
        double timestamp = 0.0;
        Eigen::Isometry3d world_from_lidar = Eigen::Isometry3d::Identity();
    };

    static bool hasOverlay(const usv::BerthMeasureResult &result)
    {
        return !result.expanded_berths.empty()
            || !result.highest_points.empty()
            || !result.second_highest_points.empty()
            || result.has_highest_point
            || result.has_second_highest_point;
    }

    template <typename T>
    static void trimHistory(std::deque<T> &history, std::size_t maxSize)
    {
        const std::size_t boundedSize = std::max<std::size_t>(1, maxSize);
        while (history.size() > boundedSize)
            history.pop_front();
    }

    void upsertPose(double timestamp,
                    const Eigen::Isometry3d &worldFromLidar)
    {
        constexpr double kSameTimestampSec = 1e-6;
        for (PoseSample &pose : poses_) {
            if (std::abs(pose.timestamp - timestamp) <= kSameTimestampSec) {
                pose.world_from_lidar = worldFromLidar;
                return;
            }
        }
        poses_.push_back({timestamp, worldFromLidar});
    }

    void upsertResult(const usv::BerthMeasureResult &result)
    {
        constexpr double kSameTimestampSec = 1e-6;
        for (usv::BerthMeasureResult &stored : results_) {
            if (std::abs(stored.timestamp - result.timestamp)
                <= kSameTimestampSec) {
                stored = result;
                return;
            }
        }
        results_.push_back(result);
    }

    const PoseSample *nearestPose(double timestamp, double &bestDt) const
    {
        const PoseSample *best = nullptr;
        bestDt = std::numeric_limits<double>::infinity();
        for (const PoseSample &pose : poses_) {
            const double dt = std::abs(pose.timestamp - timestamp);
            if (dt < bestDt) {
                bestDt = dt;
                best = &pose;
            }
        }
        return best;
    }

    void tryProjectLatestMatch()
    {
        const usv::BerthMeasureResult *bestResult = nullptr;
        const PoseSample *bestPose = nullptr;
        double bestResultTimestamp = -std::numeric_limits<double>::infinity();
        double bestDt = std::numeric_limits<double>::infinity();

        for (const usv::BerthMeasureResult &result : results_) {
            double dt = std::numeric_limits<double>::infinity();
            const PoseSample *pose = nearestPose(result.timestamp, dt);
            if (!pose || dt > config_.max_sync_sec)
                continue;
            if (result.timestamp > bestResultTimestamp
                || (result.timestamp == bestResultTimestamp && dt < bestDt)) {
                bestResult = &result;
                bestPose = pose;
                bestResultTimestamp = result.timestamp;
                bestDt = dt;
            }
        }

        if (!bestResult || !bestPose
            || bestResultTimestamp < projected_timestamp_) {
            return;
        }
        world_result_ = usv::berth_geometry::transformResult(
            *bestResult, bestPose->world_from_lidar);
        projected_timestamp_ = bestResultTimestamp;
    }

    Config config_;
    std::deque<PoseSample> poses_;
    std::deque<usv::BerthMeasureResult> results_;
    std::optional<usv::BerthMeasureResult> world_result_;
    double projected_timestamp_ = -std::numeric_limits<double>::infinity();
    double latest_slam_timestamp_ = 0.0;
    bool has_slam_timestamp_ = false;
};


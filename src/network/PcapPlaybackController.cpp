#include "PcapPlaybackController.h"

#include <algorithm>
#include <chrono>
#include <cmath>

double PcapPlaybackController::normalizeSpeed(double speed)
{
    if (!std::isfinite(speed))
        return 1.0;
    return std::clamp(speed, 0.5, 4.0);
}

void PcapPlaybackController::reset(double speed)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = false;
        paused_ = false;
        has_previous_timestamp_ = false;
        previous_timestamp_sec_ = 0.0;
        scheduled_capture_sec_ = 0.0;
        played_capture_sec_ = 0.0;
        last_released_timestamp_sec_ = 0.0;
        speed_ = normalizeSpeed(speed);
        last_wall_time_ = std::chrono::steady_clock::now();
    }
    condition_.notify_all();
}

void PcapPlaybackController::resetTimeline()
{
    std::lock_guard<std::mutex> lock(mutex_);
    has_previous_timestamp_ = false;
    previous_timestamp_sec_ = 0.0;
    scheduled_capture_sec_ = 0.0;
    played_capture_sec_ = 0.0;
    last_wall_time_ = std::chrono::steady_clock::now();
}

void PcapPlaybackController::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        paused_ = false;
    }
    condition_.notify_all();
}

void PcapPlaybackController::setPaused(bool paused)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_)
            return;
        if (paused_ == paused)
            return;
        const auto now = std::chrono::steady_clock::now();
        advancePlaybackClockLocked(now);
        paused_ = paused;
        last_wall_time_ = now;
    }
    condition_.notify_all();
}

bool PcapPlaybackController::isPaused() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return paused_;
}

void PcapPlaybackController::setSpeed(double speed)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        advancePlaybackClockLocked(now);
        speed_ = normalizeSpeed(speed);
        last_wall_time_ = now;
    }
    condition_.notify_all();
}

double PcapPlaybackController::speed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return speed_;
}

double PcapPlaybackController::lastReleasedTimestamp() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_released_timestamp_sec_;
}

void PcapPlaybackController::advancePlaybackClockLocked(
    std::chrono::steady_clock::time_point now)
{
    if (!paused_ && !stopped_) {
        const double elapsedWallSec =
            std::chrono::duration<double>(now - last_wall_time_).count();
        if (elapsedWallSec > 0.0)
            played_capture_sec_ += elapsedWallSec * speed_;
    }
    last_wall_time_ = now;
}

bool PcapPlaybackController::waitForPacket(double captureTimestampSec)
{
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() { return stopped_ || !paused_; });
    if (stopped_)
        return false;

    if (!has_previous_timestamp_) {
        previous_timestamp_sec_ = captureTimestampSec;
        has_previous_timestamp_ = true;
        scheduled_capture_sec_ = 0.0;
        played_capture_sec_ = 0.0;
        last_wall_time_ = std::chrono::steady_clock::now();
        last_released_timestamp_sec_ = captureTimestampSec;
        return true;
    }

    scheduled_capture_sec_ +=
        std::max(0.0, captureTimestampSec - previous_timestamp_sec_);
    previous_timestamp_sec_ = captureTimestampSec;

    while (true) {
        condition_.wait(lock, [this]() { return stopped_ || !paused_; });
        if (stopped_)
            return false;

        advancePlaybackClockLocked(std::chrono::steady_clock::now());
        const double remainingCaptureSec =
            scheduled_capture_sec_ - played_capture_sec_;
        if (remainingCaptureSec <= 0.0) {
            last_released_timestamp_sec_ = captureTimestampSec;
            return true;
        }

        condition_.wait_for(
            lock,
            std::chrono::duration<double>(remainingCaptureSec / speed_));
    }
}

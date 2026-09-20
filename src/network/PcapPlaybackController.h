#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

/**
 * PCAP 离线回放时钟。
 *
 * 所有接口均可从界面线程直接调用；waitForPacket() 只在 PCAP 读取线程调用。
 * 控制器只决定何时放行抓包记录，不修改记录携带的原始时间戳。
 */
class PcapPlaybackController
{
public:
    void reset(double speed = 1.0);
    void resetTimeline();
    void stop();

    void setPaused(bool paused);
    bool isPaused() const;

    void setSpeed(double speed);
    double speed() const;
    double lastReleasedTimestamp() const;

    /** 返回 false 表示播放已被 stop() 取消。 */
    bool waitForPacket(double captureTimestampSec);

private:
    static double normalizeSpeed(double speed);
    void advancePlaybackClockLocked(
        std::chrono::steady_clock::time_point now);

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool stopped_ = false;
    bool paused_ = false;
    bool has_previous_timestamp_ = false;
    double speed_ = 1.0;
    double previous_timestamp_sec_ = 0.0;
    double scheduled_capture_sec_ = 0.0;
    double played_capture_sec_ = 0.0;
    double last_released_timestamp_sec_ = 0.0;
    std::chrono::steady_clock::time_point last_wall_time_{};
};

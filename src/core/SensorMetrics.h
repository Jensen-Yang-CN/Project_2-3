#ifndef SENSORMETRICS_H
#define SENSORMETRICS_H

#include <atomic>
#include <array>
#include <mutex>

/** 各传感器帧计数与最新时间戳；由后台线程定期汇总写入系统日志 */
class SensorMetrics {
public:
    enum class Channel : int {
        Lidar210 = 0,
        Lidar211,
        CameraLeft,
        Imu,
        FusedCloud,
        Count
    };

    static SensorMetrics &instance();

    void recordFrame(Channel channel, double timestamp);

    /** 由 SystemFileLogger 监控线程调用，interval_sec 为统计窗口长度 */
    void flushSummary(double interval_sec);

private:
    SensorMetrics() = default;

    struct Slot {
        std::atomic<uint64_t> frame_count{0};
        std::atomic<double> latest_ts{0.0};
    };

    static const char *channelName(Channel channel);

    std::array<Slot, static_cast<size_t>(Channel::Count)> slots_{};
    std::mutex flush_mutex_;
};

#endif  // SENSORMETRICS_H

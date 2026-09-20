#include "SensorMetrics.h"

#include "SystemFileLogger.h"

#include <QJsonObject>

SensorMetrics &SensorMetrics::instance()
{
    static SensorMetrics inst;
    return inst;
}

const char *SensorMetrics::channelName(Channel channel)
{
    switch (channel) {
    case Channel::Lidar210: return "lidar210";
    case Channel::Lidar211: return "lidar211";
    case Channel::CameraLeft: return "camera_left";
    case Channel::Imu: return "imu";
    case Channel::FusedCloud: return "fused_cloud";
    default: return "unknown";
    }
}

void SensorMetrics::recordFrame(Channel channel, double timestamp)
{
    const size_t idx = static_cast<size_t>(channel);
    if (idx >= slots_.size())
        return;
    slots_[idx].frame_count.fetch_add(1, std::memory_order_relaxed);
    if (timestamp > 0.0) {
        slots_[idx].latest_ts.store(timestamp, std::memory_order_relaxed);
    }
}

void SensorMetrics::flushSummary(double interval_sec)
{
    if (interval_sec <= 0.0)
        return;

    std::lock_guard<std::mutex> lock(flush_mutex_);

    QJsonObject data;
    data.insert(QStringLiteral("interval_sec"), interval_sec);

    for (size_t i = 0; i < slots_.size(); ++i) {
        const uint64_t count = slots_[i].frame_count.exchange(0, std::memory_order_relaxed);
        const double fps = static_cast<double>(count) / interval_sec;
        const QString prefix = QString::fromUtf8(channelName(static_cast<Channel>(i)));
        data.insert(prefix + QStringLiteral("_fps"), fps);
        data.insert(prefix + QStringLiteral("_ts"),
                    slots_[i].latest_ts.load(std::memory_order_relaxed));
    }

    SystemFileLogger::instance().logEvent(
        SystemLogLevel::Info,
        "Sensor",
        "sensor_fps",
        data);
}

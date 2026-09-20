#pragma once

#include <QStringList>
#include <QtGlobal>

/** 描述一次普通 PCAP 回放或原始时间戳跳转任务。 */
struct PcapPlaybackRequest
{
    QStringList files;
    double playback_speed = 1.0;
    bool seek_enabled = false;
    double target_timestamp_sec = 0.0;
    double preroll_sec = 2.0;
    bool scan_range = true;
    /** 数据源/跳转代号；用于丢弃上一条时间线迟到的 UDP 包。 */
    quint64 timeline_generation = 1;
};

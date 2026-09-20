#pragma once

#include <QString>
#include <QTimeZone>

namespace pcap_playback_time {

struct ParseResult
{
    bool ok = false;
    double unix_seconds = 0.0;
    QString error;
};

ParseResult parseTimestamp(
    const QString &text,
    const QTimeZone &zone = QTimeZone::systemTimeZone());

QString formatLocal(
    double unixSeconds,
    const QTimeZone &zone = QTimeZone::systemTimeZone());

QString snapshotBaseName(
    double unixSeconds,
    const QTimeZone &zone = QTimeZone::systemTimeZone());

} // namespace pcap_playback_time

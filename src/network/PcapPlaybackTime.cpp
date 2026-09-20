#include "PcapPlaybackTime.h"

#include <QDateTime>

#include <cmath>

namespace pcap_playback_time {
namespace {

QDateTime localDateTime(double unixSeconds, const QTimeZone &zone)
{
    if (!std::isfinite(unixSeconds) || unixSeconds <= 0.0 || !zone.isValid())
        return {};
    const qint64 milliseconds =
        static_cast<qint64>(std::llround(unixSeconds * 1000.0));
    return QDateTime::fromMSecsSinceEpoch(milliseconds, zone);
}

} // namespace

ParseResult parseTimestamp(const QString &text, const QTimeZone &zone)
{
    ParseResult result;
    const QString trimmed = text.trimmed();
    if (trimmed.isEmpty()) {
        result.error = QStringLiteral("时间不能为空");
        return result;
    }

    bool numericOk = false;
    const double numeric = trimmed.toDouble(&numericOk);
    if (numericOk) {
        if (std::isfinite(numeric) && numeric > 0.0) {
            result.ok = true;
            result.unix_seconds = numeric;
        } else {
            result.error = QStringLiteral("Unix 时间戳必须是有限正数");
        }
        return result;
    }

    if (!zone.isValid()) {
        result.error = QStringLiteral("系统时区无效");
        return result;
    }

    const QString format = QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz");
    const QDateTime parsed = QDateTime::fromString(trimmed, format);
    if (!parsed.isValid() || parsed.toString(format) != trimmed) {
        result.error = QStringLiteral(
            "时间格式无效，请输入 yyyy-MM-dd HH:mm:ss.zzz 或 Unix 时间戳");
        return result;
    }

    const QDateTime zoned(parsed.date(), parsed.time(), zone);
    if (!zoned.isValid()) {
        result.error = QStringLiteral("该本地时间在当前时区中无效");
        return result;
    }

    result.ok = true;
    result.unix_seconds = zoned.toMSecsSinceEpoch() / 1000.0;
    return result;
}

QString formatLocal(double unixSeconds, const QTimeZone &zone)
{
    const QDateTime dateTime = localDateTime(unixSeconds, zone);
    return dateTime.isValid()
        ? dateTime.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        : QString();
}

QString snapshotBaseName(double unixSeconds, const QTimeZone &zone)
{
    const QDateTime dateTime = localDateTime(unixSeconds, zone);
    return dateTime.isValid()
        ? dateTime.toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))
        : QString();
}

} // namespace pcap_playback_time

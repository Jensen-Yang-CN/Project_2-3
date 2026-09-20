#include "UdpDispatcher.h"
#include "common/types.h"   // PacketPtr
#include <QThread>     // ⭐ 必须加
#include <QPointer>
#include <QMetaObject>
#include <QDebug>
#include <QJsonObject>
#include <QAbstractSocket>

#include "SystemFileLogger.h"

namespace {

constexpr int kDispatchStatsIntervalMs = 5000;
constexpr int kMaxFirstMissLog = 8;

bool isIpv4MappedAddress(const QHostAddress &addr)
{
    return addr.protocol() == QAbstractSocket::IPv6Protocol && addr.toIPv4Address() != 0;
}

QHostAddress toIpv4Endpoint(const QHostAddress &addr)
{
    if (isIpv4MappedAddress(addr))
        return QHostAddress(addr.toIPv4Address());
    return addr;
}

}  // namespace

UdpDispatcher::UdpDispatcher(QObject *parent)
{
    (void)parent;
}

QString UdpDispatcher::normalizedSourceKey(const QHostAddress &ip, quint16 port)
{
    const QHostAddress normalized = toIpv4Endpoint(ip);
    return normalized.toString() + QLatin1Char(':') + QString::number(port);
}

void UdpDispatcher::invokeRule(const Rule &rule,
                               const QByteArray &payload,
                               double timestamp,
                               quint64 timelineGeneration)
{
    if (!rule.receiver || !rule.callback)
        return;

    PacketPtr pkt = PacketPtr::create(payload);

    QThread *targetThread = rule.receiver->thread();
    QThread *currentThread = QThread::currentThread();

    if (targetThread == currentThread) {
        rule.callback(*pkt, timestamp, timelineGeneration);
        return;
    }

    QPointer<QObject> safeReceiver = rule.receiver;
    auto cb = rule.callback;

    QMetaObject::invokeMethod(rule.receiver,
        [safeReceiver, cb, pkt, timestamp, timelineGeneration]() {
            if (!safeReceiver)
                return;
            cb(*pkt, timestamp, timelineGeneration);
        },
        Qt::QueuedConnection);
}

void UdpDispatcher::recordDispatch(bool matched,
                                   const QString &sourceKey,
                                   const QString &matchMode)
{
    QMutexLocker lock(&m_statsMutex);

    if (matched) {
        ++m_matchedTotal;
        const QString key = matchMode.isEmpty() ? sourceKey : sourceKey + QLatin1Char('(') + matchMode + QLatin1Char(')');
        ++m_matchedBySource[key];
    } else {
        ++m_unmatchedTotal;
        ++m_unmatchedBySource[sourceKey];

        if (!m_loggedMissKeys.contains(sourceKey) && m_loggedMissKeys.size() < kMaxFirstMissLog) {
            m_loggedMissKeys.insert(sourceKey);
            qWarning() << "[UdpDispatcher] 无匹配分流规则:" << sourceKey
                       << "请在 devices.json 核对 ip 与 port(源端口)";
        }
    }

    flushDispatchStats(false);
}

void UdpDispatcher::flushDispatchStats(bool force)
{
    if (!m_statsTimerStarted) {
        m_statsTimer.start();
        m_statsTimerStarted = true;
        if (!force)
            return;
    }

    if (!force && m_statsTimer.elapsed() < kDispatchStatsIntervalMs)
        return;

    if (m_matchedTotal == 0 && m_unmatchedTotal == 0) {
        m_statsTimer.restart();
        return;
    }

    const double intervalSec = qMax(m_statsTimer.elapsed() / 1000.0, 0.001);

    QStringList matchedLines;
    for (auto it = m_matchedBySource.constBegin(); it != m_matchedBySource.constEnd(); ++it) {
        const double pps = static_cast<double>(it.value()) / intervalSec;
        matchedLines.append(QStringLiteral("%1 %2包/s").arg(it.key()).arg(pps, 0, 'f', 1));
    }
    matchedLines.sort();

    QStringList unmatchedLines;
    for (auto it = m_unmatchedBySource.constBegin(); it != m_unmatchedBySource.constEnd(); ++it) {
        const double pps = static_cast<double>(it.value()) / intervalSec;
        unmatchedLines.append(QStringLiteral("%1 %2包/s").arg(it.key()).arg(pps, 0, 'f', 1));
    }
    unmatchedLines.sort();

    qInfo() << "[UdpDispatcher] 分流统计"
            << QStringLiteral("命中 %1 包/s").arg(static_cast<double>(m_matchedTotal) / intervalSec, 0, 'f', 1)
            << QStringLiteral("未命中 %1 包/s").arg(static_cast<double>(m_unmatchedTotal) / intervalSec, 0, 'f', 1);
    if (!matchedLines.isEmpty())
        qInfo() << "[UdpDispatcher] 命中:" << matchedLines.join(QStringLiteral(" | "));
    if (!unmatchedLines.isEmpty())
        qWarning() << "[UdpDispatcher] 未命中:" << unmatchedLines.join(QStringLiteral(" | "));

    if (SystemFileLogger::instance().isEnabled()) {
        QJsonObject data;
        data.insert(QStringLiteral("interval_sec"), intervalSec);
        data.insert(QStringLiteral("matched_packets"), static_cast<qint64>(m_matchedTotal));
        data.insert(QStringLiteral("unmatched_packets"), static_cast<qint64>(m_unmatchedTotal));
        QJsonObject matchedObj;
        for (auto it = m_matchedBySource.constBegin(); it != m_matchedBySource.constEnd(); ++it)
            matchedObj.insert(it.key(), static_cast<qint64>(it.value()));
        QJsonObject unmatchedObj;
        for (auto it = m_unmatchedBySource.constBegin(); it != m_unmatchedBySource.constEnd(); ++it)
            unmatchedObj.insert(it.key(), static_cast<qint64>(it.value()));
        data.insert(QStringLiteral("matched_by_source"), matchedObj);
        data.insert(QStringLiteral("unmatched_by_source"), unmatchedObj);
        SystemFileLogger::instance().logEvent(
            SystemLogLevel::Info, "UdpDispatcher", "dispatch_stats", data);
    }

    m_statsTimer.restart();
    m_matchedTotal = 0;
    m_unmatchedTotal = 0;
    m_matchedBySource.clear();
    m_unmatchedBySource.clear();
}

void UdpDispatcher::dispatch(const QHostAddress &ip,
                             quint16 port,
                             const QByteArray &payload,
                             double timestamp)
{
    dispatchAccepted(
        ip, port, payload, timestamp,
        m_acceptedTimelineGeneration.load(std::memory_order_acquire));
}

void UdpDispatcher::dispatchAccepted(const QHostAddress &ip,
                                     quint16 port,
                                     const QByteArray &payload,
                                     double timestamp,
                                     quint64 timelineGeneration)
{
    const QString sourceKey = normalizedSourceKey(ip, port);

    // 1. 精确匹配 IP + 端口（雷达 / IMU / 固定端口设备）
    RouteKey key{ip, port};
    auto it = routeTable.constFind(key);
    if (it != routeTable.constEnd()) {
        recordDispatch(true, sourceKey, QString());
        invokeRule(it.value(), payload, timestamp, timelineGeneration);
        return;
    }

    // IPv4-mapped 地址再试一次，避免 Qt 返回 ::ffff:x.x.x.x 导致查表失败
    if (isIpv4MappedAddress(ip)) {
        RouteKey v4Key{toIpv4Endpoint(ip), port};
        auto v4It = routeTable.constFind(v4Key);
        if (v4It != routeTable.constEnd()) {
            recordDispatch(true, sourceKey, QStringLiteral("ipv4_mapped"));
            invokeRule(v4It.value(), payload, timestamp, timelineGeneration);
            return;
        }
    }

    // 2. 仅匹配源 IP（RTP 动态端口相机）
    auto ipIt = ipRouteTable.constFind(ip);
    if (ipIt != ipRouteTable.constEnd()) {
        recordDispatch(true, sourceKey, QStringLiteral("by_ip"));
        invokeRule(ipIt.value(), payload, timestamp, timelineGeneration);
        return;
    }

    if (isIpv4MappedAddress(ip)) {
        auto v4IpIt = ipRouteTable.constFind(toIpv4Endpoint(ip));
        if (v4IpIt != ipRouteTable.constEnd()) {
            recordDispatch(true, sourceKey, QStringLiteral("by_ip,ipv4_mapped"));
            invokeRule(v4IpIt.value(), payload, timestamp, timelineGeneration);
            return;
        }
    }

    recordDispatch(false, sourceKey, QString());
}

void UdpDispatcher::dispatchWithGeneration(const QHostAddress &ip,
                                           quint16 port,
                                           const QByteArray &payload,
                                           double timestamp,
                                           quint64 timelineGeneration)
{
    if (timelineGeneration
        != m_acceptedTimelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    dispatchAccepted(ip, port, payload, timestamp, timelineGeneration);
}

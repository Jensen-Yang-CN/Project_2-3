#include "UdpSocketReceiver.h"

#include "AppConfig.h"
#include "DeviceConfig.h"
#include "devicemanager.h"

#include <QDateTime>
#include <QDebug>
#include <QElapsedTimer>
#include <QHash>
#include <QJsonObject>
#include <QMap>
#include <QStringList>
#include <QUdpSocket>

#include "SystemFileLogger.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace {

struct PlannedBind {
    QHostAddress local_ip;
    quint16 port = 0;
    QStringList multicast_groups;
};

constexpr int kRecvStatsIntervalMs = 5000;

bool isIpv4MappedAddress(const QHostAddress &addr)
{
    return addr.protocol() == QAbstractSocket::IPv6Protocol && addr.toIPv4Address() != 0;
}

QString normalizedSourceKey(const QHostAddress &ip, quint16 port)
{
    QHostAddress normalized = ip;
    if (isIpv4MappedAddress(normalized))
        normalized = QHostAddress(normalized.toIPv4Address());
    return normalized.toString() + QLatin1Char(':') + QString::number(port);
}

QString localIpForDevice(const device::DeviceInfo &dev, const NetworkReceiveConfig &net)
{
    if (dev.metadata.contains(QStringLiteral("bind_local_ip")))
        return dev.metadata.value(QStringLiteral("bind_local_ip")).toString();

    switch (dev.type) {
    case device::DeviceType::Lidar_LS:
    case device::DeviceType::Lidar_RS:
    case device::DeviceType::Camera_H264:
        return net.lidar_local_ip;
    case device::DeviceType::IMU:
        return net.navigation_local_ip;
    case device::DeviceType::Camera_H265:
        return net.video_local_ip;
    default:
        return net.lidar_local_ip;
    }
}

QString defaultLsMulticastGroup(const QString &lidarIp)
{
    const QStringList parts = lidarIp.split(QLatin1Char('.'));
    if (parts.size() == 4)
        return QStringLiteral("226.1.1.") + parts.at(3);
    return QString();
}

void addPlannedPort(QMap<QString, PlannedBind> &planned,
                    const QHostAddress &localIp,
                    quint16 port,
                    const QString &multicastGroup)
{
    if (port == 0)
        return;

    const QString key = localIp.toString() + QLatin1Char(':') + QString::number(port);
    if (!planned.contains(key)) {
        PlannedBind bind;
        bind.local_ip = localIp;
        bind.port = port;
        planned.insert(key, bind);
    }
    if (!multicastGroup.isEmpty() && !planned[key].multicast_groups.contains(multicastGroup))
        planned[key].multicast_groups.append(multicastGroup);
}

QList<PlannedBind> buildPlannedBinds()
{
    const NetworkReceiveConfig &net = AppConfig::instance().network();
    QMap<QString, PlannedBind> planned;

    for (const device::DeviceInfo &dev : DeviceManager::instance().devices()) {
        if (dev.type == device::DeviceType::Camera_H265)
            continue;

        const QHostAddress localIp(localIpForDevice(dev, net));
        if (localIp.protocol() != QAbstractSocket::IPv4Protocol) {
            qWarning() << "[UdpSocketReceiver] 跳过设备" << dev.name << "：无效本地 IP";
            continue;
        }

        QString multicastGroup = dev.metadata.value(QStringLiteral("multicast_group")).toString();
        if (multicastGroup.isEmpty() && dev.type == device::DeviceType::Lidar_LS)
            multicastGroup = defaultLsMulticastGroup(dev.ip);

        const int bindPort = dev.metadata.value(QStringLiteral("bind_port"), dev.port).toInt();
        addPlannedPort(planned, localIp, static_cast<quint16>(bindPort), multicastGroup);

        if (dev.extraPort > 0) {
            const int extraBindPort =
                dev.metadata.value(QStringLiteral("bind_extra_port"), dev.extraPort).toInt();
            addPlannedPort(planned, localIp, static_cast<quint16>(extraBindPort), QString());
        }
    }

    return planned.values();
}

bool joinMulticastGroup(qintptr socketFd, const QHostAddress &group, const QHostAddress &iface)
{
    if (group.protocol() != QAbstractSocket::IPv4Protocol
        || iface.protocol() != QAbstractSocket::IPv4Protocol) {
        return false;
    }

    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = htonl(group.toIPv4Address());
    mreq.imr_interface.s_addr = htonl(iface.toIPv4Address());
    return setsockopt(static_cast<int>(socketFd), IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      reinterpret_cast<const char *>(&mreq), sizeof(mreq)) == 0;
}

bool leaveMulticastGroup(qintptr socketFd, const QHostAddress &group, const QHostAddress &iface)
{
    if (group.protocol() != QAbstractSocket::IPv4Protocol
        || iface.protocol() != QAbstractSocket::IPv4Protocol) {
        return false;
    }

    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = htonl(group.toIPv4Address());
    mreq.imr_interface.s_addr = htonl(iface.toIPv4Address());
    return setsockopt(static_cast<int>(socketFd), IPPROTO_IP, IP_DROP_MEMBERSHIP,
                      reinterpret_cast<const char *>(&mreq), sizeof(mreq)) == 0;
}

}  // namespace

struct UdpSocketReceiver::SocketBinding {
    QUdpSocket *socket = nullptr;
    QHostAddress local_ip;
    quint16 port = 0;
    QStringList joined_groups;
};

UdpSocketReceiver::UdpSocketReceiver(QObject *parent)
    : QObject(parent)
{
}

UdpSocketReceiver::~UdpSocketReceiver()
{
    stopReceiving();
}

void UdpSocketReceiver::clearBindings()
{
    for (SocketBinding *binding : m_bindings)
        teardownBinding(binding);
    qDeleteAll(m_bindings);
    m_bindings.clear();
}

bool UdpSocketReceiver::setupBinding(SocketBinding *binding)
{
    if (!binding || binding->local_ip.protocol() != QAbstractSocket::IPv4Protocol)
        return false;

    binding->socket = new QUdpSocket(this);
    binding->socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 8 * 1024 * 1024);

    const QHostAddress bindAddress = binding->local_ip;
    if (!binding->socket->bind(bindAddress, binding->port,
                               QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        return false;
    }

    const qintptr fd = binding->socket->socketDescriptor();
    for (const QString &groupText : binding->joined_groups) {
        const QHostAddress group(groupText);
        if (!joinMulticastGroup(fd, group, bindAddress)) {
            qWarning() << "[UdpSocketReceiver] 加入组播失败"
                       << groupText << "iface" << bindAddress.toString();
        } else {
            qInfo() << "[UdpSocketReceiver] 已加入组播" << groupText
                    << "本地" << bindAddress.toString() << ':' << binding->port;
        }
    }

    connect(binding->socket, &QUdpSocket::readyRead, this, &UdpSocketReceiver::onDatagramReady);
    qInfo() << "[UdpSocketReceiver] 已绑定" << bindAddress.toString() << ':' << binding->port;
    return true;
}

void UdpSocketReceiver::teardownBinding(SocketBinding *binding)
{
    if (!binding)
        return;

    if (binding->socket) {
        const qintptr fd = binding->socket->socketDescriptor();
        if (fd >= 0) {
            for (const QString &groupText : binding->joined_groups) {
                leaveMulticastGroup(fd, QHostAddress(groupText), binding->local_ip);
            }
        }
        binding->socket->close();
        binding->socket->deleteLater();
        binding->socket = nullptr;
    }
}

void UdpSocketReceiver::startReceiving()
{
    if (m_receiving.load()) {
        emit receiveError(QStringLiteral("UDP 接收已在运行"));
        return;
    }

    clearBindings();
    m_recvStatsStarted = false;
    m_firstPacketLogged = false;
    m_totalPackets = 0;
    m_totalBytes = 0;
    m_packetsBySource.clear();
    m_bytesBySource.clear();

    const QList<PlannedBind> planned = buildPlannedBinds();
    if (planned.isEmpty()) {
        emit receiveError(QStringLiteral("未生成任何 UDP 绑定规则，请检查 devices.json"));
        return;
    }

    QStringList okSummary;
    QStringList failSummary;
    for (const PlannedBind &plan : planned) {
        auto *binding = new SocketBinding;
        binding->local_ip = plan.local_ip;
        binding->port = plan.port;
        binding->joined_groups = plan.multicast_groups;

        if (!setupBinding(binding)) {
            teardownBinding(binding);
            delete binding;
            const QString failLine =
                QStringLiteral("%1:%2").arg(plan.local_ip.toString()).arg(plan.port);
            failSummary.append(failLine);
            continue;
        }

        m_bindings.append(binding);
        QString line = QStringLiteral("%1:%2").arg(plan.local_ip.toString()).arg(plan.port);
        if (!plan.multicast_groups.isEmpty())
            line += QStringLiteral(" 组播[%1]").arg(plan.multicast_groups.join(QLatin1Char(',')));
        okSummary.append(line);
    }

    if (m_bindings.isEmpty()) {
        const QString detail = failSummary.isEmpty()
            ? QStringLiteral("未生成任何有效绑定")
            : failSummary.join(QStringLiteral(", "));
        const QString err = QStringLiteral("所有 UDP 绑定均失败：%1").arg(detail);
        qWarning() << "[UdpSocketReceiver]" << err;
        emit receiveError(err);
        return;
    }

    m_receiving = true;
    emit receiveStarted(okSummary.join(QStringLiteral(" | ")));
    qInfo() << "[UdpSocketReceiver] 在线 UDP 接收已启动:" << okSummary.join(QStringLiteral(" | "));

    if (!failSummary.isEmpty()) {
        const QString warn =
            QStringLiteral("部分 UDP 绑定失败已跳过：%1").arg(failSummary.join(QStringLiteral(", ")));
        qWarning() << "[UdpSocketReceiver]" << warn;
        emit receiveWarning(warn);
    }
}

void UdpSocketReceiver::stopReceiving()
{
    if (!m_receiving.load() && m_bindings.isEmpty())
        return;

    flushRecvStats(true);
    if (!m_firstPacketLogged) {
        qWarning() << "[UdpSocketReceiver] 停止时未收到任何 UDP 包"
                   << "请检查 local_ips、multicast_group、bind_port 及网卡连接";
    }
    clearBindings();
    m_receiving = false;
    emit receiveStopped();
    qInfo() << "[UdpSocketReceiver] 在线 UDP 接收已停止";
}

void UdpSocketReceiver::recordDatagram(const QHostAddress &senderIp,
                                         quint16 senderPort,
                                         int payloadSize)
{
    const QString key = normalizedSourceKey(senderIp, senderPort);
    ++m_totalPackets;
    m_totalBytes += static_cast<quint64>(payloadSize);
    ++m_packetsBySource[key];
    m_bytesBySource[key] += static_cast<quint64>(payloadSize);

    if (!m_firstPacketLogged) {
        m_firstPacketLogged = true;
        qInfo() << "[UdpSocketReceiver] 收到首包"
                << key << "payload" << payloadSize << "字节";
    }

    flushRecvStats(false);
}

void UdpSocketReceiver::flushRecvStats(bool force)
{
    if (!m_recvStatsStarted) {
        m_recvStatsTimer.start();
        m_recvStatsStarted = true;
        if (!force)
            return;
    }

    if (!force && m_recvStatsTimer.elapsed() < kRecvStatsIntervalMs)
        return;

    const double intervalSec = qMax(m_recvStatsTimer.elapsed() / 1000.0, 0.001);
    const double totalKibPerSec = (static_cast<double>(m_totalBytes) / 1024.0) / intervalSec;

    QStringList sourceLines;
    for (auto it = m_packetsBySource.constBegin(); it != m_packetsBySource.constEnd(); ++it) {
        const quint64 bytes = m_bytesBySource.value(it.key());
        const double pps = static_cast<double>(it.value()) / intervalSec;
        const double kibPerSec = (static_cast<double>(bytes) / 1024.0) / intervalSec;
        sourceLines.append(QStringLiteral("%1 %2包/s %.1fKiB/s")
                               .arg(it.key())
                               .arg(pps, 0, 'f', 1)
                               .arg(kibPerSec, 0, 'f', 1));
    }
    sourceLines.sort();

    const QString summary = sourceLines.isEmpty()
        ? QStringLiteral("无数据")
        : sourceLines.join(QStringLiteral(" | "));

    qInfo() << "[UdpSocketReceiver] 收包统计"
            << QStringLiteral("%.1fKiB/s").arg(totalKibPerSec, 0, 'f', 1)
            << "总包数" << m_totalPackets << ":" << summary;

    if (SystemFileLogger::instance().isEnabled()) {
        QJsonObject data;
        data.insert(QStringLiteral("interval_sec"), intervalSec);
        data.insert(QStringLiteral("total_packets"), static_cast<qint64>(m_totalPackets));
        data.insert(QStringLiteral("total_kib_per_sec"), totalKibPerSec);
        QJsonObject bySource;
        for (auto it = m_packetsBySource.constBegin(); it != m_packetsBySource.constEnd(); ++it) {
            QJsonObject one;
            one.insert(QStringLiteral("packets"), static_cast<qint64>(it.value()));
            one.insert(QStringLiteral("bytes"), static_cast<qint64>(m_bytesBySource.value(it.key())));
            bySource.insert(it.key(), one);
        }
        data.insert(QStringLiteral("by_source"), bySource);
        SystemFileLogger::instance().logEvent(
            SystemLogLevel::Info, "UdpSocketReceiver", "recv_stats", data);
    }

    m_recvStatsTimer.restart();
    m_totalPackets = 0;
    m_totalBytes = 0;
    m_packetsBySource.clear();
    m_bytesBySource.clear();
}

void UdpSocketReceiver::onDatagramReady()
{
    auto *socket = qobject_cast<QUdpSocket *>(sender());
    if (!socket)
        return;

    const double timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    while (socket->hasPendingDatagrams()) {
        QByteArray payload;
        payload.resize(static_cast<int>(socket->pendingDatagramSize()));
        QHostAddress senderIp;
        quint16 senderPort = 0;
        socket->readDatagram(payload.data(), payload.size(), &senderIp, &senderPort);
        recordDatagram(senderIp, senderPort, payload.size());
        emit udpPacket(senderIp, senderPort, payload, timestamp);
        emit udpPacketWithGeneration(
            senderIp, senderPort, payload, timestamp,
            m_timelineGeneration.load(std::memory_order_acquire));
    }
}

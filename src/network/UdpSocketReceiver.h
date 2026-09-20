#pragma once

#include <QElapsedTimer>
#include <QHash>
#include <QHostAddress>
#include <QList>
#include <QObject>
#include <QString>
#include <QStringList>

#include <atomic>

class QUdpSocket;

/**
 * 在线模式（非镜像交换机）：在指定本地 IP:端口 上 bind，并对组播地址执行 join/leave。
 * 发出与 LivePcapCapture / PcapngReader 相同格式的 udpPacket 信号，供 UdpDispatcher 分流。
 */
class UdpSocketReceiver : public QObject {
    Q_OBJECT
public:
    explicit UdpSocketReceiver(QObject *parent = nullptr);
    ~UdpSocketReceiver() override;

    bool isReceiving() const { return m_receiving.load(); }
    void setTimelineGeneration(quint64 generation) {
        m_timelineGeneration.store(generation, std::memory_order_release);
    }

public slots:
    void startReceiving();
    void stopReceiving();

signals:
    void udpPacket(QHostAddress ip, quint16 port, QByteArray payload, double timestamp);
    void udpPacketWithGeneration(QHostAddress ip, quint16 port, QByteArray payload,
                                 double timestamp, quint64 timelineGeneration);
    void receiveStarted(const QString &summary);
    void receiveStopped();
    /** 全部绑定失败等致命错误 */
    void receiveError(const QString &message);
    /** 部分端口绑定失败，已成功端口继续接收 */
    void receiveWarning(const QString &message);

private:
    struct SocketBinding;

    void clearBindings();
    bool setupBinding(SocketBinding *binding);
    void teardownBinding(SocketBinding *binding);
    void onDatagramReady();
    void recordDatagram(const QHostAddress &senderIp, quint16 senderPort, int payloadSize);
    void flushRecvStats(bool force = false);

    QList<SocketBinding *> m_bindings;
    std::atomic<bool> m_receiving{false};
    std::atomic<quint64> m_timelineGeneration{1};

    QElapsedTimer m_recvStatsTimer;
    bool m_recvStatsStarted = false;
    bool m_firstPacketLogged = false;
    quint64 m_totalPackets = 0;
    quint64 m_totalBytes = 0;
    QHash<QString, quint64> m_packetsBySource;
    QHash<QString, quint64> m_bytesBySource;
};

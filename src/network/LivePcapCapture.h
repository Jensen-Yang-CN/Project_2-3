#pragma once

#include <QHostAddress>
#include <QObject>
#include <QString>

#include <atomic>

// 前置声明，头文件不直接包含 pcap.h
struct pcap;
typedef struct pcap pcap_t;

/**
 * 在线模式：通过 Npcap 在指定网卡上实时抓包，解析 UDP 后发出与 PcapngReader 相同格式的信号，
 * 从而复用 UdpDispatcher → 各 Parser → Worker 的整条解码链路。
 *
 * 与 PcapngReader 的差异：
 * - 使用 pcap_open_live / pcap_create+activate，而非 pcap_open_offline
 * - 不做 pcap 回放控速（来一包处理一包）
 * - 默认开启混杂模式，便于交换机端口镜像场景
 */
class LivePcapCapture : public QObject {
    Q_OBJECT
public:
    explicit LivePcapCapture(QObject *parent = nullptr);
    ~LivePcapCapture() override;

    bool isCapturing() const { return m_capturing.load(); }
    void setTimelineGeneration(quint64 generation) {
        m_timelineGeneration.store(generation, std::memory_order_release);
    }

public slots:
    /** 在 deviceName 对应网卡上开始抓包（须在对象所在线程调用） */
    void startCapture(const QString &deviceName);
    /** 请求停止抓包（线程安全，可从 UI 线程 DirectConnection 调用） */
    void stopCapture();

signals:
    /** 与 PcapngReader::udpPacket 完全一致，供 UdpDispatcher::dispatch 消费 */
    void udpPacket(QHostAddress ip, quint16 port, QByteArray payload, double timestamp);
    void udpPacketWithGeneration(QHostAddress ip, quint16 port, QByteArray payload,
                                 double timestamp, quint64 timelineGeneration);
    void captureStarted(const QString &deviceName);
    void captureStopped();
    void captureError(const QString &message);

private:
    void closeHandle();
    void captureLoop();

    pcap_t *m_handle = nullptr;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_capturing{false};
    std::atomic<quint64> m_timelineGeneration{1};
    QString m_deviceName;
};

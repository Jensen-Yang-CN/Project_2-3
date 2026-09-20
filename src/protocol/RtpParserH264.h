#ifndef RTPPARSERH264_H
#define RTPPARSERH264_H

#include <QObject>
#include <QByteArray>
#include <QHostAddress>
#include <QtEndian>
#include <atomic>

class RtpParserH264 : public QObject
{
    Q_OBJECT
public:
    explicit RtpParserH264(const QHostAddress &ip, quint16 port, QObject *parent = nullptr)
        : QObject(parent), m_ip(ip), m_port(port) {
        m_fuBuffer.reserve(1024 * 512); // 预留512KB
        // 初始化状态变量
        m_bindSsrc = 0;
        m_lastSeq = 0;
        m_isSsrcLocked = false;
        m_isWaitingForEnd = false;
    }

    void setParserName(const QString &name) { parserName = name; }

signals:
    // 发送给 VideoWorker 的信号
    void rtpPayloadReady(const QByteArray &nal, double timestamp);
    void rtpPayloadReadyWithGeneration(const QByteArray &nal, double timestamp,
                                       quint64 timelineGeneration);

public slots:
    // 由 UdpDispatcher 调用
    void inputPacket(const QByteArray &data, double timestamp);
    void inputPacketWithGeneration(const QByteArray &data, double timestamp,
                                   quint64 timelineGeneration);

    /** 停止/切换数据源时调用，清除 SSRC 锁定与组包缓存 */
    void resetStreamState(quint64 timelineGeneration = 0);

    // 【新增槽函数】接收来自其他端口解析器的 SPS/PPS 数据
    void onExtraConfigReceived(const QByteArray &config, double timestamp) {
        Q_UNUSED(timestamp);
        m_extraConfig = config;
    }

private:
    // 基础接口变量（保持不变）
    QHostAddress m_ip;
    quint16 m_port;
    QString parserName;
    QByteArray m_fuBuffer;      // 分片组包缓存
    QByteArray m_extraConfig;   // 缓存参数包
    bool m_isWaitingForEnd = false;

    // 内部流控变量
    quint32 m_bindSsrc;         // 锁定的 SSRC
    quint16 m_lastSeq;          // 上一次的序列号
    bool m_isSsrcLocked;        // 锁定标志
    std::atomic<quint64> m_timelineGeneration{1};
    quint64 m_fuGeneration = 0;
};

#endif // RTPPARSERH264_H

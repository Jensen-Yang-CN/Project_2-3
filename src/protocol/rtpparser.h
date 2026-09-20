#pragma once
#include <QObject>
#include <QByteArray>
#include <QMap>
#include <QHostAddress>
#include <atomic>

struct FrameBuffer {
    QByteArray data;
    quint32 timestamp;
};

class RtpParser : public QObject {
    Q_OBJECT
public:
    explicit RtpParser(const QHostAddress &ip, quint16 port, QObject *parent = nullptr)
            : QObject(parent), m_ip(ip), m_port(port) {}

    void setParserName(const QString &name) { m_name = name; }

public slots:
    void inputPacket(const QByteArray &data, double timestamp);
    void inputPacketWithGeneration(const QByteArray &data, double timestamp,
                                   quint64 timelineGeneration);
    /** 停止/切换数据源时调用，清除 SSRC 锁定与组包缓存 */
    void resetStreamState(quint64 timelineGeneration = 0);

signals:
    void rtpPayloadReady(const QByteArray &nal,double timestamp);
    void rtpPayloadReadyWithGeneration(const QByteArray &nal, double timestamp,
                                       quint64 timelineGeneration);

private:
    bool acceptRtpStream(const QByteArray &packet);

    QByteArray fuBuffer;
    bool fuStarted = false;
    QString m_name;
    QHostAddress m_ip;
    quint16 m_port;

    quint32 m_bindSsrc = 0;
    quint16 m_lastSeq = 0;
    bool m_isSsrcLocked = false;
    std::atomic<quint64> m_timelineGeneration{1};
    quint64 m_fuGeneration = 0;
};

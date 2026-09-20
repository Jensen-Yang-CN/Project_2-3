#ifndef IMUPARSER_H
#define IMUPARSER_H

#include <QObject>
#include <QHostAddress>
#include <atomic>
#include "NaviProtocol.h"
class IMUParser : public QObject
{
    Q_OBJECT
public:
    explicit IMUParser(const QHostAddress &ip, quint16 port,QObject *parent = nullptr)
        :QObject(parent),m_ip(ip),m_port(port){}

signals:
    void rtpPayloadReady(const QByteArray &payload);
    /** 导航载荷 + 抓包时间戳（与雷达点云时间轴一致） */
    void navPayloadReady(const QByteArray &payload, double captureTimestamp);
    void navPayloadReadyWithGeneration(const QByteArray &payload,
                                       double captureTimestamp,
                                       quint64 timelineGeneration);
    void imuDataReady(const IMUParsedData &data);
public slots:
    void inputPacket(const QByteArray &packet, double timestamp);
    void inputPacketWithGeneration(const QByteArray &packet, double timestamp,
                                   quint64 timelineGeneration);
    void resetTimeline(quint64 timelineGeneration);
private:
    QHostAddress m_ip;
    quint16 m_port;
    bool isFirst = false;
    std::atomic<quint64> m_timelineGeneration{1};
};

#endif // IMUPARSER_H

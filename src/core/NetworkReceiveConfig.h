#ifndef NETWORKRECEIVECONFIG_H
#define NETWORKRECEIVECONFIG_H

#include <QString>

/** 在线 UDP 接收：本地网卡 IP 与接收模式（socket / pcap） */
struct NetworkReceiveConfig {
    /** socket：绑定本地 IP+端口并加入组播；pcap：Npcap 被动抓包（交换机镜像场景） */
    QString receive_mode = QStringLiteral("socket");

    QString lidar_local_ip = QStringLiteral("192.168.1.177");
    QString video_local_ip = QStringLiteral("192.168.58.177");
    QString navigation_local_ip = QStringLiteral("101.1.101.177");

    bool useSocketMode() const { return receive_mode.compare(QStringLiteral("socket"), Qt::CaseInsensitive) == 0; }
};

#endif  // NETWORKRECEIVECONFIG_H

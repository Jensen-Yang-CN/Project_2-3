#pragma once

#include <QString>
#include <QVector>

class QWidget;

/**
 * 通过 Npcap 的 pcap_findalldevs() 枚举本机可用网卡，供在线抓包前选择。
 */
struct PcapDeviceEntry {
    /** pcap_open_live / pcap_create 使用的设备名，如 \\Device\\NPF_{...} */
    QString name;
    /** 人类可读描述，如 "Intel(R) Ethernet..." */
    QString description;
    /** 该网卡已配置的 IPv4/IPv6 地址（多地址用逗号拼接，可能为空） */
    QString addresses;
};

class PcapDeviceDialog {
public:
    /** 调用 pcap_findalldevs 刷新网卡列表；失败时 error 写入错误信息 */
    static QVector<PcapDeviceEntry> listDevices(QString *error = nullptr);

    /**
     * 弹出模态对话框让用户选择一块网卡。
     * @return 选中设备的 name；用户取消或列表为空时返回空字符串。
     */
    static QString pickDevice(QWidget *parent, QString *error = nullptr);
};

#include "PcapDeviceDialog.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

// MinGW/ucrt64 下 Npcap SDK 不提供 pcap_inet_ntop，使用标准 inet_ntop
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <pcap.h>

namespace {

QString formatSockaddr(const sockaddr *sa)
{
    if (!sa)
        return {};

    char host[INET6_ADDRSTRLEN] = {};
    if (sa->sa_family == AF_INET) {
        const auto *sin = reinterpret_cast<const sockaddr_in *>(sa);
        if (!inet_ntop(AF_INET, &sin->sin_addr, host, sizeof(host)))
            return {};
    } else if (sa->sa_family == AF_INET6) {
        const auto *sin6 = reinterpret_cast<const sockaddr_in6 *>(sa);
        if (!inet_ntop(AF_INET6, &sin6->sin6_addr, host, sizeof(host)))
            return {};
    } else {
        return {};
    }
    return QString::fromLocal8Bit(host);
}

} // namespace

QVector<PcapDeviceEntry> PcapDeviceDialog::listDevices(QString *error)
{
    QVector<PcapDeviceEntry> result;

    char errbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_if_t *alldevs = nullptr;
    if (pcap_findalldevs(&alldevs, errbuf) == -1) {
        if (error)
            *error = QString::fromLocal8Bit(errbuf);
        return result;
    }

    for (pcap_if_t *dev = alldevs; dev != nullptr; dev = dev->next) {
        if (!dev->name)
            continue;

        PcapDeviceEntry entry;
        entry.name = QString::fromLocal8Bit(dev->name);
        entry.description = dev->description
            ? QString::fromLocal8Bit(dev->description)
            : entry.name;

        QStringList addrLines;
        for (pcap_addr_t *addr = dev->addresses; addr != nullptr; addr = addr->next) {
            const QString ip = formatSockaddr(addr->addr);
            if (!ip.isEmpty())
                addrLines.append(ip);
        }
        entry.addresses = addrLines.join(QStringLiteral(", "));
        result.append(entry);
    }

    pcap_freealldevs(alldevs);
    return result;
}

QString PcapDeviceDialog::pickDevice(QWidget *parent, QString *error)
{
    QString listError;
    QVector<PcapDeviceEntry> devices = listDevices(&listError);
    if (devices.isEmpty()) {
        const QString msg = listError.isEmpty()
            ? QStringLiteral("未找到可用网卡。请确认已安装 Npcap，且以管理员权限运行。")
            : listError;
        if (error)
            *error = msg;
        QMessageBox::warning(parent, QStringLiteral("在线抓包"), msg);
        return {};
    }

    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("选择抓包网卡"));
    dialog.resize(640, 420);

    auto *hint = new QLabel(QStringLiteral(
        "请选择连接交换机（或镜像口）的网卡。\n"
        "交换机需配置端口镜像(SPAN)，否则可能抓不到雷达/相机/IMU 的 UDP 包。"));
    hint->setWordWrap(true);

    auto *list = new QListWidget(&dialog);
    for (const PcapDeviceEntry &dev : devices) {
        QString line = dev.description;
        if (!dev.addresses.isEmpty())
            line += QStringLiteral("  [") + dev.addresses + QLatin1Char(']');
        line += QStringLiteral("\n") + dev.name;
        auto *item = new QListWidgetItem(line, list);
        item->setData(Qt::UserRole, dev.name);
    }
    if (list->count() > 0)
        list->setCurrentRow(0);

    auto *refreshBtn = new QPushButton(QStringLiteral("刷新列表"), &dialog);
    QObject::connect(refreshBtn, &QPushButton::clicked, &dialog, [&]() {
        QString refreshError;
        const QVector<PcapDeviceEntry> refreshed = listDevices(&refreshError);
        list->clear();
        for (const PcapDeviceEntry &dev : refreshed) {
            QString line = dev.description;
            if (!dev.addresses.isEmpty())
                line += QStringLiteral("  [") + dev.addresses + QLatin1Char(']');
            line += QStringLiteral("\n") + dev.name;
            auto *item = new QListWidgetItem(line, list);
            item->setData(Qt::UserRole, dev.name);
        }
        if (list->count() > 0)
            list->setCurrentRow(0);
        else if (!refreshError.isEmpty())
            QMessageBox::warning(&dialog, QStringLiteral("刷新失败"), refreshError);
    });

    auto *buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    auto *btnRow = new QHBoxLayout();
    btnRow->addWidget(refreshBtn);
    btnRow->addStretch();
    btnRow->addWidget(buttons);

    auto *layout = new QVBoxLayout(&dialog);
    layout->addWidget(hint);
    layout->addWidget(list, 1);
    layout->addLayout(btnRow);

    if (dialog.exec() != QDialog::Accepted)
        return {};

    const QListWidgetItem *current = list->currentItem();
    if (!current)
        return {};

    return current->data(Qt::UserRole).toString();
}

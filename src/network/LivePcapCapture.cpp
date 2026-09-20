#include "LivePcapCapture.h"

#include <pcap.h>

#include <QByteArray>
#include <QDebug>
#include <QElapsedTimer>
#include <QtEndian>

#include <cstring>

namespace {

/** 从以太网帧解析 UDP 载荷；逻辑与 PcapngReader::readLoop 保持一致 */
bool extractUdpPayload(const u_char *data, int caplen,
                       QHostAddress *outIp, quint16 *outPort,
                       QByteArray *outPayload, double *outTimestamp,
                       const struct pcap_pkthdr *header)
{
    if (!data || caplen < 42 || !outIp || !outPort || !outPayload || !outTimestamp || !header)
        return false;

    const u_char *ip = data + 14;
    if (((ip[0] >> 4) & 0x0F) != 4)
        return false;

    const quint8 ihl = static_cast<quint8>((ip[0] & 0x0F) * 4);
    if (caplen < static_cast<int>(14 + ihl + 8))
        return false;

    const u_char *udp = ip + ihl;
    *outPort = qFromBigEndian<quint16>(*(const quint16 *)(udp));

    quint32 ipAddr = 0;
    std::memcpy(&ipAddr, ip + 12, 4);
    *outIp = QHostAddress(qFromBigEndian(ipAddr));

    const int payloadLen = caplen - (14 + ihl + 8);
    if (payloadLen <= 0)
        return false;

    *outPayload = QByteArray(reinterpret_cast<const char *>(udp + 8), payloadLen);
    *outTimestamp = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
    return true;
}

} // namespace

LivePcapCapture::LivePcapCapture(QObject *parent)
    : QObject(parent)
{
}

LivePcapCapture::~LivePcapCapture()
{
    stopCapture();
    closeHandle();
}

void LivePcapCapture::closeHandle()
{
    if (m_handle) {
        pcap_close(m_handle);
        m_handle = nullptr;
    }
}

void LivePcapCapture::startCapture(const QString &deviceName)
{
    if (deviceName.isEmpty()) {
        emit captureError(QStringLiteral("网卡名称为空，无法开始在线抓包"));
        return;
    }

    if (m_capturing.load()) {
        emit captureError(QStringLiteral("在线抓包已在运行，请先停止"));
        return;
    }

    closeHandle();
    m_stopRequested = false;
    m_deviceName = deviceName;

    char errbuf[PCAP_ERRBUF_SIZE] = {};
    const QByteArray dev = deviceName.toLocal8Bit();

    // 推荐 API：pcap_create + 参数设置 + activate
    m_handle = pcap_create(dev.constData(), errbuf);
    if (!m_handle) {
        emit captureError(QStringLiteral("pcap_create 失败: %1")
                              .arg(QString::fromLocal8Bit(errbuf)));
        return;
    }

    pcap_set_snaplen(m_handle, 65535);
    pcap_set_promisc(m_handle, 1);   // 端口镜像场景通常需要混杂模式
    pcap_set_timeout(m_handle, 100); // 100ms 超时，便于响应 stopCapture
    if (pcap_set_buffer_size(m_handle, 16 * 1024 * 1024) != 0) {
        // 非致命：部分驱动不支持更大缓冲
        qDebug() << "LivePcapCapture: pcap_set_buffer_size 未生效，使用默认缓冲";
    }

    const int activateRc = pcap_activate(m_handle);
    if (activateRc < 0) {
        const QString msg = QStringLiteral("pcap_activate 失败(%1): %2")
                                .arg(activateRc)
                                .arg(QString::fromLocal8Bit(pcap_geterr(m_handle)));
        closeHandle();
        emit captureError(msg);
        return;
    }
    if (activateRc > 0) {
        qDebug() << "LivePcapCapture warning:" << QString::fromLocal8Bit(pcap_geterr(m_handle));
    }

    // 仅抓 UDP，减轻 CPU（仍由 UdpDispatcher 按 IP:端口二次过滤）
    struct bpf_program fp;
    const char filterExpr[] = "udp";
    if (pcap_compile(m_handle, &fp, filterExpr, 1, PCAP_NETMASK_UNKNOWN) == 0) {
        if (pcap_setfilter(m_handle, &fp) != 0) {
            qDebug() << "LivePcapCapture: BPF 过滤器设置失败，将抓全部以太网帧";
        }
        pcap_freecode(&fp);
    }

    m_capturing = true;
    emit captureStarted(deviceName);
    qDebug() << "LivePcapCapture: 开始在网卡上抓包" << deviceName;

    captureLoop();

    m_capturing = false;
    closeHandle();
    emit captureStopped();
    qDebug() << "LivePcapCapture: 抓包结束" << deviceName;
}

void LivePcapCapture::stopCapture()
{
    m_stopRequested = true;
}

void LivePcapCapture::captureLoop()
{
    if (!m_handle)
        return;

    struct pcap_pkthdr *header = nullptr;
    const u_char *data = nullptr;

    int udpCount = 0;
    int frameCount = 0;      // pcap 返回的帧数（含非 UDP）
    int nonUdpCount = 0;     // 有帧但解析不出 IPv4/UDP
    int timeoutCount = 0;
    QElapsedTimer summaryTimer;
    summaryTimer.start();
    qint64 lastSummaryMs = 0;
    qint64 lastHeartbeatMs = 0;

    qDebug() << "[在线抓包] 等待 UDP 数据...（无输出表示该网卡尚未收到 UDP，或镜像/网卡选错）";

    while (!m_stopRequested) {
        const int res = pcap_next_ex(m_handle, &header, &data);
        if (res == 0) {
            ++timeoutCount;
            const qint64 now = summaryTimer.elapsed();
            if (udpCount == 0 && now - lastHeartbeatMs >= 3000) {
                lastHeartbeatMs = now;
                qDebug().nospace()
                    << "[在线抓包] 运行中，已等待 " << (now / 1000)
                    << "s，UDP=0（超时次数=" << timeoutCount
                    << "，请确认交换机镜像/网卡是否正确）";
            }
            continue; // 100ms 超时，继续等
        }
        if (res == PCAP_ERROR_BREAK) {
            qDebug() << "[在线抓包] pcap_next_ex 中断";
            break;
        }
        if (res < 0) {
            qDebug() << "[在线抓包] pcap_next_ex 错误:" << pcap_geterr(m_handle);
            break;
        }

        ++frameCount;

        QHostAddress srcIp;
        quint16 srcPort = 0;
        QByteArray payload;
        double timestamp = 0.0;
        if (!extractUdpPayload(data, header->caplen, &srcIp, &srcPort, &payload, &timestamp, header)) {
            ++nonUdpCount;
            const qint64 now = summaryTimer.elapsed();
            if (udpCount == 0 && now - lastHeartbeatMs >= 3000) {
                lastHeartbeatMs = now;
                qDebug().nospace()
                    << "[在线抓包] 已收到以太网帧=" << frameCount
                    << "，但可解析 UDP=0，非 UDP/IPv4 帧=" << nonUdpCount
                    << "（可能有流量但不是 UDP）";
            }
            continue;
        }

        ++udpCount;
        // 调试：前 15 包逐条打印；之后约每 1 秒一条汇总，避免控制台刷屏
        if (udpCount <= 15) {
            qDebug().nospace()
                << "[在线抓包] #" << udpCount << ' '
                << srcIp.toString() << ':' << srcPort
                << " 载荷=" << payload.size() << "B";
        } else {
            const qint64 now = summaryTimer.elapsed();
            if (now - lastSummaryMs >= 1000) {
                lastSummaryMs = now;
                qDebug().nospace()
                    << "[在线抓包] 累计=" << udpCount << " 最近 "
                    << srcIp.toString() << ':' << srcPort
                    << " 载荷=" << payload.size() << "B";
            }
        }

        // 在线模式：不做离线回放那种 msleep 控速，直接按到达顺序下发
        emit udpPacket(srcIp, srcPort, payload, timestamp);
        emit udpPacketWithGeneration(
            srcIp, srcPort, payload, timestamp,
            m_timelineGeneration.load(std::memory_order_acquire));
    }

    if (udpCount > 0) {
        qDebug() << "[在线抓包] 结束，共收到 UDP" << udpCount << "包，以太网帧" << frameCount;
    } else if (frameCount > 0) {
        qDebug() << "[在线抓包] 结束，收到以太网帧" << frameCount
                 << "个，但无有效 UDP（非 UDP/IPv4=" << nonUdpCount << "）";
    } else {
        qDebug() << "[在线抓包] 结束，未收到任何数据（网卡无流量或镜像未生效）";
    }
}

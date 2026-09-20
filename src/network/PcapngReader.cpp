#include "pcapngreader.h"

#include <pcap.h>

#include <QDebug>
#include <QElapsedTimer>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr double kTimestampEpsilonSec = 1e-9;
constexpr int kTimestampSignalIntervalMs = 200;

double packetTimestamp(const pcap_pkthdr *header)
{
    return header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
}

} // namespace

PcapngReader::PcapngReader(QObject *parent)
    : QObject(parent), m_handle(nullptr)
{
}

PcapngReader::~PcapngReader()
{
    if (m_handle)
        pcap_close(m_handle);
}

void PcapngReader::setPlaybackConfig(const PcapPlaybackConfig &config)
{
    m_playbackConfig = config;
    m_playbackConfig.default_speed =
        std::clamp(m_playbackConfig.default_speed, 0.5, 4.0);
}

void PcapngReader::stopReading()
{
    m_stopRequested = true;
    m_playbackController.stop();
}

void PcapngReader::setPaused(bool paused)
{
    m_playbackController.setPaused(paused);
}

bool PcapngReader::isPaused() const
{
    return m_playbackController.isPaused();
}

void PcapngReader::setPlaybackSpeed(double speed)
{
    m_playbackController.setSpeed(speed);
}

double PcapngReader::playbackSpeed() const
{
    return m_playbackController.speed();
}

double PcapngReader::lastReleasedTimestamp() const
{
    return m_playbackController.lastReleasedTimestamp();
}

void PcapngReader::startAnalysis(const QStringList &files)
{
    PcapPlaybackRequest request;
    request.files = files;
    request.playback_speed = m_playbackConfig.default_speed;
    request.scan_range = true;
    startPlayback(request);
}

void PcapngReader::startPlayback(const PcapPlaybackRequest &request)
{
    PcapPlaybackRequest effective = request;
    effective.playback_speed =
        std::clamp(effective.playback_speed, 0.5, 4.0);
    effective.preroll_sec = std::clamp(effective.preroll_sec, 0.0, 30.0);

    m_stopRequested = false;
    m_playbackController.reset(effective.playback_speed);

    if (effective.files.isEmpty()) {
        emit playbackError(QStringLiteral("没有可播放的 PCAP 文件"));
        emit finished();
        return;
    }
    if (effective.seek_enabled
        && (!std::isfinite(effective.target_timestamp_sec)
            || effective.target_timestamp_sec <= 0.0)) {
        emit playbackError(QStringLiteral("原始时间戳无效"));
        emit finished();
        return;
    }

    if (effective.scan_range) {
        double firstTimestamp = 0.0;
        double lastTimestamp = 0.0;
        if (!scanCaptureRange(effective.files, firstTimestamp, lastTimestamp)) {
            if (!m_stopRequested)
                emit playbackError(QStringLiteral("PCAP 文件中没有可读取的数据包"));
            emit finished();
            return;
        }
        emit captureRangeReady(firstTimestamp, lastTimestamp);

        if (effective.seek_enabled
            && (effective.target_timestamp_sec + kTimestampEpsilonSec < firstTimestamp
                || effective.target_timestamp_sec - kTimestampEpsilonSec > lastTimestamp)) {
            emit playbackError(
                QStringLiteral("目标时间戳 %1 超出 PCAP 范围 %2 ~ %3")
                    .arg(effective.target_timestamp_sec, 0, 'f', 6)
                    .arg(firstTimestamp, 0, 'f', 6)
                    .arg(lastTimestamp, 0, 'f', 6));
            emit finished();
            return;
        }
    }

    bool seekReached = !effective.seek_enabled;
    bool prerollActive = false;
    for (const QString &file : effective.files) {
        if (m_stopRequested)
            break;

        if (open(file)) {
            m_playbackController.resetTimeline();
            readLoop(effective, seekReached, prerollActive);
            close();
        }
    }

    if (prerollActive) {
        emit seekPrerollChanged(false, effective.target_timestamp_sec);
        prerollActive = false;
    }
    if (effective.seek_enabled && !seekReached && !m_stopRequested) {
        emit playbackError(
            QStringLiteral("没有找到不小于目标时间戳 %1 的数据包")
                .arg(effective.target_timestamp_sec, 0, 'f', 6));
    }
    emit finished();
}

bool PcapngReader::scanCaptureRange(const QStringList &files,
                                    double &firstTimestamp,
                                    double &lastTimestamp)
{
    bool foundPacket = false;
    firstTimestamp = std::numeric_limits<double>::infinity();
    lastTimestamp = -std::numeric_limits<double>::infinity();

    for (const QString &file : files) {
        if (m_stopRequested)
            return false;

        char errbuf[PCAP_ERRBUF_SIZE] = {};
        const QByteArray path = file.toLocal8Bit();
        pcap_t *handle = pcap_open_offline(path.constData(), errbuf);
        if (!handle) {
            qWarning() << "[PCAP回放] 范围扫描无法打开:" << file << errbuf;
            continue;
        }

        pcap_pkthdr *header = nullptr;
        const u_char *data = nullptr;
        int result = 0;
        while (!m_stopRequested
               && (result = pcap_next_ex(handle, &header, &data)) >= 0) {
            Q_UNUSED(data);
            if (result == 0)
                continue;
            const double timestamp = packetTimestamp(header);
            firstTimestamp = std::min(firstTimestamp, timestamp);
            lastTimestamp = std::max(lastTimestamp, timestamp);
            foundPacket = true;
        }
        pcap_close(handle);
    }
    return foundPacket && !m_stopRequested;
}

bool PcapngReader::open(const QString &file)
{
    char errbuf[PCAP_ERRBUF_SIZE] = {};
    const QByteArray path = file.toLocal8Bit();
    m_handle = pcap_open_offline(path.constData(), errbuf);
    if (!m_handle)
        qWarning() << "[PCAP回放] 打开失败:" << file << errbuf;
    return m_handle != nullptr;
}

void PcapngReader::close()
{
    if (m_handle) {
        pcap_close(m_handle);
        m_handle = nullptr;
    }
}

void PcapngReader::readLoop(const PcapPlaybackRequest &request,
                            bool &seekReached,
                            bool &prerollActive)
{
    if (!m_handle)
        return;

    pcap_pkthdr *header = nullptr;
    const u_char *data = nullptr;
    int result = 0;
    QElapsedTimer timestampSignalTimer;
    timestampSignalTimer.start();
    bool timestampSignalSent = false;
    const double prerollStart =
        request.target_timestamp_sec - request.preroll_sec;

    qInfo() << "[PCAP回放] 时间戳调度器，倍速="
            << m_playbackController.speed()
            << "seek=" << request.seek_enabled
            << "target=" << request.target_timestamp_sec;

    while (!m_stopRequested
           && (result = pcap_next_ex(m_handle, &header, &data)) >= 0) {
        if (result == 0)
            continue;

        const double currentPacketTime = packetTimestamp(header);

        if (request.seek_enabled && !seekReached) {
            if (currentPacketTime + kTimestampEpsilonSec < prerollStart)
                continue;

            if (!prerollActive) {
                prerollActive = true;
                m_playbackController.reset(4.0);
                emit seekPrerollChanged(true, request.target_timestamp_sec);
            }

            if (currentPacketTime + kTimestampEpsilonSec
                >= request.target_timestamp_sec) {
                emit seekPrerollChanged(false, request.target_timestamp_sec);
                prerollActive = false;
                seekReached = true;
                m_playbackController.reset(request.playback_speed);
                emit seekCompleted(
                    request.target_timestamp_sec, currentPacketTime);
                emit playbackTimestampChanged(currentPacketTime);
                timestampSignalTimer.restart();
                timestampSignalSent = true;
            }
        }

        if (!m_playbackController.waitForPacket(currentPacketTime))
            break;

        if (!timestampSignalSent
            || timestampSignalTimer.elapsed() >= kTimestampSignalIntervalMs) {
            emit playbackTimestampChanged(currentPacketTime);
            timestampSignalTimer.restart();
            timestampSignalSent = true;
        }

        if (header->caplen < 42)
            continue;
        const u_char *ip = data + 14;
        if (((ip[0] >> 4) & 0x0f) != 4)
            continue;

        const quint8 ihl = (ip[0] & 0x0f) * 4;
        if (ip[9] != 17)
            continue;
        if (header->caplen < static_cast<uint>(14 + ihl + 8))
            continue;

        const u_char *udp = ip + ihl;
        quint16 sourcePortNetworkOrder = 0;
        std::memcpy(&sourcePortNetworkOrder, udp, sizeof(sourcePortNetworkOrder));
        const quint16 sourcePort = qFromBigEndian(sourcePortNetworkOrder);

        quint32 sourceAddressNetworkOrder = 0;
        std::memcpy(&sourceAddressNetworkOrder,
                    ip + 12,
                    sizeof(sourceAddressNetworkOrder));
        const QHostAddress sourceAddress(
            qFromBigEndian(sourceAddressNetworkOrder));

        const int payloadLength = header->caplen - (14 + ihl + 8);
        if (payloadLength <= 0)
            continue;

        const QByteArray payload(
            reinterpret_cast<const char *>(udp + 8), payloadLength);
        // 保留旧信号供独立读取器使用；主程序使用带时间线代号的信号，
        // 从而能够拒绝切源/跳转后才到达 GUI 队列的旧包。
        emit udpPacket(sourceAddress, sourcePort, payload, currentPacketTime);
        emit udpPacketWithGeneration(
            sourceAddress, sourcePort, payload, currentPacketTime,
            request.timeline_generation);
    }

    qDebug() << "[PCAP回放] 读取循环结束";
}

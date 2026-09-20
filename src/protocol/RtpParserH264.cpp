#include "RtpParserH264.h"
#include <QDebug>

void RtpParserH264::resetStreamState(quint64 timelineGeneration)
{
    m_bindSsrc = 0;
    m_lastSeq = 0;
    m_isSsrcLocked = false;
    m_isWaitingForEnd = false;
    m_fuBuffer.clear();
    m_extraConfig.clear();
    m_fuGeneration = 0;
    if (timelineGeneration != 0)
        m_timelineGeneration.store(timelineGeneration, std::memory_order_release);
}

void RtpParserH264::inputPacket(const QByteArray &data, double timestamp) {
    inputPacketWithGeneration(
        data, timestamp,
        m_timelineGeneration.load(std::memory_order_acquire));
}

void RtpParserH264::inputPacketWithGeneration(const QByteArray &data,
                                              double timestamp,
                                              quint64 timelineGeneration) {
    if (timelineGeneration
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    // RTP Header 至少 12 字节，加上 1 字节 Payload Header
    if (data.size() < 13) return;

    const unsigned char* rtpRaw = reinterpret_cast<const unsigned char*>(data.data());

    // 1. 提取 RTP 头部信息 (用于解决你日志中的 Sequence 跳变问题)
    // 序列号在 2-3 字节，SSRC 在 8-11 字节
    quint16 currentSeq = qFromBigEndian<quint16>(*reinterpret_cast<const quint16*>(rtpRaw + 2));
    quint32 currentSsrc = qFromBigEndian<quint32>(*reinterpret_cast<const quint32*>(rtpRaw + 8));

    // 2. SSRC 锁定逻辑：确保 20082 端口只处理同一路视频流
    if (!m_isSsrcLocked) {
        m_bindSsrc = currentSsrc;
        m_isSsrcLocked = true;
        //qDebug() << parserName << "已锁定唯一源 SSRC:" << QString::number(m_bindSsrc, 16);
    }

    // 过滤掉混入该端口的其他流数据
    if (currentSsrc != m_bindSsrc) {
        return;
    }

    // 3. 丢包检测逻辑
    if (m_lastSeq != 0) {
        quint16 nextExpected = static_cast<quint16>(m_lastSeq + 1);
        if (currentSeq != nextExpected && !(m_lastSeq == 65535 && currentSeq == 0)) {
           // qDebug() << parserName << "网络真实丢包! Expected:" << nextExpected << "Got:" << currentSeq;
            m_fuBuffer.clear();
            m_isWaitingForEnd = false;
        }
    }
    m_lastSeq = currentSeq;

    // 4. 解析 RTP 负载内容
    const unsigned char* rtpPayload = rtpRaw + 12;
    int rtpPayloadSize = data.size() - 12;

    unsigned char indicator = rtpPayload[0];
    unsigned char nalType = indicator & 0x1F;
    const char startCode[] = {0x00, 0x00, 0x00, 0x01};

    // --- 情况 A: FU-A 分片包 (Type 28) ---
    if (nalType == 28) {
        unsigned char fuHeader = rtpPayload[1];
        bool startBit = (fuHeader & 0x80) != 0;
        bool endBit   = (fuHeader & 0x40) != 0;
        unsigned char trueNalType = fuHeader & 0x1F;

        if (startBit) {
            m_fuBuffer.clear();

            // 如果是关键帧 (Type 5)，注入缓存的 SPS/PPS
            if (trueNalType == 5 && !m_extraConfig.isEmpty()) {
                m_fuBuffer.append(m_extraConfig);
            }

            m_fuBuffer.append(startCode, 4);
            // 重构 NALU Header (F | NRI | Type)
            unsigned char reconstructedHeader = (indicator & 0xE0) | trueNalType;
            m_fuBuffer.append(static_cast<char>(reconstructedHeader));
            m_fuBuffer.append(reinterpret_cast<const char*>(rtpPayload + 2), rtpPayloadSize - 2);
            m_isWaitingForEnd = true;
            m_fuGeneration = timelineGeneration;
        }
        else if (m_isWaitingForEnd && m_fuGeneration == timelineGeneration) {
            m_fuBuffer.append(reinterpret_cast<const char*>(rtpPayload + 2), rtpPayloadSize - 2);
            if (endBit) {
                emit rtpPayloadReady(m_fuBuffer, timestamp);
                emit rtpPayloadReadyWithGeneration(
                    m_fuBuffer, timestamp, m_fuGeneration);
                m_isWaitingForEnd = false;
                m_fuBuffer.clear();
                m_fuGeneration = 0;
            }
        }
    }
    // --- 情况 B: 单个 NAL 包 (Type 1-23) ---
    else if (nalType >= 1 && nalType <= 23) {
        QByteArray singleUnit;

        // 如果是 P 帧或 I 帧，且存在备份参数，则拼上去
        if ((nalType == 1 || nalType == 5) && !m_extraConfig.isEmpty()) {
            singleUnit.append(m_extraConfig);
        }

        singleUnit.append(startCode, 4);
        singleUnit.append(reinterpret_cast<const char*>(rtpPayload), rtpPayloadSize);

        emit rtpPayloadReady(singleUnit, timestamp);
        emit rtpPayloadReadyWithGeneration(
            singleUnit, timestamp, timelineGeneration);
        m_isWaitingForEnd = false;
    }
    // --- 情况 C: STAP-A 组合包 (Type 24) ---
    else if (nalType == 24) {
        int offset = 1;
        while (offset < rtpPayloadSize - 2) {
            unsigned short subSize = (static_cast<unsigned char>(rtpPayload[offset]) << 8) |
                                      static_cast<unsigned char>(rtpPayload[offset + 1]);
            offset += 2;
            if (offset + subSize <= rtpPayloadSize) {
                QByteArray subUnit;
                subUnit.append(startCode, 4);
                subUnit.append(reinterpret_cast<const char*>(rtpPayload + offset), subSize);
                emit rtpPayloadReady(subUnit, timestamp);
                emit rtpPayloadReadyWithGeneration(
                    subUnit, timestamp, timelineGeneration);
                offset += subSize;
            } else break;
        }
        m_isWaitingForEnd = false;
    }
}

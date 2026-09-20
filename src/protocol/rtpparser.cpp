#include "rtpparser.h"
#include <QtEndian>
#include <QDebug>

void RtpParser::resetStreamState(quint64 timelineGeneration)
{
    m_bindSsrc = 0;
    m_lastSeq = 0;
    m_isSsrcLocked = false;
    fuBuffer.clear();
    fuStarted = false;
    m_fuGeneration = 0;
    if (timelineGeneration != 0)
        m_timelineGeneration.store(timelineGeneration, std::memory_order_release);
}

bool RtpParser::acceptRtpStream(const QByteArray &packet)
{
    if (packet.size() < 12)
        return false;

    const auto *rtpRaw = reinterpret_cast<const unsigned char *>(packet.constData());
    const quint8 version = (rtpRaw[0] >> 6) & 0x03;
    if (version != 2)
        return false;

    const quint16 currentSeq = qFromBigEndian<quint16>(
        *reinterpret_cast<const quint16 *>(rtpRaw + 2));
    const quint32 currentSsrc = qFromBigEndian<quint32>(
        *reinterpret_cast<const quint32 *>(rtpRaw + 8));

    if (!m_isSsrcLocked) {
        m_bindSsrc = currentSsrc;
        m_isSsrcLocked = true;
    }

    if (currentSsrc != m_bindSsrc)
        return false;

    if (m_lastSeq != 0) {
        const quint16 nextExpected = static_cast<quint16>(m_lastSeq + 1);
        if (currentSeq != nextExpected && !(m_lastSeq == 65535 && currentSeq == 0)) {
            fuBuffer.clear();
            fuStarted = false;
        }
    }
    m_lastSeq = currentSeq;
    return true;
}

void RtpParser::inputPacket(const QByteArray &packet, double timestamp)
{
    inputPacketWithGeneration(
        packet, timestamp,
        m_timelineGeneration.load(std::memory_order_acquire));
}

void RtpParser::inputPacketWithGeneration(const QByteArray &packet,
                                          double timestamp,
                                          quint64 timelineGeneration)
{
    if (timelineGeneration
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    if (!acceptRtpStream(packet))
        return;

    const uchar* data = (const uchar*)packet.data();

    quint8 cc = data[0] & 0x0F;
    int headerSize = 12 + cc * 4;

    if (packet.size() <= headerSize) return;

    QByteArray payload = packet.mid(headerSize);

    // ===== H265 =====
    quint8 nalType = (payload[0] >> 1) & 0x3F;

    // ✅ 单NAL（直接输出）
    if (nalType >= 0 && nalType <= 40) {
        emit rtpPayloadReady(payload,timestamp);
        emit rtpPayloadReadyWithGeneration(payload, timestamp, timelineGeneration);
        return;
    }

    // =========================
    // 🔥 FU 分片（重点）
    // =========================
    if (nalType == 49) {

        quint8 fuHeader = payload[2];

        bool start = fuHeader & 0x80;
        bool end   = fuHeader & 0x40;

        quint8 originalNalType = fuHeader & 0x3F;

        if (start) {
            fuBuffer.clear();

            // 重建 NAL Header（2字节）
            quint8 header0 = (payload[0] & 0x81) | (originalNalType << 1);
            quint8 header1 = payload[1];

            fuBuffer.append(header0);
            fuBuffer.append(header1);

            fuBuffer.append(payload.mid(3));

            fuStarted = true;
            m_fuGeneration = timelineGeneration;
        }
        else if (fuStarted && m_fuGeneration == timelineGeneration) {
            fuBuffer.append(payload.mid(3));
        }

        if (end && fuStarted) {
            emit rtpPayloadReady(fuBuffer,timestamp);
            emit rtpPayloadReadyWithGeneration(
                fuBuffer, timestamp, m_fuGeneration);
            fuBuffer.clear();
            fuStarted = false;
            m_fuGeneration = 0;
        }

        return;
    }

    // =========================
    // STAP-A（简单支持）
    // =========================
    if (nalType == 48) {
        int offset = 2;

        while (offset + 2 < payload.size()) {
            quint16 size = qFromBigEndian<quint16>((uchar*)payload.data() + offset);
            offset += 2;

            if (offset + size > payload.size()) break;

            QByteArray nal = payload.mid(offset, size);
            emit rtpPayloadReady(nal,timestamp);
            emit rtpPayloadReadyWithGeneration(nal, timestamp, timelineGeneration);

            offset += size;
        }
    }
}

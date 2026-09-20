#include "IMUParser.h"
#include "NaviProtocol.h"

void IMUParser::inputPacket(const QByteArray &packet, double timestamp)
{
    inputPacketWithGeneration(
        packet, timestamp,
        m_timelineGeneration.load(std::memory_order_acquire));
}

void IMUParser::inputPacketWithGeneration(const QByteArray &packet,
                                          double timestamp,
                                          quint64 timelineGeneration)
{
    if (timelineGeneration
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    // 1. 长度基本校验 (Header 16 + Payload 80)
    if (packet.size() < 96) return;

    // 2. 检查信息单元标识 (在偏移量 17 的位置)
    const char* data = packet.data();
    uint8_t iuid = static_cast<uint8_t>(data[17]);

    if (iuid == 0xB0) {
        // 3. 提取 80 字节的原始 Payload 块
        QByteArray rawPayload = packet.mid(16, 80);

        // 发送给 Worker 进行后处理
        //emit rawPayloadReady(rawPayload);

        emit navPayloadReady(rawPayload, timestamp);
        emit navPayloadReadyWithGeneration(
            rawPayload, timestamp, timelineGeneration);
        emit rtpPayloadReady(rawPayload);
    }
}

void IMUParser::resetTimeline(quint64 timelineGeneration)
{
    m_timelineGeneration.store(timelineGeneration, std::memory_order_release);
}

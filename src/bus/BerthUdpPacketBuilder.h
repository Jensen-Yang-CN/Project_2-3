#pragma once

#include "PerceptionUdpProtocol.h"
#include "common_types_extended.h"

#include <QByteArray>

#include <array>
#include <cstdint>
#include <vector>

namespace berth_udp {

struct BerthProtocolPoint {
    double longitude_deg = 0.0;
    double latitude_deg = 0.0;
    double altitude_m = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
};

/**
 * 根据已经按协议顺序排列的四个点创建主机字节序泊位单元。
 * 点 1、4 位于开口侧，点 2、3 位于泊位内部侧。
 */
BerthUdpUnit makeBerthUnit(
    uint8_t type,
    const usv::Berth &berth,
    const std::array<BerthProtocolPoint, 4> &points);

/**
 * 将一批泊位分成每包最多 10 个的 UDP 数据报。
 * 协议的 packet_count 为 uint8，因此最多实际发送 2550 个泊位。
 */
std::vector<QByteArray> buildBerthPackets(
    int32_t timestamp,
    uint16_t frame_id,
    const std::vector<BerthUdpUnit> &host_units);

}  // namespace berth_udp

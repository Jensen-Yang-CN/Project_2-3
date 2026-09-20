#include "BerthUdpPacketBuilder.h"
#include "PerceptionUdpProtocol.h"

#include <QtEndian>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
    if (condition)
        return;
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
}

BerthUdpUnit sampleUnit(int index = 0)
{
    usv::Berth berth;
    berth.opening_edge = 0;
    berth.edge_heights = {99.0, 2.0, 6.0, 4.0};
    berth.edge_height_valid = {true, true, true, true};

    std::array<berth_udp::BerthProtocolPoint, 4> points{};
    for (int i = 0; i < 4; ++i) {
        points[i].longitude_deg = 119.5 + index * 1e-5 + i * 1e-6;
        points[i].latitude_deg = 35.4 + index * 1e-5 + i * 1e-6;
        points[i].altitude_m = 1.0 + i;
        points[i].x_m = 1.2 + index + i;
        points[i].y_m = -2.3 - index - i;
        points[i].z_m = 0.4 + i;
    }
    return berth_udp::makeBerthUnit(
        BerthProtocolType::kGroove, berth, points);
}

const BerthUdpHeader *headerOf(const QByteArray &packet)
{
    return reinterpret_cast<const BerthUdpHeader *>(packet.constData());
}

void testWireSizesAndSinglePacket()
{
    static_assert(sizeof(BerthUdpHeader) == 20,
                  "V2 berth header must be 20 bytes");
    static_assert(sizeof(BerthUdpUnit) == 72,
                  "V2 berth unit must be 72 bytes");

    const std::vector<QByteArray> packets =
        berth_udp::buildBerthPackets(123456, 7, {sampleUnit()});
    check(packets.size() == 1, "one berth must produce one packet");
    if (packets.empty())
        return;

    const QByteArray &packet = packets.front();
    check(packet.size() == 92, "one berth packet length must be 92 bytes");
    const BerthUdpHeader *header = headerOf(packet);
    check(qFromBigEndian(header->total_len) == 92,
          "total_len must use network byte order");
    check(qFromBigEndian(header->timestamp) == 123456,
          "timestamp must use network byte order");
    check(qFromBigEndian(header->frame_id) == 7,
          "frame_id must use network byte order");
    check(qFromBigEndian(header->total_count) == 1,
          "total_count must describe the whole batch");
    check(header->berth_num == 1, "berth_num must describe this packet");
    check(header->packet_index == 0, "first packet index must be zero");
    check(header->packet_count == 1, "single packet batch count must be one");

    const auto *unit = reinterpret_cast<const BerthUdpUnit *>(
        packet.constData() + sizeof(BerthUdpHeader));
    check(unit->deck_height == 40,
          "deck height must be median non-opening edge height in decimeters");
    check(unit->clear_height == 0 && unit->clear_depth == 0
              && unit->reserve4 == 0,
          "unknown clearance fields and reserve4 must be zero");
    check(qFromBigEndian(unit->x1) == 12,
          "x1 must encode the supplied LiDAR-local coordinate");
    check(qFromBigEndian(unit->y1) == -23,
          "y1 must encode the supplied LiDAR-local coordinate");
    check(qFromBigEndian(unit->z1) == 4,
          "z1 must encode the supplied LiDAR-local coordinate");
}

void testTenAndElevenBerthSplitting()
{
    std::vector<BerthUdpUnit> ten;
    for (int i = 0; i < 10; ++i)
        ten.push_back(sampleUnit(i));
    const auto onePacket = berth_udp::buildBerthPackets(200, 9, ten);
    check(onePacket.size() == 1, "ten berths must fit in one packet");
    if (!onePacket.empty())
        check(onePacket.front().size() == 740,
              "ten berth packet length must be 740 bytes");

    ten.push_back(sampleUnit(10));
    const auto twoPackets = berth_udp::buildBerthPackets(201, 10, ten);
    check(twoPackets.size() == 2, "eleven berths must produce two packets");
    if (twoPackets.size() != 2)
        return;

    const BerthUdpHeader *first = headerOf(twoPackets[0]);
    const BerthUdpHeader *second = headerOf(twoPackets[1]);
    check(twoPackets[0].size() == 740 && twoPackets[1].size() == 92,
          "11-berth packet sizes must be 740 and 92");
    check(first->berth_num == 10 && second->berth_num == 1,
          "split packet berth counts must be 10 and 1");
    check(first->packet_index == 0 && second->packet_index == 1,
          "packet indexes must increase from zero");
    check(first->packet_count == 2 && second->packet_count == 2,
          "all packets must share packet_count");
    check(qFromBigEndian(first->timestamp) == qFromBigEndian(second->timestamp)
              && qFromBigEndian(first->frame_id) == qFromBigEndian(second->frame_id)
              && qFromBigEndian(first->total_count) == qFromBigEndian(second->total_count),
          "all split packets must share batch metadata");
}

void testBatchLimit()
{
    std::vector<BerthUdpUnit> units(2551, sampleUnit());
    const auto packets = berth_udp::buildBerthPackets(300, 11, units);
    check(packets.size() == 255,
          "packet_count byte limits a batch to 255 packets");
    if (!packets.empty())
        check(qFromBigEndian(headerOf(packets.front())->total_count) == 2550,
              "total_count must report the actually transmitted 2550 berths");
}

}  // namespace

int main()
{
    testWireSizesAndSinglePacket();
    testTenAndElevenBerthSplitting();
    testBatchLimit();
    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "berth UDP protocol tests passed\n";
    return EXIT_SUCCESS;
}

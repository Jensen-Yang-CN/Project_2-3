#include "BerthUdpPacketBuilder.h"

#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace berth_udp {
namespace {

int32_t encodeGeo(double degrees)
{
    const long double scaled =
        static_cast<long double>(degrees) * PerceptionUdpConstants::kGeoScale;
    return static_cast<int32_t>(std::clamp(
        std::llround(scaled),
        static_cast<long long>(std::numeric_limits<int32_t>::min()),
        static_cast<long long>(std::numeric_limits<int32_t>::max())));
}

int16_t encodeSignedTenths(double meters)
{
    return static_cast<int16_t>(std::clamp(
        std::llround(meters * 10.0),
        static_cast<long long>(std::numeric_limits<int16_t>::min()),
        static_cast<long long>(std::numeric_limits<int16_t>::max())));
}

uint8_t encodeUnsignedTenths(double meters)
{
    return static_cast<uint8_t>(std::clamp(
        std::llround(meters * 10.0), 0LL, 255LL));
}

uint8_t deckHeight(const usv::Berth &berth)
{
    std::vector<double> valid_heights;
    valid_heights.reserve(3);
    for (int edge = 0; edge < 4; ++edge) {
        if (edge == berth.opening_edge || !berth.edge_height_valid[edge])
            continue;
        const double height = berth.edge_heights[edge];
        if (std::isfinite(height))
            valid_heights.push_back(height);
    }
    if (valid_heights.empty())
        return 0;

    std::sort(valid_heights.begin(), valid_heights.end());
    const std::size_t middle = valid_heights.size() / 2;
    double median = valid_heights[middle];
    if ((valid_heights.size() % 2) == 0)
        median = 0.5 * (valid_heights[middle - 1] + valid_heights[middle]);
    return encodeUnsignedTenths(median);
}

void fillPoint(BerthUdpUnit &unit, int index, const BerthProtocolPoint &point)
{
    int32_t *longitudes[] = {
        &unit.longitude1, &unit.longitude2, &unit.longitude3, &unit.longitude4};
    int32_t *latitudes[] = {
        &unit.latitude1, &unit.latitude2, &unit.latitude3, &unit.latitude4};
    int16_t *altitudes[] = {
        &unit.altitude1, &unit.altitude2, &unit.altitude3, &unit.altitude4};
    int16_t *xs[] = {&unit.x1, &unit.x2, &unit.x3, &unit.x4};
    int16_t *ys[] = {&unit.y1, &unit.y2, &unit.y3, &unit.y4};
    int16_t *zs[] = {&unit.z1, &unit.z2, &unit.z3, &unit.z4};

    *longitudes[index] = encodeGeo(point.longitude_deg);
    *latitudes[index] = encodeGeo(point.latitude_deg);
    *altitudes[index] = encodeSignedTenths(point.altitude_m);
    *xs[index] = encodeSignedTenths(point.x_m);
    *ys[index] = encodeSignedTenths(point.y_m);
    *zs[index] = encodeSignedTenths(point.z_m);
}

BerthUdpUnit toNetworkOrder(const BerthUdpUnit &host)
{
    BerthUdpUnit network = host;
#define BERTH_TO_BIG_ENDIAN(field) network.field = qToBigEndian(host.field)
    BERTH_TO_BIG_ENDIAN(longitude1);
    BERTH_TO_BIG_ENDIAN(latitude1);
    BERTH_TO_BIG_ENDIAN(altitude1);
    BERTH_TO_BIG_ENDIAN(x1);
    BERTH_TO_BIG_ENDIAN(y1);
    BERTH_TO_BIG_ENDIAN(z1);
    BERTH_TO_BIG_ENDIAN(longitude2);
    BERTH_TO_BIG_ENDIAN(latitude2);
    BERTH_TO_BIG_ENDIAN(altitude2);
    BERTH_TO_BIG_ENDIAN(x2);
    BERTH_TO_BIG_ENDIAN(y2);
    BERTH_TO_BIG_ENDIAN(z2);
    BERTH_TO_BIG_ENDIAN(longitude3);
    BERTH_TO_BIG_ENDIAN(latitude3);
    BERTH_TO_BIG_ENDIAN(altitude3);
    BERTH_TO_BIG_ENDIAN(x3);
    BERTH_TO_BIG_ENDIAN(y3);
    BERTH_TO_BIG_ENDIAN(z3);
    BERTH_TO_BIG_ENDIAN(longitude4);
    BERTH_TO_BIG_ENDIAN(latitude4);
    BERTH_TO_BIG_ENDIAN(altitude4);
    BERTH_TO_BIG_ENDIAN(x4);
    BERTH_TO_BIG_ENDIAN(y4);
    BERTH_TO_BIG_ENDIAN(z4);
#undef BERTH_TO_BIG_ENDIAN
    return network;
}

}  // namespace

BerthUdpUnit makeBerthUnit(
    uint8_t type,
    const usv::Berth &berth,
    const std::array<BerthProtocolPoint, 4> &points)
{
    BerthUdpUnit unit{};
    unit.type = type;
    unit.deck_height = deckHeight(berth);
    unit.clear_height = 0;
    unit.clear_depth = 0;
    unit.reserve4 = 0;
    for (int i = 0; i < 4; ++i)
        fillPoint(unit, i, points[static_cast<std::size_t>(i)]);
    return unit;
}

std::vector<QByteArray> buildBerthPackets(
    int32_t timestamp,
    uint16_t frame_id,
    const std::vector<BerthUdpUnit> &host_units)
{
    const std::size_t total_count = std::min(
        host_units.size(), PerceptionUdpConstants::kMaxBerthsPerBatch);
    if (total_count == 0)
        return {};

    const std::size_t packet_count =
        (total_count + PerceptionUdpConstants::kMaxBerthsPerPacket - 1)
        / PerceptionUdpConstants::kMaxBerthsPerPacket;
    std::vector<QByteArray> packets;
    packets.reserve(packet_count);

    for (std::size_t packet_index = 0;
         packet_index < packet_count;
         ++packet_index) {
        const std::size_t first =
            packet_index * PerceptionUdpConstants::kMaxBerthsPerPacket;
        const std::size_t berth_count = std::min(
            PerceptionUdpConstants::kMaxBerthsPerPacket, total_count - first);
        const std::size_t packet_size =
            sizeof(BerthUdpHeader) + berth_count * sizeof(BerthUdpUnit);

        QByteArray packet(static_cast<int>(packet_size), '\0');
        auto *header = reinterpret_cast<BerthUdpHeader *>(packet.data());
        std::memcpy(
            header->prefix, PerceptionUdpConstants::kPrefix,
            sizeof(header->prefix));
        header->total_len = qToBigEndian(static_cast<uint32_t>(packet_size));
        header->timestamp = qToBigEndian(timestamp);
        header->frame_id = qToBigEndian(frame_id);
        header->total_count =
            qToBigEndian(static_cast<uint16_t>(total_count));
        header->id = PerceptionUdpConstants::kBerthId;
        header->berth_num = static_cast<uint8_t>(berth_count);
        header->packet_index = static_cast<uint8_t>(packet_index);
        header->packet_count = static_cast<uint8_t>(packet_count);

        auto *wire_units = reinterpret_cast<BerthUdpUnit *>(
            packet.data() + sizeof(BerthUdpHeader));
        for (std::size_t i = 0; i < berth_count; ++i)
            wire_units[i] = toNetworkOrder(host_units[first + i]);
        packets.push_back(std::move(packet));
    }
    return packets;
}

}  // namespace berth_udp

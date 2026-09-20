#pragma once

#include <cstddef>
#include <cstdint>

#pragma pack(push, 1)

/** 泊位 UDP 协议 0xFB，网络字节序 */
struct BerthUdpHeader {
    uint8_t prefix[4];   // A1 B2 B2 A1
    uint32_t total_len;  // 20 + berth_num * 72
    int32_t timestamp;
    uint16_t frame_id;     // 同一批次保持一致
    uint16_t total_count;  // 本批次实际发送的泊位总数
    uint8_t id;            // 0xFB
    uint8_t berth_num;     // 当前包泊位数量，最大 10
    uint8_t packet_index;  // 从 0 开始
    uint8_t packet_count;  // 本批次总包数
};

struct BerthUdpUnit {
    uint8_t type;      // 1=一字型 2=凹槽型（见 BerthProtocolType）
    uint8_t reserve1;  // 0
    uint8_t reserve2;
    uint8_t reserve3;
    uint8_t deck_height;   // 泊位面高度（米 * 10）
    uint8_t clear_height;  // 净空高度（米 * 10），未知时为 0
    uint8_t clear_depth;   // 净空深度（米 * 10），未知时为 0
    uint8_t reserve4;      // 0
    int32_t longitude1;
    int32_t latitude1;
    int16_t altitude1;
    int16_t x1;
    int16_t y1;
    int16_t z1;
    int32_t longitude2;
    int32_t latitude2;
    int16_t altitude2;
    int16_t x2;
    int16_t y2;
    int16_t z2;
    int32_t longitude3;
    int32_t latitude3;
    int16_t altitude3;
    int16_t x3;
    int16_t y3;
    int16_t z3;
    int32_t longitude4;
    int32_t latitude4;
    int16_t altitude4;
    int16_t x4;
    int16_t y4;
    int16_t z4;
};

/** 栅格地图 UDP 协议 0xFC，固定 1316 字节 */
struct GridUdpPacket {
    uint8_t prefix[4];
    uint16_t total_len;   // 1316
    uint8_t id;           // 0xFC
    uint8_t reserve1;
    uint32_t sequence_id;
    int32_t longitude;
    int32_t latitude;
    int16_t altitude;
    uint16_t azimuth;
    int16_t chunk_x;
    int16_t chunk_y;
    int16_t chunk_z;
    uint16_t resolution;  // 米 * 100
    uint8_t chunk_x_len; // 32
    uint8_t chunk_y_len; // 32
    uint8_t chunk_z_len; // 10
    uint8_t reserve2;
    uint8_t grid_bit[1280];
};

#pragma pack(pop)

static_assert(sizeof(BerthUdpHeader) == 20, "BerthUdpHeader must be 20 bytes");
static_assert(sizeof(BerthUdpUnit) == 72, "BerthUdpUnit must be 72 bytes");
static_assert(sizeof(GridUdpPacket) == 1316, "GridUdpPacket must be 1316 bytes");

namespace PerceptionUdpConstants {
constexpr uint8_t kPrefix[4] = {0xA1, 0xB2, 0xB2, 0xA1};
constexpr uint8_t kBerthId = 0xFB;
constexpr uint8_t kGridId = 0xFC;
constexpr double kGeoScale = 11930464.711111111;
constexpr double kAltScale = 10.0;
constexpr double kAzimuthScale = 182.04444444444444;
constexpr double kLidarScale = 10.0;
constexpr uint16_t kGridPacketLen = 1316;
constexpr int kChunkXLen = 32;
constexpr int kChunkYLen = 32;
constexpr int kChunkZLen = 10;
constexpr double kDefaultResolutionM = 0.1;
constexpr std::size_t kMaxBerthsPerPacket = 10;
constexpr std::size_t kMaxBerthPacketCount = 255;
constexpr std::size_t kMaxBerthsPerBatch =
    kMaxBerthsPerPacket * kMaxBerthPacketCount;
}  // namespace PerceptionUdpConstants

/** 0xFB 泊位 type 字段（bowei2.h）：与内部 BerthKind 枚举值不同 */
namespace BerthProtocolType {
constexpr uint8_t kLine = 1;    // 一字型：点1/4开口侧，点2/3泊位线
constexpr uint8_t kGroove = 2;  // 凹槽型：点1/4开口顶点，点2/3内部顶点
}  // namespace BerthProtocolType

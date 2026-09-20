#ifndef LIDARPACKETSTRUCT_H
#define LIDARPACKETSTRUCT_H


#include <cstdint>

#pragma pack(push,1)

struct LidarUnit {
    uint16_t distance; // 距离单位通常是 2mm
    uint8_t intensity; // 反射强度
};

struct LidarBlock {
    uint16_t header;       // 应为 0xFFEE
    uint16_t azimuth;      // 当前块的起始方位角 (0-35999, 单位 0.01°)
    LidarUnit units[32];   // 32个激光通道的数据
};


struct LidarPacket {
    LidarBlock blocks[12]; // 12 * 100 = 1200 bytes
    uint32_t timestamp;    // 4 bytes
    uint16_t factory;      // 2 bytes
    uint8_t reserved[42];  // 补充剩余的 42 字节，使总大小等于 1248
};

struct LASHeader {
    char signature[4] = {'L', 'A', 'S', 'F'};
    uint16_t sourceId = 0;
    uint16_t globalEncoding = 0;
    uint32_t guid1 = 0; uint16_t guid2 = 0; uint16_t guid3 = 0; uint8_t guid4[8] = {0};
    uint8_t versionMajor = 1; uint8_t versionMinor = 2;
    char systemIdentifier[32] = "Qt_Lidar_System";
    char generatingSoftware[32] = "LS400_Parser";
    uint16_t creationDay = 1; uint16_t creationYear = 2026;
    uint16_t headerSize = 227;
    uint32_t offsetToPoints = 227;
    uint32_t vlrCount = 0;
    uint8_t pointFormat = 0; // 格式0，每个点20字节
    uint16_t pointRecordLength = 20;
    uint32_t pointCount = 0;
    uint32_t pointsByReturn[5] = {0};
    double scaleX = 0.001, scaleY = 0.001, scaleZ = 0.001; // 毫米级精度
    double offsetX = 0, offsetY = 0, offsetZ = 0;
    double maxX = 0, minX = 0, maxY = 0, minY = 0, maxZ = 0, minZ = 0;
};

struct LASPoint0 {
    int32_t x, y, z;          // 原始坐标 / scale
    uint16_t intensity;       // 强度
    uint8_t returnInfo = 1;   // 第1回波
    uint8_t classification = 0;
    int8_t scanAngle = 0;
    uint8_t userData = 0;
    uint16_t pointSourceId = 0;
};

#pragma pack(pop)

#endif // LIDARPACKETSTRUCT_H







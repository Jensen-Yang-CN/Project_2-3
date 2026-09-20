#ifndef NAVIPROTOCOL_H
#define NAVIPROTOCOL_H

#include <QtGlobal>

#pragma pack(push, 1) // 必须强制1字节对齐

// 原始信息单元 (80字节，严格对应硬件报文)
struct NaviRawPayload {
    uint8_t  infoSeq;
    uint8_t  cIUID;
    uint16_t wIULength;
    uint16_t stateWord;
    uint16_t save;
    uint32_t yaw;
    int32_t  pitch;
    int32_t  roll;
    int32_t  yawRate;
    int32_t  pitchRate;
    int32_t  rollRate;
    uint32_t timestamp;
    int32_t  longitude;
    int32_t  latitude;
    int32_t  x_Rate;     // 载体系Vx
    int32_t  y_Rate;     // 载体系Vy
    int32_t  z_Rate;     // 载体系Vz
    int32_t  x_RateAcc;
    int32_t  y_RateAcc;
    int32_t  z_RateAcc;
    int32_t  eastRate;
    int32_t  northRate;
    int32_t  skyRate;
};
#pragma pack(pop)

// 解析后的物理量结构 (供UI使用)
struct IMUParsedData {
    int    naviState;
    bool   yawValid, pitchValid, rollValid;
    double yaw, pitch, roll;
    double yawRate, pitchRate, rollRate;
    double timestamp;
    double longitude, latitude;
    double vx, vy, vz;
    double ax, ay, az;
    double vEast, vNorth, vSky;
    quint64 timelineGeneration = 1;
};

namespace NaviFactors {
    const double ANGLE     = 180.0 / 1073741824.0; // 2^30
    const double LL        = 180.0 / 2147483648.0; // 2^31
    const double ANG_RATE  = 0.00001;
    const double TIMESTAMP = 0.0001;
    const double METRIC    = 0.001; // mm -> m
}

#endif

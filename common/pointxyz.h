#ifndef POINTXYZ_H
#define POINTXYZ_H

#include <QtGlobal>


#pragma pack(push, 1)
struct M_PointXYZI {
    float x;
    float y;
    float z;
    uint8_t intensity;
    uint8_t padding[3]; // 补齐到 16 字节，确保内存对齐
};
#pragma pack(pop)

template<typename T>
struct TimedFrame {
    double timestamp = 0.0;
    quint64 timeline_generation = 1;
    T data;
};


#endif // POINTXYZ_H

#ifndef DEVICECONFIG_H
#define DEVICECONFIG_H
#include <QString>
#include <QVariantMap>
#include <QObject>

// 1. 使用强类型枚举，并指定底层为 uint8_t 节省空间
namespace device {
enum class DeviceType : uint8_t {
    Lidar_RS = 0,    // 速腾聚创
    Lidar_LS = 1,    // 镭神
    IMU = 2,         // 惯导
    Camera_H264 = 3, // 红外相机等
    Camera_H265 = 4  // RGB相机等
};

// 2. 设备信息结构体
struct DeviceInfo {
    QString name;          // 设备唯一标识，如 "Front_Lidar"
    DeviceType type;       // 设备类型
    QString ip;            // IP 地址
    int port;              // 主数据端口
    int extraPort;         // 备用端口 (如 RS 雷达的 7788 或 2368)
    // 关键：metadata 用于存储不通用的特殊配置 // 比如：
    // Lidar -> "transform_matrix": QVariant(QMatrix4x4)
    // Camera -> "rtsp_url": "rtsp://..."
    QVariantMap metadata;
};

inline const char* toString(DeviceType type)
{
    switch(type)
    {
    case DeviceType::Lidar_RS: return "Lidar_RS";
    case DeviceType::Lidar_LS: return "Lidar_LS";
    case DeviceType::IMU: return "IMU";
    case DeviceType::Camera_H264: return "Camera_H264";
    case DeviceType::Camera_H265: return "Camera_H265";
    default: return "Unknown";
    }
}
inline DeviceType fromString(const QString& str) {
    if (str == "Lidar_RS")    return DeviceType::Lidar_RS;
    if (str == "Lidar_LS")    return DeviceType::Lidar_LS;
    if (str == "IMU")         return DeviceType::IMU;
    if (str == "Camera_H264") return DeviceType::Camera_H264;
    if (str == "Camera_H265") return DeviceType::Camera_H265;
    return DeviceType::Lidar_RS; // 默认值
}

}

#endif // DEVICECONFIG_H

/*name 的作用： 在你的 RadarFusionManager 中，
 * 你之前是用 "201"、"213" 这样的字符串来区分雷达。
 * 现在你可以直接用 info.name 作为 Key，逻辑更清晰。
 * metadata 的灵活性（解决 Eigen 矩阵存储）：
 * 你之前提到使用 Eigen 进行 5 个雷达的坐标变换。
 * 不同的雷达安装在船体不同位置，变换矩阵是不一样的。
 * 你可以将每个雷达的旋转和平移参数存进 metadata，
 * 这样在 JSON 中读取后，直接传给 RadarFusionManager。
 * 兼容性： QVariantMap 可以无缝转换为 JSON 对象，这为你后续实现“读取 JSON 注册设备”铺平了道路。
 */

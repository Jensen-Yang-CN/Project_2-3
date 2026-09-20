#ifndef RSLIDARPARSER_H
#define RSLIDARPARSER_H

#include <QObject>
#include <QVector>
#include <QHostAddress>
#include <atomic>
#include <thread>
#include "common/pointxyz.h"
#include "common/types.h"

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

typedef ::PointCloudT<::PointXYZI> PointCloudMsg;

class RsLidarParser : public QObject
{
    Q_OBJECT
public:
    // 接口绝对不动：保持原有构造函数签名
    explicit RsLidarParser(const QHostAddress &ip, quint16 port, QObject *parent = nullptr);
    ~RsLidarParser();

    // --- 新增设置器 (Setters)：通过这些函数传递初始化值 ---
    void setDifopPort(quint16 p) { m_difopPort = p; }
    void setIsCustomDevice(bool b) { m_isCustomDevice = b; }

    // 参数设置完成后，显式调用此函数启动驱动
    bool start();

public slots:
    void inputPacket(const QByteArray& packet, double timestamp);
    void inputPacketWithGeneration(const QByteArray& packet, double timestamp,
                                   quint64 timelineGeneration);
    void requestPlaybackTimelineReset(quint64 timelineGeneration = 0);

signals:
    void frameReady(TimedFrame<QVector<M_PointXYZI>> frame);

private:
    struct QueuedCloud {
        std::shared_ptr<PointCloudMsg> cloud;
        double pcap_timestamp = 0.0;
        quint64 timeline_generation = 0;
    };

    // 定义在这里：用来存放当前正在处理的 PCAP 包的原始时间戳
    std::atomic<double> m_currentPcapTimestamp{0.0};
    void manualDecode(const QByteArray& packet, double timestamp);
    std::shared_ptr<PointCloudMsg> driverGetCallback();
    void exceptionCallback(const robosense::lidar::Error& code);
    void driverReturnCallback(const std::shared_ptr<PointCloudMsg>& msg);

    void driverPutCallback(std::shared_ptr<PointCloudMsg> msg); // 供驱动还数据 (改名为这个更符合逻辑)
    void processCloudLoop();
    TimedFrame<QVector<M_PointXYZI>> convertToYourFormat(const std::shared_ptr<PointCloudMsg>& msg);

private:
    robosense::lidar::LidarDriver<PointCloudMsg> m_driver;
    robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_queue;
    robosense::lidar::SyncQueue<QueuedCloud> stuffed_queue;

    std::atomic<bool> m_stop{false};
    std::thread m_processThread;
    std::atomic<bool> m_isReady{false};
    std::atomic<quint64> m_timelineGeneration{1};
    std::atomic<quint64> m_currentPacketGeneration{1};

    QHostAddress m_ip;
    quint16 m_port;         // 构造函数传入的端口
    quint16 m_msopPort;     // m_msopPort端口
    quint16 m_difopPort;     // 默认标准端口

    bool m_isCustomDevice = false;  // 默认非特殊设备
    TimedFrame<QVector<M_PointXYZI>> m_cacheFrame;
    int m_packetCounter = 0; // 用于计数的变量也建议声明在这里

    // 时间戳算法相关的状态变量
    bool is_first_frame_of_file_ = true;
    double file_start_unix_time_ = 0.0;
    double last_frame_lidar_time_ = 0.0;
    double time_offset_accumulator_ = 0.0;

    bool isFirst = false;

    double m_currentTimestamp; // 暂存 inputPacket 传入的时间戳供回调使用

public:
    // --- 性能调优常量区 ---
    // Helios 32线单帧点数理论最大值约为 64,000，取 70,000 留有余量
    static constexpr size_t MAX_POINTS_PER_FRAME = 70000;

    // 预充对象数量。20个可以缓冲约 1 秒的数据（针对 20Hz 雷达）
    static constexpr int INITIAL_BUFFER_COUNT = 50;

};

#endif

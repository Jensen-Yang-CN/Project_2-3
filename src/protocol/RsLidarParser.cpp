#include "RsLidarParser.h"
#include <QDebug>
#include <chrono>

using namespace robosense::lidar;

RsLidarParser::RsLidarParser(const QHostAddress &ip, quint16 port, QObject *parent)
    : QObject(parent),
      m_ip(ip),
      m_port(port),
      m_msopPort(port), // 默认将传入的端口作为 MSOP 端口
      m_difopPort(7788) // 默认 DIFOP 端口
{
    // 预充 Buffer 队列，避免在解码高频运行时频繁 new 内存
    for (int i = 0; i < INITIAL_BUFFER_COUNT; ++i) {
        free_queue.push(std::make_shared<PointCloudMsg>());
    }
}

RsLidarParser::~RsLidarParser()
{
    m_stop = true;
    if (m_processThread.joinable()) {
        m_processThread.join();
    }

    if (m_isReady) {
        m_driver.stop();
    }
}

bool RsLidarParser::start()
{
    if (m_isReady) return true;

    RSDriverParam param;

    // 【核心改变 1】：使用 RAW_PACKET 模式，接受外部 feed 数据
    param.input_type = InputType::RAW_PACKET;

    // 【判断特殊雷达类型】：根据项目需要配置，这里默认以 HELIOS 为例
    param.lidar_type = m_isCustomDevice ? LidarType::RSHELIOS_16P : LidarType::RSHELIOS;

    param.input_param.msop_port = m_msopPort;
    param.input_param.difop_port = m_difopPort;

    // 硬件时钟和解包配置
    param.decoder_param.use_lidar_clock = true;
    param.decoder_param.wait_for_difop = true;
    param.decoder_param.dense_points = true;

    // 注册回调
    m_driver.regPointCloudCallback(
        std::bind(&RsLidarParser::driverGetCallback, this),
        std::bind(&RsLidarParser::driverPutCallback, this, std::placeholders::_1)
    );

    m_driver.regExceptionCallback(
        std::bind(&RsLidarParser::exceptionCallback, this, std::placeholders::_1)
    );

    if (!m_driver.init(param)) {
        qWarning() << "[RsLidarParser] Lidar Driver Init Failed!";
        return false;
    }

    m_driver.start();

    // 启动异步点云处理线程
    m_stop = false;
    m_processThread = std::thread(&RsLidarParser::processCloudLoop, this);

    m_isReady = true;
    //qInfo() << "[RsLidarParser] Parser started successfully in RAW_PACKET mode.";
    return true;
}

void RsLidarParser::inputPacket(const QByteArray& packet, double timestamp)
{
    inputPacketWithGeneration(
        packet, timestamp,
        m_timelineGeneration.load(std::memory_order_acquire));
}

void RsLidarParser::inputPacketWithGeneration(const QByteArray& packet,
                                              double timestamp,
                                              quint64 timelineGeneration)
{
    if (!m_isReady
        || timelineGeneration
            != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }

    // 记录外部传进来的 PCAP 包时间戳，用于防抽风逻辑
    m_currentPcapTimestamp.store(timestamp, std::memory_order_relaxed);
    m_currentPacketGeneration.store(timelineGeneration, std::memory_order_release);

    // 【核心改变 2】：将外部 QByteArray 转换为 rs_driver 的 Packet
    // 注意：传入的 packet 必须是剥离了 MAC/IP/UDP 头的纯 Payload 数据（如 1248 字节）
    Packet pkt;
    pkt.buf_.assign(packet.begin(), packet.end());

    // 手动将包喂给驱动解码
    m_driver.decodePacket(pkt);
}

void RsLidarParser::requestPlaybackTimelineReset(quint64 timelineGeneration)
{
    const quint64 nextGeneration = timelineGeneration != 0
        ? timelineGeneration
        : m_timelineGeneration.load(std::memory_order_acquire) + 1;

    // RAW_PACKET 驱动内部还可能有旧包和未完成的半帧。stop()
    // 会等待解码线程退出并清空当前点集，再 start() 后首帧只由
    // 新时间线的包构成。
    if (m_isReady.load(std::memory_order_acquire)) {
        m_driver.stop();
        stuffed_queue.clear();
    }
    m_currentPacketGeneration.store(nextGeneration, std::memory_order_release);
    m_timelineGeneration.store(nextGeneration, std::memory_order_release);
    m_currentPcapTimestamp.store(0.0, std::memory_order_relaxed);
    if (m_isReady.load(std::memory_order_acquire))
        m_driver.start();
}

std::shared_ptr<PointCloudMsg> RsLidarParser::driverGetCallback()
{
    // 驱动需要空闲 Buffer 时调用
    std::shared_ptr<PointCloudMsg> msg = free_queue.pop();
    if (msg) {
        return msg;
    }
    return std::make_shared<PointCloudMsg>();
}

void RsLidarParser::driverPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
    // 把帧完成时对应的 PCAP 时间和时间线代号一起入队，避免跳转后
    // 异步处理线程把旧帧时间累计到新时间线上。
    stuffed_queue.push({
        msg,
        m_currentPcapTimestamp.load(std::memory_order_relaxed),
        m_currentPacketGeneration.load(std::memory_order_acquire)});
}

void RsLidarParser::exceptionCallback(const robosense::lidar::Error& code)
{
    // 错误状态记录（可根据 common/error_code.hpp 增加具体的错误处理）
    qWarning() << "[RsLidarParser] Robosense Driver Exception Code:" << code.error_code;
}

void RsLidarParser::processCloudLoop()
{
    quint64 processingGeneration = 0;
    while (!m_stop) {
        // 从完成队列中取出点云帧，超时设置为 50ms
        QueuedCloud queued = stuffed_queue.popWait(50000);
        std::shared_ptr<PointCloudMsg> msg = queued.cloud;

        if (!msg) {
            continue;
        }

        const quint64 currentGeneration =
            m_timelineGeneration.load(std::memory_order_acquire);
        if (queued.timeline_generation != currentGeneration) {
            // 时间跳转前已经完成、但尚未来得及消费的帧不能进入新时间线。
            msg->points.clear();
            free_queue.push(msg);
            continue;
        }

        if (queued.timeline_generation != processingGeneration) {
            processingGeneration = queued.timeline_generation;
            is_first_frame_of_file_ = true;
            file_start_unix_time_ = 0.0;
            last_frame_lidar_time_ = 0.0;
            time_offset_accumulator_ = 0.0;
        }

        // --- 提取 RS-main.cpp 中的防抽风时间逻辑（帧级别） ---
        double frame_timestamp = msg->timestamp; // 驱动解析出的时间戳
        double external_pcap_time = queued.pcap_timestamp;

        if (frame_timestamp != 0.0) {
            if (is_first_frame_of_file_) {
                file_start_unix_time_ = external_pcap_time > 0 ? external_pcap_time : frame_timestamp;
                time_offset_accumulator_ = 0.0;
                last_frame_lidar_time_ = frame_timestamp;
                is_first_frame_of_file_ = false;
            } else {
                double diff_sec = frame_timestamp - last_frame_lidar_time_;
                // 硬件时间异常时，强行加上标准的 0.1 秒 (10Hz)
                time_offset_accumulator_ += (diff_sec > 0.001 && diff_sec < 0.25) ? diff_sec : 0.1;
                last_frame_lidar_time_ = frame_timestamp;
            }

            // 计算修正后的最终绝对时间
            frame_timestamp = file_start_unix_time_ + time_offset_accumulator_;
        }

        // 格式转换与分发
        TimedFrame<QVector<M_PointXYZI>> parsedFrame = convertToYourFormat(msg);
        parsedFrame.timestamp = frame_timestamp;
        parsedFrame.timeline_generation = queued.timeline_generation;

        emit frameReady(parsedFrame);

        // 回收 Buffer 到空闲队列
        msg->points.clear();
        free_queue.push(msg);
    }
}

TimedFrame<QVector<M_PointXYZI>> RsLidarParser::convertToYourFormat(const std::shared_ptr<PointCloudMsg>& msg)
{
    TimedFrame<QVector<M_PointXYZI>> targetFrame;
    targetFrame.data.reserve(msg->points.size());

    // 将 PointXYZI 转化为外部所需的 M_PointXYZI
    // 【注意】：因为 RsLidarParser.h 中限定了 PointCloudMsg 为 PointXYZI 类型，
    // 我们无法获取每个点的相对 timestamp 和 ring。
    for (const auto& p : msg->points) {
        // 过滤掉无效点
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            continue;
        }

        M_PointXYZI customPoint;
        customPoint.x = p.x;
        customPoint.y = p.y;
        customPoint.z = p.z;
        customPoint.intensity = p.intensity;

        targetFrame.data.push_back(customPoint);
    }

    return targetFrame;
}

#include "Ls400Parser.h"
#include <QDebug>

Ls400Parser::Ls400Parser(const QHostAddress &ip, quint16 port, QObject *parent)
    : QObject(parent), m_ip(ip), m_port(port)
{
    qRegisterMetaType<std::shared_ptr<TimedFrame<QVector<M_PointXYZI>>>>("std::shared_ptr<TimedFrame<QVector<M_PointXYZI>>>");

    // 信号改为
    // void frameReady(std::shared_ptr<TimedFrame<QVector<M_PointXYZI>>> frame);
    // 1. 初始化 SDK 驱动
    m_sdkDriver = std::make_shared<GetLidarData_LS>();

    // 2. 模仿学习代码中的回调机制，但对接 Qt 信号
    m_sdkCallback = [this](std::shared_ptr<std::vector<MuchLidarData>> points, int flag, std::string info) {
        this->handleSdkData(points);
    };

    m_sdkDriver->setCallbackFunction(&m_sdkCallback);

    // 启动离线/实时解析模式
    m_sdkDriver->LidarOfflineDataStar();
}

Ls400Parser::~Ls400Parser() {
    if (m_sdkDriver) m_sdkDriver->LidarStop();
}

void Ls400Parser::inputPacket(const QByteArray& packet, double timestamp) {
    inputPacketWithGeneration(
        packet, timestamp,
        m_timelineGeneration.load(std::memory_order_acquire));
}

void Ls400Parser::inputPacketWithGeneration(const QByteArray& packet,
                                            double timestamp,
                                            quint64 timelineGeneration) {
    if (timelineGeneration
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    if (!isFirst) {
        qDebug() << "Ls400Parser: 开始接收数据流，源 IP:" << m_ip.toString();
        isFirst = true;
    }

    // 暂存当前包的时间戳，SDK 回调时会用到
    m_currentTimestamp.store(timestamp, std::memory_order_relaxed);
    m_currentPacketGeneration.store(timelineGeneration, std::memory_order_release);

    // 3. 将原始字节流送入 SDK（模仿 CollectionDataArrive）
    // 注意：这里传入的是你已经在外部过滤好的干净数据
    m_sdkDriver->CollectionDataArrive((void*)packet.constData(), (uint16_t)packet.size());
}

void Ls400Parser::requestPlaybackTimelineReset(quint64 timelineGeneration)
{
    const quint64 nextGeneration = timelineGeneration != 0
        ? timelineGeneration
        : m_timelineGeneration.load(std::memory_order_acquire) + 1;
    if (m_sdkDriver)
        m_sdkDriver->ClearPendingOfflinePackets();
    m_currentTimestamp.store(0.0, std::memory_order_relaxed);
    m_currentPacketGeneration.store(nextGeneration, std::memory_order_release);
    m_timelineGeneration.store(nextGeneration, std::memory_order_release);
    m_discardFirstFrameGeneration.store(nextGeneration, std::memory_order_release);
}

void Ls400Parser::handleSdkData(std::shared_ptr<std::vector<MuchLidarData>> points) {
    if (!points || points->empty()) return;

    // 4. 构建输出帧
    TimedFrame<QVector<M_PointXYZI>> frame;
    frame.timestamp = m_currentTimestamp.load(std::memory_order_relaxed);
    frame.timeline_generation =
        m_currentPacketGeneration.load(std::memory_order_acquire);

    if (frame.timeline_generation
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    quint64 discardGeneration = frame.timeline_generation;
    if (m_discardFirstFrameGeneration.compare_exchange_strong(
            discardGeneration, 0, std::memory_order_acq_rel)) {
        // SDK 是预编译库，无法从外部清除其半帧。跳转后丢弃
        // 第一个完整回调，后续帧即只由新时间线的包组成。
        return;
    }

    frame.data.reserve(points->size());

    // 5. 模仿学习代码中的坐标转换逻辑
    for (const auto& p : *points) {
        M_PointXYZI pt;
        pt.x = p.X;
        pt.y = p.Y;
        pt.z = p.Z;
        pt.intensity = static_cast<float>(p.Intensity);
        frame.data.append(pt);
    }

    // 6. 发射信号（对外接口完全兼容）
    emit frameReady(frame);
}

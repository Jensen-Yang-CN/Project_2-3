#ifndef LS400PARSER_H
#define LS400PARSER_H
#include <QObject>
#include <QHostAddress>
#include <QtEndian>
#include <memory>
#include <atomic>
#include "common/types.h"
#include "common/pointxyz.h"

#include "LidarPacketStruct.h"
#include "lib/LS_SDK/Include/GetLidarData_LS.h" // 引入学习代码中的镭神SDK


class Ls400Parser : public QObject {
    Q_OBJECT
public:
    // 构造函数接口保持不变
    explicit Ls400Parser(const QHostAddress &ip, quint16 port, QObject *parent = nullptr);
    ~Ls400Parser();

public slots:
    // 严格匹配对外接口：接收原始包与时间戳
    void inputPacket(const QByteArray& packet, double timestamp);
    void inputPacketWithGeneration(const QByteArray& packet, double timestamp,
                                   quint64 timelineGeneration);
    void requestPlaybackTimelineReset(quint64 timelineGeneration = 0);

signals:
    // 严格匹配对外接口：发送带有时间戳的点云帧
    void frameReady(TimedFrame<QVector<M_PointXYZI>> frame);

private:
    // SDK 回调处理逻辑
    void handleSdkData(std::shared_ptr<std::vector<MuchLidarData>> points);
    // SDK 相关成员
    std::shared_ptr<GetLidarData_LS> m_sdkDriver;
    FunDataPrt m_sdkCallback;
    std::atomic<double> m_currentTimestamp{0.0}; // 暂存 inputPacket 传入的时间戳供回调使用
    std::atomic<quint64> m_timelineGeneration{1};
    std::atomic<quint64> m_currentPacketGeneration{1};
    std::atomic<quint64> m_discardFirstFrameGeneration{0};
private:
    QHostAddress m_ip;
    quint16 m_port;
    bool isFirst = false;


};

#endif // LS400PARSER_H

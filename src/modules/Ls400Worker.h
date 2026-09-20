#ifndef LS400WORKER_H
#define LS400WORKER_H


#include <QObject>
#include <QtMath>
#include "common/pointxyz.h"
#include "LidarPacketStruct.h"

// 假设 LidarFrame 定义为 TimedFrame<QVector<PointXYZI>>
using LidarFrame = TimedFrame<QVector<M_PointXYZI>>;

class Ls400Worker : public QObject {
    Q_OBJECT
public:
    explicit Ls400Worker(QObject *parent = nullptr);
    ~Ls400Worker();
public slots:
    // 核心：接收来自 Parser 的帧进行坐标转换
    void handleFrame(const LidarFrame &frame);

    // 兼容接口
    void processPacket(QByteArray payload);

signals:
    // 严格匹配：发送给 Widget 可视化
    void cloudReady(const QVector<M_PointXYZI> &cloud,double ts);
    void cloudReadyWithGeneration(const QVector<M_PointXYZI> &cloud, double ts,
                                  quint64 timelineGeneration);

private:
    float m_vAngleLUT[400];
    void initLUT();
};

#endif // LS400WORKER_H

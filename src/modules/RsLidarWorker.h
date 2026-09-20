#ifndef RSLIDARWORKER_H
#define RSLIDARWORKER_H

#include "BaseWorker.h"
#include "common/pointxyz.h" // 引用你定义的结构体
#include <QVector>
#include <QFile>
#include <QTextStream>

// 定义类型别名方便使用
typedef TimedFrame<QVector<M_PointXYZI>> LidarFrame;

class RsLidarWorker : public BaseWorker
{
    Q_OBJECT
public:
    explicit RsLidarWorker(QObject *parent = nullptr);
    ~RsLidarWorker();

signals:
    // 发送给 LidarWidget 进行可视化的信号
    void cloudReady(const QVector<M_PointXYZI> &cloud,double ts);
    void cloudReadyWithGeneration(const QVector<M_PointXYZI> &cloud, double ts,
                                  quint64 timelineGeneration);

public slots:
    // 实现 BaseWorker 的纯虚接口
    void processPacket(QByteArray payload) override;

    // 核心：接收来自 Parser 的流式点云帧
    // 参数匹配 emit frameReady(const QVector<M_PointXYZI> &cloud)
    void handleFrame(const LidarFrame& frame);

private:
    void savePointToFile(QVector<M_PointXYZI> validPoints, QString filename);
private:
    // 如果需要对点云进行预处理（如镜像、旋转），可以在这里定义私有方法

    QFile *m_file = nullptr;       // 声明文件指针
    QTextStream *m_stream = nullptr; // 声明流指针 <--- 添加这一行
};

#endif // RSLIDARWORKER_H

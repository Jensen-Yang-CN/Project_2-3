#ifndef RADARFUSIONMANAGER_H
#define RADARFUSIONMANAGER_H

#include <QObject>
#include <QString>
#include <QColor>
#include <QTimer>
#include <QMatrix4x4>
#include "common/pointxyz.h"

class RadarFusionManager : public QObject
{
    Q_OBJECT
public:
    explicit RadarFusionManager(QObject *parent = nullptr);

public slots:
    /**
     * @brief 接收各路雷达原始点云的槽函数
     * @param id 雷达编号 ("210", "211" 等)
     * @param cloud 该雷达的一帧点云数据
     */
    void handleRadarCloud(const QString &id, const QVector<M_PointXYZI> &cloud, double ts);
    void handleRadarCloudForTimeline(const QString &id,
                                     const QVector<M_PointXYZI> &cloud,
                                     double ts,
                                     quint64 timelineGeneration);
    void resetForPlaybackSeek(quint64 timelineGeneration = 1);

signals:
    /** 兼容主窗口现有链路：一次发送完整融合点云。 */
    void fusedCloudReady(const QVector<M_PointXYZI> &cloud,
                         double timestampSec);
    /** 发送给渲染窗体的 201/210/211/213/214 显示点云。 */
    void displayCloudReady(const QVector<M_PointXYZI> &displayCloud,
                           double timestampSec);
    /** 发送给 SLAM 和感知总线的 210/211 算法点云。 */
    void algorithmCloudReady(const QVector<M_PointXYZI> &algorithmCloud,
                             double timestampSec);

private slots:
    // 定时器触发后的真正融合逻辑
    void onSyncTimeout();

private:
    /**
     * @brief 执行所有已缓存雷达的点云融合
     */
    void fuseSynchronized(double masterTs);

private:
    // 存储每个雷达最近收到的一帧数据
    QMap<QString, TimedFrame<QVector<M_PointXYZI>>> m_framesCache;

    // 为每个 ID 维护一个队列，存储最近的几帧
    QMap<QString, QList<TimedFrame<QVector<M_PointXYZI>>>> m_queues;

    QTimer *m_syncTimer;
    double m_currentMasterTs = 0.0; // 当前正在等待的 210/211 主时间戳
    bool m_hasSeenPrimary210 = false;
    double m_latestPrimary210Ts = 0.0;
    quint64 m_timelineGeneration = 1;
    const int m_maxQueueSize = 5;   // 每个雷达只保留最近 5 帧，防止内存溢出
    const double m_maxTolerance = 0.10;
    const double m_primaryFallbackTimeoutSec = 0.50;
};

#endif // RADARFUSIONMANAGER_H

#include "radarfusionmanager.h"
#include "radartransform.h"
#include <QTimer>
#include <cmath>

RadarFusionManager::RadarFusionManager(QObject *parent) : QObject(parent) {
    m_syncTimer = new QTimer(this);
    m_syncTimer->setSingleShot(true); // 确保只触发一次
    connect(m_syncTimer, &QTimer::timeout, this, &RadarFusionManager::onSyncTimeout);
}

void RadarFusionManager::handleRadarCloud(const QString &id, const QVector<M_PointXYZI> &cloud, double ts) {
//    if (id == "214") {
//            qDebug() << ">>> Fusion 收到 214 数据 | 点数:" << cloud.size() << " | TS:" << QString::number(ts, 'f', 6);
//    }
//    if (id == "213") {
//            qDebug() << ">>> Fusion 收到 214 数据 | 点数:" << cloud.size() << " | TS:" << QString::number(ts, 'f', 6);

//    }
//    if (id == "201") {
//            qDebug() << ">>> Fusion 收到 214 数据 | 点数:" << cloud.size() << " | TS:" << QString::number(ts, 'f', 6);

//    }

    // 1. 存入队列 (代替了 m_latestClouds)
    TimedFrame<QVector<M_PointXYZI>> frame;
    frame.timestamp = ts;
    frame.timeline_generation = m_timelineGeneration;
    frame.data = cloud;
    m_queues[id].append(frame);

    // 限制队列长度，防止内存爆炸（只留最近 5 帧）
    if (m_queues[id].size() > m_maxQueueSize) {
        m_queues[id].removeFirst();
    }

    // 2. 如果是主雷达 210，启动等待窗口
    if (id == "210") {
        m_currentMasterTs = ts;
        // 如果 10ms 定时器没跑，就启动它
        if (!m_syncTimer->isActive()) {
            m_syncTimer->start(10);
        }
    }
}

void RadarFusionManager::handleRadarCloudForTimeline(
    const QString &id,
    const QVector<M_PointXYZI> &cloud,
    double ts,
    quint64 timelineGeneration)
{
    if (timelineGeneration != m_timelineGeneration)
        resetForPlaybackSeek(timelineGeneration);
    handleRadarCloud(id, cloud, ts);
}

void RadarFusionManager::resetForPlaybackSeek(quint64 timelineGeneration)
{
    m_timelineGeneration = timelineGeneration;
    m_queues.clear();
    m_framesCache.clear();
    m_syncTimer->stop();
    m_currentMasterTs = 0.0;
    m_hasSeenPrimary210 = false;
    m_latestPrimary210Ts = 0.0;
}

void RadarFusionManager::onSyncTimeout() {
    // 触发真正的同步融合逻辑
    fuseSynchronized(m_currentMasterTs);
}


void RadarFusionManager::fuseSynchronized(double masterTs) {
    QVector<M_PointXYZI> totalCloud;
    // 201 为船尾雷达，已从融合白名单中排除。
    static const QStringList radarIds = {"210", "211", "213", "214"};

    for (const QString &id : radarIds) {
        if (!m_queues.contains(id) || m_queues[id].isEmpty()) continue;

        auto &queue = m_queues[id];
        int bestIdx = -1;
        double minDiff = 999.0;

        // 1. 寻找最匹配的一帧
        for (int i = 0; i < queue.size(); ++i) {
            double diff = std::abs(queue[i].timestamp - masterTs);
            if (diff < minDiff) {
                minDiff = diff;
                bestIdx = i;
            }
        }

        // 2. 暴力通过测试 (为了验证 214 是否能显示)
        // 只要队列里有数据，我们就取最接近的那一帧，不管差多少秒
        bool forcePass = true;

        if (bestIdx != -1 && (minDiff < m_maxTolerance || forcePass)) {
            const auto &frame = queue[bestIdx];
            // 获取该雷达的安装位置和外参 (RadarTransform 里的配置)
            RadarConfig config = RadarTransform::getConfig(id);

            // --- 这一部分必须存在，否则 totalCloud 永远为空 ---
            for (const auto &p : frame.data) {
                // 执行外参矩阵变换
                QVector3D newPos = config.transform.map(QVector3D(p.x, p.y, p.z));

                M_PointXYZI tp;
                tp.x = newPos.x();
                tp.y = newPos.y();
                tp.z = newPos.z();
                // 使用配置的强度值，以便在界面上区分不同雷达的颜色
                tp.intensity = config.intensity;

                totalCloud.append(tp);
            }

            // 调试：看看 214 到底塞进去了多少点
            if (id == "214") {
                //qDebug() << "Fusion: 214 added" << frame.data.size() << "points to totalCloud.";
            }
        }
    }

    // 3. 最后统一发送（与 fusedCloudReady 信号声明一致：点云 + 主时间戳）
    if (!totalCloud.isEmpty()) {
        emit fusedCloudReady(totalCloud, masterTs);
    } else {
        //qDebug() << "Fusion: totalCloud is EMPTY!";
    }
}

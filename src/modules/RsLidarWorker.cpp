#include "RsLidarWorker.h"
#include <QDebug>
#include <QtEndian>
#include <QDateTime>

RsLidarWorker::RsLidarWorker(QObject *parent) : BaseWorker(parent)
{
    // 跨线程传输自定义结构体必须注册元类型
    qRegisterMetaType<LidarFrame>("LidarFrame");
    qRegisterMetaType<QVector<M_PointXYZI>>("QVector<M_PointXYZI>");
}

RsLidarWorker::~RsLidarWorker() {}

void RsLidarWorker::processPacket(QByteArray payload)
{
    // BaseWorker 要求的接口，如果不需要直接处理原始包，可以留空
    // 所有的解析工作已经在 Parser 中由 inputPacket 完成了
    (void)payload;
}

void RsLidarWorker::handleFrame(const LidarFrame &frame)
{

    if (frame.data.isEmpty()) return;
    static int p_count = 0;
    if(++p_count % 2000 == 0){
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(frame.timestamp * 1000));
        qDebug() << "RsLidarWorker Worker 处理帧，点数:" << frame.data.size() << " 时间戳:" << dt.toString("yyyy-MM-dd HH:mm:ss.zzz");
    }

    emit cloudReady(frame.data,frame.timestamp);
    emit cloudReadyWithGeneration(
        frame.data, frame.timestamp, frame.timeline_generation);

}

void RsLidarWorker::savePointToFile( QVector<M_PointXYZI> validPoints, QString filename)
{
    if (!validPoints.isEmpty()) {
        // 1. 确保 QFile 已经实例化
        if (m_file == nullptr) {
            m_file = new QFile(filename, this);
        }

        // 2. 确保文件是打开状态
        if (!m_file->isOpen()) {
            if (!m_file->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)) {
                qWarning() << "无法打开文件:" << m_file->errorString();
                return;
            }
        }
        // 3. 确保 m_stream 绑定
        if (m_stream == nullptr) {
            m_stream = new QTextStream();
        }
        if (m_stream->device() == nullptr) {
            m_stream->setDevice(m_file);
        }

        // 4. 写入数据并过滤 nan
        if (m_stream->device() && m_stream->device()->isOpen()) {
            if (m_file->size() == 0) {
                *m_stream << "X,Y,Z" << Qt::endl;
            }

            int nanCount = 0;   // 记录本帧中无效点的个数
            int writeCount = 0; // 记录本帧中成功写入的个数

            for (const auto& point : validPoints) {
                // --- 过滤逻辑开始 ---
                // 检查 x, y, z 是否有任何一个是非法数值 (nan)
                if (qIsNaN(point.x) || qIsNaN(point.y) || qIsNaN(point.z)) {
                    nanCount++;
                    continue;
                }
                // --- 过滤逻辑结束 ---

                *m_stream << point.x << ","
                          << point.y << ","
                          << point.z << "\n"; // 改用 \n 提高写入效率，Qt::endl 会强制 flush 磁盘
                writeCount++;
            }

            // 每帧处理完后，打印统计信息（可选）
            if (nanCount > 0) {
                qDebug() << "帧处理完成: 写入" << writeCount << "点, 过滤无效(nan)" << nanCount << "点";
            }

            // 建议：如果不需要实时查看，可以每隔几十帧 flush 一次，或者让系统自动管理
            // m_stream->flush();
        } else {
            qWarning() << "QTextStream 依然没有有效的设备！";
        }
    }
}

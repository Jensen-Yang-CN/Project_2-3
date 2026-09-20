#include "Ls400Worker.h"
#include <QDebug>
#include <QDateTime>

Ls400Worker::Ls400Worker(QObject *parent) : QObject(parent) {
}

Ls400Worker::~Ls400Worker() {
}

void Ls400Worker::processPacket(QByteArray payload) {
    Q_UNUSED(payload);
}

void Ls400Worker::handleFrame(const LidarFrame &frame) {
    if (frame.data.isEmpty()) return;

    static int p_count = 0;
    if (++p_count % 1000 == 0) {
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(frame.timestamp * 1000));
        qDebug() << "Ls400Worker 处理帧，点数:" << frame.data.size() << " 时间戳:" << dt.toString("yyyy-MM-dd HH:mm:ss.zzz");
    }

    emit cloudReady(frame.data, frame.timestamp);
    emit cloudReadyWithGeneration(
        frame.data, frame.timestamp, frame.timeline_generation);
}

void Ls400Worker::initLUT() {
}

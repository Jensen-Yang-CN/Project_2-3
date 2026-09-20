#ifndef BRIDGE_DETECTION_WORKER_H
#define BRIDGE_DETECTION_WORKER_H

#include <QObject>
#include <QImage>
#include <QLineF>
#include <QVector>
#include "common/pointxyz.h"

/** 在独立线程中运行嵌入式桥洞检测（BridgeDetector + CUDA） */
class BridgeDetectionWorker : public QObject {
    Q_OBJECT
public:
    explicit BridgeDetectionWorker(bool leftCamera = true, QObject *parent = nullptr);
    ~BridgeDetectionWorker();

public slots:
    void initialize();
    void resetForPlaybackSeek();
    void runDetection(double timestamp,
                      QVector<M_PointXYZI> cloud210,
                      QVector<M_PointXYZI> cloud211,
                      QImage cameraImage,
                      quint64 timelineGeneration);

signals:
    void logMessage(const QString &text);
    void taskLogMessage(const QString &text, quint64 timelineGeneration);
    void bridgeOverlayReady(const QImage &overlay, double timestampSec,
                            quint64 timelineGeneration);
    /** 供 UI/融合层使用的传统视觉桥面线结果。 */
    void traditionalDeckLineReady(const QLineF &deckLine, bool detected,
                                  double timestampSec,
                                  quint64 timelineGeneration);

private:
    struct Impl;
    Impl *m_impl = nullptr;
    bool m_leftCamera = true;
};

#endif // BRIDGE_DETECTION_WORKER_H

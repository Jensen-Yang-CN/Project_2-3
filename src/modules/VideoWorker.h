#pragma once
#include <QObject>
#include <QQueue>
#include <QMutex>
#include <QImage>
#include <QTimer>
#include <atomic>
#include "AVDecoder.h"

extern "C" {
#include <libavcodec/avcodec.h>    // 核心编解码库
#include <libavutil/avutil.h>      // 工具库（包括像素格式定义）
#include <libavutil/imgutils.h>    // 图像处理工具
#include <libswscale/swscale.h>    // 像素格式转换（YUV 转 RGB）
}

class VideoWorker : public QObject {
    Q_OBJECT
public:
    explicit VideoWorker(QObject *parent = nullptr,int codecId = AV_CODEC_ID_HEVC)
        : QObject(parent), codecId(codecId){}

    void setVideoWorkerName(QString workerName);

public slots:
    void initDecoder();
    void resetDecoder();
    void handleNal(const QByteArray &nal);
    void resetDecoderForTimeline(quint64 timelineGeneration);
    void handleNalWithGeneration(const QByteArray &nal, double timestamp,
                                 quint64 timelineGeneration);

signals:
    void frameReady(const QImage &img);
    void frameReadyWithGeneration(const QImage &img, double timestamp,
                                  quint64 timelineGeneration);

private:
    QString m_workerName="";
    AVDecoder decoder;
    int codecId;
    bool ready = false;
    AVCodecID m_codecId;
    AVCodec* m_codec = nullptr;           // 每个对象独立的解码器指针
    AVCodecContext* m_codecContext = nullptr; // 每个对象独立的上下文（核心！）
    AVFrame* m_frame = nullptr;           // 每个对象独立的帧缓存
    AVPacket* m_packet = nullptr;         // 每个对象独立的包缓存
    struct SwsContext* m_swsContext = nullptr; // 每个对象独立的缩放转换器
    std::atomic<quint64> m_timelineGeneration{1};
};

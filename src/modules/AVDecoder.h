#ifndef AVDECODER_H
#define AVDECODER_H

#include <QImage>
#include <QByteArray>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
}

class AVDecoder {
public:
    AVDecoder();
    ~AVDecoder();

    // 初始化解码器 (支持 AV_CODEC_ID_H264 或 AV_CODEC_ID_HEVC)
    bool init(int codecId);

    // 核心解码函数
    QImage decodeFrame(const QByteArray &data);

    // 释放资源
    void release();

private:
    AVCodecContext* m_pCodecCtx = nullptr;
    AVFrame* m_pFrame    = nullptr;
    AVPacket* m_pPacket   = nullptr;
    SwsContext* m_pSwsCtx   = nullptr;

    uint8_t* m_outBuffer = nullptr;
    int             m_imageSize = 0;
    bool            m_isInit    = false;
};

#endif // AVDECODER_H

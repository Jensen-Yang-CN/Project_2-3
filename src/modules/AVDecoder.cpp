#include "AVDecoder.h"
#include <QDebug>

extern "C" {
#include <libavutil/log.h>
#include <libavutil/opt.h>
}

// 屏蔽警告日志，减少控制台负担
static bool ffmpeg_log_init = []() {
    av_log_set_level(AV_LOG_ERROR);
    return true;
}();

AVDecoder::AVDecoder() {
    m_pPacket = av_packet_alloc();
    m_pFrame = av_frame_alloc();
}

AVDecoder::~AVDecoder() {
    release();
    if (m_pPacket) av_packet_free(&m_pPacket);
    if (m_pFrame)  av_frame_free(&m_pFrame);
}

bool AVDecoder::init(int codecId) {
    if (m_isInit) release();

    const AVCodec* codec = avcodec_find_decoder((AVCodecID)codecId);
    if (!codec) return false;

    m_pCodecCtx = avcodec_alloc_context3(codec);
    if (!m_pCodecCtx) return false;

    // --- 【关键】解决马赛克与实时性优化 ---

    // 1. 禁用多线程以消除由于多线程参考帧导致的马赛克
    m_pCodecCtx->thread_count = 1;

    // 2. 强制开启低延迟模式
    m_pCodecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    m_pCodecCtx->flags2 |= AV_CODEC_FLAG2_CHUNKS;

    // 3. 容错处理：当丢帧或数据不全时，不尝试盲目参考上一帧
    // 这能减少“拖影”式的马赛克
    av_opt_set(m_pCodecCtx->priv_data, "tune", "zerolatency", 0);
    m_pCodecCtx->err_recognition = AV_EF_EXPLODE; // 遇到错误包立即丢弃，防止花屏扩散

    if (avcodec_open2(m_pCodecCtx, codec, nullptr) < 0) return false;

    m_isInit = true;
    return true;
}

QImage AVDecoder::decodeFrame(const QByteArray &data) {
    if (!m_isInit || data.isEmpty()) return QImage();

    m_pPacket->data = (uint8_t*)data.data();
    m_pPacket->size = data.size();

    // 发送包
    int ret = avcodec_send_packet(m_pCodecCtx, m_pPacket);
    if (ret < 0) return QImage();

    // 接收帧
    ret = avcodec_receive_frame(m_pCodecCtx, m_pFrame);
    if (ret != 0) return QImage();

    // --- 【关键】解决画质不清晰（颜色范围） ---

    // 红外视频必须显式设置为 Full Range，否则对比度极差，看起来像蒙了层灰
    m_pCodecCtx->color_range = AVCOL_RANGE_JPEG;

    m_pSwsCtx = sws_getCachedContext(m_pSwsCtx,
        m_pCodecCtx->width, m_pCodecCtx->height, m_pCodecCtx->pix_fmt,
        m_pCodecCtx->width, m_pCodecCtx->height, AV_PIX_FMT_RGB32,
        SWS_POINT, nullptr, nullptr, nullptr); // 使用 SWS_POINT 保持原始红外像素锐度

    if (m_pSwsCtx) {
        int inv_table[4], table[4];
        int srcRange, dstRange, brightness, contrast, saturation;
        sws_getColorspaceDetails(m_pSwsCtx, (int**)&inv_table, &srcRange, (int**)&table, &dstRange, &brightness, &contrast, &saturation);

        // 强制 1:1 色彩转换
        sws_setColorspaceDetails(m_pSwsCtx,
                                 sws_getCoefficients(SWS_CS_ITU601), 1,
                                 sws_getCoefficients(SWS_CS_ITU601), 1,
                                 0, 1 << 16, 1 << 16);
    }

    if (m_imageSize == 0 || !m_outBuffer) {
        m_imageSize = av_image_get_buffer_size(AV_PIX_FMT_RGB32, m_pCodecCtx->width, m_pCodecCtx->height, 1);
        m_outBuffer = (uint8_t*)av_malloc(m_imageSize);
    }

    uint8_t* dstData[4] = { m_outBuffer, nullptr, nullptr, nullptr };
    int dstLinesize[4] = { m_pCodecCtx->width * 4, 0, 0, 0 };

    sws_scale(m_pSwsCtx, m_pFrame->data, m_pFrame->linesize, 0, m_pCodecCtx->height, dstData, dstLinesize);

    // copy() 确保数据拷贝到主线程，防止子线程修改导致的花屏
    QImage img(m_outBuffer, m_pCodecCtx->width, m_pCodecCtx->height, QImage::Format_RGB32);
    return img.copy();
}

void AVDecoder::release() {
    if (m_pCodecCtx) { avcodec_free_context(&m_pCodecCtx); m_pCodecCtx = nullptr; }
    if (m_pSwsCtx) { sws_freeContext(m_pSwsCtx); m_pSwsCtx = nullptr; }
    if (m_outBuffer) { av_free(m_outBuffer); m_outBuffer = nullptr; }
    m_imageSize = 0;
    m_isInit = false;
}

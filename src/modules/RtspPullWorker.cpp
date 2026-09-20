#include "RtspPullWorker.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

// 完整定义须与 RtspPullWorker.h 中前向声明为同一类型
struct RtspPullWorker::FFmpegState {
    AVFormatContext *fmtCtx = nullptr;
    AVCodecContext *codecCtx = nullptr;
    SwsContext *swsCtx = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *packet = nullptr;
    int videoStreamIndex = -1;
    uint8_t *rgbBuffer = nullptr;
    int rgbBufferSize = 0;
};

namespace {

static bool s_networkInited = false;

void ensureNetworkInit()
{
    if (!s_networkInited) {
        avformat_network_init();
        av_log_set_level(AV_LOG_ERROR);
        s_networkInited = true;
    }
}

} // namespace

RtspPullWorker::RtspPullWorker(QObject *parent)
    : QObject(parent)
{
}

void RtspPullWorker::startPull(const QString &url)
{
    if (url.isEmpty()) {
        emit pullFailed(tr("RTSP 地址为空"));
        return;
    }
    if (m_running.load())
        stopPull();

    m_url = url;
    m_stopRequested = false;
    QMetaObject::invokeMethod(this, "runPullLoop", Qt::QueuedConnection);
}

void RtspPullWorker::stopPull()
{
    m_stopRequested = true;
}

void RtspPullWorker::runPullLoop()
{
    ensureNetworkInit();
    m_stopRequested = false;

    if (!openStream(m_url)) {
        emit pullFailed(tr("[%1] 无法打开: %2").arg(m_streamName, m_url));
        return;
    }

    runReadLoop();
}

void RtspPullWorker::runReadLoop()
{
    m_running = true;
    emit logMessage(tr("[%1] RTSP PLAY 成功: %2").arg(m_streamName, m_url));

    while (!m_stopRequested.load()) {
        const int ret = av_read_frame(m_ff->fmtCtx, m_ff->packet);
        if (ret < 0) {
            if (ret == AVERROR_EOF)
                emit logMessage(tr("[%1] RTSP 流结束").arg(m_streamName));
            else
                emit logMessage(tr("[%1] 读流错误: %2").arg(m_streamName).arg(ret));
            break;
        }

        if (m_ff->packet->stream_index != m_ff->videoStreamIndex) {
            av_packet_unref(m_ff->packet);
            continue;
        }

        if (avcodec_send_packet(m_ff->codecCtx, m_ff->packet) < 0) {
            av_packet_unref(m_ff->packet);
            continue;
        }
        av_packet_unref(m_ff->packet);

        while (!m_stopRequested.load()) {
            const int recv = avcodec_receive_frame(m_ff->codecCtx, m_ff->frame);
            if (recv == AVERROR(EAGAIN) || recv == AVERROR_EOF)
                break;
            if (recv < 0)
                break;

            const QImage img = decodeFrameToImage();
            if (!img.isNull())
                emit frameReady(img);
        }
    }

    closeStream();
    m_running = false;
}

bool RtspPullWorker::openStream(const QString &url)
{
    closeStream();
    m_ff = new FFmpegState();
    m_ff->packet = av_packet_alloc();
    m_ff->frame = av_frame_alloc();
    if (!m_ff->packet || !m_ff->frame)
        return false;

    // RTSP 选项：TCP 更稳；超时 5s
    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);
    av_dict_set(&opts, "stimeout", "5000000", 0);
    av_dict_set(&opts, "max_delay", "500000", 0);
    if (!m_localBindAddress.isEmpty())
        av_dict_set(&opts, "localaddr", m_localBindAddress.toUtf8().constData(), 0);

    const QByteArray urlUtf8 = url.toUtf8();
    if (avformat_open_input(&m_ff->fmtCtx, urlUtf8.constData(), nullptr, &opts) < 0) {
        av_dict_free(&opts);
        closeStream();
        return false;
    }
    av_dict_free(&opts);

    if (avformat_find_stream_info(m_ff->fmtCtx, nullptr) < 0) {
        closeStream();
        return false;
    }

    for (unsigned i = 0; i < m_ff->fmtCtx->nb_streams; ++i) {
        if (m_ff->fmtCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            m_ff->videoStreamIndex = static_cast<int>(i);
            break;
        }
    }
    if (m_ff->videoStreamIndex < 0) {
        closeStream();
        return false;
    }

    AVCodecParameters *par = m_ff->fmtCtx->streams[m_ff->videoStreamIndex]->codecpar;
    const AVCodec *codec = avcodec_find_decoder(par->codec_id);
    if (!codec) {
        closeStream();
        return false;
    }

    m_ff->codecCtx = avcodec_alloc_context3(codec);
    if (!m_ff->codecCtx) {
        closeStream();
        return false;
    }

    if (avcodec_parameters_to_context(m_ff->codecCtx, par) < 0) {
        closeStream();
        return false;
    }

    // 低延迟解码（与 AVDecoder 一致）
    m_ff->codecCtx->thread_count = 1;
    m_ff->codecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    m_ff->codecCtx->flags2 |= AV_CODEC_FLAG2_CHUNKS;
    av_opt_set(m_ff->codecCtx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(m_ff->codecCtx, codec, nullptr) < 0) {
        closeStream();
        return false;
    }

    return true;
}

void RtspPullWorker::closeStream()
{
    if (!m_ff)
        return;

    if (m_ff->codecCtx)
        avcodec_free_context(&m_ff->codecCtx);
    if (m_ff->fmtCtx)
        avformat_close_input(&m_ff->fmtCtx);
    if (m_ff->swsCtx)
        sws_freeContext(m_ff->swsCtx);
    if (m_ff->frame)
        av_frame_free(&m_ff->frame);
    if (m_ff->packet)
        av_packet_free(&m_ff->packet);
    if (m_ff->rgbBuffer) {
        av_free(m_ff->rgbBuffer);
        m_ff->rgbBuffer = nullptr;
    }

    delete m_ff;
    m_ff = nullptr;
}

QImage RtspPullWorker::decodeFrameToImage()
{
    if (!m_ff || !m_ff->codecCtx || !m_ff->frame)
        return QImage();

    const int w = m_ff->codecCtx->width;
    const int h = m_ff->codecCtx->height;
    if (w <= 0 || h <= 0)
        return QImage();

    m_ff->swsCtx = sws_getCachedContext(
        m_ff->swsCtx, w, h, m_ff->codecCtx->pix_fmt,
        w, h, AV_PIX_FMT_RGB32, SWS_POINT, nullptr, nullptr, nullptr);

    if (!m_ff->swsCtx)
        return QImage();

    const int bufSize = av_image_get_buffer_size(AV_PIX_FMT_RGB32, w, h, 1);
    if (m_ff->rgbBufferSize < bufSize) {
        av_free(m_ff->rgbBuffer);
        m_ff->rgbBuffer = static_cast<uint8_t *>(av_malloc(static_cast<size_t>(bufSize)));
        m_ff->rgbBufferSize = bufSize;
    }
    if (!m_ff->rgbBuffer)
        return QImage();

    uint8_t *dstData[4] = { m_ff->rgbBuffer, nullptr, nullptr, nullptr };
    int dstLinesize[4] = { w * 4, 0, 0, 0 };
    sws_scale(m_ff->swsCtx, m_ff->frame->data, m_ff->frame->linesize, 0, h,
              dstData, dstLinesize);

    QImage img(m_ff->rgbBuffer, w, h, QImage::Format_RGB32);
    return img.copy();
}

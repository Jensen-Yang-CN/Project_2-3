#include "VideoWorker.h"
#include <QDebug>
//#include <QThread>

//extern "C" {
//#include <>
//#include <>
//}

void VideoWorker::setVideoWorkerName(QString workerName)
{
    this->m_workerName = workerName;
}

void VideoWorker::initDecoder() {

    //m_codecContext->err_recognition = AV_EF_IGNORE_ERR;
    //qDebug()<<"codecId"<<codecId;
    if (!decoder.init(codecId)) {
        qDebug() << "Decoder init FAILED";
        return;
    }
    ready = true;
    //qDebug() << "Decoder init OK in thread:" << QThread::currentThreadId();
}

void VideoWorker::resetDecoder()
{
    ready = false;
    if (!decoder.init(codecId)) {
        qWarning() << "Decoder reset FAILED" << m_workerName;
        return;
    }
    ready = true;
}

void VideoWorker::resetDecoderForTimeline(quint64 timelineGeneration)
{
    m_timelineGeneration.store(timelineGeneration, std::memory_order_release);
    resetDecoder();
}

void VideoWorker::handleNalWithGeneration(const QByteArray &nal,
                                          double timestamp,
                                          quint64 timelineGeneration)
{
    if (timelineGeneration
        != m_timelineGeneration.load(std::memory_order_acquire)) {
        return;
    }
    if (!ready || nal.isEmpty())
        return;

    QByteArray buf = QByteArray("\x00\x00\x00\x01", 4) + nal;
    const QImage img = decoder.decodeFrame(buf);
    if (!img.isNull()
        && timelineGeneration
            == m_timelineGeneration.load(std::memory_order_acquire)) {
        emit frameReadyWithGeneration(img, timestamp, timelineGeneration);
    }
}

// 这个函数会代替原来的 pushRtp + processQueue
void VideoWorker::handleNal(const QByteArray &nal) {

    if (!ready || nal.isEmpty()) return;

    // --- 调试检测逻辑 ---
    // H.264 的 NAL Header 通常在起始码之后的第一个字节
    // 如果 nal 不含起始码，直接看 nal[0]；如果含起始码，看 nal[4]
//    unsigned char header = (unsigned char)nal[0];
//    int nalType = header & 0x1F; // 取低5位

//    if (nalType == 7) {
//        qDebug() << ">>> [" << m_workerName << "] Found SPS (Sequence Parameter Set)";
//    }
//    if (nalType == 5) {
//        qDebug() << ">>> [" << m_workerName << "] Found IDR (Key Frame)";
//    }

//-------------------------------------------------------------
    // --- 改进的起始码添加逻辑 ---
//    QByteArray buf;
//    if (nal.startsWith("\x00\x00\x00\x01") || nal.startsWith("\x00\x00\x01")) {
//        buf = nal;
//    } else {
//        buf = QByteArray("\x00\x00\x00\x01", 4) + nal;
//    }

//    QImage img = decoder.decodeFrame(buf);
//    if (!img.isNull()) {
//        emit frameReady(img);
//    }
//----------------------------------------------------------------
    //qDebug()<<"VideoWorker::handleNal";
    if (!ready || nal.isEmpty()) return;

    // 1. 添加 Start Code
    QByteArray buf = QByteArray("\x00\x00\x00\x01", 4) + nal;

    // 2. 直接调用解码逻辑
    QImage img = decoder.decodeFrame(buf);

    if (!img.isNull()) {

        //QImage imgtemp = FOO(img);

        emit frameReady(img);

    }
}

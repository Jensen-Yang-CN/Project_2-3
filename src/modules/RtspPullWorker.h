#pragma once

#include <QImage>
#include <QObject>
#include <QString>
#include <atomic>

/** 在线模式：FFmpeg RTSP 拉流 + 解码，独立于 Npcap 抓包路径 */
class RtspPullWorker : public QObject {
    Q_OBJECT
public:
    explicit RtspPullWorker(QObject *parent = nullptr);

    /** 日志/失败信息前缀，如「USV02-右固连相机」 */
    void setStreamName(const QString &name) { m_streamName = name; }
    /** RTSP 建连使用的本地网卡 IP（如 192.168.58.177） */
    void setLocalBindAddress(const QString &ip) { m_localBindAddress = ip; }

public slots:
    /** 连接固定 RTSP URL 并阻塞读帧，直到 stopPull() */
    void startPull(const QString &url);
    void stopPull();

signals:
    void frameReady(const QImage &img);
    void logMessage(const QString &text);
    void pullFailed(const QString &reason);

private slots:
    void runPullLoop();

private:
    bool openStream(const QString &url);
    void closeStream();
    void runReadLoop();
    QImage decodeFrameToImage();

    QString m_streamName;
    QString m_localBindAddress;
    QString m_url;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_running{false};

    struct FFmpegState;
    FFmpegState *m_ff = nullptr;
};

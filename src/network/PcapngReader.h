#ifndef PCAPNGREADER_H
#define PCAPNGREADER_H

#include <QObject>
#include <QHostAddress>
#include <QStringList>
#include <atomic>

#include "AppConfig.h"
#include "PcapPlaybackController.h"
#include "PcapPlaybackRequest.h"

// 前置声明，无需在头文件包含 pcap.h
struct pcap;
typedef struct pcap pcap_t;

class PcapngReader : public QObject {
    Q_OBJECT
public:
    explicit PcapngReader(QObject *parent = nullptr);
    ~PcapngReader();

    // 核心控制
    void stopReading();
    bool isStopRequested() const { return m_stopRequested.load(); }
    void setPlaybackConfig(const PcapPlaybackConfig &config);
    void setPaused(bool paused);
    bool isPaused() const;
    void setPlaybackSpeed(double speed);
    double playbackSpeed() const;
    double lastReleasedTimestamp() const;

public slots:
    // 外部调用此槽函数开始批量任务
    void startAnalysis(const QStringList &files);
    void startPlayback(const PcapPlaybackRequest &request);

signals:
    void udpPacket(QHostAddress ip, quint16 port, QByteArray payload, double timestamp);
    void udpPacketWithGeneration(QHostAddress ip, quint16 port, QByteArray payload,
                                 double timestamp, quint64 timelineGeneration);
    void captureRangeReady(double firstTimestamp, double lastTimestamp);
    void playbackTimestampChanged(double timestamp);
    void seekPrerollChanged(bool active, double targetTimestamp);
    void seekCompleted(double requestedTimestamp, double actualTimestamp);
    void playbackError(const QString &message);
    void finished(); // 整个列表处理完或停止时发射

private:
    bool open(const QString &file);
    void close();
    bool scanCaptureRange(const QStringList &files,
                          double &firstTimestamp,
                          double &lastTimestamp);
    void readLoop(const PcapPlaybackRequest &request,
                  bool &seekReached,
                  bool &prerollActive);

    pcap_t* m_handle = nullptr;
    std::atomic<bool> m_stopRequested{false};
    PcapPlaybackConfig m_playbackConfig;
    PcapPlaybackController m_playbackController;
};

#endif

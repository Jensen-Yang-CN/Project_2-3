#pragma once
#include <QMainWindow>
#include <QThread>
#include <QToolButton>
#include <QImage>
#include <QMetaObject>
#include "PcapngReader.h"
#include "LivePcapCapture.h"
#include "UdpSocketReceiver.h"
#include "UdpDispatcher.h"
#include "rtpparser.h"
#include "VideoWorker.h"
#include "videowidget.h"
#include "RsLidarParser.h"
#include "IMUParser.h"
#include "Ls400Parser.h"
#include "RtpParserH264.h"
#include "RsLidarWorker.h"
#include "Ls400Worker.h"
#include "ImuWorker.h"
#include "imuwidget.h"
#include "MultiLidarWidget.h"
#include "QDebugStream.h"
#include "RtspPullWorker.h"
#ifdef ENABLE_SLAM
#include "DloSlamNode.h"
#endif
#ifdef ENABLE_OCTOMAP
#include "OctoMapNode.h"
#endif
#include "PerceptionExportNode.h"
#include <QVector>
#include <memory>

#include "common/pointxyz.h"
#include "CommunicationManager_v2.h"
#include "common_types_extended.h"
#ifdef ENABLE_SLAM
#include "common/slam_types.h"
#endif

class QPlainTextEdit;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onStopBtnClicked();
    void onOnlineBtnClicked();
    void onLoadBtnClicked();
    void onPathplanClicked();
    void onBerthDetectionClicked();
    void onTopViewClicked();
    void onSlamMapClicked();
    void onSlamBuildClicked();
    void onSaveSlamMapClicked();
    void onLoadSlamMapClicked();
    void onExportClicked();
    void ondeviceQueryClicked();

    void onBridgeCloud210(const QVector<M_PointXYZI> &cloud, double ts);
    void onBridgeCloud211(const QVector<M_PointXYZI> &cloud, double ts);
    void onBridgeCameraFrame(const QImage &image);
    void onBridgeCameraFrameRight(const QImage &image);
    void onFusedCloudToBus(const QVector<M_PointXYZI> &cloud, double ts);
    void onImuPublishToBus(const IMUParsedData &data);

private:
    void initSlamNode();
    void stopSlamNode();
    void prepareHistoricalMapForExport(const QString &mapPath);
    void loadPendingHistoricalMapForExport();
    void ensureOctoMapNode();
    void ensureExportThread();
    void startPerceptionExport();
    void stopPerceptionExport();
    void pcapngRead();
    void liveCaptureInit();
    void udpSocketInit();
    void stopOfflineCapture();
    void stopLiveCapture();
    void ipPortDispatch();
    void videoProcess();
    void lidarProcess();
    void ls400Process();
    void imuProcess();
    void stopAllThreads();
    void setupFullLogging(QPlainTextEdit* edit);
    void appendDetectionLog(const QString &line);
    void setupCommBus();
    void startAlgorithmPipeline();
    void stopAlgorithmPipeline();
    void subscribeBerthResultBus();
    void unsubscribeBerthResultBus();
    void subscribeSlamResultBus();
    void unsubscribeSlamResultBus();
    void syncPathPlanButton();
    void resetVideoRtpParsers();
    void initRtspPull();   // 创建 RTSP 拉流线程
    void startRtsp();      // 在线模式：PLAY 两路固连相机
    void stopRtsp();
    void setH265PcapVideoEnabled(bool enabled); // 切换离线 pcap / 在线 RTSP 解码

private:
    Ui::MainWindow *ui;
    QToolButton *onlineBtn;
    QToolButton *pathPlanBtn;
    QToolButton *berthDetectBtn;
    QToolButton *topViewBtn;
    QToolButton *slamMapBtn;
    QToolButton *stopBtn;
    QToolButton *loadBtn;
    QToolButton *slamBuildBtn;
    QToolButton *slamSaveMapBtn;
    QToolButton *slamLoadMapBtn;
    QToolButton *exportBtn;
    QToolButton *deviceQueryBtn;

    PcapngReader *reader = nullptr;
    QThread *readerThread = nullptr;
    LivePcapCapture *liveCapture = nullptr;
    QThread *liveCaptureThread = nullptr;
    UdpSocketReceiver *udpReceiver = nullptr;
    QThread *udpReceiverThread = nullptr;
    UdpDispatcher *udpDispatcher = nullptr;
    bool m_liveCaptureWired = false;
    bool m_udpSocketWired = false;
    bool m_offlineRunning = false;
    bool m_liveRunning = false;
    RtpParser *rtpParser1 = nullptr;
    RtpParser *rtpParser2 = nullptr;
    RtpParserH264*  rtpParserH264Left = nullptr;
    RtpParserH264*  rtpParserH264Right = nullptr;

    VideoWorker *h265WorkerLeft = nullptr;
    QThread *h265ThreadLeft = nullptr;

    VideoWorker *h265WorkerRight = nullptr;
    QThread *h265ThreadRight = nullptr;

    VideoWorker *h264WorkerLeft = nullptr;
    QThread *h264ThreadLeft = nullptr;
    VideoWidget *h264videoLeft = nullptr;

    VideoWorker *h264WorkerRight = nullptr;
    QThread *h264ThreadRight = nullptr;
    VideoWidget *h264videoRight = nullptr;

    RtspPullWorker *rtspWorkerRight = nullptr;
    QThread *rtspThreadRight = nullptr;
    RtspPullWorker *rtspWorkerLeft = nullptr;
    QThread *rtspThreadLeft = nullptr;
    bool m_rtspRunning = false;
    /** 离线 pcap：RtpParser → VideoWorker；在线 RTSP 时临时 disconnect */
    QMetaObject::Connection m_connRtp1ToWorker;
    QMetaObject::Connection m_connRtp2ToWorker;

    RsLidarParser* rsLidarParser213 = nullptr;
    RsLidarParser* rsLidarParser214 = nullptr;
    IMUParser* imuparser = nullptr;

    Ls400Parser* ls400Parser210 = nullptr;
    Ls400Worker* ls400Worker210 = nullptr;
    QThread* ls400Thread210 = nullptr;
    Ls400Parser* ls400Parser211 = nullptr;
    Ls400Worker* ls400Worker211 = nullptr;
    QThread* ls400Thread211 = nullptr;

    RsLidarWorker* rs213Worker = nullptr;
    QThread *rs213Thread = nullptr;

    RsLidarWorker* rs214Worker = nullptr;
    QThread *rs214Thread = nullptr;

    MultiLidarWidget* multiLidarWidget = nullptr;
    ImuWorker* imuWorker = nullptr;
    ImuWidget* imuWidget = nullptr;
    QThread* imuThread = nullptr;

    QDebugStream *m_outStream = nullptr;
    QDebugStream *m_errStream = nullptr;

    /** 桥梁与泊位检测独立开关；SLAM 不依赖这两个开关。 */
    bool m_bridgeDetectionEnabled = false;
    bool m_berthDetectionEnabled = false;
    bool m_bridgeLogConnected = false;
    bool m_berthLogConnected = false;
    bool m_berthBusConnected = false;
    bool m_bridgeOverlayConnected = false;
    bool m_slamBusConnected = false;
    bool m_slamRunning = false;
    bool m_exportRunning = false;
    double m_lastLidarTsForSync = 0.0;

    std::shared_ptr<usv::StateTopic<usv::BerthMeasureResult>> m_berthResultTopic;
    uint64_t m_berthBusSubId = 0;
#ifdef ENABLE_SLAM
    usv::DloSlamNode *slam_node_ = nullptr;
    std::shared_ptr<usv::StateTopic<usv::SlamOdometryState>> m_slamOdomTopic;
    std::shared_ptr<usv::StateTopic<usv::SlamKeyframe>> m_slamKeyframeTopic;
    std::shared_ptr<usv::StateTopic<usv::SlamScanCloudMessage>> m_slamScanTopic;
    uint64_t m_slamOdomBusSubId = 0;
    uint64_t m_slamKeyframeBusSubId = 0;
    uint64_t m_slamScanBusSubId = 0;
    QString pending_historical_manifest_path_;
    quint64 historical_map_load_generation_ = 0;
#endif
#ifdef ENABLE_OCTOMAP
    usv::OctoMapNode *octomap_node_ = nullptr;
    QThread *octomap_thread_ = nullptr;
#endif
    QThread *export_thread_ = nullptr;
    PerceptionExportNode *export_node_ = nullptr;
    QMetaObject::Connection m_exportBerthConnection;
    bool m_exportBerthConnected = false;

    QPlainTextEdit *m_detectionLogEdit = nullptr;
    QPlainTextEdit *m_usvStateEdit = nullptr;
};

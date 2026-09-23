#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "PcapngReader.h"
#include "radarfusionmanager.h"
#include "dialogdevicemanager.h"
#include "CommBusManager.h"
#include "PcapDeviceDialog.h"
#ifdef ENABLE_SLAM
#include "common/slam_types.h"
#include "SlamMapLoadTask.h"
#endif
#include "BerthDetectionNode.h"
#include "BridgeDetectionNode.h"
#include "SlamMapBinaryIO.h"
#include "AppConfig.h"
#include "CalibrationConfig.h"
#include "SystemFileLogger.h"
#include "devicemanager.h"

#include <QDateTime>
#include <QVariant>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QDebug>
#include <QtConcurrent/QtConcurrent>
#include <QPlainTextEdit>
#include <QToolButton>
#include <QDialog>
#include <QDebug>
#include <QTimer>
#include <QSignalBlocker>
#include <QInputDialog>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QDir>
#include <QPointer>
#include <cmath>
extern "C" {
#include <libavutil/log.h>
}

namespace {

DeviceInfo requireDevice(const QString &name)
{
    const DeviceInfo dev = DeviceManager::instance().deviceByName(name);
    if (dev.name.isEmpty())
        qCritical() << "[配置] devices.json 缺少设备:" << name;
    return dev;
}

void styleToolbarButton(QToolButton *btn)
{
    btn->setStyleSheet(QStringLiteral("background: transparent; border: none;"));
    btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    btn->setMinimumHeight(80);
    btn->setMinimumWidth(72);
}

}  // namespace

// 1、--- 关键修复：在这里定义静态指针 ---
static QPlainTextEdit* s_logEditor = nullptr;

// 2. Qt 消息拦截器函数（处理 qDebug, qWarning 等）
//void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
//    if (s_logEditor) {
//        QString time = QDateTime::currentDateTime().toString("hh:mm:ss");
//        QString finalMsg = QString("[%1][Qt] %2").arg(time, msg);

//        // 跨线程安全更新 UI
//        QMetaObject::invokeMethod(s_logEditor, "appendPlainText", Q_ARG(QString, finalMsg));
//    }
//}
void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
    // 1. 无论开关如何，先格式化时间
    QString time = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString finalMsg = QString("[%1][Qt] %2").arg(time, msg);

    // 2. 【核心】：无论 UI 开关是否打开，控制台必须打印 (Stderr 不走 Qt 拦截逻辑，安全)
    fprintf(stderr, "%s\n", finalMsg.toLocal8Bit().constData());
    fflush(stderr);

    // 3. 【拦截开关】：只有 s_logEditor 被赋值（开关打开）时才发往 UI
    if (s_logEditor) {
        // 跨线程安全更新 UI
        QMetaObject::invokeMethod(s_logEditor, "appendPlainText",
                                  Qt::QueuedConnection,
                                  Q_ARG(QString, finalMsg));
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
#ifdef ENABLE_SLAM
    qRegisterMetaType<usv::SlamKeyframe>("usv::SlamKeyframe");
    qRegisterMetaType<usv::SlamOdometryState>("usv::SlamOdometryState");
#endif
    //ui->dataPanel1->setPanelInfo("导航信息");
    ui->dataPanel2->setPanelInfo("检测信息");
    m_detectionLogEdit = new QPlainTextEdit(ui->dataPanel2);
    m_detectionLogEdit->setReadOnly(true);
    m_detectionLogEdit->setMaximumBlockCount(60);
    m_detectionLogEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_detectionLogEdit->setPlaceholderText(tr("桥洞检测算法输出将显示于此"));
    ui->dataPanel2->setCentralWidget(m_detectionLogEdit);

    ui->dataPanel3->setPanelInfo("USV位置姿态");
    m_usvStateEdit = new QPlainTextEdit();
    m_usvStateEdit->setReadOnly(true);
    m_usvStateEdit->setPlaceholderText(tr("USV 当前位置与姿态将显示于此"));
    ui->dataPanel3->setCentralWidget(m_usvStateEdit);

    ui->dataPanel4->setPanelInfo("设备状态信息");
    // 1. 创建 PlainTextEdit，注意 parent 可以先传 nullptr
    QPlainTextEdit *edit = new QPlainTextEdit();
    // 2. 设置它自身的自适应策略
    edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    edit->setObjectName("logEditor");
    // 3. 通过接口塞进 datapanel4
    ui->dataPanel4->setCentralWidget(edit);
    setupFullLogging(edit);
    connect(ui->openGLWidget, &MultiLidarWidget::usvStateUpdated,
            this, [this](const QString &text) {
                if (m_usvStateEdit)
                    m_usvStateEdit->setPlainText(text);
            });
    // toolBar：等宽自适应布局，避免 QToolBar 折叠隐藏按钮
    onlineBtn = new QToolButton(this);
    styleToolbarButton(onlineBtn);
    onlineBtn->setIcon(QIcon(":/icon/online.png"));
    onlineBtn->setText(tr("在线"));

    loadBtn = new QToolButton(this);
    styleToolbarButton(loadBtn);
    loadBtn->setIcon(QIcon(":/icon/load.png"));
    loadBtn->setText(tr("加载离线数据回放"));

    pathPlanBtn = new QToolButton(this);
    styleToolbarButton(pathPlanBtn);
    pathPlanBtn->setIcon(QIcon(":/icon/pathplan.png"));
    pathPlanBtn->setCheckable(true);
    pathPlanBtn->setChecked(false);
    pathPlanBtn->setText(tr("桥梁检测"));

    berthDetectBtn = new QToolButton(this);
    styleToolbarButton(berthDetectBtn);
    berthDetectBtn->setIcon(QIcon(":/icon/pathplan.png"));
    berthDetectBtn->setCheckable(true);
    berthDetectBtn->setChecked(false);
    berthDetectBtn->setText(tr("泊位检测"));

    topViewBtn = new QToolButton(this);
    styleToolbarButton(topViewBtn);
    topViewBtn->setIcon(QIcon(":/icon/topview.png"));
    topViewBtn->setCheckable(true);
    topViewBtn->setText(tr("切换俯视图"));

    slamMapBtn = new QToolButton(this);
    styleToolbarButton(slamMapBtn);
    slamMapBtn->setIcon(QIcon(":/icon/topview.png"));
    slamMapBtn->setCheckable(true);
    slamMapBtn->setText(tr("SLAM地图"));

    stopBtn = new QToolButton(this);
    styleToolbarButton(stopBtn);
    stopBtn->setIcon(QIcon(":/icon/stop.png"));
    stopBtn->setText(tr("停止读取"));

    slamBuildBtn = new QToolButton(this);
    styleToolbarButton(slamBuildBtn);
    slamBuildBtn->setIcon(QIcon(":/icon/DeviceRegister.png"));
    slamBuildBtn->setCheckable(true);
    slamBuildBtn->setChecked(false);
    slamBuildBtn->setText(tr("SLAM建图"));

    slamSaveMapBtn = new QToolButton(this);
    styleToolbarButton(slamSaveMapBtn);
    slamSaveMapBtn->setIcon(QIcon(":/icon/load.png"));
    slamSaveMapBtn->setText(tr("保存地图"));

    slamLoadMapBtn = new QToolButton(this);
    styleToolbarButton(slamLoadMapBtn);
    slamLoadMapBtn->setIcon(QIcon(":/icon/load.png"));
    slamLoadMapBtn->setText(tr("加载地图"));

    exportBtn = new QToolButton(this);
    styleToolbarButton(exportBtn);
    exportBtn->setIcon(QIcon(":/icon/online.png"));
    exportBtn->setCheckable(true);
    exportBtn->setChecked(false);
    exportBtn->setText(tr("信息投递"));

    deviceQueryBtn = new QToolButton(this);
    styleToolbarButton(deviceQueryBtn);
    deviceQueryBtn->setIcon(QIcon(":/icon/deviceQuery.png"));
    deviceQueryBtn->setText(tr("设备查询"));

    //与Click事件链接
    connect(onlineBtn,&QToolButton::clicked,this,&MainWindow::onOnlineBtnClicked);
    connect(loadBtn,&QToolButton::clicked,this,&MainWindow::onLoadBtnClicked);
    connect(pathPlanBtn,&QToolButton::clicked,this,&MainWindow::onPathplanClicked);
    connect(berthDetectBtn,&QToolButton::clicked,this,&MainWindow::onBerthDetectionClicked);
    connect(topViewBtn,&QToolButton::clicked,this,&MainWindow::onTopViewClicked);
    connect(slamMapBtn,&QToolButton::clicked,this,&MainWindow::onSlamMapClicked);
    connect(slamBuildBtn,&QToolButton::clicked,this,&MainWindow::onSlamBuildClicked);
    connect(slamSaveMapBtn,&QToolButton::clicked,this,&MainWindow::onSaveSlamMapClicked);
    connect(slamLoadMapBtn,&QToolButton::clicked,this,&MainWindow::onLoadSlamMapClicked);
    connect(exportBtn,&QToolButton::clicked,this,&MainWindow::onExportClicked);
    connect(deviceQueryBtn,&QToolButton::clicked,this,&MainWindow::ondeviceQueryClicked);

    QWidget *toolbarHost = new QWidget(this);
    toolbarHost->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout *toolbarLayout = new QHBoxLayout(toolbarHost);
    toolbarLayout->setContentsMargins(8, 4, 8, 4);
    toolbarLayout->setSpacing(6);
    const QList<QToolButton *> toolbarButtons = {
        onlineBtn, loadBtn, pathPlanBtn, berthDetectBtn, topViewBtn, slamMapBtn, stopBtn,
        slamBuildBtn, slamSaveMapBtn, slamLoadMapBtn, exportBtn, deviceQueryBtn};
    for (QToolButton *btn : toolbarButtons)
        toolbarLayout->addWidget(btn, 1);

    ui->toolBar->clear();
    ui->toolBar->setMovable(false);
    ui->toolBar->setFloatable(false);
    ui->toolBar->setIconSize(QSize(40, 40));
    ui->toolBar->addWidget(toolbarHost);




    // 加载 config.json / devices.json / calibration.json
    AppConfig::instance().load();
    SystemFileLogger::instance().init(AppConfig::instance().systemLog());
    SystemFileLogger::instance().logLifecycle("Application", "start");
    CalibrationConfig::instance().load();
    DeviceManager::instance().loadConfigOrDefaults();

    //pcapng 流式读取过程在独立线程完成读取
    pcapngRead();

    //根据Ip+port分流处理
    ipPortDispatch();

    lidarProcess();
    ls400Process();
    imuProcess();
    videoProcess();
    setupCommBus();
}

MainWindow::~MainWindow() {

    SystemFileLogger::instance().logLifecycle("Application", "stop");
    qDebug() << "正在关闭系统，清理线程...";
    stopRtsp();
#ifdef ENABLE_SLAM
    stopSlamNode();
#endif
    unsubscribeBerthResultBus();
#ifdef ENABLE_OCTOMAP
    if (octomap_thread_) {
        octomap_thread_->quit();
        octomap_thread_->wait(3000);
        octomap_thread_ = nullptr;
    }
#endif
    stopPerceptionExport();
    if (export_thread_) {
        export_thread_->quit();
        export_thread_->wait(3000);
        export_thread_ = nullptr;
    }
    CommBusManager::instance().stopPipeline();
    // 1. 停止读取器（最源头的数据流）
    if (reader) {
        reader->stopReading();
    }
    if (liveCapture) {
        liveCapture->stopCapture();
    }
    if (udpReceiver && m_liveRunning) {
        QMetaObject::invokeMethod(udpReceiver, &UdpSocketReceiver::stopReceiving, Qt::QueuedConnection);
    }

    // 2. 逐个停止并回收线程 (建议写个辅助 Lambda 减少重复代码)
    auto stopThread = [](QThread* t) {
        if (t && t->isRunning()) {
            t->quit();
            if (!t->wait(2000)) { // 最多等2秒，不等就强制结束
                t->terminate();
                t->wait();
            }
            delete t;
        }
    };
    stopThread(readerThread);
    stopThread(liveCaptureThread);
    stopThread(udpReceiverThread);
    stopThread(h265ThreadLeft);
    stopThread(h265ThreadRight);
    stopThread(rtspThreadLeft);
    stopThread(rtspThreadRight);
    stopThread(h264ThreadLeft);
    stopThread(h264ThreadRight);
    stopThread(rs213Thread);
    stopThread(rs214Thread);
    stopThread(imuThread);
    stopThread(ls400Thread210);
    stopThread(ls400Thread211);
    SystemFileLogger::instance().shutdown();
    delete ui;

}
//pcapng 数据读取
void MainWindow::pcapngRead()
{
    if (readerThread) return; // 防护：防止重复创建

    readerThread = new QThread(this);
    reader = new PcapngReader();
    reader->moveToThread(readerThread);



    // --- 信号连接：UI 状态恢复 ---
    connect(reader, &PcapngReader::finished, this, [this](){
        m_offlineRunning = false;
        loadBtn->setEnabled(true);
        onlineBtn->setEnabled(true);
        stopBtn->setEnabled(false);
        qDebug() << "解析器已回到待命状态，可再次点击开始";
    });

    // --- 信号连接：手动停止控制 ---
    connect(stopBtn, &QToolButton::clicked, reader, &PcapngReader::stopReading, Qt::DirectConnection);

    // 【重要】删掉那三行关于 readerThread->quit() 和 deleteLater 的连接！！
    // 我们让线程和对象在 MainWindow 整个生命周期内一直活着。

    readerThread->start();
}

void MainWindow::liveCaptureInit()
{
    if (liveCaptureThread)
        return;

    liveCaptureThread = new QThread(this);
    liveCapture = new LivePcapCapture();
    liveCapture->moveToThread(liveCaptureThread);

    // 与离线 PcapngReader 相同：udpPacket → UdpDispatcher → 各 Parser
    if (udpDispatcher && !m_liveCaptureWired) {
        connect(liveCapture, &LivePcapCapture::udpPacket,
                udpDispatcher, &UdpDispatcher::dispatch,
                Qt::DirectConnection);
        m_liveCaptureWired = true;
    }

    connect(liveCapture, &LivePcapCapture::captureStarted,
            this, [this](const QString &deviceName) {
                m_liveRunning = true;
                appendDetectionLog(tr("在线抓包已启动：%1").arg(deviceName));
                ui->statusbar->showMessage(tr("在线抓包运行中"), 3000);
            }, Qt::QueuedConnection);

    connect(liveCapture, &LivePcapCapture::captureStopped,
            this, [this]() {
                m_liveRunning = false;
                onlineBtn->setEnabled(true);
                loadBtn->setEnabled(true);
                stopBtn->setEnabled(false);
                ui->statusbar->showMessage(tr("在线抓包已停止"), 3000);
            }, Qt::QueuedConnection);

    connect(liveCapture, &LivePcapCapture::captureError,
            this, [this](const QString &message) {
                m_liveRunning = false;
                appendDetectionLog(tr("[在线抓包] %1").arg(message));
                onlineBtn->setEnabled(true);
                loadBtn->setEnabled(true);
                stopBtn->setEnabled(false);
                ui->statusbar->showMessage(tr("在线抓包失败"), 3000);
            }, Qt::QueuedConnection);

    connect(stopBtn, &QToolButton::clicked,
            liveCapture, &LivePcapCapture::stopCapture,
            Qt::DirectConnection);

    liveCaptureThread->start();
}

void MainWindow::udpSocketInit()
{
    if (udpReceiverThread)
        return;

    udpReceiverThread = new QThread(this);
    udpReceiver = new UdpSocketReceiver();
    udpReceiver->moveToThread(udpReceiverThread);

    if (udpDispatcher && !m_udpSocketWired) {
        connect(udpReceiver, &UdpSocketReceiver::udpPacket,
                udpDispatcher, &UdpDispatcher::dispatch,
                Qt::DirectConnection);
        m_udpSocketWired = true;
        qInfo() << "[MainWindow] UdpSocketReceiver::udpPacket 已连接 UdpDispatcher::dispatch (Direct)";
    } else if (!udpDispatcher) {
        qCritical() << "[MainWindow] UdpDispatcher 未初始化，在线 Socket 收包无法分流";
    }

    connect(udpReceiver, &UdpSocketReceiver::receiveStarted,
            this, [this](const QString &summary) {
                m_liveRunning = true;
                appendDetectionLog(tr("在线 UDP 接收已启动：%1").arg(summary));
                ui->statusbar->showMessage(tr("在线 UDP 接收运行中"), 3000);
            }, Qt::QueuedConnection);

    connect(udpReceiver, &UdpSocketReceiver::receiveStopped,
            this, [this]() {
                m_liveRunning = false;
                onlineBtn->setEnabled(true);
                loadBtn->setEnabled(true);
                stopBtn->setEnabled(false);
                ui->statusbar->showMessage(tr("在线 UDP 接收已停止"), 3000);
            }, Qt::QueuedConnection);

    connect(udpReceiver, &UdpSocketReceiver::receiveError,
            this, [this](const QString &message) {
                m_liveRunning = false;
                onlineBtn->setEnabled(true);
                loadBtn->setEnabled(true);
                stopBtn->setEnabled(false);
                appendDetectionLog(tr("[在线 UDP 错误] %1").arg(message));
                ui->statusbar->showMessage(tr("在线 UDP 接收失败"), 3000);
            }, Qt::QueuedConnection);

    connect(udpReceiver, &UdpSocketReceiver::receiveWarning,
            this, [this](const QString &message) {
                appendDetectionLog(tr("[在线 UDP 警告] %1").arg(message));
            }, Qt::QueuedConnection);

    connect(stopBtn, &QToolButton::clicked,
            udpReceiver, &UdpSocketReceiver::stopReceiving,
            Qt::QueuedConnection);

    udpReceiverThread->start();
}

void MainWindow::stopOfflineCapture()
{
    if (reader)
        reader->stopReading();
    m_offlineRunning = false;
}

void MainWindow::stopLiveCapture()
{
    stopRtsp();
    if (liveCapture && m_liveRunning)
        liveCapture->stopCapture();
    if (udpReceiver && m_liveRunning) {
        QMetaObject::invokeMethod(udpReceiver, &UdpSocketReceiver::stopReceiving, Qt::QueuedConnection);
    }
}

//IP及端口分流处理
void MainWindow::ipPortDispatch()
{
    udpDispatcher = new UdpDispatcher(this);
    qInfo() << "[MainWindow] stern lidar 201 is disabled";

    const DeviceInfo dev210 = requireDevice(QStringLiteral("210"));
    const DeviceInfo dev211 = requireDevice(QStringLiteral("211"));
    const DeviceInfo dev213 = requireDevice(QStringLiteral("213"));
    const DeviceInfo dev214 = requireDevice(QStringLiteral("214"));
    const DeviceInfo devImu = requireDevice(QStringLiteral("IMU"));
    const DeviceInfo devH265R = requireDevice(QStringLiteral("h265Right"));
    const DeviceInfo devH265L = requireDevice(QStringLiteral("h265Left"));
    const DeviceInfo devH264L = requireDevice(QStringLiteral("h264Left"));
    const DeviceInfo devH264R = requireDevice(QStringLiteral("h264Right"));

    rtpParser1 = new RtpParser(QHostAddress(devH265R.ip), devH265R.port, nullptr);
    rtpParser2 = new RtpParser(QHostAddress(devH265L.ip), devH265L.port, nullptr);
    rsLidarParser213 = new RsLidarParser(QHostAddress(dev213.ip), dev213.port, nullptr);
    rsLidarParser214 = new RsLidarParser(QHostAddress(dev214.ip), dev214.port, nullptr);
    imuparser = new IMUParser(QHostAddress(devImu.ip), devImu.port, nullptr);
    ls400Parser210 = new Ls400Parser(QHostAddress(dev210.ip), dev210.port, nullptr);
    ls400Parser211 = new Ls400Parser(QHostAddress(dev211.ip), dev211.port, nullptr);
    rtpParserH264Left = new RtpParserH264(QHostAddress(devH264L.ip), devH264L.port, nullptr);
    rtpParserH264Right = new RtpParserH264(QHostAddress(devH264R.ip), devH264R.port, nullptr);

    rtpParserH264Right->setParserName(QStringLiteral("rtpParserH264Right 20082"));
    rtpParserH264Left->setParserName(QStringLiteral("rtpParserH264Left 20080"));

    udpDispatcher->addRule(QHostAddress(dev210.ip), dev210.port, ls400Parser210, &Ls400Parser::inputPacket);
    udpDispatcher->addRule(QHostAddress(dev211.ip), dev211.port, ls400Parser211, &Ls400Parser::inputPacket);
    udpDispatcher->addRule(QHostAddress(devImu.ip), devImu.port, imuparser, &IMUParser::inputPacket);

    qInfo() << "[MainWindow] UDP 分流规则(核心):"
            << QStringLiteral("210=%1:%2").arg(dev210.ip).arg(dev210.port)
            << QStringLiteral("211=%1:%2").arg(dev211.ip).arg(dev211.port)
            << QStringLiteral("IMU=%1:%2").arg(devImu.ip).arg(devImu.port);

    const QString h265RightName = devH265R.metadata.value(QStringLiteral("parser_name"), QStringLiteral("h265Right")).toString();
    const QString h265LeftName = devH265L.metadata.value(QStringLiteral("parser_name"), QStringLiteral("h265Left")).toString();
    rtpParser1->setParserName(h265RightName);
    rtpParser2->setParserName(h265LeftName);
    udpDispatcher->addRuleByIp(QHostAddress(devH265R.ip), rtpParser1, &RtpParser::inputPacket);
    udpDispatcher->addRuleByIp(QHostAddress(devH265L.ip), rtpParser2, &RtpParser::inputPacket);
    for (const QVariant &ip : devH265R.metadata.value(QStringLiteral("pcap_alias_ips")).toList())
        udpDispatcher->addRuleByIp(QHostAddress(ip.toString()), rtpParser1, &RtpParser::inputPacket);
    for (const QVariant &ip : devH265L.metadata.value(QStringLiteral("pcap_alias_ips")).toList())
        udpDispatcher->addRuleByIp(QHostAddress(ip.toString()), rtpParser2, &RtpParser::inputPacket);

    udpDispatcher->addRule(QHostAddress(devH264L.ip), devH264L.port, rtpParserH264Left, &RtpParserH264::inputPacket);
    udpDispatcher->addRule(QHostAddress(devH264R.ip), devH264R.port, rtpParserH264Right, &RtpParserH264::inputPacket);

    if (rsLidarParser213->start() &&
            rsLidarParser214->start())
    {
        qDebug() << "所有解析器已就绪，正在开启 UDP 数据流...";

        udpDispatcher->addRule(QHostAddress(dev213.ip), dev213.port, rsLidarParser213, &RsLidarParser::inputPacket);
        if (dev213.extraPort > 0)
            udpDispatcher->addRule(QHostAddress(dev213.ip), dev213.extraPort, rsLidarParser213, &RsLidarParser::inputPacket);

        udpDispatcher->addRule(QHostAddress(dev214.ip), dev214.port, rsLidarParser214, &RsLidarParser::inputPacket);
        if (dev214.extraPort > 0)
            udpDispatcher->addRule(QHostAddress(dev214.ip), dev214.extraPort, rsLidarParser214, &RsLidarParser::inputPacket);
    }
    else
    {
        qCritical() << "雷达启动失败！请检查 IP 配置或网线连接。";
    }
}
//H265，H264 视频处理

void MainWindow::lidarProcess()
{
    //192.168.1.213 Rs32线雷达

    rs213Worker = new RsLidarWorker();
    rs213Thread = new QThread(this);   // 传入 this 方便随主窗口销毁
    // 2. 将 Worker 移动到独立工作线程
    rs213Worker->moveToThread(rs213Thread);
    //3.--- 关键链路 2: Worker (rs213Thread) -> Widget (UI线程) ---
    connect(rsLidarParser213, &RsLidarParser::frameReady,
            rs213Worker, &RsLidarWorker::handleFrame,
            Qt::DirectConnection);
    //    connect(rs213Worker, &RsLidarWorker::cloudReady,
    //            ui->openGLWidget, &MultiLidarWidget::updateLsCloud,
    //            Qt::QueuedConnection);
    //4. 资源清理连接：线程停止时自动清理
    connect(rs213Thread, &QThread::finished, rs213Worker, &QObject::deleteLater);
    // 5. 启动工作线程
    rs213Thread->start();

    //192.168.1.214 Rs32线雷达
    rs214Worker = new RsLidarWorker();
    rs214Thread = new QThread(this);   // 传入 this 方便随主窗口销毁
    // 2. 将 Worker 移动到独立工作线程
    rs214Worker->moveToThread(rs214Thread);
    //3.--- 关键链路 2: Worker (rs213Thread) -> Widget (UI线程) ---
    connect(rsLidarParser214, &RsLidarParser::frameReady,
            rs214Worker, &RsLidarWorker::handleFrame,
            Qt::DirectConnection);
    //    connect(rs214Worker, &RsLidarWorker::cloudReady,
    //            ui->openGLWidget, &MultiLidarWidget::updateLsCloud,
    //            Qt::QueuedConnection);
    //4. 资源清理连接：线程停止时自动清理
    connect(rs214Thread, &QThread::finished, rs214Worker, &QObject::deleteLater);
    // 5. 启动工作线程
    rs214Thread->start();

}
//LS400线雷达处理
void MainWindow::ls400Process()
{
    // 1. 初始化成员变量
    ls400Worker210 = new Ls400Worker();
    ls400Thread210 = new QThread(this);   // 传入 this 方便随主窗口销毁

    // 2. 将 Worker 移动到独立工作线程
    ls400Worker210->moveToThread(ls400Thread210);

    // --- 关键链路 1: Parser (Pcap读取线程) -> 210 Worker ---
    // 由于 Parser 和 Worker 在不同线程，必须用 QueuedConnection
    connect(ls400Parser210, &Ls400Parser::frameReady,
            ls400Worker210, &Ls400Worker::handleFrame,
            Qt::DirectConnection);

    //    connect(ls400Worker210, &Ls400Worker::cloudReady,
    //            ui->openGLWidget, &MultiLidarWidget::updateLsCloud,
    //            Qt::QueuedConnection);


    // 3. 资源清理连接：线程停止时自动清理 Worker (可选，如果 Worker 不是 Widget 的子对象)
    connect(ls400Thread210, &QThread::finished, ls400Worker210, &QObject::deleteLater);

    // 4. 启动工作线程
    ls400Thread210->start();


    //211雷达
    // 1. 初始化成员变量
    ls400Worker211 = new Ls400Worker();
    ls400Thread211 = new QThread(this);   // 传入 this 方便随主窗口销毁

    // 2. 将 Worker 移动到独立工作线程
    ls400Worker211->moveToThread(ls400Thread211);

    // --- 关键链路 1: Parser (Pcap读取线程) -> 211 Worker ---
    // 由于 Parser 和 Worker 在不同线程，必须用 QueuedConnection
    connect(ls400Parser211, &Ls400Parser::frameReady,
            ls400Worker211, &Ls400Worker::handleFrame,
            Qt::DirectConnection);

    //    connect(ls400Worker211, &Ls400Worker::cloudReady,
    //            ui->openGLWidget, &MultiLidarWidget::updateLsCloud,
    //            Qt::QueuedConnection);


    // 3. 资源清理连接：线程停止时自动清理 Worker (可选，如果 Worker 不是 Widget 的子对象)
    connect(ls400Thread211, &QThread::finished, ls400Worker211, &QObject::deleteLater);

    // 4. 启动工作线程
    ls400Thread211->start();

    // 四个启用雷达的空间、时间融合（201 船尾雷达已停用）
    RadarFusionManager *radarFusionmanager = new RadarFusionManager();

    connect(rs213Worker, &RsLidarWorker::cloudReady, radarFusionmanager, [=](const QVector<M_PointXYZI> &cloud, double ts){
        radarFusionmanager->handleRadarCloud("213", cloud, ts);
    });
    connect(rs214Worker, &RsLidarWorker::cloudReady, radarFusionmanager, [=](const QVector<M_PointXYZI> &cloud, double ts){
        radarFusionmanager->handleRadarCloud("214", cloud, ts);
    });
    connect(ls400Worker211, &Ls400Worker::cloudReady, radarFusionmanager, [=](const QVector<M_PointXYZI> &cloud, double ts){
        radarFusionmanager->handleRadarCloud("211", cloud, ts);
    });
    connect(ls400Worker210, &Ls400Worker::cloudReady, radarFusionmanager, [=](const QVector<M_PointXYZI> &cloud, double ts){
        radarFusionmanager->handleRadarCloud("210", cloud, ts);
    });

    // 合并后显示：同线程 Direct，避免整帧再堆进 GUI 事件队列
    connect(radarFusionmanager, &RadarFusionManager::fusedCloudReady,
            ui->openGLWidget, &MultiLidarWidget::updateFusedCloud, Qt::DirectConnection);
    connect(radarFusionmanager, &RadarFusionManager::fusedCloudReady,
            this, &MainWindow::onFusedCloudToBus, Qt::DirectConnection);

    connect(ls400Worker210, &Ls400Worker::cloudReady,
            this, &MainWindow::onBridgeCloud210, Qt::QueuedConnection);
    connect(ls400Worker211, &Ls400Worker::cloudReady,
            this, &MainWindow::onBridgeCloud211, Qt::QueuedConnection);
}

void MainWindow::imuProcess()
{
    // 1. 注册自定义结构体 (跨线程信号槽的通行证)
    qRegisterMetaType<IMUParsedData>("IMUParsedData");
    qRegisterMetaType<QByteArray>("QByteArray");

    // 2. 初始化 Worker 和 Thread (不要给 Worker 传 this)
    imuWorker = new ImuWorker();
    imuThread = new QThread(this); // Thread 由 MainWindow 自动回收
    imuWorker->moveToThread(imuThread);

    // 3. 设置 UI (dataPanel1 已经在 Designer 中提升为 ImuWidget)
    //ui->dataPanel1->setPanelInfo("导航信息 (IMU)");

    // 4. 建立信号槽“接线”

    // [数据输入] Parser -> Worker
    connect(imuparser, &IMUParser::navPayloadReady,
            imuWorker, &ImuWorker::processNavPacket,
            Qt::QueuedConnection);

    // [数据输出] Worker -> ImuWidget (dataPanel1)
    connect(imuWorker, &ImuWorker::imuDataReady,
            ui->dataPanel1, &ImuWidget::updateNavigationData,
            Qt::QueuedConnection);
    connect(imuWorker, &ImuWorker::imuDataReady,
                ui->openGLWidget, &MultiLidarWidget::onNewTrajectoryPoint);
    connect(imuWorker, &ImuWorker::imuDataReady,
            this, &MainWindow::onImuPublishToBus,
            Qt::QueuedConnection);

    // [安全销毁] 线程结束时自动清理 Worker 内存
    connect(imuThread, &QThread::finished, imuWorker, &QObject::deleteLater);
    // 5. 启动线程进入事件循环
    imuThread->start();
}

void MainWindow::setupFullLogging(QPlainTextEdit *edit)
{
    // A. 捕获 qDebug(), qWarning(), qCritical()
    s_logEditor = edit;
    //qDebug()<<"控制流流向了这里";
    qInstallMessageHandler(myMessageOutput);

    // B. 捕获 std::cout 和 printf (部分情况)
    m_outStream = new QDebugStream(std::cout, edit);

    // C. 捕获 std::cerr 和 运行时报错 (Runtime Errors)
    m_errStream = new QDebugStream(std::cerr, edit);

//    std::cout << "标准输出已重定向." << std::endl;
//    std::cerr << "标准错误已重定向." << std::endl;

}

void MainWindow::appendDetectionLog(const QString &line)
{
    if (!m_detectionLogEdit)
        return;

    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));
    const QString row = QStringLiteral("[%1] %2").arg(ts, line);
    QMetaObject::invokeMethod(m_detectionLogEdit, "appendPlainText", Qt::QueuedConnection,
                              Q_ARG(QString, row + QLatin1Char('\n')));
}


void MainWindow::onBridgeCloud210(const QVector<M_PointXYZI> &cloud, double ts)
{
    if (!m_bridgeDetectionEnabled)
        return;
    if (auto *pub = CommBusManager::instance().sensorPublisher())
        pub->publishLidar210(cloud, ts);
}

void MainWindow::onBridgeCloud211(const QVector<M_PointXYZI> &cloud, double ts)
{
    m_lastLidarTsForSync = ts;
    if (!m_bridgeDetectionEnabled)
        return;
    if (auto *pub = CommBusManager::instance().sensorPublisher())
        pub->publishLidar211(cloud, ts);
}

void MainWindow::onBridgeCameraFrame(const QImage &image)
{
    if (image.isNull())
        return;

    const double cameraMetricTs = (m_lastLidarTsForSync > 0.0)
        ? m_lastLidarTsForSync
        : static_cast<double>(QDateTime::currentMSecsSinceEpoch()) / 1000.0;

    if (m_lastLidarTsForSync > 0.0) {
        ui->openGLWidgetLeft->updateFrame(image, m_lastLidarTsForSync);
        if (m_bridgeDetectionEnabled) {
            if (auto *pub = CommBusManager::instance().sensorPublisher())
                pub->publishCameraLeft(image, m_lastLidarTsForSync);
        }
    } else {
        ui->openGLWidgetLeft->updateFrame(image, cameraMetricTs);
        if (m_bridgeDetectionEnabled) {
            if (auto *pub = CommBusManager::instance().sensorPublisher())
                pub->publishCameraLeft(image, cameraMetricTs);
        }
    }

    if (m_bridgeDetectionEnabled) {
        if (auto *node = CommBusManager::instance().bridgeNode())
            node->submitCameraFrame(image, cameraMetricTs, false);
    }
}

void MainWindow::onBridgeCameraFrameRight(const QImage &image)
{
    if (image.isNull())
        return;

    const double timestampSec = static_cast<double>(QDateTime::currentMSecsSinceEpoch()) / 1000.0;
    ui->openGLWidgetRight->updateFrame(image, timestampSec);
    if (m_bridgeDetectionEnabled) {
        if (auto *node = CommBusManager::instance().bridgeNode())
            node->submitCameraFrame(image, timestampSec, true);
    }
}

void MainWindow::onFusedCloudToBus(const QVector<M_PointXYZI> &cloud, double ts)
{
    CommBusManager::instance().ensurePublisher();
    if (auto *pub = CommBusManager::instance().sensorPublisher()) {
        pub->publishFusedSensors(cloud, ts, m_berthDetectionEnabled, true);
    }
}

void MainWindow::onImuPublishToBus(const IMUParsedData &data)
{
#ifdef ENABLE_SLAM
    // DLO SLAM 使用首个有效 GNSS 建立本次会话 ENU 原点。及时把同一锚点
    // 送给本机地图显示，避免历史地图与实时地图只同向但原点平移不一致。
    if (slam_node_ && ui && ui->openGLWidget)
        ui->openGLWidget->setRealtimeSlamGeoAnchor(slam_node_->getGeoAnchor());
#endif
    CommBusManager::instance().ensurePublisher();
    if (auto *pub = CommBusManager::instance().sensorPublisher())
        pub->publishImu(data);
    if (auto *node = CommBusManager::instance().berthNode())
        node->feedNavigationData(data);

    // 离线回放时，投递节点位于独立 QThread。它自己的 QTimer 成员
    // 不能可靠地跨线程轮询 /sensor/imu/raw，因此实时泊位投递不能只
    // 依赖总线轮询。沿着与泊位结果相同的线程安全桥，直接把有效 IMU
    // 排队送到投递节点；历史地图/泊位编码逻辑不受影响。
    if (m_exportRunning && export_node_) {
        PerceptionExportNode *node = export_node_;
        const IMUParsedData copy = data;
        QMetaObject::invokeMethod(
            node,
            [node, copy]() {
                if (node->isRunning())
                    node->feedImu(copy);
            },
            Qt::QueuedConnection);
    }
}

void MainWindow::initSlamNode()
{
#ifdef ENABLE_SLAM
    if (slam_node_)
        return;

    CommBusManager::instance().ensurePublisher();

    slam_node_ = new usv::DloSlamNode(this);
    const SlamAppConfig &slamCfg = AppConfig::instance().slam();
    usv::DloSlamConfig slam_config;
    slam_config.max_keyframes = 1000;
    slam_config.max_keyframe_age_sec = 3600.0;
    slam_config.min_range = 2.0;
    slam_config.max_range = 100.0;
    slam_config.voxel_leaf_size = slamCfg.voxel_leaf_size;
    slam_config.keyframe_translation_thresh = 0.5;
    slam_config.keyframe_rotation_thresh_deg = 5.0;
    slam_config.gicp_num_threads = 4;
    slam_config.require_gnss_ins = true;
    slam_config.max_gnss_sync_sec = slamCfg.max_gnss_sync_sec;

    if (!slam_node_->init(slam_config)) {
        appendDetectionLog(tr("DLO SLAM 节点初始化失败"));
        delete slam_node_;
        slam_node_ = nullptr;
        return;
    }

    connect(slam_node_, &usv::DloSlamNode::logMessage,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);

    // 检测结果经总线：UI 订阅 slam/* ，不再 connect 结果信号
    subscribeSlamResultBus();

#ifdef ENABLE_OCTOMAP
    ensureOctoMapNode();
    if (octomap_node_) {
        // OctoMap 自行订阅 slam/keyframe、slam/odometry
        connect(octomap_node_, &usv::OctoMapNode::occupancyMapReady,
                ui->openGLWidget, &MultiLidarWidget::updateSlamOccupancyGrid,
                Qt::QueuedConnection);
        QMetaObject::invokeMethod(
            octomap_node_,
            [this]() {
                octomap_node_->clearMap();
                octomap_node_->start();
            },
            Qt::QueuedConnection);
        if (slamMapBtn && !slamMapBtn->isChecked()) {
            QSignalBlocker blocker(slamMapBtn);
            slamMapBtn->setChecked(true);
            slamMapBtn->setText(tr("实时点云"));
        }
        if (ui->openGLWidget)
            ui->openGLWidget->setSlamMapMode(true);
    }
#endif

    slam_node_->start();
    m_slamRunning = true;
    appendDetectionLog(tr("DLO SLAM 节点已启动（结果经总线 slam/odometry|keyframe|scan_cloud）"));
#ifdef ENABLE_OCTOMAP
    appendDetectionLog(tr("SLAM 地图显示已切换为 OctoMap 占据栅格（线框方块）"));
#endif
    if (m_exportRunning)
        startPerceptionExport();
#else
    appendDetectionLog(tr("SLAM 未编译（需要 PCL，请确认 CMake 中 ENABLE_SLAM 与 PCL 路径）"));
#endif
}

void MainWindow::ensureOctoMapNode()
{
#ifdef ENABLE_OCTOMAP
    if (octomap_node_)
        return;

    octomap_thread_ = new QThread(this);
    octomap_node_ = new usv::OctoMapNode();

    usv::OctoMapNode::Config cfg;
    cfg.resolution_m = 0.2;
    cfg.max_range_m = 50.0;
    cfg.log_every_n_keyframes = 5;
    cfg.display_every_n_keyframes = 1;
    cfg.display_resolution_m = 0.5;
    cfg.display_occ_prob_min = 0.55f;
    cfg.max_display_voxels = 150000;   // 全量显示时提高上限，减少抽稀
    cfg.insert_raycast_free = false;   // 仅 occupied，抑制 free 叶子暴涨
    cfg.local_map_radius_m = 0.0;      // 不裁剪：树内保留全量
    cfg.crop_every_n_keyframes = 10;
    cfg.prune_every_n_keyframes = 20;
    cfg.display_radius_m = 0.0;        // 0=全图导出显示（历史栅格都画）
    if (!octomap_node_->init(cfg)) {
        appendDetectionLog(tr("OctoMap 节点初始化失败"));
        delete octomap_node_;
        octomap_node_ = nullptr;
        delete octomap_thread_;
        octomap_thread_ = nullptr;
        return;
    }

    octomap_node_->moveToThread(octomap_thread_);
    connect(octomap_thread_, &QThread::finished, octomap_node_, &QObject::deleteLater);
    connect(octomap_node_, &QObject::destroyed, this, [this]() { octomap_node_ = nullptr; });
    connect(octomap_node_, &usv::OctoMapNode::logMessage,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);

    octomap_thread_->start();
    appendDetectionLog(tr("OctoMap 节点已就绪（仅 occupied；全图显示与存盘）"));
#endif
}

void MainWindow::stopSlamNode()
{
#ifdef ENABLE_SLAM
    if (!slam_node_)
        return;

    unsubscribeSlamResultBus();

#ifdef ENABLE_OCTOMAP
    if (octomap_node_) {
        if (ui->openGLWidget)
            disconnect(octomap_node_, nullptr, ui->openGLWidget, nullptr);
        QMetaObject::invokeMethod(
            octomap_node_,
            [this]() { octomap_node_->stop(); },
            Qt::BlockingQueuedConnection);
    }
#endif

    if (export_node_) {
        QMetaObject::invokeMethod(
            export_node_,
            [this]() { export_node_->clearSlamPoseCache(); },
            Qt::BlockingQueuedConnection);
    }

    slam_node_->stop();
    delete slam_node_;
    slam_node_ = nullptr;
    m_slamRunning = false;

    if (ui->openGLWidget) {
        ui->openGLWidget->clearSlamMap();
        ui->openGLWidget->setSlamMapMode(false);
    }
    if (slamMapBtn) {
        QSignalBlocker blocker(slamMapBtn);
        slamMapBtn->setChecked(false);
        slamMapBtn->setText(tr("SLAM地图"));
    }
    appendDetectionLog(tr("SLAM 建图已停止，地图已清屏"));
#endif
}

#ifdef ENABLE_SLAM
void MainWindow::prepareHistoricalMapForExport(const QString &mapPath)
{
    ++historical_map_load_generation_;
    const quint64 generation = historical_map_load_generation_;
    pending_historical_manifest_path_.clear();

    // 选择新地图后，立即停止旧历史地图的投递，避免新旧地图混发。
    if (export_node_) {
        QMetaObject::invokeMethod(
            export_node_, &PerceptionExportNode::clearHistoricalMap,
            Qt::BlockingQueuedConnection);
    }

    const SlamTileMapAppConfig &tileConfig =
        AppConfig::instance().slam().offline_tile_map;
    slam_tile::BuildOptions tileOptions;
    tileOptions.tile_size_m = tileConfig.tile_size_m;
    tileOptions.lods.clear();
    for (int level = 0; level < tileConfig.lod_voxel_sizes_m.size(); ++level) {
        const double voxelSize = tileConfig.lod_voxel_sizes_m.at(level);
        if (std::isfinite(voxelSize) && voxelSize > 0.0)
            tileOptions.lods.append({level, voxelSize});
    }
    if (tileOptions.lods.isEmpty())
        tileOptions.lods = {{0, 1.0}, {1, 0.3}, {2, 0.1}};

    const QString cacheRoot = QDir(slam_map_io::defaultMapsDirectory())
                                  .filePath(QStringLiteral("tiled_cache"));
    const QPointer<MainWindow> self(this);
    appendDetectionLog(tr("[历史地图投递] 正在准备分块清单…"));

    QtConcurrent::run([self, mapPath, cacheRoot, tileOptions, generation]() {
        if (!self)
            return;
        const auto result = std::make_shared<slam_map_load::Result>(
            slam_map_load::loadAndPrepareTiles(
                {mapPath}, cacheRoot, slam_map_stitch::Options{}, tileOptions));
        if (!self)
            return;
        QMetaObject::invokeMethod(
            self.data(),
            [self, result, generation]() {
                if (!self || self->historical_map_load_generation_ != generation)
                    return;
                if (!result->success || !result->tile_build.success
                    || result->tile_build.manifest_path.isEmpty()) {
                    self->appendDetectionLog(
                        self->tr("[历史地图投递] 分块清单准备失败: %1")
                            .arg(result->error));
                    return;
                }

                self->pending_historical_manifest_path_ =
                    result->tile_build.manifest_path;
                self->appendDetectionLog(
                    self->tr("[历史地图投递] 已准备 %1 个 Tile、%2 个历史泊位")
                        .arg(result->tile_build.manifest.tiles.size())
                        .arg(result->tile_build.manifest.berths.size()));
                if (self->m_exportRunning)
                    self->loadPendingHistoricalMapForExport();
            },
            Qt::QueuedConnection);
    });
}

void MainWindow::loadPendingHistoricalMapForExport()
{
    if (!export_node_ || pending_historical_manifest_path_.isEmpty())
        return;

    const QString manifestPath = pending_historical_manifest_path_;
    PerceptionExportNode *node = export_node_;
    bool loaded = false;
    QString error;
    QMetaObject::invokeMethod(
        node,
        [node, manifestPath, &loaded, &error]() {
            loaded = node->loadHistoricalMap(manifestPath, &error);
        },
        Qt::BlockingQueuedConnection);
    if (!loaded) {
        appendDetectionLog(
            tr("[历史地图投递] 接入投递节点失败: %1").arg(error));
        return;
    }
    appendDetectionLog(
        tr("[历史地图投递] 已接入投递节点: %1").arg(manifestPath));
}
#endif

void MainWindow::syncPathPlanButton()
{
    if (pathPlanBtn) {
        QSignalBlocker blocker(pathPlanBtn);
        pathPlanBtn->setChecked(m_bridgeDetectionEnabled);
    }
    if (berthDetectBtn) {
        QSignalBlocker blocker(berthDetectBtn);
        berthDetectBtn->setChecked(m_berthDetectionEnabled);
    }
}

void MainWindow::stopAlgorithmPipeline()
{
    m_bridgeDetectionEnabled = false;
    m_berthDetectionEnabled = false;
    unsubscribeBerthResultBus();
    CommBusManager::instance().stopPipeline();
    if (ui->openGLWidgetLeft)
        ui->openGLWidgetLeft->clearBridgeOverlay();
    if (ui->openGLWidgetRight)
        ui->openGLWidgetRight->clearBridgeOverlay();
    if (ui->openGLWidget)
        ui->openGLWidget->clearBerthResult();
    syncPathPlanButton();
}

void MainWindow::subscribeBerthResultBus()
{
    if (m_berthBusConnected)
        return;
    auto &comm = usv::CommunicationManager::instance();
    m_berthResultTopic = comm.getStateTopic<usv::BerthMeasureResult>("/perception/berth_result", 16);
    m_berthBusSubId = m_berthResultTopic->subscribe(
        [this](std::shared_ptr<const usv::BerthMeasureResult> msg) {
            if (!msg || !ui || !ui->openGLWidget)
                return;
            const usv::BerthMeasureResult copy = *msg;
            QMetaObject::invokeMethod(
                ui->openGLWidget,
                [w = ui->openGLWidget, copy]() { w->updateBerthResult(copy); },
                Qt::QueuedConnection);
        });
    m_berthBusConnected = true;
}

void MainWindow::unsubscribeBerthResultBus()
{
    if (!m_berthBusConnected)
        return;
    if (m_berthResultTopic && m_berthBusSubId != 0)
        m_berthResultTopic->unsubscribe(m_berthBusSubId);
    m_berthBusSubId = 0;
    m_berthResultTopic.reset();
    m_berthBusConnected = false;
}

void MainWindow::subscribeSlamResultBus()
{
#ifdef ENABLE_SLAM
    if (m_slamBusConnected || !ui || !ui->openGLWidget)
        return;

    auto &comm = usv::CommunicationManager::instance();
    MultiLidarWidget *gl = ui->openGLWidget;

    m_slamOdomTopic = comm.getStateTopic<usv::SlamOdometryState>("slam/odometry", 16);
    m_slamOdomBusSubId = m_slamOdomTopic->subscribe(
        [gl](std::shared_ptr<const usv::SlamOdometryState> msg) {
            if (!msg || !gl)
                return;
            const usv::SlamOdometryState copy = *msg;
            QMetaObject::invokeMethod(
                gl,
                [gl, copy]() { gl->updateSlamOdometry(copy); },
                Qt::QueuedConnection);
        });

#ifndef ENABLE_OCTOMAP
    m_slamKeyframeTopic = comm.getStateTopic<usv::SlamKeyframe>("slam/keyframe", 32);
    m_slamKeyframeBusSubId = m_slamKeyframeTopic->subscribe(
        [gl](std::shared_ptr<const usv::SlamKeyframe> msg) {
            if (!msg || !gl)
                return;
            const usv::SlamKeyframe copy = *msg;
            QMetaObject::invokeMethod(
                gl,
                [gl, copy]() { gl->appendSlamKeyframe(copy); },
                Qt::QueuedConnection);
        });

    m_slamScanTopic = comm.getStateTopic<usv::SlamScanCloudMessage>("slam/scan_cloud", 8);
    m_slamScanBusSubId = m_slamScanTopic->subscribe(
        [gl](std::shared_ptr<const usv::SlamScanCloudMessage> msg) {
            if (!msg || !gl || msg->points.empty())
                return;
            QVector<M_PointXYZI> cloud;
            cloud.reserve(static_cast<int>(msg->points.size()));
            for (const M_PointXYZI &p : msg->points)
                cloud.append(p);
            QMetaObject::invokeMethod(
                gl,
                [gl, cloud = std::move(cloud)]() mutable {
                    gl->updateSlamLiveScan(cloud);
                },
                Qt::QueuedConnection);
        });
#endif

    m_slamBusConnected = true;
#endif
}

void MainWindow::unsubscribeSlamResultBus()
{
#ifdef ENABLE_SLAM
    if (!m_slamBusConnected)
        return;
    if (m_slamOdomTopic && m_slamOdomBusSubId != 0)
        m_slamOdomTopic->unsubscribe(m_slamOdomBusSubId);
    if (m_slamKeyframeTopic && m_slamKeyframeBusSubId != 0)
        m_slamKeyframeTopic->unsubscribe(m_slamKeyframeBusSubId);
    if (m_slamScanTopic && m_slamScanBusSubId != 0)
        m_slamScanTopic->unsubscribe(m_slamScanBusSubId);
    m_slamOdomBusSubId = 0;
    m_slamKeyframeBusSubId = 0;
    m_slamScanBusSubId = 0;
    m_slamOdomTopic.reset();
    m_slamKeyframeTopic.reset();
    m_slamScanTopic.reset();
    m_slamBusConnected = false;
#endif
}

void MainWindow::startAlgorithmPipeline()
{
    auto &bus = CommBusManager::instance();
    bus.ensurePublisher();
    bus.ensureBridgeNode();
    bus.ensureBerthNode();
    if (!m_bridgeLogConnected && bus.bridgeNode()) {
        connect(bus.bridgeNode(), &BridgeDetectionNode::logMessage,
                this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
        m_bridgeLogConnected = true;
    }
    if (!m_bridgeOverlayConnected && bus.bridgeNode()) {
        // 叠图仅为显示；结构化结果走 /perception/traditional_bridge_vision
        connect(bus.bridgeNode(), &BridgeDetectionNode::bridgeOverlayReady,
                ui->openGLWidgetLeft,
                QOverload<const QImage &, double>::of(&VideoWidget::updateBridgeOverlay),
                Qt::QueuedConnection);
        connect(bus.bridgeNode(), &BridgeDetectionNode::bridgeOverlayReadyRight,
                ui->openGLWidgetRight,
                QOverload<const QImage &, double>::of(&VideoWidget::updateBridgeOverlay),
                Qt::QueuedConnection);
        m_bridgeOverlayConnected = true;
    }
    if (!m_berthLogConnected && bus.berthNode()) {
        connect(bus.berthNode(), &BerthDetectionNode::logMessage,
                this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
        m_berthLogConnected = true;
    }
    if (auto *node = bus.bridgeNode())
        node->setProcessingEnabled(m_bridgeDetectionEnabled);
    if (auto *node = bus.berthNode()) {
        node->setDetectionModes(m_berthDetectionEnabled, m_berthDetectionEnabled);
        if (m_berthDetectionEnabled)
            node->resetWorldCoordinateState();
    }
    if (m_berthDetectionEnabled)
        subscribeBerthResultBus();
    else
        unsubscribeBerthResultBus();

    if (!bus.pipelineRunning())
        bus.startPipeline();

    syncPathPlanButton();
}

void MainWindow::setupCommBus()
{
    QTimer::singleShot(0, this, [this]() {
        CommBusManager::instance().ensureInitialized();
        CommBusManager::instance().ensurePublisher();
        appendDetectionLog(tr("Topic 总线已就绪：检测结果 /perception/* 与 slam/*。"));
    });
}

void MainWindow::videoProcess()
{
    // 离线/在线共用：pcap → UdpDispatcher
    connect(reader, &PcapngReader::udpPacket, udpDispatcher, &UdpDispatcher::dispatch,
            Qt::DirectConnection);
    liveCaptureInit();

    connect(rtpParserH264Right, &RtpParserH264::rtpPayloadReady,
            rtpParserH264Left, &RtpParserH264::onExtraConfigReceived);

    // --- H264 红外（仍走抓包 + RtpParser）---
    h264ThreadLeft = new QThread(this);
    h264WorkerLeft = new VideoWorker(nullptr, AV_CODEC_ID_H264);
    h264WorkerLeft->setVideoWorkerName("h264WorkerLeft");
    h264videoLeft = new VideoWidget();
    h264WorkerLeft->moveToThread(h264ThreadLeft);
    connect(h264ThreadLeft, &QThread::started, h264WorkerLeft, &VideoWorker::initDecoder);
    connect(rtpParserH264Left, &RtpParserH264::rtpPayloadReady, h264WorkerLeft, &VideoWorker::handleNal, Qt::QueuedConnection);
    connect(h264WorkerLeft, &VideoWorker::frameReady, h264videoLeft,
            QOverload<const QImage &>::of(&VideoWidget::updateFrame), Qt::QueuedConnection);
    h264ThreadLeft->start();

    h264ThreadRight = new QThread(this);
    h264WorkerRight = new VideoWorker(nullptr, AV_CODEC_ID_H264);
    h264WorkerRight->setVideoWorkerName("h264WorkerRight");
    h264videoRight = new VideoWidget();
    h264WorkerRight->moveToThread(h264ThreadRight);
    connect(h264ThreadRight, &QThread::started, h264WorkerRight, &VideoWorker::initDecoder);
    connect(rtpParserH264Right, &RtpParserH264::rtpPayloadReady, h264WorkerRight, &VideoWorker::handleNal, Qt::QueuedConnection);
    connect(h264WorkerRight, &VideoWorker::frameReady, h264videoRight,
            QOverload<const QImage &>::of(&VideoWidget::updateFrame), Qt::QueuedConnection);
    h264ThreadRight->start();

    // --- H265 固连相机：离线 pcap 走 RtpParser→Worker；在线走 RtspPullWorker ---
    h265ThreadLeft = new QThread(this);
    h265WorkerLeft = new VideoWorker(nullptr, AV_CODEC_ID_HEVC);
    h265WorkerLeft->setVideoWorkerName("h265WorkerLeft");
    h265WorkerLeft->moveToThread(h265ThreadLeft);
    connect(h265ThreadLeft, &QThread::started, h265WorkerLeft, &VideoWorker::initDecoder);
    m_connRtp2ToWorker = connect(rtpParser2, &RtpParser::rtpPayloadReady,
                                 h265WorkerLeft, &VideoWorker::handleNal, Qt::QueuedConnection);
    connect(h265WorkerLeft, &VideoWorker::frameReady,
            this, &MainWindow::onBridgeCameraFrame, Qt::QueuedConnection);
    h265ThreadLeft->start();

    h265ThreadRight = new QThread(this);
    h265WorkerRight = new VideoWorker(nullptr, AV_CODEC_ID_HEVC);
    h265WorkerRight->setVideoWorkerName("h265WorkerRight");
    h265WorkerRight->moveToThread(h265ThreadRight);
    connect(h265ThreadRight, &QThread::started, h265WorkerRight, &VideoWorker::initDecoder);
    m_connRtp1ToWorker = connect(rtpParser1, &RtpParser::rtpPayloadReady,
                                 h265WorkerRight, &VideoWorker::handleNal, Qt::QueuedConnection);
    connect(h265WorkerRight, &VideoWorker::frameReady,
            this, &MainWindow::onBridgeCameraFrameRight, Qt::QueuedConnection);
    h265ThreadRight->start();

    initRtspPull();

    qDebug() << "初始化流水线完成。";
}

/** 创建两路 RTSP 拉流线程（在线点击后由 startRtsp 启动） */
void MainWindow::initRtspPull()
{
    if (rtspThreadRight)
        return;

    rtspThreadRight = new QThread(this);
    rtspWorkerRight = new RtspPullWorker();
    const QString videoLocalIp = AppConfig::instance().network().video_local_ip;
    rtspWorkerRight->setLocalBindAddress(videoLocalIp);
    rtspWorkerRight->setStreamName(
        requireDevice(QStringLiteral("h265Right")).metadata.value(QStringLiteral("stream_name"),
            QStringLiteral("USV02-右固连相机")).toString());
    rtspWorkerRight->moveToThread(rtspThreadRight);
    connect(rtspThreadRight, &QThread::finished, rtspWorkerRight, &QObject::deleteLater);
    // 右路 → 右侧 VideoWidget + 传统视觉桥梁检测
    connect(rtspWorkerRight, &RtspPullWorker::frameReady,
            this, &MainWindow::onBridgeCameraFrameRight, Qt::QueuedConnection);
    connect(rtspWorkerRight, &RtspPullWorker::logMessage,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
    connect(rtspWorkerRight, &RtspPullWorker::pullFailed,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
    rtspThreadRight->start();

    rtspThreadLeft = new QThread(this);
    rtspWorkerLeft = new RtspPullWorker();
    rtspWorkerLeft->setLocalBindAddress(videoLocalIp);
    rtspWorkerLeft->setStreamName(
        requireDevice(QStringLiteral("h265Left")).metadata.value(QStringLiteral("stream_name"),
            QStringLiteral("USV02-左固连相机")).toString());
    rtspWorkerLeft->moveToThread(rtspThreadLeft);
    connect(rtspThreadLeft, &QThread::finished, rtspWorkerLeft, &QObject::deleteLater);
    // 左路 → 左侧窗口 + 桥洞检测
    connect(rtspWorkerLeft, &RtspPullWorker::frameReady,
            this, &MainWindow::onBridgeCameraFrame, Qt::QueuedConnection);
    connect(rtspWorkerLeft, &RtspPullWorker::logMessage,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
    connect(rtspWorkerLeft, &RtspPullWorker::pullFailed,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
    rtspThreadLeft->start();
}

void MainWindow::startRtsp()
{
    initRtspPull();
    setH265PcapVideoEnabled(false);

    const DeviceInfo devH265R = requireDevice(QStringLiteral("h265Right"));
    const DeviceInfo devH265L = requireDevice(QStringLiteral("h265Left"));
    const QString rtspRight = devH265R.metadata.value(QStringLiteral("rtsp_url")).toString();
    const QString rtspLeft = devH265L.metadata.value(QStringLiteral("rtsp_url")).toString();

    m_rtspRunning = true;

    QMetaObject::invokeMethod(rtspWorkerRight, "startPull", Qt::QueuedConnection,
                              Q_ARG(QString, rtspRight));
    QMetaObject::invokeMethod(rtspWorkerLeft, "startPull", Qt::QueuedConnection,
                              Q_ARG(QString, rtspLeft));
}

void MainWindow::stopRtsp()
{
    if (!rtspWorkerRight && !rtspWorkerLeft)
        return;

    if (rtspWorkerRight)
        QMetaObject::invokeMethod(rtspWorkerRight, "stopPull", Qt::QueuedConnection);
    if (rtspWorkerLeft)
        QMetaObject::invokeMethod(rtspWorkerLeft, "stopPull", Qt::QueuedConnection);

    setH265PcapVideoEnabled(true); // 恢复离线 pcap 解码路径
    m_rtspRunning = false;
}

/** 在线 RTSP 时断开 RtpParser→Worker，停止后恢复（供离线 pcap 使用） */
void MainWindow::setH265PcapVideoEnabled(bool enabled)
{
    if (enabled) {
        if (!m_connRtp2ToWorker && rtpParser2 && h265WorkerLeft) {
            m_connRtp2ToWorker = connect(rtpParser2, &RtpParser::rtpPayloadReady,
                                         h265WorkerLeft, &VideoWorker::handleNal,
                                         Qt::QueuedConnection);
        }
        if (!m_connRtp1ToWorker && rtpParser1 && h265WorkerRight) {
            m_connRtp1ToWorker = connect(rtpParser1, &RtpParser::rtpPayloadReady,
                                         h265WorkerRight, &VideoWorker::handleNal,
                                         Qt::QueuedConnection);
        }
    } else {
        if (m_connRtp2ToWorker) {
            disconnect(m_connRtp2ToWorker);
            m_connRtp2ToWorker = {};
        }
        if (m_connRtp1ToWorker) {
            disconnect(m_connRtp1ToWorker);
            m_connRtp1ToWorker = {};
        }
    }
}


void MainWindow::resetVideoRtpParsers()
{
    if (rtpParser1)
        rtpParser1->resetStreamState();
    if (rtpParser2)
        rtpParser2->resetStreamState();
    if (rtpParserH264Left)
        rtpParserH264Left->resetStreamState();
    if (rtpParserH264Right)
        rtpParserH264Right->resetStreamState();
}

void MainWindow::onStopBtnClicked()
{
    stopOfflineCapture();
    stopLiveCapture();
    resetVideoRtpParsers();

    stopBtn->setEnabled(false);
    ui->statusbar->showMessage(tr("正在停止数据接收..."), 2000);
    qDebug() << "MainWindow: 已请求停止离线回放与在线抓包";
}

void MainWindow::onOnlineBtnClicked()
{
    if (m_liveRunning) {
        ui->statusbar->showMessage(tr("在线接收已在运行"), 2000);
        return;
    }

    stopOfflineCapture();
    resetVideoRtpParsers();

    const NetworkReceiveConfig &netCfg = AppConfig::instance().network();
    CommBusManager::instance().ensurePublisher();

    onlineBtn->setEnabled(false);
    loadBtn->setEnabled(false);
    stopBtn->setEnabled(true);

    if (netCfg.useSocketMode()) {
        udpSocketInit();
        QMetaObject::invokeMethod(udpReceiver, &UdpSocketReceiver::startReceiving, Qt::QueuedConnection);
        startRtsp();
        return;
    }

    QString pickError;
    const QString deviceName = PcapDeviceDialog::pickDevice(this, &pickError);
    if (deviceName.isEmpty()) {
        onlineBtn->setEnabled(true);
        loadBtn->setEnabled(true);
        stopBtn->setEnabled(false);
        return;
    }

    liveCaptureInit();
    QMetaObject::invokeMethod(liveCapture, [this, deviceName]() {
        liveCapture->startCapture(deviceName);
    }, Qt::QueuedConnection);

    startRtsp();
}

void MainWindow::onLoadBtnClicked()
{
    // 1. 获取文件
    QStringList fileNames = QFileDialog::getOpenFileNames(this, "选择PCAP", ".", "*.pcap");
    if(fileNames.isEmpty()) return;

    CommBusManager::instance().ensureBerthNode();
    if (auto *node = CommBusManager::instance().berthNode())
        node->resetWorldCoordinateState();

    // 离线回放与在线抓包互斥
    stopLiveCapture();
    resetVideoRtpParsers();

    // 2. 检查资源（如果还没初始化，就初始化一次）
    if (!reader) {
        pcapngRead(); // 这里面必须包含 reader 和 udpDispatcher 的 connect!
    }

    // 3. 如果上个任务还在跑，先喊停
    if (readerThread && readerThread->isRunning()) {
        reader->stopReading();
    }

    // 4. 重置 UI
    loadBtn->setEnabled(false);
    onlineBtn->setEnabled(false);
    stopBtn->setEnabled(true);
    m_offlineRunning = true;

    // 5. 【最关键的改动】直接投递任务
    // 注意：不要在 startAnalysis 里做复杂的 open 逻辑，直接把文件名传进去
    QMetaObject::invokeMethod(reader, [this, fileNames](){
        // 记得在 startAnalysis 第一行重置 m_stopRequested = false
        reader->startAnalysis(fileNames);
    }, Qt::QueuedConnection);

    qDebug() << "解析任务已投递";

}

void MainWindow::onPathplanClicked()
{
    m_bridgeDetectionEnabled = pathPlanBtn->isChecked();
    if (m_bridgeDetectionEnabled) {
        startAlgorithmPipeline();
        appendDetectionLog(tr("已开启桥梁检测。"));
        ui->statusbar->showMessage(tr("桥梁检测已开启"), 3000);
    } else {
        if (auto *node = CommBusManager::instance().bridgeNode())
            node->setProcessingEnabled(false);
        if (ui->openGLWidgetLeft)
            ui->openGLWidgetLeft->clearBridgeOverlay();
        if (ui->openGLWidgetRight)
            ui->openGLWidgetRight->clearBridgeOverlay();
        if (!m_berthDetectionEnabled)
            stopAlgorithmPipeline();
        appendDetectionLog(tr("已关闭桥梁检测。"));
        ui->statusbar->showMessage(tr("桥梁检测已关闭"), 3000);
    }
}

void MainWindow::onBerthDetectionClicked()
{
    m_berthDetectionEnabled = berthDetectBtn->isChecked();
    if (m_berthDetectionEnabled) {
        startAlgorithmPipeline();
        appendDetectionLog(tr("已开启泊位检测。"));
        ui->statusbar->showMessage(tr("泊位检测已开启"), 3000);
    } else {
        if (auto *node = CommBusManager::instance().berthNode())
            node->setDetectionModes(false, false);
        unsubscribeBerthResultBus();
        if (ui->openGLWidget)
            ui->openGLWidget->clearBerthResult();
        if (!m_bridgeDetectionEnabled)
            stopAlgorithmPipeline();
        appendDetectionLog(tr("已关闭泊位检测。"));
        ui->statusbar->showMessage(tr("泊位检测已关闭"), 3000);
    }
}

void MainWindow::onTopViewClicked()
{
    // isChecked() 会根据按钮状态自动返回 true 或 false

    bool checked = topViewBtn->isChecked();

    ui->openGLWidget->setBevMode(checked);

    if (checked) {
        topViewBtn->setText("恢复3D视图"); // 如果用了方案2的Label，这里改 label->setText
    } else {
        topViewBtn->setText("切换俯视图");
    }
}

void MainWindow::onSlamMapClicked()
{
    const bool checked = slamMapBtn->isChecked();
    ui->openGLWidget->setSlamMapMode(checked);
    if (checked) {
        slamMapBtn->setText(tr("实时点云"));
        ui->statusbar->showMessage(tr("SLAM 地图模式：俯视图下左键拖拽平移，滚轮缩放"), 4000);
    } else {
        slamMapBtn->setText(tr("SLAM地图"));
        ui->statusbar->showMessage(tr("实时点云模式"), 3000);
    }
}

void MainWindow::onSaveSlamMapClicked()
{
    if (!ui->openGLWidget) {
        appendDetectionLog(tr("[地图保存] OpenGL 视图未就绪"));
        return;
    }
    if (!ui->openGLWidget->hasSlamMapData()) {
        appendDetectionLog(tr("[地图保存] 当前 SLAM 地图为空，请先开启 SLAM 建图并累积关键帧"));
        ui->statusbar->showMessage(tr("SLAM 地图为空，无法保存"), 4000);
        return;
    }

    const QString dir = slam_map_io::defaultMapsDirectory();
    const QString fileName = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))
                           + QStringLiteral(".slammap");
    const QString filePath = dir + QLatin1Char('/') + fileName;

#ifdef ENABLE_SLAM
    if (slam_node_)
        ui->openGLWidget->setSlamGeoAnchor(slam_node_->getGeoAnchor());
#endif

    QString error;
    if (!ui->openGLWidget->saveCurrentSlamMap(filePath, &error)) {
        appendDetectionLog(tr("[地图保存] 失败: %1").arg(error));
        ui->statusbar->showMessage(tr("地图保存失败"), 4000);
        return;
    }

    appendDetectionLog(tr("[地图保存] 已保存 %1").arg(filePath));
    ui->statusbar->showMessage(tr("地图已保存: %1").arg(filePath), 8000);

#ifdef ENABLE_OCTOMAP
    if (octomap_node_ && octomap_node_->keyframeCount() > 0) {
        QString btPath = filePath;
        if (btPath.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive))
            btPath.chop(4);
        btPath += QStringLiteral(".bt");
        bool ok = false;
        QString btError;
        QMetaObject::invokeMethod(
            octomap_node_,
            [this, btPath, &ok, &btError]() {
                ok = octomap_node_->saveBinary(btPath, &btError);
            },
            Qt::BlockingQueuedConnection);
        if (ok)
            appendDetectionLog(tr("[OctoMap] 已保存 %1 (leaves=%2)")
                                   .arg(btPath)
                                   .arg(octomap_node_->leafCount()));
        else
            appendDetectionLog(tr("[OctoMap] 保存失败: %1").arg(btError));
    }
#endif
}

void MainWindow::onLoadSlamMapClicked()
{
    if (!ui->openGLWidget) {
        appendDetectionLog(tr("[地图加载] OpenGL 视图未就绪"));
        return;
    }

    const QString dir = slam_map_io::defaultMapsDirectory();
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("加载 SLAM 地图"),
        dir,
        tr("SLAM 地图 (*.slammap *.pcd);;所有文件 (*.*)"));
    if (filePath.isEmpty())
        return;

    QString error;
    if (!ui->openGLWidget->loadSlamMap(filePath, &error)) {
        appendDetectionLog(tr("[地图加载] 失败: %1").arg(error));
        ui->statusbar->showMessage(tr("地图加载失败"), 4000);
        return;
    }

    if (slamMapBtn && !slamMapBtn->isChecked()) {
        QSignalBlocker blocker(slamMapBtn);
        slamMapBtn->setChecked(true);
        slamMapBtn->setText(tr("实时点云"));
    }
    ui->openGLWidget->setSlamMapMode(true);

#ifdef ENABLE_SLAM
    prepareHistoricalMapForExport(filePath);
#endif

    appendDetectionLog(tr("[地图加载] 已加载 %1").arg(filePath));
    ui->statusbar->showMessage(tr("地图已加载，已切换到 SLAM 地图视图"), 5000);
}

void MainWindow::onSlamBuildClicked()
{
#ifdef ENABLE_SLAM
    if (slamBuildBtn->isChecked()) {
        initSlamNode();
        if (!slam_node_) {
            QSignalBlocker blocker(slamBuildBtn);
            slamBuildBtn->setChecked(false);
            return;
        }
        slamBuildBtn->setText(tr("关闭建图"));
        appendDetectionLog(tr("已开启 SLAM 建图。"));
        ui->statusbar->showMessage(tr("SLAM 建图运行中"), 3000);
        if (m_exportRunning)
            startPerceptionExport();
    } else {
        stopSlamNode();
        slamBuildBtn->setText(tr("SLAM建图"));
        ui->statusbar->showMessage(tr("SLAM 建图已关闭"), 3000);
    }
#else
    QSignalBlocker blocker(slamBuildBtn);
    slamBuildBtn->setChecked(false);
    appendDetectionLog(tr("SLAM 未编译（需要 PCL，请确认 CMake 中 ENABLE_SLAM 与 PCL 路径）"));
#endif
}

void MainWindow::ensureExportThread()
{
    if (export_node_)
        return;

    export_thread_ = new QThread(this);
    export_node_ = new PerceptionExportNode();
    connect(export_node_, &PerceptionExportNode::logMessage,
            this, &MainWindow::appendDetectionLog, Qt::QueuedConnection);
    export_node_->moveToThread(export_thread_);
    connect(export_thread_, &QThread::finished, export_node_, &QObject::deleteLater);
    connect(export_node_, &QObject::destroyed, this, [this]() { export_node_ = nullptr; });
    export_thread_->start();
}

void MainWindow::startPerceptionExport()
{
    ensureExportThread();
    if (!export_node_)
        return;

    CommBusManager::instance().ensurePublisher();

    // The export node normally consumes /perception/berth_result itself.  Keep
    // this direct signal bridge as a lifecycle-safe fallback: it covers builds
    // where a topic was recreated during playback or the result was emitted
    // before the export subscription was installed.  PerceptionExportNode
    // de-duplicates the topic and signal copies by frame signature.
    if (!m_exportBerthConnected) {
        if (auto *berthNode = CommBusManager::instance().berthNode()) {
            m_exportBerthConnection = connect(
                berthNode, &BerthDetectionNode::berthResultReady,
                this,
                [this](const usv::BerthMeasureResult &result) {
                    if (!export_node_)
                        return;
                    PerceptionExportNode *node = export_node_;
                    const usv::BerthMeasureResult copy = result;
                    QMetaObject::invokeMethod(
                        node,
                        [node, copy]() { node->onBerthResult(copy); },
                        Qt::QueuedConnection);
                },
                Qt::QueuedConnection);
            m_exportBerthConnected = true;
        }
    }

    // 泊位 / SLAM 结果由 Export 自行订阅 Topic，不再经 MainWindow 转接信号
    bool started = false;
    QMetaObject::invokeMethod(
        export_node_,
        [this, &started]() {
            export_node_->start();
            started = export_node_->isRunning();
        },
        Qt::BlockingQueuedConnection);
    m_exportRunning = started;
    if (!m_exportRunning) {
        appendDetectionLog(tr("信息投递启动失败，请检查本机绑定 IP/端口是否可用"));
        return;
    }

#ifdef ENABLE_SLAM
    // 地图可能在投递节点之前加载；启动完成后补接待发送的历史分块地图。
    loadPendingHistoricalMapForExport();
#endif

#ifdef ENABLE_SLAM
    if (slam_node_) {
        const std::vector<usv::SlamKeyframe> keyframes = slam_node_->getKeyframes();
        if (!keyframes.empty()) {
            const std::vector<usv::SlamKeyframe> copy = keyframes;
            QMetaObject::invokeMethod(
                export_node_,
                [this, copy]() { export_node_->ingestSlamKeyframes(copy); },
                Qt::QueuedConnection);
        }
        if (auto odom = slam_node_->getLatestOdometry()) {
            if (odom->pose_lidar.valid) {
                const usv::SlamOdometryState odom_copy = *odom;
                QMetaObject::invokeMethod(
                    export_node_,
                    [this, odom_copy]() { export_node_->onSlamOdometry(odom_copy); },
                    Qt::QueuedConnection);
            }
        }
    }
#endif
    appendDetectionLog(tr("信息投递已启动（订阅 /perception/berth_result 与 slam/*）"));
}

void MainWindow::stopPerceptionExport()
{
    if (m_exportBerthConnected) {
        QObject::disconnect(m_exportBerthConnection);
        m_exportBerthConnection = {};
        m_exportBerthConnected = false;
    }
    if (export_node_) {
        QMetaObject::invokeMethod(export_node_, &PerceptionExportNode::stop,
                                  Qt::BlockingQueuedConnection);
    }

    m_exportRunning = false;

    if (exportBtn) {
        QSignalBlocker blocker(exportBtn);
        exportBtn->setChecked(false);
        exportBtn->setText(tr("信息投递"));
    }
}

void MainWindow::onExportClicked()
{
    if (exportBtn->isChecked()) {
        QDialog dialog(this);
        dialog.setWindowTitle(tr("信息投递配置"));
        auto *layout = new QFormLayout(&dialog);
        auto *ipEdit = new QLineEdit(QStringLiteral("101.1.101.105"), &dialog);
        auto *portEdit = new QSpinBox(&dialog);
        auto *localIpEdit = new QLineEdit(QStringLiteral("101.1.101.177"), &dialog);
        auto *localPortEdit = new QSpinBox(&dialog);
        portEdit->setRange(1, 65535);
        localPortEdit->setRange(1, 65535);
        portEdit->setValue(6789);
        localPortEdit->setValue(6789);
        layout->addRow(tr("对端 IP（艇载 101.1.101.105）"), ipEdit);
        layout->addRow(tr("对端端口"), portEdit);
        layout->addRow(tr("本机 IP（PointCloud 101.1.101.177）"), localIpEdit);
        layout->addRow(tr("本机端口"), localPortEdit);
        auto *buttons = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
        layout->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        if (dialog.exec() != QDialog::Accepted) {
            QSignalBlocker blocker(exportBtn);
            exportBtn->setChecked(false);
            return;
        }

        const QHostAddress host(ipEdit->text().trimmed());
        if (host.isNull() || host.protocol() != QAbstractSocket::IPv4Protocol) {
            appendDetectionLog(tr("信息投递失败：IP 地址无效"));
            QSignalBlocker blocker(exportBtn);
            exportBtn->setChecked(false);
            return;
        }

        ensureExportThread();

        PerceptionExportNode::Config cfg;
        cfg.remote_host = host;
        cfg.remote_port = static_cast<quint16>(portEdit->value());
        const QHostAddress localHost(localIpEdit->text().trimmed());
        cfg.local_host = localHost;
        cfg.local_port = static_cast<quint16>(localPortEdit->value());
        cfg.bind_local = !localHost.isNull()
                      && localHost.protocol() == QAbstractSocket::IPv4Protocol;
        cfg.send_map = true;
        QMetaObject::invokeMethod(
            export_node_,
            [this, cfg]() { export_node_->setConfig(cfg); },
            Qt::BlockingQueuedConnection);
        startPerceptionExport();

        exportBtn->setText(tr("停止投递"));
        appendDetectionLog(tr("信息投递已开启 → %1:%2")
                               .arg(host.toString())
                               .arg(portEdit->value()));
        if (!m_berthDetectionEnabled)
            appendDetectionLog(tr("提示：泊位数据需同时开启「泊位检测」。"));
#ifdef ENABLE_SLAM
        if (!m_slamRunning && pending_historical_manifest_path_.isEmpty())
            appendDetectionLog(
                tr("提示：尚未接入历史地图；请先点击「加载地图」，无需开启 SLAM 建图。"));
#endif
        ui->statusbar->showMessage(tr("信息投递运行中"), 3000);
    } else {
        stopPerceptionExport();
        appendDetectionLog(tr("信息投递已停止"));
        ui->statusbar->showMessage(tr("信息投递已停止"), 3000);
    }
}

void MainWindow::ondeviceQueryClicked()
{
    DialogDeviceManager dialog(this);
    dialog.exec();
}

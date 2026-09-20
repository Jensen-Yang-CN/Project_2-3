#include "bridge_detection_worker.h"
#include "TraditionalBridgeVisionDetector.h"
#include <QElapsedTimer>
#include "CalibrationConfig.h"
#include "CommunicationManager_v2.h"
#include "common_types_extended.h"
#include <algorithm>
#include <cmath>

namespace {
QString formatPierAzimuths(const std::vector<cv::Point2f> &centers, bool leftCamera)
{
    const BridgeCameraCalibration camera = leftCamera
        ? CalibrationConfig::instance().bridgeCamera()
        : CalibrationConfig::instance().rightCamera();
    const double fx = camera.intrinsic_k[0];
    const double cx = camera.intrinsic_k[2];
    if (fx <= 0.0 || centers.empty()) return QStringLiteral("无");
    QStringList values;
    for (int i = 0; i < static_cast<int>(centers.size()); ++i) {
        const double deg = std::atan((centers[i].x - cx) / fx) * 180.0 / M_PI;
        values << QStringLiteral("Pier-%1=%2%3deg").arg(i + 1).arg(deg >= 0.0 ? "+" : "").arg(deg, 0, 'f', 1);
    }
    return values.join(QStringLiteral("  "));
}

usv::TraditionalBridgeVisionBusResult makeVisionBusResult(
    const usv::TraditionalBridgeVisionResult &vision,
    bool leftCamera,
    double timestamp,
    quint64 timelineGeneration)
{
    usv::TraditionalBridgeVisionBusResult out;
    out.timestamp = timestamp;
    out.timeline_generation = timelineGeneration;
    out.camera_side = leftCamera ? usv::CameraSide::Left : usv::CameraSide::Right;
    out.bridge_detected = vision.detected;
    out.confidence = vision.confidence;
    out.has_deck_line = vision.hasDeckLine;

    const BridgeCameraCalibration camera = leftCamera
        ? CalibrationConfig::instance().bridgeCamera()
        : CalibrationConfig::instance().rightCamera();
    const double fx = camera.intrinsic_k[0];
    const double cx = camera.intrinsic_k[2];
    auto azimuthDeg = [fx, cx](double pixelX) {
        return fx > 0.0 ? std::atan((pixelX - cx) / fx) * 180.0 / M_PI : 0.0;
    };

    std::vector<cv::Point2f> centers = vision.pierCenters;
    std::sort(centers.begin(), centers.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.x < b.x;
    });
    out.pier_count = static_cast<int>(centers.size());
    out.piers.reserve(centers.size());
    for (int i = 0; i < static_cast<int>(centers.size()); ++i) {
        usv::BridgePierObservation pier;
        pier.id = i + 1;
        pier.pixel_x = centers[i].x;
        pier.pixel_y = centers[i].y;
        pier.azimuth_deg = azimuthDeg(centers[i].x);
        out.piers.push_back(pier);
    }

    if (centers.size() >= 2 && fx > 0.0) {
        const double leftAzimuth = azimuthDeg(centers.front().x);
        const double rightAzimuth = azimuthDeg(centers.back().x);
        const double openingCenterX = (centers.front().x + centers.back().x) * 0.5;
        out.has_bridge_opening = true;
        out.bridge_opening_center_azimuth_deg = azimuthDeg(openingCenterX);
        out.pier_angular_separation_deg = std::abs(rightAzimuth - leftAzimuth);
    }

    if (vision.hasDeckLine) {
        out.deck_inclination_deg = std::atan2(
            static_cast<double>(vision.deckEnd.y - vision.deckStart.y),
            static_cast<double>(vision.deckEnd.x - vision.deckStart.x)) * 180.0 / M_PI;
    }
    return out;
}
}

#ifdef ENABLE_BRIDGE_DETECT

#include "SystemLogHelpers.h"

namespace boost {
void throw_exception(std::exception const &e)
{
    (void)e;
    std::terminate();
}
} // namespace boost

#include "bridge_common_types.h"
#include "BridgeDetector.h"
#include "TraditionalBridgeVisionDetector.h"
#include <opencv2/core.hpp>
#include <exception>
#include <QDebug>
#include <QElapsedTimer>

namespace {

usv::LidarFrame toLidarFrame(const QVector<M_PointXYZI> &cloud, double ts, int maxPoints = 80000)
{
    usv::LidarFrame frame;
    frame.timestamp = ts;
    const int total = cloud.size();
    if (total <= 0) {
        frame.point_count = 0;
        return frame;
    }
    const int step = total > maxPoints ? (total + maxPoints - 1) / maxPoints : 1;
    frame.points.reserve(static_cast<size_t>(total / step) + 1);
    for (int i = 0; i < total; i += step) {
        const M_PointXYZI &p = cloud[i];
        usv::LidarPoint lp;
        lp.x = p.x;
        lp.y = p.y;
        lp.z = p.z;
        lp.intensity = static_cast<float>(p.intensity);
        frame.points.push_back(lp);
    }
    frame.point_count = static_cast<uint32_t>(frame.points.size());
    return frame;
}

usv::ImageFrame toImageFrame(const QImage &img, double ts)
{
    usv::ImageFrame frame;
    frame.timestamp = ts;
    if (img.isNull())
        return frame;

    QImage rgb = img.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                   const_cast<uchar *>(rgb.bits()), static_cast<size_t>(rgb.bytesPerLine()));
    frame.image.create(rgbMat.rows, rgbMat.cols, CV_8UC3);
    for (int y = 0; y < rgbMat.rows; ++y) {
        const cv::Vec3b *src = rgbMat.ptr<cv::Vec3b>(y);
        cv::Vec3b *dst = frame.image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < rgbMat.cols; ++x) {
            dst[x] = cv::Vec3b(src[x][2], src[x][1], src[x][0]);
        }
    }
    return frame;
}

QImage bgraMatToQImage(const cv::Mat &bgra)
{
    if (bgra.empty() || bgra.type() != CV_8UC4)
        return QImage();
    return QImage(bgra.data, bgra.cols, bgra.rows, static_cast<int>(bgra.step),
                  QImage::Format_ARGB32)
        .copy();
}

} // namespace

struct BridgeDetectionWorker::Impl {
    usv::BridgeDetector detector;
    usv::TraditionalBridgeVisionDetector visionDetector;
    QElapsedTimer visionStatsTimer;
    qint64 visionTotalMs = 0;
    qint64 visionMaxMs = 0;
    int visionFrameCount = 0;
    int visionDetectedCount = 0;
    QString pierAzimuths = QStringLiteral("无");
    usv::TraditionalBridgeVisionBusResult latestVisionBusResult;
    std::shared_ptr<usv::EventTopic<usv::TraditionalBridgeVisionBusResult>> visionResultTopic;
    bool initialized = false;
};

BridgeDetectionWorker::BridgeDetectionWorker(bool leftCamera, QObject *parent)
    : QObject(parent)
    , m_impl(new Impl)
    , m_leftCamera(leftCamera)
{
}

BridgeDetectionWorker::~BridgeDetectionWorker()
{
    delete m_impl;
}

void BridgeDetectionWorker::initialize()
{
    m_impl->visionResultTopic = usv::CommunicationManager::instance()
        .getEventTopic<usv::TraditionalBridgeVisionBusResult>(
            "/perception/traditional_bridge_vision", 32);
    m_impl->detector.init();
    m_impl->visionStatsTimer.start();
    m_impl->initialized = true;
    emit logMessage(QStringLiteral(
        "桥梁检测已初始化（传统双相机视觉 + 210/211 点云桥洞）。"));
}

void BridgeDetectionWorker::resetForPlaybackSeek()
{
    if (!m_impl)
        return;
    m_impl->detector.resetPlaybackTimeline();
    m_impl->visionDetector.reset();
    m_impl->visionStatsTimer.restart();
    m_impl->visionTotalMs = 0;
    m_impl->visionMaxMs = 0;
    m_impl->visionFrameCount = 0;
    m_impl->visionDetectedCount = 0;
    m_impl->pierAzimuths = QStringLiteral("无");
    m_impl->latestVisionBusResult = {};
}

void BridgeDetectionWorker::runDetection(double timestamp,
                                         QVector<M_PointXYZI> cloud210,
                                         QVector<M_PointXYZI> cloud211,
                                         QImage cameraImage,
                                         quint64 timelineGeneration)
{
    if (!m_impl->initialized) {
        emit taskLogMessage(QStringLiteral("桥洞/泊位检测未启动，请先点击「检测开关」开启。"),
                            timelineGeneration);
        return;
    }

    const usv::LidarFrame f210 = toLidarFrame(cloud210, timestamp);
    const usv::LidarFrame f211 = toLidarFrame(cloud211, timestamp);
    const usv::ImageFrame img = toImageFrame(cameraImage, timestamp);

    if (f210.points.empty() && f211.points.empty() && img.image.empty()) {
        emit taskLogMessage(QStringLiteral("[桥洞检测] 210/211 点云均为空，跳过本帧。"),
                            timelineGeneration);
        return;
    }

    QElapsedTimer visionFrameTimer;
    visionFrameTimer.start();
    const usv::TraditionalBridgeVisionResult vision = m_impl->visionDetector.detect(img.image);
    m_impl->pierAzimuths = formatPierAzimuths(vision.pierCenters, m_leftCamera);
    m_impl->latestVisionBusResult = makeVisionBusResult(
        vision, m_leftCamera, timestamp, timelineGeneration);
    const QLineF deckLine(vision.deckStart.x, vision.deckStart.y, vision.deckEnd.x, vision.deckEnd.y);
    emit traditionalDeckLineReady(
        deckLine, vision.hasDeckLine, timestamp, timelineGeneration);
    cv::Mat ui_overlay;
    const usv::BridgeMeasureResult r = m_impl->detector.process(f210, f211, img, &ui_overlay);

    system_log::logBridgeResultIfChanged(r);

    cv::Mat combined_overlay = vision.overlay.clone();
    if (!ui_overlay.empty()) {
        if (combined_overlay.empty()) {
            combined_overlay = ui_overlay;
        } else {
            std::vector<cv::Mat> channels;
            cv::split(ui_overlay, channels);
            ui_overlay.copyTo(combined_overlay, channels[3]);
        }
    }
    if (!combined_overlay.empty())
        emit bridgeOverlayReady(
            bgraMatToQImage(combined_overlay), timestamp, timelineGeneration);
    else
        emit bridgeOverlayReady(QImage(), timestamp, timelineGeneration);

    const qint64 visionElapsedMs = visionFrameTimer.elapsed();
    ++m_impl->visionFrameCount;
    m_impl->visionTotalMs += visionElapsedMs;
    m_impl->visionMaxMs = std::max(m_impl->visionMaxMs, visionElapsedMs);
    if (vision.detected)
        ++m_impl->visionDetectedCount;
    if (m_impl->visionStatsTimer.elapsed() >= 1000) {
        const double seconds = std::max(0.001, m_impl->visionStatsTimer.elapsed() / 1000.0);
        const double fps = m_impl->visionFrameCount / seconds;
        emit taskLogMessage(QStringLiteral("视觉检测性能：FPS=%1，命中=%2/%3，%4相机桥墩方位角：%5")
                            .arg(fps, 0, 'f', 1)
                            .arg(m_impl->visionDetectedCount)
                            .arg(m_impl->visionFrameCount)
                            .arg(m_leftCamera ? QStringLiteral("左") : QStringLiteral("右"))
                            .arg(m_impl->pierAzimuths), timelineGeneration);
        if (m_impl->visionResultTopic) {
            m_impl->visionResultTopic->publish(
                std::make_shared<usv::TraditionalBridgeVisionBusResult>(
                    m_impl->latestVisionBusResult));
        }
        m_impl->visionStatsTimer.restart();
        m_impl->visionTotalMs = 0;
        m_impl->visionMaxMs = 0;
        m_impl->visionFrameCount = 0;
        m_impl->visionDetectedCount = 0;
    }

    if (r.is_detected) {
        emit taskLogMessage(
            QStringLiteral("[桥洞检测] 检测到桥洞 | 中心=(%1,%2,%3) 宽=%4m 高=%5m | 210点=%6 211点=%7 图像是否传递：%8")
                .arg(r.average_center_3d.x(), 0, 'f', 2)
                .arg(r.average_center_3d.y(), 0, 'f', 2)
                .arg(r.average_center_3d.z(), 0, 'f', 2)
                .arg(r.width, 0, 'f', 2)
                .arg(r.height, 0, 'f', 2)
                .arg(f210.point_count)
                .arg(f211.point_count)
                .arg(img.image.empty() ? QStringLiteral("否") : QStringLiteral("是")),
            timelineGeneration);
    } else {
        emit taskLogMessage(
            QStringLiteral("[桥洞检测] 未检测到桥洞 | 210点=%1 211点=%2 图片是否传递：%3")
                .arg(f210.point_count)
                .arg(f211.point_count)
                .arg(img.image.empty() ? QStringLiteral("否") : QStringLiteral("是")),
            timelineGeneration);
    }
}

#else // ENABLE_BRIDGE_DETECT

namespace {
QImage visionOverlayToQImage(const cv::Mat &bgra)
{
    if (bgra.empty() || bgra.type() != CV_8UC4)
        return QImage();
    return QImage(bgra.data, bgra.cols, bgra.rows, static_cast<int>(bgra.step), QImage::Format_ARGB32).copy();
}

cv::Mat imageToBgr(const QImage &image)
{
    QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3, const_cast<uchar *>(rgb.bits()), static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgr;
    cv::cvtColor(rgbMat, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}
} // namespace

struct BridgeDetectionWorker::Impl {
    usv::TraditionalBridgeVisionDetector visionDetector;
    QElapsedTimer timer;
    qint64 totalMs = 0;
    qint64 maxMs = 0;
    int frames = 0;
    int hits = 0;
    QString pierAzimuths = QStringLiteral("无");
    usv::TraditionalBridgeVisionBusResult latestVisionBusResult;
    std::shared_ptr<usv::EventTopic<usv::TraditionalBridgeVisionBusResult>> visionResultTopic;
};

BridgeDetectionWorker::BridgeDetectionWorker(bool leftCamera, QObject *parent)
    : QObject(parent)
    , m_impl(new Impl)
    , m_leftCamera(leftCamera)
{
}

BridgeDetectionWorker::~BridgeDetectionWorker()
{
    delete m_impl;
}

void BridgeDetectionWorker::initialize()
{
    m_impl->visionResultTopic = usv::CommunicationManager::instance()
        .getEventTopic<usv::TraditionalBridgeVisionBusResult>(
            "/perception/traditional_bridge_vision", 32);
    m_impl->timer.start();
    emit logMessage(QStringLiteral("传统视觉桥梁检测已初始化；CUDA/PCL 点云桥洞检测未启用。"));
}

void BridgeDetectionWorker::resetForPlaybackSeek()
{
    if (!m_impl)
        return;
    m_impl->visionDetector.reset();
    m_impl->timer.restart();
    m_impl->totalMs = 0;
    m_impl->maxMs = 0;
    m_impl->frames = 0;
    m_impl->hits = 0;
    m_impl->pierAzimuths = QStringLiteral("无");
    m_impl->latestVisionBusResult = {};
}

void BridgeDetectionWorker::runDetection(double timestamp, QVector<M_PointXYZI>,
                                         QVector<M_PointXYZI>, QImage image,
                                         quint64 timelineGeneration)
{
    if (image.isNull())
        return;
    QElapsedTimer frameTimer;
    frameTimer.start();
    const usv::TraditionalBridgeVisionResult result = m_impl->visionDetector.detect(imageToBgr(image));
    const QLineF deckLine(result.deckStart.x, result.deckStart.y, result.deckEnd.x, result.deckEnd.y);
    emit traditionalDeckLineReady(
        deckLine, result.hasDeckLine, timestamp, timelineGeneration);
    m_impl->pierAzimuths = formatPierAzimuths(result.pierCenters, m_leftCamera);
    m_impl->latestVisionBusResult = makeVisionBusResult(
        result, m_leftCamera, timestamp, timelineGeneration);
    emit bridgeOverlayReady(
        visionOverlayToQImage(result.overlay), timestamp, timelineGeneration);
    ++m_impl->frames;
    m_impl->totalMs += frameTimer.elapsed();
    m_impl->maxMs = std::max(m_impl->maxMs, frameTimer.elapsed());
    if (result.detected)
        ++m_impl->hits;
    if (m_impl->timer.elapsed() >= 1000) {
        const double sec = std::max(0.001, m_impl->timer.elapsed() / 1000.0);
        emit taskLogMessage(QStringLiteral("视觉检测性能：FPS=%1，命中=%2/%3，%4相机桥墩方位角：%5")
                            .arg(m_impl->frames / sec, 0, 'f', 1)
                            .arg(m_impl->hits).arg(m_impl->frames)
                            .arg(m_leftCamera ? QStringLiteral("左") : QStringLiteral("右"))
                            .arg(m_impl->pierAzimuths), timelineGeneration);
        if (m_impl->visionResultTopic) {
            m_impl->visionResultTopic->publish(
                std::make_shared<usv::TraditionalBridgeVisionBusResult>(
                    m_impl->latestVisionBusResult));
        }
        m_impl->timer.restart(); m_impl->totalMs = 0; m_impl->maxMs = 0; m_impl->frames = 0; m_impl->hits = 0;
    }
}

#endif // ENABLE_BRIDGE_DETECT

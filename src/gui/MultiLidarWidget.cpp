#include "MultiLidarWidget.h"
#include "SlamMapBinaryIO.h"
#include "SlamMapGeoCorrector.h"
#include "SlamMapGeoUtils.h"
#include "SlamRealtimeMapAlignment.h"
#include "AppConfig.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QDateTime>
#include <QFileInfo>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <QHash>

MultiLidarWidget::MultiLidarWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_vbo(QOpenGLBuffer::VertexBuffer)
    , m_slamVbo(QOpenGLBuffer::VertexBuffer)
    , m_overlayVbo(QOpenGLBuffer::VertexBuffer)
{
    setFocusPolicy(Qt::StrongFocus);
    m_rotationX = 30.0f;
    m_rotationY = 45.0f;
    m_zOffset = -50.0f;
    m_isBevMode = false;
    m_bevRange = 60.0f;
    m_usvTextTimer.start();

    m_fusedRepaintTimer_.setSingleShot(true);
    m_fusedRepaintTimer_.setInterval(kFusedRepaintIntervalMs);
    connect(&m_fusedRepaintTimer_, &QTimer::timeout,
            this, &MultiLidarWidget::onFusedRepaintTick);
}

MultiLidarWidget::~MultiLidarWidget() {
    makeCurrent();
    m_vbo.destroy();
    m_slamVbo.destroy();
    m_overlayVbo.destroy();
    delete m_program;
    doneCurrent();
}

void MultiLidarWidget::updateFusedCloud(const QVector<M_PointXYZI> &cloud, double timestampSec)
{
    m_fusedData = cloud;
    if (timestampSec >= 0.0) {
        m_cloudTimestamp = timestampSec;
        m_hasCloudTimestamp = true;
    } else {
        m_hasCloudTimestamp = false;
    }
    if (!m_fusedRepaintTimer_.isActive())
        m_fusedRepaintTimer_.start();
}

void MultiLidarWidget::onFusedRepaintTick()
{
    update();
}

void MultiLidarWidget::updateBerthResult(const usv::BerthMeasureResult &res)
{
    m_slamBerthOverlay.addBerthResult(res);
    persistSynchronizedBerthResult();
    const bool hasOverlay =
        !res.expanded_berths.empty() ||
        !res.highest_points.empty() ||
        !res.second_highest_points.empty() ||
        res.has_highest_point ||
        res.has_second_highest_point;

    if (res.is_detected && !res.expanded_berths.empty()) {
        m_berthResult = res;
        m_berthVisible = true;
    } else if (hasOverlay) {
        m_berthResult = res;
        m_berthVisible = true;
    }
    update();
}

void MultiLidarWidget::clearBerthResult()
{
    m_berthResult = usv::BerthMeasureResult{};
    m_berthVisible = false;
    update();
}

void MultiLidarWidget::setSlamMapMode(bool enable)
{
    m_showSlamMap = enable;
    if (enable) {
        recenterSlamViewOnShip();
    } else {
        m_viewPanX = 0.0f;
        m_viewPanY = 0.0f;
    }
    update();
}

void MultiLidarWidget::recenterSlamViewOnShip()
{
    if (m_hasSlamPose) {
        m_viewPanX = m_slamX;
        m_viewPanY = m_slamY;
    } else {
        m_viewPanX = 0.0f;
        m_viewPanY = 0.0f;
    }
}

void MultiLidarWidget::clearSlamMap()
{
    m_slamMapCloud.clear();
    m_slamRealtimeMapCloud.clear();
    m_slamLiveScan.clear();
    m_slamKeyframes.clear();
    m_slamRealtimeKeyframes.clear();
    m_slamBerthOverlay.resetAll();
    m_slamMapBerthStore.clear();
    m_slamRealtimeBerthStore.clear();
    m_slamOccupancyWire.clear();
    m_slamMapIsOccupancy = false;
    m_hasHistoricalSlamMap = false;
    m_historicalMapUsesEnu = false;
    m_slamPath.clear();
    m_hasSlamPose = false;
    m_slamPose = Eigen::Isometry3d::Identity();
    m_realtimeMapToEnu = Eigen::Isometry3d::Identity();
    m_hasRealtimeMapAlignment = false;
    m_realtimeSlamGeoAnchor = {};
    m_slamX = 0.0f;
    m_slamY = 0.0f;
    m_slamYaw = 0.0f;
    m_viewPanX = 0.0f;
    m_viewPanY = 0.0f;
    update();
}

bool MultiLidarWidget::hasSlamMapData() const
{
    return !m_slamMapCloud.isEmpty();
}

bool MultiLidarWidget::saveCurrentSlamMap(const QString &filePath, QString *errorMsg) const
{
    slam_map_io::MapArchive archive;
    archive.anchor = m_slamGeoAnchor;
    const bool hasRealtimeSession = !m_slamRealtimeKeyframes.empty();
    archive.keyframes = hasRealtimeSession
        ? m_slamRealtimeKeyframes : m_slamKeyframes;
    archive.berths = hasRealtimeSession
        ? m_slamRealtimeBerthStore.records()
        : m_slamMapBerthStore.records();
    if (archive.keyframes.empty()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("当前没有可保存的 SLAM 关键帧");
        return false;
    }
    return slam_map_io::saveArchive(filePath, archive, errorMsg);
}

bool MultiLidarWidget::loadSlamMap(const QString &filePath, QString *errorMsg)
{
    QVector<M_PointXYZI> loaded;
    const bool isArchive = QFileInfo(filePath).suffix().compare(
        QStringLiteral("slammap"), Qt::CaseInsensitive) == 0;
    const bool isCanonicalEnuMap = isArchive
        && QFileInfo(filePath).fileName().compare(
            QStringLiteral("Mergedclouds_enu_optimized.slammap"),
            Qt::CaseInsensitive) == 0;
    if (isArchive) {
        slam_map_io::MapArchive archive;
        if (!slam_map_io::loadArchive(filePath, archive, errorMsg))
            return false;
        // 普通历史地图沿用原有逐关键帧 ENU 地理校正；canonical 地图已经
        // 是 ENU 坐标，直接读取，避免对离线点云再做一次变换。
        bool geoCorrectionApplied = false;
        if (!isCanonicalEnuMap) {
            const slam_map_geo_correct::Result corrected =
                slam_map_geo_correct::correct(archive);
            archive = corrected.archive;
            geoCorrectionApplied = corrected.diagnostic.applied;
        }
        loaded = slam_map_io::rebuildWorldCloud(
            archive.keyframes, kMaxSlamMapPoints, kSlamMapIntensity);
        m_slamKeyframes = std::move(archive.keyframes);
        m_slamRealtimeKeyframes.clear();
        m_slamGeoAnchor = archive.anchor;
        m_slamMapBerthStore.setRecords(archive.berths);
        m_slamRealtimeBerthStore.clear();
        m_hasHistoricalSlamMap = true;
        // canonical 离线地图本身已经是 ENU 坐标，不能依赖关键帧中不存在的
        // GNSS 参考来推导变换；实时数据使用 georeference(1).json 中的固定矩阵。
        m_historicalMapUsesEnu = geoCorrectionApplied || isCanonicalEnuMap;
    } else {
        if (!slam_map_io::load(filePath, loaded, errorMsg))
            return false;
        m_slamKeyframes.clear();
        m_slamRealtimeKeyframes.clear();
        m_slamGeoAnchor = {};
        m_slamMapBerthStore.clear();
        m_slamRealtimeBerthStore.clear();
        m_hasHistoricalSlamMap = true;
        m_historicalMapUsesEnu = false;
    }

    for (M_PointXYZI &point : loaded)
        point.intensity = kSlamHistoricalMapIntensity;
    m_slamMapCloud = std::move(loaded);
    m_slamRealtimeMapCloud.clear();
    m_slamLiveScan.clear();
    m_slamOccupancyWire.clear();
    m_slamMapIsOccupancy = false;
    m_slamPath.clear();
    m_hasSlamPose = false;
    const slam_realtime_alignment::InitialRealtimeMapAlignment initialAlignment =
        slam_realtime_alignment::initialRealtimeMapAlignment();
    m_realtimeMapToEnu = initialAlignment.mapToEnu;
    m_hasRealtimeMapAlignment = initialAlignment.ready;
    m_realtimeSlamGeoAnchor = {};
    if (isCanonicalEnuMap) {
        // 历史点云保持原样。实时 SLAM 每次会话都有新的 map 原点，
        // 等首个有效关键帧后再动态计算本次会话的 map->ENU 变换。
        qInfo() << "[地图加载] 已加载固定 ENU 历史地图，等待首个实时关键帧建立会话对齐";
    }
    update();
    return true;
}

void MultiLidarWidget::setSlamGeoAnchor(const usv::SlamGeoAnchor &anchor)
{
    m_slamGeoAnchor = anchor;
}

void MultiLidarWidget::setRealtimeSlamGeoAnchor(
    const usv::SlamGeoAnchor &anchor)
{
    if (!m_realtimeSlamGeoAnchor.valid && anchor.valid)
        m_realtimeSlamGeoAnchor = anchor;
}

std::vector<usv::SlamMapBerth> MultiLidarWidget::slamMapBerths() const
{
    return m_slamMapBerthStore.records();
}

void MultiLidarWidget::setSlamMapBerths(
    const std::vector<usv::SlamMapBerth> &berths)
{
    m_slamMapBerthStore.setRecords(berths);
    update();
}

void MultiLidarWidget::updateSlamOdometry(const usv::SlamOdometryState &state)
{
    if (!state.pose_lidar.valid)
        return;

    Eigen::Isometry3d pose = state.pose_lidar.pose;
    if (m_hasHistoricalSlamMap && m_historicalMapUsesEnu
        && m_hasRealtimeMapAlignment)
        pose = slam_realtime_alignment::transformPose(
            state.pose_lidar.pose, m_realtimeMapToEnu);

    m_slamPose = pose;
    m_slamBerthOverlay.addSlamPose(state.timestamp, pose);
    persistSynchronizedBerthResult();
    m_hasSlamPose = true;

    const Eigen::Vector3d t = m_slamPose.translation();
    m_slamX = static_cast<float>(t.x());
    m_slamY = static_cast<float>(t.y());
    m_slamYaw = yawDegFromPose(m_slamPose);
    appendUsvPathPoint(m_slamPath, m_slamX, m_slamY);
    // 俯视图跟随船位，避免船移出视野只剩历史地图
    if (m_isBevMode) {
        m_viewPanX = m_slamX;
        m_viewPanY = m_slamY;
    }

    update();
}

void MultiLidarWidget::updateSlamLiveScan(const QVector<M_PointXYZI> &worldCloud)
{
    m_slamLiveScan = worldCloud;
    if (m_hasHistoricalSlamMap && m_historicalMapUsesEnu
        && m_hasRealtimeMapAlignment) {
        for (M_PointXYZI &point : m_slamLiveScan) {
            const Eigen::Vector3d transformed =
                m_realtimeMapToEnu * Eigen::Vector3d(point.x, point.y, point.z);
            point.x = static_cast<float>(transformed.x());
            point.y = static_cast<float>(transformed.y());
            point.z = static_cast<float>(transformed.z());
        }
    }
    for (M_PointXYZI &p : m_slamLiveScan)
        p.intensity = kSlamLiveScanIntensity;
    update();
}

void MultiLidarWidget::appendSlamKeyframe(const usv::SlamKeyframe &keyframe)
{
    // 启用 OctoMap 栅格显示后，关键帧点云不再累积进 SLAM 地图
    if (m_slamMapIsOccupancy)
        return;

    usv::SlamKeyframe displayKeyframe = keyframe;
    if (m_hasHistoricalSlamMap && m_historicalMapUsesEnu) {
        if (!m_hasRealtimeMapAlignment
            && !captureRealtimeMapAlignment(keyframe)) {
            // 等待首个带有效地理参考的关键帧，禁止把尚未对齐的
            // runtime map 点云混入固定 ENU 历史地图。
            return;
        }
        displayKeyframe = slam_realtime_alignment::transformKeyframe(
            keyframe, m_realtimeMapToEnu);
    }

    usv::SlamKeyframe savedKeyframe = displayKeyframe;
    if (m_hasHistoricalSlamMap && m_historicalMapUsesEnu
        && m_hasRealtimeMapAlignment) {
        // This session keyframe is already in the display/ENU frame.  Clear
        // the raw navigation reference so loading the saved map does not
        // apply the geographic correction for a second time.
        savedKeyframe.geo_reference = usv::SlamGeoPoseReference{};
    }
    m_slamRealtimeKeyframes.push_back(std::move(savedKeyframe));
    m_slamKeyframes.push_back(displayKeyframe);
    mergeKeyframeIntoMap(displayKeyframe);
    update();
}

bool MultiLidarWidget::captureRealtimeMapAlignment(
    const usv::SlamKeyframe &keyframe)
{
    if (m_hasRealtimeMapAlignment)
        return true;

    Eigen::Isometry3d mapToEnu = Eigen::Isometry3d::Identity();
    if (!slam_realtime_alignment::computeMapToEnu(keyframe, mapToEnu))
        return false;

    // 历史地图和本次实时回放通常以不同的首个 GNSS 为 ENU 原点。
    // 先把实时 ENU 原点换算到历史 ENU，再补到 map->ENU 平移中；旋转
    // 仍然使用本次会话首帧的动态结果。
    if (slam_map_geo::validAnchor(m_slamGeoAnchor)) {
        if (!slam_map_geo::validAnchor(m_realtimeSlamGeoAnchor))
            return false;
        Eigen::Vector3d liveOriginInHistoricalEnu;
        QString anchorError;
        if (!slam_map_geo::anchorOffsetEnu(
                m_slamGeoAnchor, m_realtimeSlamGeoAnchor,
                liveOriginInHistoricalEnu, &anchorError)
            || !slam_realtime_alignment::rebaseMapToHistoricalEnu(
                mapToEnu, liveOriginInHistoricalEnu)) {
            qWarning() << "[地图加载] 实时/历史 ENU 原点平移计算失败："
                       << anchorError;
            return false;
        }
        qInfo() << "[地图加载] 实时 ENU 原点相对历史地图偏移："
                << liveOriginInHistoricalEnu.x()
                << liveOriginInHistoricalEnu.y()
                << liveOriginInHistoricalEnu.z();
    }

    m_realtimeMapToEnu = mapToEnu;
    m_hasRealtimeMapAlignment = true;
    // 丢弃变换建立前可能积累的传感器系泊位轨迹，避免混合两个坐标系。
    m_slamBerthOverlay.resetAll();
    m_lastPersistedBerthTimestamp = -std::numeric_limits<double>::infinity();
    return true;
}

void MultiLidarWidget::persistSynchronizedBerthResult()
{
    const auto &worldResult = m_slamBerthOverlay.worldResult();
    if (!worldResult || worldResult->timestamp <= m_lastPersistedBerthTimestamp)
        return;
    m_slamMapBerthStore.observe(*worldResult);
    m_slamRealtimeBerthStore.observe(*worldResult);
    m_lastPersistedBerthTimestamp = worldResult->timestamp;
}

usv::BerthMeasureResult MultiLidarWidget::savedBerthResultForPaint() const
{
    usv::BerthMeasureResult result;
    result.is_detected = !m_slamMapBerthStore.records().empty();
    result.is_using_memory = true;
    for (const usv::SlamMapBerth &record : m_slamMapBerthStore.records()) {
        usv::Berth berth;
        berth.cx = record.x;
        berth.cy = record.y;
        berth.w = record.width;
        berth.l = record.length;
        berth.angle = record.angle_deg;
        berth.kind = static_cast<usv::BerthKind>(record.kind);
        berth.source = usv::BerthDetectionSource::CoordinateLibrary;
        berth.opening_edge = record.opening_edge;
        result.expanded_berths.push_back(berth);
    }
    return result;
}

void MultiLidarWidget::updateSlamOccupancyGrid(QVector<M_PointXYZI> centers, float resolution_m)
{
    m_slamMapIsOccupancy = true;
    m_hasHistoricalSlamMap = false;
    m_slamRealtimeMapCloud.clear();
    m_slamLiveScan.clear();
    if (resolution_m > 1e-4f)
        m_occupancyResolution = resolution_m;
    m_slamMapCloud = std::move(centers);
    rebuildOccupancyWireframe();
    update();
}

void MultiLidarWidget::rebuildOccupancyWireframe()
{
    m_slamOccupancyWire.clear();
    if (m_slamMapCloud.isEmpty())
        return;

    const int n = m_slamMapCloud.size();
    const int stride = (n > kMaxOccupancyWireCubes)
        ? (n + kMaxOccupancyWireCubes - 1) / kMaxOccupancyWireCubes
        : 1;
    const float h = m_occupancyResolution * 0.5f;
    m_slamOccupancyWire.reserve((n / stride + 1) * 24);

    auto pushEdge = [this](float x0, float y0, float z0,
                           float x1, float y1, float z1,
                           unsigned char intensity) {
        M_PointXYZI a, b;
        a.x = x0; a.y = y0; a.z = z0; a.intensity = intensity;
        b.x = x1; b.y = y1; b.z = z1; b.intensity = intensity;
        m_slamOccupancyWire.append(a);
        m_slamOccupancyWire.append(b);
    };

    for (int i = 0; i < n; i += stride) {
        const M_PointXYZI &c = m_slamMapCloud[i];
        const float x = c.x, y = c.y, z = c.z;
        const unsigned char inten = c.intensity ? c.intensity : kSlamMapIntensity;
        const float x0 = x - h, x1 = x + h;
        const float y0 = y - h, y1 = y + h;
        const float z0 = z - h, z1 = z + h;
        // 底面 4 边
        pushEdge(x0, y0, z0, x1, y0, z0, inten);
        pushEdge(x1, y0, z0, x1, y1, z0, inten);
        pushEdge(x1, y1, z0, x0, y1, z0, inten);
        pushEdge(x0, y1, z0, x0, y0, z0, inten);
        // 顶面 4 边
        pushEdge(x0, y0, z1, x1, y0, z1, inten);
        pushEdge(x1, y0, z1, x1, y1, z1, inten);
        pushEdge(x1, y1, z1, x0, y1, z1, inten);
        pushEdge(x0, y1, z1, x0, y0, z1, inten);
        // 立柱 4 边
        pushEdge(x0, y0, z0, x0, y0, z1, inten);
        pushEdge(x1, y0, z0, x1, y0, z1, inten);
        pushEdge(x1, y1, z0, x1, y1, z1, inten);
        pushEdge(x0, y1, z0, x0, y1, z1, inten);
    }
}

void MultiLidarWidget::mergeKeyframeIntoMap(const usv::SlamKeyframe &keyframe)
{
    const Eigen::Isometry3d &pose = keyframe.pose;
    QVector<M_PointXYZI> &target = m_hasHistoricalSlamMap
        ? m_slamRealtimeMapCloud : m_slamMapCloud;
    target.reserve(target.size() + static_cast<int>(keyframe.cloud.size()));
    for (const M_PointXYZI &p : keyframe.cloud) {
        const Eigen::Vector3d wp = pose * Eigen::Vector3d(p.x, p.y, p.z);
        M_PointXYZI q;
        q.x = static_cast<float>(wp.x());
        q.y = static_cast<float>(wp.y());
        q.z = static_cast<float>(wp.z());
        q.intensity = kSlamLiveScanIntensity;
        target.append(q);
    }

    downsampleCloudIfNeeded(target);
}

void MultiLidarWidget::downsampleCloudIfNeeded(QVector<M_PointXYZI> &cloud)
{
    if (cloud.size() <= kMaxSlamMapPoints)
        return;

    QHash<qint64, M_PointXYZI> cells;
    cells.reserve(kMaxSlamMapPoints);
    const float leaf = AppConfig::instance().slamMapVoxelSize();
    for (const M_PointXYZI &p : cloud) {
        const int ix = static_cast<int>(std::floor(p.x / leaf));
        const int iy = static_cast<int>(std::floor(p.y / leaf));
        const int iz = static_cast<int>(std::floor(p.z / leaf));
        const qint64 key = (static_cast<qint64>(ix) & 0x1FFFFF)
            | ((static_cast<qint64>(iy) & 0x1FFFFF) << 21)
            | ((static_cast<qint64>(iz) & 0x1FFFFF) << 42);
        cells.insert(key, p);
    }

    cloud = cells.values().toVector();
    if (cloud.size() > kMaxSlamMapPoints) {
        const int step = (cloud.size() + kMaxSlamMapPoints - 1) / kMaxSlamMapPoints;
        QVector<M_PointXYZI> reduced;
        reduced.reserve(kMaxSlamMapPoints);
        for (int i = 0; i < cloud.size(); i += step)
            reduced.append(cloud[i]);
        cloud.swap(reduced);
    }
}

bool MultiLidarWidget::berthAlignedForPaint() const
{
    const bool hasOverlay =
        !m_berthResult.expanded_berths.empty() ||
        !m_berthResult.highest_points.empty() ||
        !m_berthResult.second_highest_points.empty() ||
        m_berthResult.has_highest_point ||
        m_berthResult.has_second_highest_point;

    if (!m_berthVisible || !hasOverlay)
        return false;
    if (!m_hasCloudTimestamp || m_berthResult.timestamp <= 0.0)
        return true;

    const double diff = m_cloudTimestamp - m_berthResult.timestamp;
    // 回放/多时钟源时可能差出很大一截，此时不做对齐门控，避免框永远不画
    if (std::abs(diff) > 3600.0)
        return true;

    if (diff > kBerthMaxStaleSec)
        return false;
    if (diff < -kBerthToleranceSec)
        return false;

    if (std::abs(diff) <= kBerthToleranceSec)
        return true;
    if (diff > 0.0 && diff <= kBerthMaxLeadSec)
        return true;
    return false;
}

void MultiLidarWidget::onNewTrajectoryPoint(const IMUParsedData &data)
{
    m_lastImu = data;
    m_hasLastImu = true;
    if (data.yawValid)
        m_currentYaw = static_cast<float>(data.yaw);

    // 1. 坐标系初始化：设置第一个点为原点 (0,0,0)
    if (!m_originSet) {
        if (data.longitude != 0.0 && data.latitude != 0.0) {
            m_refLon = data.longitude;
            m_refLat = data.latitude;
            m_originSet = true;
            m_navX = 0.0f;
            m_navY = 0.0f;
            m_hasNavPose = true;
            qDebug() << "GPS Origin Set:" << m_refLon << m_refLat;
            emitUsvStateText();
        }
        return;
    }

    // 2. 经纬度转米 (简易墨卡托投影近似)
    const double radLat = m_refLat * M_PI / 180.0;
    const float deltaX = static_cast<float>((data.longitude - m_refLon) * 111320.0 * std::cos(radLat));
    const float deltaY = static_cast<float>((data.latitude - m_refLat) * 111000.0);

    m_navX = deltaX;
    m_navY = deltaY;
    m_hasNavPose = true;

    // 3. 距离抽稀：避免点云过于密集消耗性能
    appendUsvPathPoint(m_shipPath, deltaX, deltaY);
    if (!m_showSlamMap) {
        prunePathByDistanceFrom(m_shipPath, deltaX, deltaY,
                                radarTrailMaxDistanceM() * 1.05f);
    }

    emitUsvStateText();
    update();
}

void MultiLidarWidget::appendUsvPathPoint(QList<TrajectoryPoint> &path, float x, float y)
{
    bool shouldAdd = false;
    if (path.isEmpty()) {
        shouldAdd = true;
    } else {
        const float lastX = path.last().x;
        const float lastY = path.last().y;
        const float dist2 = (x - lastX) * (x - lastX) + (y - lastY) * (y - lastY);
        if (dist2 > 0.04f)
            shouldAdd = true;
    }

    if (shouldAdd) {
        TrajectoryPoint newPt;
        newPt.x = x;
        newPt.y = y;
        path.append(newPt);
        if (path.size() > 10000)
            path.removeFirst();
    }
}

void MultiLidarWidget::prunePathByDistanceFrom(QList<TrajectoryPoint> &path,
                                               float cx, float cy, float maxDist)
{
    if (maxDist <= 0.0f)
        return;
    const float maxDist2 = maxDist * maxDist;
    while (path.size() > 1) {
        const float dx = path.first().x - cx;
        const float dy = path.first().y - cy;
        if (dx * dx + dy * dy <= maxDist2)
            break;
        path.removeFirst();
    }
}

float MultiLidarWidget::radarTrailMaxDistanceM() const
{
    return qMax(kRadarTrailMinDistM, m_bevRange * kRadarTrailRangeFactor);
}

QList<MultiLidarWidget::TrajectoryPoint> MultiLidarWidget::buildRadarFramePath(
    const QList<TrajectoryPoint> &worldPath,
    float shipWorldX, float shipWorldY, float yawDeg,
    float maxDistM) const
{
    QList<TrajectoryPoint> radarPath;
    if (worldPath.isEmpty())
        return radarPath;

    const float yawRad = qDegreesToRadians(yawDeg);
    const float cosY = std::cos(yawRad);
    const float sinY = std::sin(yawRad);
    const float maxDist2 = maxDistM * maxDistM;

    for (const TrajectoryPoint &p : worldPath) {
        const float dx = p.x - shipWorldX;
        const float dy = p.y - shipWorldY;
        const float rx = dx * cosY + dy * sinY;
        const float ry = -dx * sinY + dy * cosY;
        if (rx * rx + ry * ry > maxDist2)
            continue;

        TrajectoryPoint rp;
        rp.x = rx;
        rp.y = ry;
        radarPath.append(rp);
    }

    if (radarPath.isEmpty()
        || std::abs(radarPath.last().x) > 1e-3f
        || std::abs(radarPath.last().y) > 1e-3f) {
        radarPath.append({0.0f, 0.0f});
    }
    return radarPath;
}

float MultiLidarWidget::yawDegFromPose(const Eigen::Isometry3d &pose)
{
    const Eigen::Matrix3d &R = pose.linear();
    return static_cast<float>(std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI);
}

// 供外部按钮调用的公共槽函数
void MultiLidarWidget::setBevMode(bool enable) {
    //qDebug() << "Switching BEV Mode to:" << enable; // 确保输出了 true
    m_isBevMode = enable;
    update();
}

void MultiLidarWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // 深色背景
    glEnable(GL_DEPTH_TEST);
    glPointSize(1.0f); // 设置点云大小

    m_program = new QOpenGLShaderProgram(this);

    // --- 顶点着色器：处理位置与强度动态上色 ---
    const char *vsrc =
        "attribute vec3 posAttr;\n"
        "attribute float intensityAttr;\n"
        "varying vec3 vColor;\n"
        "uniform mat4 matrix;\n"
        "uniform int useUsvColor;\n"
        "uniform vec3 usvColor;\n"
        "void main() {\n"
        "   gl_Position = matrix * vec4(posAttr, 1.0);\n"
        "   if (useUsvColor != 0) gl_PointSize = 10.0;\n"
        "   else gl_PointSize = 1.0;\n"
        "   if (useUsvColor != 0) {\n"
        "       vColor = usvColor;\n"
        "       return;\n"
        "   }\n"
        "   float val = intensityAttr * 255.0 + 0.5;\n"
        "\n"
         "   if (val >= 230.0)      vColor = vec3(1.0, 1.0, 1.0); // 210: 白色\n"
         "   else if (val >= 210.0) vColor = vec3(1.0, 0.92, 0.35); // 历史地图浅黄色\n"
         "   else if (val >= 180.0) vColor = vec3(1.0, 1.0, 0.0); // 214: 黄色\n"
        "   else if (val >= 130.0) vColor = vec3(1.0, 0.0, 0.0); // 211: 红色\n"
        "   else if (val >= 80.0)  vColor = vec3(0.0, 1.0, 0.0); // 213: 绿色\n"
        "   else if (val >= 30.0)  vColor = vec3(0.0, 0.5, 1.0); // 201: 蓝色\n"
        "   else                   vColor = vec3(0.4, 0.4, 0.4); // 默认灰色\n"
        "}\n";

    const char *fsrc =
        "varying vec3 vColor;\n"
        "void main() {\n"
        "   gl_FragColor = vec4(vColor, 1.0);\n"
        "}\n";

    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vsrc);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fsrc);
    m_program->link();

    m_vbo.create();
    m_vbo.bind();
    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_vbo.allocate(MAX_POINTS * sizeof(M_PointXYZI));
    m_vbo.release();

    m_slamVbo.create();
    m_slamVbo.bind();
    m_slamVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_slamVbo.allocate(kMaxSlamMapPoints * sizeof(M_PointXYZI));
    m_slamVbo.release();

    m_overlayVbo.create();
    m_overlayVbo.bind();
    m_overlayVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_overlayVbo.allocate(kMaxOverlayVerts * sizeof(M_PointXYZI));
    m_overlayVbo.release();
}
void MultiLidarWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const bool slamCloudReady = m_showSlamMap
        && (!m_slamMapCloud.isEmpty() || !m_slamLiveScan.isEmpty()
            || !m_slamOccupancyWire.isEmpty());
    const bool fusedCloudReady = !m_showSlamMap
        && (!m_fusedData.isEmpty() || m_berthVisible);
    const bool useSlamUsv = m_showSlamMap && m_hasSlamPose;
    const bool useGnssUsv = !m_showSlamMap && m_hasNavPose;
    const bool usvReady = useSlamUsv || useGnssUsv
        || (!m_showSlamMap && m_shipPath.size() >= 2)
        || (m_showSlamMap && m_slamPath.size() >= 2);

    if (slamCloudReady || fusedCloudReady || m_isBevMode || usvReady) {
        m_program->bind();

        QMatrix4x4 matrix;
        if (m_isBevMode) {
            const float aspect = static_cast<float>(width()) / static_cast<float>(height());
            const float range = m_bevRange;
            matrix.ortho(-range * aspect, range * aspect, -range, range, 0.1f, 1000.0f);
            matrix.lookAt(QVector3D(0, 0, 100), QVector3D(0, 0, 0), QVector3D(0, 1, 0));
            if (m_showSlamMap)
                matrix.translate(-m_viewPanX, -m_viewPanY, 0.0f);
        } else {
            matrix.perspective(45.0f, static_cast<float>(width()) / static_cast<float>(height()),
                               0.1f, 2000.0f);
            matrix.translate(0, 0, m_zOffset);
            matrix.rotate(m_rotationX, 1, 0, 0);
            matrix.rotate(m_rotationY, 0, 0, 1);
        }
        m_program->setUniformValue("matrix", matrix);
        m_program->setUniformValue("useUsvColor", 0);

        if (m_isBevMode) {
            if (m_showSlamMap)
                drawRadarGrid(m_slamX, m_slamY);  // 同心圆锁定船位，随船移动
            else
                drawRadarGrid();
        }
        if (m_showSlamMap)
            renderSlamMap(matrix);
        else
            renderFusedCloud(matrix);

        if (m_showSlamMap) {
            const auto &worldResult = m_slamBerthOverlay.worldResult();
            const usv::BerthMeasureResult *slamBerthResult =
                (worldResult && m_slamBerthOverlay.visibleAtLatestSlamPose())
                ? &(*worldResult) : nullptr;
            if (slamBerthResult) {
                drawBerthOverlays(matrix, *slamBerthResult, false);
            } else {
                const usv::BerthMeasureResult savedBerthResult =
                    savedBerthResultForPaint();
                if (!savedBerthResult.expanded_berths.empty())
                    drawBerthOverlays(matrix, savedBerthResult, false);
            }
        } else if (berthAlignedForPaint()) {
            drawBerthOverlays(matrix, m_berthResult, true);
        }

        if (usvReady)
            drawUsvOverlays(matrix);

        m_program->release();
    }
}

//void MultiLidarWidget::paintGL() {
//    // 1. 清除颜色缓冲区和深度缓冲区（擦除上一帧）
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    // 2. 切换到模型视图矩阵（操作物体和相机）
//    glMatrixMode(GL_MODELVIEW);

//    // 3. 重置坐标系（重要！）
//    // 此时，绘图的原点位于屏幕正中心，且没有任何旋转
//    glLoadIdentity();

//    // 4. 设置摄像机（视角控制）
//    // 注意：这里的平移和旋转会累加在单位矩阵上
//    glTranslatef(0.0f, 0.0f, -m_zOffset); // 镜头拉远
//    glRotatef(m_rotationX, 1.0f, 0.0f, 0.0f); // 俯仰角
//    glRotatef(m_rotationY, 0.0f, 0.0f, 1.0f); // 航向角

//    // 5. 此时绘制船只图标，它会受到上述摄像机变换的影响
//    drawShipIcon();

//    // 6. 绘制点云
//    //drawPointClouds();


//    // 如果没有点云且没开俯视图网格，直接返回
//    if (m_fusedData.isEmpty() && !m_isBevMode) return;

//    m_program->bind();

//    // --- 1. 计算投影变换矩阵 (切换 BEV 与 3D) ---
//    QMatrix4x4 matrix;
//    if (m_isBevMode) {
//        // 正交投影：适合测距，没有近大远小
//        float aspect = (float)width() / height();
//        float range = m_bevRange;
//        matrix.ortho(-range * aspect, range * aspect, -range, range, 0.1f, 1000.0f);
//        // 相机垂直向下看 (LookAt: 位置, 焦点, 上方向)
//        matrix.lookAt(QVector3D(0, 0, 100), QVector3D(0, 0, 0), QVector3D(0, 1, 0));
//    } else {
//        // 透视投影：原有的 3D 自由视角
//        matrix.perspective(45.0f, (float)width() / height(), 0.1f, 2000.0f);
//        matrix.translate(0, 0, m_zOffset);
//        matrix.rotate(m_rotationX, 1, 0, 0);
//        matrix.rotate(m_rotationY, 0, 0, 1);
//    }
//    m_program->setUniformValue("matrix", matrix);

//    // --- 2. 绘制俯视图网格 (仅在 BEV 模式下显示) ---
//    if (m_isBevMode) {
//        m_vbo.release(); // 绘制内存数组前必须释放 VBO
//        drawRadarGrid();
//    }

//    // --- 3. 绘制融合点云 ---
//    if (!m_fusedData.isEmpty()) {
//        m_vbo.bind();
//        m_vbo.write(0, m_fusedData.constData(), m_fusedData.size() * sizeof(M_PointXYZI));

//        int posLoc = m_program->attributeLocation("posAttr");
//        m_program->enableAttributeArray(posLoc);
//        // 步长设为 sizeof(M_PointXYZI)，即 16
//        m_program->setAttributeBuffer(posLoc, GL_FLOAT, 0, 3, sizeof(M_PointXYZI));

//        int intensityLoc = m_program->attributeLocation("intensityAttr");
//        m_program->enableAttributeArray(intensityLoc);
//        // 偏移量使用 offsetof，步长同样设为 16
//        m_program->setAttributeBuffer(intensityLoc, GL_UNSIGNED_BYTE, offsetof(M_PointXYZI, intensity), 1, sizeof(M_PointXYZI));

//        glDrawArrays(GL_POINTS, 0, m_fusedData.size());
//        m_vbo.release();
//    }

//    m_program->release();
//}

void MultiLidarWidget::drawOverlayArrays(GLenum mode, const QVector<M_PointXYZI> &verts,
                                         bool solidColor, const QVector3D &color)
{
    if (!m_program || verts.isEmpty() || !m_overlayVbo.isCreated())
        return;
    if (verts.size() > kMaxOverlayVerts)
        return;

    const int posLoc = m_program->attributeLocation("posAttr");
    const int intensityLoc = m_program->attributeLocation("intensityAttr");
    if (posLoc < 0)
        return;

    m_program->setUniformValue("useUsvColor", solidColor ? 1 : 0);
    if (solidColor)
        m_program->setUniformValue("usvColor", color);

    m_overlayVbo.bind();
    m_overlayVbo.write(0, verts.constData(), verts.size() * static_cast<int>(sizeof(M_PointXYZI)));

    m_program->enableAttributeArray(posLoc);
    m_program->setAttributeBuffer(posLoc, GL_FLOAT, 0, 3, sizeof(M_PointXYZI));
    if (intensityLoc >= 0) {
        m_program->enableAttributeArray(intensityLoc);
        m_program->setAttributeBuffer(intensityLoc, GL_UNSIGNED_BYTE,
                                      offsetof(M_PointXYZI, intensity), 1, sizeof(M_PointXYZI));
    }

    glDrawArrays(mode, 0, verts.size());

    m_program->disableAttributeArray(posLoc);
    if (intensityLoc >= 0)
        m_program->disableAttributeArray(intensityLoc);
    m_overlayVbo.release();
    m_program->setUniformValue("useUsvColor", 0);
}

void MultiLidarWidget::drawBerthOverlays(
    const QMatrix4x4 &viewMatrix,
    const usv::BerthMeasureResult &result,
    bool useEdgeHeights)
{
    if (!m_program || !m_program->isLinked())
        return;

    m_program->bind();
    m_program->setUniformValue("matrix", viewMatrix);

    const float z = 0.15f;
    const float anchorZ = 0.35f;
    constexpr unsigned char kUShapeBerthEdgeIntensity = 80;   // shader: 绿色
    constexpr unsigned char kLineBerthEdgeIntensity = 50;     // shader: 蓝色
    constexpr unsigned char kLibraryUShapeIntensity = 180;    // shader: 黄色
    constexpr unsigned char kHighestAnchorIntensity = 130;
    constexpr unsigned char kSecondAnchorIntensity = 30;
    constexpr unsigned char kOccupiedMarkerIntensity = 130;

    auto pushPt = [](QVector<M_PointXYZI> &out, float x, float y, float zz, unsigned char inten) {
        M_PointXYZI p;
        p.x = x; p.y = y; p.z = zz; p.intensity = inten;
        out.append(p);
    };

    auto drawRect = [&](const usv::Berth &b, bool skipOpeningEdge, bool useEdgeHeights,
                        unsigned char inten) {
        const double rad = b.angle * M_PI / 180.0;
        const double cosA = std::cos(rad);
        const double sinA = std::sin(rad);
        const double hw = b.w * 0.5;
        const double hl = b.l * 0.5;

        const double localX[4] = {-hw, hw, hw, -hw};
        const double localY[4] = {-hl, -hl, hl, hl};

        float cx[4], cy[4];
        for (int i = 0; i < 4; ++i) {
            cx[i] = static_cast<float>(b.cx + localX[i] * cosA - localY[i] * sinA);
            cy[i] = static_cast<float>(b.cy + localX[i] * sinA + localY[i] * cosA);
        }

        const int skip_edge = skipOpeningEdge ? b.opening_edge : -1;
        auto edgeZ = [&](int edge) -> float {
            if (useEdgeHeights &&
                edge >= 0 && edge < 4 &&
                b.edge_height_valid[edge] &&
                std::isfinite(b.edge_heights[edge])) {
                return static_cast<float>(b.edge_heights[edge]);
            }
            return z;
        };

        QVector<M_PointXYZI> edges;
        edges.reserve(8);
        for (int edge = 0; edge < 4; ++edge) {
            if (edge == skip_edge)
                continue;
            const float edge_height = edgeZ(edge);
            const int j = (edge + 1) % 4;
            pushPt(edges, cx[edge], cy[edge], edge_height, inten);
            pushPt(edges, cx[j], cy[j], edge_height, inten);
        }
        glLineWidth(2.0f);
        drawOverlayArrays(GL_LINES, edges, false);
    };

    auto drawAnchor = [&](const Eigen::Vector3d &p, unsigned char inten, float radius) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()))
            return;
        const float x = static_cast<float>(p.x());
        const float y = static_cast<float>(p.y());
        QVector<M_PointXYZI> marker;
        marker.reserve(8);
        pushPt(marker, x - radius, y, anchorZ, inten);
        pushPt(marker, x + radius, y, anchorZ, inten);
        pushPt(marker, x, y - radius, anchorZ, inten);
        pushPt(marker, x, y + radius, anchorZ, inten);
        pushPt(marker, x - radius * 0.7f, y - radius * 0.7f, anchorZ, inten);
        pushPt(marker, x + radius * 0.7f, y + radius * 0.7f, anchorZ, inten);
        pushPt(marker, x - radius * 0.7f, y + radius * 0.7f, anchorZ, inten);
        pushPt(marker, x + radius * 0.7f, y - radius * 0.7f, anchorZ, inten);
        glLineWidth(4.0f);
        drawOverlayArrays(GL_LINES, marker, false);
    };

    for (int i = 0; i < static_cast<int>(result.expanded_berths.size()); ++i) {
        const usv::Berth &b = result.expanded_berths[i];
        unsigned char edgeIntensity = (b.kind == usv::BerthKind::Line)
            ? kLineBerthEdgeIntensity
            : kUShapeBerthEdgeIntensity;
        if (b.source == usv::BerthDetectionSource::CoordinateLibrary) {
            if (b.kind == usv::BerthKind::Line)
                edgeIntensity = static_cast<unsigned char>(edgeIntensity * 0.55f);
            else
                edgeIntensity = kLibraryUShapeIntensity;
        }
        drawRect(b, true, useEdgeHeights, edgeIntensity);

        float centerZ = z;
        int validHeightCount = 0;
        for (int edge = 0; edge < 4; ++edge) {
            if (edge != b.opening_edge &&
                b.edge_height_valid[edge] &&
                std::isfinite(b.edge_heights[edge])) {
                centerZ += static_cast<float>(b.edge_heights[edge]);
                ++validHeightCount;
            }
        }
        if (validHeightCount > 0)
            centerZ = (centerZ - z) / static_cast<float>(validHeightCount);

        QVector<M_PointXYZI> cross;
        pushPt(cross, static_cast<float>(b.cx - 1.0), static_cast<float>(b.cy), centerZ, edgeIntensity);
        pushPt(cross, static_cast<float>(b.cx + 1.0), static_cast<float>(b.cy), centerZ, edgeIntensity);
        pushPt(cross, static_cast<float>(b.cx), static_cast<float>(b.cy - 1.0), centerZ, edgeIntensity);
        pushPt(cross, static_cast<float>(b.cx), static_cast<float>(b.cy + 1.0), centerZ, edgeIntensity);
        glLineWidth(2.0f);
        drawOverlayArrays(GL_LINES, cross, false);

        if (b.has_ship) {
            const double markerRadius = std::clamp(std::min(b.w, b.l) * 0.18, 0.8, 1.8);
            const double rad = b.angle * M_PI / 180.0;
            const double cosA = std::cos(rad);
            const double sinA = std::sin(rad);
            const double localTriangle[3][2] = {
                {0.0, markerRadius},
                {-0.866 * markerRadius, -0.5 * markerRadius},
                {0.866 * markerRadius, -0.5 * markerRadius},
            };
            QVector<M_PointXYZI> triangle;
            triangle.reserve(3);
            for (const auto &local : localTriangle) {
                pushPt(triangle,
                    static_cast<float>(b.cx + cosA * local[0] - sinA * local[1]),
                    static_cast<float>(b.cy + sinA * local[0] + cosA * local[1]),
                    centerZ + 0.5f, kOccupiedMarkerIntensity);
            }

            // An occupied berth contains dense points above the berth-edge height.
            // Draw the marker as an overlay so the ship cloud cannot hide it.
            const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
            glDisable(GL_DEPTH_TEST);
            drawOverlayArrays(GL_TRIANGLES, triangle, true, QVector3D(1.0f, 0.0f, 0.0f));

            QVector<M_PointXYZI> outline = triangle;
            outline.append(triangle.front());
            glLineWidth(4.0f);
            drawOverlayArrays(GL_LINE_STRIP, outline, true, QVector3D(1.0f, 0.0f, 0.0f));
            if (depthTestEnabled)
                glEnable(GL_DEPTH_TEST);
        }
    }

    if (!result.highest_points.empty()) {
        for (const Eigen::Vector3d &p : result.highest_points)
            drawAnchor(p, kHighestAnchorIntensity, 0.8f);
    } else if (result.has_highest_point) {
        drawAnchor(result.highest_point, kHighestAnchorIntensity, 0.8f);
    }
    if (!result.second_highest_points.empty()) {
        for (const Eigen::Vector3d &p : result.second_highest_points)
            drawAnchor(p, kSecondAnchorIntensity, 0.8f);
    } else if (result.has_second_highest_point) {
        drawAnchor(result.second_highest_point, kSecondAnchorIntensity, 0.8f);
    }
}

void MultiLidarWidget::drawRadarGrid(float centerX, float centerY) {
    auto pushPt = [](QVector<M_PointXYZI> &out, float x, float y) {
        M_PointXYZI p;
        p.x = x; p.y = y; p.z = 0.0f; p.intensity = 0;
        out.append(p);
    };

    glLineWidth(1.0f);
    for (int r = 10; r <= 60; r += 10) {
        QVector<M_PointXYZI> circlePoints;
        circlePoints.reserve(73);
        for (int i = 0; i <= 360; i += 5) {
            const float theta = qDegreesToRadians(static_cast<float>(i));
            pushPt(circlePoints, centerX + r * std::cos(theta), centerY + r * std::sin(theta));
        }
        drawOverlayArrays(GL_LINE_STRIP, circlePoints, false);
    }

    QVector<M_PointXYZI> crossLines;
    pushPt(crossLines, centerX - 70, centerY);
    pushPt(crossLines, centerX + 70, centerY);
    pushPt(crossLines, centerX, centerY - 70);
    pushPt(crossLines, centerX, centerY + 70);
    drawOverlayArrays(GL_LINES, crossLines, false);
}

void MultiLidarWidget::drawUsvOverlays(const QMatrix4x4 &viewMatrix)
{
    if (!m_program || !m_program->isLinked())
        return;

    m_program->bind();
    m_program->setUniformValue("matrix", viewMatrix);

    if (m_showSlamMap && m_hasSlamPose) {
        drawUsvPoseOverlay(viewMatrix, m_slamX, m_slamY, m_slamYaw, m_slamPath, true);
    } else if (!m_showSlamMap && (m_hasNavPose || m_shipPath.size() >= 1)) {
        // 实时点云在 210 雷达系：船体与点云同系固定，船首沿局部 +Y（与俯视 up 一致）。
        // GNSS yaw 只用于把 ENU 航迹拧到雷达系，不可再用来转船体/箭头，否则与点云错系。
        const QList<TrajectoryPoint> radarPath =
            buildRadarFramePath(m_shipPath, m_navX, m_navY, m_currentYaw,
                                radarTrailMaxDistanceM());
        constexpr float kRadarBodyYawDeg = 0.0f;
        drawUsvPoseOverlay(viewMatrix, 0.0f, 0.0f, kRadarBodyYawDeg, radarPath, m_hasNavPose);
    }
}

void MultiLidarWidget::drawUsvPoseOverlay(const QMatrix4x4 &viewMatrix,
                                          float shipX, float shipY, float yawDeg,
                                          const QList<TrajectoryPoint> &path, bool hasPose)
{
    if (!m_program || !m_program->isLinked())
        return;
    if (path.size() < 2 && !hasPose)
        return;

    m_program->setUniformValue("matrix", viewMatrix);

    const float zTrail = 0.8f;
    const float zPose = 1.2f;
    const float yawRad = qDegreesToRadians(yawDeg);
    const float cosY = std::cos(yawRad);
    const float sinY = std::sin(yawRad);

    auto pushPt = [](QVector<M_PointXYZI> &out, float x, float y, float z) {
        M_PointXYZI p;
        p.x = x; p.y = y; p.z = z; p.intensity = 0;
        out.append(p);
    };

    auto worldPt = [&](float lx, float ly, float z) {
        return QVector3D(shipX + lx * cosY - ly * sinY,
                         shipY + lx * sinY + ly * cosY,
                         z);
    };

    glDisable(GL_DEPTH_TEST);

    if (path.size() >= 2) {
        QVector<M_PointXYZI> trail;
        trail.reserve(path.size());
        for (const TrajectoryPoint &p : path)
            pushPt(trail, p.x, p.y, zTrail);
        glLineWidth(6.0f);
        drawOverlayArrays(GL_LINE_STRIP, trail, true, QVector3D(0.15f, 1.0f, 0.35f));
    }

    if (hasPose) {
        QVector<M_PointXYZI> cross;
        const float crossHalf = 3.0f;
        pushPt(cross, shipX - crossHalf, shipY, zPose);
        pushPt(cross, shipX + crossHalf, shipY, zPose);
        pushPt(cross, shipX, shipY - crossHalf, zPose);
        pushPt(cross, shipX, shipY + crossHalf, zPose);
        glLineWidth(5.0f);
        drawOverlayArrays(GL_LINES, cross, true, QVector3D(0.0f, 1.0f, 1.0f));

        const float arrowLen = 14.0f;
        const QVector3D arrowTip = worldPt(0.0f, arrowLen, zPose);
        QVector<M_PointXYZI> heading;
        pushPt(heading, shipX, shipY, zPose);
        pushPt(heading, arrowTip.x(), arrowTip.y(), arrowTip.z());
        glLineWidth(6.0f);
        drawOverlayArrays(GL_LINES, heading, true, QVector3D(1.0f, 0.95f, 0.0f));

        const float wing = 3.5f;
        const float back = 4.0f;
        const QVector3D w0 = worldPt(-wing, arrowLen - back, zPose);
        const QVector3D w1 = worldPt(wing, arrowLen - back, zPose);
        QVector<M_PointXYZI> arrowHead;
        pushPt(arrowHead, arrowTip.x(), arrowTip.y(), arrowTip.z());
        pushPt(arrowHead, w0.x(), w0.y(), w0.z());
        pushPt(arrowHead, arrowTip.x(), arrowTip.y(), arrowTip.z());
        pushPt(arrowHead, w1.x(), w1.y(), w1.z());
        glLineWidth(4.0f);
        drawOverlayArrays(GL_LINES, arrowHead, true, QVector3D(1.0f, 0.95f, 0.0f));

        const float halfL = 6.0f;
        const float halfW = 2.5f;
        const QVector3D hull[4] = {
            worldPt(-halfW, -halfL, zPose),
            worldPt(halfW, -halfL, zPose),
            worldPt(halfW, halfL, zPose),
            worldPt(-halfW, halfL, zPose),
        };
        QVector<M_PointXYZI> hullLines;
        for (int i = 0; i < 4; ++i) {
            const QVector3D &a = hull[i];
            const QVector3D &b = hull[(i + 1) % 4];
            pushPt(hullLines, a.x(), a.y(), a.z());
            pushPt(hullLines, b.x(), b.y(), b.z());
        }
        glLineWidth(4.0f);
        drawOverlayArrays(GL_LINES, hullLines, true, QVector3D(0.0f, 0.85f, 1.0f));
    }

    glEnable(GL_DEPTH_TEST);
    m_program->setUniformValue("useUsvColor", 0);
}

void MultiLidarWidget::emitUsvStateText()
{
    if (!m_hasLastImu)
        return;

    const qint64 now = m_usvTextTimer.elapsed();
    if (now - m_lastUsvTextMs < kUsvTextIntervalMs)
        return;
    m_lastUsvTextMs = now;

    const IMUParsedData &d = m_lastImu;
    QString text;
    text += QStringLiteral("USV 当前位置（GNSS/INS）\n");
    if (m_originSet) {
        text += QStringLiteral("参考原点 经度: %1\n").arg(m_refLon, 0, 'f', 8);
        text += QStringLiteral("参考原点 纬度: %1\n").arg(m_refLat, 0, 'f', 8);
    }
    text += QStringLiteral("位置 X(E): %1 m\n").arg(m_navX, 0, 'f', 2);
    text += QStringLiteral("位置 Y(N): %1 m\n").arg(m_navY, 0, 'f', 2);
    text += QStringLiteral("经度: %1\n").arg(d.longitude, 0, 'f', 8);
    text += QStringLiteral("纬度: %1\n").arg(d.latitude, 0, 'f', 8);
    const QDateTime dt = QDateTime::fromMSecsSinceEpoch(
        static_cast<qint64>(d.timestamp * 1000.0 + 0.5), Qt::LocalTime);
    text += QStringLiteral("时间: %1\n").arg(dt.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")));
    emit usvStateUpdated(text);
}

void MultiLidarWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

// --- 交互逻辑优化 ---
void MultiLidarWidget::wheelEvent(QWheelEvent *event) {
    if (m_isBevMode) {
        // 俯视图模式控制可见半径的缩放
        if (event->angleDelta().y() > 0) m_bevRange *= 0.9f;
        else m_bevRange *= 1.1f;
        m_bevRange = qBound(10.0f, m_bevRange, 300.0f);
    } else {
        // 3D 模式控制相机 Z 轴位移
        m_zOffset += event->angleDelta().y() / 120.0f * 5.0f;
    }
    update();
}

void MultiLidarWidget::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->pos();
}

void MultiLidarWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        int dx = event->x() - m_lastMousePos.x();
        int dy = event->y() - m_lastMousePos.y();

        if (m_showSlamMap && m_isBevMode) {
            const float aspect = static_cast<float>(width()) / static_cast<float>(qMax(1, height()));
            const float unitsPerPixelX = (2.0f * m_bevRange * aspect) / static_cast<float>(qMax(1, width()));
            const float unitsPerPixelY = (2.0f * m_bevRange) / static_cast<float>(qMax(1, height()));
            m_viewPanX -= static_cast<float>(dx) * unitsPerPixelX;
            m_viewPanY += static_cast<float>(dy) * unitsPerPixelY;
        } else if (!m_isBevMode) {
            m_rotationX += dy * 0.5f;
            m_rotationY += dx * 0.5f;
        }
        update();
    }
    m_lastMousePos = event->pos();
}
void MultiLidarWidget::renderFusedCloud(const QMatrix4x4 &viewMatrix) {
    if (m_fusedData.isEmpty()) return;

    m_program->bind();

    // 1. 关键：同步矩阵，确保点云位置与船体/网格对齐
    m_program->setUniformValue("matrix", viewMatrix);

    // 2. 强制缩小点的大小（针对 Shader 模式）
    glPointSize(1.0f);
    // 如果你在 initializeGL 开启了 GL_PROGRAM_POINT_SIZE，
    // 请确保 Shader 里的 gl_PointSize 也是 1.0

    // 3. 绑定并写入 VBO
    m_vbo.bind();
    m_vbo.write(0, m_fusedData.constData(), m_fusedData.size() * sizeof(M_PointXYZI));

    // 4. 设置顶点属性
    int posLoc = m_program->attributeLocation("posAttr");
    m_program->enableAttributeArray(posLoc);
    m_program->setAttributeBuffer(posLoc, GL_FLOAT, 0, 3, sizeof(M_PointXYZI));

    int intensityLoc = m_program->attributeLocation("intensityAttr");
    m_program->enableAttributeArray(intensityLoc);
    // 假设 M_PointXYZI 结构中 intensity 偏移量正确
    m_program->setAttributeBuffer(intensityLoc, GL_UNSIGNED_BYTE, offsetof(M_PointXYZI, intensity), 1, sizeof(M_PointXYZI));

    // 5. 绘制
    glDrawArrays(GL_POINTS, 0, m_fusedData.size());

    // 6. 清理状态
    m_program->disableAttributeArray(posLoc);
    m_program->disableAttributeArray(intensityLoc);
    m_vbo.release();
    m_program->release();
}

// SLAM 地图模式：OctoMap occupied 线框方块（或回退关键帧点云）+ 可选当前扫描
void MultiLidarWidget::renderSlamMap(const QMatrix4x4 &viewMatrix)
{
    if (!m_program)
        return;

    auto drawArrays = [&](const QVector<M_PointXYZI> &cloud, GLenum mode) {
        if (cloud.isEmpty())
            return;
        const int bytes = cloud.size() * static_cast<int>(sizeof(M_PointXYZI));
        if (bytes > kMaxSlamMapPoints * static_cast<int>(sizeof(M_PointXYZI)))
            return;
        m_slamVbo.bind();
        m_slamVbo.write(0, cloud.constData(), bytes);

        const int posLoc = m_program->attributeLocation("posAttr");
        const int intensityLoc = m_program->attributeLocation("intensityAttr");
        m_program->enableAttributeArray(posLoc);
        m_program->setAttributeBuffer(posLoc, GL_FLOAT, 0, 3, sizeof(M_PointXYZI));
        m_program->enableAttributeArray(intensityLoc);
        m_program->setAttributeBuffer(intensityLoc, GL_UNSIGNED_BYTE,
                                      offsetof(M_PointXYZI, intensity), 1, sizeof(M_PointXYZI));
        glDrawArrays(mode, 0, cloud.size());
        m_program->disableAttributeArray(posLoc);
        m_program->disableAttributeArray(intensityLoc);
        m_slamVbo.release();
    };

    if (m_slamMapCloud.isEmpty() && m_slamRealtimeMapCloud.isEmpty()
        && m_slamLiveScan.isEmpty() && m_slamOccupancyWire.isEmpty())
        return;

    m_program->bind();
    m_program->setUniformValue("matrix", viewMatrix);
    m_program->setUniformValue("useUsvColor", 0);

    if (m_slamMapIsOccupancy && !m_slamOccupancyWire.isEmpty()) {
        glLineWidth(1.0f);
        drawArrays(m_slamOccupancyWire, GL_LINES);
    } else if (!m_slamMapCloud.isEmpty()) {
        glPointSize(m_slamMapIsOccupancy ? 3.0f : 1.0f);
        drawArrays(m_slamMapCloud, GL_POINTS);
    }

    if (!m_slamRealtimeMapCloud.isEmpty()) {
        glPointSize(1.0f);
        drawArrays(m_slamRealtimeMapCloud, GL_POINTS);
    }

    if (!m_slamLiveScan.isEmpty()) {
        glPointSize(1.0f);
        drawArrays(m_slamLiveScan, GL_POINTS);
    }
    m_program->release();
}

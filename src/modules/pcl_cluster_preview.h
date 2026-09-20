#pragma once

#include <QColor>
#include <QHash>
#include <QVector>
#include <QVector3D>

#include "common/pointxyz.h"

struct PclClusterGroup
{
    int trackId = -1;
    bool bridgeCandidate = false;
    bool bridgePier = false;
    QColor color;
    QVector3D center;
    QVector3D minBounds;
    QVector3D maxBounds;
    QVector<QVector3D> points;
};

struct PclBridgePierInfo
{
    float nearestDistanceMeters = 0.0f;
    float centerDistanceMeters = 0.0f;
    float bearingDegrees = 0.0f; // 0°正前，正值向左，负值向右
    QVector3D center;
    int pointCount = 0;
};

struct PclBridgeHoleRoi
{
    QVector3D minBounds;
    QVector3D maxBounds;
};

// 聚类在 XY 平面的凸包边界。界面在俯视图绘制橙色轮廓，
// 在 3D/正视图按 minZ、maxZ 拉伸成橙色线框。
struct PclClusterBoundary
{
    int trackId = -1;
    float minZ = 0.0f;
    float maxZ = 0.0f;
    QVector<QVector3D> footprint;
};

struct PclObstacleInfo
{
    int trackId = -1;
    float rawClearanceMeters = 0.0f;
    float clearanceMeters = 0.0f;
    float sensorDistanceMeters = 0.0f;
    float bearingDegrees = 0.0f;
    // 正值表示正在接近，负值表示正在远离。
    float closingSpeedMetersPerSecond = 0.0f;
    QVector3D obstaclePoint;
    QVector3D shipPoint;
};

struct PclClusterPreviewResult
{
    double timestampSec = 0.0;
    bool classificationReady = false;
    bool bridgeDetected = false;
    bool bridgeUnderPassMode = false;
    float bridgeNearestDistanceMeters = 0.0f;
    float bridgeCenterDistanceMeters = 0.0f;
    float bridgeDeckHeightMeters = 0.0f;
    float bridgeVerticalSpanMeters = 0.0f;
    int bridgePointCount = 0;
    bool nearestObstacleDetected = false;
    int nearestObstacleTrackId = -1;
    float nearestObstacleClearanceMeters = 0.0f;
    float nearestObstacleSensorDistanceMeters = 0.0f;
    float nearestObstacleBearingDegrees = 0.0f;
    QVector3D nearestObstaclePoint;
    QVector3D nearestShipPoint;
    // 100 m 内全部有效障碍物，按船体净距离从近到远排列。
    QVector<PclObstacleInfo> obstacles;
    QVector<PclBridgePierInfo> bridgePiers;
    QVector<PclBridgeHoleRoi> bridgeHoleRois;
    // 后台在下采样点云上识别，再把最终语义映射回本帧原始点云。
    // 界面只绘制这两组高分辨率高亮点，不再覆盖绘制稀疏聚类点。
    QVector<QVector3D> highResolutionBridgePoints;
    QVector<QVector3D> highResolutionPierPoints;
    QVector<PclClusterBoundary> clusterBoundaries;
    QVector<PclClusterGroup> clusters;
    QVector<QVector3D> unclusteredPoints;
};

class PclClusterPreview
{
public:
    struct Config {
        float voxelLeaf = 0.18f;
        int meanK = 16;
        float stddevMultiplier = 1.2f;
        float clusterTolerance = 0.8f;
        int minClusterSize = 20;
        int maxClusterSize = 300000;
        int maxInputPoints = 500000;
        int normalTrackMaxMissedFrames = 8;
        int lockedPierMaxMissedFrames = 30;
        float underBridgeEntryGraceSeconds = 6.0f;
        float underBridgePierLostTimeoutSeconds = 4.0f;
        float underBridgeMaxDurationSeconds = 30.0f;
        float highResolutionMappingRadius = 0.40f;
        int maxBoundaryVertices = 160;
        float obstacleDetectionRangeMeters = 100.0f;
        // 当前工程坐标为 +X 船艏、+Y 左侧；与界面船体线框保持一致。
        float shipHalfExtentX = 6.0f;
        float shipHalfExtentY = 2.5f;
        int maxHighlightedObstaclesPerSide = 3;
    };

    PclClusterPreview();
    explicit PclClusterPreview(const Config &config);
    void reset();
    PclClusterPreviewResult process(const QVector<M_PointXYZI> &cloud,
                                    double timestampSec);

private:
    struct Track {
        int id = -1;
        QVector3D center;
        QVector3D size;
        int missed = 0;
        // 严格确认后锁定桥墩语义，直到该轨迹连续丢失并被删除。
        bool pierLocked = false;
        bool hasObstacleRange = false;
        float lastRawClearance = 0.0f;
        float smoothedClearance = 0.0f;
        float smoothedClosingSpeed = 0.0f;
        double lastObstacleUpdateSec = -1.0;
    };

    Config config_;
    QVector<Track> tracks_;
    // 按稳定 trackId 保存桥梁时序置信度，用于抑制闪烁和短时漏检。
    QHash<int, int> bridgeConfidence_;
    QHash<int, int> pierConfidence_;
    enum class BridgeTrackingState {
        Searching,
        Confirmed,
        UnderBridge
    };
    BridgeTrackingState bridgeTrackingState_ = BridgeTrackingState::Searching;
    bool hasCachedBridgeModel_ = false;
    float cachedDeckBottomZ_ = 0.0f;
    float cachedDeckTopZ_ = 0.0f;
    float cachedDeckHeight_ = 0.0f;
    double trackingTimeSec_ = 0.0;
    double lastInputTimestampSec_ = -1.0;
    double lastStableDeckTimeSec_ = -1.0e9;
    double lastLockedPierTimeSec_ = -1.0e9;
    double underBridgeStartTimeSec_ = -1.0e9;
    bool hasSmoothedBridgeDistance_ = false;
    float smoothedNearestDistance_ = 0.0f;
    float smoothedCenterDistance_ = 0.0f;
    int nextTrackId_ = 0;
};


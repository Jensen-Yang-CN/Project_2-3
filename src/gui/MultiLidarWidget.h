#ifndef MULTILIDARWIDGET_H
#define MULTILIDARWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QHash>
#include <QVector>
#include <QTimer>
#include "common/pointxyz.h" // 确保包含你的结构体定义
#include <QElapsedTimer>
#include "common/slam_types.h"
#include "common_types_extended.h"
#include "NaviProtocol.h"
#include "SlamBerthOverlaySynchronizer.h"
#include "SlamMapBerthStore.h"
#include "SlamMapBinaryIO.h"
#include "SlamMapStitcher.h"
#include "SlamTileMapTypes.h"
#include "UsvDirectionUtils.h"
#ifdef ENABLE_PCL_PERCEPTION
#include "pcl_cluster_preview.h"
#endif

#include <Eigen/Geometry>

#include <memory>
#include <limits>
#include <vector>

namespace slam_map_display {
struct BuildStats;
}

class MultiLidarWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit MultiLidarWidget(QWidget *parent = nullptr);
    ~MultiLidarWidget();

public slots:
    // 唯一的点云更新入口：接收来自 RadarFusionManager 的融合数据
    void updateFusedCloud(const QVector<M_PointXYZI> &cloud, double timestampSec);

    // 接收来自 ImuWorker 的解析数据
    void onNewTrajectoryPoint(const IMUParsedData &data);

    // 泊位检测结果（210 系 x-y 平面矩形）
    void updateBerthResult(const usv::BerthMeasureResult &res);
    void clearBerthResult();
    // SLAM 地图
    void setSlamMapMode(bool enable);
    void clearSlamMap();
    void appendSlamKeyframe(const usv::SlamKeyframe &keyframe);
    void updateSlamOdometry(const usv::SlamOdometryState &state);
    void updateSlamLiveScan(const QVector<M_PointXYZI> &worldCloud);
    void updateSlamOccupancyGrid(QVector<M_PointXYZI> centers, float resolution_m);
    bool hasSlamMapData() const;
    bool saveCurrentSlamMap(const QString &filePath, QString *errorMsg = nullptr) const;
    bool loadSlamMap(const QString &filePath, QString *errorMsg = nullptr);
    void setSlamGeoAnchor(const usv::SlamGeoAnchor &anchor);
    std::vector<usv::SlamMapBerth> slamMapBerths() const;
    void setSlamMapBerths(const std::vector<usv::SlamMapBerth> &berths);

signals:
    /** USV 当前位置/姿态数值（GNSS/INS，供右侧面板显示） */
    void usvStateUpdated(const QString &text);
    void slamTileViewChanged(slam_tile::Bounds2d bounds, int lod);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    // 交互事件
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    using TrajectoryPoint = usv_direction::Point2f;

private:
    void drawRadarGrid(float centerX = 0.0f, float centerY = 0.0f);
    void drawBerthOverlays(const QMatrix4x4 &viewMatrix,
                           const usv::BerthMeasureResult &result,
                           bool useEdgeHeights);
#ifdef ENABLE_PCL_PERCEPTION
    void drawPclClusterPreview(const QMatrix4x4 &viewMatrix);
    void drawOverlayArrays(GLenum mode,
                           const QVector<M_PointXYZI> &vertices,
                           bool solidColor,
                           const QVector3D &color = QVector3D());
#endif
    void drawUsvOverlays(const QMatrix4x4 &viewMatrix);
    void drawUsvPoseOverlay(const QMatrix4x4 &viewMatrix,
                            float shipX, float shipY, float yawDeg,
                            const QList<TrajectoryPoint> &path, bool hasPose);
    static void appendUsvPathPoint(QList<TrajectoryPoint> &path, float x, float y);
    static void prunePathByDistanceFrom(QList<TrajectoryPoint> &path, float cx, float cy, float maxDist);
    QList<TrajectoryPoint> buildRadarFramePath(const QList<TrajectoryPoint> &worldPath,
                                               float shipWorldX, float shipWorldY, float yawDeg,
                                               float maxDistM) const;
    float radarTrailMaxDistanceM() const;
    static float yawDegFromPose(const Eigen::Isometry3d &pose);
    bool berthAlignedForPaint() const;
    void persistSynchronizedBerthResult();
    usv::BerthMeasureResult savedBerthResultForPaint() const;
    void mergeKeyframeIntoMap(const usv::SlamKeyframe &keyframe);
    void rebuildOccupancyWireframe();
    void downsampleCloudIfNeeded(QVector<M_PointXYZI> &cloud);
    void renderSlamMap(const QMatrix4x4 &viewMatrix);
    void renderFusedCloud(const QMatrix4x4 &viewMatrix);
    void emitUsvStateText();
    void recenterSlamViewOnShip();
    void emitSlamTileViewChange();
    slam_tile::Bounds2d currentSlamTileViewBounds() const;
    void destroySlamTileVbos();

    struct SlamTileGpu {
        explicit SlamTileGpu(slam_tile::TileDataPtr source)
            : data(std::move(source))
            , vbo(QOpenGLBuffer::VertexBuffer)
        {
        }

        slam_tile::TileDataPtr data;
        QOpenGLBuffer vbo;
        bool uploaded = false;
        int uploaded_points = 0;
    };

private:
    QOpenGLShaderProgram *m_program = nullptr;
    QOpenGLBuffer m_vbo;
    QOpenGLBuffer m_slamVbo;
#ifdef ENABLE_PCL_PERCEPTION
    QOpenGLBuffer m_overlayVbo;
    static constexpr int kMaxOverlayVerts = 65536;
    PclClusterPreviewResult m_pclClusterPreview;
#endif
    QVector<M_PointXYZI> m_fusedData;
    QTimer m_fusedRepaintTimer_;
    static constexpr int kFusedRepaintIntervalMs = 33;  // ~30 FPS
    static constexpr qint64 kSlamUpdateIntervalMs = 33;
    qint64 m_lastSlamUpdateMs = 0;
    // 视图变换参数
    float m_zOffset = -50.0f;
    float m_rotationX = 30.0f;
    float m_rotationY = 45.0f;
    QPoint m_lastMousePos;
    bool m_isBevMode = false;  // 俯视图模式开关
    float m_bevRange = 60.0f;  // 俯视图可见范围（米）
    float m_viewPanX = 0.0f;   // 俯视图/SLAM 地图平移中心（世界坐标，米）
    float m_viewPanY = 0.0f;

    usv::BerthMeasureResult m_berthResult;
    bool m_berthVisible = false;
    SlamBerthOverlaySynchronizer m_slamBerthOverlay;
    SlamMapBerthStore m_slamMapBerthStore;
    double m_lastPersistedBerthTimestamp = -std::numeric_limits<double>::infinity();
    double m_cloudTimestamp = -1.0;
    bool m_hasCloudTimestamp = false;

    // 点云领先检测结果的容忍窗口（检测慢时仍显示框，与桥洞 overlay 一致）
    static constexpr double kBerthToleranceSec = 0.15;
    static constexpr double kBerthMaxLeadSec = 5.0;
    // 超过此滞后强制隐藏，避免框长期钉在已过时的位置上
    static constexpr double kBerthMaxStaleSec = 8.0;

    bool m_showSlamMap = false;
    QVector<M_PointXYZI> m_slamMapCloud;
    // 历史地图与实时关键帧分层绘制，避免加载历史地图后看起来像重复地图。
    QVector<M_PointXYZI> m_slamRealtimeMapCloud;
    QVector<M_PointXYZI> m_slamLiveScan;
    std::vector<usv::SlamKeyframe> m_slamKeyframes;
    QVector<M_PointXYZI> m_slamOccupancyWire;
    bool m_slamMapIsOccupancy = false;
    bool m_hasHistoricalSlamMap = false;
    float m_occupancyResolution = 0.5f;
    static constexpr int kMaxOccupancyWireCubes = 100000;
    QHash<slam_tile::TileId, std::shared_ptr<SlamTileGpu>> m_slamTiles;
    slam_tile::Manifest m_slamTileManifest;
    bool m_hasSlamTileManifest = false;
    int m_currentSlamTileLod = 0;
    int m_slamTileGpuPointBudget = 1500000;
    usv::SlamGeoAnchor m_slamGeoAnchor;
    Eigen::Isometry3d m_slamPose = Eigen::Isometry3d::Identity();
    bool m_hasSlamPose = false;
    static constexpr int kMaxSlamMapPoints = 1500000;
    static constexpr unsigned char kSlamMapIntensity = 90;
    static constexpr unsigned char kSlamLiveScanIntensity = 90;
    static constexpr unsigned char kSlamHistoricalMapIntensity = 220;

    // 最大缓存点数 (5路雷达建议设大一点)
    const int MAX_POINTS = 500000;

    QList<TrajectoryPoint> m_shipPath;
    QList<TrajectoryPoint> m_slamPath;
    float m_slamX = 0.0f;
    float m_slamY = 0.0f;
    float m_slamYaw = 0.0f;
    double m_refLon = 0.0;
    double m_refLat = 0.0;
    float m_currentYaw = 0.0f;
    float m_navX = 0.0f;
    float m_navY = 0.0f;
    bool m_hasNavPose = false;
    bool m_originSet = false;
    IMUParsedData m_lastImu{};
    bool m_hasLastImu = false;
    QElapsedTimer m_usvTextTimer;
    qint64 m_lastUsvTextMs = 0;
    static constexpr int kUsvTextIntervalMs = 300;
    /** 融合点云模式：雷达系轨迹最大保留距离（相对当前船位） */
    static constexpr float kRadarTrailMinDistM = 25.0f;
    static constexpr float kRadarTrailRangeFactor = 0.9f;
    static constexpr float kCourseDirectionMinDistanceM = 1.0f;

private slots:
    void onFusedRepaintTick();

public slots:
    // 供外部按钮调用的槽函数
    void setBevMode(bool enable);
};

#endif // MULTILIDARWIDGET_H

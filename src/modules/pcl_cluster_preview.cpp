#include "pcl_cluster_preview.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace {

QColor clusterColor(int index)
{
    // 黄金角步进，使相邻聚类颜色差异明显且每次运行保持稳定。
    const int hue = (index * 137) % 360;
    return QColor::fromHsv(hue, 230, 255);
}

double cross2d(const QVector3D &origin,
               const QVector3D &a,
               const QVector3D &b)
{
    return static_cast<double>(a.x() - origin.x()) * (b.y() - origin.y())
         - static_cast<double>(a.y() - origin.y()) * (b.x() - origin.x());
}

QVector<QVector3D> convexHullXY(const QVector<QVector3D> &clusterPoints,
                               int maxVertices)
{
    std::vector<QVector3D> points;
    points.reserve(static_cast<std::size_t>(clusterPoints.size()));
    for (const QVector3D &point : clusterPoints)
        points.emplace_back(point.x(), point.y(), 0.0f);

    std::sort(points.begin(), points.end(), [](const QVector3D &a, const QVector3D &b) {
        if (a.x() != b.x())
            return a.x() < b.x();
        return a.y() < b.y();
    });
    points.erase(std::unique(points.begin(), points.end(), [](const QVector3D &a,
                                                               const QVector3D &b) {
        return std::abs(a.x() - b.x()) < 1.0e-4f
            && std::abs(a.y() - b.y()) < 1.0e-4f;
    }), points.end());

    if (points.size() <= 2) {
        QVector<QVector3D> result;
        result.reserve(static_cast<int>(points.size()));
        for (const QVector3D &point : points)
            result.append(point);
        return result;
    }

    std::vector<QVector3D> hull;
    hull.reserve(points.size() * 2);
    for (const QVector3D &point : points) {
        while (hull.size() >= 2
               && cross2d(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const std::size_t lowerSize = hull.size();
    for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
        while (hull.size() > lowerSize
               && cross2d(hull[hull.size() - 2], hull.back(), *it) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    if (hull.size() > 1)
        hull.pop_back();

    QVector<QVector3D> result;
    const int requestedLimit = (std::max)(3, maxVertices);
    const int outputCount = (std::min)(static_cast<int>(hull.size()), requestedLimit);
    result.reserve(outputCount);
    if (static_cast<int>(hull.size()) <= requestedLimit) {
        for (const QVector3D &point : hull)
            result.append(point);
    } else {
        for (int i = 0; i < outputCount; ++i) {
            const std::size_t index = static_cast<std::size_t>(
                static_cast<double>(i) * hull.size() / outputCount);
            result.append(hull[index]);
        }
    }
    return result;
}

struct BoundaryDistanceResult
{
    double distanceSquared = std::numeric_limits<double>::infinity();
    QVector3D obstaclePoint;
    QVector3D shipPoint;
};

void updateBoundaryDistance(BoundaryDistanceResult &best,
                            const QVector3D &obstaclePoint,
                            const QVector3D &shipPoint)
{
    const double dx = static_cast<double>(obstaclePoint.x() - shipPoint.x());
    const double dy = static_cast<double>(obstaclePoint.y() - shipPoint.y());
    const double distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < best.distanceSquared) {
        best.distanceSquared = distanceSquared;
        best.obstaclePoint = obstaclePoint;
        best.shipPoint = shipPoint;
    }
}

QVector3D closestPointOnSegmentXY(const QVector3D &point,
                                  const QVector3D &a,
                                  const QVector3D &b)
{
    const double dx = static_cast<double>(b.x() - a.x());
    const double dy = static_cast<double>(b.y() - a.y());
    const double lengthSquared = dx * dx + dy * dy;
    double t = 0.0;
    if (lengthSquared > 1.0e-12) {
        t = ((point.x() - a.x()) * dx + (point.y() - a.y()) * dy)
          / lengthSquared;
        t = (std::max)(0.0, (std::min)(1.0, t));
    }
    return QVector3D(static_cast<float>(a.x() + t * dx),
                     static_cast<float>(a.y() + t * dy), 0.0f);
}

bool segmentIntersectsRectangle(const QVector3D &a,
                                const QVector3D &b,
                                float halfX,
                                float halfY,
                                QVector3D &intersection)
{
    const double dx = static_cast<double>(b.x() - a.x());
    const double dy = static_cast<double>(b.y() - a.y());
    double enter = 0.0;
    double leave = 1.0;
    auto clip = [&enter, &leave](double p, double q) {
        if (std::abs(p) < 1.0e-12)
            return q >= 0.0;
        const double ratio = q / p;
        if (p < 0.0)
            enter = (std::max)(enter, ratio);
        else
            leave = (std::min)(leave, ratio);
        return enter <= leave;
    };
    if (!clip(-dx, a.x() + halfX)
        || !clip(dx, halfX - a.x())
        || !clip(-dy, a.y() + halfY)
        || !clip(dy, halfY - a.y())) {
        return false;
    }
    intersection = QVector3D(static_cast<float>(a.x() + enter * dx),
                             static_cast<float>(a.y() + enter * dy), 0.0f);
    return true;
}

bool pointInsidePolygonXY(const QVector3D &point,
                          const QVector<QVector3D> &polygon)
{
    if (polygon.size() < 3)
        return false;
    bool inside = false;
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const QVector3D &a = polygon[i];
        const QVector3D &b = polygon[j];
        const bool crosses = ((a.y() > point.y()) != (b.y() > point.y()))
            && (point.x() < (b.x() - a.x()) * (point.y() - a.y())
                               / (b.y() - a.y() + 1.0e-12f) + a.x());
        if (crosses)
            inside = !inside;
    }
    return inside;
}

BoundaryDistanceResult distanceFromBoundaryToShip(
    const QVector<QVector3D> &boundary,
    float shipHalfX,
    float shipHalfY)
{
    BoundaryDistanceResult best;
    if (boundary.size() < 2)
        return best;

    const QVector<QVector3D> shipCorners{
        QVector3D(-shipHalfX, -shipHalfY, 0.0f),
        QVector3D(shipHalfX, -shipHalfY, 0.0f),
        QVector3D(shipHalfX, shipHalfY, 0.0f),
        QVector3D(-shipHalfX, shipHalfY, 0.0f)
    };
    if (pointInsidePolygonXY(QVector3D(0.0f, 0.0f, 0.0f), boundary)) {
        best.distanceSquared = 0.0;
        best.obstaclePoint = QVector3D(0.0f, 0.0f, 0.0f);
        best.shipPoint = best.obstaclePoint;
        return best;
    }

    const int edgeCount = boundary.size() >= 3 ? boundary.size() : 1;
    for (int i = 0; i < edgeCount; ++i) {
        const QVector3D &a = boundary[i];
        const QVector3D &b = boundary[(i + 1) % boundary.size()];
        QVector3D intersection;
        if (segmentIntersectsRectangle(a, b, shipHalfX, shipHalfY, intersection)) {
            best.distanceSquared = 0.0;
            best.obstaclePoint = intersection;
            best.shipPoint = intersection;
            return best;
        }

        for (const QVector3D &endpoint : {a, b}) {
            const QVector3D shipPoint(
                (std::max)(-shipHalfX, (std::min)(shipHalfX, endpoint.x())),
                (std::max)(-shipHalfY, (std::min)(shipHalfY, endpoint.y())),
                0.0f);
            updateBoundaryDistance(best, endpoint, shipPoint);
        }
        for (const QVector3D &corner : shipCorners) {
            updateBoundaryDistance(best, closestPointOnSegmentXY(corner, a, b), corner);
        }
    }
    return best;
}

} // namespace

PclClusterPreview::PclClusterPreview() = default;

PclClusterPreview::PclClusterPreview(const Config &config)
    : config_(config)
{
}

void PclClusterPreview::reset()
{
    // 保留现场配置，只清除轨迹、时序置信度和桥下状态。
    *this = PclClusterPreview(config_);
}

PclClusterPreviewResult PclClusterPreview::process(
    const QVector<M_PointXYZI> &cloud,
    double timestampSec)
{
    PclClusterPreviewResult result;
    result.timestampSec = timestampSec;
    // 使用输入时间戳推进内部状态；时间戳异常或重置时按 10 Hz 保守推进，
    // 避免桥下模式因设备时间跳变而永久保持。
    if (std::isfinite(timestampSec) && lastInputTimestampSec_ >= 0.0) {
        const double delta = timestampSec - lastInputTimestampSec_;
        trackingTimeSec_ += (delta > 0.0 && delta <= 1.0) ? delta : 0.1;
    } else {
        trackingTimeSec_ += 0.1;
    }
    if (std::isfinite(timestampSec))
        lastInputTimestampSec_ = timestampSec;
    if (cloud.isEmpty())
        return result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    const int step = cloud.size() > config_.maxInputPoints
        ? (cloud.size() + config_.maxInputPoints - 1) / config_.maxInputPoints
        : 1;
    input->reserve(static_cast<std::size_t>(cloud.size() / step + 1));
    for (int i = 0; i < cloud.size(); i += step) {
        const M_PointXYZI &p = cloud[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        // 只屏蔽当前船体矩形内部点，不再删除原点 5 m 圆形区域，
        // 以免漏掉紧贴左右船舷但仍位于船体外部的障碍物。
        const bool insideShip = std::abs(p.x) <= config_.shipHalfExtentX
                             && std::abs(p.y) <= config_.shipHalfExtentY;
        if (insideShip
            || p.x < -250.0f || p.x > 250.0f
            || p.y < -250.0f || p.y > 250.0f
            || p.z < -15.0f || p.z > 50.0f) {
            continue;
        }
        input->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    if (input->empty())
        return result;

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(config_.voxelLeaf, config_.voxelLeaf, config_.voxelLeaf);
    voxel.filter(*voxelized);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (voxelized->size() >= static_cast<std::size_t>(config_.meanK + 1)) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(voxelized);
        sor.setMeanK(config_.meanK);
        sor.setStddevMulThresh(config_.stddevMultiplier);
        sor.filter(*filtered);
    } else {
        *filtered = *voxelized;
    }
    if (filtered->empty())
        return result;

    // 使用 PCL KD-Tree 做欧式连通聚类，避免依赖当前构建树中不稳定的
    // pcl_segmentation DLL；邻域定义与 EuclideanClusterExtraction 相同。
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(filtered);
    std::vector<pcl::PointIndices> indices;
    std::vector<unsigned char> visited(filtered->size(), 0);
    std::vector<int> neighborIndices;
    std::vector<float> neighborDistances;
    for (int seed = 0; seed < static_cast<int>(filtered->size()); ++seed) {
        if (visited[static_cast<std::size_t>(seed)])
            continue;

        std::deque<int> pending;
        std::vector<int> component;
        pending.push_back(seed);
        visited[static_cast<std::size_t>(seed)] = 1;

        while (!pending.empty()
               && component.size() <= static_cast<std::size_t>(config_.maxClusterSize)) {
            const int current = pending.front();
            pending.pop_front();
            component.push_back(current);

            neighborIndices.clear();
            neighborDistances.clear();
            if (tree.radiusSearch((*filtered)[static_cast<std::size_t>(current)],
                                  config_.clusterTolerance,
                                  neighborIndices,
                                  neighborDistances) <= 0) {
                continue;
            }
            for (int neighbor : neighborIndices) {
                if (neighbor < 0 || neighbor >= static_cast<int>(filtered->size()))
                    continue;
                if (!visited[static_cast<std::size_t>(neighbor)]) {
                    visited[static_cast<std::size_t>(neighbor)] = 1;
                    pending.push_back(neighbor);
                }
            }
        }

        if (component.size() >= static_cast<std::size_t>(config_.minClusterSize)
            && component.size() <= static_cast<std::size_t>(config_.maxClusterSize)) {
            pcl::PointIndices cluster;
            cluster.indices = std::move(component);
            indices.push_back(std::move(cluster));
        }
    }

    std::sort(indices.begin(), indices.end(), [](const pcl::PointIndices &a,
                                                  const pcl::PointIndices &b) {
        return a.indices.size() > b.indices.size();
    });

    result.clusters.reserve(static_cast<int>(indices.size()));
    for (int clusterIndex = 0; clusterIndex < static_cast<int>(indices.size()); ++clusterIndex) {
        PclClusterGroup group;
        group.minBounds = QVector3D(1.0e9f, 1.0e9f, 1.0e9f);
        group.maxBounds = QVector3D(-1.0e9f, -1.0e9f, -1.0e9f);
        QVector3D centerSum;
        group.points.reserve(static_cast<int>(indices[clusterIndex].indices.size()));
        for (int pointIndex : indices[clusterIndex].indices) {
            if (pointIndex < 0 || pointIndex >= static_cast<int>(filtered->size()))
                continue;
            const pcl::PointXYZ &p = (*filtered)[static_cast<std::size_t>(pointIndex)];
            const QVector3D point(p.x, p.y, p.z);
            group.points.append(point);
            centerSum += point;
            group.minBounds.setX((std::min)(group.minBounds.x(), p.x));
            group.minBounds.setY((std::min)(group.minBounds.y(), p.y));
            group.minBounds.setZ((std::min)(group.minBounds.z(), p.z));
            group.maxBounds.setX((std::max)(group.maxBounds.x(), p.x));
            group.maxBounds.setY((std::max)(group.maxBounds.y(), p.y));
            group.maxBounds.setZ((std::max)(group.maxBounds.z(), p.z));
        }
        if (!group.points.isEmpty()) {
            group.center = centerSum / static_cast<float>(group.points.size());
            result.clusters.append(std::move(group));
        }
    }

    // 当前聚类与历史轨迹做贪心一对一匹配。大桥等大型目标允许更大的
    // 中心漂移，小目标保持严格门限，避免颜色在相邻物体间跳转。
    for (Track &track : tracks_)
        ++track.missed;
    QVector<unsigned char> trackUsed(tracks_.size(), 0);
    for (PclClusterGroup &group : result.clusters) {
        const QVector3D groupSize = group.maxBounds - group.minBounds;
        int bestTrack = -1;
        float bestCost = 1.0e9f;
        for (int ti = 0; ti < tracks_.size(); ++ti) {
            if (trackUsed[ti])
                continue;
            const Track &track = tracks_[ti];
            const float centerDistance = (group.center - track.center).length();
            const float scale = (std::max)(groupSize.length(), track.size.length());
            const float gate = (std::min)(15.0f, (std::max)(3.0f, 0.28f * scale));
            if (centerDistance > gate)
                continue;
            const float sizeDifference = (groupSize - track.size).length()
                / (1.0f + (std::max)(groupSize.length(), track.size.length()));
            const float cost = centerDistance + 2.0f * sizeDifference;
            if (cost < bestCost) {
                bestCost = cost;
                bestTrack = ti;
            }
        }

        if (bestTrack >= 0) {
            Track &track = tracks_[bestTrack];
            track.center = 0.65f * track.center + 0.35f * group.center;
            track.size = 0.65f * track.size + 0.35f * groupSize;
            track.missed = 0;
            trackUsed[bestTrack] = 1;
            group.trackId = track.id;
        } else {
            Track track;
            track.id = nextTrackId_++;
            track.center = group.center;
            track.size = groupSize;
            track.missed = 0;
            tracks_.append(track);
            trackUsed.append(1);
            group.trackId = track.id;
        }
        group.color = clusterColor(group.trackId);
    }

    for (int i = tracks_.size() - 1; i >= 0; --i) {
        const int maxMissedFrames = tracks_[i].pierLocked
            ? config_.lockedPierMaxMissedFrames
            : config_.normalTrackMaxMissedFrames;
        if (tracks_[i].missed > maxMissedFrames)
            tracks_.removeAt(i);
    }

    // 将被欧式距离切碎的桥面小簇按同一条俯视中心线重新聚类。
    // 每个成员都直接与种子中心线比较，避免普通物体通过邻接关系链式污染桥梁。
    const int clusterCount = result.clusters.size();
    if (clusterCount > 0) {
        struct ClusterShape {
            float directionX = 1.0f;
            float directionY = 0.0f;
            float length = 0.0f;
            float width = 0.0f;
            float linearity = 0.0f;
            float centerZ = 0.0f;
            float horizontalNormalScore = 0.0f;
            float verticalAxisScore = 0.0f;
            float planarity = 0.0f;
        };
        std::vector<ClusterShape> shapes(static_cast<std::size_t>(clusterCount));

        for (int i = 0; i < clusterCount; ++i) {
            const PclClusterGroup &cluster = result.clusters[i];
            ClusterShape &shape = shapes[static_cast<std::size_t>(i)];
            shape.centerZ = cluster.center.z();
            if (cluster.points.size() < 3)
                continue;

            double xx = 0.0;
            double xy = 0.0;
            double xz = 0.0;
            double yy = 0.0;
            double yz = 0.0;
            double zz = 0.0;
            for (const QVector3D &point : cluster.points) {
                const double x = point.x() - cluster.center.x();
                const double y = point.y() - cluster.center.y();
                const double z = point.z() - cluster.center.z();
                xx += x * x;
                xy += x * y;
                xz += x * z;
                yy += y * y;
                yz += y * z;
                zz += z * z;
            }
            xx /= cluster.points.size();
            xy /= cluster.points.size();
            xz /= cluster.points.size();
            yy /= cluster.points.size();
            yz /= cluster.points.size();
            zz /= cluster.points.size();

            Eigen::Matrix3d covariance3d;
            covariance3d << xx, xy, xz,
                            xy, yy, yz,
                            xz, yz, zz;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance3d);
            if (eigenSolver.info() == Eigen::Success) {
                const Eigen::Vector3d eigenvalues = eigenSolver.eigenvalues();
                const Eigen::Matrix3d eigenvectors = eigenSolver.eigenvectors();
                shape.horizontalNormalScore = static_cast<float>(std::abs(eigenvectors(2, 0)));
                shape.verticalAxisScore = static_cast<float>(std::abs(eigenvectors(2, 2)));
                shape.planarity = static_cast<float>((eigenvalues(1) - eigenvalues(0))
                    / (eigenvalues(2) + 1.0e-9));
            }
            const double trace = xx + yy;
            const double delta = std::sqrt((xx - yy) * (xx - yy) + 4.0 * xy * xy);
            const double major = 0.5 * (trace + delta);
            const double minor = 0.5 * (trace - delta);
            if (major <= 1.0e-6)
                continue;
            shape.linearity = static_cast<float>(1.0 - minor / major);
            double directionX = xy;
            double directionY = major - xx;
            double directionLength = std::hypot(directionX, directionY);
            if (directionLength < 1.0e-6) {
                directionX = 1.0;
                directionY = 0.0;
                directionLength = 1.0;
            }
            shape.directionX = static_cast<float>(directionX / directionLength);
            shape.directionY = static_cast<float>(directionY / directionLength);

            float minAlong = 1.0e9f;
            float maxAlong = -1.0e9f;
            float minAcross = 1.0e9f;
            float maxAcross = -1.0e9f;
            for (const QVector3D &point : cluster.points) {
                const float x = point.x() - cluster.center.x();
                const float y = point.y() - cluster.center.y();
                const float along = x * shape.directionX + y * shape.directionY;
                const float across = -x * shape.directionY + y * shape.directionX;
                minAlong = (std::min)(minAlong, along);
                maxAlong = (std::max)(maxAlong, along);
                minAcross = (std::min)(minAcross, across);
                maxAcross = (std::max)(maxAcross, across);
            }
            shape.length = maxAlong - minAlong;
            shape.width = maxAcross - minAcross;
        }

        QVector<int> bestBridgeMembers;
        float bestBridgeScore = 0.0f;
        QVector3D bestBridgeOrigin;
        float bestDirectionX = 1.0f;
        float bestDirectionY = 0.0f;
        float bestMinLongitudinal = 0.0f;
        float bestMaxLongitudinal = 0.0f;
        float bestHalfWidth = 0.0f;
        float bestDeckBottomZ = 0.0f;
        float bestDeckTopZ = 0.0f;
        for (int seed = 0; seed < clusterCount; ++seed) {
            const PclClusterGroup &seedCluster = result.clusters[seed];
            const ClusterShape &seedShape = shapes[static_cast<std::size_t>(seed)];
            if (seedShape.length < 5.0f || seedShape.linearity < 0.65f)
                continue;

            // 正视横跨约束：桥面方向应与“雷达到候选中心”的水平视线近似垂直。
            // 巷道/道路通常沿视线向远处延伸，其 transverseScore 会接近 0。
            const float range = std::hypot(seedCluster.center.x(), seedCluster.center.y());
            if (range < 5.0f)
                continue;
            const float viewX = seedCluster.center.x() / range;
            const float viewY = seedCluster.center.y() / range;
            const float transverseScore = std::abs(seedShape.directionX * viewY
                                                  - seedShape.directionY * viewX);
            if (transverseScore < 0.72f)
                continue;

            QVector<int> members;
            QVector<QVector3D> candidatePoints;
            float minZ = 1.0e9f;
            float maxZ = -1.0e9f;
            for (int candidate = 0; candidate < clusterCount; ++candidate) {
                const PclClusterGroup &cluster = result.clusters[candidate];
                const ClusterShape &shape = shapes[static_cast<std::size_t>(candidate)];
                const float dx = cluster.center.x() - seedCluster.center.x();
                const float dy = cluster.center.y() - seedCluster.center.y();
                const float lateralDistance = std::abs(-dx * seedShape.directionY
                                                       + dy * seedShape.directionX);
                const float directionDot = std::abs(shape.directionX * seedShape.directionX
                                                    + shape.directionY * seedShape.directionY);
                const bool directionCompatible = shape.length < 4.0f || directionDot >= 0.78f;
                if (lateralDistance > 15.0f
                    || std::abs(shape.centerZ - seedShape.centerZ) > 4.5f
                    || !directionCompatible) {
                    continue;
                }
                members.append(candidate);
                candidatePoints += cluster.points;
                minZ = (std::min)(minZ, cluster.minBounds.z());
                maxZ = (std::max)(maxZ, cluster.maxBounds.z());
            }
            if (candidatePoints.size() < 100)
                continue;

            std::vector<float> longitudinal;
            longitudinal.reserve(static_cast<std::size_t>(candidatePoints.size()));
            float minLongitudinal = 1.0e9f;
            float maxLongitudinal = -1.0e9f;
            float minLateral = 1.0e9f;
            float maxLateral = -1.0e9f;
            for (const QVector3D &point : candidatePoints) {
                const float x = point.x() - seedCluster.center.x();
                const float y = point.y() - seedCluster.center.y();
                const float along = x * seedShape.directionX + y * seedShape.directionY;
                const float across = -x * seedShape.directionY + y * seedShape.directionX;
                longitudinal.push_back(along);
                minLongitudinal = (std::min)(minLongitudinal, along);
                maxLongitudinal = (std::max)(maxLongitudinal, along);
                minLateral = (std::min)(minLateral, across);
                maxLateral = (std::max)(maxLateral, across);
            }
            std::sort(longitudinal.begin(), longitudinal.end());
            float largestGap = 0.0f;
            for (std::size_t i = 1; i < longitudinal.size(); ++i)
                largestGap = (std::max)(largestGap, longitudinal[i] - longitudinal[i - 1]);

            const float length = maxLongitudinal - minLongitudinal;
            const float width = maxLateral - minLateral;
            const float verticalThickness = (std::max)(0.0f, maxZ - minZ);
            const float aspectRatio = length / (0.5f + width);

            if (length < 35.0f
                || aspectRatio < 2.8f
                || width > 28.0f
                || largestGap > 18.0f
                || verticalThickness > 16.0f) {
                continue;
            }

            // 对合并后的桥面点整体估计法向，避免单个稀疏扫描线法向不稳定。
            Eigen::Vector3d deckMean = Eigen::Vector3d::Zero();
            for (const QVector3D &point : candidatePoints)
                deckMean += Eigen::Vector3d(point.x(), point.y(), point.z());
            deckMean /= candidatePoints.size();
            Eigen::Matrix3d deckCovariance = Eigen::Matrix3d::Zero();
            std::vector<float> deckHeights;
            deckHeights.reserve(static_cast<std::size_t>(candidatePoints.size()));
            for (const QVector3D &point : candidatePoints) {
                const Eigen::Vector3d offset(point.x() - deckMean.x(),
                                             point.y() - deckMean.y(),
                                             point.z() - deckMean.z());
                deckCovariance += offset * offset.transpose();
                deckHeights.push_back(point.z());
            }
            deckCovariance /= candidatePoints.size();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> deckSolver(deckCovariance);
            if (deckSolver.info() != Eigen::Success)
                continue;
            const float deckNormalScore = static_cast<float>(
                std::abs(deckSolver.eigenvectors()(2, 0)));
            if (deckNormalScore < 0.35f)
                continue;
            std::sort(deckHeights.begin(), deckHeights.end());
            const float candidateDeckBottomZ = deckHeights[deckHeights.size() / 10];
            const float candidateDeckTopZ = deckHeights[(deckHeights.size() * 9) / 10];

            // π形先验：在当前横梁下寻找直接连接、主轴竖直的支撑簇。
            float supportQuality = 0.0f;
            QVector<float> supportAlongPositions;
            for (int support = 0; support < clusterCount; ++support) {
                if (members.contains(support))
                    continue;
                const PclClusterGroup &cluster = result.clusters[support];
                const ClusterShape &shape = shapes[static_cast<std::size_t>(support)];
                const QVector3D size = cluster.maxBounds - cluster.minBounds;
                const float dx = cluster.center.x() - seedCluster.center.x();
                const float dy = cluster.center.y() - seedCluster.center.y();
                const float along = dx * seedShape.directionX + dy * seedShape.directionY;
                const float across = -dx * seedShape.directionY + dy * seedShape.directionX;
                if (along < minLongitudinal - 3.0f || along > maxLongitudinal + 3.0f
                    || std::abs(across) > 0.5f * width + 5.0f
                    || shape.verticalAxisScore < 0.50f
                    || size.z() < 1.0f
                    || cluster.center.z() >= candidateDeckBottomZ) {
                    continue;
                }

                bool attached = false;
                for (int deckMember : members) {
                    const PclClusterGroup &deck = result.clusters[deckMember];
                    auto gap = [](float a0, float a1, float b0, float b1) {
                        if (a1 < b0) return b0 - a1;
                        if (b1 < a0) return a0 - b1;
                        return 0.0f;
                    };
                    const float horizontalGap = std::hypot(
                        gap(cluster.minBounds.x(), cluster.maxBounds.x(),
                            deck.minBounds.x(), deck.maxBounds.x()),
                        gap(cluster.minBounds.y(), cluster.maxBounds.y(),
                            deck.minBounds.y(), deck.maxBounds.y()));
                    const float verticalGap = std::abs(cluster.maxBounds.z() - deck.minBounds.z());
                    if (horizontalGap <= 2.0f && verticalGap <= 2.2f) {
                        attached = true;
                        break;
                    }
                }
                if (!attached || cluster.maxBounds.z() > candidateDeckTopZ + 0.5f)
                    continue;
                supportQuality += shape.verticalAxisScore;
                supportAlongPositions.append(along);
            }
            if (supportQuality < 0.50f)
                continue;
            float piModelScore = (std::min)(1.0f, supportQuality / 1.5f);
            if (supportAlongPositions.size() >= 2) {
                const auto minmax = std::minmax_element(supportAlongPositions.begin(),
                                                        supportAlongPositions.end());
                if (*minmax.second - *minmax.first >= 8.0f)
                    piModelScore = (std::min)(1.0f, piModelScore + 0.25f);
            }

            // 长度仍占主导，但桥面法向与 π 形支撑质量共同决定最终候选。
            const float score = length * length * transverseScore * transverseScore
                              * (0.55f + 0.45f * deckNormalScore)
                              * (0.40f + 0.60f * piModelScore)
                              * std::log1p(static_cast<float>(candidatePoints.size()))
                              / (1.0f + 0.20f * width + 0.08f * verticalThickness);
            if (score > bestBridgeScore) {
                bestBridgeScore = score;
                bestBridgeMembers = members;
                bestBridgeOrigin = seedCluster.center;
                bestDirectionX = seedShape.directionX;
                bestDirectionY = seedShape.directionY;
                bestMinLongitudinal = minLongitudinal;
                bestMaxLongitudinal = maxLongitudinal;
                bestHalfWidth = 0.5f * width;
                bestDeckBottomZ = candidateDeckBottomZ;
                bestDeckTopZ = candidateDeckTopZ;
            }
        }

        // 默认全部置灰，仅把通过几何约束的最佳桥梁候选高亮。
        const QColor backgroundColor(90, 90, 90);
        for (PclClusterGroup &cluster : result.clusters) {
            cluster.bridgeCandidate = false;
            cluster.bridgePier = false;
            cluster.color = backgroundColor;
        }
        if (!bestBridgeMembers.isEmpty()) {
            const QColor bridgeColor(0, 255, 230); // 统一亮青色
            for (int member : bestBridgeMembers) {
                result.clusters[member].bridgeCandidate = true;
                result.clusters[member].color = bridgeColor;
            }

            // 在已确认桥面下寻找桥墩。先选取顶部接近桥底、竖向明显的簇作为桥墩种子，
            // 再只在同一局部 XY 位置向下拼接断开的簇，形成近似“π”形整体。
            QVector<unsigned char> isDeck(clusterCount, 0);
            for (int member : bestBridgeMembers)
                isDeck[member] = 1;

            QVector<int> pierSeeds;
            for (int i = 0; i < clusterCount; ++i) {
                if (isDeck[i])
                    continue;
                const PclClusterGroup &cluster = result.clusters[i];
                const ClusterShape &shape = shapes[static_cast<std::size_t>(i)];
                const QVector3D size = cluster.maxBounds - cluster.minBounds;
                const float dx = cluster.center.x() - bestBridgeOrigin.x();
                const float dy = cluster.center.y() - bestBridgeOrigin.y();
                const float along = dx * bestDirectionX + dy * bestDirectionY;
                const float across = -dx * bestDirectionY + dy * bestDirectionX;
                const float horizontalSize = std::hypot(size.x(), size.y());
                const float verticalSize = (std::max)(0.0f, size.z());

                const bool underDeckPlan = along >= bestMinLongitudinal - 3.0f
                                        && along <= bestMaxLongitudinal + 3.0f
                                        && std::abs(across) <= bestHalfWidth + 5.0f;
                // 桥墩必须真正连接到某个桥面小簇，而不只是落在桥面下方的大范围内。
                bool attachedToDeck = false;
                for (int deckMember : bestBridgeMembers) {
                    const PclClusterGroup &deck = result.clusters[deckMember];
                    auto intervalGap = [](float a0, float a1, float b0, float b1) {
                        if (a1 < b0) return b0 - a1;
                        if (b1 < a0) return a0 - b1;
                        return 0.0f;
                    };
                    const float gapX = intervalGap(cluster.minBounds.x(), cluster.maxBounds.x(),
                                                   deck.minBounds.x(), deck.maxBounds.x());
                    const float gapY = intervalGap(cluster.minBounds.y(), cluster.maxBounds.y(),
                                                   deck.minBounds.y(), deck.maxBounds.y());
                    const float horizontalGap = std::hypot(gapX, gapY);
                    const float verticalGap = std::abs(cluster.maxBounds.z() - deck.minBounds.z());
                    if (horizontalGap <= 1.8f && verticalGap <= 2.0f) {
                        attachedToDeck = true;
                        break;
                    }
                }
                const bool belowDeck = cluster.center.z() < bestDeckBottomZ - 0.3f
                                    && cluster.minBounds.z() <= bestDeckBottomZ - 1.0f
                                    && cluster.maxBounds.z() <= bestDeckTopZ + 0.5f;
                const bool pillarLike = verticalSize >= 1.0f
                                     && horizontalSize <= 12.0f
                                     && verticalSize >= 0.30f * horizontalSize
                                     && shape.verticalAxisScore >= 0.50f;
                if (underDeckPlan && attachedToDeck && belowDeck && pillarLike)
                    pierSeeds.append(i);
            }

            QVector<unsigned char> isPier(clusterCount, 0);
            for (int seed : pierSeeds) {
                isPier[seed] = 1;
                bool added = true;
                while (added) {
                    added = false;
                    for (int candidate = 0; candidate < clusterCount; ++candidate) {
                        if (isDeck[candidate] || isPier[candidate])
                            continue;
                        const PclClusterGroup &part = result.clusters[candidate];
                        if (part.maxBounds.z() > bestDeckBottomZ + 1.0f)
                            continue;
                        for (int attached = 0; attached < clusterCount; ++attached) {
                            if (!isPier[attached])
                                continue;
                            const PclClusterGroup &base = result.clusters[attached];
                            const float horizontalDistance = std::hypot(
                                part.center.x() - base.center.x(),
                                part.center.y() - base.center.y());
                            float verticalGap = 0.0f;
                            if (part.minBounds.z() > base.maxBounds.z())
                                verticalGap = part.minBounds.z() - base.maxBounds.z();
                            else if (base.minBounds.z() > part.maxBounds.z())
                                verticalGap = base.minBounds.z() - part.maxBounds.z();
                            if (horizontalDistance <= 3.0f && verticalGap <= 2.5f) {
                                isPier[candidate] = 1;
                                added = true;
                                break;
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < clusterCount; ++i) {
                if (!isPier[i])
                    continue;
                result.clusters[i].bridgeCandidate = true;
                result.clusters[i].bridgePier = true;
            }

            // 冷启动阶段只出现长直线、却没有任何桥墩支撑时，不把它作为
            // 当前桥梁证据。真正桥梁至少应形成一个“横梁 + 竖柱”的 π 形局部结构。
            if (pierSeeds.isEmpty()) {
                for (int member : bestBridgeMembers) {
                    result.clusters[member].bridgeCandidate = false;
                    result.clusters[member].color = backgroundColor;
                }
            }
        }

        // 时序稳定：当前命中快速加分，漏检缓慢衰减。横跨约束加入后将
        // 衰减加快到每帧 2 分，使旧的巷道误检约 1 秒内退出高亮。
        for (auto it = bridgeConfidence_.begin(); it != bridgeConfidence_.end(); ++it)
            it.value() = (std::max)(0, it.value() - 2);
        for (auto it = pierConfidence_.begin(); it != pierConfidence_.end(); ++it)
            it.value() = (std::max)(0, it.value() - 2);

        for (PclClusterGroup &cluster : result.clusters) {
            int confidence = bridgeConfidence_.value(cluster.trackId, 0);
            if (cluster.bridgeCandidate)
                confidence = (std::min)(12, confidence + 4);
            bridgeConfidence_.insert(cluster.trackId, confidence);

            int pierConfidence = pierConfidence_.value(cluster.trackId, 0);
            if (cluster.bridgePier)
                pierConfidence = (std::min)(12, pierConfidence + 4);
            pierConfidence_.insert(cluster.trackId, pierConfidence);

            // 桥墩通过严格几何条件和连续帧门槛后，把语义写入稳定轨迹。
            // 后续即使因遮挡或点云稀疏暂时不满足几何条件，标签仍随轨迹保留。
            if (pierConfidence >= 8) {
                for (Track &track : tracks_) {
                    if (track.id == cluster.trackId) {
                        track.pierLocked = true;
                        break;
                    }
                }
            }
        }

        // 当前帧的几何命中只用于更新置信度，不能直接显示。清空瞬时标签后，
        // 仅让累计置信度通过门槛的稳定目标进入最终高亮和桥洞 ROI。
        for (PclClusterGroup &cluster : result.clusters) {
            cluster.bridgeCandidate = false;
            cluster.bridgePier = false;
            cluster.color = backgroundColor;
        }

        const QColor bridgeColor(0, 255, 230);
        const QColor pierColor(255, 150, 0);
        auto isLockedPierTrack = [this](int trackId) {
            for (const Track &track : tracks_) {
                if (track.id == trackId)
                    return track.pierLocked;
            }
            return false;
        };
        bool hasStableDeck = false;
        for (PclClusterGroup &cluster : result.clusters) {
            const int confidence = bridgeConfidence_.value(cluster.trackId, 0);
            const int pierConfidence = pierConfidence_.value(cluster.trackId, 0);
            if (confidence >= 8 && pierConfidence < 8
                && !isLockedPierTrack(cluster.trackId)) {
                cluster.bridgeCandidate = true;
                cluster.color = bridgeColor;
                hasStableDeck = true;
            }
        }

        // 严格 π 形检测只负责首次确认。确认后缓存桥面高度模型；船进入桥下、
        // 桥面暂时不可见时，只续跟踪已经严格确认并锁定的桥墩轨迹。
        // 这里绝不把新的竖直物体直接升级为桥墩，从而避免放宽条件导致误检。
        if (hasStableDeck) {
            std::vector<float> stableDeckHeights;
            for (const PclClusterGroup &cluster : result.clusters) {
                if (!cluster.bridgeCandidate || cluster.bridgePier)
                    continue;
                stableDeckHeights.reserve(stableDeckHeights.size()
                    + static_cast<std::size_t>(cluster.points.size()));
                for (const QVector3D &point : cluster.points)
                    stableDeckHeights.push_back(point.z());
            }
            if (!stableDeckHeights.empty()) {
                std::sort(stableDeckHeights.begin(), stableDeckHeights.end());
                cachedDeckBottomZ_ = stableDeckHeights[stableDeckHeights.size() / 10];
                cachedDeckTopZ_ = stableDeckHeights[(stableDeckHeights.size() * 9) / 10];
                cachedDeckHeight_ = stableDeckHeights[stableDeckHeights.size() / 2];
                hasCachedBridgeModel_ = true;
            }
            bridgeTrackingState_ = BridgeTrackingState::Confirmed;
            lastStableDeckTimeSec_ = trackingTimeSec_;
        }

        auto isValidLockedPierObservation = [&](const PclClusterGroup &cluster, int index) {
            if (!isLockedPierTrack(cluster.trackId) || !hasCachedBridgeModel_)
                return false;
            const QVector3D size = cluster.maxBounds - cluster.minBounds;
            const float horizontalSize = std::hypot(size.x(), size.y());
            const float verticalSize = (std::max)(0.0f, size.z());
            const ClusterShape &shape = shapes[static_cast<std::size_t>(index)];
            return cluster.center.z() < cachedDeckBottomZ_ + 0.5f
                && cluster.minBounds.z() < cachedDeckBottomZ_ - 0.5f
                && cluster.maxBounds.z() <= cachedDeckTopZ_ + 1.5f
                && verticalSize >= 0.8f
                && horizontalSize <= 14.0f
                && shape.verticalAxisScore >= 0.35f;
        };

        bool hasLockedPierObservation = false;
        for (int i = 0; i < result.clusters.size(); ++i) {
            if (isValidLockedPierObservation(result.clusters[i], i)) {
                hasLockedPierObservation = true;
                break;
            }
        }

        if (!hasStableDeck && hasLockedPierObservation && hasCachedBridgeModel_) {
            const bool withinEntryGrace = trackingTimeSec_ - lastStableDeckTimeSec_
                <= config_.underBridgeEntryGraceSeconds;
            if (bridgeTrackingState_ == BridgeTrackingState::Confirmed && withinEntryGrace) {
                bridgeTrackingState_ = BridgeTrackingState::UnderBridge;
                underBridgeStartTimeSec_ = trackingTimeSec_;
            }
            if (bridgeTrackingState_ == BridgeTrackingState::UnderBridge)
                lastLockedPierTimeSec_ = trackingTimeSec_;
        }

        if (bridgeTrackingState_ == BridgeTrackingState::Confirmed
            && !hasStableDeck
            && trackingTimeSec_ - lastStableDeckTimeSec_ > config_.underBridgeEntryGraceSeconds) {
            bridgeTrackingState_ = BridgeTrackingState::Searching;
            hasCachedBridgeModel_ = false;
        }
        if (bridgeTrackingState_ == BridgeTrackingState::UnderBridge) {
            const bool pierTimedOut = trackingTimeSec_ - lastLockedPierTimeSec_
                > config_.underBridgePierLostTimeoutSeconds;
            const bool modeTimedOut = trackingTimeSec_ - underBridgeStartTimeSec_
                > config_.underBridgeMaxDurationSeconds;
            if (pierTimedOut || modeTimedOut) {
                bridgeTrackingState_ = BridgeTrackingState::Searching;
                hasCachedBridgeModel_ = false;
            }
        }

        const bool underBridgeMode = bridgeTrackingState_ == BridgeTrackingState::UnderBridge;
        result.bridgeUnderPassMode = underBridgeMode;
        // 正常阶段仍要求当前稳定桥面；桥下阶段则只显示当前可见、已锁定且
        // 仍符合缓存高度/竖直性约束的桥墩，不允许桥墩语义凭空出现。
        if (hasStableDeck || underBridgeMode) {
            for (int i = 0; i < result.clusters.size(); ++i) {
                PclClusterGroup &cluster = result.clusters[i];
                const int pierConfidence = pierConfidence_.value(cluster.trackId, 0);
                const bool lockedPier = isLockedPierTrack(cluster.trackId);
                if (pierConfidence < 8 && !lockedPier)
                    continue;
                if (underBridgeMode && !isValidLockedPierObservation(cluster, i))
                    continue;
                cluster.bridgeCandidate = true;
                cluster.bridgePier = true;
                cluster.color = pierColor;
            }
        }

        for (auto it = bridgeConfidence_.begin(); it != bridgeConfidence_.end(); ) {
            if (it.value() <= 0)
                it = bridgeConfidence_.erase(it);
            else
                ++it;
        }
        for (auto it = pierConfidence_.begin(); it != pierConfidence_.end(); ) {
            if (it.value() <= 0)
                it = pierConfidence_.erase(it);
            else
                ++it;
        }

        // 距离以融合坐标原点（船/雷达位置）为基准，只使用最终高亮点云。
        // 最近水平距离用于避碰，点云质心水平距离提供更稳定的整体参考。
        float nearestDistance = 1.0e9f;
        double centerXSum = 0.0;
        double centerYSum = 0.0;
        float bridgeMinZ = 1.0e9f;
        float bridgeMaxZ = -1.0e9f;
        std::vector<float> deckHeights;
        double deckXSum = 0.0;
        double deckYSum = 0.0;
        double deckXXSum = 0.0;
        double deckXYSum = 0.0;
        double deckYYSum = 0.0;
        int deckPointCount = 0;
        int bridgePointCount = 0;
        for (const PclClusterGroup &cluster : result.clusters) {
            if (!cluster.bridgeCandidate)
                continue;
            if (!cluster.bridgePier)
                deckHeights.reserve(deckHeights.size() + static_cast<std::size_t>(cluster.points.size()));
            for (const QVector3D &point : cluster.points) {
                nearestDistance = (std::min)(nearestDistance,
                    std::hypot(point.x(), point.y()));
                centerXSum += point.x();
                centerYSum += point.y();
                bridgeMinZ = (std::min)(bridgeMinZ, point.z());
                bridgeMaxZ = (std::max)(bridgeMaxZ, point.z());
                if (!cluster.bridgePier) {
                    deckHeights.push_back(point.z());
                    deckXSum += point.x();
                    deckYSum += point.y();
                    deckXXSum += point.x() * point.x();
                    deckXYSum += point.x() * point.y();
                    deckYYSum += point.y() * point.y();
                    ++deckPointCount;
                }
                ++bridgePointCount;
            }
        }
        if (bridgePointCount > 0) {
            const float centerDistance = std::hypot(
                static_cast<float>(centerXSum / bridgePointCount),
                static_cast<float>(centerYSum / bridgePointCount));
            if (!hasSmoothedBridgeDistance_) {
                smoothedNearestDistance_ = nearestDistance;
                smoothedCenterDistance_ = centerDistance;
                hasSmoothedBridgeDistance_ = true;
            } else {
                constexpr float alpha = 0.25f;
                smoothedNearestDistance_ += alpha * (nearestDistance - smoothedNearestDistance_);
                smoothedCenterDistance_ += alpha * (centerDistance - smoothedCenterDistance_);
            }
            result.bridgeDetected = true;
            result.bridgeNearestDistanceMeters = smoothedNearestDistance_;
            result.bridgeCenterDistanceMeters = smoothedCenterDistance_;
            result.bridgeVerticalSpanMeters = (std::max)(0.0f, bridgeMaxZ - bridgeMinZ);
            if (!deckHeights.empty()) {
                std::sort(deckHeights.begin(), deckHeights.end());
                result.bridgeDeckHeightMeters = deckHeights[deckHeights.size() / 2];
            } else if (result.bridgeUnderPassMode && hasCachedBridgeModel_) {
                result.bridgeDeckHeightMeters = cachedDeckHeight_;
            }
            result.bridgePointCount = bridgePointCount;

            // 将同一桥墩上下断裂的小簇按 XY 位置合并成独立桥墩目标。
            struct PierAggregate {
                double xSum = 0.0;
                double ySum = 0.0;
                double zSum = 0.0;
                float nearestDistance = 1.0e9f;
                QVector3D minBounds = QVector3D(1.0e9f, 1.0e9f, 1.0e9f);
                QVector3D maxBounds = QVector3D(-1.0e9f, -1.0e9f, -1.0e9f);
                int pointCount = 0;
            };
            QVector<PierAggregate> pierAggregates;
            for (const PclClusterGroup &cluster : result.clusters) {
                if (!cluster.bridgePier || cluster.points.isEmpty())
                    continue;

                int bestAggregate = -1;
                float bestDistance = 5.0f;
                for (int i = 0; i < pierAggregates.size(); ++i) {
                    const PierAggregate &aggregate = pierAggregates[i];
                    if (aggregate.pointCount <= 0)
                        continue;
                    const float aggregateX = static_cast<float>(aggregate.xSum / aggregate.pointCount);
                    const float aggregateY = static_cast<float>(aggregate.ySum / aggregate.pointCount);
                    const float distance = std::hypot(cluster.center.x() - aggregateX,
                                                      cluster.center.y() - aggregateY);
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestAggregate = i;
                    }
                }
                if (bestAggregate < 0) {
                    pierAggregates.append(PierAggregate{});
                    bestAggregate = pierAggregates.size() - 1;
                }

                PierAggregate &aggregate = pierAggregates[bestAggregate];
                for (const QVector3D &point : cluster.points) {
                    aggregate.xSum += point.x();
                    aggregate.ySum += point.y();
                    aggregate.zSum += point.z();
                    aggregate.nearestDistance = (std::min)(aggregate.nearestDistance,
                        std::hypot(point.x(), point.y()));
                    aggregate.minBounds.setX((std::min)(aggregate.minBounds.x(), point.x()));
                    aggregate.minBounds.setY((std::min)(aggregate.minBounds.y(), point.y()));
                    aggregate.minBounds.setZ((std::min)(aggregate.minBounds.z(), point.z()));
                    aggregate.maxBounds.setX((std::max)(aggregate.maxBounds.x(), point.x()));
                    aggregate.maxBounds.setY((std::max)(aggregate.maxBounds.y(), point.y()));
                    aggregate.maxBounds.setZ((std::max)(aggregate.maxBounds.z(), point.z()));
                    ++aggregate.pointCount;
                }
            }

            for (const PierAggregate &aggregate : pierAggregates) {
                if (aggregate.pointCount <= 0)
                    continue;
                PclBridgePierInfo info;
                info.center = QVector3D(
                    static_cast<float>(aggregate.xSum / aggregate.pointCount),
                    static_cast<float>(aggregate.ySum / aggregate.pointCount),
                    static_cast<float>(aggregate.zSum / aggregate.pointCount));
                info.nearestDistanceMeters = aggregate.nearestDistance;
                info.centerDistanceMeters = std::hypot(info.center.x(), info.center.y());
                info.bearingDegrees = static_cast<float>(
                    std::atan2(info.center.y(), info.center.x()) * 180.0 / 3.14159265358979323846);
                info.pointCount = aggregate.pointCount;
                result.bridgePiers.append(info);
            }
            std::sort(result.bridgePiers.begin(), result.bridgePiers.end(),
                      [](const PclBridgePierInfo &a, const PclBridgePierInfo &b) {
                return a.nearestDistanceMeters < b.nearestDistanceMeters;
            });

            // 只有至少两个稳定桥墩时才生成桥洞 ROI。先由稳定桥面点重新估计
            // 桥梁主方向，再把相邻桥墩之间的走廊切成多个小 AABB，近似定向 ROI。
            if (pierAggregates.size() >= 2 && deckPointCount >= 30 && !deckHeights.empty()) {
                const double meanX = deckXSum / deckPointCount;
                const double meanY = deckYSum / deckPointCount;
                const double covarianceXX = deckXXSum / deckPointCount - meanX * meanX;
                const double covarianceXY = deckXYSum / deckPointCount - meanX * meanY;
                const double covarianceYY = deckYYSum / deckPointCount - meanY * meanY;
                const double trace = covarianceXX + covarianceYY;
                const double delta = std::sqrt((covarianceXX - covarianceYY)
                                               * (covarianceXX - covarianceYY)
                                               + 4.0 * covarianceXY * covarianceXY);
                const double major = 0.5 * (trace + delta);
                double directionX = covarianceXY;
                double directionY = major - covarianceXX;
                double directionNorm = std::hypot(directionX, directionY);
                if (directionNorm < 1.0e-6) {
                    directionX = 1.0;
                    directionY = 0.0;
                    directionNorm = 1.0;
                }
                directionX /= directionNorm;
                directionY /= directionNorm;

                float minAcross = 1.0e9f;
                float maxAcross = -1.0e9f;
                for (const PclClusterGroup &cluster : result.clusters) {
                    if (!cluster.bridgeCandidate || cluster.bridgePier)
                        continue;
                    for (const QVector3D &point : cluster.points) {
                        const float across = static_cast<float>(
                            -point.x() * directionY + point.y() * directionX);
                        minAcross = (std::min)(minAcross, across);
                        maxAcross = (std::max)(maxAcross, across);
                    }
                }
                const float corridorHalfWidth = std::clamp(
                    0.5f * (maxAcross - minAcross) + 2.0f, 4.0f, 12.0f);

                QVector<int> pierOrder;
                for (int i = 0; i < pierAggregates.size(); ++i) {
                    if (pierAggregates[i].pointCount > 0)
                        pierOrder.append(i);
                }
                auto pierCenter = [&pierAggregates](int index) {
                    const PierAggregate &pier = pierAggregates[index];
                    return QVector3D(static_cast<float>(pier.xSum / pier.pointCount),
                                     static_cast<float>(pier.ySum / pier.pointCount),
                                     static_cast<float>(pier.zSum / pier.pointCount));
                };
                std::sort(pierOrder.begin(), pierOrder.end(),
                          [&](int a, int b) {
                    const QVector3D ca = pierCenter(a);
                    const QVector3D cb = pierCenter(b);
                    return ca.x() * directionX + ca.y() * directionY
                         < cb.x() * directionX + cb.y() * directionY;
                });

                const float deckTop = deckHeights[(deckHeights.size() * 9) / 10] + 1.5f;
                for (int pair = 0; pair + 1 < pierOrder.size(); ++pair) {
                    const PierAggregate &pierA = pierAggregates[pierOrder[pair]];
                    const PierAggregate &pierB = pierAggregates[pierOrder[pair + 1]];
                    const QVector3D centerA = pierCenter(pierOrder[pair]);
                    const QVector3D centerB = pierCenter(pierOrder[pair + 1]);
                    const float alongA = static_cast<float>(centerA.x() * directionX
                                                           + centerA.y() * directionY);
                    const float alongB = static_cast<float>(centerB.x() * directionX
                                                           + centerB.y() * directionY);
                    const float separation = alongB - alongA;
                    if (separation < 6.0f || separation > 120.0f)
                        continue;
                    const float acrossA = static_cast<float>(-centerA.x() * directionY
                                                             + centerA.y() * directionX);
                    const float acrossB = static_cast<float>(-centerB.x() * directionY
                                                             + centerB.y() * directionX);
                    const float corridorAcross = 0.5f * (acrossA + acrossB);
                    const float bottom = (std::min)(pierA.minBounds.z(), pierB.minBounds.z()) - 1.0f;
                    const int segmentCount = (std::max)(1, static_cast<int>(std::ceil(separation / 6.0f)));
                    for (int segment = 0; segment < segmentCount; ++segment) {
                        const float segmentStart = alongA + separation * segment / segmentCount;
                        const float segmentEnd = alongA + separation * (segment + 1) / segmentCount;
                        const float segmentCenter = 0.5f * (segmentStart + segmentEnd);
                        const float halfAlong = 0.5f * (segmentEnd - segmentStart) + 1.0f;
                        const float centerX = static_cast<float>(segmentCenter * directionX
                                                               - corridorAcross * directionY);
                        const float centerY = static_cast<float>(segmentCenter * directionY
                                                               + corridorAcross * directionX);
                        const float halfX = static_cast<float>(std::abs(directionX) * halfAlong
                                                             + std::abs(directionY) * corridorHalfWidth);
                        const float halfY = static_cast<float>(std::abs(directionY) * halfAlong
                                                             + std::abs(directionX) * corridorHalfWidth);
                        PclBridgeHoleRoi roi;
                        roi.minBounds = QVector3D(centerX - halfX, centerY - halfY, bottom);
                        roi.maxBounds = QVector3D(centerX + halfX, centerY + halfY, deckTop);
                        result.bridgeHoleRois.append(roi);
                    }
                }
            }
        }
    }

    // 障碍物距离以当前界面船体矩形边界为基准。100 m 内全部目标参与
    // 排序和信息输出；橙色边界只保留左右两侧各最近 3 个，避免满屏线框。
    struct ObstacleCandidate {
        PclObstacleInfo info;
        PclClusterBoundary boundary;
    };
    QVector<ObstacleCandidate> obstacleCandidates;
    obstacleCandidates.reserve(result.clusters.size());
    auto updateObstacleTrack = [this](int trackId,
                                      float rawClearance,
                                      float &smoothedClearance,
                                      float &closingSpeed) {
        Track *matchedTrack = nullptr;
        for (Track &track : tracks_) {
            if (track.id == trackId) {
                matchedTrack = &track;
                break;
            }
        }
        if (!matchedTrack) {
            smoothedClearance = rawClearance;
            closingSpeed = 0.0f;
            return;
        }

        Track &track = *matchedTrack;
        if (!track.hasObstacleRange) {
            track.hasObstacleRange = true;
            track.lastRawClearance = rawClearance;
            track.smoothedClearance = rawClearance;
            track.smoothedClosingSpeed = 0.0f;
            track.lastObstacleUpdateSec = trackingTimeSec_;
        } else {
            const double deltaSec = trackingTimeSec_ - track.lastObstacleUpdateSec;
            track.smoothedClearance += 0.35f
                * (rawClearance - track.smoothedClearance);
            if (deltaSec >= 0.03 && deltaSec <= 2.0) {
                const float measuredClosingSpeed = std::clamp(
                    static_cast<float>((track.lastRawClearance - rawClearance) / deltaSec),
                    -15.0f, 15.0f);
                track.smoothedClosingSpeed += 0.25f
                    * (measuredClosingSpeed - track.smoothedClosingSpeed);
            }
            track.lastRawClearance = rawClearance;
            track.lastObstacleUpdateSec = trackingTimeSec_;
        }
        smoothedClearance = track.smoothedClearance;
        closingSpeed = track.smoothedClosingSpeed;
    };

    for (const PclClusterGroup &cluster : result.clusters) {
        PclClusterBoundary boundary;
        boundary.trackId = cluster.trackId;
        boundary.minZ = cluster.minBounds.z();
        boundary.maxZ = cluster.maxBounds.z();
        boundary.footprint = convexHullXY(cluster.points, config_.maxBoundaryVertices);
        if (boundary.footprint.size() < 2)
            continue;

        const BoundaryDistanceResult distance = distanceFromBoundaryToShip(
            boundary.footprint, config_.shipHalfExtentX, config_.shipHalfExtentY);
        if (!std::isfinite(distance.distanceSquared))
            continue;
        const double clearance = std::sqrt(distance.distanceSquared);
        if (clearance > config_.obstacleDetectionRangeMeters)
            continue;

        const float obstacleZ = 0.5f * (boundary.minZ + boundary.maxZ);
        ObstacleCandidate candidate;
        candidate.info.trackId = boundary.trackId;
        candidate.info.rawClearanceMeters = static_cast<float>(clearance);
        updateObstacleTrack(candidate.info.trackId,
                            candidate.info.rawClearanceMeters,
                            candidate.info.clearanceMeters,
                            candidate.info.closingSpeedMetersPerSecond);
        candidate.info.obstaclePoint = distance.obstaclePoint;
        candidate.info.obstaclePoint.setZ(obstacleZ);
        candidate.info.shipPoint = distance.shipPoint;
        candidate.info.sensorDistanceMeters = std::hypot(
            candidate.info.obstaclePoint.x(), candidate.info.obstaclePoint.y());
        candidate.info.bearingDegrees = static_cast<float>(
            std::atan2(candidate.info.obstaclePoint.y(), candidate.info.obstaclePoint.x())
            * 180.0 / 3.14159265358979323846);
        candidate.boundary = std::move(boundary);
        obstacleCandidates.append(std::move(candidate));
    }

    std::sort(obstacleCandidates.begin(), obstacleCandidates.end(),
              [](const ObstacleCandidate &a, const ObstacleCandidate &b) {
        // 安全排序使用当前帧原始距离，信息显示使用平滑距离。
        return a.info.rawClearanceMeters < b.info.rawClearanceMeters;
    });
    result.obstacles.reserve(obstacleCandidates.size());
    for (const ObstacleCandidate &candidate : obstacleCandidates)
        result.obstacles.append(candidate.info);

    if (!obstacleCandidates.isEmpty()) {
        const PclObstacleInfo &nearest = obstacleCandidates.front().info;
        result.nearestObstacleDetected = true;
        result.nearestObstacleTrackId = nearest.trackId;
        result.nearestObstacleClearanceMeters = nearest.clearanceMeters;
        result.nearestObstacleSensorDistanceMeters = nearest.sensorDistanceMeters;
        result.nearestObstacleBearingDegrees = nearest.bearingDegrees;
        result.nearestObstaclePoint = nearest.obstaclePoint;
        result.nearestShipPoint = nearest.shipPoint;
    }

    int highlightedLeft = 0;
    int highlightedRight = 0;
    for (ObstacleCandidate &candidate : obstacleCandidates) {
        // 项目现有方位定义为 +Y 左侧、-Y 右侧。
        int &sideCount = candidate.info.obstaclePoint.y() >= 0.0f
            ? highlightedLeft : highlightedRight;
        if (sideCount >= config_.maxHighlightedObstaclesPerSide)
            continue;
        result.clusterBoundaries.append(std::move(candidate.boundary));
        ++sideCount;
    }

    // 检测、滤波和聚类始终停留在后台的下采样点云。最终语义确定后，
    // 用单棵 KD-Tree 将标签映射回本帧原始点云，供界面高分辨率显示。
    // 最近邻半径较小，避免用大包围盒把桥下车辆或邻近设施一起染色。
    pcl::PointCloud<pcl::PointXYZ>::Ptr labeledReference(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<unsigned char> referenceLabels;
    std::size_t referencePointCount = 0;
    for (const PclClusterGroup &cluster : result.clusters) {
        if (cluster.bridgeCandidate)
            referencePointCount += static_cast<std::size_t>(cluster.points.size());
    }
    labeledReference->reserve(referencePointCount);
    referenceLabels.reserve(referencePointCount);
    for (const PclClusterGroup &cluster : result.clusters) {
        if (!cluster.bridgeCandidate)
            continue;
        const unsigned char label = cluster.bridgePier ? 2 : 1;
        for (const QVector3D &point : cluster.points) {
            labeledReference->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
            referenceLabels.push_back(label);
        }
    }

    if (!labeledReference->empty()) {
        pcl::KdTreeFLANN<pcl::PointXYZ> labelTree;
        labelTree.setInputCloud(labeledReference);
        std::vector<int> nearestIndex(1);
        std::vector<float> nearestSquaredDistance(1);
        const float radiusSquared = config_.highResolutionMappingRadius
                                  * config_.highResolutionMappingRadius;
        result.highResolutionBridgePoints.reserve(
            static_cast<int>((std::min)(referencePointCount * 4,
                                        static_cast<std::size_t>(cloud.size()))));
        result.highResolutionPierPoints.reserve(
            static_cast<int>((std::min)(referencePointCount * 2,
                                        static_cast<std::size_t>(cloud.size()))));
        for (const M_PointXYZI &point : cloud) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            const bool insideShip = std::abs(point.x) <= config_.shipHalfExtentX
                                 && std::abs(point.y) <= config_.shipHalfExtentY;
            if (insideShip
                || point.x < -250.0f || point.x > 250.0f
                || point.y < -250.0f || point.y > 250.0f
                || point.z < -15.0f || point.z > 50.0f) {
                continue;
            }
            const pcl::PointXYZ query(point.x, point.y, point.z);
            if (labelTree.nearestKSearch(query, 1,
                                         nearestIndex,
                                         nearestSquaredDistance) <= 0
                || nearestSquaredDistance[0] > radiusSquared) {
                continue;
            }
            const int labelIndex = nearestIndex[0];
            if (labelIndex < 0
                || labelIndex >= static_cast<int>(referenceLabels.size())) {
                continue;
            }
            const QVector3D originalPoint(point.x, point.y, point.z);
            if (referenceLabels[static_cast<std::size_t>(labelIndex)] == 2)
                result.highResolutionPierPoints.append(originalPoint);
            else
                result.highResolutionBridgePoints.append(originalPoint);
        }
    }

    // 下采样簇只在后台参与判断，不再随结果复制到 UI。界面只接收轻量的
    // 检测信息和原始分辨率高亮点，灰色底图直接使用已有融合原始点云。
    result.classificationReady = true;
    result.clusters.clear();
    result.unclusteredPoints.clear();
    return result;
}


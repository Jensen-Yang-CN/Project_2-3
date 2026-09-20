#include "SlamMapStitcher.h"

#include "SlamMapGeoUtils.h"
#include "SlamMapBerthGeometry.h"

#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <QStringList>
#include <unordered_map>
#include <utility>
#include <vector>

namespace slam_map_stitch {

namespace {

constexpr std::array<uint8_t, 4> kSourceIntensities{{90, 130, 30, 180}};
constexpr double kRadiansToDegrees = 57.2957795130823208768;

using RegistrationPoint = pcl::PointXYZ;
using RegistrationCloud = pcl::PointCloud<RegistrationPoint>;

struct Bounds2d {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
};

struct TimeBounds {
    double first = std::numeric_limits<double>::infinity();
    double last = -std::numeric_limits<double>::infinity();
};

bool validKeyframe(const usv::SlamKeyframe &keyframe)
{
    if (!std::isfinite(keyframe.timestamp) || keyframe.cloud.empty()
        || !keyframe.pose.matrix().allFinite()) {
        return false;
    }
    for (const M_PointXYZI &point : keyframe.cloud) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)
            || !std::isfinite(point.z)) {
            return false;
        }
    }
    return true;
}

bool validArchive(const slam_map_io::MapArchive &archive)
{
    if (archive.keyframes.empty())
        return false;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        if (!validKeyframe(keyframe))
            return false;
    }
    return true;
}

TimeBounds timeBoundsOf(const slam_map_io::MapArchive &archive)
{
    TimeBounds bounds;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        bounds.first = std::min(bounds.first, keyframe.timestamp);
        bounds.last = std::max(bounds.last, keyframe.timestamp);
    }
    return bounds;
}

bool interpolatePosition(const std::vector<const usv::SlamKeyframe *> &trajectory,
                         double timestamp,
                         Eigen::Vector2d &position)
{
    if (trajectory.empty()
        || timestamp < trajectory.front()->timestamp
        || timestamp > trajectory.back()->timestamp) {
        return false;
    }
    const auto upper = std::lower_bound(
        trajectory.begin(), trajectory.end(), timestamp,
        [](const usv::SlamKeyframe *keyframe, double value) {
            return keyframe->timestamp < value;
        });
    if (upper == trajectory.begin()) {
        position = trajectory.front()->pose.translation().head<2>();
        return true;
    }
    if (upper == trajectory.end()) {
        position = trajectory.back()->pose.translation().head<2>();
        return true;
    }
    if ((*upper)->timestamp == timestamp) {
        position = (*upper)->pose.translation().head<2>();
        return true;
    }
    const usv::SlamKeyframe *right = *upper;
    const usv::SlamKeyframe *left = *(upper - 1);
    const double duration = right->timestamp - left->timestamp;
    if (!(duration > 0.0))
        return false;
    const double ratio = (timestamp - left->timestamp) / duration;
    position = (1.0 - ratio) * left->pose.translation().head<2>()
             + ratio * right->pose.translation().head<2>();
    return position.allFinite();
}

double quantile(std::vector<double> values, double probability)
{
    if (values.empty())
        return std::numeric_limits<double>::infinity();
    std::sort(values.begin(), values.end());
    const double index = probability
        * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(index));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
    const double fraction = index - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

bool estimateTemporalCorrection(
    const slam_map_io::MapArchive &targetArchive,
    const slam_map_io::MapArchive &sourceArchive,
    const Eigen::Isometry3d &coarse,
    double overlapEnd,
    const Options &options,
    Eigen::Isometry3d &correction,
    Diagnostic &diagnostic)
{
    std::vector<const usv::SlamKeyframe *> targetTrajectory;
    targetTrajectory.reserve(targetArchive.keyframes.size());
    for (const usv::SlamKeyframe &keyframe : targetArchive.keyframes)
        targetTrajectory.push_back(&keyframe);
    std::stable_sort(
        targetTrajectory.begin(), targetTrajectory.end(),
        [](const usv::SlamKeyframe *left,
           const usv::SlamKeyframe *right) {
            return left->timestamp < right->timestamp;
        });

    const double windowStart = overlapEnd - options.temporal_seam_window_s;
    std::vector<Eigen::Vector2d> sourcePositions;
    std::vector<Eigen::Vector2d> targetPositions;
    for (const usv::SlamKeyframe &keyframe : sourceArchive.keyframes) {
        if (keyframe.timestamp < windowStart
            || keyframe.timestamp > overlapEnd) {
            continue;
        }
        Eigen::Vector2d targetPosition;
        if (!interpolatePosition(targetTrajectory, keyframe.timestamp,
                                 targetPosition)) {
            continue;
        }
        const Eigen::Vector3d sourcePosition =
            (coarse * keyframe.pose).translation();
        sourcePositions.push_back(sourcePosition.head<2>());
        targetPositions.push_back(targetPosition);
    }
    diagnostic.temporal_pose_matches =
        static_cast<int>(sourcePositions.size());
    if (diagnostic.temporal_pose_matches
        < options.min_temporal_pose_matches) {
        diagnostic.message = QStringLiteral(
            "时间接缝回退：匹配位姿不足（%1）")
                                 .arg(diagnostic.temporal_pose_matches);
        return false;
    }

    Eigen::Vector2d sourceCenter = Eigen::Vector2d::Zero();
    Eigen::Vector2d targetCenter = Eigen::Vector2d::Zero();
    for (std::size_t index = 0; index < sourcePositions.size(); ++index) {
        sourceCenter += sourcePositions[index];
        targetCenter += targetPositions[index];
    }
    sourceCenter /= static_cast<double>(sourcePositions.size());
    targetCenter /= static_cast<double>(targetPositions.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (std::size_t index = 0; index < sourcePositions.size(); ++index) {
        covariance += (sourcePositions[index] - sourceCenter)
                    * (targetPositions[index] - targetCenter).transpose();
    }
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0) {
        Eigen::Matrix2d adjustedV = svd.matrixV();
        adjustedV.col(1) *= -1.0;
        rotation = adjustedV * svd.matrixU().transpose();
    }
    const Eigen::Vector2d translation =
        targetCenter - rotation * sourceCenter;
    diagnostic.correction_translation_m = translation.norm();
    diagnostic.correction_yaw_deg =
        std::atan2(rotation(1, 0), rotation(0, 0)) * kRadiansToDegrees;

    std::vector<double> residuals;
    residuals.reserve(sourcePositions.size());
    for (std::size_t index = 0; index < sourcePositions.size(); ++index) {
        residuals.push_back(
            (rotation * sourcePositions[index] + translation
             - targetPositions[index]).norm());
    }
    diagnostic.temporal_median_error_m = quantile(residuals, 0.5);
    diagnostic.temporal_p90_error_m = quantile(residuals, 0.9);

    const bool accepted = rotation.allFinite() && translation.allFinite()
        && diagnostic.temporal_median_error_m
               <= options.max_temporal_median_error_m
        && diagnostic.temporal_p90_error_m
               <= options.max_temporal_p90_error_m
        && diagnostic.correction_translation_m
               <= options.max_temporal_correction_translation_m
        && std::abs(diagnostic.correction_yaw_deg)
               <= options.max_temporal_correction_yaw_deg;
    if (!accepted) {
        diagnostic.message = QStringLiteral(
            "时间接缝回退：匹配=%1，中位残差=%2 m，P90=%3 m，平移=%4 m，偏航=%5°")
                                 .arg(diagnostic.temporal_pose_matches)
                                 .arg(diagnostic.temporal_median_error_m,
                                      0, 'f', 3)
                                 .arg(diagnostic.temporal_p90_error_m,
                                      0, 'f', 3)
                                 .arg(diagnostic.correction_translation_m,
                                      0, 'f', 3)
                                 .arg(diagnostic.correction_yaw_deg,
                                      0, 'f', 3);
        return false;
    }

    correction = Eigen::Isometry3d::Identity();
    correction.linear().topLeftCorner<2, 2>() = rotation;
    correction.translation().head<2>() = translation;
    diagnostic.message = QStringLiteral(
        "时间接缝已接受：重叠=%1 s，匹配=%2，中位残差=%3 m，P90=%4 m，平移=%5 m，偏航=%6°，裁剪=%7 帧，新增=%8 帧")
                             .arg(diagnostic.temporal_overlap_s, 0, 'f', 1)
                             .arg(diagnostic.temporal_pose_matches)
                             .arg(diagnostic.temporal_median_error_m, 0, 'f', 3)
                             .arg(diagnostic.temporal_p90_error_m, 0, 'f', 3)
                             .arg(diagnostic.correction_translation_m,
                                  0, 'f', 3)
                             .arg(diagnostic.correction_yaw_deg,
                                  0, 'f', 3)
                             .arg(diagnostic.trimmed_keyframes)
                             .arg(diagnostic.appended_keyframes);
    return true;
}

RegistrationCloud::Ptr buildWorldCloud(
    const slam_map_io::MapArchive &archive,
    const Eigen::Isometry3d &baseFromArchive = Eigen::Isometry3d::Identity())
{
    RegistrationCloud::Ptr cloud(new RegistrationCloud);
    std::size_t totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes)
        totalPoints += keyframe.cloud.size();
    cloud->points.reserve(totalPoints);
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        const Eigen::Isometry3d worldFromLidar =
            baseFromArchive * keyframe.pose;
        for (const M_PointXYZI &point : keyframe.cloud) {
            const Eigen::Vector3d world = worldFromLidar
                * Eigen::Vector3d(point.x, point.y, point.z);
            cloud->points.emplace_back(static_cast<float>(world.x()),
                                       static_cast<float>(world.y()),
                                       0.0f);
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

struct StableCellKey {
    std::int64_t x = 0;
    std::int64_t y = 0;

    bool operator==(const StableCellKey &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct StableCellKeyHash {
    std::size_t operator()(const StableCellKey &key) const
    {
        const std::size_t xHash = std::hash<std::int64_t>{}(key.x);
        const std::size_t yHash = std::hash<std::int64_t>{}(key.y);
        return xHash ^ (yHash + 0x9e3779b97f4a7c15ULL
                        + (xHash << 6U) + (xHash >> 2U));
    }
};

struct StableCell {
    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t point_count = 0;
    std::size_t last_keyframe = std::numeric_limits<std::size_t>::max();
    int observation_count = 0;
};

struct RegistrationSelection {
    RegistrationCloud::Ptr cloud;
    bool used_stable_points = false;
};

RegistrationSelection buildRegistrationWorldCloud(
    const slam_map_io::MapArchive &archive,
    const Eigen::Isometry3d &baseFromArchive,
    const Options &options)
{
    RegistrationSelection selection;
    if (!(options.stable_voxel_size_m > 0.0)
        || options.min_stable_observations <= 1
        || archive.keyframes.size()
               < static_cast<std::size_t>(options.min_stable_observations)) {
        selection.cloud = buildWorldCloud(archive, baseFromArchive);
        return selection;
    }

    std::unordered_map<StableCellKey, StableCell, StableCellKeyHash> cells;
    const double inverseLeaf = 1.0 / options.stable_voxel_size_m;
    for (std::size_t frameIndex = 0;
         frameIndex < archive.keyframes.size(); ++frameIndex) {
        const usv::SlamKeyframe &keyframe = archive.keyframes[frameIndex];
        const Eigen::Isometry3d worldFromLidar =
            baseFromArchive * keyframe.pose;
        for (const M_PointXYZI &point : keyframe.cloud) {
            const Eigen::Vector3d world = worldFromLidar
                * Eigen::Vector3d(point.x, point.y, point.z);
            const StableCellKey key{
                static_cast<std::int64_t>(std::floor(world.x() * inverseLeaf)),
                static_cast<std::int64_t>(std::floor(world.y() * inverseLeaf))};
            StableCell &cell = cells[key];
            cell.sum_x += world.x();
            cell.sum_y += world.y();
            ++cell.point_count;
            if (cell.last_keyframe != frameIndex) {
                cell.last_keyframe = frameIndex;
                ++cell.observation_count;
            }
        }
    }

    RegistrationCloud::Ptr stable(new RegistrationCloud);
    stable->points.reserve(cells.size());
    for (const auto &entry : cells) {
        const StableCell &cell = entry.second;
        if (cell.observation_count < options.min_stable_observations
            || cell.point_count == 0) {
            continue;
        }
        stable->points.emplace_back(
            static_cast<float>(cell.sum_x
                               / static_cast<double>(cell.point_count)),
            static_cast<float>(cell.sum_y
                               / static_cast<double>(cell.point_count)),
            0.0f);
    }
    stable->width = static_cast<uint32_t>(stable->points.size());
    stable->height = 1;
    stable->is_dense = true;
    if (stable->points.size()
        >= static_cast<std::size_t>(std::max(1, options.min_stable_points))) {
        selection.cloud = stable;
        selection.used_stable_points = true;
        return selection;
    }

    selection.cloud = buildWorldCloud(archive, baseFromArchive);
    return selection;
}

RegistrationCloud::Ptr voxelDownsample(const RegistrationCloud::Ptr &input,
                                       double leafSize)
{
    RegistrationCloud::Ptr output(new RegistrationCloud);
    pcl::VoxelGrid<RegistrationPoint> filter;
    filter.setInputCloud(input);
    const float leaf = static_cast<float>(leafSize);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
    return output;
}

Bounds2d boundsOf(const RegistrationCloud::Ptr &cloud)
{
    Bounds2d bounds;
    for (const RegistrationPoint &point : cloud->points) {
        bounds.min_x = std::min(bounds.min_x, static_cast<double>(point.x));
        bounds.max_x = std::max(bounds.max_x, static_cast<double>(point.x));
        bounds.min_y = std::min(bounds.min_y, static_cast<double>(point.y));
        bounds.max_y = std::max(bounds.max_y, static_cast<double>(point.y));
    }
    return bounds;
}

RegistrationCloud::Ptr extractOverlap(const RegistrationCloud::Ptr &cloud,
                                      double minX, double maxX,
                                      double minY, double maxY)
{
    RegistrationCloud::Ptr output(new RegistrationCloud);
    output->points.reserve(cloud->points.size());
    for (const RegistrationPoint &point : cloud->points) {
        if (point.x >= minX && point.x <= maxX
            && point.y >= minY && point.y <= maxY) {
            output->points.push_back(point);
        }
    }
    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

RegistrationCloud::Ptr capped(const RegistrationCloud::Ptr &cloud,
                              int maxPoints)
{
    if (maxPoints <= 0
        || cloud->points.size() <= static_cast<std::size_t>(maxPoints)) {
        return cloud;
    }
    RegistrationCloud::Ptr output(new RegistrationCloud);
    output->points.reserve(static_cast<std::size_t>(maxPoints));
    const double step = static_cast<double>(cloud->points.size())
                      / static_cast<double>(maxPoints);
    for (int index = 0; index < maxPoints; ++index) {
        const std::size_t sourceIndex = std::min(
            static_cast<std::size_t>(index * step),
            cloud->points.size() - 1);
        output->points.push_back(cloud->points[sourceIndex]);
    }
    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

struct DirectionalQuality {
    double robust_rmse_m = std::numeric_limits<double>::infinity();
    double match_ratio = 0.0;
};

struct BidirectionalQuality {
    double robust_rmse_m = std::numeric_limits<double>::infinity();
    double source_match_ratio = 0.0;
    double target_match_ratio = 0.0;
    double score = std::numeric_limits<double>::infinity();
};

DirectionalQuality directionalQuality(
    const RegistrationCloud::Ptr &query,
    const RegistrationCloud::Ptr &reference,
    double maxDistance)
{
    DirectionalQuality quality;
    if (query->empty() || reference->empty() || !(maxDistance > 0.0))
        return quality;

    pcl::KdTreeFLANN<RegistrationPoint> tree;
    tree.setInputCloud(reference);
    std::vector<double> squaredDistances;
    squaredDistances.reserve(query->points.size());
    std::vector<int> nearestIndex(1);
    std::vector<float> nearestSquaredDistance(1);
    const double maxSquaredDistance = maxDistance * maxDistance;
    for (const RegistrationPoint &point : query->points) {
        if (tree.nearestKSearch(point, 1, nearestIndex,
                                nearestSquaredDistance) > 0
            && nearestSquaredDistance[0] <= maxSquaredDistance) {
            squaredDistances.push_back(nearestSquaredDistance[0]);
        }
    }
    quality.match_ratio = static_cast<double>(squaredDistances.size())
                        / static_cast<double>(query->points.size());
    if (squaredDistances.empty())
        return quality;

    std::sort(squaredDistances.begin(), squaredDistances.end());
    const std::size_t retained = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(0.9 * static_cast<double>(squaredDistances.size()))));
    double sum = 0.0;
    for (std::size_t index = 0; index < retained; ++index)
        sum += squaredDistances[index];
    quality.robust_rmse_m =
        std::sqrt(sum / static_cast<double>(retained));
    return quality;
}

DirectionalQuality directionalQuality(
    const RegistrationCloud::Ptr &query,
    pcl::KdTreeFLANN<RegistrationPoint> &referenceTree,
    double maxDistance)
{
    DirectionalQuality quality;
    if (query->empty() || !(maxDistance > 0.0))
        return quality;

    std::vector<double> squaredDistances;
    squaredDistances.reserve(query->points.size());
    std::vector<int> nearestIndex(1);
    std::vector<float> nearestSquaredDistance(1);
    const double maxSquaredDistance = maxDistance * maxDistance;
    for (const RegistrationPoint &point : query->points) {
        if (referenceTree.nearestKSearch(point, 1, nearestIndex,
                                         nearestSquaredDistance) > 0
            && nearestSquaredDistance[0] <= maxSquaredDistance) {
            squaredDistances.push_back(nearestSquaredDistance[0]);
        }
    }
    quality.match_ratio = static_cast<double>(squaredDistances.size())
                        / static_cast<double>(query->points.size());
    if (squaredDistances.empty())
        return quality;

    std::sort(squaredDistances.begin(), squaredDistances.end());
    const std::size_t retained = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(0.9 * static_cast<double>(squaredDistances.size()))));
    double sum = 0.0;
    for (std::size_t index = 0; index < retained; ++index)
        sum += squaredDistances[index];
    quality.robust_rmse_m =
        std::sqrt(sum / static_cast<double>(retained));
    return quality;
}

std::vector<double> symmetricSamples(double maximum, double step)
{
    std::vector<double> samples{0.0};
    if (!(maximum > 0.0) || !(step > 0.0))
        return samples;
    for (double value = step; value < maximum - 1.0e-9; value += step) {
        samples.push_back(-value);
        samples.push_back(value);
    }
    samples.push_back(-maximum);
    samples.push_back(maximum);
    std::sort(samples.begin(), samples.end());
    return samples;
}

Eigen::Matrix4f findPlanarInitialGuess(
    const RegistrationCloud::Ptr &source,
    const RegistrationCloud::Ptr &target,
    const Options &options)
{
    Eigen::Matrix4f bestGuess = Eigen::Matrix4f::Identity();
    if (source->empty() || target->empty()
        || !(options.initial_search_max_distance_m > 0.0)) {
        return bestGuess;
    }

    const RegistrationCloud::Ptr searchSource = capped(
        source, options.max_initial_search_points);
    const RegistrationCloud::Ptr searchTarget = capped(
        target, options.max_initial_search_points);
    pcl::KdTreeFLANN<RegistrationPoint> targetTree;
    targetTree.setInputCloud(searchTarget);

    const std::vector<double> translations = symmetricSamples(
        options.max_correction_translation_m,
        options.initial_search_translation_step_m);
    const double searchMaxYaw = options.allow_yaw_correction
        ? options.initial_search_max_yaw_deg
        : 0.0;
    const std::vector<double> yaws = symmetricSamples(
        searchMaxYaw,
        options.initial_search_yaw_step_deg);
    struct Candidate {
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        DirectionalQuality source_to_target;
        double directional_score = std::numeric_limits<double>::infinity();
        double normalized_correction = 0.0;
    };
    std::vector<Candidate> candidates;
    for (const double yawDegrees : yaws) {
        const double yaw = yawDegrees / kRadiansToDegrees;
        const float cosine = static_cast<float>(std::cos(yaw));
        const float sine = static_cast<float>(std::sin(yaw));
        for (const double x : translations) {
            for (const double y : translations) {
                const double translationNorm = std::hypot(x, y);
                if (translationNorm
                    > options.max_correction_translation_m + 1.0e-9) {
                    continue;
                }
                Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
                guess(0, 0) = cosine;
                guess(0, 1) = -sine;
                guess(1, 0) = sine;
                guess(1, 1) = cosine;
                guess(0, 3) = static_cast<float>(x);
                guess(1, 3) = static_cast<float>(y);
                RegistrationCloud::Ptr transformed(new RegistrationCloud);
                pcl::transformPointCloud(*searchSource, *transformed, guess);
                const DirectionalQuality quality = directionalQuality(
                    transformed, targetTree,
                    options.initial_search_max_distance_m);
                if (!std::isfinite(quality.robust_rmse_m))
                    continue;
                const double normalizedTranslation =
                    options.max_correction_translation_m > 0.0
                        ? translationNorm
                              / options.max_correction_translation_m
                        : 0.0;
                const double normalizedYaw =
                    searchMaxYaw > 0.0
                        ? std::abs(yawDegrees)
                              / searchMaxYaw
                        : 0.0;
                const double score = quality.robust_rmse_m
                    + options.initial_search_max_distance_m
                          * (1.0 - quality.match_ratio)
                    + 0.01 * (normalizedTranslation + normalizedYaw);
                Candidate candidate;
                candidate.guess = guess;
                candidate.source_to_target = quality;
                candidate.directional_score = score;
                candidate.normalized_correction =
                    normalizedTranslation + normalizedYaw;
                candidates.push_back(std::move(candidate));
            }
        }
    }
    if (candidates.empty())
        return bestGuess;

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &left, const Candidate &right) {
                  return left.directional_score < right.directional_score;
              });
    const std::size_t finalistCount = std::min(
        candidates.size(),
        static_cast<std::size_t>(
            std::max(1, options.max_initial_search_finalists)));
    struct ScoredCandidate {
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        double score = std::numeric_limits<double>::infinity();
    };
    std::vector<ScoredCandidate> scoredCandidates;
    scoredCandidates.reserve(finalistCount);
    for (std::size_t index = 0; index < finalistCount; ++index) {
        const Candidate &candidate = candidates[index];
        RegistrationCloud::Ptr transformed(new RegistrationCloud);
        pcl::transformPointCloud(*searchSource, *transformed,
                                 candidate.guess);
        const DirectionalQuality targetToSource = directionalQuality(
            searchTarget, transformed,
            options.initial_search_max_distance_m);
        if (!std::isfinite(targetToSource.robust_rmse_m))
            continue;
        const double robustRmse = std::sqrt(
            0.5 * (candidate.source_to_target.robust_rmse_m
                       * candidate.source_to_target.robust_rmse_m
                   + targetToSource.robust_rmse_m
                       * targetToSource.robust_rmse_m));
        const double averageMatchRatio = 0.5
            * (candidate.source_to_target.match_ratio
               + targetToSource.match_ratio);
        const double score = robustRmse
            + options.initial_search_max_distance_m
                  * (1.0 - averageMatchRatio)
            + 0.01 * candidate.normalized_correction;
        scoredCandidates.push_back({candidate.guess, score});
    }
    if (scoredCandidates.empty())
        return bestGuess;
    std::sort(scoredCandidates.begin(), scoredCandidates.end(),
              [](const ScoredCandidate &left,
                 const ScoredCandidate &right) {
                  return left.score < right.score;
              });
    bestGuess = scoredCandidates.front().guess;
    double bestScore = scoredCandidates.front().score;

    const std::size_t refinementCount = std::min(
        scoredCandidates.size(),
        static_cast<std::size_t>(
            std::max(1, options.max_initial_search_refinements)));
    const std::array<double, 3> localOffsets{{-0.5, 0.0, 0.5}};
    for (std::size_t candidateIndex = 0;
         candidateIndex < refinementCount; ++candidateIndex) {
        const Eigen::Matrix4f &coarseGuess =
            scoredCandidates[candidateIndex].guess;
        const double coarseX = coarseGuess(0, 3);
        const double coarseY = coarseGuess(1, 3);
        const double coarseYawDegrees = std::atan2(
            static_cast<double>(coarseGuess(1, 0)),
            static_cast<double>(coarseGuess(0, 0))) * kRadiansToDegrees;
        for (const double xOffset : localOffsets) {
            for (const double yOffset : localOffsets) {
                const double x = coarseX
                    + xOffset * options.initial_search_translation_step_m;
                const double y = coarseY
                    + yOffset * options.initial_search_translation_step_m;
                const double translationNorm = std::hypot(x, y);
                if (translationNorm
                    > options.max_correction_translation_m + 1.0e-9) {
                    continue;
                }
                for (const double yawOffset : localOffsets) {
                    if (!options.allow_yaw_correction
                        && std::abs(yawOffset) > 1.0e-9) {
                        continue;
                    }
                    const double yawDegrees = options.allow_yaw_correction
                        ? coarseYawDegrees
                              + yawOffset
                                    * options.initial_search_yaw_step_deg
                        : 0.0;
                    if (std::abs(yawDegrees)
                        > searchMaxYaw + 1.0e-9) {
                        continue;
                    }
                    const double yaw = yawDegrees / kRadiansToDegrees;
                    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
                    guess(0, 0) = static_cast<float>(std::cos(yaw));
                    guess(0, 1) = static_cast<float>(-std::sin(yaw));
                    guess(1, 0) = static_cast<float>(std::sin(yaw));
                    guess(1, 1) = static_cast<float>(std::cos(yaw));
                    guess(0, 3) = static_cast<float>(x);
                    guess(1, 3) = static_cast<float>(y);
                    RegistrationCloud::Ptr transformed(
                        new RegistrationCloud);
                    pcl::transformPointCloud(
                        *searchSource, *transformed, guess);
                    const DirectionalQuality sourceToTarget =
                        directionalQuality(
                            transformed, targetTree,
                            options.initial_search_max_distance_m);
                    const DirectionalQuality targetToSource =
                        directionalQuality(
                            searchTarget, transformed,
                            options.initial_search_max_distance_m);
                    if (!std::isfinite(sourceToTarget.robust_rmse_m)
                        || !std::isfinite(targetToSource.robust_rmse_m)) {
                        continue;
                    }
                    const double robustRmse = std::sqrt(
                        0.5 * (sourceToTarget.robust_rmse_m
                                   * sourceToTarget.robust_rmse_m
                               + targetToSource.robust_rmse_m
                                   * targetToSource.robust_rmse_m));
                    const double averageMatchRatio = 0.5
                        * (sourceToTarget.match_ratio
                           + targetToSource.match_ratio);
                    const double normalizedTranslation =
                        options.max_correction_translation_m > 0.0
                            ? translationNorm
                                  / options.max_correction_translation_m
                            : 0.0;
                    const double normalizedYaw =
                        searchMaxYaw > 0.0
                            ? std::abs(yawDegrees)
                                  / searchMaxYaw
                            : 0.0;
                    const double normalizedCorrection =
                        normalizedTranslation + normalizedYaw;
                    const double score = robustRmse
                        + options.initial_search_max_distance_m
                              * (1.0 - averageMatchRatio)
                        + 0.01 * normalizedCorrection;
                    if (score < bestScore) {
                        bestScore = score;
                        bestGuess = guess;
                    }
                }
            }
        }
    }
    return bestGuess;
}

BidirectionalQuality bidirectionalQuality(
    const RegistrationCloud::Ptr &source,
    const RegistrationCloud::Ptr &target,
    double maxDistance)
{
    const DirectionalQuality sourceToTarget =
        directionalQuality(source, target, maxDistance);
    const DirectionalQuality targetToSource =
        directionalQuality(target, source, maxDistance);
    BidirectionalQuality quality;
    quality.source_match_ratio = sourceToTarget.match_ratio;
    quality.target_match_ratio = targetToSource.match_ratio;
    if (std::isfinite(sourceToTarget.robust_rmse_m)
        && std::isfinite(targetToSource.robust_rmse_m)) {
        quality.robust_rmse_m = std::sqrt(
            0.5 * (sourceToTarget.robust_rmse_m
                       * sourceToTarget.robust_rmse_m
                   + targetToSource.robust_rmse_m
                       * targetToSource.robust_rmse_m));
        const double averageMatchRatio =
            0.5 * (quality.source_match_ratio + quality.target_match_ratio);
        quality.score = quality.robust_rmse_m
                      + maxDistance * (1.0 - averageMatchRatio);
    }
    return quality;
}

pcl::registration::TransformationEstimation2D<
    RegistrationPoint, RegistrationPoint>::Ptr planarEstimator()
{
    return pcl::registration::TransformationEstimation2D<
        RegistrationPoint, RegistrationPoint>::Ptr(
            new pcl::registration::TransformationEstimation2D<
                RegistrationPoint, RegistrationPoint>);
}

class TranslationEstimation2D final
    : public pcl::registration::TransformationEstimation<
          RegistrationPoint, RegistrationPoint> {
public:
    using Base = pcl::registration::TransformationEstimation<
        RegistrationPoint, RegistrationPoint>;
    using Matrix4 = Base::Matrix4;

    void estimateRigidTransformation(
        const RegistrationCloud &source,
        const RegistrationCloud &target,
        Matrix4 &transformation) const override
    {
        std::vector<double> deltaX;
        std::vector<double> deltaY;
        const std::size_t count = std::min(source.size(), target.size());
        deltaX.reserve(count);
        deltaY.reserve(count);
        for (std::size_t index = 0; index < count; ++index)
            appendDelta(source[index], target[index], deltaX, deltaY);
        setTranslation(deltaX, deltaY, transformation);
    }

    void estimateRigidTransformation(
        const RegistrationCloud &source,
        const pcl::Indices &sourceIndices,
        const RegistrationCloud &target,
        Matrix4 &transformation) const override
    {
        std::vector<double> deltaX;
        std::vector<double> deltaY;
        const std::size_t count = std::min(sourceIndices.size(), target.size());
        deltaX.reserve(count);
        deltaY.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            const int sourceIndex = sourceIndices[index];
            if (sourceIndex < 0
                || static_cast<std::size_t>(sourceIndex) >= source.size()) {
                continue;
            }
            appendDelta(source[static_cast<std::size_t>(sourceIndex)],
                        target[index], deltaX, deltaY);
        }
        setTranslation(deltaX, deltaY, transformation);
    }

    void estimateRigidTransformation(
        const RegistrationCloud &source,
        const pcl::Indices &sourceIndices,
        const RegistrationCloud &target,
        const pcl::Indices &targetIndices,
        Matrix4 &transformation) const override
    {
        std::vector<double> deltaX;
        std::vector<double> deltaY;
        const std::size_t count = std::min(sourceIndices.size(),
                                           targetIndices.size());
        deltaX.reserve(count);
        deltaY.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            const int sourceIndex = sourceIndices[index];
            const int targetIndex = targetIndices[index];
            if (sourceIndex < 0 || targetIndex < 0
                || static_cast<std::size_t>(sourceIndex) >= source.size()
                || static_cast<std::size_t>(targetIndex) >= target.size()) {
                continue;
            }
            appendDelta(source[static_cast<std::size_t>(sourceIndex)],
                        target[static_cast<std::size_t>(targetIndex)],
                        deltaX, deltaY);
        }
        setTranslation(deltaX, deltaY, transformation);
    }

    void estimateRigidTransformation(
        const RegistrationCloud &source,
        const RegistrationCloud &target,
        const pcl::Correspondences &correspondences,
        Matrix4 &transformation) const override
    {
        std::vector<double> deltaX;
        std::vector<double> deltaY;
        deltaX.reserve(correspondences.size());
        deltaY.reserve(correspondences.size());
        for (const pcl::Correspondence &correspondence : correspondences) {
            if (correspondence.index_query < 0
                || correspondence.index_match < 0
                || static_cast<std::size_t>(correspondence.index_query)
                       >= source.size()
                || static_cast<std::size_t>(correspondence.index_match)
                       >= target.size()) {
                continue;
            }
            appendDelta(
                source[static_cast<std::size_t>(correspondence.index_query)],
                target[static_cast<std::size_t>(correspondence.index_match)],
                deltaX, deltaY);
        }
        setTranslation(deltaX, deltaY, transformation);
    }

private:
    static void appendDelta(const RegistrationPoint &source,
                            const RegistrationPoint &target,
                            std::vector<double> &deltaX,
                            std::vector<double> &deltaY)
    {
        deltaX.push_back(static_cast<double>(target.x - source.x));
        deltaY.push_back(static_cast<double>(target.y - source.y));
    }

    static void setTranslation(const std::vector<double> &deltaX,
                               const std::vector<double> &deltaY,
                               Matrix4 &transformation)
    {
        transformation.setIdentity();
        if (deltaX.empty() || deltaY.empty())
            return;
        transformation(0, 3) = static_cast<float>(quantile(deltaX, 0.5));
        transformation(1, 3) = static_cast<float>(quantile(deltaY, 0.5));
    }
};

using TransformationEstimator =
    pcl::registration::TransformationEstimation<
        RegistrationPoint, RegistrationPoint>;

TransformationEstimator::Ptr registrationEstimator(const Options &options)
{
    if (options.allow_yaw_correction)
        return planarEstimator();
    return TransformationEstimator::Ptr(new TranslationEstimation2D);
}

bool estimateIcpCorrection(const slam_map_io::MapArchive &targetArchive,
                           const slam_map_io::MapArchive &sourceArchive,
                           const Eigen::Isometry3d &coarse,
                           const Options &options,
                           Eigen::Isometry3d &correction,
                           Diagnostic &diagnostic)
{
    const RegistrationSelection targetSelection =
        buildRegistrationWorldCloud(
            targetArchive, Eigen::Isometry3d::Identity(), options);
    const RegistrationSelection sourceSelection =
        buildRegistrationWorldCloud(sourceArchive, coarse, options);
    const RegistrationCloud::Ptr targetWorld = targetSelection.cloud;
    const RegistrationCloud::Ptr sourceWorld = sourceSelection.cloud;
    diagnostic.target_registration_points =
        static_cast<int>(targetWorld->points.size());
    diagnostic.source_registration_points =
        static_cast<int>(sourceWorld->points.size());
    diagnostic.target_used_stable_points =
        targetSelection.used_stable_points;
    diagnostic.source_used_stable_points =
        sourceSelection.used_stable_points;
    const RegistrationCloud::Ptr targetCoarse = voxelDownsample(
        targetWorld, options.coarse_voxel_size_m);
    const RegistrationCloud::Ptr sourceCoarse = voxelDownsample(
        sourceWorld, options.coarse_voxel_size_m);
    if (targetCoarse->empty() || sourceCoarse->empty()) {
        diagnostic.message = QStringLiteral("ICP 回退：降采样后点云为空");
        return false;
    }

    const Bounds2d targetBounds = boundsOf(targetCoarse);
    const Bounds2d sourceBounds = boundsOf(sourceCoarse);
    const double trueMinX = std::max(targetBounds.min_x, sourceBounds.min_x);
    const double trueMaxX = std::min(targetBounds.max_x, sourceBounds.max_x);
    const double trueMinY = std::max(targetBounds.min_y, sourceBounds.min_y);
    const double trueMaxY = std::min(targetBounds.max_y, sourceBounds.max_y);
    if (trueMinX >= trueMaxX || trueMinY >= trueMaxY) {
        diagnostic.message = QStringLiteral(
            "ICP 回退：地图在 XY 平面无真实重叠区（padding 不参与重叠判定）");
        return false;
    }
    diagnostic.actual_overlap_area_m2 =
        (trueMaxX - trueMinX) * (trueMaxY - trueMinY);

    const double minX = trueMinX - options.overlap_padding_m;
    const double maxX = trueMaxX + options.overlap_padding_m;
    const double minY = trueMinY - options.overlap_padding_m;
    const double maxY = trueMaxY + options.overlap_padding_m;
    RegistrationCloud::Ptr coarseTargetOverlap = extractOverlap(
        targetCoarse, minX, maxX, minY, maxY);
    RegistrationCloud::Ptr coarseSourceOverlap = extractOverlap(
        sourceCoarse, minX, maxX, minY, maxY);
    diagnostic.target_overlap_points =
        static_cast<int>(coarseTargetOverlap->points.size());
    diagnostic.source_overlap_points =
        static_cast<int>(coarseSourceOverlap->points.size());
    if (diagnostic.target_overlap_points < options.min_overlap_points
        || diagnostic.source_overlap_points < options.min_overlap_points) {
        diagnostic.message = QStringLiteral(
            "ICP 回退：真实重叠点不足（基准 %1，来源 %2，面积 %3 m²）")
                                 .arg(diagnostic.target_overlap_points)
                                 .arg(diagnostic.source_overlap_points)
                                 .arg(diagnostic.actual_overlap_area_m2,
                                      0, 'f', 1);
        return false;
    }
    coarseTargetOverlap = capped(
        coarseTargetOverlap, options.max_registration_points);
    coarseSourceOverlap = capped(
        coarseSourceOverlap, options.max_registration_points);

    pcl::IterativeClosestPoint<RegistrationPoint, RegistrationPoint> coarseIcp;
    coarseIcp.setTransformationEstimation(registrationEstimator(options));
    coarseIcp.setInputTarget(coarseTargetOverlap);
    coarseIcp.setInputSource(coarseSourceOverlap);
    coarseIcp.setMaximumIterations(options.coarse_max_iterations);
    coarseIcp.setMaxCorrespondenceDistance(
        options.coarse_max_correspondence_m);
    const Eigen::Matrix4f initialGuess = findPlanarInitialGuess(
        coarseSourceOverlap, coarseTargetOverlap, options);
    RegistrationCloud coarseAligned;
    coarseIcp.align(coarseAligned, initialGuess);
    if (!coarseIcp.hasConverged()) {
        diagnostic.message = QStringLiteral("ICP 回退：粗级 ICP 未收敛");
        return false;
    }

    RegistrationCloud::Ptr fineTargetOverlap = extractOverlap(
        voxelDownsample(targetWorld, options.voxel_size_m),
        minX, maxX, minY, maxY);
    RegistrationCloud::Ptr fineSourceOverlap = extractOverlap(
        voxelDownsample(sourceWorld, options.voxel_size_m),
        minX, maxX, minY, maxY);
    fineTargetOverlap = capped(
        fineTargetOverlap, options.max_registration_points);
    fineSourceOverlap = capped(
        fineSourceOverlap, options.max_registration_points);
    if (fineTargetOverlap->empty() || fineSourceOverlap->empty()) {
        diagnostic.message = QStringLiteral("ICP 回退：细级重叠点云为空");
        return false;
    }

    const BidirectionalQuality anchorQuality = bidirectionalQuality(
        fineSourceOverlap, fineTargetOverlap,
        options.quality_max_distance_m);
    RegistrationCloud::Ptr sourceAfterCoarse(new RegistrationCloud);
    pcl::transformPointCloud(*fineSourceOverlap, *sourceAfterCoarse,
                             coarseIcp.getFinalTransformation());

    pcl::IterativeClosestPoint<RegistrationPoint, RegistrationPoint> fineIcp;
    fineIcp.setTransformationEstimation(registrationEstimator(options));
    fineIcp.setInputTarget(fineTargetOverlap);
    fineIcp.setInputSource(sourceAfterCoarse);
    fineIcp.setMaximumIterations(options.max_iterations);
    fineIcp.setMaxCorrespondenceDistance(options.max_correspondence_m);
    RegistrationCloud fineAligned;
    fineIcp.align(fineAligned);

    diagnostic.icp_converged = fineIcp.hasConverged();
    diagnostic.fitness_m2 =
        fineIcp.getFitnessScore(options.max_correspondence_m);
    const Eigen::Matrix4f matrix =
        fineIcp.getFinalTransformation()
        * coarseIcp.getFinalTransformation();
    RegistrationCloud::Ptr sourceAfterFinal(new RegistrationCloud);
    pcl::transformPointCloud(*fineSourceOverlap, *sourceAfterFinal, matrix);
    const BidirectionalQuality finalQuality = bidirectionalQuality(
        sourceAfterFinal, fineTargetOverlap,
        options.quality_max_distance_m);

    diagnostic.anchor_robust_rmse_m = anchorQuality.robust_rmse_m;
    diagnostic.final_robust_rmse_m = finalQuality.robust_rmse_m;
    diagnostic.anchor_quality_score = anchorQuality.score;
    diagnostic.final_quality_score = finalQuality.score;
    diagnostic.source_match_ratio = finalQuality.source_match_ratio;
    diagnostic.target_match_ratio = finalQuality.target_match_ratio;
    const double absoluteImprovement =
        anchorQuality.score - finalQuality.score;
    diagnostic.quality_improvement_ratio =
        std::isfinite(anchorQuality.score) && anchorQuality.score > 1e-9
            ? absoluteImprovement / anchorQuality.score
            : 0.0;
    diagnostic.correction_translation_m = std::hypot(
        static_cast<double>(matrix(0, 3)),
        static_cast<double>(matrix(1, 3)));
    diagnostic.correction_yaw_deg = std::atan2(
        static_cast<double>(matrix(1, 0)),
        static_cast<double>(matrix(0, 0))) * kRadiansToDegrees;

    QStringList rejectionReasons;
    if (!diagnostic.icp_converged)
        rejectionReasons.append(QStringLiteral("细级未收敛"));
    if (!std::isfinite(finalQuality.robust_rmse_m)
        || finalQuality.robust_rmse_m > options.max_robust_rmse_m) {
        rejectionReasons.append(QStringLiteral("双向 RMSE 超限"));
    }
    if (finalQuality.source_match_ratio
            < options.min_bidirectional_match_ratio
        || finalQuality.target_match_ratio
            < options.min_bidirectional_match_ratio) {
        rejectionReasons.append(QStringLiteral("双向匹配率不足"));
    }
    if (!(absoluteImprovement
              >= options.min_absolute_quality_improvement
          || diagnostic.quality_improvement_ratio
              >= options.min_relative_quality_improvement)) {
        rejectionReasons.append(QStringLiteral("相对锚点无有效改善"));
    }
    if (!std::isfinite(diagnostic.correction_translation_m)
        || diagnostic.correction_translation_m
               > options.max_correction_translation_m) {
        rejectionReasons.append(QStringLiteral("平移修正超限"));
    }
    const double safeTranslationLimit = std::max(
        0.0, options.max_correction_translation_m
                 - std::max(0.0, options.correction_boundary_margin_m));
    if (options.correction_boundary_margin_m > 0.0
        && options.max_correction_translation_m
               > options.correction_boundary_margin_m
        && std::isfinite(diagnostic.correction_translation_m)
        && diagnostic.correction_translation_m >= safeTranslationLimit) {
        rejectionReasons.append(QStringLiteral("平移候选贴近搜索边界"));
    }
    if (!std::isfinite(diagnostic.correction_yaw_deg)
        || std::abs(diagnostic.correction_yaw_deg)
               > options.max_correction_yaw_deg) {
        rejectionReasons.append(QStringLiteral("偏航修正超限"));
    }
    if (!options.allow_yaw_correction
        && std::abs(diagnostic.correction_yaw_deg) > 1.0e-4) {
        rejectionReasons.append(QStringLiteral("默认模式禁止改变 ENU 航向"));
    }

    const QString metrics = QStringLiteral(
        "模式=%1，稳定点=%2/%3（来源/基准），锚点RMSE=%4 m，最终RMSE=%5 m，"
        "匹配率=%6/%7，改善=%8%，平移=%9 m，偏航=%10°")
                                .arg(options.allow_yaw_correction
                                         ? QStringLiteral("平移+偏航")
                                         : QStringLiteral("仅平移"))
                                .arg(diagnostic.source_used_stable_points
                                         ? diagnostic.source_registration_points
                                         : 0)
                                .arg(diagnostic.target_used_stable_points
                                         ? diagnostic.target_registration_points
                                         : 0)
                                .arg(diagnostic.anchor_robust_rmse_m,
                                     0, 'f', 3)
                                .arg(diagnostic.final_robust_rmse_m,
                                     0, 'f', 3)
                                .arg(diagnostic.source_match_ratio * 100.0,
                                     0, 'f', 1)
                                .arg(diagnostic.target_match_ratio * 100.0,
                                     0, 'f', 1)
                                .arg(diagnostic.quality_improvement_ratio
                                         * 100.0,
                                     0, 'f', 1)
                                .arg(diagnostic.correction_translation_m,
                                     0, 'f', 3)
                                .arg(diagnostic.correction_yaw_deg,
                                     0, 'f', 3);
    if (!rejectionReasons.isEmpty()) {
        diagnostic.message = QStringLiteral("ICP 回退：%1；%2")
                                 .arg(rejectionReasons.join(
                                          QStringLiteral("、")),
                                      metrics);
        return false;
    }

    correction = Eigen::Isometry3d::Identity();
    const double yaw = diagnostic.correction_yaw_deg / kRadiansToDegrees;
    correction.linear() = Eigen::AngleAxisd(
        yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    correction.translation() = Eigen::Vector3d(
        matrix(0, 3), matrix(1, 3), 0.0);
    diagnostic.message = QStringLiteral("ICP 修正已接受：%1").arg(metrics);
    return true;
}

double normalizeBerthAngle(double angle)
{
    while (angle >= 90.0)
        angle -= 180.0;
    while (angle < -90.0)
        angle += 180.0;
    return angle;
}

double berthAngleDelta(double angle, double reference)
{
    return normalizeBerthAngle(angle - reference);
}

bool similarMapBerth(const usv::SlamMapBerth &left,
                     const usv::SlamMapBerth &right)
{
    if (left.kind != right.kind)
        return false;
    if (std::hypot(left.x - right.x, left.y - right.y) > 1.5)
        return false;
    if (std::abs(left.width - right.width)
            > std::max(2.0, 0.35 * std::abs(right.width))
        || std::abs(left.length - right.length)
            > std::max(2.0, 0.35 * std::abs(right.length))) {
        return false;
    }
    return std::abs(berthAngleDelta(left.angle_deg, right.angle_deg)) <= 20.0;
}

void appendBerths(Result &result,
                  const slam_map_io::MapArchive &source,
                  const Eigen::Isometry3d &baseFromSource)
{
    const double yawDeg = std::atan2(
        baseFromSource.linear()(1, 0), baseFromSource.linear()(0, 0))
        * 180.0 / kRadiansToDegrees;
    for (const usv::SlamMapBerth &sourceBerth : source.berths) {
        if (!std::isfinite(sourceBerth.x)
            || !std::isfinite(sourceBerth.y)
            || !std::isfinite(sourceBerth.z)
            || !std::isfinite(sourceBerth.width)
            || !std::isfinite(sourceBerth.length)
            || !std::isfinite(sourceBerth.angle_deg)
            || sourceBerth.width <= 1.0e-3
            || sourceBerth.length <= 1.0e-3) {
            continue;
        }
        const Eigen::Vector3d world = baseFromSource * Eigen::Vector3d(
            sourceBerth.x, sourceBerth.y, sourceBerth.z);
        usv::SlamMapBerth transformed = sourceBerth;
        transformed.x = world.x();
        transformed.y = world.y();
        transformed.z = world.z();
        int opening_edge = static_cast<int>(sourceBerth.opening_edge);
        transformed.angle_deg =
            slam_map_berth::normalizeAngleAndOpeningEdge(
                sourceBerth.angle_deg + yawDeg, opening_edge);
        transformed.opening_edge = static_cast<int8_t>(opening_edge);

        usv::SlamMapBerth *match = nullptr;
        for (usv::SlamMapBerth &existing : result.archive.berths) {
            if (similarMapBerth(existing, transformed)) {
                match = &existing;
                break;
            }
        }
        if (!match) {
            transformed.id = static_cast<uint64_t>(
                result.archive.berths.size() + 1);
            result.archive.berths.push_back(std::move(transformed));
            continue;
        }

        const uint64_t existingCount = std::max<uint32_t>(
            1, match->observation_count);
        const uint64_t incomingCount = std::max<uint32_t>(
            1, transformed.observation_count);
        const uint64_t totalCount = existingCount + incomingCount;
        const double alpha = static_cast<double>(incomingCount)
            / static_cast<double>(totalCount);
        match->x += (transformed.x - match->x) * alpha;
        match->y += (transformed.y - match->y) * alpha;
        match->z += (transformed.z - match->z) * alpha;
        match->width += (transformed.width - match->width) * alpha;
        match->length += (transformed.length - match->length) * alpha;
        match->angle_deg = normalizeBerthAngle(
            match->angle_deg
            + berthAngleDelta(transformed.angle_deg, match->angle_deg)
                * alpha);
        match->timestamp = std::max(match->timestamp, transformed.timestamp);
        if (transformed.opening_edge >= 0)
            match->opening_edge = transformed.opening_edge;
        match->confidence = std::max(match->confidence,
                                     transformed.confidence);
        match->observation_count = totalCount > std::numeric_limits<uint32_t>::max()
            ? std::numeric_limits<uint32_t>::max()
            : static_cast<uint32_t>(totalCount);
    }
}

int appendArchive(Result &result,
                  const slam_map_io::MapArchive &source,
                  const Eigen::Isometry3d &baseFromSource,
                  int sourceIndex,
                  double minimumTimestampExclusive =
                      -std::numeric_limits<double>::infinity())
{
    SourceRange range;
    range.first_keyframe = static_cast<int>(result.archive.keyframes.size());
    range.display_intensity =
        kSourceIntensities[static_cast<std::size_t>(sourceIndex)
                           % kSourceIntensities.size()];
    for (const usv::SlamKeyframe &sourceKeyframe : source.keyframes) {
        if (sourceKeyframe.timestamp <= minimumTimestampExclusive)
            continue;
        usv::SlamKeyframe keyframe = sourceKeyframe;
        keyframe.keyframe_id =
            static_cast<uint64_t>(result.archive.keyframes.size());
        keyframe.pose = baseFromSource * sourceKeyframe.pose;
        result.archive.keyframes.push_back(std::move(keyframe));
    }
    range.keyframe_count = static_cast<int>(result.archive.keyframes.size())
                         - range.first_keyframe;
    if (range.keyframe_count > 0)
        appendBerths(result, source, baseFromSource);
    result.source_ranges.append(range);
    return range.keyframe_count;
}

}  // namespace

Result stitch(const QVector<Input> &inputs, const Options &options)
{
    Result result;
    if (inputs.isEmpty()) {
        result.error = QStringLiteral("没有可拼接的 SLAM 地图");
        return result;
    }

    for (const Input &input : inputs) {
        if (!validArchive(input.archive)) {
            result.error = QStringLiteral("地图为空或包含无效关键帧: %1")
                               .arg(input.source_path);
            return result;
        }
    }
    if (inputs.size() > 1) {
        for (const Input &input : inputs) {
            if (!slam_map_geo::validAnchor(input.archive.anchor)) {
                result.error = QStringLiteral("多地图拼接需要每张地图都包含有效地理锚点: %1")
                                   .arg(input.source_path);
                return result;
            }
        }
    }

    QVector<Input> orderedInputs = inputs;
    std::stable_sort(orderedInputs.begin(), orderedInputs.end(),
                     [](const Input &left, const Input &right) {
                         return timeBoundsOf(left.archive).first
                              < timeBoundsOf(right.archive).first;
                     });

    result.archive.anchor = orderedInputs.first().archive.anchor;
    Diagnostic baseDiagnostic;
    baseDiagnostic.source_path = orderedInputs.first().source_path;
    baseDiagnostic.mode = AlignmentMode::Base;
    baseDiagnostic.message = QStringLiteral("基准地图");
    baseDiagnostic.appended_keyframes = appendArchive(
        result, orderedInputs.first().archive,
        Eigen::Isometry3d::Identity(), 0);
    result.diagnostics.append(baseDiagnostic);

    for (int index = 1; index < orderedInputs.size(); ++index) {
        const Input &input = orderedInputs[index];
        Diagnostic diagnostic;
        diagnostic.source_path = input.source_path;
        QString geoError;
        if (!slam_map_geo::anchorOffsetEnu(
                result.archive.anchor, input.archive.anchor,
                diagnostic.anchor_offset_enu, &geoError)) {
            result.error = QStringLiteral("地图地理粗对齐失败: %1 (%2)")
                               .arg(input.source_path, geoError);
            return result;
        }

        Eigen::Isometry3d baseFromSource = Eigen::Isometry3d::Identity();
        baseFromSource.translation() = diagnostic.anchor_offset_enu;
        diagnostic.mode = AlignmentMode::AnchorOnly;
        const TimeBounds targetTime = timeBoundsOf(result.archive);
        const TimeBounds sourceTime = timeBoundsOf(input.archive);
        diagnostic.temporal_overlap_s = std::max(
            0.0, std::min(targetTime.last, sourceTime.last)
                     - std::max(targetTime.first, sourceTime.first));
        if (options.deduplicate_temporal_overlap
            && diagnostic.temporal_overlap_s
                   >= options.min_temporal_overlap_s) {
            if (sourceTime.last <= targetTime.last) {
                diagnostic.mode = AlignmentMode::DuplicateSkipped;
                diagnostic.trimmed_keyframes =
                    static_cast<int>(input.archive.keyframes.size());
                diagnostic.appended_keyframes = 0;
                diagnostic.message = QStringLiteral(
                    "时间范围已被现有地图覆盖：重叠=%1 s，跳过重复关键帧 %2 帧")
                                         .arg(diagnostic.temporal_overlap_s,
                                              0, 'f', 1)
                                         .arg(diagnostic.trimmed_keyframes);
                appendArchive(result, input.archive, baseFromSource, index,
                              std::numeric_limits<double>::infinity());
                result.diagnostics.append(diagnostic);
                continue;
            }

            Eigen::Isometry3d temporalCorrection =
                Eigen::Isometry3d::Identity();
            diagnostic.trimmed_keyframes = static_cast<int>(std::count_if(
                input.archive.keyframes.begin(),
                input.archive.keyframes.end(),
                [targetTime](const usv::SlamKeyframe &keyframe) {
                    return keyframe.timestamp <= targetTime.last;
                }));
            diagnostic.appended_keyframes =
                static_cast<int>(input.archive.keyframes.size())
                - diagnostic.trimmed_keyframes;
            if (options.enable_temporal_seam
                && estimateTemporalCorrection(
                    result.archive, input.archive, baseFromSource,
                    targetTime.last, options, temporalCorrection,
                    diagnostic)) {
                baseFromSource = temporalCorrection * baseFromSource;
                diagnostic.mode = AlignmentMode::TemporalSeam;
                diagnostic.appended_keyframes = appendArchive(
                    result, input.archive, baseFromSource, index,
                    targetTime.last);
                result.diagnostics.append(diagnostic);
                continue;
            }

            // 两张地图均已使用 ENU。时间重叠只负责去掉同一时刻的重复
            // 关键帧，新增尾部仍严格使用经纬度锚点换算出的平移，不把
            // 独立 SLAM 轨迹误差变成整张地图的二次旋转/平移。
            diagnostic.mode = AlignmentMode::AnchorOnly;
            diagnostic.correction_translation_m = 0.0;
            diagnostic.correction_yaw_deg = 0.0;
            diagnostic.appended_keyframes = appendArchive(
                result, input.archive, baseFromSource, index,
                targetTime.last);
            diagnostic.message = QStringLiteral(
                "ENU 锚点直接拼接：偏移=(%1,%2,%3) m，时间重叠=%4 s，"
                "仅裁剪重复关键帧 %5 帧，追加 %6 帧；未施加 ICP/时间接缝修正")
                                     .arg(diagnostic.anchor_offset_enu.x(), 0, 'f', 3)
                                     .arg(diagnostic.anchor_offset_enu.y(), 0, 'f', 3)
                                     .arg(diagnostic.anchor_offset_enu.z(), 0, 'f', 3)
                                     .arg(diagnostic.temporal_overlap_s, 0, 'f', 1)
                                     .arg(diagnostic.trimmed_keyframes)
                                     .arg(diagnostic.appended_keyframes);
            result.diagnostics.append(diagnostic);
            continue;
        }
        if (options.enable_icp) {
            Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
            if (estimateIcpCorrection(result.archive, input.archive,
                                      baseFromSource, options,
                                      correction, diagnostic)) {
                baseFromSource = correction * baseFromSource;
                diagnostic.mode = AlignmentMode::AnchorAndIcp;
            }
        } else {
            diagnostic.message = QStringLiteral(
                "ENU 锚点直接拼接：偏移=(%1,%2,%3) m；ICP 已禁用")
                                     .arg(diagnostic.anchor_offset_enu.x(), 0, 'f', 3)
                                     .arg(diagnostic.anchor_offset_enu.y(), 0, 'f', 3)
                                     .arg(diagnostic.anchor_offset_enu.z(), 0, 'f', 3);
        }
        diagnostic.appended_keyframes = appendArchive(
            result, input.archive, baseFromSource, index);
        result.diagnostics.append(diagnostic);
    }

    result.success = true;
    return result;
}

}  // namespace slam_map_stitch

#include "WaterSurfaceFilter.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <unordered_map>
#include <utility>

namespace usv {

WaterSurfaceFilter::WaterSurfaceFilter(const WaterSurfaceFilterConfig &config)
{
    setConfig(config);
}

void WaterSurfaceFilter::setConfig(const WaterSurfaceFilterConfig &config)
{
    config_ = config;
    // A negative clearance keeps the fitted water-surface band while still
    // rejecting points that are clearly below the surface.
    config_.clearance_m = std::clamp(config_.clearance_m, -5.0, 5.0);
    config_.plane_distance_m =
        std::clamp(config_.plane_distance_m, 0.01, 1.0);
    config_.max_tilt_deg = std::clamp(config_.max_tilt_deg, 1.0, 45.0);
    config_.min_inliers = std::max<std::size_t>(config_.min_inliers, 3);
    config_.max_sample_points =
        std::max(config_.max_sample_points, config_.min_inliers);
    config_.ransac_iterations =
        std::clamp(config_.ransac_iterations, 10, 2000);
    config_.near_surface_band_m =
        std::clamp(config_.near_surface_band_m, 0.0, 5.0);
    config_.outlier_radius_m =
        std::clamp(config_.outlier_radius_m, 0.05, 5.0);
    config_.outlier_min_neighbors =
        std::clamp(config_.outlier_min_neighbors, 0, 100);
    config_.max_fallback_frames =
        std::clamp(config_.max_fallback_frames, 0, 1000);
    reset();
}

void WaterSurfaceFilter::reset()
{
    last_plane_ = Plane{};
    fallback_frames_ = 0;
}

WaterSurfaceFilterResult WaterSurfaceFilter::filter(
    const std::vector<M_PointXYZI> &input)
{
    WaterSurfaceFilterResult result;
    if (!config_.enabled || input.empty()) {
        result.points = input;
        return result;
    }

    Plane active_plane;
    if (estimatePlane(input, active_plane)) {
        last_plane_ = active_plane;
        fallback_frames_ = 0;
        result.plane_detected = true;
    } else if (last_plane_.valid
               && fallback_frames_ < config_.max_fallback_frames) {
        // 浪花或遮挡可能让个别帧无法重新拟合水面；只短时沿用历史值，
        // 防止长时间使用过期平面导致码头或岸边低矮目标被误删。
        active_plane = last_plane_;
        ++fallback_frames_;
        result.used_fallback_plane = true;
    } else {
        result.points = input;
        if (fallback_frames_ >= config_.max_fallback_frames)
            last_plane_ = Plane{};
        return result;
    }

    result.water_z_at_origin =
        std::abs(active_plane.normal.z()) > 1e-9
            ? -active_plane.d / active_plane.normal.z()
            : 0.0;

    std::vector<M_PointXYZI> surface_filtered;
    surface_filtered.reserve(input.size());
    for (const M_PointXYZI &p : input) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)
            || !std::isfinite(p.z)) {
            ++result.removed_surface_or_below;
            continue;
        }

        // 法向已统一朝向 +Z，所以 signed_height 表示点高出水面的距离。
        // 小于等于 0.25 m（默认值）的点属于水面保护带或水下反射。
        const Eigen::Vector3d v(p.x, p.y, p.z);
        const double signed_height =
            active_plane.normal.dot(v) + active_plane.d;
        if (signed_height <= config_.clearance_m) {
            ++result.removed_surface_or_below;
            continue;
        }
        surface_filtered.push_back(p);
    }

    removeNearSurfaceOutliers(active_plane, surface_filtered, result);
    return result;
}

bool WaterSurfaceFilter::estimatePlane(
    const std::vector<M_PointXYZI> &input,
    Plane &plane) const
{
    std::vector<Eigen::Vector3d> sample;
    const std::size_t requested =
        std::min(input.size(), config_.max_sample_points);
    sample.reserve(requested);

    // 均匀抽样限制每帧 RANSAC 成本，同时避免只取点云头部某一台雷达。
    for (std::size_t i = 0; i < requested; ++i) {
        const std::size_t index =
            requested == input.size() ? i : (i * input.size()) / requested;
        const M_PointXYZI &p = input[index];
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            sample.emplace_back(p.x, p.y, p.z);
    }

    if (sample.size() < config_.min_inliers)
        return false;

    const double min_normal_z =
        std::cos(config_.max_tilt_deg * M_PI / 180.0);
    std::mt19937 rng(static_cast<std::uint32_t>(
        0x57534631U ^ static_cast<std::uint32_t>(sample.size())));
    std::uniform_int_distribution<std::size_t> pick(0, sample.size() - 1);

    Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
    double best_d = 0.0;
    std::size_t best_count = 0;

    for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
        const std::size_t ia = pick(rng);
        const std::size_t ib = pick(rng);
        const std::size_t ic = pick(rng);
        if (ia == ib || ia == ic || ib == ic)
            continue;

        Eigen::Vector3d normal =
            (sample[ib] - sample[ia]).cross(sample[ic] - sample[ia]);
        const double norm = normal.norm();
        if (norm < 1e-8)
            continue;
        normal /= norm;
        if (normal.z() < 0.0)
            normal = -normal;
        if (normal.z() < min_normal_z)
            continue;

        const double d = -normal.dot(sample[ia]);
        std::size_t count = 0;
        for (const Eigen::Vector3d &p : sample) {
            if (std::abs(normal.dot(p) + d) <= config_.plane_distance_m)
                ++count;
        }
        if (count > best_count) {
            best_count = count;
            best_normal = normal;
            best_d = d;
        }
    }

    if (best_count < config_.min_inliers)
        return false;

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    std::size_t inlier_count = 0;
    for (const Eigen::Vector3d &p : sample) {
        if (std::abs(best_normal.dot(p) + best_d)
            <= config_.plane_distance_m) {
            centroid += p;
            ++inlier_count;
        }
    }
    if (inlier_count < config_.min_inliers)
        return false;
    centroid /= static_cast<double>(inlier_count);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d &p : sample) {
        if (std::abs(best_normal.dot(p) + best_d)
            <= config_.plane_distance_m) {
            const Eigen::Vector3d centered = p - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;

    Eigen::Vector3d refined_normal = solver.eigenvectors().col(0).normalized();
    if (refined_normal.z() < 0.0)
        refined_normal = -refined_normal;
    if (refined_normal.z() < min_normal_z)
        return false;

    plane.normal = refined_normal;
    plane.d = -refined_normal.dot(centroid);
    plane.valid = true;
    return true;
}

void WaterSurfaceFilter::removeNearSurfaceOutliers(
    const Plane &plane,
    const std::vector<M_PointXYZI> &surface_filtered,
    WaterSurfaceFilterResult &result) const
{
    if (surface_filtered.empty()) {
        result.points.clear();
        return;
    }
    if (config_.outlier_min_neighbors <= 0
        || config_.near_surface_band_m <= 0.0) {
        result.points = surface_filtered;
        return;
    }

    struct Cell {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;
        bool operator==(const Cell &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct CellHash {
        std::size_t operator()(const Cell &cell) const
        {
            std::size_t seed = std::hash<std::int64_t>{}(cell.x);
            seed ^= std::hash<std::int64_t>{}(cell.y)
                    + 0x9e3779b9U + (seed << 6) + (seed >> 2);
            seed ^= std::hash<std::int64_t>{}(cell.z)
                    + 0x9e3779b9U + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    const double cell_size = config_.outlier_radius_m;
    const double inv_cell = 1.0 / cell_size;
    const double radius2 = cell_size * cell_size;
    auto cellFor = [inv_cell](const M_PointXYZI &p) {
        return Cell{
            static_cast<std::int64_t>(std::floor(p.x * inv_cell)),
            static_cast<std::int64_t>(std::floor(p.y * inv_cell)),
            static_cast<std::int64_t>(std::floor(p.z * inv_cell)),
        };
    };

    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells;
    cells.reserve(surface_filtered.size());
    for (std::size_t i = 0; i < surface_filtered.size(); ++i)
        cells[cellFor(surface_filtered[i])].push_back(i);

    result.points.reserve(surface_filtered.size());
    const double near_surface_max =
        config_.clearance_m + config_.near_surface_band_m;

    for (std::size_t i = 0; i < surface_filtered.size(); ++i) {
        const M_PointXYZI &p = surface_filtered[i];
        const double height =
            plane.normal.dot(Eigen::Vector3d(p.x, p.y, p.z)) + plane.d;

        // 只对紧邻水面的点做密度判断；高目标直接保留，防止远处细杆、
        // 桥梁边缘等本来就稀疏的有效结构被半径滤波误删。
        if (height > near_surface_max) {
            result.points.push_back(p);
            continue;
        }

        const Cell center = cellFor(p);
        int neighbors = 0;
        for (int dx = -1; dx <= 1 && neighbors < config_.outlier_min_neighbors;
             ++dx) {
            for (int dy = -1;
                 dy <= 1 && neighbors < config_.outlier_min_neighbors; ++dy) {
                for (int dz = -1;
                     dz <= 1 && neighbors < config_.outlier_min_neighbors;
                     ++dz) {
                    const auto it = cells.find(
                        Cell{center.x + dx, center.y + dy, center.z + dz});
                    if (it == cells.end())
                        continue;
                    for (const std::size_t candidate_index : it->second) {
                        if (candidate_index == i)
                            continue;
                        const M_PointXYZI &q =
                            surface_filtered[candidate_index];
                        const double qx = static_cast<double>(q.x) - p.x;
                        const double qy = static_cast<double>(q.y) - p.y;
                        const double qz = static_cast<double>(q.z) - p.z;
                        if (qx * qx + qy * qy + qz * qz <= radius2
                            && ++neighbors >= config_.outlier_min_neighbors) {
                            break;
                        }
                    }
                }
            }
        }

        if (neighbors >= config_.outlier_min_neighbors)
            result.points.push_back(p);
        else
            ++result.removed_near_surface_outliers;
    }
}

}  // namespace usv

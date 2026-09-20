#include "SlamMapDisplayBuilder.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace slam_map_display {

namespace {

uint8_t intensityForKeyframe(
    int keyframeIndex,
    const QVector<slam_map_stitch::SourceRange> &ranges)
{
    for (const slam_map_stitch::SourceRange &range : ranges) {
        if (keyframeIndex >= range.first_keyframe
            && keyframeIndex < range.first_keyframe + range.keyframe_count) {
            return range.display_intensity;
        }
    }
    return 90;
}

struct VoxelKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const noexcept
    {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y)
              + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int64_t>{}(key.z)
              + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct StableCellKey {
    std::int64_t x = 0;
    std::int64_t y = 0;

    bool operator==(const StableCellKey &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct StableCellKeyHash {
    std::size_t operator()(const StableCellKey &key) const noexcept
    {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y)
              + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct StableCellAccumulator {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    quint64 point_count = 0;
    int last_keyframe = -1;
    int observation_count = 0;
};

}  // namespace

PointCountStats countPointsAtResolution(
    const slam_map_io::MapArchive &archive,
    double resolution_m)
{
    PointCountStats stats;
    if (!std::isfinite(resolution_m) || resolution_m <= 0.0)
        return stats;

    quint64 totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes)
        totalPoints += keyframe.cloud.size();
    stats.source_points = totalPoints;

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    occupied.reserve(static_cast<std::size_t>(totalPoints));
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        for (const M_PointXYZI &point : keyframe.cloud) {
            const Eigen::Vector3d world = keyframe.pose
                * Eigen::Vector3d(point.x, point.y, point.z);
            if (!world.allFinite())
                continue;
            ++stats.valid_world_points;
            occupied.insert({
                static_cast<std::int64_t>(std::floor(
                    world.x() / resolution_m)),
                static_cast<std::int64_t>(std::floor(
                    world.y() / resolution_m)),
                static_cast<std::int64_t>(std::floor(
                    world.z() / resolution_m))});
        }
    }
    stats.resolution_points = static_cast<quint64>(occupied.size());
    return stats;
}

StableRegionAnalysis analyzeStableRegions(
    const slam_map_io::MapArchive &archive,
    double cell_size_m,
    int min_observations,
    int min_region_cells)
{
    StableRegionAnalysis analysis;
    analysis.cell_size_m = cell_size_m;
    analysis.min_observations = min_observations;
    analysis.min_region_cells = min_region_cells;
    if (!std::isfinite(cell_size_m) || cell_size_m <= 0.0
        || min_observations <= 0 || min_region_cells <= 0) {
        return analysis;
    }

    quint64 totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes)
        totalPoints += keyframe.cloud.size();

    const double inverseCellSize = 1.0 / cell_size_m;
    std::unordered_map<StableCellKey, StableCellAccumulator,
                       StableCellKeyHash> cells;
    cells.reserve(static_cast<std::size_t>(totalPoints / 4U + 1U));
    for (int keyframeIndex = 0;
         keyframeIndex < static_cast<int>(archive.keyframes.size());
         ++keyframeIndex) {
        const usv::SlamKeyframe &keyframe = archive.keyframes[
            static_cast<std::size_t>(keyframeIndex)];
        for (const M_PointXYZI &point : keyframe.cloud) {
            const Eigen::Vector3d world = keyframe.pose
                * Eigen::Vector3d(point.x, point.y, point.z);
            if (!world.allFinite())
                continue;
            const StableCellKey key{
                static_cast<std::int64_t>(std::floor(
                    world.x() * inverseCellSize)),
                static_cast<std::int64_t>(std::floor(
                    world.y() * inverseCellSize))};
            StableCellAccumulator &cell = cells[key];
            cell.sum_x += world.x();
            cell.sum_y += world.y();
            cell.sum_z += world.z();
            cell.min_x = std::min(cell.min_x, world.x());
            cell.max_x = std::max(cell.max_x, world.x());
            cell.min_y = std::min(cell.min_y, world.y());
            cell.max_y = std::max(cell.max_y, world.y());
            cell.min_z = std::min(cell.min_z, world.z());
            cell.max_z = std::max(cell.max_z, world.z());
            ++cell.point_count;
            if (cell.last_keyframe != keyframeIndex) {
                cell.last_keyframe = keyframeIndex;
                ++cell.observation_count;
            }
        }
    }
    analysis.candidate_cells = static_cast<quint64>(cells.size());

    std::unordered_set<StableCellKey, StableCellKeyHash> stableCells;
    stableCells.reserve(cells.size());
    for (const auto &entry : cells) {
        if (entry.second.observation_count >= min_observations)
            stableCells.insert(entry.first);
    }
    analysis.stable_cells = static_cast<quint64>(stableCells.size());

    std::unordered_set<StableCellKey, StableCellKeyHash> visited;
    visited.reserve(stableCells.size());
    std::unordered_map<StableCellKey, std::size_t, StableCellKeyHash>
        regionForCell;
    regionForCell.reserve(stableCells.size());

    for (const StableCellKey &start : stableCells) {
        if (!visited.insert(start).second)
            continue;

        std::queue<StableCellKey> pending;
        std::vector<StableCellKey> component;
        pending.push(start);
        while (!pending.empty()) {
            const StableCellKey current = pending.front();
            pending.pop();
            component.push_back(current);
            for (int deltaY = -1; deltaY <= 1; ++deltaY) {
                for (int deltaX = -1; deltaX <= 1; ++deltaX) {
                    if (deltaX == 0 && deltaY == 0)
                        continue;
                    const StableCellKey neighbour{
                        current.x + deltaX, current.y + deltaY};
                    if (stableCells.find(neighbour) != stableCells.end()
                        && visited.insert(neighbour).second) {
                        pending.push(neighbour);
                    }
                }
            }
        }

        if (static_cast<int>(component.size()) < min_region_cells) {
            ++analysis.discarded_small_regions;
            continue;
        }

        StableRegion region;
        region.stable_cells = static_cast<quint64>(component.size());
        region.min_x = std::numeric_limits<double>::infinity();
        region.max_x = -std::numeric_limits<double>::infinity();
        region.min_y = std::numeric_limits<double>::infinity();
        region.max_y = -std::numeric_limits<double>::infinity();
        region.min_z = std::numeric_limits<double>::infinity();
        region.max_z = -std::numeric_limits<double>::infinity();
        region.min_observations = std::numeric_limits<int>::max();
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        quint64 observationSum = 0;
        for (const StableCellKey &key : component) {
            const StableCellAccumulator &cell = cells.at(key);
            sumX += cell.sum_x;
            sumY += cell.sum_y;
            sumZ += cell.sum_z;
            region.point_count += cell.point_count;
            region.min_x = std::min(region.min_x, cell.min_x);
            region.max_x = std::max(region.max_x, cell.max_x);
            region.min_y = std::min(region.min_y, cell.min_y);
            region.max_y = std::max(region.max_y, cell.max_y);
            region.min_z = std::min(region.min_z, cell.min_z);
            region.max_z = std::max(region.max_z, cell.max_z);
            region.min_observations = std::min(
                region.min_observations, cell.observation_count);
            region.max_observations = std::max(
                region.max_observations, cell.observation_count);
            observationSum += static_cast<quint64>(cell.observation_count);
        }
        if (region.point_count > 0) {
            const double inversePointCount =
                1.0 / static_cast<double>(region.point_count);
            region.center_x = sumX * inversePointCount;
            region.center_y = sumY * inversePointCount;
            region.center_z = sumZ * inversePointCount;
        }
        region.estimated_area_m2 =
            static_cast<double>(region.stable_cells)
            * cell_size_m * cell_size_m;
        region.average_observations = component.empty()
            ? 0.0
            : static_cast<double>(observationSum)
                  / static_cast<double>(component.size());

        const std::size_t regionIndex =
            static_cast<std::size_t>(analysis.regions.size());
        analysis.regions.append(region);
        for (const StableCellKey &key : component)
            regionForCell.emplace(key, regionIndex);
    }

    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        std::unordered_set<std::size_t> regionsSeen;
        for (const M_PointXYZI &point : keyframe.cloud) {
            const Eigen::Vector3d world = keyframe.pose
                * Eigen::Vector3d(point.x, point.y, point.z);
            if (!world.allFinite())
                continue;
            const StableCellKey key{
                static_cast<std::int64_t>(std::floor(
                    world.x() * inverseCellSize)),
                static_cast<std::int64_t>(std::floor(
                    world.y() * inverseCellSize))};
            const auto regionIt = regionForCell.find(key);
            if (regionIt != regionForCell.end())
                regionsSeen.insert(regionIt->second);
        }
        for (const std::size_t regionIndex : regionsSeen)
            ++analysis.regions[static_cast<int>(regionIndex)]
                  .observed_keyframes;
    }

    std::sort(analysis.regions.begin(), analysis.regions.end(),
              [](const StableRegion &left, const StableRegion &right) {
                  if (left.stable_cells != right.stable_cells)
                      return left.stable_cells > right.stable_cells;
                  return left.point_count > right.point_count;
              });
    return analysis;
}

QVector<M_PointXYZI> rebuild(
    const slam_map_io::MapArchive &archive,
    const QVector<slam_map_stitch::SourceRange> &ranges,
    int max_points,
    double voxel_size_m,
    BuildStats *stats)
{
    BuildStats localStats;
    BuildStats &buildStats = stats ? *stats : localStats;
    buildStats = BuildStats{};
    QVector<M_PointXYZI> worldCloud;
    if (max_points <= 0 || archive.keyframes.empty())
        return worldCloud;

    quint64 totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes)
        totalPoints += keyframe.cloud.size();
    buildStats.source_points = totalPoints;
    if (totalPoints == 0)
        return worldCloud;

    const quint64 pointLimit = static_cast<quint64>(max_points);
    const quint64 step = totalPoints > pointLimit
        ? (totalPoints + pointLimit - 1) / pointLimit
        : 1;
    worldCloud.reserve(static_cast<int>(
        std::min<quint64>(totalPoints, pointLimit)));
    const bool deduplicate = std::isfinite(voxel_size_m)
        && voxel_size_m > 0.0;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    if (deduplicate) {
        occupied.reserve(static_cast<std::size_t>(
            std::min<quint64>(totalPoints, pointLimit)));
    }

    quint64 sourcePointIndex = 0;
    for (int keyframeIndex = 0;
         keyframeIndex < static_cast<int>(archive.keyframes.size());
         ++keyframeIndex) {
        const usv::SlamKeyframe &keyframe = archive.keyframes[
            static_cast<std::size_t>(keyframeIndex)];
        const uint8_t displayIntensity =
            intensityForKeyframe(keyframeIndex, ranges);
        for (const M_PointXYZI &point : keyframe.cloud) {
            if (sourcePointIndex++ % step != 0)
                continue;
            ++buildStats.sampled_points;
            const Eigen::Vector3d world = keyframe.pose
                * Eigen::Vector3d(point.x, point.y, point.z);
            if (!world.allFinite())
                continue;
            if (deduplicate) {
                const VoxelKey key{
                    static_cast<std::int64_t>(std::floor(
                        world.x() / voxel_size_m)),
                    static_cast<std::int64_t>(std::floor(
                        world.y() / voxel_size_m)),
                    static_cast<std::int64_t>(std::floor(
                        world.z() / voxel_size_m))};
                if (!occupied.insert(key).second) {
                    ++buildStats.duplicate_points_removed;
                    continue;
                }
            }
            M_PointXYZI output{};
            output.x = static_cast<float>(world.x());
            output.y = static_cast<float>(world.y());
            output.z = static_cast<float>(world.z());
            output.intensity = displayIntensity;
            worldCloud.append(output);
        }
    }
    buildStats.output_points = static_cast<quint64>(worldCloud.size());
    return worldCloud;
}

}  // namespace slam_map_display

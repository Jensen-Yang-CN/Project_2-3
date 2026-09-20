#pragma once

#include "common/pointxyz.h"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace usv {

struct WaterSurfaceFilterConfig {
    bool enabled = true;
    double clearance_m = 0.25;
    double plane_distance_m = 0.15;
    double max_tilt_deg = 20.0;
    std::size_t min_inliers = 80;
    std::size_t max_sample_points = 12000;
    int ransac_iterations = 160;
    double near_surface_band_m = 1.0;
    double outlier_radius_m = 0.60;
    int outlier_min_neighbors = 2;
    int max_fallback_frames = 15;
};

struct WaterSurfaceFilterResult {
    std::vector<M_PointXYZI> points;
    std::size_t removed_surface_or_below = 0;
    std::size_t removed_near_surface_outliers = 0;
    bool plane_detected = false;
    bool used_fallback_plane = false;
    double water_z_at_origin = 0.0;
};

class WaterSurfaceFilter {
public:
    explicit WaterSurfaceFilter(
        const WaterSurfaceFilterConfig &config = WaterSurfaceFilterConfig{});

    void setConfig(const WaterSurfaceFilterConfig &config);
    const WaterSurfaceFilterConfig &config() const { return config_; }
    void reset();

    WaterSurfaceFilterResult filter(const std::vector<M_PointXYZI> &input);

private:
    struct Plane {
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        double d = 0.0;
        bool valid = false;
    };

    bool estimatePlane(const std::vector<M_PointXYZI> &input,
                       Plane &plane) const;
    void removeNearSurfaceOutliers(
        const Plane &plane,
        const std::vector<M_PointXYZI> &surface_filtered,
        WaterSurfaceFilterResult &result) const;

    WaterSurfaceFilterConfig config_;
    Plane last_plane_;
    int fallback_frames_ = 0;
};

}  // namespace usv

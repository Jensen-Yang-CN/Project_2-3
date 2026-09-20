#pragma once

#include "SlamMapBinaryIO.h"

#include <QString>

#include <cstddef>

namespace slam_map_geo_correct {

struct Diagnostic {
    bool applied = false;
    std::size_t valid_references = 0;
    std::size_t rejected_references = 0;
    double median_translation_m = 0.0;
    double max_translation_m = 0.0;
    double max_yaw_deg = 0.0;
    QString message;
};

struct Result {
    slam_map_io::MapArchive archive;
    Diagnostic diagnostic;
};

// Uses per-keyframe GNSS/ENU references to remove low-frequency SLAM drift.
// At least three trustworthy references are required; otherwise the archive is
// returned unchanged. Corrections are planar so lidar height is not distorted.
Result correct(const slam_map_io::MapArchive &archive);

}  // namespace slam_map_geo_correct

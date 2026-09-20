#pragma once

namespace usv::slam_config {

// Dynamic-object preprocessing parameters imported from the source archive.
constexpr int kDynamicSorMeanK = 12;
constexpr double kDynamicSorStddev = 1.0;
constexpr double kDynamicClusterTolerance = 1.5;
constexpr int kDynamicClusterMinPoints = 20;
constexpr double kDynamicMatchDistance = 3.0;
constexpr double kDynamicBboxOverlap = 0.05;
constexpr double kDynamicSpeedThreshold = 1.0;
constexpr double kDynamicStaticMotionDistance = 0.15;
constexpr double kDynamicStaticBboxOverlap = 0.7;

}  // namespace usv::slam_config

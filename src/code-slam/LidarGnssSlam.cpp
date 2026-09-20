#include "LidarGnssSlam.h"
#include "LidarGnssSlamConfig.h"

#include "GnssInsGeoUtils.h"
#include "SlamPoseUtils.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numeric>
#include <utility>

#ifndef LIDAR_GNSS_SLAM_ENABLE_LOG
#define LIDAR_GNSS_SLAM_ENABLE_LOG 1
#endif

// Set to 1 to disable the lidar Kalman correction entirely and rely only on
// the GNSS/INS propagated pose.
#ifndef LIDAR_GNSS_SLAM_DISABLE_LIDAR_KALMAN_CORRECTION
#define LIDAR_GNSS_SLAM_DISABLE_LIDAR_KALMAN_CORRECTION 0
#endif

// Dynamic-object removal. Set to 0 to disable this preprocessing stage.
#ifndef LIDAR_GNSS_SLAM_DYNAMIC_FILTER_ENABLE
#define LIDAR_GNSS_SLAM_DYNAMIC_FILTER_ENABLE 1
#endif

namespace {

// Input buffering and minimum cloud sizes.
constexpr int kMinPoints = 100;             // Raw points required to process a frame.
constexpr int kMaxQueueSize = 20;           // Maximum pending lidar frames.
constexpr size_t kMaxOdomQueueSize = 2000;  // Maximum cached GNSS/INS samples.
constexpr size_t kMinOdomSamplesToTrim = 2; // Samples retained for interpolation.

// Point-count fallback: below the low limit, the scan is considered too weak
// for registration; above the high limit, point count no longer changes the
// legacy odometry diagnostic weight.
constexpr int kOdomWeightPointsLow = 300;
constexpr int kOdomWeightPointsHigh = 3000;
constexpr double kOdomWeightMin = 0.0;
constexpr double kOdomWeightMax = 1.0;

// NanoGICP registration parameters.
constexpr int kGicpKCorrespondences = 20;
constexpr double kGicpMaxCorrespondenceDistance = 1.0e6;
constexpr int kGicpMaxIterations = 64;
constexpr double kGicpTransformationEpsilon = 5e-4;
constexpr double kGicpFitnessEpsilon = -1.0;
constexpr int kGicpRansacIterations = 0;
constexpr double kGicpRansacOutlierThresh = 0.05;
// Reject converged but geometrically wrong registrations before they can
// corrupt the pose used by all following frames.
constexpr double kMaxAcceptedGicpRmse = 1.5;
constexpr double kMaxAcceptedGicpFitness = 3.0;
// A low-residual match with very few correspondences can still be a wrong
// local minimum. Require substantial geometric overlap before accepting it.
constexpr int kMinAcceptedGicpCorrespondences = 500;
constexpr double kMinAcceptedGicpOverlap = 0.35;
// A single bad registration must not introduce a large pose jump. The
// vertical component is especially strict because the current navigation
// stream has no usable altitude measurement and therefore cannot recover from
// a false GICP height correction.
constexpr double kMaxAcceptedGicpTranslationInnovationM = 0.75;
constexpr double kMaxAcceptedGicpVerticalInnovationM = 0.15;
// Navigation is the trusted attitude prior during turns. Reject a single
// scan that proposes a large attitude jump, even if its residual is small.
constexpr double kMaxAcceptedGicpRollPitchInnovationDeg = 2.0;
constexpr double kMaxAcceptedGicpYawInnovationDeg = 15.0;
constexpr double kMaxAppliedGicpRollPitchCorrectionDeg = 0.5;
// The fused lidar stream is about 2 Hz in the replay data.  A 0.35 s limit
// would re-anchor every frame and prevent registration from running.
constexpr double kMaxGicpReferenceAgeSec = 0.80;
constexpr double kMaxGicpOdomSyncSec = 0.30;
// A stale-reference recovery must not insert a single large navigation jump
// into the map. The following frame can establish a new local reference.
constexpr double kMaxStaleReferenceReanchorJumpM = 5.0;
constexpr bool kEnablePreRegistration = true;
constexpr size_t kDynamicRegistrationMaxPoints = 64;
constexpr double kDynamicRegistrationMaxBoxGap = 4.0;
// A confirmed dynamic track may lose geometric registration for a single
// scan when its visible surface changes. Reassociate only under a tighter
// bound than the ordinary registration prefilter.
constexpr double kConfirmedTrackRecoveryMaxBoxGap = 1.5;
constexpr double kConfirmedTrackRecoveryMinPointRatio = 0.35;
constexpr double kTentativeTrackRecoveryMaxBoxGap = 0.5;
constexpr double kTentativeTrackRecoveryMinPointRatio = 0.35;
constexpr int kTentativeRecoveryMinimumEvidenceFrames = 2;
// A visible vessel face can split into several valid clusters.  Keep the
// confirmed identity across that split, but demand a tighter world-space
// position than the ordinary association path so nearby static structure is
// not absorbed into a dynamic track.
constexpr double kConfirmedMemoryMaxBoxGap = 0.5;
constexpr double kConfirmedMemoryMinPointRatio = 0.05;
constexpr int kConfirmedMemoryFrames = 6;
constexpr double kConfirmedMemoryFragmentContainmentMargin = 0.25;

// Full error-state Kalman model: [p, rotation, linear velocity,
// angular velocity].  These are continuous-time acceleration noise values.
constexpr double kLinearAccelerationNoise = 0.6;  // m/s^2 (1-sigma)
constexpr double kAngularAccelerationNoise = 0.15; // rad/s^2 (1-sigma)
// Per-lidar-update navigation uncertainty floor; it prevents a short lidar
// interval from making the GNSS/INS prediction mathematically certain.
constexpr double kNavigationPositionVariance = 0.04;
constexpr double kNavigationAttitudeVariance = 0.01;
constexpr double kCovarianceNumericalFloor = 1.0e-9;

// GICP observation covariance parameters.  The Hessian inverse provides the
// local geometric uncertainty; the score only scales that uncertainty.
constexpr double kHessianRegularization = 1.0e-6;
// Optimistic lidar calibration: a good, well-conditioned GICP update should
// be able to obtain a gain close to one instead of being capped at 0.8.
constexpr double kMinGicpVariance = 0.001;
constexpr double kMaxGicpVariance = 4.0;
constexpr double kGicpReferencePoints = 2000.0;
constexpr double kGicpResidualVarianceFloor = 0.25;
constexpr double kGicpResidualVarianceCeiling = 4.0;
constexpr double kGicpEffectivePointsFloor = 200.0;

// Registration rejection parameters.  The Mahalanobis gate is six
// dimensional; the direction gate protects against a sideways false match.
constexpr double kInnovationMahalanobisGate = 36.0;
constexpr double kDirectionMotionMin = 0.5;
constexpr double kDirectionLateralRatio = 0.75;
constexpr double kDirectionLateralMin = 0.5;

// Numerical lower bound for frame-to-frame velocity calculations.
constexpr double kMinFilterDt = 1.0e-3;
constexpr int kDynamicConfirmationFrames = 4;
constexpr size_t kDynamicWorldHistoryFrames = 24;
constexpr int kDynamicMinimumTrackPoints = 20;
constexpr double kDynamicMinimumClusterTolerance = 1.5;
constexpr double kConsistentMotionMinSpeedMps = 0.5;
constexpr int kConsistentMotionFrames = 5;
constexpr double kMaximumTrackedRelativeSpeedMps = 5.0;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool DisableLidarKalmanCorrection() {
  return LIDAR_GNSS_SLAM_DISABLE_LIDAR_KALMAN_CORRECTION != 0;
}

std::mutex& SlamCsvMutex() {
  static std::mutex mutex;
  return mutex;
}

bool& SlamCsvFirstWrite() {
  static bool first_write = true;
  return first_write;
}

std::mutex& LeftmostClusterLogMutex() {
  static std::mutex mutex;
  return mutex;
}

std::mutex& DynamicClusterLogMutex() {
  static std::mutex mutex;
  return mutex;
}

bool& DynamicClusterLogFirstWrite() {
  static bool first_write = true;
  return first_write;
}

std::mutex& DynamicMemoryLogMutex() {
  static std::mutex mutex;
  return mutex;
}

bool& DynamicMemoryLogFirstWrite() {
  static bool first_write = true;
  return first_write;
}

void BeginDynamicClusterLogRun() {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(DynamicClusterLogMutex());
  DynamicClusterLogFirstWrite() = true;
  std::lock_guard<std::mutex> memory_lock(DynamicMemoryLogMutex());
  DynamicMemoryLogFirstWrite() = true;
#endif
}

void WriteDynamicMemoryDecisionLog(
    double timestamp, size_t point_count, const Eigen::Vector3d& centroid_world,
    uint64_t memory_track_id, int memory_remaining_frames,
    double point_ratio, double box_gap, const char* decision) {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(DynamicMemoryLogMutex());
  const std::filesystem::path path =
      std::filesystem::current_path() / "slam_dynamic_memory.csv";
  bool& first_write = DynamicMemoryLogFirstWrite();
  std::ofstream file(path, first_write ? (std::ios::out | std::ios::trunc)
                                       : (std::ios::out | std::ios::app));
  if (!file)
    return;
  if (first_write) {
    file << "timestamp,candidate_point_count,centroid_x_world_m,"
            "centroid_y_world_m,centroid_z_world_m,memory_track_id,"
            "memory_remaining_frames,point_ratio,bbox_gap_m,decision\n";
    first_write = false;
  }
  file << std::fixed << std::setprecision(9) << timestamp << ','
       << point_count << ',' << centroid_world.x() << ','
       << centroid_world.y() << ',' << centroid_world.z() << ','
       << memory_track_id << ',' << memory_remaining_frames << ','
       << point_ratio << ',' << box_gap << ',' << decision << '\n';
#else
  (void)timestamp;
  (void)point_count;
  (void)centroid_world;
  (void)memory_track_id;
  (void)memory_remaining_frames;
  (void)point_ratio;
  (void)box_gap;
  (void)decision;
#endif
}

void WriteDynamicClusterLog(
    double timestamp, uint64_t track_id, int current_index,
    int previous_index, size_t point_count, const Eigen::Vector3d& centroid,
    const Eigen::Vector3d& predicted_centroid, double match_distance,
    double bbox_overlap, double movement_residual, double movement_speed,
    int suspicious_frames,
    bool moving, bool confirmed_moving, bool removed, const char* status) {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(DynamicClusterLogMutex());
  const std::filesystem::path path =
      std::filesystem::current_path() / "slam_dynamic_clusters.csv";
  bool& first_write = DynamicClusterLogFirstWrite();
  std::ofstream file(path, first_write ? (std::ios::out | std::ios::trunc)
                                       : (std::ios::out | std::ios::app));
  if (!file)
    return;
  if (first_write) {
    file << "timestamp,track_id,current_index,previous_index,point_count,"
            "centroid_x_lidar_m,centroid_y_lidar_m,centroid_z_lidar_m,"
            "predicted_x_lidar_m,predicted_y_lidar_m,predicted_z_lidar_m,"
            "match_distance_m,bbox_overlap,movement_residual_m,"
            "movement_speed_mps,suspicious_frames,moving,confirmed_moving,"
            "removed,status\n";
    first_write = false;
  }
  file << std::fixed << std::setprecision(9) << timestamp << ',' << track_id
       << ',' << current_index << ',' << previous_index << ',' << point_count
       << ',' << centroid.x() << ',' << centroid.y() << ',' << centroid.z()
       << ',' << predicted_centroid.x() << ',' << predicted_centroid.y() << ','
       << predicted_centroid.z() << ',' << match_distance << ',' << bbox_overlap
       << ',' << movement_residual << ',' << movement_speed << ','
       << suspicious_frames << ','
       << (moving ? 1 : 0) << ',' << (confirmed_moving ? 1 : 0) << ','
       << (removed ? 1 : 0) << ',' << status << '\n';
#else
  (void)timestamp;
  (void)track_id;
  (void)current_index;
  (void)previous_index;
  (void)point_count;
  (void)centroid;
  (void)predicted_centroid;
  (void)match_distance;
  (void)bbox_overlap;
  (void)movement_residual;
  (void)movement_speed;
  (void)suspicious_frames;
  (void)moving;
  (void)confirmed_moving;
  (void)removed;
  (void)status;
#endif
}

bool& LeftmostClusterLogFirstWrite() {
  static bool first_write = true;
  return first_write;
}

void BeginLeftmostClusterLogRun() {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(LeftmostClusterLogMutex());
  LeftmostClusterLogFirstWrite() = true;
#endif
}

void WriteLeftmostClusterLog(double timestamp, bool has_cluster,
                             const Eigen::Vector3d& centroid_body,
                             double centroid_offset, double bbox_overlap,
                             double motion_residual, bool dynamic) {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(LeftmostClusterLogMutex());
  const std::filesystem::path path =
      std::filesystem::current_path() / "slam_leftmost_cluster.csv";
  bool& first_write = LeftmostClusterLogFirstWrite();
  std::ofstream file(path, first_write ? (std::ios::out | std::ios::trunc)
                                       : (std::ios::out | std::ios::app));
  if (!file)
    return;
  if (first_write) {
    file << "timestamp,has_leftmost_cluster,centroid_x_body_m,"
            "centroid_y_body_m,centroid_z_body_m,centroid_offset_m,"
            "bbox_overlap,motion_residual_m,is_dynamic\n";
    first_write = false;
  }
  file << std::fixed << std::setprecision(9) << timestamp << ','
       << (has_cluster ? 1 : 0) << ',' << centroid_body.x() << ','
       << centroid_body.y() << ',' << centroid_body.z() << ','
       << centroid_offset << ',' << bbox_overlap << ',' << motion_residual
       << ',' << (dynamic ? 1 : 0) << '\n';
#else
  (void)timestamp;
  (void)has_cluster;
  (void)centroid_body;
  (void)centroid_offset;
  (void)bbox_overlap;
  (void)motion_residual;
  (void)dynamic;
#endif
}

void BeginSlamCsvRun() {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(SlamCsvMutex());
  SlamCsvFirstWrite() = true;
#endif
}

Eigen::Vector3d ExtractRollPitchYawDeg(const Eigen::Matrix3d& rotation) {
  // This is the inverse of EulerFrdToRnb's ZYX convention.  The logged
  // attitude is expressed in the SLAM world (ENU) frame, not the raw INS FRD
  // frame used at input.
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch =
      std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw) * (180.0 / kPi);
}

struct SlamLogDiagnostics {
  Eigen::Vector3d navigation_xyz =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d prediction_xyz =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d gicp_delta_xyz =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d innovation_xyz =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d navigation_rpy_deg =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d prediction_rpy_deg =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d gicp_delta_rpy_deg =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d innovation_rpy_deg =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  // The local rotation-vector Y component is the pitch correction direction.
  double kalman_pitch_gain = std::numeric_limits<double>::quiet_NaN();
  double mahalanobis_distance = std::numeric_limits<double>::quiet_NaN();
  int direction_consistent = -1;
  int innovation_consistent = -1;
  int dynamic_clusters = 0;
  size_t dynamic_points = 0;
  double max_dynamic_residual_m =
      std::numeric_limits<double>::quiet_NaN();
};

void WriteSlamCsv(double timestamp, size_t input_points, size_t filtered_points,
                  bool odom_available, double odom_dt, bool converged,
                  int correspondences, double rmse, double overlap,
                  double fitness_score, double odom_weight,
                  const Eigen::Isometry3d* pose, const char* status,
                  const SlamLogDiagnostics& diagnostics =
                      SlamLogDiagnostics()) {
#if LIDAR_GNSS_SLAM_ENABLE_LOG
  std::lock_guard<std::mutex> lock(SlamCsvMutex());
  const std::filesystem::path path =
      std::filesystem::current_path() / "slam_metrics.csv";
  bool& first_write_in_run = SlamCsvFirstWrite();
  std::ofstream file(
      path, first_write_in_run ? (std::ios::out | std::ios::trunc)
                               : (std::ios::out | std::ios::app));
  if (!file) {
    return;
  }
  if (first_write_in_run) {
    file << "timestamp,input_points,filtered_points,odom_available,odom_dt,"
            "converged,correspondences,rmse,overlap,fitness_score,"
            "odom_weight,lidar_weight,roll_deg,pitch_deg,yaw_deg,"
            "nav_x,nav_y,nav_z,prediction_x,prediction_y,prediction_z,"
            "gicp_delta_x,gicp_delta_y,gicp_delta_z,innovation_x,"
            "innovation_y,innovation_z,"
            "nav_roll_deg,nav_pitch_deg,nav_yaw_deg,"
            "prediction_roll_deg,prediction_pitch_deg,prediction_yaw_deg,"
            "gicp_delta_roll_deg,gicp_delta_pitch_deg,gicp_delta_yaw_deg,"
            "innovation_roll_deg,innovation_pitch_deg,innovation_yaw_deg,"
            "kalman_pitch_gain,mahalanobis_distance,direction_consistent,"
            "innovation_consistent,dynamic_clusters,dynamic_points,"
            "max_dynamic_residual_m,status\n";
    first_write_in_run = false;
  }
  const Eigen::Vector3d rpy_deg =
      pose ? ExtractRollPitchYawDeg(pose->rotation())
           : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  file << std::fixed << std::setprecision(9)
       << timestamp << ',' << input_points << ',' << filtered_points << ','
       << (odom_available ? 1 : 0) << ',' << odom_dt << ','
       << (converged ? 1 : 0) << ',' << correspondences << ',' << rmse << ','
       << overlap << ',' << fitness_score << ',' << odom_weight << ','
       << (1.0 - odom_weight) << ',' << rpy_deg.x() << ',' << rpy_deg.y()
       << ',' << rpy_deg.z() << ',' << diagnostics.navigation_xyz.x() << ','
       << diagnostics.navigation_xyz.y() << ','
       << diagnostics.navigation_xyz.z() << ','
       << diagnostics.prediction_xyz.x() << ','
       << diagnostics.prediction_xyz.y() << ','
       << diagnostics.prediction_xyz.z() << ','
       << diagnostics.gicp_delta_xyz.x() << ','
       << diagnostics.gicp_delta_xyz.y() << ','
       << diagnostics.gicp_delta_xyz.z() << ','
       << diagnostics.innovation_xyz.x() << ','
       << diagnostics.innovation_xyz.y() << ','
       << diagnostics.innovation_xyz.z() << ','
       << diagnostics.navigation_rpy_deg.x() << ','
       << diagnostics.navigation_rpy_deg.y() << ','
       << diagnostics.navigation_rpy_deg.z() << ','
       << diagnostics.prediction_rpy_deg.x() << ','
       << diagnostics.prediction_rpy_deg.y() << ','
       << diagnostics.prediction_rpy_deg.z() << ','
       << diagnostics.gicp_delta_rpy_deg.x() << ','
       << diagnostics.gicp_delta_rpy_deg.y() << ','
       << diagnostics.gicp_delta_rpy_deg.z() << ','
       << diagnostics.innovation_rpy_deg.x() << ','
       << diagnostics.innovation_rpy_deg.y() << ','
       << diagnostics.innovation_rpy_deg.z() << ','
       << diagnostics.kalman_pitch_gain << ','
       << diagnostics.mahalanobis_distance << ','
       << diagnostics.direction_consistent << ','
       << diagnostics.innovation_consistent << ','
       << diagnostics.dynamic_clusters << ',' << diagnostics.dynamic_points
       << ',' << diagnostics.max_dynamic_residual_m << ',' << status << '\n';
#else
  // Logging is intentionally disabled in production builds by default.
  (void)timestamp;
  (void)input_points;
  (void)filtered_points;
  (void)odom_available;
  (void)odom_dt;
  (void)converged;
  (void)correspondences;
  (void)rmse;
  (void)overlap;
  (void)fitness_score;
  (void)odom_weight;
  (void)pose;
  (void)status;
  (void)diagnostics;
#endif
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rotation) {
  Eigen::AngleAxisd aa(rotation);
  return aa.axis() * aa.angle();
}

Eigen::Matrix<double, 6, 6> EstimateGicpCovariance(
    const Eigen::Matrix<double, 6, 6>& hessian, int correspondences,
    double final_error, double fitness_score) {
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Identity();
  const double point_count = std::max(
      kGicpEffectivePointsFloor, static_cast<double>(correspondences));
  const double degrees_of_freedom =
      std::max(1.0, point_count - 6.0);
  const double residual_variance = std::clamp(
      std::isfinite(final_error) ? final_error / degrees_of_freedom : 1.0,
      kGicpResidualVarianceFloor, kGicpResidualVarianceCeiling);

  // H is a sum over points.  Normalize it first, then use a capped effective
  // point count so dense/correlated scans cannot create artificial certainty.
  const double effective_points = std::min(point_count, kGicpReferencePoints);
  const Eigen::Matrix<double, 6, 6> normalized_hessian =
      hessian / point_count;
  const Eigen::Matrix<double, 6, 6> regularized =
      normalized_hessian + kHessianRegularization *
                               Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> inverse = regularized.inverse();
  const double fitness_scale = 1.0 + std::max(0.0, fitness_score);

  // The Hessian supplies the local observability of the registration.  Use
  // only its diagonal here because translation and rotation have different
  // units; the residual scale prevents an apparently well-conditioned but
  // high-error match from becoming overconfident.
  for (int i = 0; i < 6; ++i) {
    const double hessian_variance = std::abs(inverse(i, i)) *
                                    residual_variance * fitness_scale /
                                    effective_points;
    covariance(i, i) = std::clamp(
        std::isfinite(hessian_variance) ? hessian_variance : 1.0,
        kMinGicpVariance, kMaxGicpVariance);
  }
  return covariance;
}

Eigen::Matrix3d RotX(double angle_rad) {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d rotation;
  rotation << 1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c;
  return rotation;
}

Eigen::Matrix3d RotY(double angle_rad) {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d rotation;
  rotation << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return rotation;
}

Eigen::Matrix3d RotZ(double angle_rad) {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d rotation;
  rotation << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return rotation;
}

Eigen::Matrix3d EulerFrdToRnb(double roll_rad, double pitch_rad,
                              double yaw_rad) {
  return RotZ(yaw_rad) * RotY(pitch_rad) * RotX(roll_rad);
}

Eigen::Matrix3d REnuNed() {
  Eigen::Matrix3d rotation;
  rotation << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
  return rotation;
}

void GetDr(const Eigen::Vector3d& blh, double& rm, double& rn,
           Eigen::Matrix3d& dr) {
  constexpr double kEarthA = 6378137.0;
  constexpr double kEarthB = 6356752.3142;
  const double e = std::sqrt(1.0 - (kEarthB / kEarthA) * (kEarthB / kEarthA));
  const double e2 = e * e;

  const double lat = blh.x();
  const double height = blh.z();

  rn = kEarthA / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
  rm = kEarthA * (1.0 - e2) /
       std::pow(1.0 - e2 * std::sin(lat) * std::sin(lat), 1.5);

  dr = -Eigen::Matrix3d::Identity();
  dr(0, 0) = rm + height;
  dr(1, 1) = (rn + height) * std::cos(lat);
}

}  // namespace

namespace usv {

LidarGnssSlam::LidarGnssSlam(int gicp_num_threads, bool require_gnss_ins,
                             double max_gnss_sync_sec, double voxel_leaf_size)
    : require_gnss_ins_(require_gnss_ins),
      max_gnss_sync_sec_(max_gnss_sync_sec),
      voxel_leaf_size_(voxel_leaf_size) {
  BeginSlamCsvRun();
  BeginLeftmostClusterLogRun();
  BeginDynamicClusterLogRun();
  lidar_211_to_body_ = gnss_geo::lidar211ToBodyExtrinsic();
  lidar_210_to_211_ = gnss_geo::lidar210To211Extrinsic();

  gicp_.setCorrespondenceRandomness(kGicpKCorrespondences);
  gicp_.setMaxCorrespondenceDistance(kGicpMaxCorrespondenceDistance);
  gicp_.setMaximumIterations(kGicpMaxIterations);
  gicp_.setTransformationEpsilon(kGicpTransformationEpsilon);
  gicp_.setEuclideanFitnessEpsilon(kGicpFitnessEpsilon);
  gicp_.setRANSACIterations(kGicpRansacIterations);
  gicp_.setRANSACOutlierRejectionThreshold(kGicpRansacOutlierThresh);
  gicp_.setNumThreads(gicp_num_threads);

  prereg_gicp_.setCorrespondenceRandomness(kGicpKCorrespondences);
  prereg_gicp_.setMaxCorrespondenceDistance(kGicpMaxCorrespondenceDistance);
  prereg_gicp_.setMaximumIterations(kGicpMaxIterations);
  prereg_gicp_.setTransformationEpsilon(kGicpTransformationEpsilon);
  prereg_gicp_.setEuclideanFitnessEpsilon(kGicpFitnessEpsilon);
  prereg_gicp_.setRANSACIterations(kGicpRansacIterations);
  prereg_gicp_.setRANSACOutlierRejectionThreshold(kGicpRansacOutlierThresh);
  prereg_gicp_.setNumThreads(gicp_num_threads);

  const float leaf = static_cast<float>(voxel_leaf_size_);
  voxel_filter_.setLeafSize(leaf, leaf, leaf);

  running_ = true;
  slam_thread_ = std::thread(&LidarGnssSlam::slamLoop, this);
}

LidarGnssSlam::~LidarGnssSlam() {
  running_ = false;
  lidar_cv_.notify_all();
  if (slam_thread_.joinable()) {
    slam_thread_.join();
  }
}

void LidarGnssSlam::addGnssInsData(const GnssInsMessage& gnss_ins) {
  if (gnss_ins.longitude == 0.0 && gnss_ins.latitude == 0.0) {
    return;
  }
  enqueueOdomData(convertGnssInsToOdom(gnss_ins));
}

void LidarGnssSlam::enqueueOdomData(const OdomData& odom) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_queue_.push_back(odom);

  while (odom_queue_.size() > kMaxOdomQueueSize) {
    odom_queue_.pop_front();
  }
}

OdomData LidarGnssSlam::convertGnssInsToOdom(const GnssInsMessage& gnss_ins) {
  if (!has_last_gnss_ins_) {
    last_gnss_ins_ = gnss_ins;
    has_last_gnss_ins_ = true;
    accumulated_enu_.setZero();
  } else {
    const Eigen::Vector3d current_blh(
        gnss_ins.latitude * kDegToRad, gnss_ins.longitude * kDegToRad,
        gnss_ins.altitude);
    const Eigen::Vector3d last_blh(last_gnss_ins_.latitude * kDegToRad,
                                   last_gnss_ins_.longitude * kDegToRad,
                                   last_gnss_ins_.altitude);

    double rm = 0.0;
    double rn = 0.0;
    Eigen::Matrix3d dr = Eigen::Matrix3d::Identity();
    GetDr(current_blh, rm, rn, dr);

    const Eigen::Vector3d dblh = current_blh - last_blh;
    const Eigen::Vector3d dned = dr * dblh;
    accumulated_enu_ += Eigen::Vector3d(dned.y(), dned.x(), -dned.z());
    last_gnss_ins_ = gnss_ins;
  }

  const Eigen::Matrix3d rnb =
      EulerFrdToRnb(gnss_ins.roll * kDegToRad, gnss_ins.pitch * kDegToRad,
                    gnss_ins.yaw * kDegToRad);
  const Eigen::Matrix3d reb = REnuNed() * rnb;

  Eigen::Isometry3d body_pose = Eigen::Isometry3d::Identity();
  body_pose.linear() = reb;
  body_pose.translation() = accumulated_enu_;

  OdomData odom;
  odom.timestamp = gnss_ins.timestamp;
  odom.pose = body_pose * lidar_211_to_body_ * lidar_210_to_211_;
  return odom;
}

void LidarGnssSlam::addLidarData(
    const std::shared_ptr<const LidarFrame>& lidar) {
  if (!lidar) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    // Keep only the newest pending frame so a slow registration cannot build
    // an unbounded stale backlog.
    lidar_queue_.clear();
    lidar_queue_.push_back(*lidar);
    lidar_queue_.back().point_count =
        static_cast<uint32_t>(lidar_queue_.back().points.size());
  }

  lidar_cv_.notify_one();
}

PoseState LidarGnssSlam::getPose() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_pose_;
}

LidarFrame LidarGnssSlam::getLatestWorldLidarFrame() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_world_lidar_frame_;
}

double LidarGnssSlam::getLastProcessedLidarTimestamp() const {
  return last_processed_lidar_timestamp_.load();
}

std::vector<M_PointXYZI> LidarGnssSlam::getLastClusteredCloud() const {
  std::lock_guard<std::mutex> lock(display_cloud_mutex_);
  return last_clustered_cloud_;
}

std::vector<M_PointXYZI> LidarGnssSlam::getLastDynamicDisplayCloud() const {
  std::lock_guard<std::mutex> lock(display_cloud_mutex_);
  return last_dynamic_display_cloud_;
}

std::vector<M_PointXYZI> LidarGnssSlam::getLastConfirmedDynamicCloud() const {
  std::lock_guard<std::mutex> lock(display_cloud_mutex_);
  return last_confirmed_dynamic_cloud_;
}

std::vector<std::vector<M_PointXYZI>>
LidarGnssSlam::getLastConfirmedDynamicHistoryWorld() const {
  std::lock_guard<std::mutex> lock(display_cloud_mutex_);
  return last_confirmed_dynamic_history_world_;
}

std::vector<M_PointXYZI> LidarGnssSlam::getLastStaticCloud() const {
  std::lock_guard<std::mutex> lock(display_cloud_mutex_);
  return last_static_cloud_;
}

size_t LidarGnssSlam::getPendingLidarCount() const {
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  return lidar_queue_.size();
}

size_t LidarGnssSlam::getOdomQueueSize() const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return odom_queue_.size();
}

SlamGeoPoseReference LidarGnssSlam::getLastNavigationReference() const {
  std::lock_guard<std::mutex> lock(navigation_reference_mutex_);
  return last_navigation_reference_;
}

void LidarGnssSlam::markLidarProcessed(double lidar_timestamp) {
  last_processed_lidar_timestamp_.store(lidar_timestamp);
}

void LidarGnssSlam::slamLoop() {
  while (running_) {
    LidarFrame lidar;

    {
      std::unique_lock<std::mutex> lock(lidar_mutex_);
      lidar_cv_.wait(lock,
                     [this] { return !running_ || !lidar_queue_.empty(); });

      if (!running_ && lidar_queue_.empty()) {
        return;
      }

      lidar = std::move(lidar_queue_.front());
      lidar_queue_.pop_front();
    }

    processLidarFrame(lidar);
  }
}

bool LidarGnssSlam::fallbackOdom(double lidar_timestamp, OdomData& odom) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (latest_pose_.valid) {
    odom.pose = latest_pose_.pose;
  } else {
    odom.pose = Eigen::Isometry3d::Identity();
  }
  odom.timestamp = lidar_timestamp;
  return true;
}

void LidarGnssSlam::processLidarFrame(const LidarFrame& lidar) {
  // Only an explicitly accepted GICP update is trusted for long-term map
  // keyframes. Prediction/odometry-only frames remain usable for tracking.
  last_pose_lidar_accepted_ = false;
  OdomData matched_odom;
  double nearest_source_time_error_sec =
      std::numeric_limits<double>::infinity();
  bool has_odom = interpolateOdom(
      lidar.timestamp, matched_odom, &nearest_source_time_error_sec);
  double odom_dt = has_odom
                       ? nearest_source_time_error_sec
                       : std::numeric_limits<double>::quiet_NaN();

  if (has_odom) {
    const double dt = std::abs(matched_odom.timestamp - lidar.timestamp);
    if (dt > max_gnss_sync_sec_) {
      has_odom = false;
    }
  }

  if (!has_odom) {
    if (require_gnss_ins_) {
      skipped_no_gnss_count_.fetch_add(1);
      {
        std::lock_guard<std::mutex> lock(navigation_reference_mutex_);
        last_navigation_reference_ = SlamGeoPoseReference{};
      }
      WriteSlamCsv(lidar.timestamp, lidar.points.size(), 0, false, odom_dt,
                   false, 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                   std::numeric_limits<double>::quiet_NaN(), 1.0,
                   nullptr, "skipped_no_odom");
      markLidarProcessed(lidar.timestamp);
      return;
    }
    fallbackOdom(lidar.timestamp, matched_odom);
  }

  {
    std::lock_guard<std::mutex> lock(navigation_reference_mutex_);
    last_navigation_reference_ = slam_pose::makeNavigationReference(
        lidar.timestamp, matched_odom, nearest_source_time_error_sec,
        has_odom);
  }

  PoseState previous_pose;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_pose = latest_pose_;
  }

  Eigen::Isometry3d odom_delta = Eigen::Isometry3d::Identity();
  if (has_last_lidar_odom_) {
    odom_delta = last_lidar_odom_.pose.inverse() * matched_odom.pose;
  }

  const double filter_dt = has_last_lidar_odom_
                               ? std::max(kMinFilterDt, lidar.timestamp -
                                              last_lidar_timestamp_)
                               : 0.0;
  // Constant-velocity error-state propagation.  The navigation delta is the
  // control input for the nominal pose; F/Q propagate uncertainty for the
  // 12-dimensional error state.
  if (has_filter_state_) {
    Eigen::Matrix<double, 12, 12> F =
        Eigen::Matrix<double, 12, 12>::Identity();
    F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * filter_dt;
    F.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity() * filter_dt;

    Eigen::Matrix<double, 12, 12> Q =
        Eigen::Matrix<double, 12, 12>::Zero();
    const double dt2 = filter_dt * filter_dt;
    const double dt3 = dt2 * filter_dt;
    const double dt4 = dt2 * dt2;
    const double linear_noise = kLinearAccelerationNoise *
                                 kLinearAccelerationNoise;
    const double angular_noise = kAngularAccelerationNoise *
                                  kAngularAccelerationNoise;
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Q.block<3, 3>(0, 0) = identity * (0.25 * dt4 * linear_noise);
    Q.block<3, 3>(0, 6) = identity * (0.5 * dt3 * linear_noise);
    Q.block<3, 3>(6, 0) = Q.block<3, 3>(0, 6).transpose();
    Q.block<3, 3>(6, 6) = identity * (dt2 * linear_noise);
    Q.block<3, 3>(3, 3) = identity * (0.25 * dt4 * angular_noise);
    Q.block<3, 3>(3, 9) = identity * (0.5 * dt3 * angular_noise);
    Q.block<3, 3>(9, 3) = Q.block<3, 3>(3, 9).transpose();
    Q.block<3, 3>(9, 9) = identity * (dt2 * angular_noise);
    Q.block<3, 3>(0, 0).diagonal().array() +=
        kNavigationPositionVariance;
    Q.block<3, 3>(3, 3).diagonal().array() +=
        kNavigationAttitudeVariance;

    state_covariance_ = F * state_covariance_ * F.transpose() + Q;
    state_covariance_ =
        0.5 * (state_covariance_ + state_covariance_.transpose());
  }

  const Eigen::Isometry3d odom_propagated_pose =
      previous_pose.valid ? previous_pose.pose * odom_delta
                          : Eigen::Isometry3d::Identity();
  SlamLogDiagnostics diagnostics;
  diagnostics.navigation_xyz = matched_odom.pose.translation();
  diagnostics.prediction_xyz = odom_propagated_pose.translation();
  diagnostics.navigation_rpy_deg =
      ExtractRollPitchYawDeg(matched_odom.pose.rotation());
  diagnostics.prediction_rpy_deg =
      ExtractRollPitchYawDeg(odom_propagated_pose.rotation());
  if (has_last_lidar_odom_) {
    // Nominal velocity state propagated by the navigation control.  The
    // lidar update below corrects this value through the Kalman gain.
    linear_velocity_ = odom_delta.translation() /
                       std::max(filter_dt, kMinFilterDt);
    angular_velocity_ = RotationVector(odom_delta.rotation()) /
                        std::max(filter_dt, kMinFilterDt);
  }

  // Apply the navigation jump guard before any point-count or registration
  // early return. Otherwise a sparse frame can bypass stale-reference
  // handling and still write a large predicted pose into the map.
  if (previous_pose.valid && has_last_lidar_odom_ &&
      odom_delta.translation().norm() > kMaxStaleReferenceReanchorJumpM) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_pose_.timestamp = lidar.timestamp;
      latest_pose_.pose = previous_pose.pose;
      latest_pose_.valid = true;
      latest_world_lidar_frame_ = LidarFrame{};
      state_covariance_.setIdentity();
      has_filter_state_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(display_cloud_mutex_);
      last_static_cloud_.clear();
      last_clustered_cloud_.clear();
      last_dynamic_display_cloud_.clear();
      last_confirmed_dynamic_cloud_.clear();
      last_confirmed_dynamic_history_world_.clear();
    }
    last_scan_.reset();
    last_unfiltered_scan_.reset();
    // The dynamic-track indices refer to the previous unfiltered scan.  Once
    // that scan is invalidated by a navigation jump, keeping the tracks would
    // make the next frame try to match against a null/stale cloud.
    dynamic_cluster_tracks_.clear();
    confirmed_dynamic_memory_.clear();
    has_scan_reference_ = false;
    reanchor_scan_reference_ = true;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), 0, has_odom, odom_dt,
                 false, 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &previous_pose.pose, "skipped_navigation_motion_jump",
                 diagnostics);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  PointCloud::Ptr raw_cloud = convertCloud(lidar);
  if (!raw_cloud || raw_cloud->size() < static_cast<size_t>(kMinPoints)) {
    const Eigen::Isometry3d logged_pose =
        previous_pose.valid ? odom_propagated_pose
                            : Eigen::Isometry3d::Identity();
    WriteSlamCsv(lidar.timestamp, lidar.points.size(),
                 raw_cloud ? raw_cloud->size() : 0, has_odom, odom_dt, false,
                 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &logged_pose, "skipped_too_few_points", diagnostics);
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_.timestamp = lidar.timestamp;
    if (!latest_pose_.valid) {
      latest_pose_.pose = Eigen::Isometry3d::Identity();
      latest_pose_.valid = true;
    } else if (has_last_lidar_odom_) {
      latest_pose_.pose = odom_propagated_pose;
    }
    state_covariance_.setIdentity();
    has_filter_state_ = true;
    latest_world_lidar_frame_ = LidarFrame{};
    reanchor_scan_reference_ = true;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  PointCloud::Ptr current_scan = downsampleCloud(raw_cloud);
  const PointCloud::Ptr unfiltered_scan = current_scan;
  if (!current_scan || current_scan->empty()) {
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), 0, has_odom, odom_dt,
                 false, 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 previous_pose.valid ? &previous_pose.pose : nullptr,
                 "skipped_empty_scan", diagnostics);
    // Advance the navigation state and timestamp even when filtering removed
    // the whole scan.  Otherwise the next valid frame is compared against an
    // old navigation pose and an old point-cloud reference.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_pose_.timestamp = lidar.timestamp;
      latest_pose_.pose = odom_propagated_pose;
      latest_pose_.valid = true;
      latest_world_lidar_frame_ = LidarFrame{};
      state_covariance_ *= 1.05;
      has_filter_state_ = true;
    }
    reanchor_scan_reference_ = true;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  if (!initialized_) {
    const Eigen::Isometry3d logged_pose = Eigen::Isometry3d::Identity();
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), current_scan->size(),
                 has_odom, odom_dt, false, 0,
                 std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &logged_pose, "initialized", diagnostics);
    last_scan_ = current_scan;
    last_unfiltered_scan_ = unfiltered_scan;
    last_scan_timestamp_ = lidar.timestamp;
    has_scan_reference_ = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_.timestamp = lidar.timestamp;
    if (!latest_pose_.valid) {
      latest_pose_.pose = Eigen::Isometry3d::Identity();
      latest_pose_.valid = true;
    }
    latest_world_lidar_frame_ =
        transformLidarFrameToWorld(lidar, latest_pose_.pose);
    state_covariance_.setIdentity();
    has_filter_state_ = true;
    latest_pose_.valid = true;
    initialized_ = true;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  if (!previous_pose.valid || !has_last_lidar_odom_) {
    const Eigen::Isometry3d logged_pose = Eigen::Isometry3d::Identity();
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), current_scan->size(),
                 has_odom, odom_dt, false, 0,
                 std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &logged_pose, "reset_without_previous_pose", diagnostics);
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_.timestamp = lidar.timestamp;
    latest_pose_.pose = Eigen::Isometry3d::Identity();
    latest_pose_.valid = true;
    latest_world_lidar_frame_ =
        transformLidarFrameToWorld(lidar, latest_pose_.pose);
    state_covariance_.setIdentity();
    has_filter_state_ = true;
    last_scan_ = current_scan;
    last_unfiltered_scan_ = unfiltered_scan;
    last_scan_timestamp_ = lidar.timestamp;
    has_scan_reference_ = true;
    initialized_ = true;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  const size_t num_points = current_scan->size();
  const double odom_weight = computeOdomWeight(num_points);

  if (static_cast<int>(num_points) <= kOdomWeightPointsLow) {
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), num_points, has_odom,
                 odom_dt, false, 0,
                 std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), odom_weight,
                 &odom_propagated_pose, "odom_only_low_point_count",
                 diagnostics);
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_.timestamp = lidar.timestamp;
    latest_pose_.pose = odom_propagated_pose;
    latest_pose_.valid = true;
    latest_world_lidar_frame_ =
        transformLidarFrameToWorld(lidar, latest_pose_.pose);
    last_scan_ = current_scan;
    last_unfiltered_scan_ = unfiltered_scan;
    // This frame becomes the next unfiltered reference but did not run
    // dynamic clustering. Its predecessor's point indices are therefore no
    // longer valid for it.
    dynamic_cluster_tracks_.clear();
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    return;
  }

  // Registration 1 uses the complete, unfiltered fused scan. Its accepted
  // relative pose drives dynamic-cluster prediction; rejected registration
  // falls back to the navigation relative pose.
  Eigen::Isometry3d previous_to_current =
      matched_odom.pose.inverse() * last_lidar_odom_.pose;
  bool prereg_accepted = false;
  Eigen::Matrix4f second_initial_guess = odom_delta.matrix().cast<float>();
  if (kEnablePreRegistration && last_unfiltered_scan_ &&
      last_unfiltered_scan_->size() >= static_cast<size_t>(kMinPoints)) {
    prereg_gicp_.setInputTarget(last_unfiltered_scan_);
    prereg_gicp_.setInputSource(unfiltered_scan);
    prereg_gicp_.calculateTargetCovariances();
    PointCloud prereg_aligned;
    prereg_gicp_.align(prereg_aligned, second_initial_guess);
    if (prereg_gicp_.hasConverged()) {
      const Eigen::Isometry3d prereg_delta(
          prereg_gicp_.getFinalTransformation().cast<double>());
      const Eigen::Isometry3d prereg_measurement =
          previous_pose.pose * prereg_delta;
      const Eigen::Isometry3d prereg_innovation_pose =
          odom_propagated_pose.inverse() * prereg_measurement;
      Eigen::Matrix<double, 6, 1> prereg_innovation;
      prereg_innovation.head<3>() = prereg_innovation_pose.translation();
      prereg_innovation.tail<3>() =
          RotationVector(prereg_innovation_pose.rotation());
      Eigen::Matrix<double, 6, 6> prereg_covariance =
          EstimateGicpCovariance(prereg_gicp_.getFinalHessian(),
                                 prereg_gicp_.num_correspondences,
                                 prereg_gicp_.getFinalError(),
                                 prereg_gicp_.getFitnessScore());
      Eigen::Matrix<double, 6, 12> H =
          Eigen::Matrix<double, 6, 12>::Zero();
      H.block<6, 6>(0, 0).setIdentity();
      const Eigen::Matrix<double, 6, 6> S =
          H * state_covariance_ * H.transpose() + prereg_covariance +
          kCovarianceNumericalFloor * Eigen::Matrix<double, 6, 6>::Identity();
      const double mahalanobis_distance =
          prereg_innovation.dot(S.ldlt().solve(prereg_innovation));
      const Eigen::Vector3d measured_translation = prereg_delta.translation();
      bool direction_consistent = true;
      if (odom_delta.translation().norm() > kDirectionMotionMin &&
          measured_translation.norm() > kDirectionMotionMin) {
        const Eigen::Vector3d direction = odom_delta.translation().normalized();
        const double lateral =
            (measured_translation - direction * measured_translation.dot(direction)).norm();
        direction_consistent =
            lateral <= std::max(kDirectionLateralMin,
                                kDirectionLateralRatio * odom_delta.translation().norm());
      }
      prereg_accepted = direction_consistent &&
                         mahalanobis_distance <= kInnovationMahalanobisGate &&
                         !DisableLidarKalmanCorrection();
      if (prereg_accepted) {
        // Do not use the full-cloud GICP motion for dynamic-object
        // compensation. A nearby moving vessel can dominate this registration
        // and make its own motion disappear from the residual. Dynamic-cluster
        // prediction stays on the GNSS/INS motion initialized above; the
        // accepted pre-registration remains useful only as the second GICP
        // initial guess.
        second_initial_guess = prereg_gicp_.getFinalTransformation();
      }
    }
  }

  int dynamic_clusters = 0;
  size_t dynamic_points = 0;
  double max_dynamic_residual = std::numeric_limits<double>::quiet_NaN();
  if (LIDAR_GNSS_SLAM_DYNAMIC_FILTER_ENABLE || LIDAR_GNSS_SLAM_ENABLE_LOG) {
    PointCloud::Ptr processed_scan = removeDynamicPoints(
        unfiltered_scan, last_unfiltered_scan_, previous_to_current,
        dynamic_clusters, dynamic_points, max_dynamic_residual,
        lidar.timestamp, odom_propagated_pose);
    if (LIDAR_GNSS_SLAM_DYNAMIC_FILTER_ENABLE)
      current_scan = processed_scan;
  } else {
    dynamic_cluster_tracks_.clear();
  }
  diagnostics.dynamic_clusters = dynamic_clusters;
  diagnostics.dynamic_points = dynamic_points;
  diagnostics.max_dynamic_residual_m = max_dynamic_residual;
  if (!current_scan || current_scan->empty()) {
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), 0, has_odom, odom_dt,
                 false, 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &odom_propagated_pose, "skipped_empty_static_scan", diagnostics);
    // removeDynamicPoints() has already advanced dynamic_cluster_tracks_ to
    // this unfiltered frame. Keep its point-index reference synchronized and
    // force the next static registration to establish a fresh target.
    last_unfiltered_scan_ = unfiltered_scan;
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    reanchor_scan_reference_ = true;
    markLidarProcessed(lidar.timestamp);
    return;
  }

  const double reference_age =
      has_scan_reference_ ? std::max(0.0, lidar.timestamp - last_scan_timestamp_)
                          : std::numeric_limits<double>::infinity();
  const bool reference_stale =
      reanchor_scan_reference_ || !has_scan_reference_ ||
      reference_age > kMaxGicpReferenceAgeSec ||
      (has_odom && odom_dt > kMaxGicpOdomSyncSec);
  if (reference_stale) {
    // Re-anchor on the first usable scan after a gap.  It is deliberately not
    // registered against the stale scan from before the gap.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_pose_.timestamp = lidar.timestamp;
      latest_pose_.pose = odom_propagated_pose;
      latest_pose_.valid = true;
      latest_world_lidar_frame_ = transformLidarFrameToWorld(lidar, odom_propagated_pose);
      state_covariance_.setIdentity();
      has_filter_state_ = true;
    }
    last_scan_ = current_scan;
    last_unfiltered_scan_ = unfiltered_scan;
    last_scan_timestamp_ = lidar.timestamp;
    has_scan_reference_ = true;
    reanchor_scan_reference_ = false;
    dynamic_cluster_tracks_.clear();
    last_lidar_odom_ = matched_odom;
    has_last_lidar_odom_ = true;
    last_lidar_timestamp_ = lidar.timestamp;
    trimOdomQueue(lidar.timestamp);
    markLidarProcessed(lidar.timestamp);
    WriteSlamCsv(lidar.timestamp, lidar.points.size(), current_scan->size(),
                 has_odom, odom_dt, false, 0,
                 std::numeric_limits<double>::quiet_NaN(), 0.0,
                 std::numeric_limits<double>::quiet_NaN(), 1.0,
                 &odom_propagated_pose, "reanchored_stale_reference",
                 diagnostics);
    return;
  }

  // Registration 2 estimates the final SLAM pose after dynamic removal.
  gicp_.setInputTarget(last_scan_);
  gicp_.setInputSource(current_scan);
  gicp_.calculateTargetCovariances();
  PointCloud aligned;
  gicp_.align(aligned, second_initial_guess);

  const bool primary_converged = gicp_.hasConverged();
  last_gicp_score_ = gicp_.getFitnessScore();
  const int correspondences = gicp_.num_correspondences;
  const double overlap =
      static_cast<double>(correspondences) / std::max<size_t>(1, num_points);
  const double rmse =
      correspondences > 0
          ? std::sqrt(std::max(0.0, gicp_.getFinalError()) /
                      static_cast<double>(correspondences))
          : std::numeric_limits<double>::quiet_NaN();
  Eigen::Isometry3d refined_pose = odom_propagated_pose;
  bool accepted_measurement = false;
  if (primary_converged) {
    const Eigen::Matrix4f current_to_last = gicp_.getFinalTransformation();
    const Eigen::Isometry3d lidar_delta(current_to_last.cast<double>());
    diagnostics.gicp_delta_xyz = lidar_delta.translation();
    diagnostics.gicp_delta_rpy_deg =
        ExtractRollPitchYawDeg(lidar_delta.rotation());
    const Eigen::Isometry3d lidar_measurement =
        previous_pose.pose * lidar_delta;
    const Eigen::Isometry3d prediction_to_measurement =
        odom_propagated_pose.inverse() * lidar_measurement;
    diagnostics.innovation_rpy_deg =
        ExtractRollPitchYawDeg(prediction_to_measurement.rotation());
    const Eigen::Vector3d innovation_translation =
        prediction_to_measurement.translation();
    diagnostics.innovation_xyz = innovation_translation;
    const Eigen::Vector3d innovation_rotation =
        RotationVector(prediction_to_measurement.rotation());
    Eigen::Matrix<double, 6, 6> measurement_covariance =
        EstimateGicpCovariance(gicp_.getFinalHessian(), correspondences,
                               gicp_.getFinalError(), last_gicp_score_);
    Eigen::Matrix<double, 6, 1> innovation;
    innovation.head<3>() = innovation_translation;
    innovation.tail<3>() = innovation_rotation;

    const Eigen::Vector3d prediction_translation = odom_delta.translation();
    const Eigen::Vector3d measured_translation = lidar_delta.translation();

    bool direction_consistent = true;
    if (prediction_translation.norm() > kDirectionMotionMin &&
        measured_translation.norm() > kDirectionMotionMin) {
      const Eigen::Vector3d direction = prediction_translation.normalized();
      const double lateral =
          (measured_translation - direction * measured_translation.dot(direction)).norm();
      direction_consistent =
          lateral <= std::max(kDirectionLateralMin,
                              kDirectionLateralRatio *
                                  prediction_translation.norm());
    }
    Eigen::Matrix<double, 6, 12> H =
        Eigen::Matrix<double, 6, 12>::Zero();
    H.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 6> S =
        H * state_covariance_ * H.transpose() + measurement_covariance +
        kCovarianceNumericalFloor *
            Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> solved_innovation =
        S.ldlt().solve(innovation);
    const double mahalanobis_distance = innovation.dot(solved_innovation);
    // A six-dimensional 4-sigma gate rejects gross scan failures while
    // allowing ordinary lidar/nav disagreement to be corrected by the filter.
    const bool innovation_consistent =
        mahalanobis_distance <= kInnovationMahalanobisGate;
    const bool registration_quality_consistent =
        std::isfinite(rmse) && std::isfinite(last_gicp_score_) &&
        correspondences >= kMinAcceptedGicpCorrespondences &&
        overlap >= kMinAcceptedGicpOverlap &&
        rmse <= kMaxAcceptedGicpRmse &&
        last_gicp_score_ <= kMaxAcceptedGicpFitness &&
        std::isfinite(diagnostics.innovation_rpy_deg.x()) &&
        std::isfinite(diagnostics.innovation_rpy_deg.y()) &&
        std::isfinite(diagnostics.innovation_rpy_deg.z()) &&
        std::abs(diagnostics.innovation_rpy_deg.x()) <=
            kMaxAcceptedGicpRollPitchInnovationDeg &&
        std::abs(diagnostics.innovation_rpy_deg.y()) <=
            kMaxAcceptedGicpRollPitchInnovationDeg &&
        std::abs(diagnostics.innovation_rpy_deg.z()) <=
            kMaxAcceptedGicpYawInnovationDeg &&
        innovation_translation.norm() <=
            kMaxAcceptedGicpTranslationInnovationM &&
        std::abs(innovation_translation.z()) <=
            kMaxAcceptedGicpVerticalInnovationM;
    diagnostics.mahalanobis_distance = mahalanobis_distance;
    diagnostics.direction_consistent = direction_consistent ? 1 : 0;
    diagnostics.innovation_consistent = innovation_consistent ? 1 : 0;
    accepted_measurement = direction_consistent && innovation_consistent &&
                           registration_quality_consistent &&
                           !DisableLidarKalmanCorrection();
    if (accepted_measurement) {
      const Eigen::Matrix<double, 12, 6> K =
          state_covariance_ * H.transpose() * S.ldlt().solve(
                                             Eigen::Matrix<double, 6, 6>::Identity());
      const Eigen::Matrix<double, 12, 1> state_correction = K * innovation;
      diagnostics.kalman_pitch_gain = K(4, 4);
      const Eigen::Matrix<double, 12, 12> identity12 =
          Eigen::Matrix<double, 12, 12>::Identity();
      const Eigen::Matrix<double, 12, 12> joseph_left = identity12 - K * H;
      state_covariance_ = joseph_left * state_covariance_ * joseph_left.transpose() +
                          K * measurement_covariance * K.transpose();
      state_covariance_ =
          0.5 * (state_covariance_ + state_covariance_.transpose());

      // Apply the filtered pose correction in the predicted pose's local
      // frame.  The innovation was computed as
      // prediction.inverse() * measurement, so using the same composition
      // convention prevents a GICP roll/pitch bias from being accumulated as
      // a full pose replacement.
      const Eigen::Vector3d filtered_translation =
          state_correction.head<3>();
      // Roll/pitch are navigation-controlled.  A scan can constrain yaw and
      // translation, but allowing small repeated roll/pitch corrections to
      // accumulate during a turn creates a large vertical map tilt.
      Eigen::Vector3d filtered_rotation = state_correction.segment<3>(3);
      filtered_rotation.x() = 0.0;
      filtered_rotation.y() = 0.0;
      const double max_applied_rp =
          kMaxAppliedGicpRollPitchCorrectionDeg * kDegToRad;
      if (std::abs(filtered_rotation.x()) > max_applied_rp ||
          std::abs(filtered_rotation.y()) > max_applied_rp) {
        accepted_measurement = false;
      }
      if (!accepted_measurement) {
        state_covariance_ *= 1.05;
      } else {
        Eigen::Isometry3d filtered_delta = Eigen::Isometry3d::Identity();
        filtered_delta.translation() = filtered_translation;
        const double filtered_angle = filtered_rotation.norm();
        if (filtered_angle > 1.0e-12) {
          filtered_delta.linear() =
              Eigen::AngleAxisd(filtered_angle,
                                filtered_rotation / filtered_angle)
                  .toRotationMatrix();
        }
        refined_pose = odom_propagated_pose * filtered_delta;
        linear_velocity_ += state_correction.segment<3>(6);
        Eigen::Vector3d angular_correction = state_correction.segment<3>(9);
        angular_correction.x() = 0.0;
        angular_correction.y() = 0.0;
        angular_velocity_ += angular_correction;
      }
    }
    last_pose_lidar_accepted_.store(accepted_measurement,
                                    std::memory_order_release);
  }
  if (!accepted_measurement) {
    state_covariance_ *= 1.05;
    linear_velocity_ = odom_delta.translation() /
                       std::max(filter_dt, kMinFilterDt);
    angular_velocity_ = RotationVector(odom_delta.rotation()) /
                        std::max(filter_dt, kMinFilterDt);
  }
  WriteSlamCsv(lidar.timestamp, lidar.points.size(), num_points, has_odom,
               odom_dt, primary_converged, correspondences, rmse, overlap,
               last_gicp_score_, accepted_measurement ? 0.0 : 1.0,
               &refined_pose,
               accepted_measurement
                   ? "second_gicp_direct_update"
                   : (primary_converged
                          ? (DisableLidarKalmanCorrection()
                                 ? "gnss_ins_only_prediction"
                                 : "second_gicp_rejected_odom_fallback")
                          : "prediction_only"),
               diagnostics);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_pose_.timestamp = lidar.timestamp;
    latest_pose_.pose = refined_pose;
    latest_pose_.valid = true;
    latest_world_lidar_frame_ =
        transformLidarFrameToWorld(lidar, latest_pose_.pose);
  }

  last_scan_ = current_scan;
  last_unfiltered_scan_ = unfiltered_scan;
  last_scan_timestamp_ = lidar.timestamp;
  has_scan_reference_ = true;
  last_lidar_odom_ = matched_odom;
  has_last_lidar_odom_ = true;
  last_lidar_timestamp_ = lidar.timestamp;
  trimOdomQueue(lidar.timestamp);
  markLidarProcessed(lidar.timestamp);
}

LidarGnssSlam::PointCloud::Ptr LidarGnssSlam::convertCloud(
    const LidarFrame& frame) const {
  PointCloud::Ptr cloud(new PointCloud);
  cloud->reserve(frame.points.size());

  for (const auto& point : frame.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    PointType pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.intensity;
    cloud->push_back(pcl_point);
  }

  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

LidarGnssSlam::PointCloud::Ptr LidarGnssSlam::downsampleCloud(
    const PointCloud::Ptr& cloud) const {
  PointCloud::Ptr filtered(new PointCloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

  PointCloud::Ptr downsampled(new PointCloud);
  auto voxel_filter = voxel_filter_;
  voxel_filter.setInputCloud(filtered);
  voxel_filter.filter(*downsampled);

  if (downsampled->size() <
      static_cast<size_t>(usv::slam_config::kDynamicSorMeanK))
    return downsampled;

  pcl::StatisticalOutlierRemoval<PointType> sor;
  sor.setInputCloud(downsampled);
  sor.setMeanK(usv::slam_config::kDynamicSorMeanK);
  sor.setStddevMulThresh(usv::slam_config::kDynamicSorStddev);
  PointCloud::Ptr denoised(new PointCloud);
  sor.filter(*denoised);
  return denoised;
}

LidarGnssSlam::PointCloud::Ptr LidarGnssSlam::removeDynamicPoints(
    const PointCloud::Ptr& current, const PointCloud::Ptr& previous,
    const Eigen::Isometry3d& previous_to_current, int& dynamic_clusters,
    size_t& dynamic_points, double& max_residual, double timestamp,
    const Eigen::Isometry3d& lidar_pose_in_world) {
  dynamic_clusters = 0;
  dynamic_points = 0;
  max_residual = std::numeric_limits<double>::quiet_NaN();
  last_confirmed_dynamic_removed_.store(false, std::memory_order_release);
  last_dynamic_confirmation_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(display_cloud_mutex_);
    last_confirmed_dynamic_cloud_.clear();
    last_confirmed_dynamic_history_world_.clear();
  }
  const double frame_dt =
      std::max(kMinFilterDt, timestamp - last_lidar_timestamp_);

  for (ConfirmedDynamicMemory& memory : confirmed_dynamic_memory_)
    --memory.remaining_frames;
  confirmed_dynamic_memory_.erase(
      std::remove_if(confirmed_dynamic_memory_.begin(),
                     confirmed_dynamic_memory_.end(),
                     [](const ConfirmedDynamicMemory& memory) {
                       return memory.remaining_frames < 0;
                     }),
      confirmed_dynamic_memory_.end());

  if (!current || current->empty()) {
    dynamic_cluster_tracks_.clear();
    std::lock_guard<std::mutex> lock(display_cloud_mutex_);
    last_clustered_cloud_.clear();
    last_dynamic_display_cloud_.clear();
    last_confirmed_dynamic_cloud_.clear();
    last_confirmed_dynamic_history_world_.clear();
    last_static_cloud_.clear();
    WriteLeftmostClusterLog(
        timestamp, false, Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), false);
    return current;
  }

  using Cluster = DynamicClusterTrack;
  auto extractClusters = [](const PointCloud::Ptr& cloud,
                            int minimum_cluster_size) {
    std::vector<Cluster> clusters;
    if (!cloud || cloud->empty())
      return clusters;

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointType> extraction;
    extraction.setClusterTolerance(std::max(
        kDynamicMinimumClusterTolerance,
        static_cast<double>(usv::slam_config::kDynamicClusterTolerance)));
    extraction.setMinClusterSize(std::max(1, minimum_cluster_size));
    extraction.setMaxClusterSize(static_cast<int>(cloud->size()));
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    extraction.extract(cluster_indices);
    clusters.reserve(cluster_indices.size());
    for (const pcl::PointIndices& indices : cluster_indices) {
      if (indices.indices.empty())
        continue;
      Cluster cluster;
      cluster.min_bound = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::infinity());
      cluster.max_bound = Eigen::Vector3d::Constant(
          -std::numeric_limits<double>::infinity());
      for (const int index : indices.indices) {
        if (index < 0 || static_cast<size_t>(index) >= cloud->size())
          continue;
        const PointType& point = cloud->points[static_cast<size_t>(index)];
        const Eigen::Vector3d value(point.x, point.y, point.z);
        cluster.indices.push_back(index);
        cluster.centroid += value;
        cluster.min_bound = cluster.min_bound.cwiseMin(value);
        cluster.max_bound = cluster.max_bound.cwiseMax(value);
      }
      if (cluster.indices.empty())
        continue;
      cluster.centroid /= static_cast<double>(cluster.indices.size());
      clusters.push_back(std::move(cluster));
    }
    return clusters;
  };

  const int large_cluster_min_points = std::max(
      kDynamicMinimumTrackPoints,
      static_cast<int>(usv::slam_config::kDynamicClusterMinPoints));
  std::vector<Cluster> all_current_clusters = extractClusters(current, 1);
  std::vector<Cluster> current_clusters;
  current_clusters.reserve(all_current_clusters.size());
  std::vector<bool> small_cluster_point(current->size(), false);
  size_t small_cluster_count = 0;
  size_t small_cluster_points = 0;
  Eigen::Vector3d small_cluster_centroid_sum = Eigen::Vector3d::Zero();
  for (Cluster& cluster : all_current_clusters) {
    if (static_cast<int>(cluster.indices.size()) >= large_cluster_min_points) {
      current_clusters.push_back(std::move(cluster));
      continue;
    }
    ++small_cluster_count;
    small_cluster_points += cluster.indices.size();
    small_cluster_centroid_sum +=
        cluster.centroid * static_cast<double>(cluster.indices.size());
    for (const int index : cluster.indices) {
      if (index < 0 || static_cast<size_t>(index) >= current->size())
        continue;
      small_cluster_point[static_cast<size_t>(index)] = true;
    }
  }
  if (small_cluster_count > 0) {
    const Eigen::Vector3d small_cluster_centroid =
        small_cluster_centroid_sum /
        static_cast<double>(std::max<size_t>(1, small_cluster_points));
    WriteDynamicClusterLog(
        timestamp, next_dynamic_cluster_track_id_++, -1, -1,
        small_cluster_points, small_cluster_centroid,
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        static_cast<int>(small_cluster_count), false, false, true,
        "removed_small_clusters");
  }
  if (current_clusters.empty()) {
    dynamic_cluster_tracks_.clear();
    std::vector<M_PointXYZI> display_cloud;
    display_cloud.reserve(current->size());
    PointCloud::Ptr static_cloud(new PointCloud);
    std::vector<M_PointXYZI> removed_cloud;
    for (size_t i = 0; i < current->size(); ++i) {
      const PointType& point = current->points[i];
      if (small_cluster_point[i]) {
        removed_cloud.push_back(
            {point.x, point.y, point.z,
             static_cast<unsigned char>(point.intensity)});
      } else {
        display_cloud.push_back({point.x, point.y, point.z, 0});
        static_cloud->push_back(point);
      }
    }
    static_cloud->width = static_cast<uint32_t>(static_cloud->size());
    static_cloud->height = 1;
    std::lock_guard<std::mutex> lock(display_cloud_mutex_);
    last_clustered_cloud_ = std::move(display_cloud);
    last_dynamic_display_cloud_ = std::move(removed_cloud);
    last_static_cloud_.clear();
    for (const PointType& point : static_cloud->points)
      last_static_cloud_.push_back(
          {point.x, point.y, point.z,
           static_cast<unsigned char>(point.intensity)});
    WriteLeftmostClusterLog(
        timestamp, false, Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN()),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), false);
    return static_cloud;
  }

  // Keep the recent world-space geometry for each tracked cluster. The CLI
  // uses these historical positions when confirmation arrives; the current
  // position alone is insufficient for a fast-moving target.
  for (Cluster& cluster : current_clusters) {
    std::vector<M_PointXYZI> world_points;
    world_points.reserve(cluster.indices.size());
    for (const int index : cluster.indices) {
      if (index < 0 || static_cast<size_t>(index) >= current->size())
        continue;
      const PointType& point = current->points[static_cast<size_t>(index)];
      const Eigen::Vector3d world =
          lidar_pose_in_world * Eigen::Vector3d(point.x, point.y, point.z);
      if (!world.allFinite())
        continue;
      world_points.push_back(
          {static_cast<float>(world.x()), static_cast<float>(world.y()),
           static_cast<float>(world.z()),
           static_cast<unsigned char>(point.intensity)});
    }
    cluster.world_history.push_back(std::move(world_points));
    while (cluster.world_history.size() > kDynamicWorldHistoryFrames)
      cluster.world_history.pop_front();
  }

  auto worldBounds = [&](const Cluster& cluster) {
    Eigen::Vector3d min_bound = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_bound = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    for (int ix = 0; ix <= 1; ++ix) {
      for (int iy = 0; iy <= 1; ++iy) {
        for (int iz = 0; iz <= 1; ++iz) {
          const Eigen::Vector3d corner(
              ix ? cluster.max_bound.x() : cluster.min_bound.x(),
              iy ? cluster.max_bound.y() : cluster.min_bound.y(),
              iz ? cluster.max_bound.z() : cluster.min_bound.z());
          const Eigen::Vector3d world = lidar_pose_in_world * corner;
          min_bound = min_bound.cwiseMin(world);
          max_bound = max_bound.cwiseMax(world);
        }
      }
    }
    return std::make_pair(min_bound, max_bound);
  };

  // The input cloud is in the current LiDAR frame. Convert through the
  // current vessel pose before deciding which cluster is leftmost. In the
  // vessel FRD frame, +X is forward and +Y is right, so the leftmost cluster
  // has the smallest body-frame Y. This is independent of its forward range.
  const Eigen::Isometry3d lidar_to_body =
      lidar_211_to_body_ * lidar_210_to_211_;
  const Eigen::Isometry3d body_pose_in_world =
      lidar_pose_in_world * lidar_to_body.inverse();
  size_t leftmost_index = 0;
  double leftmost_y = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < current_clusters.size(); ++i) {
    const Eigen::Vector3d centroid_world =
        lidar_pose_in_world * current_clusters[i].centroid;
    const Eigen::Vector3d centroid_body =
        body_pose_in_world.inverse() * centroid_world;
    const double body_y = centroid_body.y();
    if (body_y < leftmost_y) {
      leftmost_y = body_y;
      leftmost_index = i;
    }
  }
  const Eigen::Vector3d leftmost_centroid_world =
      lidar_pose_in_world * current_clusters[leftmost_index].centroid;
  const Eigen::Vector3d leftmost_centroid_body =
      body_pose_in_world.inverse() * leftmost_centroid_world;
  const double leftmost_centroid_offset =
      (leftmost_centroid_world - body_pose_in_world.translation()).norm();
  double leftmost_overlap = std::numeric_limits<double>::quiet_NaN();
  double leftmost_residual = std::numeric_limits<double>::quiet_NaN();
  bool leftmost_dynamic = false;

  std::vector<bool> clustered_point(current->size(), false);
  std::vector<unsigned char> cluster_color_index(current->size(), 0);
  for (size_t cluster_index = 0; cluster_index < current_clusters.size();
       ++cluster_index) {
    // The display path carries a byte-sized color id. Reserve zero for
    // points without a cluster and wrap only after the available ids end.
    const unsigned char color_id = static_cast<unsigned char>(
        (cluster_index % 254) + 1);
    for (const int index : current_clusters[cluster_index].indices) {
      if (index < 0 || static_cast<size_t>(index) >= current->size())
        continue;
      clustered_point[static_cast<size_t>(index)] = true;
      cluster_color_index[static_cast<size_t>(index)] = color_id;
    }
  }
  std::vector<M_PointXYZI> clustered_cloud;
  clustered_cloud.reserve(current->size());
  for (size_t i = 0; i < current->size(); ++i) {
    if (clustered_point[i]) {
      const PointType& point = current->points[i];
      clustered_cloud.push_back(
          {point.x, point.y, point.z, cluster_color_index[i]});
    }
  }

  // A gap or navigation reset can leave no previous cloud while an older
  // track list is still populated.  Track indices are only meaningful for
  // the cloud they were extracted from, so discard both pieces of state
  // before attempting registration.  This also makes the first frame after
  // a reset follow the normal track-initialization path below.
  if (!previous || previous->empty()) {
    dynamic_cluster_tracks_.clear();
    confirmed_dynamic_memory_.clear();
  }

  // The first frame only initializes cluster tracks. There is no prior frame
  // against which a motion residual can be computed yet.
  if (dynamic_cluster_tracks_.empty() && previous && !previous->empty())
    dynamic_cluster_tracks_ =
        extractClusters(previous, large_cluster_min_points);
  for (Cluster& cluster : dynamic_cluster_tracks_) {
    if (cluster.track_id == 0)
      cluster.track_id = next_dynamic_cluster_track_id_++;
  }
  if (dynamic_cluster_tracks_.empty()) {
    const Eigen::Vector3d nan_centroid =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    for (size_t i = 0; i < current_clusters.size(); ++i) {
      current_clusters[i].track_id = next_dynamic_cluster_track_id_++;
      WriteDynamicClusterLog(
          timestamp, current_clusters[i].track_id, static_cast<int>(i), -1,
          current_clusters[i].indices.size(), current_clusters[i].centroid,
          nan_centroid, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), 0, false, false, false,
          "initialized");
    }
    dynamic_cluster_tracks_ = current_clusters;
    std::lock_guard<std::mutex> lock(display_cloud_mutex_);
    last_clustered_cloud_ = clustered_cloud;
    last_dynamic_display_cloud_.clear();
    last_static_cloud_.clear();
    last_static_cloud_.reserve(current->size());
    for (size_t i = 0; i < current->size(); ++i) {
      if (clustered_point[i]) {
        const PointType& point = current->points[i];
        last_static_cloud_.push_back(
            {point.x, point.y, point.z,
             static_cast<unsigned char>(point.intensity)});
      }
    }
    WriteLeftmostClusterLog(
        timestamp, true, leftmost_centroid_body,
        leftmost_centroid_offset, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), false);
    return current;
  }

  auto transformedBounds = [](const Cluster& cluster,
                              const Eigen::Isometry3d& transform) {
    Eigen::Vector3d min_bound = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_bound = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    for (int ix = 0; ix <= 1; ++ix) {
      for (int iy = 0; iy <= 1; ++iy) {
        for (int iz = 0; iz <= 1; ++iz) {
          const Eigen::Vector3d corner(
              ix ? cluster.max_bound.x() : cluster.min_bound.x(),
              iy ? cluster.max_bound.y() : cluster.min_bound.y(),
              iz ? cluster.max_bound.z() : cluster.min_bound.z());
          const Eigen::Vector3d transformed = transform * corner;
          min_bound = min_bound.cwiseMin(transformed);
          max_bound = max_bound.cwiseMax(transformed);
        }
      }
    }
    return std::pair<Eigen::Vector3d, Eigen::Vector3d>(min_bound, max_bound);
  };

  auto overlapRatio = [](const Eigen::Vector3d& min_a,
                         const Eigen::Vector3d& max_a,
                         const Eigen::Vector3d& min_b,
                         const Eigen::Vector3d& max_b) {
    const Eigen::Vector3d overlap =
        (max_a.cwiseMin(max_b) - min_a.cwiseMax(min_b)).cwiseMax(0.0);
    const double intersection = overlap.x() * overlap.y() * overlap.z();
    const Eigen::Vector3d size_a = (max_a - min_a).cwiseMax(0.05);
    const Eigen::Vector3d size_b = (max_b - min_b).cwiseMax(0.05);
    const double volume_a = size_a.x() * size_a.y() * size_a.z();
    const double volume_b = size_b.x() * size_b.y() * size_b.z();
    return intersection / std::max(1.0e-6, std::min(volume_a, volume_b));
  };

  auto axisAlignedBoxGap = [](const Eigen::Vector3d& min_a,
                              const Eigen::Vector3d& max_a,
                              const Eigen::Vector3d& min_b,
                              const Eigen::Vector3d& max_b) {
    const Eigen::Vector3d gap =
        (min_b - max_a).cwiseMax(0.0) + (min_a - max_b).cwiseMax(0.0);
    return gap.norm();
  };

  std::vector<bool> used(current_clusters.size(), false);
  std::vector<Cluster> next_tracks = current_clusters;
  for (Cluster& cluster : next_tracks)
    cluster.track_id = next_dynamic_cluster_track_id_++;
  std::vector<bool> remove_cluster(current_clusters.size(), false);
  // A recovered vessel can be split into several components by occlusion.
  // Fragment recoveries inherit its label but must not replace the primary
  // component in the velocity/memory state below.
  std::vector<bool> memory_fragment_recovery(current_clusters.size(), false);
  std::vector<uint64_t> newly_confirmed_track_ids;
  double largest_residual = 0.0;
  bool has_residual = false;

  struct RegistrationResult {
    double rmse = std::numeric_limits<double>::infinity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    size_t inliers = 0;
  };

  // Match cluster geometry, not cluster centroids. The visible part of a
  // vessel can change between scans, so identity is based on robust nearest
  // point correspondences after the navigation transform.
  auto registerCluster = [&](const Cluster& previous_cluster,
                             const Cluster& current_cluster,
                             const Eigen::Isometry3d& transform) {
    RegistrationResult result;
    // A rejected/invalid pose must not be allowed to create a NaN query for
    // PCL's kd-tree. Treat this cluster as unmatched instead.
    if (!transform.matrix().allFinite())
      return result;
    PointCloud::Ptr current_points(new PointCloud);
    const size_t current_step = std::max<size_t>(
        1, (current_cluster.indices.size() + kDynamicRegistrationMaxPoints - 1) /
               kDynamicRegistrationMaxPoints);
    for (size_t offset = 0; offset < current_cluster.indices.size();
         offset += current_step) {
      const int index = current_cluster.indices[offset];
      if (index < 0 || static_cast<size_t>(index) >= current->size())
        continue;
      current_points->push_back(current->points[static_cast<size_t>(index)]);
    }
    if (current_points->empty())
      return result;

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(current_points);
    std::vector<int> sampled_previous_indices;
    const size_t previous_step = std::max<size_t>(
        1, (previous_cluster.indices.size() + kDynamicRegistrationMaxPoints - 1) /
               kDynamicRegistrationMaxPoints);
    for (size_t offset = 0; offset < previous_cluster.indices.size();
         offset += previous_step) {
      const int index = previous_cluster.indices[offset];
      if (index < 0 || static_cast<size_t>(index) >= previous->size())
        continue;
      sampled_previous_indices.push_back(index);
    }
    if (sampled_previous_indices.empty())
      return result;
    Eigen::Vector3d correction = Eigen::Vector3d::Zero();
    size_t correspondence_count = 0;
    const float max_correspondence_squared = 1.5F * 1.5F;
    std::vector<int> nearest(1);
    std::vector<float> squared_distance(1);
    for (const int index : sampled_previous_indices) {
      const PointType& point = previous->points[static_cast<size_t>(index)];
      const Eigen::Vector3d predicted =
          transform * Eigen::Vector3d(point.x, point.y, point.z);
      if (!predicted.allFinite())
        continue;
      PointType query;
      query.x = static_cast<float>(predicted.x());
      query.y = static_cast<float>(predicted.y());
      query.z = static_cast<float>(predicted.z());
      query.intensity = point.intensity;
      if (tree.nearestKSearch(query, 1, nearest, squared_distance) > 0 &&
          nearest[0] >= 0 &&
          static_cast<size_t>(nearest[0]) < current_points->size() &&
          squared_distance[0] <= max_correspondence_squared) {
        const PointType& match = current_points->points[
            static_cast<size_t>(nearest[0])];
        correction += Eigen::Vector3d(match.x, match.y, match.z) - predicted;
        ++correspondence_count;
      }
    }
    const size_t minimum_correspondences = std::max<size_t>(
        8, static_cast<size_t>(std::ceil(sampled_previous_indices.size() * 0.35)));
    if (correspondence_count < minimum_correspondences)
      return result;
    correction /= static_cast<double>(correspondence_count);

    double squared_error = 0.0;
    size_t inliers = 0;
    for (const int index : sampled_previous_indices) {
      const PointType& point = previous->points[static_cast<size_t>(index)];
      const Eigen::Vector3d predicted =
          transform * Eigen::Vector3d(point.x, point.y, point.z) + correction;
      if (!predicted.allFinite())
        continue;
      PointType query;
      query.x = static_cast<float>(predicted.x());
      query.y = static_cast<float>(predicted.y());
      query.z = static_cast<float>(predicted.z());
      query.intensity = point.intensity;
      if (tree.nearestKSearch(query, 1, nearest, squared_distance) > 0 &&
          nearest[0] >= 0 &&
          static_cast<size_t>(nearest[0]) < current_points->size() &&
          squared_distance[0] <= max_correspondence_squared) {
        squared_error += static_cast<double>(squared_distance[0]);
        ++inliers;
      }
    }
    if (inliers < minimum_correspondences)
      return result;
    result.rmse =
        std::sqrt(squared_error / static_cast<double>(inliers));
    result.translation = correction;
    result.inliers = inliers;
    return result;
  };

  std::vector<std::vector<RegistrationResult>> registration_scores(
      dynamic_cluster_tracks_.size(),
      std::vector<RegistrationResult>(current_clusters.size()));
  for (size_t previous_index = 0;
       previous_index < dynamic_cluster_tracks_.size(); ++previous_index) {
    const Cluster& previous_cluster = dynamic_cluster_tracks_[previous_index];
    Eigen::Isometry3d predicted_transform = previous_to_current;
    predicted_transform.translation() +=
        previous_cluster.relative_velocity * frame_dt;
    const auto predicted_bounds =
        transformedBounds(previous_cluster, predicted_transform);
    for (size_t current_index = 0; current_index < current_clusters.size();
         ++current_index) {
      const Cluster& current_cluster = current_clusters[current_index];
      const double point_ratio =
          static_cast<double>(std::min(previous_cluster.indices.size(),
                                       current_cluster.indices.size())) /
          static_cast<double>(std::max(previous_cluster.indices.size(),
                                       current_cluster.indices.size()));
      if (point_ratio >= 0.15)
        if (axisAlignedBoxGap(predicted_bounds.first, predicted_bounds.second,
                              current_cluster.min_bound,
                              current_cluster.max_bound) <=
            kDynamicRegistrationMaxBoxGap)
          registration_scores[previous_index][current_index] = registerCluster(
              previous_cluster, current_cluster, predicted_transform);
    }
  }

  auto bestCandidateScore = [&](size_t previous_index) {
    double best = std::numeric_limits<double>::infinity();
    for (const RegistrationResult& registration :
         registration_scores[previous_index]) {
      if (registration.inliers > 0)
        best = std::min(best, registration.rmse);
    }
    return best;
  };
  std::vector<size_t> previous_order(dynamic_cluster_tracks_.size());
  std::vector<bool> previous_matched(dynamic_cluster_tracks_.size(), false);
  std::iota(previous_order.begin(), previous_order.end(), 0);
  std::stable_sort(previous_order.begin(), previous_order.end(),
                   [&](size_t a, size_t b) {
                     return bestCandidateScore(a) < bestCandidateScore(b);
                   });

  for (const size_t previous_index : previous_order) {
    const Cluster& previous_cluster = dynamic_cluster_tracks_[previous_index];
    Eigen::Isometry3d predicted_transform = previous_to_current;
    predicted_transform.translation() +=
        previous_cluster.relative_velocity * frame_dt;
    const Eigen::Vector3d predicted_centroid =
        predicted_transform * previous_cluster.centroid;
    const auto predicted_bounds =
        transformedBounds(previous_cluster, predicted_transform);

    int best_index = -1;
    double best_score = std::numeric_limits<double>::infinity();
    RegistrationResult best_registration;
    for (size_t i = 0; i < current_clusters.size(); ++i) {
      if (used[i])
        continue;
      const RegistrationResult& registration = registration_scores[previous_index][i];
      if (registration.inliers > 0 && registration.rmse < best_score) {
        best_index = static_cast<int>(i);
        best_score = registration.rmse;
        best_registration = registration;
      }
    }
    if (best_index < 0) {
      const Eigen::Vector3d nan_centroid =
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
      WriteDynamicClusterLog(
          timestamp, previous_cluster.track_id, -1,
          static_cast<int>(previous_index), previous_cluster.indices.size(),
          nan_centroid, predicted_centroid,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          previous_cluster.suspicious_frames, false, false, false,
          "unmatched_previous");
      continue;  // Conservative: unmatched clusters remain in the output.
    }

    used[static_cast<size_t>(best_index)] = true;
    previous_matched[previous_index] = true;
    Cluster& next = next_tracks[static_cast<size_t>(best_index)];
    next.track_id = previous_cluster.track_id;
    next.dynamic_confirmed = previous_cluster.dynamic_confirmed;
    next.last_motion_direction = previous_cluster.last_motion_direction;
    next.relative_velocity = previous_cluster.relative_velocity;
    next.consistent_motion_frames = previous_cluster.consistent_motion_frames;
    std::deque<std::vector<M_PointXYZI>> current_history =
        std::move(next.world_history);
    next.world_history = previous_cluster.world_history;
    for (auto& points : current_history)
      next.world_history.push_back(std::move(points));
    while (next.world_history.size() > kDynamicWorldHistoryFrames)
      next.world_history.pop_front();
    const double overlap = overlapRatio(
        predicted_bounds.first, predicted_bounds.second,
        current_clusters[static_cast<size_t>(best_index)].min_bound,
        current_clusters[static_cast<size_t>(best_index)].max_bound);
    const Eigen::Vector3d total_motion =
        previous_cluster.relative_velocity * frame_dt +
        best_registration.translation;
    const double movement_residual = total_motion.norm();
    const double movement_speed = movement_residual / frame_dt;
    bool moving = true;
    if (movement_residual <=
        usv::slam_config::kDynamicStaticMotionDistance) {
      moving = false;
      next.relative_velocity *= 0.5;
    } else {
      const Eigen::Vector3d measured_relative_velocity =
          total_motion / frame_dt;
      next.relative_velocity = 0.5 * previous_cluster.relative_velocity +
                               0.5 * measured_relative_velocity;
      const double relative_speed = next.relative_velocity.norm();
      if (relative_speed > kMaximumTrackedRelativeSpeedMps)
        next.relative_velocity *=
            kMaximumTrackedRelativeSpeedMps / relative_speed;
      if (movement_speed >= kConsistentMotionMinSpeedMps) {
        const Eigen::Vector3d direction =
            total_motion / movement_residual;
        const bool same_direction =
            previous_cluster.last_motion_direction.norm() > 0.5 &&
            previous_cluster.last_motion_direction.dot(direction) >= 0.5;
        next.consistent_motion_frames =
            same_direction ? previous_cluster.consistent_motion_frames + 1 : 1;
        next.last_motion_direction = direction;
      } else {
        next.consistent_motion_frames = std::max(
            0, previous_cluster.consistent_motion_frames - 1);
      }
      const bool consistent_motion =
          next.consistent_motion_frames >= kConsistentMotionFrames;
      moving = movement_speed > usv::slam_config::kDynamicSpeedThreshold ||
               consistent_motion;
    }
    if (static_cast<size_t>(best_index) == leftmost_index) {
      leftmost_overlap = overlap;
      leftmost_residual = movement_residual;
      leftmost_dynamic = moving;
    }
    if (moving) {
      next.suspicious_frames = std::min(
          kDynamicConfirmationFrames,
          previous_cluster.suspicious_frames + 1);
      has_residual = true;
      largest_residual = std::max(largest_residual, movement_residual);
    } else {
      next.suspicious_frames = std::max(0, previous_cluster.suspicious_frames - 1);
    }
    // Dynamic classification is deliberately one-way. Once a track has
    // provided enough consecutive motion evidence, later low-motion frames
    // are treated as the same already-confirmed object rather than allowing
    // it to re-enter the static scene.
    if (next.suspicious_frames >= kDynamicConfirmationFrames &&
        !previous_cluster.dynamic_confirmed) {
      next.dynamic_confirmed = true;
      newly_confirmed_track_ids.push_back(next.track_id);
    }
    const bool confirmed_moving = next.dynamic_confirmed;
    if (static_cast<size_t>(best_index) == leftmost_index)
      leftmost_dynamic = confirmed_moving;
    if (LIDAR_GNSS_SLAM_DYNAMIC_FILTER_ENABLE && confirmed_moving) {
      remove_cluster[static_cast<size_t>(best_index)] = true;
      last_confirmed_dynamic_removed_.store(true, std::memory_order_release);
      ++dynamic_clusters;
      dynamic_points += next.indices.size();
    }
    const bool removed = remove_cluster[static_cast<size_t>(best_index)];
    const char* status = removed
                             ? (moving ? "removed_dynamic"
                                      : "removed_confirmed_static")
                             : (moving ? (confirmed_moving
                                              ? "confirmed_dynamic_not_removed"
                                              : "moving_unconfirmed")
                                       : "matched_static");
    WriteDynamicClusterLog(
        timestamp, next.track_id, best_index,
        static_cast<int>(previous_index), next.indices.size(),
        next.centroid, predicted_centroid, best_score, overlap,
        movement_residual, movement_speed, next.suspicious_frames, moving,
        confirmed_moving,
        removed, status);
  }

  // A different visible face can make geometric registration fail for one
  // scan. Confirmed tracks keep their one-way label. Tentative tracks with
  // two or more motion observations also retain their evidence, but remain
  // in the static output until a later ordinary registration reaches the
  // normal confirmation threshold.
  for (size_t previous_index = 0;
       previous_index < dynamic_cluster_tracks_.size(); ++previous_index) {
    if (previous_matched[previous_index])
      continue;
    const Cluster& previous_cluster = dynamic_cluster_tracks_[previous_index];
    const bool tentative_recovery =
        !previous_cluster.dynamic_confirmed &&
        previous_cluster.suspicious_frames >=
            kTentativeRecoveryMinimumEvidenceFrames;
    if (!previous_cluster.dynamic_confirmed && !tentative_recovery)
      continue;

    Eigen::Isometry3d predicted_transform = previous_to_current;
    predicted_transform.translation() +=
        previous_cluster.relative_velocity * frame_dt;
    const auto predicted_bounds =
        transformedBounds(previous_cluster, predicted_transform);
    int recovered_index = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    for (size_t current_index = 0;
         current_index < current_clusters.size(); ++current_index) {
      if (used[current_index])
        continue;
      const Cluster& candidate = current_clusters[current_index];
      const double point_ratio = static_cast<double>(
          std::min(previous_cluster.indices.size(), candidate.indices.size())) /
          static_cast<double>(
              std::max(previous_cluster.indices.size(), candidate.indices.size()));
      const double min_point_ratio = previous_cluster.dynamic_confirmed
          ? kConfirmedTrackRecoveryMinPointRatio
          : kTentativeTrackRecoveryMinPointRatio;
      const double max_box_gap = previous_cluster.dynamic_confirmed
          ? kConfirmedTrackRecoveryMaxBoxGap
          : kTentativeTrackRecoveryMaxBoxGap;
      if (point_ratio < min_point_ratio)
        continue;
      const double gap = axisAlignedBoxGap(
          predicted_bounds.first, predicted_bounds.second,
          candidate.min_bound, candidate.max_bound);
      if (gap <= max_box_gap && gap < best_gap) {
        recovered_index = static_cast<int>(current_index);
        best_gap = gap;
      }
    }
    if (recovered_index < 0)
      continue;

    const size_t current_index = static_cast<size_t>(recovered_index);
    used[current_index] = true;
    Cluster& next = next_tracks[current_index];
    std::deque<std::vector<M_PointXYZI>> current_history =
        std::move(next.world_history);
    next.track_id = previous_cluster.track_id;
    next.dynamic_confirmed = previous_cluster.dynamic_confirmed;
    next.suspicious_frames = previous_cluster.dynamic_confirmed
        ? kDynamicConfirmationFrames : previous_cluster.suspicious_frames;
    next.last_motion_direction = previous_cluster.last_motion_direction;
    next.relative_velocity = previous_cluster.relative_velocity;
    next.consistent_motion_frames = previous_cluster.consistent_motion_frames;
    next.world_history = previous_cluster.world_history;
    for (auto& points : current_history)
      next.world_history.push_back(std::move(points));
    while (next.world_history.size() > kDynamicWorldHistoryFrames)
      next.world_history.pop_front();

    if (next.dynamic_confirmed) {
      remove_cluster[current_index] = true;
      last_confirmed_dynamic_removed_.store(true, std::memory_order_release);
      ++dynamic_clusters;
      dynamic_points += next.indices.size();
    }
    WriteDynamicClusterLog(
        timestamp, next.track_id, recovered_index,
        static_cast<int>(previous_index), next.indices.size(), next.centroid,
        predicted_transform * previous_cluster.centroid,
        std::numeric_limits<double>::quiet_NaN(),
        overlapRatio(predicted_bounds.first, predicted_bounds.second,
                     next.min_bound, next.max_bound),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), next.suspicious_frames,
        true, next.dynamic_confirmed, next.dynamic_confirmed,
        next.dynamic_confirmed ? "removed_confirmed_reassociated"
                               : "reassociated_tentative");
  }

  // Keep a confirmed target alive across a short occlusion or a sequence of
  // changing visible surfaces. This only considers tracks that were already
  // confirmed and requires a tight world-space prediction overlap.
  for (const ConfirmedDynamicMemory& memory : confirmed_dynamic_memory_) {
    const double memory_dt = std::max(0.0, timestamp - memory.timestamp);
    const Eigen::Vector3d predicted_offset =
        memory.velocity_world * memory_dt;
    const Eigen::Vector3d predicted_min =
        memory.min_bound_world + predicted_offset;
    const Eigen::Vector3d predicted_max =
        memory.max_bound_world + predicted_offset;
    int recovered_index = -1;
    double best_gap = std::numeric_limits<double>::infinity();
    for (size_t current_index = 0;
         current_index < current_clusters.size(); ++current_index) {
      if (used[current_index])
        continue;
      const Cluster& candidate = current_clusters[current_index];
      const double point_ratio = static_cast<double>(
          std::min(memory.point_count, candidate.indices.size())) /
          static_cast<double>(
              std::max(memory.point_count, candidate.indices.size()));
      if (point_ratio < kConfirmedMemoryMinPointRatio)
        continue;
      const auto candidate_bounds = worldBounds(candidate);
      const double gap = axisAlignedBoxGap(
          predicted_min, predicted_max,
          candidate_bounds.first, candidate_bounds.second);
      if (gap <= kConfirmedMemoryMaxBoxGap && gap < best_gap) {
        recovered_index = static_cast<int>(current_index);
        best_gap = gap;
      }
    }
    if (recovered_index < 0)
      continue;

    const size_t current_index = static_cast<size_t>(recovered_index);
    used[current_index] = true;
    Cluster& next = next_tracks[current_index];
    std::deque<std::vector<M_PointXYZI>> current_history =
        std::move(next.world_history);
    next.track_id = memory.track_id;
    next.dynamic_confirmed = true;
    next.suspicious_frames = kDynamicConfirmationFrames;
    next.world_history = memory.world_history;
    for (auto& points : current_history)
      next.world_history.push_back(std::move(points));
    while (next.world_history.size() > kDynamicWorldHistoryFrames)
      next.world_history.pop_front();
    remove_cluster[current_index] = true;
    last_confirmed_dynamic_removed_.store(true, std::memory_order_release);
    ++dynamic_clusters;
    dynamic_points += next.indices.size();
    WriteDynamicClusterLog(
        timestamp, next.track_id, recovered_index, -1, next.indices.size(),
        next.centroid, lidar_pose_in_world.inverse() *
                           (memory.centroid_world + predicted_offset),
        std::numeric_limits<double>::quiet_NaN(),
        overlapRatio(predicted_min, predicted_max,
                     worldBounds(next).first, worldBounds(next).second),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), next.suspicious_frames,
        true, true, true, "removed_confirmed_memory");
  }

  // The normal memory association above intentionally chooses only one main
  // component per track.  Remove additional components only when their whole
  // world-space box lies inside that same predicted object volume.  This is a
  // containment test, not a relaxed nearest-neighbour match, so it handles
  // fragmented returns without broadening association to nearby static map.
  for (const ConfirmedDynamicMemory& memory : confirmed_dynamic_memory_) {
    const double memory_dt = std::max(0.0, timestamp - memory.timestamp);
    const Eigen::Vector3d predicted_offset =
        memory.velocity_world * memory_dt;
    const Eigen::Vector3d allowed_min = memory.min_bound_world +
        predicted_offset - Eigen::Vector3d::Constant(
                               kConfirmedMemoryFragmentContainmentMargin);
    const Eigen::Vector3d allowed_max = memory.max_bound_world +
        predicted_offset + Eigen::Vector3d::Constant(
                               kConfirmedMemoryFragmentContainmentMargin);
    for (size_t current_index = 0;
         current_index < current_clusters.size(); ++current_index) {
      if (used[current_index])
        continue;
      const Cluster& candidate = current_clusters[current_index];
      const auto candidate_bounds = worldBounds(candidate);
      if (!(candidate_bounds.first.array() >= allowed_min.array()).all() ||
          !(candidate_bounds.second.array() <= allowed_max.array()).all()) {
        continue;
      }

      used[current_index] = true;
      Cluster& next = next_tracks[current_index];
      std::deque<std::vector<M_PointXYZI>> current_history =
          std::move(next.world_history);
      next.track_id = memory.track_id;
      next.dynamic_confirmed = true;
      next.suspicious_frames = kDynamicConfirmationFrames;
      next.world_history = memory.world_history;
      for (auto& points : current_history)
        next.world_history.push_back(std::move(points));
      while (next.world_history.size() > kDynamicWorldHistoryFrames)
        next.world_history.pop_front();
      remove_cluster[current_index] = true;
      memory_fragment_recovery[current_index] = true;
      last_confirmed_dynamic_removed_.store(true, std::memory_order_release);
      ++dynamic_clusters;
      dynamic_points += next.indices.size();
      WriteDynamicClusterLog(
          timestamp, next.track_id, static_cast<int>(current_index), -1,
          next.indices.size(), next.centroid, lidar_pose_in_world.inverse() *
              (memory.centroid_world + predicted_offset),
          std::numeric_limits<double>::quiet_NaN(),
          overlapRatio(memory.min_bound_world + predicted_offset,
                       memory.max_bound_world + predicted_offset,
                       candidate_bounds.first, candidate_bounds.second),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), next.suspicious_frames,
          true, true, true, "removed_confirmed_memory_fragment");
    }
  }

  const Eigen::Vector3d nan_centroid =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  for (size_t i = 0; i < current_clusters.size(); ++i) {
    if (used[i])
      continue;
    // This is deliberately recorded for every static-output candidate.  It
    // answers whether a trail is caused by an expired confirmed track, a
    // fragmented point count, or a geometric prediction miss.
    const Cluster& candidate = current_clusters[i];
    const auto candidate_bounds = worldBounds(candidate);
    const ConfirmedDynamicMemory* nearest_memory = nullptr;
    double nearest_ratio = std::numeric_limits<double>::quiet_NaN();
    double nearest_gap = std::numeric_limits<double>::infinity();
    for (const ConfirmedDynamicMemory& memory : confirmed_dynamic_memory_) {
      const double memory_dt = std::max(0.0, timestamp - memory.timestamp);
      const Eigen::Vector3d offset = memory.velocity_world * memory_dt;
      const double ratio = static_cast<double>(
          std::min(memory.point_count, candidate.indices.size())) /
          static_cast<double>(
              std::max(memory.point_count, candidate.indices.size()));
      const double gap = axisAlignedBoxGap(
          memory.min_bound_world + offset, memory.max_bound_world + offset,
          candidate_bounds.first, candidate_bounds.second);
      if (gap < nearest_gap) {
        nearest_memory = &memory;
        nearest_ratio = ratio;
        nearest_gap = gap;
      }
    }
    const Eigen::Vector3d centroid_world =
        lidar_pose_in_world * candidate.centroid;
    const char* memory_decision = "no_confirmed_memory";
    uint64_t memory_track_id = 0;
    int memory_remaining_frames = -1;
    if (nearest_memory) {
      memory_track_id = nearest_memory->track_id;
      memory_remaining_frames = nearest_memory->remaining_frames;
      memory_decision = nearest_ratio < kConfirmedMemoryMinPointRatio
                            ? "rejected_point_ratio"
                            : "rejected_bbox_gap";
    }
    WriteDynamicMemoryDecisionLog(
        timestamp, candidate.indices.size(), centroid_world, memory_track_id,
        memory_remaining_frames, nearest_ratio, nearest_gap, memory_decision);
    WriteDynamicClusterLog(
        timestamp, next_tracks[i].track_id, static_cast<int>(i), -1,
        current_clusters[i].indices.size(), current_clusters[i].centroid,
        nan_centroid, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), 0, false, false, false,
        "unmatched_current");
  }

  PointCloud::Ptr static_cloud(new PointCloud);
  // Small connected components are always removed, independently of motion
  // tracking. They are isolated returns/noise rather than stable scene
  // geometry for this filtering stage.
  std::vector<bool> remove_point = small_cluster_point;
  std::vector<M_PointXYZI> removed_cloud;
  removed_cloud.reserve(current->size());
  for (size_t i = 0; i < current->size(); ++i) {
    if (!small_cluster_point[i])
      continue;
    const PointType& point = current->points[i];
    removed_cloud.push_back(
        {point.x, point.y, point.z, static_cast<unsigned char>(point.intensity)});
  }
  for (size_t i = 0; i < current_clusters.size(); ++i) {
    if (remove_cluster[i]) {
      for (const int index : current_clusters[i].indices) {
        remove_point[static_cast<size_t>(index)] = true;
        const PointType& point = current->points[static_cast<size_t>(index)];
        removed_cloud.push_back(
            {point.x, point.y, point.z, static_cast<unsigned char>(point.intensity)});
      }
    }
  }
  static_cloud->reserve(current->size());
  std::vector<M_PointXYZI> static_display_cloud;
  static_display_cloud.reserve(current->size() - dynamic_points);
  for (size_t i = 0; i < current->size(); ++i) {
    // Clustering is only used to identify removable dynamic objects. Points
    // that are not part of a sufficiently dense cluster remain valid scene
    // geometry and must stay in the registration cloud.
    if (!remove_point[i]) {
      static_cloud->push_back(current->points[i]);
      const PointType& point = current->points[i];
      static_display_cloud.push_back(
          {point.x, point.y, point.z,
           static_cast<unsigned char>(point.intensity)});
    }
  }
  static_cloud->width = static_cast<uint32_t>(static_cloud->size());
  static_cloud->height = 1;
  static_cloud->is_dense = false;
  dynamic_cluster_tracks_ = std::move(next_tracks);

  for (size_t i = 0; i < current_clusters.size(); ++i) {
    if (!remove_cluster[i] || memory_fragment_recovery[i])
      continue;
    const DynamicClusterTrack& track = dynamic_cluster_tracks_[i];
    const Eigen::Vector3d centroid_world =
        lidar_pose_in_world * current_clusters[i].centroid;
    const auto bounds_world = worldBounds(current_clusters[i]);
    auto memory_it = std::find_if(
        confirmed_dynamic_memory_.begin(), confirmed_dynamic_memory_.end(),
        [&](const ConfirmedDynamicMemory& memory) {
          return memory.track_id == track.track_id;
        });
    if (memory_it == confirmed_dynamic_memory_.end()) {
      confirmed_dynamic_memory_.push_back({});
      memory_it = std::prev(confirmed_dynamic_memory_.end());
      memory_it->track_id = track.track_id;
    }
    const double dt = timestamp - memory_it->timestamp;
    if (dt > kMinFilterDt) {
      const Eigen::Vector3d measured_velocity =
          (centroid_world - memory_it->centroid_world) / dt;
      if (measured_velocity.allFinite() && measured_velocity.norm() <=
                                             kMaximumTrackedRelativeSpeedMps) {
        memory_it->velocity_world = measured_velocity;
      }
    }
    memory_it->point_count = track.indices.size();
    memory_it->centroid_world = centroid_world;
    memory_it->min_bound_world = bounds_world.first;
    memory_it->max_bound_world = bounds_world.second;
    memory_it->timestamp = timestamp;
    memory_it->remaining_frames = kConfirmedMemoryFrames;
    memory_it->world_history = track.world_history;
  }
  {
    std::lock_guard<std::mutex> lock(display_cloud_mutex_);
    last_clustered_cloud_ = std::move(clustered_cloud);
    last_dynamic_display_cloud_ = std::move(removed_cloud);
    last_confirmed_dynamic_cloud_.clear();
    last_confirmed_dynamic_history_world_.clear();
    for (size_t i = 0; i < current_clusters.size(); ++i) {
      if (!remove_cluster[i])
        continue;
      const DynamicClusterTrack& track = dynamic_cluster_tracks_[i];
      for (const int index : current_clusters[i].indices) {
        const PointType& point = current->points[static_cast<size_t>(index)];
        last_confirmed_dynamic_cloud_.push_back(
            {point.x, point.y, point.z,
             static_cast<unsigned char>(point.intensity)});
      }
      if (std::find(newly_confirmed_track_ids.begin(),
                    newly_confirmed_track_ids.end(), track.track_id) !=
          newly_confirmed_track_ids.end()) {
        for (const auto& history_points : track.world_history)
          last_confirmed_dynamic_history_world_.push_back(history_points);
      }
    }
    last_static_cloud_ = std::move(static_display_cloud);
  }
  last_dynamic_confirmation_.store(!newly_confirmed_track_ids.empty(),
                                   std::memory_order_release);
  if (has_residual)
    max_residual = largest_residual;
  WriteLeftmostClusterLog(
      timestamp, true, leftmost_centroid_body,
      leftmost_centroid_offset, leftmost_overlap, leftmost_residual,
      leftmost_dynamic);
  return static_cloud;
}

LidarFrame LidarGnssSlam::transformLidarFrameToWorld(
    const LidarFrame& lidar, const Eigen::Isometry3d& pose) const {
  LidarFrame world_lidar = lidar;
  world_lidar.timestamp = lidar.timestamp;
  world_lidar.point_count = static_cast<uint32_t>(world_lidar.points.size());

  for (auto& point : world_lidar.points) {
    const Eigen::Vector3d local_point(point.x, point.y, point.z);
    const Eigen::Vector3d world_point = pose * local_point;
    point.x = static_cast<float>(world_point.x());
    point.y = static_cast<float>(world_point.y());
    point.z = static_cast<float>(world_point.z());
  }

  return world_lidar;
}

double LidarGnssSlam::computeOdomWeight(size_t num_points) const {
  const int n_low = std::max(1, kOdomWeightPointsLow);
  const int n_high = std::max(n_low + 1, kOdomWeightPointsHigh);

  if (static_cast<int>(num_points) <= n_low) {
    return kOdomWeightMax;
  }
  if (static_cast<int>(num_points) >= n_high) {
    return kOdomWeightMin;
  }

  const double t =
      (static_cast<double>(num_points) - static_cast<double>(n_low)) /
      (static_cast<double>(n_high) - static_cast<double>(n_low));
  return kOdomWeightMax + (kOdomWeightMin - kOdomWeightMax) * t;
}

bool LidarGnssSlam::interpolateOdom(
    double timestamp, OdomData& odom,
    double* nearest_source_time_error_sec) const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (nearest_source_time_error_sec) {
    *nearest_source_time_error_sec =
        std::numeric_limits<double>::infinity();
  }
  if (odom_queue_.empty()) {
    return false;
  }

  auto upper = std::lower_bound(
      odom_queue_.begin(), odom_queue_.end(), timestamp,
      [](const OdomData& sample, double value) {
        return sample.timestamp < value;
      });
  if (upper == odom_queue_.begin()) {
    odom = *upper;
    if (nearest_source_time_error_sec)
      *nearest_source_time_error_sec = std::abs(odom.timestamp - timestamp);
    return true;
  }
  if (upper == odom_queue_.end()) {
    odom = odom_queue_.back();
    if (nearest_source_time_error_sec)
      *nearest_source_time_error_sec = std::abs(odom.timestamp - timestamp);
    return true;
  }

  const OdomData& after = *upper;
  const OdomData& before = *(upper - 1);
  const double span = after.timestamp - before.timestamp;
  if (span <= 1.0e-9) {
    odom = before;
    if (nearest_source_time_error_sec)
      *nearest_source_time_error_sec = std::abs(odom.timestamp - timestamp);
    return true;
  }

  const double alpha = std::clamp((timestamp - before.timestamp) / span,
                                  0.0, 1.0);
  Eigen::Quaterniond q_before(before.pose.rotation());
  Eigen::Quaterniond q_after(after.pose.rotation());
  if (q_before.dot(q_after) < 0.0) {
    q_after.coeffs() *= -1.0;
  }
  odom.timestamp = timestamp;
  odom.pose = Eigen::Isometry3d::Identity();
  odom.pose.translation() = (1.0 - alpha) * before.pose.translation() +
                            alpha * after.pose.translation();
  odom.pose.linear() =
      q_before.slerp(alpha, q_after).normalized().toRotationMatrix();
  if (nearest_source_time_error_sec) {
    *nearest_source_time_error_sec = std::min(
        std::abs(timestamp - before.timestamp),
        std::abs(after.timestamp - timestamp));
  }
  return true;
}

void LidarGnssSlam::trimOdomQueue(double min_timestamp) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  while (odom_queue_.size() > kMinOdomSamplesToTrim &&
         odom_queue_[1].timestamp < min_timestamp) {
    odom_queue_.pop_front();
  }
}

}  // namespace usv


#pragma once
#ifndef LIDAR_GNSS_SLAM_H
#define LIDAR_GNSS_SLAM_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <nano_gicp/nano_gicp.h>
#include <thread>
#include <vector>

#include "bridge_common_types.h"
#include "common/slam_types.h"

namespace usv {

class LidarGnssSlam {
 public:
  explicit LidarGnssSlam(int gicp_num_threads = 4,
                         bool require_gnss_ins = true,
                         double max_gnss_sync_sec = 0.5,
                         double voxel_leaf_size = 0.5);
  ~LidarGnssSlam();

  void addGnssInsData(const GnssInsMessage& gnss_ins);
  void addLidarData(const std::shared_ptr<const LidarFrame>& lidar);

  PoseState getPose() const;
  double getLastGicpScore() const { return last_gicp_score_; }
  bool wasLastPoseLidarAccepted() const {
    return last_pose_lidar_accepted_.load(std::memory_order_acquire);
  }
  std::vector<M_PointXYZI> getLastClusteredCloud() const;
  std::vector<M_PointXYZI> getLastDynamicDisplayCloud() const;
  std::vector<M_PointXYZI> getLastConfirmedDynamicCloud() const;
  std::vector<std::vector<M_PointXYZI>>
  getLastConfirmedDynamicHistoryWorld() const;
  std::vector<M_PointXYZI> getLastStaticCloud() const;
  // True only when this frame removed a track that had reached the dynamic
  // confirmation threshold. Small-cluster removal does not set this flag.
  bool wasLastConfirmedDynamicRemoved() const {
    return last_confirmed_dynamic_removed_.load(std::memory_order_acquire);
  }
  // True only on the frame where a track first reaches the confirmation
  // threshold. Offline replay uses this one-shot event for point-level
  // historical cleanup; already-confirmed tracks must not repeatedly widen
  // that cleanup window.
  bool wasLastDynamicConfirmation() const {
    return last_dynamic_confirmation_.load(std::memory_order_acquire);
  }
  LidarFrame getLatestWorldLidarFrame() const;
  double getLastProcessedLidarTimestamp() const;
  size_t getPendingLidarCount() const;
  size_t getOdomQueueSize() const;
  SlamGeoPoseReference getLastNavigationReference() const;
  uint64_t getSkippedNoGnssCount() const { return skipped_no_gnss_count_.load(); }

 private:
  const bool require_gnss_ins_;
  const double max_gnss_sync_sec_;
  const double voxel_leaf_size_;

  using PointType = pcl::PointXYZI;
  using PointCloud = pcl::PointCloud<PointType>;

  void slamLoop();
  void processLidarFrame(const LidarFrame& lidar);

  PointCloud::Ptr convertCloud(const LidarFrame& frame) const;
  PointCloud::Ptr downsampleCloud(const PointCloud::Ptr& cloud) const;
  PointCloud::Ptr removeDynamicPoints(const PointCloud::Ptr& current,
                                      const PointCloud::Ptr& previous,
                                      const Eigen::Isometry3d& previous_to_current,
                                      int& dynamic_clusters,
                                      size_t& dynamic_points,
                                      double& max_residual,
                                      double timestamp,
                                      const Eigen::Isometry3d& lidar_pose_in_world);
  LidarFrame transformLidarFrameToWorld(const LidarFrame& lidar,
                                        const Eigen::Isometry3d& pose) const;
  double computeOdomWeight(size_t num_points) const;
  bool interpolateOdom(double timestamp, OdomData& odom,
                       double* nearest_source_time_error_sec = nullptr) const;
  void trimOdomQueue(double min_timestamp);
  OdomData convertGnssInsToOdom(const GnssInsMessage& gnss_ins);
  void enqueueOdomData(const OdomData& odom);
  bool fallbackOdom(double lidar_timestamp, OdomData& odom);
  void markLidarProcessed(double lidar_timestamp);

  mutable std::mutex lidar_mutex_;
  std::condition_variable lidar_cv_;
  std::deque<LidarFrame> lidar_queue_;

  mutable std::mutex odom_mutex_;
  std::deque<OdomData> odom_queue_;

  mutable std::mutex navigation_reference_mutex_;
  SlamGeoPoseReference last_navigation_reference_;

  mutable std::mutex state_mutex_;
  PoseState latest_pose_;
  LidarFrame latest_world_lidar_frame_;
  bool initialized_ = false;
  OdomData last_lidar_odom_;
  bool has_last_lidar_odom_ = false;
  double last_lidar_timestamp_ = 0.0;
  bool has_last_gnss_ins_ = false;
  GnssInsMessage last_gnss_ins_;
  Eigen::Vector3d accumulated_enu_ = Eigen::Vector3d::Zero();
  Eigen::Isometry3d lidar_211_to_body_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lidar_210_to_211_ = Eigen::Isometry3d::Identity();

  // Constant-velocity prediction state.  The pose is kept on SE(3); the
  // velocities are expressed in the lidar frame at the last update.
  Eigen::Vector3d linear_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
  // Error-state covariance: [position, rotation, linear velocity,
  // angular velocity].
  Eigen::Matrix<double, 12, 12> state_covariance_ =
      Eigen::Matrix<double, 12, 12>::Identity();
  bool has_filter_state_ = false;

  PointCloud::Ptr last_scan_;
  // A filtered-out frame must not leave an old scan as a valid registration
  // reference across a turn.
  double last_scan_timestamp_ = 0.0;
  bool has_scan_reference_ = false;
  bool reanchor_scan_reference_ = false;
  mutable std::mutex display_cloud_mutex_;
  std::vector<M_PointXYZI> last_clustered_cloud_;
  std::vector<M_PointXYZI> last_dynamic_display_cloud_;
  std::vector<M_PointXYZI> last_confirmed_dynamic_cloud_;
  std::vector<std::vector<M_PointXYZI>>
      last_confirmed_dynamic_history_world_;
  std::vector<M_PointXYZI> last_static_cloud_;

  struct DynamicClusterTrack {
    uint64_t track_id = 0;
    std::vector<int> indices;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d min_bound = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bound = Eigen::Vector3d::Zero();
    int suspicious_frames = 0;
    bool dynamic_confirmed = false;
    Eigen::Vector3d last_motion_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d relative_velocity = Eigen::Vector3d::Zero();
    int consistent_motion_frames = 0;
    std::deque<std::vector<M_PointXYZI>> world_history;
  };

  struct ConfirmedDynamicMemory {
    uint64_t track_id = 0;
    size_t point_count = 0;
    Eigen::Vector3d centroid_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d min_bound_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bound_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
    double timestamp = 0.0;
    int remaining_frames = 0;
    std::deque<std::vector<M_PointXYZI>> world_history;
  };
  std::vector<DynamicClusterTrack> dynamic_cluster_tracks_;
  std::vector<ConfirmedDynamicMemory> confirmed_dynamic_memory_;
  uint64_t next_dynamic_cluster_track_id_ = 1;

  nano_gicp::NanoGICP<PointType, PointType> gicp_;
  nano_gicp::NanoGICP<PointType, PointType> prereg_gicp_;
  pcl::VoxelGrid<PointType> voxel_filter_;
  double last_gicp_score_ = 0.0;
  std::atomic<bool> last_pose_lidar_accepted_{false};
  PointCloud::Ptr last_unfiltered_scan_;

  std::atomic<bool> running_{false};
  std::thread slam_thread_;
  std::atomic<double> last_processed_lidar_timestamp_{0.0};
  std::atomic<uint64_t> skipped_no_gnss_count_{0};
  std::atomic<bool> last_confirmed_dynamic_removed_{false};
  std::atomic<bool> last_dynamic_confirmation_{false};
};

}  // namespace usv

#endif  // LIDAR_GNSS_SLAM_H



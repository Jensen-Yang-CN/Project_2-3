#include "DloSlamNode.h"
#include "SlamGeoAnchorPolicy.h"
#include "SlamMapBinaryIO.h"
#include "SystemFileLogger.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QDebug>
#include <QMetaObject>

#include "CommunicationManager_v2.h"

namespace usv {

DloSlamNode::DloSlamNode(QObject *parent)
    : QObject(parent) {
}

DloSlamNode::~DloSlamNode() {
    stop();
}

void DloSlamNode::postLog(const QString &text)
{
    QMetaObject::invokeMethod(
        this,
        [this, text]() { emit logMessage(text); },
        Qt::QueuedConnection);
}

void DloSlamNode::postSlamOdometry(const usv::SlamOdometryState &state)
{
    const usv::SlamOdometryState copy = state;
    QMetaObject::invokeMethod(
        this,
        [this, copy]() { emit slamOdometryReady(copy); },
        Qt::QueuedConnection);
}

void DloSlamNode::postSlamKeyframe(const usv::SlamKeyframe &keyframe)
{
    const usv::SlamKeyframe copy = keyframe;
    if (slam_keyframe_topic_)
        slam_keyframe_topic_->publish(std::make_shared<usv::SlamKeyframe>(copy));
    QMetaObject::invokeMethod(
        this,
        [this, copy]() { emit slamKeyframeReady(copy); },
        Qt::QueuedConnection);
}

void DloSlamNode::postSlamScanCloud(QVector<M_PointXYZI> worldCloud)
{
    if (slam_scan_topic_ && !worldCloud.isEmpty()) {
        auto message = std::make_shared<usv::SlamScanCloudMessage>();
        const PoseState poseState = algorithm_ ? algorithm_->getPose() : PoseState{};
        message->timestamp = poseState.timestamp;
        message->points.reserve(static_cast<std::size_t>(worldCloud.size()));
        for (const M_PointXYZI &point : worldCloud)
            message->points.push_back(point);
        slam_scan_topic_->publish(message);
    }
    QMetaObject::invokeMethod(
        this,
        [this, cloud = std::move(worldCloud)]() { emit slamScanCloudReady(cloud); },
        Qt::QueuedConnection);
}

bool DloSlamNode::init(const DloSlamConfig& config) {
    config_ = config;
    {
        std::lock_guard<std::mutex> lock(geo_anchor_mutex_);
        geo_anchor_ = SlamGeoAnchor{};
    }

    WaterSurfaceFilterConfig water_config;
    water_config.enabled = config_.water_filter_enabled;
    water_config.clearance_m = config_.water_clearance_m;
    water_config.plane_distance_m = config_.water_plane_distance_m;
    water_config.max_tilt_deg = config_.water_max_tilt_deg;
    water_config.min_inliers =
        static_cast<std::size_t>(std::max(config_.water_min_inliers, 3));
    water_config.max_sample_points =
        static_cast<std::size_t>(std::max(config_.water_max_sample_points,
                                          config_.water_min_inliers));
    water_config.ransac_iterations = config_.water_ransac_iterations;
    water_config.near_surface_band_m = config_.water_near_surface_band_m;
    water_config.outlier_radius_m = config_.water_outlier_radius_m;
    water_config.outlier_min_neighbors =
        config_.water_outlier_min_neighbors;
    water_config.max_fallback_frames = config_.water_max_fallback_frames;
    water_surface_filter_.setConfig(water_config);

    algorithm_ = std::make_unique<LidarGnssSlam>(
        config_.gicp_num_threads,
        config_.require_gnss_ins,
        config_.max_gnss_sync_sec,
        config_.voxel_leaf_size);

    postLog("DloSlamNode initialized with config:");
    std::ostringstream oss;
    oss << "  max_keyframes: " << config_.max_keyframes
        << ", max_age: " << config_.max_keyframe_age_sec << "s"
        << ", voxel_leaf_size: " << config_.voxel_leaf_size
        << ", gicp_num_threads: " << config_.gicp_num_threads
        << ", require_gnss_ins: " << (config_.require_gnss_ins ? "true" : "false")
        << ", max_gnss_sync_sec: " << config_.max_gnss_sync_sec
        << ", water_filter: " << (config_.water_filter_enabled ? "true" : "false")
        << ", water_clearance_m: " << config_.water_clearance_m;
    postLog(QString::fromStdString(oss.str()));

    SystemFileLogger::instance().logLifecycle("DloSlam", "ready");
    return true;
}

void DloSlamNode::start() {
    if (running_.load()) {
        postLog("DloSlamNode already running");
        return;
    }

    // 订阅融合点云 Topic
    postLog("正在获取 CommunicationManager 实例...");
    auto& comm = CommunicationManager::instance();
    postLog("CommunicationManager 实例获取成功");
    slam_keyframe_topic_ = comm.getStateTopic<SlamKeyframe>("slam/keyframe", 32);
    slam_scan_topic_ = comm.getStateTopic<SlamScanCloudMessage>("slam/scan_cloud", 8);
    
    postLog("正在获取 fused_cloud Topic...");
    fused_cloud_topic_ = comm.getRawTopic<FusedCloudBundle>("/sensor/fused_cloud", 32);
    
    if (!fused_cloud_topic_) {
        postLog("错误: fused_cloud_topic_ 为 nullptr!");
        return;
    }
    
    postLog("fused_cloud Topic 获取成功，正在添加订阅者...");
    fused_subscriber_id_ = fused_cloud_topic_->addSubscriber();
    fused_cloud_topic_->seekSubscriberToLatest(fused_subscriber_id_);
    postLog("订阅者 ID: " + QString::number(fused_subscriber_id_)
            + "（从最新融合点云开始消费）");

    postLog("Subscribed to sensor/fused_cloud topic");

    imu_topic_ = comm.getRawTopic<GnssInsMessage>("/sensor/imu/raw", 64);
    if (imu_topic_) {
        imu_subscriber_id_ = imu_topic_->addSubscriber();
        imu_topic_->seekSubscriberToLatest(imu_subscriber_id_);
        postLog("Subscribed to sensor/imu/raw topic");
        if (auto latest = comm.getLatestState<GnssInsMessage>()) {
            feedGnssIns(*latest);
            postLog("已注入总线最新 GNSS/INS 状态");
        }
    } else {
        postLog("警告: imu_topic_ 为 nullptr，SLAM 将缺少 GNSS/INS");
    }

    running_ = true;
    process_thread_ = std::thread(&DloSlamNode::processLoop, this);
    SystemFileLogger::instance().logLifecycle("DloSlam", "start");
    postLog("DloSlamNode started");
}

void DloSlamNode::stop() {
    if (!running_.load()) {
        return;
    }

    running_ = false;
    queue_cv_.notify_all();

    if (process_thread_.joinable()) {
        process_thread_.join();
    }

    if (fused_cloud_topic_ && fused_subscriber_id_ != 0) {
        fused_cloud_topic_->removeSubscriber(fused_subscriber_id_);
        fused_subscriber_id_ = 0;
    }
    if (imu_topic_ && imu_subscriber_id_ != 0) {
        imu_topic_->removeSubscriber(imu_subscriber_id_);
        imu_subscriber_id_ = 0;
    }
    slam_keyframe_topic_.reset();
    slam_scan_topic_.reset();

    SystemFileLogger::instance().logLifecycle("DloSlam", "stop");
    postLog("DloSlamNode stopped");
}

std::shared_ptr<const SlamOdometryState> DloSlamNode::getLatestOdometry() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return latest_odom_;
}

size_t DloSlamNode::getKeyframeCount() const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return keyframes_.size();
}

std::vector<SlamKeyframe> DloSlamNode::getKeyframes() const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return keyframes_;  // 返回拷贝
}

bool DloSlamNode::saveMap(const std::string& directoryOrFile) {
    std::vector<SlamKeyframe> keyframes_copy = getKeyframes();

    if (keyframes_copy.empty()) {
        postLog("No keyframes to save");
        return false;
    }

    slam_map_io::MapArchive archive;
    archive.anchor = getGeoAnchor();
    archive.keyframes = std::move(keyframes_copy);

    const QString requested = QString::fromStdString(directoryOrFile);
    const QFileInfo requestedInfo(requested);
    QString outputPath;
    if (requestedInfo.suffix().compare(QStringLiteral("slammap"),
                                       Qt::CaseInsensitive) == 0) {
        outputPath = requested;
    } else {
        QDir outputDirectory(requested);
        if (!outputDirectory.mkpath(QStringLiteral("."))) {
            postLog(QStringLiteral("无法创建 SLAM 地图目录: %1")
                        .arg(requested));
            return false;
        }
        outputPath = outputDirectory.filePath(
            QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd_HHmmss_zzz"))
            + QStringLiteral(".slammap"));
    }

    if (!QFileInfo(outputPath).absoluteDir().mkpath(QStringLiteral("."))) {
        postLog(QStringLiteral("无法创建 SLAM 地图输出目录: %1")
                    .arg(QFileInfo(outputPath).absolutePath()));
        return false;
    }

    QString error;
    if (!slam_map_io::saveArchive(outputPath, archive, &error)) {
        postLog(QStringLiteral("保存 .slammap 失败: %1").arg(error));
        return false;
    }

    std::ostringstream oss;
    oss << "Saved " << archive.keyframes.size() << " keyframes to "
        << outputPath.toStdString();
    postLog(QString::fromStdString(oss.str()));

    return true;
}

void DloSlamNode::setOdometryCallback(std::function<void(std::shared_ptr<const SlamOdometryState>)> callback) {
    odometry_callback_ = std::move(callback);
}

void DloSlamNode::setKeyframeCallback(std::function<void(std::shared_ptr<const SlamKeyframe>)> callback) {
    keyframe_callback_ = std::move(callback);
}

bool DloSlamNode::isRunning() const {
    return running_.load();
}

void DloSlamNode::feedGnssIns(const GnssInsMessage& gnss_ins) {
    if (algorithm_) {
        {
            std::lock_guard<std::mutex> lock(geo_anchor_mutex_);
            captureFirstSlamGeoAnchor(gnss_ins, geo_anchor_);
        }
        algorithm_->addGnssInsData(gnss_ins);
    }
}

SlamGeoAnchor DloSlamNode::getGeoAnchor() const
{
    std::lock_guard<std::mutex> lock(geo_anchor_mutex_);
    return geo_anchor_;
}

void DloSlamNode::drainImuBus()
{
    if (!imu_topic_ || imu_subscriber_id_ == 0 || !algorithm_)
        return;

    while (true) {
        auto opt_msg = imu_topic_->tryConsumeNext(imu_subscriber_id_);
        if (!opt_msg || !(*opt_msg))
            break;
        feedGnssIns(**opt_msg);
    }
}

void DloSlamNode::processLoop() {
    postLog("[ProcessLoop] 线程启动，等待数据...");
    
    while (running_) {
        drainImuBus();

        std::shared_ptr<const FusedCloudBundle> bundle;

        // DEBUG: 定期打印等待状态
        static int wait_count = 0;
        
        // 从 RawTopic 消费数据
        if (fused_cloud_topic_) {
            // DEBUG: 添加调试信息
            auto opt_msg = fused_cloud_topic_->tryConsumeNext(fused_subscriber_id_);
            // postLog("[ProcessLoop] tryConsumeNext 返回: " + QString(opt_msg ? "有数据" : "无数据"));
            
            if (opt_msg) {
                bundle = std::dynamic_pointer_cast<const FusedCloudBundle>(*opt_msg);
                // postLog("[ProcessLoop] dynamic_pointer_cast 结果: " + QString(bundle ? "成功" : "失败"));
            }
        } else {
            // postLog("[ProcessLoop] 错误: fused_cloud_topic_ 为 nullptr!");
        }

        if (!bundle) {
            wait_count++;
            if (wait_count % 100 == 0) {
                // 每秒（约100次循环）打印一次等待状态
                // postLog("[ProcessLoop] 等待融合点云数据... (等待计数: " + QString::number(wait_count) + ")");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // DEBUG: 收到数据
        wait_count = 0;

        if (!bundle || bundle->points.empty())
            continue;

        // 间隔下采样 + 按配置剔除近距/远距点
        const double min_r2 = config_.min_range * config_.min_range;
        const double max_r2 = config_.max_range * config_.max_range;
        std::vector<M_PointXYZI> filtered_points;
        filtered_points.reserve(bundle->points.size() / 2 + 1);
        for (size_t i = 0; i < bundle->points.size(); i += 2) {
            const M_PointXYZI &p = bundle->points[i];
            const double dist2 = static_cast<double>(p.x) * p.x
                               + static_cast<double>(p.y) * p.y
                               + static_cast<double>(p.z) * p.z;
            if (dist2 >= min_r2 && dist2 <= max_r2) {
                filtered_points.push_back(p);
            }
        }

        if (filtered_points.empty())
            continue;

        // 这里是点云地图、实时世界点云和 UDP 栅格地图的共同入口。
        // 只过滤一次可保证三个输出一致，也不会改变泊位/桥梁检测的原始点云。
        const std::size_t water_input_count = filtered_points.size();
        WaterSurfaceFilterResult water_result =
            water_surface_filter_.filter(filtered_points);
        filtered_points = std::move(water_result.points);

        if (water_result.plane_detected || water_result.used_fallback_plane) {
            static std::uint64_t water_log_count = 0;
            if (++water_log_count % 30 == 1) {
                postLog(QStringLiteral(
                    "[SLAM水面滤波] 输入=%1 输出=%2 水面Z=%3 删除水面/水下=%4 "
                    "删除孤立点=%5 平面来源=%6")
                            .arg(static_cast<qulonglong>(water_input_count))
                            .arg(static_cast<qulonglong>(filtered_points.size()))
                            .arg(water_result.water_z_at_origin, 0, 'f', 3)
                            .arg(static_cast<qulonglong>(
                                water_result.removed_surface_or_below))
                            .arg(static_cast<qulonglong>(
                                water_result.removed_near_surface_outliers))
                            .arg(water_result.used_fallback_plane
                                     ? QStringLiteral("历史回退")
                                     : QStringLiteral("当前帧")));
            }
        }

        // 过滤后无有效障碍物时不送入 GICP，避免空帧影响配准。
        if (filtered_points.empty())
            continue;

        auto lidar_frame = std::make_shared<LidarFrame>();
        lidar_frame->timestamp = bundle->timestamp;
        lidar_frame->point_count = static_cast<uint32_t>(filtered_points.size());
        lidar_frame->points.reserve(filtered_points.size());
        for (const M_PointXYZI &p : filtered_points) {
            LidarPoint lp;
            lp.x = p.x;
            lp.y = p.y;
            lp.z = p.z;
            lp.intensity = static_cast<float>(p.intensity);
            lidar_frame->points.push_back(lp);
        }

        // GICP 慢于融合点云时限流，避免队列积压导致永远等不到当前帧
        if (algorithm_->getPendingLidarCount() >= 3) {
            static int throttle_log = 0;
            if (++throttle_log % 50 == 1) {
                postLog(QStringLiteral("[SLAM] 算法积压，跳过本帧 (pending=%1)")
                            .arg(algorithm_->getPendingLidarCount()));
            }
            continue;
        }

        algorithm_->addLidarData(lidar_frame);

        bool frame_done = false;
        for (int retry = 0; retry < 80 && running_.load(); ++retry) {
            // GICP 在独立线程运行时仍持续消费高频 GNSS/INS，避免 64 帧
            // 环形缓存被覆盖后只剩跳变的导航样本。
            drainImuBus();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (!running_.load())
                break;

            const double processed_ts = algorithm_->getLastProcessedLidarTimestamp();
            if (processed_ts + 1e-6 >= bundle->timestamp) {
                frame_done = true;
                break;
            }
        }

        if (!running_.load())
            break;

        if (!frame_done) {
            static int timeout_log = 0;
            if (++timeout_log % 10 == 1) {
                const PoseState pose_state = algorithm_->getPose();
                postLog(QStringLiteral(
                    "[SLAM] 处理超时 ts=%1 pose_ts=%2 pending=%3 odom=%4 skip_gnss=%5")
                            .arg(bundle->timestamp, 0, 'f', 3)
                            .arg(pose_state.timestamp, 0, 'f', 3)
                            .arg(algorithm_->getPendingLidarCount())
                            .arg(algorithm_->getOdomQueueSize())
                            .arg(algorithm_->getSkippedNoGnssCount()));
            }
            continue;
        }

        {
            PoseState pose_state = algorithm_->getPose();
            if (pose_state.valid) {
                // 处理完成，构建输出
                uint64_t seq = ++frame_seq_;
                // LidarGnssSlam removes confirmed dynamic clusters internally.
                // Reuse that exact static cloud for both persisted keyframes
                // and the live world scan; feeding filtered_points here would
                // put dynamic objects back into the map/UI.
                const std::vector<M_PointXYZI> static_points =
                    algorithm_->getLastStaticCloud();
                bool is_keyframe = !static_points.empty()
                    && needNewKeyframe(pose_state.pose);

                // postLog("[ProcessLoop] 处理完成! seq=" + QString::number(seq) + 
                //         ", is_keyframe=" + QString(is_keyframe ? "true" : "false"));

                // 构建里程计状态
                SlamOdometryState state = buildOdometryState(
                    pose_state.timestamp, seq, pose_state.pose, is_keyframe);

                // 保存最新里程计
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    latest_odom_ = std::make_shared<SlamOdometryState>(state);
                }

                // 如果是关键帧，管理关键帧
                if (is_keyframe) {
                    const SlamGeoPoseReference geo_reference =
                        algorithm_->getLastNavigationReference();
                    SlamKeyframe kf = createKeyframe(
                        bundle->timestamp, pose_state.pose, static_points,
                        geo_reference);

                    {
                        std::lock_guard<std::mutex> lock(keyframes_mutex_);
                        keyframes_.push_back(kf);
                    }

                    manageKeyframes();

                    // 触发回调
                    if (keyframe_callback_) {
                        keyframe_callback_(std::make_shared<SlamKeyframe>(kf));
                    }
                    postSlamKeyframe(kf);
                }

                state.registration_score = algorithm_->getLastGicpScore();
                publishOdometry(state);
                postSlamOdometry(state);

                QVector<M_PointXYZI> world_scan;
                world_scan.reserve(static_cast<int>(static_points.size()));
                for (const M_PointXYZI &p : static_points) {
                    const Eigen::Vector3d wp =
                        pose_state.pose * Eigen::Vector3d(p.x, p.y, p.z);
                    M_PointXYZI q;
                    q.x = static_cast<float>(wp.x());
                    q.y = static_cast<float>(wp.y());
                    q.z = static_cast<float>(wp.z());
                    q.intensity = p.intensity;
                    world_scan.append(q);
                }
                if (!world_scan.isEmpty())
                    postSlamScanCloud(std::move(world_scan));

                if (odometry_callback_) {
                    odometry_callback_(std::make_shared<SlamOdometryState>(state));
                }
            }
        }
    }
    
    postLog("[ProcessLoop] 线程结束");
}

void DloSlamNode::manageKeyframes() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);

    if (keyframes_.empty()) {
        return;
    }

    double current_time = keyframes_.back().timestamp;
    bool changed = false;

    // 移除超出数量限制的最旧关键帧
    while (static_cast<int>(keyframes_.size()) > config_.max_keyframes) {
        keyframes_.erase(keyframes_.begin());
        changed = true;
    }

    // 移除超出时间限制的最旧关键帧
    while (!keyframes_.empty() &&
           (current_time - keyframes_.front().timestamp) > config_.max_keyframe_age_sec) {
        keyframes_.erase(keyframes_.begin());
        changed = true;
    }

    if (changed) {
        postLog("Keyframe management: " + QString::number(keyframes_.size()) + " keyframes remaining");
    }
}

bool DloSlamNode::needNewKeyframe(const Eigen::Isometry3d& current_pose) {
    std::lock_guard<std::mutex> lock(last_pose_mutex_);

    // 检查是否是初始状态（第一帧）
    bool last_pose_is_zero = (last_keyframe_pose_.translation().norm() < 1e-6) &&
                            (last_keyframe_pose_.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6;

    if (last_pose_is_zero) {
        // 第一个关键帧
        last_keyframe_pose_ = current_pose;
        return true;
    }

    // 检查平移
    double trans = (current_pose.translation() - last_keyframe_pose_.translation()).norm();
    if (trans > config_.keyframe_translation_thresh) {
        last_keyframe_pose_ = current_pose;
        return true;
    }

    // 检查旋转
    Eigen::Quaterniond q_current(current_pose.linear());
    Eigen::Quaterniond q_last(last_keyframe_pose_.linear());
    if (q_current.dot(q_last) < 0) {
        q_last.coeffs() *= -1;
    }
    Eigen::Quaterniond dq = q_current * q_last.inverse();
    double angle_deg = std::abs(Eigen::AngleAxisd(dq).angle()) * 180.0 / M_PI;
    if (angle_deg > config_.keyframe_rotation_thresh_deg) {
        last_keyframe_pose_ = current_pose;
        return true;
    }

    return false;
}

SlamKeyframe DloSlamNode::createKeyframe(double timestamp,
                                          const Eigen::Isometry3d& pose,
                                          const std::vector<M_PointXYZI>& cloud,
                                          const SlamGeoPoseReference& geo_reference) {
    static std::atomic<uint64_t> next_id{0};

    SlamKeyframe kf;
    kf.keyframe_id = next_id.fetch_add(1);
    kf.timestamp = timestamp;
    kf.pose.matrix() = pose.matrix();
    if (slam_pose::navigationReferenceMatches(
            geo_reference, timestamp, config_.max_gnss_sync_sec)) {
        kf.geo_reference = geo_reference;
    }
    kf.cloud = cloud;

    return kf;
}

SlamOdometryState DloSlamNode::buildOdometryState(double timestamp,
                                                   uint64_t frame_seq,
                                                   const Eigen::Isometry3d& pose,
                                                   bool is_keyframe) {
    SlamOdometryState state;
    state.timestamp = timestamp;
    state.frame_seq = frame_seq;
    state.pose_lidar.pose.matrix() = pose.matrix();
    state.pose_lidar.valid = true;
    state.is_keyframe = is_keyframe;

    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    if (!keyframes_.empty()) {
        state.keyframe_id = keyframes_.back().keyframe_id;
    }

    return state;
}

void DloSlamNode::publishOdometry(const SlamOdometryState& state) {
    auto& comm = CommunicationManager::instance();
    auto topic = comm.getStateTopic<SlamOdometryState>("slam/odometry", 16);
    topic->publish(std::make_shared<SlamOdometryState>(state));
}

}  // namespace usv

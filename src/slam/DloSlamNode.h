#pragma once

#include <QObject>
#include <QString>
#include <QVector>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "common/slam_types.h"
#include "LidarGnssSlam.h"
#include "CommunicationManager_v2.h"
#include "WaterSurfaceFilter.h"

namespace usv {

/**
 * DloSlamNode - DLO SLAM 节点
 * 
 * 设计原则：
 * 1. 继承 QObject，支持信号槽
 * 2. 通过 Topic 总线通信（融合点云 + GNSS/INS）
 * 3. 使用 postLog 辅助函数通过 Qt::QueuedConnection 发出日志信号
 * 4. 完全由外部控制生命周期
 * 
 * 功能：
 * 1. 订阅 Topic 总线中的融合点云数据（/sensor/fused_cloud）
 * 2. 订阅 Topic 总线中的 GNSS/INS（/sensor/imu/raw）
 * 3. 调用 LidarGnssSlam 进行点云配准
 * 4. 管理关键帧数组（最大1000帧，最长3600秒）
 * 5. 发布里程计状态到 Topic 总线
 */
class DloSlamNode : public QObject {
    Q_OBJECT
public:
    explicit DloSlamNode(QObject *parent = nullptr);
    ~DloSlamNode() override;
    
signals:
    void logMessage(const QString &text);
    void slamOdometryReady(usv::SlamOdometryState state);
    void slamKeyframeReady(usv::SlamKeyframe keyframe);
    /** 每帧 SLAM 处理完成后，当前扫描的世界系点云（用于地图实时显示） */
    void slamScanCloudReady(QVector<M_PointXYZI> worldCloud);

public:
    /**
     * 初始化 SLAM 节点
     * @param config 配置参数
     * @return 是否初始化成功
     */
    bool init(const DloSlamConfig& config);

    /**
     * 启动节点，开始监听 Topic
     */
    void start();

    /**
     * 停止节点
     */
    void stop();

    /**
     * 获取最新里程计状态
     * @return 最新状态，如果不可用则返回 nullptr
     */
    std::shared_ptr<const SlamOdometryState> getLatestOdometry() const;

    /**
     * 获取关键帧数量
     * @return 关键帧数量
     */
    size_t getKeyframeCount() const;

    /**
     * 获取所有关键帧（线程安全拷贝）
     * @return 关键帧数组的拷贝
     */
    std::vector<SlamKeyframe> getKeyframes() const;

    /**
     * 手动保存地图（触发接口）
     * @param directoryOrFile 保存目录；若以 .slammap 结尾则按文件路径保存
     * @return 是否保存成功
     */
    bool saveMap(const std::string& directory);

    /**
     * 设置里程计回调（可选）
     * @param callback 回调函数
     */
    void setOdometryCallback(std::function<void(std::shared_ptr<const SlamOdometryState>)> callback);

    /**
     * 设置关键帧回调（可选）
     * @param callback 回调函数
     */
    void setKeyframeCallback(std::function<void(std::shared_ptr<const SlamKeyframe>)> callback);

    /**
     * 检查节点是否正在运行
     * @return 运行状态
     */
    bool isRunning() const;

    /** 喂入 GNSS/INS（总线订阅为主；也可手动注入最新状态） */
    void feedGnssIns(const GnssInsMessage& gnss_ins);

    /** 获取本次 SLAM 会话使用的首个有效 GNSS 地理锚点 */
    SlamGeoAnchor getGeoAnchor() const;

private:
    void drainImuBus();
    /**
     * 内部日志输出辅助函数（使用 QMetaObject::invokeMethod 确保线程安全）
     * @param text 日志文本
     */
    void postLog(const QString &text);
    void postSlamOdometry(const usv::SlamOdometryState &state);
    void postSlamKeyframe(const usv::SlamKeyframe &keyframe);
    void postSlamScanCloud(QVector<M_PointXYZI> worldCloud);

    /**
     * SLAM 处理循环（独立线程）
     */
    void processLoop();

    /**
     * 管理关键帧数组
     * - 移除超出数量限制的最旧关键帧
     * - 移除超出时间限制的最旧关键帧
     */
    void manageKeyframes();

    /**
     * 检查是否需要新增关键帧
     * @param current_pose 当前位姿
     * @return 是否需要新增
     */
    bool needNewKeyframe(const Eigen::Isometry3d& current_pose);

    /**
     * 创建新关键帧
     * @param timestamp 时间戳
     * @param pose 位姿
     * @param cloud 点云
     * @return 新关键帧
     */
    SlamKeyframe createKeyframe(double timestamp,
                                 const Eigen::Isometry3d& pose,
                                 const std::vector<M_PointXYZI>& cloud,
                                 const SlamGeoPoseReference& geo_reference);

    /**
     * 构建里程计状态
     * @param timestamp 时间戳
     * @param frame_seq 帧序号
     * @param pose 位姿
     * @param is_keyframe 是否关键帧
     * @return 里程计状态
     */
    SlamOdometryState buildOdometryState(double timestamp,
                                         uint64_t frame_seq,
                                         const Eigen::Isometry3d& pose,
                                         bool is_keyframe);

    /**
     * 发布里程计状态
     * @param state 里程计状态
     */
    void publishOdometry(const SlamOdometryState& state);

private:
    // 配置
    DloSlamConfig config_;

    // 核心 SLAM 算法
    std::unique_ptr<LidarGnssSlam> algorithm_;
    WaterSurfaceFilter water_surface_filter_;

    // 运行状态
    std::atomic<bool> running_{false};
    std::thread process_thread_;

    // 帧序号
    std::atomic<uint64_t> frame_seq_{0};

    // 关键帧数组（线程安全）
    mutable std::mutex keyframes_mutex_;
    std::vector<SlamKeyframe> keyframes_;

    // 上一帧位姿（用于关键帧判断）
    mutable std::mutex last_pose_mutex_;
    Eigen::Isometry3d last_keyframe_pose_;

    // 里程计状态（最新）
    mutable std::mutex odom_mutex_;
    std::shared_ptr<SlamOdometryState> latest_odom_;

    // 回调函数
    std::function<void(std::shared_ptr<const SlamOdometryState>)> odometry_callback_;
    std::function<void(std::shared_ptr<const SlamKeyframe>)> keyframe_callback_;

    // 数据队列（用于异步处理）
    std::mutex queue_mutex_;
    std::deque<std::shared_ptr<const FusedCloudBundle>> cloud_queue_;
    std::condition_variable queue_cv_;

    // Topic 订阅
    std::shared_ptr<usv::RawTopic<usv::FusedCloudBundle>> fused_cloud_topic_;
    uint64_t fused_subscriber_id_ = 0;
    std::shared_ptr<usv::RawTopic<usv::GnssInsMessage>> imu_topic_;
    uint64_t imu_subscriber_id_ = 0;
    std::shared_ptr<usv::StateTopic<usv::SlamKeyframe>> slam_keyframe_topic_;
    std::shared_ptr<usv::StateTopic<usv::SlamScanCloudMessage>> slam_scan_topic_;

    mutable std::mutex geo_anchor_mutex_;
    SlamGeoAnchor geo_anchor_;
};

}  // namespace usv

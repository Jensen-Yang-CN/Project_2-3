#pragma once

#include "CommunicationManager_v2.h"
#include "NaviProtocol.h"
#include "bus_convert.h"
#include "common_types_extended.h"

#include <QImage>
#include <QObject>
#include <QString>
#include <QVector>

#include "common/pointxyz.h"
#include "common/slam_types.h"
#include "bridge_common_types.h"

/** 将 PointCloud 解析管线数据发布到 RawTopic / StateTopic */
class SensorPublisherNode : public QObject {
    Q_OBJECT
public:
    struct Config {
        QString lidar210 = QStringLiteral("/sensor/lidar/210/raw");
        QString lidar211 = QStringLiteral("/sensor/lidar/211/raw");
        QString cameraLeft = QStringLiteral("/sensor/camera/left/raw");
        QString imu = QStringLiteral("/sensor/imu/raw");
        QString syncedSensors = QStringLiteral("/preprocess/synced_sensors");
        QString fusedCloud = QStringLiteral("/sensor/fused_cloud");
        size_t raw_topic_capacity = 64;
        size_t synced_history_size = 16;
    };

    explicit SensorPublisherNode(QObject *parent = nullptr);

    void Init();

public slots:
    void publishLidar210(const QVector<M_PointXYZI> &cloud, double timestamp,
                         quint64 timelineGeneration = 1);
    void publishLidar211(const QVector<M_PointXYZI> &cloud, double timestamp,
                         quint64 timelineGeneration = 1);
    void publishCameraLeft(const QImage &image, double timestamp,
                           quint64 timelineGeneration = 1);
    /** GNSS/INS 导航解算结果 → /sensor/imu/raw */
    void publishImu(const IMUParsedData &imu);
    /** 一次转换，按开关分别发布到泊位 / SLAM Topic */
    void publishFusedSensors(const QVector<M_PointXYZI> &cloud, double timestamp,
                             bool toBerth, bool toSlam);
    void publishFusedLidar(const QVector<M_PointXYZI> &cloud, double timestamp);
    void publishFusedCloud(const QVector<M_PointXYZI> &cloud, double timestamp);

private:
    void publishFusedLidarFromFrame(const std::shared_ptr<usv::LidarFrame> &frame);
    void publishFusedCloudFromFrame(const std::shared_ptr<usv::LidarFrame> &frame);

    Config cfg_;
    usv::CommunicationManager *comm_ = nullptr;
    std::shared_ptr<usv::RawTopic<usv::LidarFrame>> lidar210_topic_;
    std::shared_ptr<usv::RawTopic<usv::LidarFrame>> lidar211_topic_;
    std::shared_ptr<usv::RawTopic<usv::ImageFrame>> camera_topic_;
    std::shared_ptr<usv::RawTopic<usv::GnssInsMessage>> imu_topic_;
    std::shared_ptr<usv::StateTopic<usv::SyncedSensorPackage>> synced_sensors_topic_;
    std::shared_ptr<usv::RawTopic<usv::FusedCloudBundle>> fused_cloud_topic_;
};

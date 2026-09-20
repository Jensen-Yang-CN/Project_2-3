#include "SensorPublisherNode.h"

#include "GnssInsGeoUtils.h"
#include "SensorMetrics.h"

SensorPublisherNode::SensorPublisherNode(QObject *parent)
    : QObject(parent)
    , comm_(&usv::CommunicationManager::instance())
{
}

void SensorPublisherNode::Init()
{
    lidar210_topic_ = comm_->getRawTopic<usv::LidarFrame>(
        cfg_.lidar210.toStdString(), cfg_.raw_topic_capacity);
    lidar211_topic_ = comm_->getRawTopic<usv::LidarFrame>(
        cfg_.lidar211.toStdString(), cfg_.raw_topic_capacity);
    camera_topic_ = comm_->getRawTopic<usv::ImageFrame>(
        cfg_.cameraLeft.toStdString(), cfg_.raw_topic_capacity);
    imu_topic_ = comm_->getRawTopic<usv::GnssInsMessage>(
        cfg_.imu.toStdString(), cfg_.raw_topic_capacity);
    synced_sensors_topic_ = comm_->getStateTopic<usv::SyncedSensorPackage>(
        cfg_.syncedSensors.toStdString(), cfg_.synced_history_size);
    fused_cloud_topic_ = comm_->getRawTopic<usv::FusedCloudBundle>(
        cfg_.fusedCloud.toStdString(), cfg_.raw_topic_capacity);
}

void SensorPublisherNode::publishLidar210(const QVector<M_PointXYZI> &cloud,
                                          double timestamp,
                                          quint64 timelineGeneration)
{
    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::Lidar210, timestamp);
    if (!lidar210_topic_)
        return;
    lidar210_topic_->publish(bus_convert::lidarFromQtCloud(
        cloud, timestamp, 80000, timelineGeneration));
}

void SensorPublisherNode::publishLidar211(const QVector<M_PointXYZI> &cloud,
                                          double timestamp,
                                          quint64 timelineGeneration)
{
    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::Lidar211, timestamp);
    if (!lidar211_topic_)
        return;
    lidar211_topic_->publish(bus_convert::lidarFromQtCloud(
        cloud, timestamp, 80000, timelineGeneration));
}

void SensorPublisherNode::publishCameraLeft(const QImage &image,
                                            double timestamp,
                                            quint64 timelineGeneration)
{
    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::CameraLeft, timestamp);
    if (!camera_topic_ || image.isNull())
        return;
    camera_topic_->publish(bus_convert::imageFromQt(
        image, timestamp, timelineGeneration));
}

void SensorPublisherNode::publishImu(const IMUParsedData &imu)
{
    if (imu.longitude == 0.0 && imu.latitude == 0.0)
        return;
    if (!imu_topic_)
        return;

    auto msg = std::make_shared<usv::GnssInsMessage>(usv::gnss_geo::gnssFromImu(imu));
    imu_topic_->publish(msg);
    comm_->updateStateRepository<usv::GnssInsMessage>(msg);
}

void SensorPublisherNode::publishFusedSensors(const QVector<M_PointXYZI> &cloud,
                                              double timestamp,
                                              bool toBerth,
                                              bool toSlam)
{
    if (cloud.isEmpty() || (!toBerth && !toSlam))
        return;

    auto frame = bus_convert::lidarFromQtCloud(cloud, timestamp, 0);
    if (!frame || frame->points.empty())
        return;

    if (toBerth)
        publishFusedLidarFromFrame(frame);
    if (toSlam)
        publishFusedCloudFromFrame(frame);
}

void SensorPublisherNode::publishFusedLidarFromFrame(
    const std::shared_ptr<usv::LidarFrame> &frame)
{
    if (!synced_sensors_topic_ || !frame)
        return;

    auto pkg = std::make_shared<usv::SyncedSensorPackage>();
    pkg->lidar = frame;
    synced_sensors_topic_->publish(pkg);
    comm_->updateStateRepository<usv::SyncedSensorPackage>(pkg);
}

void SensorPublisherNode::publishFusedCloudFromFrame(
    const std::shared_ptr<usv::LidarFrame> &frame)
{
    if (!fused_cloud_topic_ || !frame)
        return;

    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::FusedCloud, frame->timestamp);

    auto bundle = std::make_shared<usv::FusedCloudBundle>();
    bundle->timestamp = frame->timestamp;
    bundle->frame_seq = 0;
    bundle->points.reserve(frame->points.size());
    for (const usv::LidarPoint &lp : frame->points) {
        M_PointXYZI p;
        p.x = lp.x;
        p.y = lp.y;
        p.z = lp.z;
        p.intensity = static_cast<unsigned char>(lp.intensity);
        bundle->points.push_back(p);
    }
    fused_cloud_topic_->publish(bundle);
}

void SensorPublisherNode::publishFusedLidar(const QVector<M_PointXYZI> &cloud, double timestamp)
{
    publishFusedSensors(cloud, timestamp, true, false);
}

void SensorPublisherNode::publishFusedCloud(const QVector<M_PointXYZI> &cloud, double timestamp)
{
    publishFusedSensors(cloud, timestamp, false, true);
}

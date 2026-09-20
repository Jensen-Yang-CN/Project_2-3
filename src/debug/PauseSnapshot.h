#pragma once

#include "pointxyz.h"

#include <QImage>
#include <QStringList>
#include <QTimeZone>
#include <QVector>

namespace pause_snapshot {

struct CloudFrame
{
    bool valid = false;
    double timestamp = 0.0;
    QVector<M_PointXYZI> data;
};

struct ImageFrame
{
    bool valid = false;
    double timestamp = 0.0;
    QImage data;
};

struct Snapshot
{
    double target_timestamp = 0.0;
    CloudFrame lidar210;
    CloudFrame lidar211;
    ImageFrame camera_left;
    ImageFrame camera_right;
    QStringList problems;

    bool complete() const { return problems.isEmpty(); }
};

class Collector
{
public:
    explicit Collector(double maxDifferenceSec = 0.5);

    void reset();
    void updateLidar210(double timestamp, const QVector<M_PointXYZI> &data);
    void updateLidar211(double timestamp, const QVector<M_PointXYZI> &data);
    void updateCameraLeft(double timestamp, const QImage &data);
    void updateCameraRight(double timestamp, const QImage &data);

    bool begin(double targetTimestamp);
    Snapshot finish();

private:
    void consider(const CloudFrame &frame, CloudFrame &candidate);
    void consider(const ImageFrame &frame, ImageFrame &candidate);

    double max_difference_sec_ = 0.5;
    double target_timestamp_ = 0.0;
    bool collecting_ = false;

    CloudFrame latest_lidar210_;
    CloudFrame latest_lidar211_;
    ImageFrame latest_camera_left_;
    ImageFrame latest_camera_right_;

    CloudFrame candidate_lidar210_;
    CloudFrame candidate_lidar211_;
    ImageFrame candidate_camera_left_;
    ImageFrame candidate_camera_right_;
};

struct SaveResult
{
    bool complete = false;
    QString base_name;
    QStringList written_files;
    QStringList errors;
};

SaveResult saveSnapshot(
    const Snapshot &snapshot,
    const QString &root,
    const QTimeZone &zone = QTimeZone::systemTimeZone());

}  // namespace pause_snapshot

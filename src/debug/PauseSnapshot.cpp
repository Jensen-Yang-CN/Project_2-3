#include "PauseSnapshot.h"

#include "PcapPlaybackTime.h"

#include <QDataStream>
#include <QDir>
#include <QSaveFile>

#include <algorithm>
#include <cmath>

namespace pause_snapshot {
namespace {

constexpr double kTimestampEpsilon = 1e-9;

bool validTimestamp(double timestamp)
{
    return std::isfinite(timestamp) && timestamp > 0.0;
}

bool writePcd(const QString &path,
              const QVector<M_PointXYZI> &points,
              QString &error)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        error = QStringLiteral("无法创建点云文件：%1").arg(path);
        return false;
    }

    QByteArray header;
    header += "# .PCD v0.7 - Point Cloud Data file format\n";
    header += "VERSION 0.7\n";
    header += "FIELDS x y z intensity\n";
    header += "SIZE 4 4 4 1\n";
    header += "TYPE F F F U\n";
    header += "COUNT 1 1 1 1\n";
    header += "WIDTH " + QByteArray::number(points.size()) + "\n";
    header += "HEIGHT 1\n";
    header += "VIEWPOINT 0 0 0 1 0 0 0\n";
    header += "POINTS " + QByteArray::number(points.size()) + "\n";
    header += "DATA binary\n";
    if (file.write(header) != header.size()) {
        file.cancelWriting();
        error = QStringLiteral("写入 PCD 文件头失败：%1").arg(path);
        return false;
    }

    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (const M_PointXYZI &point : points)
        stream << point.x << point.y << point.z << quint8(point.intensity);

    if (stream.status() != QDataStream::Ok || !file.commit()) {
        error = QStringLiteral("写入 PCD 点数据失败：%1").arg(path);
        return false;
    }
    return true;
}

bool writePng(const QString &path, const QImage &image, QString &error)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        error = QStringLiteral("无法创建图像文件：%1").arg(path);
        return false;
    }
    if (!image.save(&file, "PNG")) {
        file.cancelWriting();
        error = QStringLiteral("编码 PNG 失败：%1").arg(path);
        return false;
    }
    if (!file.commit()) {
        error = QStringLiteral("提交 PNG 文件失败：%1").arg(path);
        return false;
    }
    return true;
}

template <typename Frame>
bool frameIsUsable(const Frame &frame)
{
    return frame.valid && validTimestamp(frame.timestamp);
}

}  // namespace

Collector::Collector(double maxDifferenceSec)
    : max_difference_sec_(std::max(0.0, maxDifferenceSec))
{
}

void Collector::reset()
{
    target_timestamp_ = 0.0;
    collecting_ = false;
    latest_lidar210_ = {};
    latest_lidar211_ = {};
    latest_camera_left_ = {};
    latest_camera_right_ = {};
    candidate_lidar210_ = {};
    candidate_lidar211_ = {};
    candidate_camera_left_ = {};
    candidate_camera_right_ = {};
}

void Collector::updateLidar210(double timestamp,
                               const QVector<M_PointXYZI> &data)
{
    if (data.isEmpty() || !validTimestamp(timestamp))
        return;
    latest_lidar210_ = {!data.isEmpty() && validTimestamp(timestamp),
                        timestamp, data};
    if (collecting_)
        consider(latest_lidar210_, candidate_lidar210_);
}

void Collector::updateLidar211(double timestamp,
                               const QVector<M_PointXYZI> &data)
{
    if (data.isEmpty() || !validTimestamp(timestamp))
        return;
    latest_lidar211_ = {!data.isEmpty() && validTimestamp(timestamp),
                        timestamp, data};
    if (collecting_)
        consider(latest_lidar211_, candidate_lidar211_);
}

void Collector::updateCameraLeft(double timestamp, const QImage &data)
{
    if (data.isNull() || !validTimestamp(timestamp))
        return;
    latest_camera_left_ = {!data.isNull() && validTimestamp(timestamp),
                           timestamp, data};
    if (collecting_)
        consider(latest_camera_left_, candidate_camera_left_);
}

void Collector::updateCameraRight(double timestamp, const QImage &data)
{
    if (data.isNull() || !validTimestamp(timestamp))
        return;
    latest_camera_right_ = {!data.isNull() && validTimestamp(timestamp),
                            timestamp, data};
    if (collecting_)
        consider(latest_camera_right_, candidate_camera_right_);
}

bool Collector::begin(double targetTimestamp)
{
    if (!validTimestamp(targetTimestamp))
        return false;

    target_timestamp_ = targetTimestamp;
    collecting_ = true;
    candidate_lidar210_ = {};
    candidate_lidar211_ = {};
    candidate_camera_left_ = {};
    candidate_camera_right_ = {};

    consider(latest_lidar210_, candidate_lidar210_);
    consider(latest_lidar211_, candidate_lidar211_);
    consider(latest_camera_left_, candidate_camera_left_);
    consider(latest_camera_right_, candidate_camera_right_);
    return true;
}

void Collector::consider(const CloudFrame &frame, CloudFrame &candidate)
{
    if (!frameIsUsable(frame)
        || frame.timestamp > target_timestamp_ + kTimestampEpsilon)
        return;
    if (!candidate.valid || frame.timestamp >= candidate.timestamp)
        candidate = frame;
}

void Collector::consider(const ImageFrame &frame, ImageFrame &candidate)
{
    if (!frameIsUsable(frame)
        || frame.timestamp > target_timestamp_ + kTimestampEpsilon)
        return;
    if (!candidate.valid || frame.timestamp >= candidate.timestamp)
        candidate = frame;
}

Snapshot Collector::finish()
{
    Snapshot snapshot;
    snapshot.target_timestamp = target_timestamp_;
    snapshot.lidar210 = candidate_lidar210_;
    snapshot.lidar211 = candidate_lidar211_;
    snapshot.camera_left = candidate_camera_left_;
    snapshot.camera_right = candidate_camera_right_;
    collecting_ = false;

    auto validate = [&](const auto &frame, const QString &name) {
        if (!frameIsUsable(frame)
            || target_timestamp_ - frame.timestamp > max_difference_sec_
                   + kTimestampEpsilon) {
            snapshot.problems.append(
                QStringLiteral("%1 缺失或与暂停时间差超过 %2 秒")
                    .arg(name)
                    .arg(max_difference_sec_, 0, 'f', 1));
        }
    };
    validate(snapshot.lidar210, QStringLiteral("210 点云"));
    validate(snapshot.lidar211, QStringLiteral("211 点云"));
    validate(snapshot.camera_left, QStringLiteral("左相机图像"));
    validate(snapshot.camera_right, QStringLiteral("右相机图像"));
    return snapshot;
}

SaveResult saveSnapshot(const Snapshot &snapshot,
                        const QString &root,
                        const QTimeZone &zone)
{
    SaveResult result;
    result.base_name = pcap_playback_time::snapshotBaseName(
        snapshot.target_timestamp, zone);
    if (result.base_name.isEmpty()) {
        result.errors.append(QStringLiteral("暂停时间戳无效，无法生成文件名"));
        return result;
    }

    const QDir rootDir(root);
    const QStringList subdirs = {
        QStringLiteral("210"), QStringLiteral("211"),
        QStringLiteral("camera_left"), QStringLiteral("camera_right")};
    for (const QString &subdir : subdirs) {
        if (!QDir().mkpath(rootDir.filePath(subdir))) {
            result.errors.append(
                QStringLiteral("无法创建快照目录：%1")
                    .arg(rootDir.filePath(subdir)));
        }
    }
    if (!result.errors.isEmpty())
        return result;

    auto saveCloud = [&](const CloudFrame &frame, const QString &subdir) {
        if (!frame.valid || frame.data.isEmpty())
            return;
        const QString path = rootDir.filePath(
            subdir + QLatin1Char('/') + result.base_name
            + QStringLiteral(".pcd"));
        QString error;
        if (writePcd(path, frame.data, error))
            result.written_files.append(path);
        else
            result.errors.append(error);
    };
    auto saveImage = [&](const ImageFrame &frame, const QString &subdir) {
        if (!frame.valid || frame.data.isNull())
            return;
        const QString path = rootDir.filePath(
            subdir + QLatin1Char('/') + result.base_name
            + QStringLiteral(".png"));
        QString error;
        if (writePng(path, frame.data, error))
            result.written_files.append(path);
        else
            result.errors.append(error);
    };

    saveCloud(snapshot.lidar210, QStringLiteral("210"));
    saveCloud(snapshot.lidar211, QStringLiteral("211"));
    saveImage(snapshot.camera_left, QStringLiteral("camera_left"));
    saveImage(snapshot.camera_right, QStringLiteral("camera_right"));

    result.errors.append(snapshot.problems);
    result.complete = snapshot.complete()
        && result.errors.isEmpty()
        && result.written_files.size() == 4;
    return result;
}

}  // namespace pause_snapshot

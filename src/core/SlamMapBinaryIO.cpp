#include "SlamMapBinaryIO.h"
#include "AppConfig.h"

#include <QDataStream>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSet>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace slam_map_io {

namespace {

const QByteArray kMagic("USVSLM2", 7);
constexpr quint8 kCoordinateFrameEnu = 1;
const QString kLastMapRecordName = QStringLiteral("last_slam_map.txt");

bool fail(QString *error, const QString &message)
{
    if (error)
        *error = message;
    return false;
}

bool finite(double value)
{
    return std::isfinite(value);
}

bool validAnchor(const usv::SlamGeoAnchor &anchor)
{
    if (!anchor.valid)
        return true;
    return finite(anchor.timestamp)
        && finite(anchor.latitude_deg)
        && finite(anchor.longitude_deg)
        && finite(anchor.altitude_m)
        && anchor.latitude_deg >= -90.0
        && anchor.latitude_deg <= 90.0
        && anchor.longitude_deg >= -180.0
        && anchor.longitude_deg <= 180.0
        && (anchor.latitude_deg != 0.0 || anchor.longitude_deg != 0.0);
}

bool validPoint(const M_PointXYZI &point)
{
    return finite(point.x) && finite(point.y) && finite(point.z);
}

bool validKeyframe(const usv::SlamKeyframe &keyframe)
{
    if (!finite(keyframe.timestamp)
        || keyframe.cloud.empty()
        || keyframe.cloud.size() > kMaxPointsPerKeyframe) {
        return false;
    }

    const Eigen::Matrix4d matrix = keyframe.pose.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (!finite(matrix(row, col)))
                return false;
        }
    }

    const Eigen::Quaterniond quaternion(keyframe.pose.linear());
    if (!finite(quaternion.norm()) || quaternion.norm() < 1e-12)
        return false;

    if (keyframe.geo_reference.valid) {
        if (!finite(keyframe.geo_reference.timestamp)
            || !finite(keyframe.geo_reference.sync_error_sec)
            || keyframe.geo_reference.sync_error_sec < 0.0) {
            return false;
        }
        const Eigen::Matrix4d referenceMatrix =
            keyframe.geo_reference.pose_lidar_enu.matrix();
        if (!referenceMatrix.allFinite())
            return false;
        const Eigen::Quaterniond referenceQuaternion(
            keyframe.geo_reference.pose_lidar_enu.linear());
        if (!finite(referenceQuaternion.norm())
            || referenceQuaternion.norm() < 1e-12) {
            return false;
        }
    }

    return std::all_of(keyframe.cloud.begin(), keyframe.cloud.end(),
                       validPoint);
}

bool validMapBerth(const usv::SlamMapBerth &berth)
{
    return finite(berth.timestamp)
        && finite(berth.x) && finite(berth.y) && finite(berth.z)
        && finite(berth.width) && finite(berth.length)
        && finite(berth.angle_deg) && finite(berth.confidence)
        && berth.width > 1e-3 && berth.length > 1e-3
        && berth.confidence >= 0.0 && berth.confidence <= 1.0
        && berth.kind <= 2
        && (berth.opening_edge >= -1 && berth.opening_edge < 4);
}

void configureStream(QDataStream &stream)
{
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setVersion(QDataStream::Qt_5_12);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
}

QString normalizedExistingPath(const QString &path)
{
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    if (!canonical.isEmpty())
        return QDir::cleanPath(canonical);
    return QDir::cleanPath(info.absoluteFilePath());
}

QString lastMapRecordPath(const QString &mapsDirectory)
{
    return QDir(mapsDirectory).absoluteFilePath(kLastMapRecordName);
}

bool loadVersion2(QFile &file, MapArchive &archive, QString *error)
{
    QDataStream stream(&file);
    configureStream(stream);

    QByteArray magic(kMagic.size(), Qt::Uninitialized);
    if (stream.readRawData(magic.data(), magic.size()) != magic.size()
        || magic != kMagic) {
        return fail(error, QStringLiteral("地图文件魔数无效"));
    }

    quint16 major = 0;
    quint16 minor = 0;
    quint8 coordinateFrame = 0;
    quint8 anchorValid = 0;
    quint32 keyframeCount = 0;
    usv::SlamGeoAnchor anchor;
    stream >> major >> minor >> coordinateFrame >> anchorValid
           >> anchor.timestamp
           >> anchor.latitude_deg
           >> anchor.longitude_deg
           >> anchor.altitude_m
           >> keyframeCount;

    if (stream.status() != QDataStream::Ok)
        return fail(error, QStringLiteral("地图文件头不完整"));
    if (major != kFormatMajor)
        return fail(error, QStringLiteral("不支持的地图主版本: %1").arg(major));
    if (minor > kFormatMinor)
        return fail(error, QStringLiteral("不支持的地图次版本: %1").arg(minor));
    if (coordinateFrame != kCoordinateFrameEnu)
        return fail(error, QStringLiteral("不支持的地图坐标系: %1").arg(coordinateFrame));
    if (anchorValid > 1)
        return fail(error, QStringLiteral("地理锚点标记无效"));
    anchor.valid = anchorValid != 0;
    if (!validAnchor(anchor))
        return fail(error, QStringLiteral("地图地理锚点无效"));
    if (keyframeCount == 0 || keyframeCount > kMaxKeyframeCount)
        return fail(error, QStringLiteral("关键帧数量异常: %1").arg(keyframeCount));

    MapArchive candidate;
    candidate.source_format_major = major;
    candidate.source_format_minor = minor;
    candidate.anchor = anchor;
    candidate.keyframes.reserve(keyframeCount);
    quint64 totalPoints = 0;

    for (quint32 index = 0; index < keyframeCount; ++index) {
        usv::SlamKeyframe keyframe;
        quint64 keyframeId = 0;
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 0.0;
        quint32 pointCount = 0;
        stream >> keyframeId >> keyframe.timestamp
               >> tx >> ty >> tz >> qx >> qy >> qz >> qw
               >> pointCount;
        if (stream.status() != QDataStream::Ok)
            return fail(error, QStringLiteral("关键帧 %1 元数据不完整").arg(index));
        if (pointCount == 0 || pointCount > kMaxPointsPerKeyframe)
            return fail(error, QStringLiteral("关键帧 %1 点数异常: %2")
                                   .arg(index).arg(pointCount));
        totalPoints += pointCount;
        if (totalPoints > kMaxTotalKeyframePoints)
            return fail(error, QStringLiteral("地图总点数超出上限"));
        if (!finite(keyframe.timestamp)
            || !finite(tx) || !finite(ty) || !finite(tz)
            || !finite(qx) || !finite(qy) || !finite(qz) || !finite(qw)) {
            return fail(error, QStringLiteral("关键帧 %1 位姿包含非法数值").arg(index));
        }

        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        if (!finite(quaternion.norm()) || quaternion.norm() < 1e-12)
            return fail(error, QStringLiteral("关键帧 %1 四元数无效").arg(index));
        quaternion.normalize();
        keyframe.keyframe_id = keyframeId;
        keyframe.pose = Eigen::Isometry3d::Identity();
        keyframe.pose.translation() = Eigen::Vector3d(tx, ty, tz);
        keyframe.pose.linear() = quaternion.toRotationMatrix();

        if (minor >= 1) {
            quint8 referenceValid = 0;
            double referenceTimestamp = 0.0;
            double syncErrorSec = 0.0;
            double referenceTx = 0.0;
            double referenceTy = 0.0;
            double referenceTz = 0.0;
            double referenceQx = 0.0;
            double referenceQy = 0.0;
            double referenceQz = 0.0;
            double referenceQw = 1.0;
            stream >> referenceValid
                   >> referenceTimestamp >> syncErrorSec
                   >> referenceTx >> referenceTy >> referenceTz
                   >> referenceQx >> referenceQy >> referenceQz
                   >> referenceQw;
            if (stream.status() != QDataStream::Ok)
                return fail(error, QStringLiteral(
                    "关键帧 %1 地理参考不完整").arg(index));
            if (referenceValid > 1)
                return fail(error, QStringLiteral(
                    "关键帧 %1 地理参考标记无效").arg(index));
            if (!finite(referenceTimestamp) || !finite(syncErrorSec)
                || syncErrorSec < 0.0
                || !finite(referenceTx) || !finite(referenceTy)
                || !finite(referenceTz) || !finite(referenceQx)
                || !finite(referenceQy) || !finite(referenceQz)
                || !finite(referenceQw)) {
                return fail(error, QStringLiteral(
                    "关键帧 %1 地理参考包含非法数值").arg(index));
            }
            Eigen::Quaterniond referenceQuaternion(
                referenceQw, referenceQx, referenceQy, referenceQz);
            if (referenceQuaternion.norm() < 1e-12)
                return fail(error, QStringLiteral(
                    "关键帧 %1 地理参考四元数无效").arg(index));
            referenceQuaternion.normalize();
            keyframe.geo_reference.valid = referenceValid != 0;
            keyframe.geo_reference.timestamp = referenceTimestamp;
            keyframe.geo_reference.sync_error_sec = syncErrorSec;
            keyframe.geo_reference.pose_lidar_enu =
                Eigen::Isometry3d::Identity();
            keyframe.geo_reference.pose_lidar_enu.translation() =
                Eigen::Vector3d(referenceTx, referenceTy, referenceTz);
            keyframe.geo_reference.pose_lidar_enu.linear() =
                referenceQuaternion.toRotationMatrix();
        }
        keyframe.cloud.reserve(pointCount);

        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        for (quint32 pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            M_PointXYZI point{};
            quint8 intensity = 0;
            stream >> point.x >> point.y >> point.z >> intensity;
            point.intensity = intensity;
            if (stream.status() != QDataStream::Ok)
                return fail(error, QStringLiteral("关键帧 %1 点云数据不完整").arg(index));
            if (!validPoint(point))
                return fail(error, QStringLiteral("关键帧 %1 点云包含非法坐标").arg(index));
            keyframe.cloud.push_back(point);
        }
        stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
        candidate.keyframes.push_back(std::move(keyframe));
    }

    if (minor >= 2) {
        quint32 berthCount = 0;
        stream >> berthCount;
        if (stream.status() != QDataStream::Ok)
            return fail(error, QStringLiteral("泊位图层数量缺失"));
        if (berthCount > kMaxMapBerthCount)
            return fail(error, QStringLiteral("泊位图层数量超出上限: %1")
                               .arg(berthCount));

        candidate.berths.reserve(berthCount);
        for (quint32 index = 0; index < berthCount; ++index) {
            usv::SlamMapBerth berth;
            quint64 berthId = 0;
            quint8 kind = 0;
            qint32 openingEdge = -1;
            quint32 observationCount = 0;
            stream >> berthId >> berth.timestamp
                   >> berth.x >> berth.y >> berth.z
                   >> berth.width >> berth.length >> berth.angle_deg
                   >> kind >> openingEdge >> observationCount
                   >> berth.confidence;
            if (stream.status() != QDataStream::Ok)
                return fail(error, QStringLiteral(
                    "泊位图层记录 %1 不完整").arg(index));
            berth.id = berthId;
            berth.kind = kind;
            berth.opening_edge = static_cast<int8_t>(openingEdge);
            berth.observation_count = observationCount;
            if (!validMapBerth(berth))
                return fail(error, QStringLiteral(
                    "泊位图层记录 %1 包含非法几何").arg(index));
            candidate.berths.push_back(std::move(berth));
        }
    }

    if (stream.status() != QDataStream::Ok || !file.atEnd())
        return fail(error, QStringLiteral("地图文件包含不完整或多余数据"));

    archive = std::move(candidate);
    return true;
}

}  // namespace

QString defaultMapsDirectory()
{
    return AppConfig::instance().mapsDirectory();
}

bool save(const QString &path, const QVector<M_PointXYZI> &cloud,
          QString *error)
{
    if (cloud.isEmpty())
        return fail(error, QStringLiteral("SLAM 地图为空，无法保存"));
    if (cloud.size() > static_cast<int>(kLegacyMaxPointCount))
        return fail(error, QStringLiteral("点云数量超出上限"));

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        return fail(error, QStringLiteral("无法写入文件: %1 (%2)")
                               .arg(path, file.errorString()));
    }

    const quint32 count = static_cast<quint32>(cloud.size());
    if (file.write(reinterpret_cast<const char *>(&count), sizeof(count))
        != static_cast<qint64>(sizeof(count))) {
        return fail(error, QStringLiteral("写入点数量失败"));
    }

    const qint64 payload = static_cast<qint64>(cloud.size())
                         * static_cast<qint64>(sizeof(M_PointXYZI));
    if (file.write(reinterpret_cast<const char *>(cloud.constData()), payload)
        != payload) {
        return fail(error, QStringLiteral("写入点云数据失败"));
    }
    if (!file.commit())
        return fail(error, QStringLiteral("提交地图文件失败: %1")
                               .arg(file.errorString()));
    return true;
}

bool load(const QString &path, QVector<M_PointXYZI> &cloud, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return fail(error, QStringLiteral("无法打开文件: %1 (%2)")
                               .arg(path, file.errorString()));
    }

    const qint64 fileSize = file.size();
    if (fileSize < static_cast<qint64>(sizeof(quint32)))
        return fail(error, QStringLiteral("文件过短，不是有效的 SLAM 地图"));

    quint32 count = 0;
    if (file.read(reinterpret_cast<char *>(&count), sizeof(count))
        != static_cast<qint64>(sizeof(count))) {
        return fail(error, QStringLiteral("读取点数量失败"));
    }
    if (count == 0 || count > kLegacyMaxPointCount)
        return fail(error, QStringLiteral("点云数量异常 (%1)").arg(count));

    const qint64 expected = static_cast<qint64>(sizeof(quint32))
                          + static_cast<qint64>(count)
                              * static_cast<qint64>(sizeof(M_PointXYZI));
    if (fileSize != expected) {
        return fail(error, QStringLiteral(
            "文件大小与点数不匹配 (期望 %1 字节，实际 %2 字节)")
                           .arg(expected).arg(fileSize));
    }

    QVector<M_PointXYZI> candidate;
    candidate.resize(static_cast<int>(count));
    const qint64 payload = static_cast<qint64>(count)
                         * static_cast<qint64>(sizeof(M_PointXYZI));
    if (file.read(reinterpret_cast<char *>(candidate.data()), payload)
        != payload) {
        return fail(error, QStringLiteral("读取点云数据失败"));
    }

    cloud = std::move(candidate);
    return true;
}

bool saveArchive(const QString &path, const MapArchive &archive, QString *error)
{
    if (archive.keyframes.empty())
        return fail(error, QStringLiteral("没有关键帧可保存"));
    if (archive.keyframes.size() > kMaxKeyframeCount)
        return fail(error, QStringLiteral("关键帧数量超出上限"));
    if (archive.berths.size() > kMaxMapBerthCount)
        return fail(error, QStringLiteral("泊位图层数量超出上限"));
    if (!validAnchor(archive.anchor))
        return fail(error, QStringLiteral("地理锚点无效"));

    quint64 totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        if (!validKeyframe(keyframe))
            return fail(error, QStringLiteral("关键帧包含空点云、非法数值或点数超限"));
        totalPoints += keyframe.cloud.size();
        if (totalPoints > kMaxTotalKeyframePoints)
            return fail(error, QStringLiteral("地图总点数超出上限"));
    }
    for (const usv::SlamMapBerth &berth : archive.berths) {
        if (!validMapBerth(berth))
            return fail(error, QStringLiteral("泊位图层包含非法几何"));
    }

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        return fail(error, QStringLiteral("无法写入文件: %1 (%2)")
                               .arg(path, file.errorString()));
    }

    QDataStream stream(&file);
    configureStream(stream);
    stream.writeRawData(kMagic.constData(), kMagic.size());
    stream << static_cast<quint16>(kFormatMajor)
           << static_cast<quint16>(kFormatMinor)
           << static_cast<quint8>(kCoordinateFrameEnu)
           << static_cast<quint8>(archive.anchor.valid ? 1 : 0)
           << archive.anchor.timestamp
           << archive.anchor.latitude_deg
           << archive.anchor.longitude_deg
           << archive.anchor.altitude_m
           << static_cast<quint32>(archive.keyframes.size());

    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        const Eigen::Vector3d translation = keyframe.pose.translation();
        Eigen::Quaterniond quaternion(keyframe.pose.linear());
        quaternion.normalize();
        stream << static_cast<quint64>(keyframe.keyframe_id)
               << keyframe.timestamp
               << translation.x() << translation.y() << translation.z()
               << quaternion.x() << quaternion.y() << quaternion.z()
               << quaternion.w()
               << static_cast<quint32>(keyframe.cloud.size());

        const usv::SlamGeoPoseReference &reference =
            keyframe.geo_reference;
        const Eigen::Vector3d referenceTranslation =
            reference.pose_lidar_enu.translation();
        Eigen::Quaterniond referenceQuaternion(
            reference.pose_lidar_enu.linear());
        referenceQuaternion.normalize();
        stream << static_cast<quint8>(reference.valid ? 1 : 0)
               << reference.timestamp << reference.sync_error_sec
               << referenceTranslation.x() << referenceTranslation.y()
               << referenceTranslation.z()
               << referenceQuaternion.x() << referenceQuaternion.y()
               << referenceQuaternion.z() << referenceQuaternion.w();

        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        for (const M_PointXYZI &point : keyframe.cloud) {
            stream << point.x << point.y << point.z
                   << static_cast<quint8>(point.intensity);
        }
        stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    }

    stream << static_cast<quint32>(archive.berths.size());
    for (const usv::SlamMapBerth &berth : archive.berths) {
        stream << static_cast<quint64>(berth.id)
               << berth.timestamp
               << berth.x << berth.y << berth.z
               << berth.width << berth.length << berth.angle_deg
               << static_cast<quint8>(berth.kind)
               << static_cast<qint32>(berth.opening_edge)
               << static_cast<quint32>(berth.observation_count)
               << berth.confidence;
    }

    if (stream.status() != QDataStream::Ok)
        return fail(error, QStringLiteral("写入地图数据失败"));
    if (!file.commit())
        return fail(error, QStringLiteral("提交地图文件失败: %1").arg(file.errorString()));
    return true;
}

bool loadArchive(const QString &path, MapArchive &archive, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return fail(error, QStringLiteral("无法打开文件: %1 (%2)")
                               .arg(path, file.errorString()));
    }

    if (file.peek(kMagic.size()) != kMagic)
        return fail(error, QStringLiteral("地图文件魔数无效，仅支持新版 .slammap"));
    return loadVersion2(file, archive, error);
}

QVector<M_PointXYZI> rebuildWorldCloud(
    const std::vector<usv::SlamKeyframe> &keyframes,
    int max_points,
    uint8_t display_intensity)
{
    QVector<M_PointXYZI> world;
    if (max_points <= 0 || keyframes.empty())
        return world;

    quint64 totalPoints = 0;
    for (const usv::SlamKeyframe &keyframe : keyframes)
        totalPoints += keyframe.cloud.size();
    if (totalPoints == 0)
        return world;

    const quint64 step = totalPoints > static_cast<quint64>(max_points)
        ? (totalPoints + static_cast<quint64>(max_points) - 1)
              / static_cast<quint64>(max_points)
        : 1;
    world.reserve(static_cast<int>(
        std::min<quint64>(totalPoints, static_cast<quint64>(max_points))));

    quint64 sourceIndex = 0;
    for (const usv::SlamKeyframe &keyframe : keyframes) {
        for (const M_PointXYZI &point : keyframe.cloud) {
            if (sourceIndex++ % step != 0)
                continue;
            const Eigen::Vector3d transformed =
                keyframe.pose * Eigen::Vector3d(point.x, point.y, point.z);
            M_PointXYZI output{};
            output.x = static_cast<float>(transformed.x());
            output.y = static_cast<float>(transformed.y());
            output.z = static_cast<float>(transformed.z());
            output.intensity = display_intensity;
            world.append(output);
            if (world.size() >= max_points)
                return world;
        }
    }
    return world;
}

bool rememberLastMap(const QString &mapPath, QString *error)
{
    return rememberLastMap(mapPath, defaultMapsDirectory(), error);
}

bool rememberLastMap(const QString &mapPath, const QString &mapsDirectory,
                     QString *error)
{
    const QFileInfo mapInfo(mapPath);
    if (!mapInfo.exists() || !mapInfo.isFile()
        || mapInfo.suffix().compare(QStringLiteral("slammap"),
                                    Qt::CaseInsensitive) != 0) {
        return fail(error, QStringLiteral("最近地图必须是已存在的 .slammap 文件"));
    }
    if (!QDir().mkpath(mapsDirectory))
        return fail(error, QStringLiteral("无法创建地图目录: %1").arg(mapsDirectory));

    QSaveFile record(lastMapRecordPath(mapsDirectory));
    if (!record.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return fail(error, QStringLiteral("无法写入最近地图记录: %1")
                               .arg(record.errorString()));
    }
    const QByteArray pathBytes =
        normalizedExistingPath(mapPath).toUtf8() + '\n';
    if (record.write(pathBytes) != pathBytes.size())
        return fail(error, QStringLiteral("写入最近地图记录失败"));
    if (!record.commit())
        return fail(error, QStringLiteral("提交最近地图记录失败: %1")
                               .arg(record.errorString()));
    return true;
}

QStringList automaticRestoreCandidates()
{
    return automaticRestoreCandidates(defaultMapsDirectory());
}

QStringList automaticRestoreCandidates(const QString &mapsDirectory)
{
    QStringList candidates;
    QSet<QString> seen;
    const auto appendIfValid = [&](const QString &path,
                                   QStringList &output,
                                   QSet<QString> &dedup) {
        const QFileInfo info(path);
        if (!info.exists() || !info.isFile()
            || info.suffix().compare(QStringLiteral("slammap"),
                                     Qt::CaseInsensitive) != 0) {
            return;
        }
        const QString normalized = normalizedExistingPath(path);
        const QString key = normalized.toCaseFolded();
        if (!dedup.contains(key)) {
            dedup.insert(key);
            output.append(normalized);
        }
    };

    QFile record(lastMapRecordPath(mapsDirectory));
    if (record.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QString remembered = QString::fromUtf8(record.readAll()).trimmed();
        if (QDir::isRelativePath(remembered))
            remembered = QDir(mapsDirectory).absoluteFilePath(remembered);
        appendIfValid(remembered, candidates, seen);
    }

    QDir directory(mapsDirectory);
    QFileInfoList files = directory.entryInfoList(
        QStringList{QStringLiteral("*.slammap")},
        QDir::Files | QDir::Readable,
        QDir::NoSort);
    std::sort(files.begin(), files.end(),
              [](const QFileInfo &left, const QFileInfo &right) {
                  if (left.lastModified() != right.lastModified())
                      return left.lastModified() > right.lastModified();
                  return left.fileName() > right.fileName();
              });
    for (const QFileInfo &fileInfo : files)
        appendIfValid(fileInfo.absoluteFilePath(), candidates, seen);
    return candidates;
}

}  // namespace slam_map_io

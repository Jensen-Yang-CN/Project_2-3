#include "SlamMapGeoCorrector.h"
#include "SlamMapBerthGeometry.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace slam_map_geo_correct {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPositionOutlierM = 5.0;
constexpr double kYawOutlierRad = 5.0 * kPi / 180.0;
constexpr double kMaxReferenceSyncErrorSec = 0.25;

struct CorrectionSample {
    double timestamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

bool finitePose(const Eigen::Isometry3d &pose)
{
    if (!pose.matrix().allFinite())
        return false;
    const Eigen::Quaterniond quaternion(pose.linear());
    return std::isfinite(quaternion.norm()) && quaternion.norm() > 1.0e-9;
}

double normalizeAngle(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

double unwrapNear(double angle, double reference)
{
    while (angle - reference > kPi)
        angle -= 2.0 * kPi;
    while (angle - reference < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

double median(std::vector<double> values)
{
    if (values.empty())
        return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if ((values.size() & 1U) != 0U)
        return upper;
    std::nth_element(values.begin(), values.begin() + middle - 1,
                     values.begin() + middle);
    return 0.5 * (values[middle - 1] + upper);
}

CorrectionSample medianWindow(const std::vector<CorrectionSample> &samples,
                              std::size_t center, bool excludeCenter)
{
    const std::size_t begin = center > 2 ? center - 2 : 0;
    const std::size_t end = std::min(samples.size(), center + 3);
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws;
    for (std::size_t index = begin; index < end; ++index) {
        if (excludeCenter && index == center)
            continue;
        xs.push_back(samples[index].x);
        ys.push_back(samples[index].y);
        yaws.push_back(samples[index].yaw);
    }
    CorrectionSample result = samples[center];
    result.x = median(std::move(xs));
    result.y = median(std::move(ys));
    result.yaw = median(std::move(yaws));
    return result;
}

CorrectionSample interpolate(const std::vector<CorrectionSample> &samples,
                             double timestamp)
{
    if (timestamp <= samples.front().timestamp)
        return samples.front();
    if (timestamp >= samples.back().timestamp)
        return samples.back();
    const auto after = std::upper_bound(
        samples.begin(), samples.end(), timestamp,
        [](double value, const CorrectionSample &sample) {
            return value < sample.timestamp;
        });
    const CorrectionSample &right = *after;
    const CorrectionSample &left = *(after - 1);
    const double duration = right.timestamp - left.timestamp;
    const double ratio = duration > 1.0e-9
        ? std::clamp((timestamp - left.timestamp) / duration, 0.0, 1.0)
        : 0.0;
    CorrectionSample result;
    result.timestamp = timestamp;
    result.x = left.x + ratio * (right.x - left.x);
    result.y = left.y + ratio * (right.y - left.y);
    result.yaw = left.yaw + ratio * (right.yaw - left.yaw);
    return result;
}

Eigen::Isometry3d planarCorrection(const CorrectionSample &sample)
{
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.linear() = Eigen::AngleAxisd(
        sample.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    correction.translation() = Eigen::Vector3d(sample.x, sample.y, 0.0);
    return correction;
}

}  // namespace

Result correct(const slam_map_io::MapArchive &archive)
{
    Result result;
    result.archive = archive;

    std::vector<CorrectionSample> raw;
    raw.reserve(archive.keyframes.size());
    for (const usv::SlamKeyframe &keyframe : archive.keyframes) {
        const usv::SlamGeoPoseReference &reference = keyframe.geo_reference;
        if (!reference.valid)
            continue;
        if (!std::isfinite(keyframe.timestamp)
            || !std::isfinite(reference.timestamp)
            || !std::isfinite(reference.sync_error_sec)
            || reference.sync_error_sec < 0.0
            || reference.sync_error_sec > kMaxReferenceSyncErrorSec
            || std::abs(reference.timestamp - keyframe.timestamp) > 1.0e-6
            || !finitePose(keyframe.pose)
            || !finitePose(reference.pose_lidar_enu)) {
            ++result.diagnostic.rejected_references;
            continue;
        }
        const Eigen::Isometry3d correction =
            reference.pose_lidar_enu * keyframe.pose.inverse();
        CorrectionSample sample;
        sample.timestamp = keyframe.timestamp;
        sample.x = correction.translation().x();
        sample.y = correction.translation().y();
        sample.yaw = std::atan2(correction.linear()(1, 0),
                                correction.linear()(0, 0));
        if (!raw.empty())
            sample.yaw = unwrapNear(sample.yaw, raw.back().yaw);
        raw.push_back(sample);
    }

    if (raw.size() < 3) {
        result.diagnostic.valid_references = raw.size();
        result.diagnostic.message = QStringLiteral(
            "逐关键帧地理参考不足（%1/3），保持原始 SLAM 位姿")
            .arg(raw.size());
        return result;
    }

    std::vector<CorrectionSample> accepted;
    accepted.reserve(raw.size());
    for (std::size_t index = 0; index < raw.size(); ++index) {
        const CorrectionSample local = medianWindow(raw, index, true);
        const double positionResidual = std::hypot(
            raw[index].x - local.x, raw[index].y - local.y);
        const double yawResidual = std::abs(normalizeAngle(
            raw[index].yaw - local.yaw));
        const std::size_t begin = index > 2 ? index - 2 : 0;
        const std::size_t end = std::min(raw.size(), index + 3);
        const std::size_t neighbourCount = end - begin - 1;
        if (neighbourCount >= 2
            && (positionResidual > kPositionOutlierM
                || yawResidual > kYawOutlierRad)) {
            ++result.diagnostic.rejected_references;
            continue;
        }
        accepted.push_back(raw[index]);
    }

    if (accepted.size() < 3) {
        result.diagnostic.valid_references = accepted.size();
        result.diagnostic.message = QStringLiteral(
            "地理参考剔除异常值后不足（%1/3），保持原始 SLAM 位姿")
            .arg(accepted.size());
        return result;
    }

    std::vector<CorrectionSample> smoothed;
    smoothed.reserve(accepted.size());
    for (std::size_t index = 0; index < accepted.size(); ++index)
        smoothed.push_back(medianWindow(accepted, index, false));

    std::vector<double> translationMagnitudes;
    translationMagnitudes.reserve(smoothed.size());
    for (const CorrectionSample &sample : smoothed) {
        const double translation = std::hypot(sample.x, sample.y);
        translationMagnitudes.push_back(translation);
        result.diagnostic.max_translation_m = std::max(
            result.diagnostic.max_translation_m, translation);
        result.diagnostic.max_yaw_deg = std::max(
            result.diagnostic.max_yaw_deg,
            std::abs(normalizeAngle(sample.yaw)) * 180.0 / kPi);
    }
    result.diagnostic.median_translation_m =
        median(std::move(translationMagnitudes));

    for (usv::SlamKeyframe &keyframe : result.archive.keyframes) {
        const CorrectionSample sample = interpolate(smoothed,
                                                    keyframe.timestamp);
        keyframe.pose = planarCorrection(sample) * keyframe.pose;
    }

    // 泊位图层与关键帧共享原始 SLAM ENU 坐标。地理校正修正关键帧时，
    // 也按泊位记录时间应用同一平面修正，避免加载地图后框与点云分离。
    for (usv::SlamMapBerth &berth : result.archive.berths) {
        if (!std::isfinite(berth.timestamp)
            || !std::isfinite(berth.x) || !std::isfinite(berth.y)
            || !std::isfinite(berth.z) || !std::isfinite(berth.angle_deg)) {
            continue;
        }
        const CorrectionSample sample = interpolate(smoothed,
                                                    berth.timestamp);
        const Eigen::Vector3d corrected = planarCorrection(sample)
            * Eigen::Vector3d(berth.x, berth.y, berth.z);
        berth.x = corrected.x();
        berth.y = corrected.y();
        berth.z = corrected.z();
        int opening_edge = static_cast<int>(berth.opening_edge);
        berth.angle_deg =
            slam_map_berth::normalizeAngleAndOpeningEdge(
                berth.angle_deg + sample.yaw * 180.0 / kPi,
                opening_edge);
        berth.opening_edge = static_cast<int8_t>(opening_edge);
    }

    result.diagnostic.applied = true;
    result.diagnostic.valid_references = accepted.size();
    result.diagnostic.message = QStringLiteral(
        "逐关键帧 ENU 漂移校正已应用：参考=%1，剔除=%2，最大平移=%3 m，最大偏航=%4°")
        .arg(accepted.size())
        .arg(result.diagnostic.rejected_references)
        .arg(result.diagnostic.max_translation_m, 0, 'f', 3)
        .arg(result.diagnostic.max_yaw_deg, 0, 'f', 3);
    return result;
}

}  // namespace slam_map_geo_correct

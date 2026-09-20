#include "BerthDetectionNode.h"

#include "GnssInsGeoUtils.h"

#include <QMetaObject>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>

#ifdef _WIN32
#include <windows.h>
#endif

#include <opencv2/imgproc.hpp>

namespace {

constexpr double kPosEps = 0.05;
constexpr double kSizeEps = 0.10;
constexpr double kAngleEps = 0.5;
constexpr double kFusionIouDuplicateThreshold = 0.20;
constexpr double kLineBerthShortLengthRatio = 2.0 / 3.0;

double normalizeBerthAngle(double angle)
{
    angle = std::fmod(angle, 180.0);
    if (angle < 0.0)
        angle += 180.0;
    return angle;
}

double signedAngleDifference180(double measured, double reference)
{
    double difference = normalizeBerthAngle(measured) - normalizeBerthAngle(reference);
    while (difference > 90.0) {
        difference -= 180.0;
    }
    while (difference < -90.0) {
        difference += 180.0;
    }
    return difference;
}

template <typename T>
void remapEdgesForAxisSwap(std::array<T, 4> &values, bool axes_reversed)
{
    const std::array<T, 4> original = values;
    const int edge_shift = 3 + (axes_reversed ? 2 : 0);
    for (int old_edge = 0; old_edge < 4; ++old_edge) {
        values[(old_edge + edge_shift) % 4] = original[old_edge];
    }
}

void alignBerthAxesToReference(usv::Berth &candidate, const usv::Berth &reference,
                               double size_gate_m, double angle_gate_deg)
{
    const double direct_score = std::abs(candidate.w - reference.w) / size_gate_m
        + std::abs(candidate.l - reference.l) / size_gate_m
        + std::abs(signedAngleDifference180(candidate.angle, reference.angle)) / angle_gate_deg;
    const double raw_swapped_angle = candidate.angle + 90.0;
    const double swapped_angle = normalizeBerthAngle(raw_swapped_angle);
    const double swapped_score = std::abs(candidate.l - reference.w) / size_gate_m
        + std::abs(candidate.w - reference.l) / size_gate_m
        + std::abs(signedAngleDifference180(swapped_angle, reference.angle)) / angle_gate_deg;
    if (swapped_score >= direct_score) {
        return;
    }

    std::swap(candidate.w, candidate.l);
    candidate.angle = swapped_angle;
    const double normalized_delta_rad =
        (raw_swapped_angle - swapped_angle) * M_PI / 180.0;
    const bool axes_reversed = std::cos(normalized_delta_rad) < 0.0;
    const int edge_shift = 3 + (axes_reversed ? 2 : 0);
    if (candidate.opening_edge >= 0 && candidate.opening_edge < 4) {
        candidate.opening_edge = (candidate.opening_edge + edge_shift) % 4;
    }
    remapEdgesForAxisSwap(candidate.edge_point_counts, axes_reversed);
    remapEdgesForAxisSwap(candidate.edge_heights, axes_reversed);
    remapEdgesForAxisSwap(candidate.edge_height_counts, axes_reversed);
    remapEdgesForAxisSwap(candidate.edge_height_valid, axes_reversed);
    remapEdgesForAxisSwap(candidate.ship_edge_distances, axes_reversed);
    remapEdgesForAxisSwap(candidate.ship_edge_distance_valid, axes_reversed);
    remapEdgesForAxisSwap(candidate.ship_edge_distance_lines, axes_reversed);
}

usv::Berth averageBerthWindow(const std::deque<usv::Berth> &samples,
                              const usv::Berth &fallback)
{
    if (samples.empty()) {
        return fallback;
    }

    usv::Berth mean = samples.back();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_w = 0.0;
    double sum_l = 0.0;
    double sum_angle = 0.0;
    const double angle_reference = samples.front().angle;
    for (const usv::Berth &sample : samples) {
        sum_x += sample.cx;
        sum_y += sample.cy;
        sum_w += sample.w;
        sum_l += sample.l;
        sum_angle += angle_reference
            + signedAngleDifference180(sample.angle, angle_reference);
    }
    const double count = static_cast<double>(samples.size());
    mean.cx = sum_x / count;
    mean.cy = sum_y / count;
    mean.w = sum_w / count;
    mean.l = sum_l / count;
    mean.angle = normalizeBerthAngle(sum_angle / count);
    return mean;
}

using ProtocolBerthCorners = std::array<Eigen::Vector2d, 4>;

ProtocolBerthCorners protocolBerthCornersCounterClockwise(const usv::Berth &berth)
{
    const double rad = berth.angle * M_PI / 180.0;
    const double cos_a = std::cos(rad);
    const double sin_a = std::sin(rad);
    const double half_w = berth.w * 0.5;
    const double half_l = berth.l * 0.5;
    const double local_corners[4][2] = {
        {-half_w, -half_l},
        { half_w, -half_l},
        { half_w,  half_l},
        {-half_w,  half_l},
    };

    ProtocolBerthCorners corners;
    for (int i = 0; i < 4; ++i) {
        const double local_x = local_corners[i][0];
        const double local_y = local_corners[i][1];
        corners[i] = Eigen::Vector2d(
            berth.cx + cos_a * local_x - sin_a * local_y,
            berth.cy + sin_a * local_x + cos_a * local_y);
    }
    return corners;
}

double protocolEdgeMidpointDistanceSquared(const ProtocolBerthCorners &corners, int edge)
{
    return (0.5 * (corners[edge] + corners[(edge + 1) % 4])).squaredNorm();
}

int protocolOpeningEdge(const usv::Berth &berth, const ProtocolBerthCorners &corners)
{
    if (berth.kind == usv::BerthKind::UShape
        && berth.opening_edge >= 0 && berth.opening_edge < 4) {
        return berth.opening_edge;
    }
    if (berth.kind == usv::BerthKind::Line) {
        return protocolEdgeMidpointDistanceSquared(corners, 1)
                   <= protocolEdgeMidpointDistanceSquared(corners, 3)
            ? 1
            : 3;
    }

    int nearest_edge = 0;
    for (int edge = 1; edge < 4; ++edge) {
        if (protocolEdgeMidpointDistanceSquared(corners, edge)
            < protocolEdgeMidpointDistanceSquared(corners, nearest_edge)) {
            nearest_edge = edge;
        }
    }
    return nearest_edge;
}

ProtocolBerthCorners protocolOrderedCorners(const usv::Berth &berth)
{
    const ProtocolBerthCorners geometric_corners = protocolBerthCornersCounterClockwise(berth);
    const int opening_edge = protocolOpeningEdge(berth, geometric_corners);
    ProtocolBerthCorners ordered;
    for (int i = 0; i < 4; ++i) {
        ordered[i] = geometric_corners[(opening_edge + 1 + i) % 4];
    }
    return ordered;
}

uint8_t protocolBerthType(const usv::Berth &berth)
{
    return berth.kind == usv::BerthKind::Line ? 1 : 2;
}

QString formatProtocolPreview(const usv::Berth &berth, size_t index)
{
    const ProtocolBerthCorners corners = protocolOrderedCorners(berth);
    const bool is_fake = berth.source == usv::BerthDetectionSource::Map;
    const QString state = is_fake
        ? QStringLiteral("valid=0(假泊位/蓝色回显)")
        : QStringLiteral("valid=1(真实泊位)");
    return QStringLiteral("[泊位协议预览] #%1 %2 type=%3 P1=(%4,%5,0.00) "
                          "P2=(%6,%7,0.00) P3=(%8,%9,0.00) P4=(%10,%11,0.00)")
        .arg(index + 1)
        .arg(state)
        .arg(protocolBerthType(berth))
        .arg(corners[0].x(), 0, 'f', 2)
        .arg(corners[0].y(), 0, 'f', 2)
        .arg(corners[1].x(), 0, 'f', 2)
        .arg(corners[1].y(), 0, 'f', 2)
        .arg(corners[2].x(), 0, 'f', 2)
        .arg(corners[2].y(), 0, 'f', 2)
        .arg(corners[3].x(), 0, 'f', 2)
        .arg(corners[3].y(), 0, 'f', 2);
}

QString formatBerthKind(usv::BerthKind kind)
{
    switch (kind) {
    case usv::BerthKind::UShape:
        return QStringLiteral("U型");
    case usv::BerthKind::Line:
        return QStringLiteral("一字");
    default:
        return QStringLiteral("未知");
    }
}

QString formatBerthSource(usv::BerthDetectionSource source)
{
    return source == usv::BerthDetectionSource::Map
        ? QStringLiteral("地图")
        : QStringLiteral("实时");
}

double poseYawDeg(const Eigen::Isometry3d &pose)
{
    return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0)) * 180.0 / M_PI;
}

usv::DebugLine2D transformDebugLine(const usv::DebugLine2D &line,
                                    const Eigen::Isometry3d &transform)
{
    const Eigen::Vector3d p1 = transform * Eigen::Vector3d(line.x1, line.y1, 0.0);
    const Eigen::Vector3d p2 = transform * Eigen::Vector3d(line.x2, line.y2, 0.0);
    return usv::DebugLine2D{p1.x(), p1.y(), p2.x(), p2.y()};
}

bool validDebugLine(const usv::DebugLine2D &line)
{
    return std::isfinite(line.x1) && std::isfinite(line.y1)
        && std::isfinite(line.x2) && std::isfinite(line.y2)
        && std::hypot(line.x2 - line.x1, line.y2 - line.y1) > 1e-3;
}

Eigen::Vector2d lineDirection(const usv::DebugLine2D &line)
{
    Eigen::Vector2d direction(line.x2 - line.x1, line.y2 - line.y1);
    const double length = direction.norm();
    if (length > 1e-9) {
        return direction / length;
    }
    return Eigen::Vector2d::Zero();
}

double lineAngleDeg(const usv::DebugLine2D &line)
{
    return normalizeBerthAngle(std::atan2(line.y2 - line.y1, line.x2 - line.x1)
                               * 180.0 / M_PI);
}

double pointToLineSignedDistance(const Eigen::Vector2d &point,
                                 const usv::DebugLine2D &line)
{
    const Eigen::Vector2d direction = lineDirection(line);
    const Eigen::Vector2d normal(-direction.y(), direction.x());
    return (point - Eigen::Vector2d(line.x1, line.y1)).dot(normal);
}

double lineMidpointDistance(const usv::DebugLine2D &a, const usv::DebugLine2D &b)
{
    const Eigen::Vector2d midpoint_a(0.5 * (a.x1 + a.x2), 0.5 * (a.y1 + a.y2));
    return std::abs(pointToLineSignedDistance(midpoint_a, b));
}

bool alignUShapeBerthToAnchorBus(usv::Berth &berth, const usv::DebugLine2D &bus,
                                 double pull_distance_m, int *aligned_edge = nullptr,
                                 double *original_offset_m = nullptr)
{
    if (berth.kind != usv::BerthKind::UShape || !validDebugLine(bus)
        || !(berth.w > 1e-3) || !(berth.l > 1e-3)) {
        return false;
    }

    const ProtocolBerthCorners original_corners = protocolBerthCornersCounterClockwise(berth);
    int nearest_edge = 0;
    double nearest_offset = std::numeric_limits<double>::infinity();
    for (int edge = 0; edge < 4; ++edge) {
        const Eigen::Vector2d midpoint = 0.5 * (
            original_corners[edge] + original_corners[(edge + 1) % 4]);
        const double offset = std::abs(pointToLineSignedDistance(midpoint, bus));
        if (offset < nearest_offset) {
            nearest_offset = offset;
            nearest_edge = edge;
        }
    }
    if (nearest_offset > pull_distance_m) {
        return false;
    }

    // Keep the edge nearest to the bus as the same logical edge.  Its direction is
    // forced to the bus direction even when the stored rectangle was originally skewed.
    const double bus_angle = lineAngleDeg(bus);
    berth.angle = normalizeBerthAngle(bus_angle - (nearest_edge % 2 == 0 ? 0.0 : 90.0));
    const ProtocolBerthCorners rotated_corners = protocolBerthCornersCounterClockwise(berth);
    const Eigen::Vector2d rotated_midpoint = 0.5 * (
        rotated_corners[nearest_edge] + rotated_corners[(nearest_edge + 1) % 4]);
    const Eigen::Vector2d bus_direction = lineDirection(bus);
    const Eigen::Vector2d bus_normal(-bus_direction.y(), bus_direction.x());
    const double signed_offset = pointToLineSignedDistance(rotated_midpoint, bus);
    berth.cx -= signed_offset * bus_normal.x();
    berth.cy -= signed_offset * bus_normal.y();
    // Preserve the detector/protocol edge convention.  The bus only aligns
    // the rectangle; its nearest side may be the fixed wall, not the opening.
    if (berth.opening_edge < 0 || berth.opening_edge >= 4)
        berth.opening_edge = nearest_edge;

    const ProtocolBerthCorners aligned_corners = protocolBerthCornersCounterClockwise(berth);
    berth.has_opening_segment = true;
    const int opening_edge = berth.opening_edge;
    berth.opening_segment = usv::DebugLine2D{
        aligned_corners[opening_edge].x(), aligned_corners[opening_edge].y(),
        aligned_corners[(opening_edge + 1) % 4].x(),
        aligned_corners[(opening_edge + 1) % 4].y()};
    if (aligned_edge) {
        *aligned_edge = nearest_edge;
    }
    if (original_offset_m) {
        *original_offset_m = nearest_offset;
    }
    return true;
}

template <typename T>
void remapOppositeEdges(std::array<T, 4> &values)
{
    const std::array<T, 4> original = values;
    for (int edge = 0; edge < 4; ++edge) {
        values[(edge + 2) % 4] = original[edge];
    }
}

bool flipUShapeBerthAcrossAnchorBus(usv::Berth &berth, const usv::DebugLine2D &bus,
                                    int expected_side)
{
    if (berth.kind != usv::BerthKind::UShape || expected_side == 0
        || !validDebugLine(bus)) {
        return false;
    }
    const Eigen::Vector2d center(berth.cx, berth.cy);
    const double signed_distance = pointToLineSignedDistance(center, bus);
    const int current_side = signed_distance > 0.20 ? 1 : (signed_distance < -0.20 ? -1 : 0);
    if (current_side == 0 || current_side == expected_side) {
        return false;
    }

    const Eigen::Vector2d direction = lineDirection(bus);
    const Eigen::Vector2d normal(-direction.y(), direction.x());
    berth.cx -= 2.0 * signed_distance * normal.x();
    berth.cy -= 2.0 * signed_distance * normal.y();
    if (berth.opening_edge >= 0 && berth.opening_edge < 4) {
        berth.opening_edge = (berth.opening_edge + 2) % 4;
    }
    remapOppositeEdges(berth.edge_point_counts);
    remapOppositeEdges(berth.edge_heights);
    remapOppositeEdges(berth.edge_height_counts);
    remapOppositeEdges(berth.edge_height_valid);
    remapOppositeEdges(berth.ship_edge_distances);
    remapOppositeEdges(berth.ship_edge_distance_valid);
    berth.ship_edge_distance_lines = {};
    berth.has_ship_metrics = false;
    berth.visible_edges.clear();

    if (berth.opening_edge >= 0 && berth.opening_edge < 4) {
        const ProtocolBerthCorners corners = protocolBerthCornersCounterClockwise(berth);
        const int edge = berth.opening_edge;
        berth.has_opening_segment = true;
        berth.opening_segment = usv::DebugLine2D{
            corners[edge].x(), corners[edge].y(),
            corners[(edge + 1) % 4].x(), corners[(edge + 1) % 4].y()};
    }
    return true;
}

usv::Berth transformBerth(const usv::Berth &input, const Eigen::Isometry3d &transform,
                          usv::BerthDetectionSource source)
{
    usv::Berth result = input;
    const Eigen::Vector3d p = transform * Eigen::Vector3d(input.cx, input.cy, 0.0);
    const double transformed_angle = input.angle + poseYawDeg(transform);
    result.cx = p.x();
    result.cy = p.y();
    result.angle = normalizeBerthAngle(transformed_angle);
    result.source = source;

    // Angles are stored modulo 180 degrees. Crossing that boundary preserves the
    // rectangle geometry but reverses its local axes, so every physical edge moves
    // to the opposite edge index.
    const double normalized_delta_rad =
        (transformed_angle - result.angle) * M_PI / 180.0;
    const bool local_axes_reversed = std::cos(normalized_delta_rad) < 0.0;
    if (local_axes_reversed) {
        if (result.opening_edge >= 0 && result.opening_edge < 4) {
            result.opening_edge = (result.opening_edge + 2) % 4;
        }
        remapOppositeEdges(result.edge_point_counts);
        remapOppositeEdges(result.edge_heights);
        remapOppositeEdges(result.edge_height_counts);
        remapOppositeEdges(result.edge_height_valid);
        remapOppositeEdges(result.ship_edge_distances);
        remapOppositeEdges(result.ship_edge_distance_valid);
        remapOppositeEdges(result.ship_edge_distance_lines);
    }

    if (result.has_opening_segment) {
        result.opening_segment = transformDebugLine(input.opening_segment, transform);
    }
    for (usv::DebugLine2D &edge : result.visible_edges) {
        edge = transformDebugLine(edge, transform);
    }
    for (usv::DebugLine2D &line : result.ship_edge_distance_lines) {
        line = transformDebugLine(line, transform);
    }
    return result;
}

bool sameBerthCandidate(const usv::Berth &a, const usv::Berth &b)
{
    if (a.kind != b.kind) {
        return false;
    }
    const double center_distance = std::hypot(a.cx - b.cx, a.cy - b.cy);
    if (center_distance < 5.0) {
        return true;
    }

    const double area_a = a.w * a.l;
    const double area_b = b.w * b.l;
    if (!(area_a > 1e-6) || !(area_b > 1e-6)) {
        return false;
    }
    const cv::RotatedRect rect_a(
        cv::Point2f(static_cast<float>(a.cx), static_cast<float>(a.cy)),
        cv::Size2f(static_cast<float>(a.w), static_cast<float>(a.l)),
        static_cast<float>(a.angle));
    const cv::RotatedRect rect_b(
        cv::Point2f(static_cast<float>(b.cx), static_cast<float>(b.cy)),
        cv::Size2f(static_cast<float>(b.w), static_cast<float>(b.l)),
        static_cast<float>(b.angle));
    std::vector<cv::Point2f> intersection;
    const int intersection_type = cv::rotatedRectangleIntersection(rect_a, rect_b, intersection);
    if (intersection_type == cv::INTERSECT_NONE) {
        return false;
    }
    const double intersection_area = intersection_type == cv::INTERSECT_FULL
        ? std::min(area_a, area_b)
        : (intersection.size() >= 3 ? std::abs(cv::contourArea(intersection)) : 0.0);
    const double union_area = area_a + area_b - intersection_area;
    return union_area > 1e-6 && intersection_area / union_area >= kFusionIouDuplicateThreshold;
}

void updateLibraryBerth(usv::Berth &stored, const usv::Berth &incoming)
{
    const double locked_line_length = stored.kind == usv::BerthKind::Line && stored.l > 1e-6
        ? stored.l
        : 0.0;
    stored = incoming;
    if (locked_line_length > 0.0 && stored.kind == usv::BerthKind::Line) {
        stored.l = locked_line_length;
    }
}

bool berthNearlyEqual(const usv::Berth &a, const usv::Berth &b)
{
        const bool geometryEqual = std::abs(a.cx - b.cx) < kPosEps
        && std::abs(a.cy - b.cy) < kPosEps
        && std::abs(a.w - b.w) < kSizeEps
        && std::abs(a.l - b.l) < kSizeEps
        && std::abs(a.angle - b.angle) < kAngleEps
        && a.kind == b.kind
        && a.source == b.source
        && a.opening_edge == b.opening_edge;
    if (!geometryEqual || a.edge_point_counts != b.edge_point_counts ||
        a.edge_height_counts != b.edge_height_counts ||
        a.edge_height_valid != b.edge_height_valid ||
        a.ship_edge_distance_valid != b.ship_edge_distance_valid ||
        a.has_ship_metrics != b.has_ship_metrics) {
        return false;
    }
    for (int edge = 0; edge < 4; ++edge) {
        if (std::abs(a.edge_heights[edge] - b.edge_heights[edge]) > 0.05) {
            return false;
        }
        if (std::abs(a.ship_edge_distances[edge] - b.ship_edge_distances[edge]) > 0.05) {
            return false;
        }
    }
    if (std::abs(a.ship_center_distance - b.ship_center_distance) > 0.05 ||
        std::abs(a.ship_opening_normal_angle - b.ship_opening_normal_angle) > 0.5) {
        return false;
    }
    return true;
}

usv::DebugLine2D berthEdgeSegment(const usv::Berth &berth, int edge)
{
    const ProtocolBerthCorners corners = protocolBerthCornersCounterClockwise(berth);
    const int e0 = ((edge % 4) + 4) % 4;
    const int e1 = (e0 + 1) % 4;
    return usv::DebugLine2D{
        corners[e0].x(), corners[e0].y(),
        corners[e1].x(), corners[e1].y()};
}

usv::Berth toCommonBerth(const usv::LineBerth &line)
{
    usv::Berth berth;
    berth.cx = line.cx;
    berth.cy = line.cy;
    berth.w = line.width;
    berth.l = line.length;
    berth.angle = normalizeBerthAngle(line.angle - 90.0);
    berth.kind = usv::BerthKind::Line;
    // 一字泊位：两条长边(1/3)中离传感器更近的一侧为开口，界面据此跳过该边绘制
    const ProtocolBerthCorners corners = protocolBerthCornersCounterClockwise(berth);
    berth.opening_edge = protocolOpeningEdge(berth, corners);
    berth.opening_segment = berthEdgeSegment(berth, berth.opening_edge);
    berth.has_opening_segment = true;
    berth.visible_edges.clear();
    berth.visible_edges.reserve(3);
    for (int edge = 0; edge < 4; ++edge) {
        if (edge == berth.opening_edge)
            continue;
        berth.visible_edges.push_back(berthEdgeSegment(berth, edge));
    }
    // 支撑点落在泊位线（开口对边）上，便于高度/统计显示
    const int wall_edge = (berth.opening_edge + 2) % 4;
    berth.edge_point_counts[wall_edge] = line.edge.support_points;
    berth.edge_height_counts[wall_edge] = line.edge.support_points;
    if (std::isfinite(line.edge.support_z_mean)) {
        berth.edge_heights[wall_edge] = line.edge.support_z_mean;
        berth.edge_height_valid[wall_edge] = true;
    }
    berth.ship_center_distance = std::hypot(line.cx, line.cy);
    berth.has_ship_metrics = true;
    return berth;
}

usv::BerthMeasureResult convertLineResult(const usv::LineBerthMeasureResult &line_res)
{
    usv::BerthMeasureResult res;
    res.timestamp = line_res.timestamp;
    res.is_detected = line_res.is_detected;
    res.is_using_memory = line_res.is_using_memory;
    if (!line_res.is_detected) {
        return res;
    }

    std::vector<usv::LineBerth> line_berths = line_res.berths;
    if (line_berths.empty()) {
        line_berths.push_back(line_res.berth);
    }

    for (int i = 0; i < static_cast<int>(line_berths.size()); ++i) {
        const usv::LineBerth &line_berth = line_berths[i];
        usv::Berth berth = toCommonBerth(line_berth);
        res.expanded_berths.push_back(berth);
        res.debug_lines.push_back(usv::DebugLine2D{
            line_berth.edge.x1,
            line_berth.edge.y1,
            line_berth.edge.x2,
            line_berth.edge.y2
        });

        QString msg = QStringLiteral("[一字泊位] #%1 中心=(%2,%3) 尺寸=%4x%5m 角度=%6° line_len=%7m dist=%8m support=%9 inside=%10 sidePts=%11")
            .arg(i + 1)
            .arg(berth.cx, 0, 'f', 2)
            .arg(berth.cy, 0, 'f', 2)
            .arg(berth.w, 0, 'f', 2)
            .arg(berth.l, 0, 'f', 2)
            .arg(berth.angle, 0, 'f', 1)
            .arg(line_berth.edge.length, 0, 'f', 2)
            .arg(line_berth.edge.distance_to_sensor, 0, 'f', 2)
            .arg(line_berth.edge.support_points)
            .arg(line_berth.inner_point_count)
            .arg(line_berth.berth_side_point_count);
        if (std::isfinite(line_berth.edge.support_z_mean)) {
            msg += QStringLiteral(" zmean=%1m")
                .arg(line_berth.edge.support_z_mean, 0, 'f', 2);
        }
        if (line_res.is_using_memory) {
            msg += QStringLiteral(" memory=1");
        }
        res.debug_messages.push_back(msg.toStdString());
    }
    return res;
}

void markBerthsAsUShape(usv::BerthMeasureResult &res)
{
    for (usv::Berth &berth : res.expanded_berths) {
        berth.kind = usv::BerthKind::UShape;
    }
}

void appendBerthResult(usv::BerthMeasureResult &dst, const usv::BerthMeasureResult &src)
{
    if (src.timestamp > 0.0) {
        dst.timestamp = src.timestamp;
    }
    dst.is_detected = dst.is_detected || src.is_detected;
    dst.is_using_memory = dst.is_using_memory || src.is_using_memory;
    dst.is_line_recovered = dst.is_line_recovered || src.is_line_recovered;
    if (src.has_anchor_bus) {
        dst.has_anchor_bus = true;
        dst.anchor_bus_line = src.anchor_bus_line;
    }

    dst.expanded_berths.insert(dst.expanded_berths.end(),
        src.expanded_berths.begin(), src.expanded_berths.end());
    dst.debug_rois.insert(dst.debug_rois.end(), src.debug_rois.begin(), src.debug_rois.end());
    dst.top_search_rois.insert(dst.top_search_rois.end(),
        src.top_search_rois.begin(), src.top_search_rois.end());
    dst.debug_lines.insert(dst.debug_lines.end(), src.debug_lines.begin(), src.debug_lines.end());
    dst.highest_points.insert(dst.highest_points.end(),
        src.highest_points.begin(), src.highest_points.end());
    dst.second_highest_points.insert(dst.second_highest_points.end(),
        src.second_highest_points.begin(), src.second_highest_points.end());
    dst.debug_messages.insert(dst.debug_messages.end(),
        src.debug_messages.begin(), src.debug_messages.end());

    if (!dst.has_debug_roi && src.has_debug_roi) {
        dst.has_debug_roi = true;
        dst.debug_roi = src.debug_roi;
    }
    if (!dst.has_top_search_roi && src.has_top_search_roi) {
        dst.has_top_search_roi = true;
        dst.top_search_roi = src.top_search_roi;
    }
    if (!dst.has_highest_point && src.has_highest_point) {
        dst.has_highest_point = true;
        dst.highest_point = src.highest_point;
    }
    if (!dst.has_second_highest_point && src.has_second_highest_point) {
        dst.has_second_highest_point = true;
        dst.second_highest_point = src.second_highest_point;
    }
    if (src.anchor_debug.status != usv::AnchorRecoveryStatus::NotAttempted) {
        dst.anchor_debug = src.anchor_debug;
    }
}

QString formatEdgePointCounts(const usv::Berth &b)
{
    QString text;
    for (int edge = 0; edge < 4; ++edge) {
        text += QStringLiteral(" e%1=%2")
            .arg(edge)
            .arg(b.edge_point_counts[edge]);
    }
    return text;
}

QString formatEdgeHeights(const usv::Berth &b)
{
    QString text;
    for (int edge = 0; edge < 4; ++edge) {
        text += QStringLiteral(" e%1=").arg(edge);
        if (edge == b.opening_edge) {
            text += QStringLiteral("opening");
        } else if (b.edge_height_valid[edge]) {
            text += QStringLiteral("%1m(%2tops)")
                .arg(b.edge_heights[edge], 0, 'f', 2)
                .arg(b.edge_height_counts[edge]);
        } else {
            text += QStringLiteral("invalid(%1tops)")
                .arg(b.edge_height_counts[edge]);
        }
    }
    return text;
}

QString formatShipMetrics(const usv::Berth &b)
{
    if (!b.has_ship_metrics) {
        return QStringLiteral("invalid");
    }

    QString text = QStringLiteral("center=%1m openAngle=%2° edgeDist:")
        .arg(b.ship_center_distance, 0, 'f', 2)
        .arg(b.ship_opening_normal_angle, 0, 'f', 1);
    for (int edge = 0; edge < 4; ++edge) {
        text += QStringLiteral(" e%1=").arg(edge);
        if (edge == b.opening_edge) {
            text += QStringLiteral("opening");
        } else if (b.ship_edge_distance_valid[edge]) {
            text += QStringLiteral("%1m").arg(b.ship_edge_distances[edge], 0, 'f', 2);
        } else {
            text += QStringLiteral("invalid");
        }
    }
    return text;
}

QString formatAnchorHeights(const usv::BerthMeasureResult &res)
{
    if (res.highest_points.empty() && res.second_highest_points.empty()) {
        return {};
    }

    QString text = QStringLiteral("[泊位锚点高度]");
    for (size_t i = 0; i < res.highest_points.size(); ++i) {
        text += QStringLiteral(" A%1=%2m")
            .arg(i + 1)
            .arg(res.highest_points[i].z(), 0, 'f', 2);
    }
    for (size_t i = 0; i < res.second_highest_points.size(); ++i) {
        text += QStringLiteral(" B%1=%2m")
            .arg(i + 1)
            .arg(res.second_highest_points[i].z(), 0, 'f', 2);
    }
    return text;
}

} // namespace

BerthDetectionNode::BerthDetectionNode(QObject *parent)
    : QObject(parent)
    , comm_(&usv::CommunicationManager::instance())
    , navigation_enu_state_(std::make_unique<usv::gnss_geo::GnssEnuState>())
{
}

BerthDetectionNode::~BerthDetectionNode()
{
    stop();
}

void BerthDetectionNode::Init()
{
    u_shape_algo_.init();
    line_algo_.init();
    last_log_ = {};

    sub_sync_ = comm_->getStateTopic<usv::SyncedSensorPackage>("/preprocess/synced_sensors", 16);
    pub_result_ = comm_->getStateTopic<usv::BerthMeasureResult>("/perception/berth_result", 16);

    sub_id_ = sub_sync_->subscribe([this](usv::SyncedPackagePtr msg) {
        if (msg) {
            realtime_received_frames_.fetch_add(1, std::memory_order_relaxed);
            sync_queue_.push(msg);
        }
    });
}

void BerthDetectionNode::feedNavigationData(const IMUParsedData &data)
{
    if (data.timestamp <= 0.0 || (data.longitude == 0.0 && data.latitude == 0.0)) {
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!navigation_pose_history_.empty()
        && data.timestamp + 1.0 < navigation_pose_history_.back().timestamp) {
        // A delayed/out-of-order navigation packet must not reset the world
        // frame during one algorithm run. A real PCAP/world-frame change is
        // handled explicitly by resetWorldCoordinateState().
        return;
    }

    const usv::GnssInsMessage gnss = usv::gnss_geo::gnssFromImu(data);
    usv::gnss_geo::updateGnssEnuState(gnss, *navigation_enu_state_);
    if (!navigation_enu_state_->initialized) {
        return;
    }

    const Eigen::Isometry3d body_pose =
        usv::gnss_geo::bodyPoseInEnu(gnss, *navigation_enu_state_);
    NavigationPoseState state;
    state.timestamp = data.timestamp;
    state.pose_lidar = body_pose * usv::gnss_geo::lidarToBodyExtrinsic();

    if (!navigation_pose_history_.empty()
        && std::abs(navigation_pose_history_.back().timestamp - state.timestamp) < 1e-4) {
        navigation_pose_history_.back() = state;
    } else {
        navigation_pose_history_.push_back(state);
        while (navigation_pose_history_.size() > max_navigation_pose_history_) {
            navigation_pose_history_.pop_front();
        }
    }
}

bool BerthDetectionNode::lookupNavigationPose(double timestamp, Eigen::Isometry3d &pose,
                                               double *out_dt) const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    const NavigationPoseState *best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    for (const NavigationPoseState &state : navigation_pose_history_) {
        const double dt = std::abs(state.timestamp - timestamp);
        if (dt < best_dt) {
            best = &state;
            best_dt = dt;
        }
    }
    if (out_dt) {
        *out_dt = best_dt;
    }
    if (!best || best_dt > max_navigation_pose_sync_sec_) {
        return false;
    }
    pose = best->pose_lidar;
    return true;
}

std::vector<bool> BerthDetectionNode::confirmAndUpdateRealtimeCoordinateLibrary(
    const std::vector<usv::Berth> &berths,
    std::vector<std::string> &debug_messages)
{
    std::vector<bool> output_confirmed(berths.size(), false);
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::vector<usv::Berth> &library = realtime_navigation_coordinate_berths_;
    std::vector<RealtimeBerthConfirmation> &confirmations =
        realtime_navigation_confirmations_;
    std::vector<bool> confirmation_used(confirmations.size(), false);
    for (size_t berth_index = 0; berth_index < berths.size(); ++berth_index) {
        usv::Berth berth = berths[berth_index];
        RealtimeBerthConfirmation *confirmation = nullptr;
        for (size_t i = 0; i < confirmations.size(); ++i) {
            if (!confirmation_used[i] && sameBerthCandidate(confirmations[i].berth, berth)) {
                confirmation = &confirmations[i];
                confirmation_used[i] = true;
                break;
            }
        }
        if (confirmation) {
            // Every berth reaching this function has already passed the world
            // stability filter. Do not compare it with the stale first-confirmation
            // geometry again, otherwise a successfully replaced track can remain
            // permanently stuck at 1/3.
            confirmation->berth = berth;
            ++confirmation->hit_count;
        } else {
            confirmations.push_back(RealtimeBerthConfirmation{berth, 1});
            confirmation_used.push_back(true);
            confirmation = &confirmations.back();
        }

        // Store the first accepted observation immediately. Subsequent updates are
        // also accepted here because the world stability stage has already filtered
        // jumps and handled replacement-window convergence.
        auto it = std::find_if(library.begin(), library.end(),
            [&berth](const usv::Berth &known) { return sameBerthCandidate(known, berth); });
        if (it == library.end()) {
            library.push_back(berth);
        } else {
            updateLibraryBerth(*it, berth);
        }

        output_confirmed[berth_index] = confirmation->hit_count
            >= realtime_output_confirmation_frames_;
        if (!output_confirmed[berth_index]) {
            debug_messages.push_back(
                "[BerthOutputConfirm] pending kind="
                + std::to_string(static_cast<int>(berth.kind))
                + " samples=" + std::to_string(confirmation->hit_count)
                + "/" + std::to_string(realtime_output_confirmation_frames_));
            continue;
        }
    }
    return output_confirmed;
}

std::vector<bool> BerthDetectionNode::stabilizeRealtimeWorldBerths(
    std::vector<usv::Berth> &berths,
    std::vector<std::string> &debug_messages)
{
    std::vector<bool> accepted(berths.size(), false);

    std::lock_guard<std::mutex> lock(map_mutex_);
    std::vector<WorldBerthStabilityState> &states =
        realtime_navigation_stability_states_;
    std::vector<bool> state_observed(states.size(), false);

    for (size_t i = 0; i < berths.size(); ++i) {
        usv::Berth &measurement = berths[i];
        if (measurement.kind != usv::BerthKind::UShape) {
            accepted[i] = true;
            continue;
        }
        size_t state_index = states.size();
        double best_distance = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < states.size(); ++j) {
            const usv::Berth &reference = states[j].last_accepted_berth;
            if (reference.kind != measurement.kind) {
                continue;
            }
            const double center_distance = std::hypot(
                measurement.cx - reference.cx, measurement.cy - reference.cy);
            if (center_distance > stability_association_distance_m_) {
                continue;
            }
            if (center_distance < best_distance) {
                best_distance = center_distance;
                state_index = j;
            }
        }

        if (state_index == states.size()) {
            states.push_back(WorldBerthStabilityState{});
            WorldBerthStabilityState &state = states.back();
            state.last_accepted_berth = measurement;
            state.accepted_window.push_back(measurement);
            state_observed.push_back(true);
            accepted[i] = true;
            debug_messages.push_back(
                "[BerthStability] initialize world track kind="
                + std::to_string(static_cast<int>(measurement.kind))
                + " center=(" + std::to_string(measurement.cx)
                + "," + std::to_string(measurement.cy) + ")");
            continue;
        }

        WorldBerthStabilityState &state = states[state_index];
        state_observed[state_index] = true;
        const usv::Berth window_mean = averageBerthWindow(
            state.accepted_window, state.last_accepted_berth);

        // A rectangle has an equivalent representation after exchanging w/l and
        // rotating its local axes by 90 degrees. Align it before jump checking.
        alignBerthAxesToReference(
            measurement, window_mean, stability_size_gate_m_, stability_angle_gate_deg_);

        const double position_l1 = std::abs(measurement.cx - window_mean.cx)
            + std::abs(measurement.cy - window_mean.cy);
        const double width_jump = std::abs(measurement.w - window_mean.w);
        const double length_jump = std::abs(measurement.l - window_mean.l);
        const double angle_jump = std::abs(
            signedAngleDifference180(measurement.angle, window_mean.angle));
        if (position_l1 >= stability_position_l1_gate_m_
            || width_jump >= stability_size_gate_m_
            || length_jump >= stability_size_gate_m_
            || angle_jump >= stability_angle_gate_deg_) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "[BerthStability] reject realtime world measurement kind="
                << static_cast<int>(measurement.kind)
                << " pos_l1=" << position_l1
                << "m width_jump=" << width_jump
                << "m length_jump=" << length_jump
                << "m angle_jump=" << angle_jump
                << "deg window_samples=" << state.accepted_window.size();
            debug_messages.push_back(oss.str());

            usv::Berth replacement_mean = averageBerthWindow(
                state.replacement_window, measurement);
            alignBerthAxesToReference(
                measurement, replacement_mean,
                stability_size_gate_m_, stability_angle_gate_deg_);
            const double replacement_position_l1 =
                std::abs(measurement.cx - replacement_mean.cx)
                + std::abs(measurement.cy - replacement_mean.cy);
            const double replacement_width_jump =
                std::abs(measurement.w - replacement_mean.w);
            const double replacement_length_jump =
                std::abs(measurement.l - replacement_mean.l);
            const double replacement_angle_jump = std::abs(
                signedAngleDifference180(measurement.angle, replacement_mean.angle));
            const bool replacement_stable = state.replacement_window.empty()
                || (replacement_position_l1 < stability_position_l1_gate_m_
                    && replacement_width_jump < stability_size_gate_m_
                    && replacement_length_jump < stability_size_gate_m_
                    && replacement_angle_jump < stability_angle_gate_deg_);
            if (!replacement_stable) {
                std::ostringstream replacement_oss;
                replacement_oss << std::fixed << std::setprecision(2)
                    << "[BerthStability] reset replacement window kind="
                    << static_cast<int>(measurement.kind)
                    << " pos_l1=" << replacement_position_l1
                    << "m width_jump=" << replacement_width_jump
                    << "m length_jump=" << replacement_length_jump
                    << "m angle_jump=" << replacement_angle_jump << "deg";
                debug_messages.push_back(replacement_oss.str());
                state.replacement_window.clear();
            }
            state.replacement_window.push_back(measurement);

            if (state.replacement_window.size()
                == realtime_replacement_stability_window_frames_) {
                const usv::Berth replacement_result = averageBerthWindow(
                    state.replacement_window, measurement);
                state.last_accepted_berth = replacement_result;
                state.accepted_window = state.replacement_window;
                state.has_stable_geometry = true;
                state.replacement_window.clear();
                measurement = replacement_result;
                accepted[i] = true;
                debug_messages.push_back(
                    "[BerthStability] replace world track with 5-frame stable window kind="
                    + std::to_string(static_cast<int>(measurement.kind))
                    + " center=(" + std::to_string(measurement.cx)
                    + "," + std::to_string(measurement.cy) + ")");
            }
            else {
                debug_messages.push_back(
                    "[BerthStability] replacement window progress kind="
                    + std::to_string(static_cast<int>(measurement.kind))
                    + " samples=" + std::to_string(state.replacement_window.size())
                    + "/" + std::to_string(realtime_replacement_stability_window_frames_));
            }
            continue;
        }

        state.last_accepted_berth = measurement;
        state.replacement_window.clear();
        state.accepted_window.push_back(measurement);
        while (state.accepted_window.size() > realtime_stability_window_frames_) {
            state.accepted_window.pop_front();
        }
        if (state.accepted_window.size() == realtime_stability_window_frames_) {
            state.has_stable_geometry = true;
            measurement = averageBerthWindow(
                state.accepted_window, state.last_accepted_berth);
        }
        accepted[i] = true;
    }
    for (size_t i = 0; i < states.size(); ++i) {
        if (state_observed[i] || states[i].replacement_window.empty()) {
            continue;
        }
        debug_messages.push_back(
            "[BerthStability] reset replacement window after missing frame kind="
            + std::to_string(static_cast<int>(states[i].last_accepted_berth.kind)));
        states[i].replacement_window.clear();
    }
    return accepted;
}

void BerthDetectionNode::updateWorldAnchorBus(const usv::DebugLine2D &world_bus,
                                              std::vector<std::string> &debug_messages)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    WorldAnchorBusState &state = navigation_world_anchor_bus_;
    if (!validDebugLine(world_bus)) {
        state.has_pending = false;
        state.consecutive_frames = 0;
        if (!state.locked) {
            state.stable_consistent_frames = 0;
        }
        return;
    }

    const auto matches = [this](const usv::DebugLine2D &a, const usv::DebugLine2D &b) {
        return validDebugLine(a) && validDebugLine(b)
            && std::abs(signedAngleDifference180(lineAngleDeg(a), lineAngleDeg(b)))
                <= world_anchor_bus_angle_gate_deg_
            && lineMidpointDistance(a, b) <= world_anchor_bus_offset_gate_m_;
    };

    if (state.locked) {
        return;
    }

    const auto update_pending = [&]() {
        // Keep the first candidate as the reference for this confirmation window.
        // Updating that reference every frame would allow small legal offsets to
        // accumulate into an outward random walk.
        if (state.has_pending && matches(world_bus, state.pending_line)) {
            ++state.consecutive_frames;
        } else {
            state.pending_line = world_bus;
            state.has_pending = true;
            state.consecutive_frames = 1;
        }
    };

    if (state.valid && matches(world_bus, state.stable_line)) {
        state.has_pending = false;
        state.consecutive_frames = 0;
        ++state.stable_consistent_frames;
        if (state.stable_consistent_frames >= world_anchor_bus_lock_frames_) {
            state.locked = true;
            debug_messages.push_back(
                "[AnchorBusWorld] lock stable bus after "
                + std::to_string(world_anchor_bus_lock_frames_)
                + " continuous frames");
        }
        return;
    }

    if (state.valid) {
        // Retain the previous solid world bus while a different one is being
        // confirmed. It cannot be replaced by a one- or two-frame anchor jump.
        state.stable_consistent_frames = 0;
    }
    update_pending();

    if (state.consecutive_frames < world_anchor_bus_confirmation_frames_) {
        return;
    }

    const bool replacing = state.valid;
    state.stable_line = state.pending_line;
    state.valid = true;
    state.has_pending = false;
    state.consecutive_frames = 0;
    state.stable_consistent_frames = world_anchor_bus_confirmation_frames_;
    state.berth_side = 0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "[AnchorBusWorld] " << (replacing ? "replace" : "save")
        << " stable bus after " << world_anchor_bus_confirmation_frames_
        << " frames angle=" << lineAngleDeg(state.stable_line)
        << "deg start=(" << state.stable_line.x1 << ',' << state.stable_line.y1
        << ") end=(" << state.stable_line.x2 << ',' << state.stable_line.y2 << ')';
    debug_messages.push_back(oss.str());
}

bool BerthDetectionNode::getWorldAnchorBus(usv::DebugLine2D &world_bus,
                                           bool *locked) const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    const WorldAnchorBusState &state = navigation_world_anchor_bus_;
    if (!state.valid) {
        if (locked) {
            *locked = false;
        }
        return false;
    }
    world_bus = state.stable_line;
    if (locked) {
        *locked = state.locked;
    }
    return true;
}

void BerthDetectionNode::updateWorldAnchorBusBerthSide(
    const std::vector<usv::Berth> &world_berths,
    std::vector<std::string> &debug_messages)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    WorldAnchorBusState &state = navigation_world_anchor_bus_;
    if (!state.valid || state.berth_side != 0) {
        return;
    }

    int positive_count = 0;
    int negative_count = 0;
    for (const usv::Berth &berth : world_berths) {
        if (berth.kind != usv::BerthKind::UShape) {
            continue;
        }
        const ProtocolBerthCorners corners = protocolBerthCornersCounterClockwise(berth);
        double nearest_edge_distance = std::numeric_limits<double>::infinity();
        for (int edge = 0; edge < 4; ++edge) {
            const Eigen::Vector2d midpoint = 0.5 * (
                corners[edge] + corners[(edge + 1) % 4]);
            nearest_edge_distance = std::min(
                nearest_edge_distance,
                std::abs(pointToLineSignedDistance(midpoint, state.stable_line)));
        }
        if (nearest_edge_distance > world_anchor_bus_side_edge_gate_m_) {
            continue;
        }
        const double center_side = pointToLineSignedDistance(
            Eigen::Vector2d(berth.cx, berth.cy), state.stable_line);
        if (center_side > 0.20) {
            ++positive_count;
        } else if (center_side < -0.20) {
            ++negative_count;
        }
    }

    if (positive_count == negative_count) {
        return;
    }
    state.berth_side = positive_count > negative_count ? 1 : -1;
    debug_messages.push_back(
        "[AnchorBusWorld] berth row side=" + std::to_string(state.berth_side)
        + " positive=" + std::to_string(positive_count)
        + " negative=" + std::to_string(negative_count));
}

void BerthDetectionNode::flipLibraryBerthsOppositeWorldAnchorBus(
    std::vector<usv::Berth> &world_berths,
    std::vector<std::string> &debug_messages) const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    const WorldAnchorBusState &state = navigation_world_anchor_bus_;
    if (!state.valid || state.berth_side == 0) {
        return;
    }

    for (usv::Berth &berth : world_berths) {
        if (flipUShapeBerthAcrossAnchorBus(
                berth, state.stable_line, state.berth_side)) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "[AnchorBusWorld] flip library U-shape onto berth-row side"
                << " center=(" << berth.cx << ',' << berth.cy << ')'
                << " row_side=" << state.berth_side;
            debug_messages.push_back(oss.str());
        }
    }
}

void BerthDetectionNode::alignCoordinateLibrariesToWorldAnchorBus(
    std::vector<std::string> &debug_messages)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    const WorldAnchorBusState &bus_state = navigation_world_anchor_bus_;
    if (!bus_state.valid) {
        return;
    }

    const auto align_berth = [this, &bus_state, &debug_messages](usv::Berth &berth,
                                                                   const char *library_name) {
        const usv::Berth before = berth;
        int edge = -1;
        double offset = 0.0;
        if (!alignUShapeBerthToAnchorBus(
                berth, bus_state.stable_line, world_anchor_bus_pull_distance_m_,
                &edge, &offset)) {
            return;
        }
        const bool flipped = flipUShapeBerthAcrossAnchorBus(
            berth, bus_state.stable_line, bus_state.berth_side);
        const double center_shift = std::hypot(berth.cx - before.cx, berth.cy - before.cy);
        const double angle_shift = std::abs(
            signedAngleDifference180(berth.angle, before.angle));
        if (!flipped && center_shift < 1e-3 && angle_shift < 1e-3
            && berth.opening_edge == before.opening_edge) {
            return;
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "[AnchorBusWorld] align " << library_name
            << " U-shape edge=" << edge
            << " original_offset=" << offset << "m"
            << " center_shift=" << center_shift << "m"
            << " angle_shift=" << angle_shift << "deg"
            << " flipped=" << (flipped ? "true" : "false");
        debug_messages.push_back(oss.str());
    };

    std::vector<usv::Berth> &realtime_library = realtime_navigation_coordinate_berths_;
    for (usv::Berth &berth : realtime_library) {
        align_berth(berth, "realtime-library");
    }
    std::vector<WorldBerthStabilityState> &states =
        realtime_navigation_stability_states_;
    for (WorldBerthStabilityState &state : states) {
        align_berth(state.last_accepted_berth, "stable-track");
        for (usv::Berth &sample : state.accepted_window) {
            alignUShapeBerthToAnchorBus(
                sample, bus_state.stable_line, world_anchor_bus_pull_distance_m_);
            flipUShapeBerthAcrossAnchorBus(
                sample, bus_state.stable_line, bus_state.berth_side);
        }
        for (usv::Berth &sample : state.replacement_window) {
            alignUShapeBerthToAnchorBus(
                sample, bus_state.stable_line, world_anchor_bus_pull_distance_m_);
            flipUShapeBerthAcrossAnchorBus(
                sample, bus_state.stable_line, bus_state.berth_side);
        }
    }
}

void BerthDetectionNode::resetRealtimeCoordinateConfirmations()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    realtime_navigation_confirmations_.clear();
    realtime_navigation_stability_states_.clear();
    navigation_world_anchor_bus_ = {};
}

void BerthDetectionNode::resetWorldCoordinateState()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    navigation_pose_history_.clear();
    realtime_coordinate_berths_.clear();
    realtime_navigation_coordinate_berths_.clear();
    navigation_world_recovery_rois_.clear();
    realtime_navigation_confirmations_.clear();
    realtime_navigation_stability_states_.clear();
    navigation_world_anchor_bus_ = {};
    if (navigation_enu_state_) {
        *navigation_enu_state_ = {};
    }
}

void BerthDetectionNode::requestPlaybackTimelineReset(uint64_t sensorTimelineGeneration)
{
    (void)sensorTimelineGeneration;
    sync_queue_.clear();
    resetWorldCoordinateState();
    u_shape_algo_.init();
    line_algo_.init();
    last_log_ = {};
    postLog(QStringLiteral("[泊位检测] PCAP 时间轴已跳转，队列/姿态/跟踪状态已重置"));
}

void BerthDetectionNode::setDetectionModes(bool u_shape_enabled, bool line_enabled)
{
    u_shape_enabled_.store(u_shape_enabled, std::memory_order_relaxed);
    line_enabled_.store(line_enabled, std::memory_order_relaxed);
    postLog(QStringLiteral("[泊位检测] 模式更新：U型=%1 一字=%2")
                .arg(u_shape_enabled ? QStringLiteral("开") : QStringLiteral("关"))
                .arg(line_enabled ? QStringLiteral("开") : QStringLiteral("关")));
}

void BerthDetectionNode::postLog(const QString &text)
{
    QMetaObject::invokeMethod(
        this,
        [this, text]() { emit logMessage(text); },
        Qt::QueuedConnection);
}

void BerthDetectionNode::postBerthResult(const usv::BerthMeasureResult &res)
{
    const usv::BerthMeasureResult copy = res;
    QMetaObject::invokeMethod(
        this,
        [this, copy]() { emit berthResultReady(copy); },
        Qt::QueuedConnection);
}

void BerthDetectionNode::maybeLogDetectionResult(const usv::BerthMeasureResult &res)
{
    if (!res.is_detected) {
        last_log_.initialized = true;
        last_log_.detected = false;
        last_log_.berths.clear();
        return;
    }

    bool changed = !last_log_.initialized || !last_log_.detected
        || last_log_.berths.size() != res.expanded_berths.size();

    if (!changed) {
        for (size_t i = 0; i < res.expanded_berths.size(); ++i) {
            if (!berthNearlyEqual(res.expanded_berths[i], last_log_.berths[i])) {
                changed = true;
                break;
            }
        }
    }
    if (!changed)
        return;

    size_t realtime_count = 0;
    size_t map_count = 0;
    for (const usv::Berth &berth : res.expanded_berths) {
        if (berth.source == usv::BerthDetectionSource::Map) {
            ++map_count;
        } else {
            ++realtime_count;
        }
    }
    postLog(QStringLiteral("[地图实时融合] 实时=%1 地图补充=%2 最终输出=%3")
                .arg(realtime_count)
                .arg(map_count)
                .arg(res.expanded_berths.size()));
    for (size_t i = 0; i < res.expanded_berths.size(); ++i) {
        postLog(formatProtocolPreview(res.expanded_berths[i], i));
    }

    last_log_.initialized = true;
    last_log_.detected = true;
    last_log_.berths = res.expanded_berths;
}

void BerthDetectionNode::Start()
{
    bool expected = false;
    if (!is_running_.compare_exchange_strong(expected, true))
        return;

    last_log_ = {};
    sync_queue_.reopen();
    realtime_received_frames_.store(0, std::memory_order_relaxed);
    realtime_processed_frames_.store(0, std::memory_order_relaxed);
    last_logged_realtime_received_ = 0;
    last_logged_realtime_processed_ = 0;
    last_logged_realtime_dropped_ = 0;
    last_realtime_performance_log_ = std::chrono::steady_clock::now();
    resetRealtimeCoordinateConfirmations();
    consumer_ = std::thread(&BerthDetectionNode::processLoop, this);
}

void BerthDetectionNode::stop()
{
    bool expected = true;
    if (!is_running_.compare_exchange_strong(expected, false))
        return;

    sync_queue_.shutdown();
    if (consumer_.joinable())
        consumer_.join();
}

void BerthDetectionNode::markOccupiedBerths(
    const usv::LidarFrame &frame, usv::BerthMeasureResult &result) const
{
    for (size_t berthIndex = 0; berthIndex < result.expanded_berths.size(); ++berthIndex) {
        usv::Berth &berth = result.expanded_berths[berthIndex];
        berth.has_ship = false;
        berth.interior_point_count = 0;

        const double halfW = berth.w * 0.5 - occupied_inset_margin_m_;
        const double halfL = berth.l * 0.5 - occupied_inset_margin_m_;
        if (!std::isfinite(berth.cx) || !std::isfinite(berth.cy)
            || !std::isfinite(berth.angle) || halfW <= 0.0 || halfL <= 0.0) {
            continue;
        }

        const double rad = berth.angle * M_PI / 180.0;
        const double cosA = std::cos(rad);
        const double sinA = std::sin(rad);
        for (const usv::LidarPoint &point : frame.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                continue;
            }
            const double dx = static_cast<double>(point.x) - berth.cx;
            const double dy = static_cast<double>(point.y) - berth.cy;
            const double localX = cosA * dx + sinA * dy;
            const double localY = -sinA * dx + cosA * dy;
            if (std::abs(localX) <= halfW && std::abs(localY) <= halfL) {
                ++berth.interior_point_count;
            }
        }
        berth.has_ship = berth.interior_point_count >= occupied_min_points_;
        result.debug_messages.push_back(
            "[BerthOccupancy] berth=" + std::to_string(berthIndex + 1)
            + " inside_points=" + std::to_string(berth.interior_point_count)
            + " threshold=" + std::to_string(occupied_min_points_)
            + " occupied=" + (berth.has_ship ? "1" : "0"));
    }
}

void BerthDetectionNode::processLoop()
{
    while (is_running_) {
        usv::SyncedPackagePtr task;
        if (!sync_queue_.pop(task) || !task || !task->lidar)
            continue;

        // 检测耗时 > 帧间隔时，丢弃排队中的旧帧，只算最新融合点云
        usv::SyncedPackagePtr newer;
        while (sync_queue_.try_pop(newer)) {
            if (newer && newer->lidar)
                task = std::move(newer);
        }

        usv::BerthMeasureResult res;
        res.timestamp = task->lidar->timestamp;
        const auto realtime_detection_start = std::chrono::steady_clock::now();

        Eigen::Isometry3d navigation_pose = Eigen::Isometry3d::Identity();
        double navigation_pose_dt = std::numeric_limits<double>::infinity();
        const bool has_navigation_pose =
            lookupNavigationPose(res.timestamp, navigation_pose, &navigation_pose_dt);
        const bool has_world_pose = has_navigation_pose;
        const Eigen::Isometry3d &map_pose = navigation_pose;

        // Berth world state uses GNSS ENU exclusively. A confirmed ENU bus is
        // projected into the current lidar frame for search and correction.
        usv::DebugLine2D active_world_bus;
        const bool has_active_bus = has_world_pose
            && getWorldAnchorBus(active_world_bus);
        if (has_active_bus) {
            u_shape_algo_.setWorldAnchorBusConstraint(transformDebugLine(
                active_world_bus, map_pose.inverse()));
        } else {
            u_shape_algo_.clearWorldAnchorBusConstraint();
        }

        if (has_world_pose) {
            std::vector<usv::Berth> projected_recovery_rois;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (!navigation_world_recovery_rois_.empty()) {
                    projected_recovery_rois.reserve(navigation_world_recovery_rois_.size());
                    const Eigen::Isometry3d world_to_lidar = navigation_pose.inverse();
                    for (const usv::Berth& world_roi : navigation_world_recovery_rois_) {
                        projected_recovery_rois.push_back(transformBerth(
                            world_roi, world_to_lidar,
                            usv::BerthDetectionSource::Realtime));
                    }
                }
            }
            if (!projected_recovery_rois.empty()) {
                u_shape_algo_.setWorldProjectedRecoveryRois(projected_recovery_rois);
                res.debug_messages.push_back(
                    "[BerthDebug] projected world recovery ROI to lidar count="
                    + std::to_string(projected_recovery_rois.size()));
            }
        }

        if (has_world_pose) {
            std::vector<usv::Berth> stable_local_berths;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                const std::vector<WorldBerthStabilityState> &states =
                    realtime_navigation_stability_states_;
                stable_local_berths.reserve(states.size());
                const Eigen::Isometry3d world_to_lidar = map_pose.inverse();
                for (const WorldBerthStabilityState &state : states) {
                    if (!state.has_stable_geometry
                        || state.last_accepted_berth.kind != usv::BerthKind::UShape) {
                        continue;
                    }
                    stable_local_berths.push_back(transformBerth(
                        state.last_accepted_berth, world_to_lidar,
                        usv::BerthDetectionSource::Map));
                }
            }
            u_shape_algo_.setStableHistoricalBerths(stable_local_berths);
        } else {
            u_shape_algo_.clearStableHistoricalBerths();
        }

        const bool run_u_shape = u_shape_enabled_.load(std::memory_order_relaxed);
        const bool run_line = line_enabled_.load(std::memory_order_relaxed);
        if (run_u_shape) {
            usv::BerthMeasureResult u_res = u_shape_algo_.process(*task->lidar);
            markBerthsAsUShape(u_res);
            // 先保留检测器的全部候选，确保实时泊位和 0xFB 投递不因
            // 额外的交叉仲裁被误删；接收端稳定后再恢复该优化。
            if (!u_res.expanded_berths.empty()) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                navigation_world_recovery_rois_.clear();
                if (has_navigation_pose) {
                    navigation_world_recovery_rois_.reserve(u_res.expanded_berths.size());
                    for (const usv::Berth& local_roi : u_res.expanded_berths) {
                        navigation_world_recovery_rois_.push_back(transformBerth(
                            local_roi, navigation_pose, usv::BerthDetectionSource::Map));
                    }
                }
                u_res.debug_messages.push_back(
                    "[BerthDebug] updated recovery ROI in world coordinates count="
                    + std::to_string(u_res.expanded_berths.size()));
            } else if (!u_shape_algo_.hasRecoveryRoiMemory()) {
                std::lock_guard<std::mutex> lock(map_mutex_);
                navigation_world_recovery_rois_.clear();
                u_res.debug_messages.push_back(
                    "[BerthDebug] cleared world recovery ROI with detector memory after 10 no-output frames");
            }
            appendBerthResult(res, u_res);
        }
        if (run_line) {
            const usv::LineBerthMeasureResult line_raw = line_algo_.process(*task->lidar);
            appendBerthResult(res, convertLineResult(line_raw));
        }
        if (!run_u_shape && !run_line) {
            res.debug_messages.push_back("[泊位检测] U型泊位和一字泊位均未启用，本帧跳过泊位检测");
        }

        for (usv::Berth &berth : res.expanded_berths) {
            berth.source = usv::BerthDetectionSource::Realtime;
        }
        // Preserve the detector's direct output before world stability,
        // confirmation and coordinate-library replacement modify `res`.
        const usv::BerthMeasureResult realtime_debug_result = res;
        size_t realtime_output_count = res.expanded_berths.size();

        if (has_world_pose) {
            const Eigen::Isometry3d map_to_lidar = map_pose.inverse();
            const usv::DebugLine2D world_anchor_bus = res.has_anchor_bus
                ? transformDebugLine(res.anchor_bus_line, map_pose)
                : usv::DebugLine2D{};
            updateWorldAnchorBus(world_anchor_bus, res.debug_messages);
            usv::DebugLine2D confirmed_world_anchor_bus;
            if (getWorldAnchorBus(confirmed_world_anchor_bus)) {
                // UI/debug output follows the confirmed world bus, not the raw
                // frame-local anchor pair that is still allowed to jitter.
                res.has_anchor_bus = true;
                res.anchor_bus_line = transformDebugLine(
                    confirmed_world_anchor_bus, map_to_lidar);
            }
            alignCoordinateLibrariesToWorldAnchorBus(res.debug_messages);
            std::vector<usv::Berth> realtime_world_berths;
            realtime_world_berths.reserve(realtime_output_count);
            for (size_t i = 0; i < realtime_output_count; ++i) {
                realtime_world_berths.push_back(transformBerth(
                    res.expanded_berths[i], map_pose, usv::BerthDetectionSource::Map));
            }
            // The U-shape berths adjacent to the newly confirmed bus define the
            // valid berth-row side.  A library berth corrected onto the opposite
            // side is a historical false association and must not be replayed.
            updateWorldAnchorBusBerthSide(
                realtime_world_berths, res.debug_messages);
            const std::vector<usv::Berth> raw_realtime_world_berths = realtime_world_berths;

            const std::vector<bool> stability_accepted = stabilizeRealtimeWorldBerths(
                realtime_world_berths, res.debug_messages);
            std::vector<usv::Berth> accepted_world_berths;
            std::vector<usv::Berth> accepted_local_berths;
            accepted_world_berths.reserve(realtime_world_berths.size());
            accepted_local_berths.reserve(realtime_world_berths.size());
            for (size_t i = 0; i < realtime_world_berths.size(); ++i) {
                if (i >= stability_accepted.size() || !stability_accepted[i]) {
                    continue;
                }
                accepted_world_berths.push_back(realtime_world_berths[i]);
                accepted_local_berths.push_back(transformBerth(
                    realtime_world_berths[i], map_to_lidar,
                    usv::BerthDetectionSource::Realtime));
            }
            res.expanded_berths = std::move(accepted_local_berths);
            realtime_world_berths = std::move(accepted_world_berths);
            realtime_output_count = res.expanded_berths.size();

            // Save accepted real-time world measurements before taking the output
            // snapshot.  The anchor-bus correction below therefore affects this
            // frame's library projection as well as future blind-zone replay.
            const std::vector<bool> output_confirmed =
                confirmAndUpdateRealtimeCoordinateLibrary(
                    realtime_world_berths,
                    res.debug_messages);
            std::vector<usv::Berth> confirmed_world_berths;
            std::vector<usv::Berth> confirmed_local_berths;
            confirmed_world_berths.reserve(realtime_world_berths.size());
            confirmed_local_berths.reserve(realtime_world_berths.size());
            for (size_t i = 0; i < realtime_world_berths.size(); ++i) {
                if (i >= output_confirmed.size() || !output_confirmed[i]) {
                    continue;
                }
                confirmed_world_berths.push_back(realtime_world_berths[i]);
                confirmed_local_berths.push_back(transformBerth(
                    realtime_world_berths[i], map_to_lidar,
                    usv::BerthDetectionSource::Realtime));
            }
            realtime_world_berths = std::move(confirmed_world_berths);
            res.expanded_berths = std::move(confirmed_local_berths);
            realtime_output_count = res.expanded_berths.size();
            alignCoordinateLibrariesToWorldAnchorBus(res.debug_messages);

            std::vector<usv::Berth> coordinate_library;
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                res.expanded_berths.clear();
                res.expanded_berths.reserve(realtime_world_berths.size());
                for (const usv::Berth &berth : realtime_world_berths) {
                    res.expanded_berths.push_back(transformBerth(
                        berth, map_to_lidar, usv::BerthDetectionSource::Realtime));
                }
                realtime_output_count = res.expanded_berths.size();
                realtime_coordinate_berths_ = res.expanded_berths;

                const std::vector<RealtimeBerthConfirmation> &confirmations =
                    realtime_navigation_confirmations_;
                auto is_output_confirmed = [&](const usv::Berth &berth) {
                    return std::any_of(
                        confirmations.begin(), confirmations.end(),
                        [&](const RealtimeBerthConfirmation &confirmation) {
                            return confirmation.hit_count >= realtime_output_confirmation_frames_
                                && sameBerthCandidate(confirmation.berth, berth);
                        });
                };

                for (const usv::Berth &berth : realtime_navigation_coordinate_berths_) {
                    if (is_output_confirmed(berth)) {
                        coordinate_library.push_back(berth);
                    }
                }
            }
            flipLibraryBerthsOppositeWorldAnchorBus(
                coordinate_library, res.debug_messages);

            // A line berth's first stored length is the reference length.  When the
            // real-time line becomes much shorter in a point-cloud blind zone, show
            // the stored geometry transformed back into the current lidar frame.
            for (size_t i = 0; i < realtime_output_count; ++i) {
                const usv::Berth &realtime_berth = res.expanded_berths[i];
                if (realtime_berth.kind != usv::BerthKind::Line) {
                    continue;
                }
                const auto stored_it = std::find_if(
                    coordinate_library.begin(), coordinate_library.end(),
                    [&realtime_world_berths, i](const usv::Berth &stored) {
                        return sameBerthCandidate(stored, realtime_world_berths[i]);
                    });
                if (stored_it == coordinate_library.end() || !(stored_it->l > 1e-6)
                    || realtime_berth.l >= kLineBerthShortLengthRatio * stored_it->l) {
                    continue;
                }

                const double realtime_length = realtime_berth.l;
                res.expanded_berths[i] = transformBerth(
                    *stored_it, map_to_lidar, usv::BerthDetectionSource::Map);
                res.debug_messages.push_back(
                    "[一字泊位] 实时长度=" + std::to_string(realtime_length)
                    + "m 小于库长度=" + std::to_string(stored_it->l)
                    + "m 的2/3，使用坐标库长度回显");
            }
            for (const usv::Berth &stored_berth : coordinate_library) {
                const usv::Berth local_berth = transformBerth(
                    stored_berth, map_to_lidar, usv::BerthDetectionSource::Map);
                const bool duplicated = std::any_of(res.expanded_berths.begin(), res.expanded_berths.end(),
                    [&local_berth](const usv::Berth &existing) {
                        return sameBerthCandidate(existing, local_berth);
                    });
                if (!duplicated) {
                    res.expanded_berths.push_back(local_berth);
                }
            }
            res.is_detected = !res.expanded_berths.empty();

        } else {
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                realtime_coordinate_berths_ = res.expanded_berths;
            }
            if (res.timestamp - last_map_pose_skip_log_timestamp_ >= 2.0) {
                res.debug_messages.push_back(
                    "[泊位历史回显] 跳过：无匹配的GNSS/INS ENU位姿 navigation_dt="
                    + std::to_string(navigation_pose_dt) + "s");
                last_map_pose_skip_log_timestamp_ = res.timestamp;
            }
        }

        // 世界稳定器需要多帧确认，确认窗口尚未完成时不能把本帧真实
        // 检测结果清空：否则界面看不到泊位，0xFB 也没有任何内容可发。
        // 稳定库仍继续维护；这里只是把本帧原始实时结果作为显示/投递
        // 兜底，下一帧确认成功后仍使用稳定后的结果。
        if (has_world_pose && res.expanded_berths.empty()
            && realtime_debug_result.is_detected
            && !realtime_debug_result.expanded_berths.empty()) {
            res.expanded_berths = realtime_debug_result.expanded_berths;
            res.is_detected = true;
            res.is_using_memory = false;
            res.is_line_recovered = realtime_debug_result.is_line_recovered;
            res.debug_messages.push_back(
                "[泊位检测] 世界稳定确认未完成，本帧保留原始实时泊位用于显示和0xFB投递");
        }


        // Evaluate occupancy after all stability and coordinate-library replacement.
        markOccupiedBerths(*task->lidar, res);

        const QString anchor_height_log = formatAnchorHeights(res);
        if (!anchor_height_log.isEmpty()) {
            postLog(anchor_height_log);
        }

        const double realtime_elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - realtime_detection_start).count();
        postLog(QStringLiteral("[泊位检测耗时] ts=%1 输入点=%2 U型=%3 一字=%4 "
                               "实时输出=%5 最终输出=%6 总耗时=%7ms")
                    .arg(res.timestamp, 0, 'f', 3)
                    .arg(task->lidar->point_count)
                    .arg(run_u_shape ? QStringLiteral("开") : QStringLiteral("关"))
                    .arg(run_line ? QStringLiteral("开") : QStringLiteral("关"))
                    .arg(realtime_output_count)
                    .arg(res.expanded_berths.size())
                    .arg(realtime_elapsed_ms, 0, 'f', 1));
        const uint64_t processed_count =
            realtime_processed_frames_.fetch_add(1, std::memory_order_relaxed) + 1;
        const uint64_t received_count = realtime_received_frames_.load(std::memory_order_relaxed);
        const uint64_t dropped_count = static_cast<uint64_t>(sync_queue_.droppedCount());
        const auto now = std::chrono::steady_clock::now();
        const bool should_log_performance =
            now - last_realtime_performance_log_ >= std::chrono::seconds(1)
            || dropped_count > last_logged_realtime_dropped_;
        if (should_log_performance) {
            const uint64_t received_delta = received_count - last_logged_realtime_received_;
            const uint64_t processed_delta = processed_count - last_logged_realtime_processed_;
            const uint64_t dropped_delta = dropped_count - last_logged_realtime_dropped_;
            const QString state = dropped_delta > 0
                ? QStringLiteral("队列积压，已主动丢旧帧")
                : (realtime_elapsed_ms > 200.0
                    ? QStringLiteral("处理偏慢")
                    : QStringLiteral("正常"));
            postLog(QStringLiteral("[实时泊位性能] 本周期接收=%1 已处理=%2 队列丢帧=%3 "
                                   "累计丢帧=%4 输入点=%5 本帧耗时=%6ms 实时输出=%7 状态=%8")
                        .arg(received_delta)
                        .arg(processed_delta)
                        .arg(dropped_delta)
                        .arg(dropped_count)
                        .arg(task->lidar->point_count)
                        .arg(realtime_elapsed_ms, 0, 'f', 1)
                        .arg(realtime_output_count)
                        .arg(state));
            last_logged_realtime_received_ = received_count;
            last_logged_realtime_processed_ = processed_count;
            last_logged_realtime_dropped_ = dropped_count;
            last_realtime_performance_log_ = now;
        }

        pub_result_->publish(std::make_shared<usv::BerthMeasureResult>(res));
        comm_->updateStateRepository<usv::BerthMeasureResult>(
            std::make_shared<usv::BerthMeasureResult>(res));

        maybeLogDetectionResult(res);
        postBerthResult(res);
    }
}

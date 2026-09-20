#pragma once

#include "common_types_extended.h"

#include <array>
#include <cmath>

#include <Eigen/Geometry>

namespace usv::berth_geometry {

inline double normalizeAngle180(double angle)
{
    angle = std::fmod(angle, 180.0);
    return angle < 0.0 ? angle + 180.0 : angle;
}

inline double poseYawDeg(const Eigen::Isometry3d &pose)
{
    return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0))
        * 180.0 / 3.14159265358979323846;
}

inline DebugLine2D transformLine(const DebugLine2D &line,
                                 const Eigen::Isometry3d &transform)
{
    const Eigen::Vector3d p1 =
        transform * Eigen::Vector3d(line.x1, line.y1, 0.0);
    const Eigen::Vector3d p2 =
        transform * Eigen::Vector3d(line.x2, line.y2, 0.0);
    return DebugLine2D{p1.x(), p1.y(), p2.x(), p2.y()};
}

template <typename T>
inline void remapOppositeEdges(std::array<T, 4> &values)
{
    const std::array<T, 4> original = values;
    for (int edge = 0; edge < 4; ++edge)
        values[(edge + 2) % 4] = original[edge];
}

inline Berth transformBerth(const Berth &input,
                            const Eigen::Isometry3d &transform,
                            BerthDetectionSource source)
{
    Berth result = input;
    const Eigen::Vector3d center =
        transform * Eigen::Vector3d(input.cx, input.cy, 0.0);
    const double transformedAngle = input.angle + poseYawDeg(transform);
    result.cx = center.x();
    result.cy = center.y();
    result.angle = normalizeAngle180(transformedAngle);
    result.source = source;

    // Modulo-180 normalization can reverse the local axes. Keep every edge
    // attribute attached to the same physical edge after that reversal.
    const double angleDeltaRad =
        (transformedAngle - result.angle)
        * 3.14159265358979323846 / 180.0;
    if (std::cos(angleDeltaRad) < 0.0) {
        if (result.opening_edge >= 0 && result.opening_edge < 4)
            result.opening_edge = (result.opening_edge + 2) % 4;
        remapOppositeEdges(result.edge_point_counts);
        remapOppositeEdges(result.edge_heights);
        remapOppositeEdges(result.edge_height_counts);
        remapOppositeEdges(result.edge_height_valid);
        remapOppositeEdges(result.ship_edge_distances);
        remapOppositeEdges(result.ship_edge_distance_valid);
        remapOppositeEdges(result.ship_edge_distance_lines);
    }

    if (result.has_opening_segment)
        result.opening_segment = transformLine(input.opening_segment, transform);
    for (DebugLine2D &edge : result.visible_edges)
        edge = transformLine(edge, transform);
    for (DebugLine2D &line : result.ship_edge_distance_lines)
        line = transformLine(line, transform);
    return result;
}

inline Berth transformBerth(const Berth &input,
                            const Eigen::Isometry3d &transform)
{
    return transformBerth(input, transform, input.source);
}

inline BerthMeasureResult transformResult(
    const BerthMeasureResult &input,
    const Eigen::Isometry3d &transform)
{
    BerthMeasureResult result = input;

    for (Berth &berth : result.expanded_berths)
        berth = transformBerth(berth, transform);

    if (result.has_debug_roi)
        result.debug_roi = transformBerth(result.debug_roi, transform);
    if (result.has_top_search_roi)
        result.top_search_roi = transformBerth(result.top_search_roi, transform);
    for (Berth &berth : result.debug_rois)
        berth = transformBerth(berth, transform);
    for (Berth &berth : result.top_search_rois)
        berth = transformBerth(berth, transform);

    if (result.has_anchor_bus)
        result.anchor_bus_line = transformLine(result.anchor_bus_line, transform);
    for (DebugLine2D &line : result.debug_lines)
        line = transformLine(line, transform);

    if (result.has_highest_point)
        result.highest_point = transform * result.highest_point;
    if (result.has_second_highest_point)
        result.second_highest_point = transform * result.second_highest_point;
    for (Eigen::Vector3d &point : result.highest_points)
        point = transform * point;
    for (Eigen::Vector3d &point : result.second_highest_points)
        point = transform * point;

    return result;
}

} // namespace usv::berth_geometry


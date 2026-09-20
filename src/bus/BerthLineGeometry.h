#pragma once

#include "BerthProtocolGeometry.h"
#include "../berth/LineBerthDetection.h"

#include <cmath>

namespace usv::berth_geometry {

/**
 * Convert the line detector result to the common berth representation used by
 * the UI and 0xFB exporter.  The line detector reports the wall edge; the
 * protocol needs an explicit opening edge and the associated four corners.
 */
inline usv::DebugLine2D edgeSegment(const berth_protocol::BerthCorners &corners,
                                    int edge)
{
    const int normalized = ((edge % 4) + 4) % 4;
    return usv::DebugLine2D{
        corners[normalized].x(), corners[normalized].y(),
        corners[(normalized + 1) % 4].x(),
        corners[(normalized + 1) % 4].y()};
}

inline usv::Berth fromLineBerth(const usv::LineBerth &line)
{
    usv::Berth berth;
    berth.cx = line.cx;
    berth.cy = line.cy;
    berth.w = line.width;
    berth.l = line.length;
    // LineBerth::angle is the wall tangent.  The common rectangle angle is
    // the width (normal) axis, hence the -90 degree conversion.
    berth.angle = std::fmod(line.angle - 90.0, 180.0);
    if (berth.angle < 0.0)
        berth.angle += 180.0;
    berth.kind = usv::BerthKind::Line;
    berth.source = usv::BerthDetectionSource::Realtime;

    const berth_protocol::BerthCorners corners =
        berth_protocol::cornersCounterClockwise(berth);
    berth.opening_edge = berth_protocol::openingEdge(berth, corners);
    berth.has_opening_segment = true;
    berth.opening_segment = edgeSegment(corners, berth.opening_edge);
    berth.visible_edges.reserve(3);
    for (int edge = 0; edge < 4; ++edge) {
        if (edge != berth.opening_edge)
            berth.visible_edges.push_back(edgeSegment(corners, edge));
    }

    // The detector's support points belong to the physical wall, opposite the
    // opening.  Keeping them on that edge makes deck-height diagnostics and
    // protocol encoding agree with the geometry.
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

}  // namespace usv::berth_geometry

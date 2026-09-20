#include "BerthLineGeometry.h"

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void testLineBerthCarriesProtocolOpeningMetadata()
{
    usv::LineBerth line;
    line.cx = 0.0;
    line.cy = 10.0;
    line.width = 4.0;
    line.length = 12.0;
    line.angle = 0.0;
    line.edge.x1 = -6.0;
    line.edge.y1 = 10.0;
    line.edge.x2 = 6.0;
    line.edge.y2 = 10.0;
    line.edge.support_points = 123;
    line.edge.support_z_mean = 1.7;

    const usv::Berth berth = usv::berth_geometry::fromLineBerth(line);
    check(berth.opening_edge >= 0 && berth.opening_edge < 4,
          "line berth must expose a concrete protocol opening edge");
    check(berth.has_opening_segment,
          "line berth must expose the opening segment for UI/protocol diagnostics");
    check(berth.visible_edges.size() == 3,
          "line berth must expose the three non-opening edges");

    const int wall_edge = (berth.opening_edge + 2) % 4;
    check(berth.edge_point_counts[wall_edge] == 123,
          "line support points must be attached to the wall edge, not edge zero");
    check(berth.edge_height_valid[wall_edge]
              && std::abs(berth.edge_heights[wall_edge] - 1.7) < 1e-9,
          "line support height must be attached to the wall edge");
}

}  // namespace

int main()
{
    testLineBerthCarriesProtocolOpeningMetadata();
    return failures == 0 ? 0 : 1;
}

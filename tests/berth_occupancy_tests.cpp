#include "BerthOccupancy.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

usv::Berth berth()
{
    usv::Berth result;
    result.cx = 0.0;
    result.cy = 0.0;
    result.w = 4.0;
    result.l = 8.0;
    result.angle = 0.0;
    result.kind = usv::BerthKind::UShape;
    return result;
}

void testCountsPointsInsideRotatedInset(int &failures)
{
    usv::LidarFrame frame;
    frame.points = {
        {0.0f, 0.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 1.0f, 0.0f},
        {2.0f, 3.0f, 1.0f, 0.0f}, // outside the 0.8 m inset
        {20.0f, 20.0f, 1.0f, 0.0f},
    };

    usv::Berth result = berth();
    usv::berth_occupancy::annotate(frame, result);

    check(result.interior_point_count == 2,
          "occupancy count must use the rotated berth inset", failures);
    check(!result.has_ship,
          "a berth below the default occupancy threshold must be free", failures);
}

void testMarksOccupiedBerthAtThreshold(int &failures)
{
    usv::LidarFrame frame;
    frame.points.reserve(usv::berth_occupancy::kDefaultOccupiedMinPoints);
    for (int i = 0; i < usv::berth_occupancy::kDefaultOccupiedMinPoints; ++i) {
        frame.points.push_back({0.1f, 0.1f, 1.0f, 0.0f});
    }

    usv::Berth result = berth();
    usv::berth_occupancy::annotate(frame, result);

    check(result.interior_point_count == usv::berth_occupancy::kDefaultOccupiedMinPoints,
          "all points inside the berth must be counted", failures);
    check(result.has_ship,
          "the default threshold must mark the berth occupied", failures);
}

} // namespace

int main()
{
    int failures = 0;
    testCountsPointsInsideRotatedInset(failures);
    testMarksOccupiedBerthAtThreshold(failures);
    if (failures != 0)
        return 1;
    std::cout << "All berth occupancy tests passed\n";
    return 0;
}

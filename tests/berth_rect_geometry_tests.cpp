#include "BerthRectGeometry.h"

#include <cmath>
#include <iostream>

namespace {

bool nearlyEqual(double a, double b)
{
    return std::abs(a - b) < 1.0e-9;
}

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

}  // namespace

int main()
{
    int failures = 0;
    // Horizontal long rectangle: OpenCV reports (short,long), angle=90;
    // the short/width axis is vertical (90), not horizontal (0).
    check(nearlyEqual(
              usv::berth_geometry::commonWidthAxisAngle(2.0, 10.0, 90.0),
              90.0),
          "short,long rectangle must keep the reported short-axis angle",
          failures);
    // Vertical long rectangle: OpenCV reports (long,short), angle=90;
    // the common short/width axis is horizontal (0).
    check(nearlyEqual(
              usv::berth_geometry::commonWidthAxisAngle(10.0, 2.0, 90.0),
              0.0),
          "long,short rectangle must rotate to the short-axis angle",
          failures);
    check(nearlyEqual(
              usv::berth_geometry::commonWidthAxisAngle(10.0, 2.0, -30.0),
              60.0),
          "negative OpenCV angles must normalize to [0,180)",
          failures);

    if (failures != 0)
        return 1;
    std::cout << "Berth rectangle geometry tests passed\n";
    return 0;
}

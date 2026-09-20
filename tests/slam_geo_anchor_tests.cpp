#include "SlamGeoAnchorPolicy.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool close(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-9;
}

usv::GnssInsMessage validGnss(double latitude, double longitude,
                              double altitude, double timestamp)
{
    usv::GnssInsMessage gnss;
    gnss.timestamp = timestamp;
    gnss.latitude = latitude;
    gnss.longitude = longitude;
    gnss.altitude = altitude;
    gnss.yaw = 15.0;
    gnss.pitch = 1.0;
    gnss.roll = -2.0;
    gnss.yaw_valid = true;
    gnss.pitch_valid = true;
    gnss.roll_valid = true;
    return gnss;
}

void testFirstValidGnssWins(int &failures)
{
    usv::SlamGeoAnchor anchor;

    usv::GnssInsMessage zero = validGnss(0.0, 0.0, 0.0, 1.0);
    check(!usv::captureFirstSlamGeoAnchor(zero, anchor) && !anchor.valid,
          "zero latitude and longitude must not create an anchor", failures);

    usv::GnssInsMessage invalid = validGnss(31.0, 121.0, 0.0, 2.0);
    invalid.yaw_valid = false;
    check(!usv::captureFirstSlamGeoAnchor(invalid, anchor) && !anchor.valid,
          "GNSS/INS rejected by SLAM must not create an anchor", failures);

    const usv::GnssInsMessage first =
        validGnss(31.123456, 121.654321, 8.5, 10.0);
    check(usv::captureFirstSlamGeoAnchor(first, anchor),
          "first valid GNSS/INS should create the anchor", failures);
    check(anchor.valid
              && close(anchor.timestamp, 10.0)
              && close(anchor.latitude_deg, 31.123456)
              && close(anchor.longitude_deg, 121.654321)
              && close(anchor.altitude_m, 8.5),
          "anchor should copy the GNSS data supplied to SLAM", failures);

    const usv::GnssInsMessage later =
        validGnss(32.0, 122.0, 9.5, 11.0);
    check(!usv::captureFirstSlamGeoAnchor(later, anchor),
          "later GNSS/INS must not replace the established anchor", failures);
    check(close(anchor.latitude_deg, 31.123456)
              && close(anchor.longitude_deg, 121.654321)
              && close(anchor.timestamp, 10.0),
          "established anchor must remain unchanged", failures);
}

void testNonFiniteGnssRejected(int &failures)
{
    usv::SlamGeoAnchor anchor;
    usv::GnssInsMessage gnss = validGnss(31.0, 121.0, 0.0, 3.0);
    gnss.latitude = std::numeric_limits<double>::quiet_NaN();
    check(!usv::captureFirstSlamGeoAnchor(gnss, anchor) && !anchor.valid,
          "non-finite GNSS values must not create an anchor", failures);
}

}  // namespace

int main()
{
    int failures = 0;
    testFirstValidGnssWins(failures);
    testNonFiniteGnssRejected(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM geographic anchor tests passed\n";
    return 0;
}

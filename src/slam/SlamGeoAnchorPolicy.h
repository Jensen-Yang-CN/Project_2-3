#pragma once

#include "common/slam_types.h"
#include "SlamPoseUtils.h"

namespace usv {

/**
 * 记录真正送入 SLAM 的首个有效 GNSS/INS，保证持久化锚点与 ENU 原点一致。
 * 返回 true 表示本次调用建立了锚点。
 */
inline bool captureFirstSlamGeoAnchor(const GnssInsMessage &gnss,
                                      SlamGeoAnchor &anchor)
{
    if (anchor.valid || !slam_pose::isGnssInsUsable(gnss))
        return false;

    anchor.valid = true;
    anchor.timestamp = gnss.timestamp;
    anchor.latitude_deg = gnss.latitude;
    anchor.longitude_deg = gnss.longitude;
    anchor.altitude_m = gnss.altitude;
    return true;
}

}  // namespace usv

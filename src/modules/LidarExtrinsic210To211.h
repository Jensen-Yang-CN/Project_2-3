#pragma once

#include "CalibrationConfig.h"

#include <QMatrix4x4>

#include <Eigen/Dense>

namespace lidar_extrinsic {

inline QMatrix4x4 matrix210To211Q()
{
    return CalibrationConfig::instance().matrix210To211Q();
}

inline QMatrix4x4 matrix211To210Q()
{
    return CalibrationConfig::instance().matrix211To210Q();
}

inline Eigen::Matrix4f matrix210To211Eigen4f()
{
    return CalibrationConfig::instance().matrix210To211Eigen4f();
}

}  // namespace lidar_extrinsic

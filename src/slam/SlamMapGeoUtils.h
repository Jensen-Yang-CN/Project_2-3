#ifndef SLAMMAPGEOUTILS_H
#define SLAMMAPGEOUTILS_H

#include "common/slam_types.h"

#include <QString>

#include <Eigen/Core>

namespace slam_map_geo {

bool validAnchor(const usv::SlamGeoAnchor &anchor);

bool anchorOffsetEnu(const usv::SlamGeoAnchor &base,
                     const usv::SlamGeoAnchor &source,
                     Eigen::Vector3d &offset,
                     QString *error = nullptr);

}  // namespace slam_map_geo

#endif  // SLAMMAPGEOUTILS_H

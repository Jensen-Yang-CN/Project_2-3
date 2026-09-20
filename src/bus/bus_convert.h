#pragma once

#include "common_types_extended.h"
#include "common/pointxyz.h"

#include <QImage>
#include <QVector>
#include <memory>

namespace bus_convert {

std::shared_ptr<usv::LidarFrame> lidarFromQtCloud(const QVector<M_PointXYZI> &cloud,
                                                  double timestamp,
                                                  int maxPoints = 80000,
                                                  quint64 timelineGeneration = 1); // maxPoints<=0 表示不抽稀

std::shared_ptr<usv::ImageFrame> imageFromQt(const QImage &img, double timestamp,
                                             quint64 timelineGeneration = 1);

} // namespace bus_convert

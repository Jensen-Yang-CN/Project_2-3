#include "bus_convert.h"

#include <opencv2/core.hpp>

namespace bus_convert {

std::shared_ptr<usv::LidarFrame> lidarFromQtCloud(const QVector<M_PointXYZI> &cloud,
                                                  double timestamp,
                                                  int maxPoints,
                                                  quint64 timelineGeneration)
{
    auto frame = std::make_shared<usv::LidarFrame>();
    frame->timestamp = timestamp;
    frame->timeline_generation = timelineGeneration;
    const int total = cloud.size();
    if (total <= 0) {
        frame->point_count = 0;
        return frame;
    }

    const int step = (maxPoints > 0 && total > maxPoints) ? (total + maxPoints - 1) / maxPoints : 1;
    frame->points.reserve(static_cast<size_t>(total / step) + 1);
    for (int i = 0; i < total; i += step) {
        const M_PointXYZI &p = cloud[i];
        usv::LidarPoint lp;
        lp.x = p.x;
        lp.y = p.y;
        lp.z = p.z;
        lp.intensity = static_cast<float>(p.intensity);
        frame->points.push_back(lp);
    }
    frame->point_count = static_cast<uint32_t>(frame->points.size());
    return frame;
}

std::shared_ptr<usv::ImageFrame> imageFromQt(const QImage &img, double timestamp,
                                             quint64 timelineGeneration)
{
    auto frame = std::make_shared<usv::ImageFrame>();
    frame->timestamp = timestamp;
    frame->timeline_generation = timelineGeneration;
    if (img.isNull())
        return frame;

    QImage rgb = img.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                   const_cast<uchar *>(rgb.bits()),
                   static_cast<size_t>(rgb.bytesPerLine()));
    frame->image.create(rgbMat.rows, rgbMat.cols, CV_8UC3);
    for (int y = 0; y < rgbMat.rows; ++y) {
        const cv::Vec3b *src = rgbMat.ptr<cv::Vec3b>(y);
        cv::Vec3b *dst = frame->image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < rgbMat.cols; ++x) {
            dst[x] = cv::Vec3b(src[x][2], src[x][1], src[x][0]);
        }
    }
    return frame;
}

} // namespace bus_convert

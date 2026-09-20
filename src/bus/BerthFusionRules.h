#pragma once

#include "common_types_extended.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/imgproc.hpp>

/**
 * Frame-local U-shape arbitration used before the world-coordinate stability
 * stage.  The detector can produce two rectangles for the same berth after
 * morphology/ROI expansion; retaining both makes the protocol send duplicate
 * berths and also makes the opening direction appear to jump.  Keep the
 * candidate with the stronger fixed-edge evidence and suppress the weaker one.
 *
 * This header is intentionally self-contained: the rule is pure and does not
 * mutate detector state, so it can be reused by the node and by unit tests.
 */
namespace usv::berth_fusion {

inline double candidateScore(const Berth &berth)
{
    double score = 0.0;
    for (int edge = 0; edge < 4; ++edge) {
        score += std::max(0, berth.edge_point_counts[edge]);
        if (edge != berth.opening_edge && berth.edge_height_valid[edge])
            score += 2.0 * std::max(0, berth.edge_height_counts[edge]);
    }
    // A candidate with a valid physical opening and at least one visible edge
    // is preferable to an incomplete morphology fragment on ties.
    if (berth.opening_edge >= 0 && berth.opening_edge < 4)
        score += 1.0;
    score += std::max(0, berth.interior_point_count) * 0.01;
    return score;
}

inline double rotatedIou(const Berth &a, const Berth &b)
{
    if (a.kind != BerthKind::UShape || b.kind != BerthKind::UShape
        || !(a.w > 1e-6) || !(a.l > 1e-6)
        || !(b.w > 1e-6) || !(b.l > 1e-6)) {
        return 0.0;
    }

    const double areaA = a.w * a.l;
    const double areaB = b.w * b.l;
    if (!std::isfinite(areaA) || !std::isfinite(areaB))
        return 0.0;

    const cv::RotatedRect rectA(
        cv::Point2f(static_cast<float>(a.cx), static_cast<float>(a.cy)),
        cv::Size2f(static_cast<float>(a.w), static_cast<float>(a.l)),
        static_cast<float>(a.angle));
    const cv::RotatedRect rectB(
        cv::Point2f(static_cast<float>(b.cx), static_cast<float>(b.cy)),
        cv::Size2f(static_cast<float>(b.w), static_cast<float>(b.l)),
        static_cast<float>(b.angle));
    std::vector<cv::Point2f> intersection;
    const int status = cv::rotatedRectangleIntersection(rectA, rectB, intersection);
    if (status == cv::INTERSECT_NONE)
        return 0.0;

    const double intersectionArea = status == cv::INTERSECT_FULL
        ? std::min(areaA, areaB)
        : (intersection.size() >= 3
               ? std::abs(cv::contourArea(intersection))
               : 0.0);
    const double unionArea = areaA + areaB - intersectionArea;
    return unionArea > 1e-6 ? intersectionArea / unionArea : 0.0;
}

inline std::vector<std::size_t> suppressLowerConfidenceIntersectingUShapes(
    const std::vector<Berth> &berths,
    double iouThreshold = 0.20)
{
    std::vector<std::size_t> suppressed;
    if (berths.size() < 2)
        return suppressed;

    std::vector<bool> removed(berths.size(), false);
    for (std::size_t i = 0; i < berths.size(); ++i) {
        if (removed[i] || berths[i].kind != BerthKind::UShape)
            continue;
        for (std::size_t j = i + 1; j < berths.size(); ++j) {
            if (removed[j] || berths[j].kind != BerthKind::UShape)
                continue;
            if (rotatedIou(berths[i], berths[j]) < iouThreshold)
                continue;

            const double scoreI = candidateScore(berths[i]);
            const double scoreJ = candidateScore(berths[j]);
            const std::size_t loser = scoreI >= scoreJ ? j : i;
            removed[loser] = true;
            suppressed.push_back(loser);
            if (loser == i)
                break;
        }
    }
    std::sort(suppressed.begin(), suppressed.end());
    return suppressed;
}

}  // namespace usv::berth_fusion


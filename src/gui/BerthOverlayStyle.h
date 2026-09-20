#pragma once

#include "common_types_extended.h"

struct BerthOverlayColor {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
};

enum class BerthOutlineMode {
    Closed,
    OpenEntrance,
};

inline BerthOutlineMode berthOutlineMode(const usv::Berth &berth)
{
    return berth.kind == usv::BerthKind::UShape
               ? BerthOutlineMode::OpenEntrance
               : BerthOutlineMode::Closed;
}

inline BerthOverlayColor berthOverlayColor(usv::BerthDetectionSource source)
{
    if (source == usv::BerthDetectionSource::CoordinateLibrary)
        return {0.10f, 0.45f, 1.00f};  // 盲区回显：蓝色
    return {0.68f, 1.00f, 0.12f};      // 实时检测：黄绿色
}

inline bool berthResultHasOverlay(const usv::BerthMeasureResult &result)
{
    return !result.expanded_berths.empty() ||
           !result.highest_points.empty() ||
           !result.second_highest_points.empty() ||
           result.has_highest_point ||
           result.has_second_highest_point;
}

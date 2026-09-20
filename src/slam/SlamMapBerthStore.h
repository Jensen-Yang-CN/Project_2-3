#pragma once

#include "common/slam_types.h"
#include "common_types_extended.h"
#include "SlamMapBerthGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

/**
 * Lightweight persistent store for berth annotations in the SLAM ENU frame.
 *
 * The detector reports local-frame rectangles.  MultiLidarWidget feeds this
 * class only the synchronized world-coordinate result, so records can be
 * written next to keyframes and rendered without re-running detection when a
 * .slammap is opened again.
 */
class SlamMapBerthStore {
public:
    const std::vector<usv::SlamMapBerth> &records() const
    {
        return records_;
    }

    void clear()
    {
        records_.clear();
        next_id_ = 1;
    }

    void setRecords(const std::vector<usv::SlamMapBerth> &records)
    {
        records_.clear();
        records_.reserve(records.size());
        next_id_ = 1;
        for (const usv::SlamMapBerth &record : records) {
            if (!valid(record))
                continue;
            usv::SlamMapBerth normalized = record;
            int opening_edge = static_cast<int>(normalized.opening_edge);
            normalized.angle_deg =
                slam_map_berth::normalizeAngleAndOpeningEdge(
                    normalized.angle_deg, opening_edge);
            normalized.opening_edge = static_cast<int8_t>(opening_edge);
            records_.push_back(normalized);
            if (normalized.id < std::numeric_limits<uint64_t>::max())
                next_id_ = std::max(next_id_, normalized.id + 1);
        }
    }

    /** Merge one synchronized ENU result into the persistent annotation set. */
    void observe(const usv::BerthMeasureResult &worldResult)
    {
        if (!std::isfinite(worldResult.timestamp))
            return;
        for (const usv::Berth &detected : worldResult.expanded_berths) {
            if (!valid(detected))
                continue;
            usv::Berth berth = detected;
            int opening_edge = berth.opening_edge;
            berth.angle = slam_map_berth::normalizeAngleAndOpeningEdge(
                berth.angle, opening_edge);
            berth.opening_edge = opening_edge;

            usv::SlamMapBerth *match = findMatch(berth);
            if (!match) {
                usv::SlamMapBerth record;
                record.id = allocateId();
                record.timestamp = worldResult.timestamp;
                record.x = berth.cx;
                record.y = berth.cy;
                record.z = 0.0;
                record.width = berth.w;
                record.length = berth.l;
                record.angle_deg = berth.angle;
                record.kind = static_cast<uint8_t>(berth.kind);
                record.opening_edge = static_cast<int8_t>(opening_edge);
                record.observation_count = 1;
                // BerthMeasureResult has no calibrated confidence field.  A
                // record is only inserted after detector acceptance, so 1.0
                // denotes an accepted observation rather than a probability.
                record.confidence = 1.0;
                records_.push_back(record);
                continue;
            }

            const uint32_t oldCount = std::max<uint32_t>(
                1, match->observation_count);
            const uint64_t newCount = static_cast<uint64_t>(oldCount) + 1;
            const double alpha = 1.0 / static_cast<double>(newCount);
            match->x += (berth.cx - match->x) * alpha;
            match->y += (berth.cy - match->y) * alpha;
            match->width += (berth.w - match->width) * alpha;
            match->length += (berth.l - match->length) * alpha;
            const double angleDelta = signedAngleDelta180(
                berth.angle, match->angle_deg);
            match->angle_deg = normalizeAngle180(
                match->angle_deg + angleDelta * alpha);
            match->timestamp = worldResult.timestamp;
            if (berth.opening_edge >= 0 && berth.opening_edge < 4)
                match->opening_edge = static_cast<int8_t>(berth.opening_edge);
            match->observation_count = newCount > std::numeric_limits<uint32_t>::max()
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(newCount);
            match->confidence = 1.0;
        }
    }

private:
    static bool finite(double value)
    {
        return std::isfinite(value);
    }

    static bool valid(const usv::SlamMapBerth &record)
    {
        return finite(record.timestamp) && finite(record.x)
            && finite(record.y) && finite(record.z)
            && finite(record.width) && finite(record.length)
            && finite(record.angle_deg) && finite(record.confidence)
            && record.width > 1.0e-3 && record.length > 1.0e-3
            && record.confidence >= 0.0 && record.confidence <= 1.0
            && record.kind <= static_cast<uint8_t>(usv::BerthKind::Line)
            && record.opening_edge >= -1 && record.opening_edge < 4;
    }

    static bool valid(const usv::Berth &berth)
    {
        return finite(berth.cx) && finite(berth.cy)
            && finite(berth.w) && finite(berth.l)
            && finite(berth.angle) && berth.w > 1.0e-3
            && berth.l > 1.0e-3
            && berth.kind != usv::BerthKind::Unknown;
    }

    static double normalizeAngle180(double angle)
    {
        while (angle >= 90.0)
            angle -= 180.0;
        while (angle < -90.0)
            angle += 180.0;
        return angle;
    }

    static double signedAngleDelta180(double angle, double reference)
    {
        return normalizeAngle180(angle - reference);
    }

    static double sizeTolerance(double size)
    {
        return std::max(2.0, std::abs(size) * 0.35);
    }

    usv::SlamMapBerth *findMatch(const usv::Berth &berth)
    {
        constexpr double kMaxCenterDistanceM = 1.5;
        constexpr double kMaxAngleDifferenceDeg = 20.0;
        const uint8_t kind = static_cast<uint8_t>(berth.kind);
        usv::SlamMapBerth *best = nullptr;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (usv::SlamMapBerth &record : records_) {
            if (record.kind != kind)
                continue;
            if (std::abs(record.width - berth.w) > sizeTolerance(berth.w)
                || std::abs(record.length - berth.l) > sizeTolerance(berth.l)) {
                continue;
            }
            if (std::abs(signedAngleDelta180(
                    berth.angle, record.angle_deg)) > kMaxAngleDifferenceDeg) {
                continue;
            }
            const double distance = std::hypot(
                record.x - berth.cx, record.y - berth.cy);
            if (distance <= kMaxCenterDistanceM && distance < bestDistance) {
                best = &record;
                bestDistance = distance;
            }
        }
        return best;
    }

    uint64_t allocateId()
    {
        const uint64_t result = next_id_;
        if (next_id_ < std::numeric_limits<uint64_t>::max())
            ++next_id_;
        return result;
    }

    std::vector<usv::SlamMapBerth> records_;
    uint64_t next_id_ = 1;
};

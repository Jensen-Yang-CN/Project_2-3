#pragma once

#include "common_types_extended.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace usv::berth_realtime {

/** 世界坐标锚点总线确认、锁定和坐标库几何校正。 */
class RealtimeWorldAnchorBus {
public:
    void update(const DebugLine2D &world_bus)
    {
        if (!validLine(world_bus)) {
            pending_valid_ = false;
            pending_frames_ = 0;
            if (!locked_)
                stable_frames_ = 0;
            return;
        }
        if (locked_)
            return;

        if (valid_ && matches(world_bus, stable_line_)) {
            pending_valid_ = false;
            pending_frames_ = 0;
            if (++stable_frames_ >= lock_frames_)
                locked_ = true;
            return;
        }

        if (valid_)
            stable_frames_ = 0;
        if (pending_valid_ && matches(world_bus, pending_line_)) {
            ++pending_frames_;
        } else {
            pending_line_ = world_bus;
            pending_valid_ = true;
            pending_frames_ = 1;
        }
        if (pending_frames_ < confirm_frames_)
            return;

        stable_line_ = pending_line_;
        valid_ = true;
        pending_valid_ = false;
        pending_frames_ = 0;
        stable_frames_ = confirm_frames_;
        berth_row_side_ = 0;
    }

    bool valid() const { return valid_; }
    bool locked() const { return locked_; }
    int berthRowSide() const { return berth_row_side_; }
    const DebugLine2D &line() const { return stable_line_; }

    void updateBerthRowSide(const std::vector<Berth> &world_berths)
    {
        if (!valid_ || berth_row_side_ != 0)
            return;
        int positive = 0;
        int negative = 0;
        for (const Berth &berth : world_berths) {
            if (berth.kind != BerthKind::UShape
                || minEdgeDistance(berth, stable_line_) > 2.0) {
                continue;
            }
            const double side = signedDistance(
                Eigen::Vector2d(berth.cx, berth.cy), stable_line_);
            if (side > 0.20)
                ++positive;
            if (side < -0.20)
                ++negative;
        }
        if (positive != negative)
            berth_row_side_ = positive > negative ? 1 : -1;
    }

    bool alignBerth(Berth &berth) const
    {
        if (!valid_ || !alignToBus(berth, stable_line_, 5.0))
            return false;
        flipToExpectedSide(berth, stable_line_, berth_row_side_);
        refreshDisplayGeometry(berth);
        return true;
    }

    void alignLibrary(std::vector<Berth> &world_library) const
    {
        if (!valid_)
            return;
        for (Berth &berth : world_library)
            alignBerth(berth);
    }

    void clear() { *this = {}; }

private:
    static constexpr double pi_ = 3.14159265358979323846;
    static constexpr size_t confirm_frames_ = 3;
    static constexpr size_t lock_frames_ = 6;
    static constexpr double angle_gate_deg_ = 8.0;
    static constexpr double offset_gate_m_ = 1.0;

    bool valid_ = false;
    bool locked_ = false;
    DebugLine2D stable_line_{};
    bool pending_valid_ = false;
    DebugLine2D pending_line_{};
    size_t pending_frames_ = 0;
    size_t stable_frames_ = 0;
    int berth_row_side_ = 0;

    static double normalizeAngle(double angle)
    {
        angle = std::fmod(angle, 180.0);
        return angle < 0.0 ? angle + 180.0 : angle;
    }

    static double angleDifference(double a, double b)
    {
        double diff = normalizeAngle(a) - normalizeAngle(b);
        while (diff > 90.0) diff -= 180.0;
        while (diff < -90.0) diff += 180.0;
        return diff;
    }

    static bool validLine(const DebugLine2D &line)
    {
        return std::isfinite(line.x1) && std::isfinite(line.y1)
            && std::isfinite(line.x2) && std::isfinite(line.y2)
            && std::hypot(line.x2 - line.x1, line.y2 - line.y1) > 1e-3;
    }

    static Eigen::Vector2d direction(const DebugLine2D &line)
    {
        Eigen::Vector2d dir(line.x2 - line.x1, line.y2 - line.y1);
        const double length = dir.norm();
        if (length > 1e-9)
            return dir / length;
        return Eigen::Vector2d::Zero();
    }

    static double angle(const DebugLine2D &line)
    {
        return normalizeAngle(
            std::atan2(line.y2 - line.y1, line.x2 - line.x1)
            * 180.0 / pi_);
    }

    static double signedDistance(const Eigen::Vector2d &point,
                                 const DebugLine2D &line)
    {
        const Eigen::Vector2d dir = direction(line);
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        return (point - Eigen::Vector2d(line.x1, line.y1)).dot(normal);
    }

    static bool matches(const DebugLine2D &a, const DebugLine2D &b)
    {
        if (!validLine(a) || !validLine(b))
            return false;
        const Eigen::Vector2d midpoint(
            0.5 * (a.x1 + a.x2), 0.5 * (a.y1 + a.y2));
        return std::abs(angleDifference(angle(a), angle(b)))
                   <= angle_gate_deg_
            && std::abs(signedDistance(midpoint, b)) <= offset_gate_m_;
    }

    static std::array<Eigen::Vector2d, 4> corners(const Berth &berth)
    {
        const double rad = berth.angle * pi_ / 180.0;
        const double ca = std::cos(rad);
        const double sa = std::sin(rad);
        const double hw = berth.w * 0.5;
        const double hl = berth.l * 0.5;
        const double local[4][2] = {
            {-hw, -hl}, {hw, -hl}, {hw, hl}, {-hw, hl}};
        std::array<Eigen::Vector2d, 4> result;
        for (int i = 0; i < 4; ++i) {
            result[i] = Eigen::Vector2d(
                berth.cx + ca * local[i][0] - sa * local[i][1],
                berth.cy + sa * local[i][0] + ca * local[i][1]);
        }
        return result;
    }

    static DebugLine2D edgeLine(const std::array<Eigen::Vector2d, 4> &c,
                                int edge)
    {
        return DebugLine2D{
            c[edge].x(), c[edge].y(),
            c[(edge + 1) % 4].x(), c[(edge + 1) % 4].y()};
    }

    static void refreshDisplayGeometry(Berth &berth)
    {
        berth.visible_edges.clear();
        berth.has_opening_segment = false;
        if (berth.kind != BerthKind::UShape
            || berth.opening_edge < 0 || berth.opening_edge >= 4
            || !(berth.w > 1e-6) || !(berth.l > 1e-6)) {
            return;
        }
        const auto c = corners(berth);
        berth.opening_segment = edgeLine(c, berth.opening_edge);
        berth.has_opening_segment = true;
        berth.visible_edges.reserve(3);
        for (int edge = 0; edge < 4; ++edge) {
            if (edge != berth.opening_edge)
                berth.visible_edges.push_back(edgeLine(c, edge));
        }
    }

    static double minEdgeDistance(const Berth &berth,
                                  const DebugLine2D &line)
    {
        const auto c = corners(berth);
        double result = std::numeric_limits<double>::infinity();
        for (int edge = 0; edge < 4; ++edge) {
            result = std::min(
                result,
                std::abs(signedDistance(
                    0.5 * (c[edge] + c[(edge + 1) % 4]), line)));
        }
        return result;
    }

    template <typename T>
    static void remapOpposite(std::array<T, 4> &values)
    {
        const auto original = values;
        for (int edge = 0; edge < 4; ++edge)
            values[(edge + 2) % 4] = original[edge];
    }

    static bool alignToBus(Berth &berth,
                           const DebugLine2D &bus,
                           double max_edge_distance)
    {
        if (berth.kind != BerthKind::UShape
            || !(berth.w > 1e-3) || !(berth.l > 1e-3)) {
            return false;
        }
        const auto before = corners(berth);
        int nearest_edge = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (int edge = 0; edge < 4; ++edge) {
            const double distance = std::abs(signedDistance(
                0.5 * (before[edge] + before[(edge + 1) % 4]), bus));
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_edge = edge;
            }
        }
        if (nearest_distance > max_edge_distance)
            return false;

        berth.angle = normalizeAngle(
            angle(bus) - (nearest_edge % 2 == 0 ? 0.0 : 90.0));
        const auto rotated = corners(berth);
        const Eigen::Vector2d midpoint =
            0.5 * (rotated[nearest_edge]
                   + rotated[(nearest_edge + 1) % 4]);
        const Eigen::Vector2d dir = direction(bus);
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        const double offset = signedDistance(midpoint, bus);
        berth.cx -= offset * normal.x();
        berth.cy -= offset * normal.y();
        // The detector/protocol edge is authoritative. The bus is a fixed
        // geometry reference, not proof that its nearest side is the opening;
        // forcing that side here mirrors every berth when the bus is the wall.
        if (berth.opening_edge < 0 || berth.opening_edge >= 4)
            berth.opening_edge = nearest_edge;
        refreshDisplayGeometry(berth);
        return true;
    }

    static void flipToExpectedSide(Berth &berth,
                                   const DebugLine2D &bus,
                                   int expected_side)
    {
        if (expected_side == 0)
            return;
        const double distance = signedDistance(
            Eigen::Vector2d(berth.cx, berth.cy), bus);
        const int side =
            distance > 0.20 ? 1 : (distance < -0.20 ? -1 : 0);
        if (side == 0 || side == expected_side)
            return;

        const Eigen::Vector2d dir = direction(bus);
        const Eigen::Vector2d normal(-dir.y(), dir.x());
        berth.cx -= 2.0 * distance * normal.x();
        berth.cy -= 2.0 * distance * normal.y();
        if (berth.opening_edge >= 0 && berth.opening_edge < 4)
            berth.opening_edge = (berth.opening_edge + 2) % 4;
        remapOpposite(berth.edge_point_counts);
        remapOpposite(berth.edge_heights);
        remapOpposite(berth.edge_height_counts);
        remapOpposite(berth.edge_height_valid);
        remapOpposite(berth.ship_edge_distances);
        remapOpposite(berth.ship_edge_distance_valid);
        berth.ship_edge_distance_lines = {};
        berth.has_ship_metrics = false;
        refreshDisplayGeometry(berth);
    }
};

}  // namespace usv::berth_realtime

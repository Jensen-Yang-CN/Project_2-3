#pragma once

#include "common_types_extended.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <utility>
#include <vector>

namespace usv::berth_realtime {

/**
 * U 型泊位世界坐标稳定器。
 *
 * 正常观测累计 8 帧形成稳定库；偏离旧轨迹的观测必须形成连续 5 帧
 * 互相稳定的新簇后才能替换。调用方负责提供同一世界坐标系下的泊位。
 */
class RealtimeBerthStability {
public:
    std::vector<bool> update(std::vector<Berth> &measurements)
    {
        std::vector<bool> accepted(measurements.size(), false);
        std::vector<bool> state_observed(states_.size(), false);

        for (size_t i = 0; i < measurements.size(); ++i) {
            Berth &measurement = measurements[i];
            if (measurement.kind != BerthKind::UShape) {
                accepted[i] = true;
                continue;
            }

            const size_t state_index = findState(measurement);
            if (state_index == states_.size()) {
                State state;
                state.last = measurement;
                state.window.push_back(measurement);
                states_.push_back(std::move(state));
                state_observed.push_back(true);
                accepted[i] = true;
                continue;
            }

            State &state = states_[state_index];
            state_observed[state_index] = true;
            alignAxes(measurement, state.last);
            if (!withinGate(measurement, state.last)) {
                if (state.replacement.empty()
                    || !withinGate(measurement, state.replacement.back())) {
                    state.replacement.clear();
                }
                state.replacement.push_back(measurement);
                while (state.replacement.size() > replacement_window_frames_) {
                    state.replacement.pop_front();
                }
                if (state.replacement.size() == replacement_window_frames_) {
                    state.last = mean(state.replacement, state.last);
                    state.window = state.replacement;
                    state.replacement.clear();
                    state.stable = true;
                    measurements[i] = state.last;
                    accepted[i] = true;
                }
                continue;
            }

            state.last = measurement;
            state.replacement.clear();
            state.window.push_back(measurement);
            while (state.window.size() > stability_window_frames_) {
                state.window.pop_front();
            }
            if (state.window.size() == stability_window_frames_) {
                state.stable = true;
                measurement = mean(state.window, state.last);
                state.last = measurement;
            }
            accepted[i] = true;
        }

        for (size_t i = 0; i < states_.size(); ++i) {
            if (!state_observed[i]) {
                states_[i].replacement.clear();
            }
        }
        return accepted;
    }

    std::vector<Berth> stableLibrary() const
    {
        std::vector<Berth> result;
        for (const State &state : states_) {
            if (state.stable) {
                result.push_back(state.last);
            }
        }
        return result;
    }

    /**
     * 对稳定结果及其全部窗口样本执行同一几何校正。
     *
     * 世界锚点总线贴线必须使用此接口，不能只修改 stableLibrary() 的副本，
     * 否则下一帧窗口均值会覆盖校正结果。
     */
    template <typename Corrector>
    void correctAll(Corrector &&corrector)
    {
        for (State &state : states_) {
            corrector(state.last);
            for (Berth &sample : state.window)
                corrector(sample);
            for (Berth &sample : state.replacement)
                corrector(sample);
        }
    }

    void clear() { states_.clear(); }

    size_t stabilityWindowFrames() const { return stability_window_frames_; }
    size_t replacementWindowFrames() const { return replacement_window_frames_; }

private:
    struct State {
        Berth last;
        std::deque<Berth> window;
        std::deque<Berth> replacement;
        bool stable = false;
    };

    static double normalizeAngle(double angle)
    {
        angle = std::fmod(angle, 180.0);
        return angle < 0.0 ? angle + 180.0 : angle;
    }

    static double signedAngleDifference(double measured, double reference)
    {
        double difference = normalizeAngle(measured) - normalizeAngle(reference);
        while (difference > 90.0) difference -= 180.0;
        while (difference < -90.0) difference += 180.0;
        return difference;
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
        const auto edgeLine = [&c](int edge) {
            return DebugLine2D{
                c[edge].x(), c[edge].y(),
                c[(edge + 1) % 4].x(), c[(edge + 1) % 4].y()};
        };
        berth.opening_segment = edgeLine(berth.opening_edge);
        berth.has_opening_segment = true;
        berth.visible_edges.reserve(3);
        for (int edge = 0; edge < 4; ++edge) {
            if (edge != berth.opening_edge)
                berth.visible_edges.push_back(edgeLine(edge));
        }
    }

    static void remapEdgeArrays(Berth &berth, int edge_shift)
    {
        auto remap = [edge_shift](auto &values) {
            auto original = values;
            for (size_t old_edge = 0; old_edge < values.size(); ++old_edge) {
                values[(old_edge + edge_shift) % values.size()] =
                    original[old_edge];
            }
        };
        remap(berth.edge_point_counts);
        remap(berth.edge_heights);
        remap(berth.edge_height_counts);
        remap(berth.edge_height_valid);
        remap(berth.ship_edge_distances);
        remap(berth.ship_edge_distance_valid);
        remap(berth.ship_edge_distance_lines);
        if (berth.opening_edge >= 0) {
            berth.opening_edge =
                (berth.opening_edge + edge_shift) % 4;
        }
        refreshDisplayGeometry(berth);
    }

    static void alignAxes(Berth &candidate, const Berth &reference)
    {
        const double direct =
            std::abs(candidate.w - reference.w) / size_gate_m_
            + std::abs(candidate.l - reference.l) / size_gate_m_
            + std::abs(signedAngleDifference(
                candidate.angle, reference.angle)) / angle_gate_deg_;
        const double raw_swapped_angle = candidate.angle + 90.0;
        const double swapped_angle = normalizeAngle(raw_swapped_angle);
        const double swapped =
            std::abs(candidate.l - reference.w) / size_gate_m_
            + std::abs(candidate.w - reference.l) / size_gate_m_
            + std::abs(signedAngleDifference(
                swapped_angle, reference.angle)) / angle_gate_deg_;
        if (swapped >= direct)
            return;

        std::swap(candidate.w, candidate.l);
        candidate.angle = swapped_angle;
        const double normalized_delta_rad =
            (raw_swapped_angle - swapped_angle) * pi_ / 180.0;
        const bool axes_reversed = std::cos(normalized_delta_rad) < 0.0;
        const int edge_shift = 3 + (axes_reversed ? 2 : 0);
        remapEdgeArrays(candidate, edge_shift);
    }

    static bool withinGate(const Berth &a, const Berth &b)
    {
        return std::abs(a.cx - b.cx) + std::abs(a.cy - b.cy)
                   < position_l1_gate_m_
            && std::abs(a.w - b.w) < size_gate_m_
            && std::abs(a.l - b.l) < size_gate_m_
            && std::abs(signedAngleDifference(a.angle, b.angle))
                   < angle_gate_deg_;
    }

    static Berth mean(const std::deque<Berth> &samples,
                      const Berth &fallback)
    {
        if (samples.empty())
            return fallback;
        Berth result = fallback;
        double cx = 0.0;
        double cy = 0.0;
        double w = 0.0;
        double l = 0.0;
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const Berth &sample : samples) {
            cx += sample.cx;
            cy += sample.cy;
            w += sample.w;
            l += sample.l;
            const double rad = sample.angle * pi_ / 180.0;
            sin_sum += std::sin(2.0 * rad);
            cos_sum += std::cos(2.0 * rad);
        }
        const double count = static_cast<double>(samples.size());
        result.cx = cx / count;
        result.cy = cy / count;
        result.w = w / count;
        result.l = l / count;
        result.angle = normalizeAngle(
            0.5 * std::atan2(sin_sum, cos_sum) * 180.0 / pi_);
        refreshDisplayGeometry(result);
        return result;
    }

    size_t findState(const Berth &measurement) const
    {
        size_t best_index = states_.size();
        double best_distance = association_distance_m_;
        for (size_t i = 0; i < states_.size(); ++i) {
            const State &state = states_[i];
            if (state.last.kind != measurement.kind)
                continue;
            const double distance = std::hypot(
                measurement.cx - state.last.cx,
                measurement.cy - state.last.cy);
            if (distance < best_distance) {
                best_distance = distance;
                best_index = i;
            }
        }
        return best_index;
    }

    std::vector<State> states_;
    static constexpr size_t stability_window_frames_ = 8;
    static constexpr size_t replacement_window_frames_ = 5;
    static constexpr double pi_ = 3.14159265358979323846;
    static constexpr double association_distance_m_ = 8.0;
    static constexpr double position_l1_gate_m_ = 3.0;
    static constexpr double size_gate_m_ = 3.0;
    static constexpr double angle_gate_deg_ = 13.0;
};

}  // namespace usv::berth_realtime

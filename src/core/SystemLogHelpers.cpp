#include "SystemLogHelpers.h"

#include "SystemFileLogger.h"

#include <QJsonObject>

#include <cmath>

namespace system_log {

namespace {

constexpr double kPosEps = 0.15;
constexpr double kSizeEps = 0.20;
constexpr double kAngleEps = 1.0;
constexpr double kBridgeEps = 0.20;

struct BerthLogState {
    bool initialized = false;
    bool detected = false;
    std::vector<usv::Berth> berths;
};

struct BridgeLogState {
    bool initialized = false;
    bool detected = false;
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    double width = 0.0;
    double height = 0.0;
};

BerthLogState &berthState()
{
    static BerthLogState state;
    return state;
}

BridgeLogState &bridgeState()
{
    static BridgeLogState state;
    return state;
}

bool berthStableForLog(const usv::Berth &a, const usv::Berth &b)
{
    return std::abs(a.cx - b.cx) < kPosEps
        && std::abs(a.cy - b.cy) < kPosEps
        && std::abs(a.w - b.w) < kSizeEps
        && std::abs(a.l - b.l) < kSizeEps
        && std::abs(a.angle - b.angle) < kAngleEps
        && a.kind == b.kind
        && a.opening_edge == b.opening_edge;
}

bool bridgeStableForLog(const usv::BridgeMeasureResult &a,
                        const usv::BridgeMeasureResult &b)
{
    if (a.is_detected != b.is_detected)
        return false;
    if (!a.is_detected)
        return true;
    return std::abs(a.average_center_3d.x() - b.average_center_3d.x()) < kBridgeEps
        && std::abs(a.average_center_3d.y() - b.average_center_3d.y()) < kBridgeEps
        && std::abs(a.average_center_3d.z() - b.average_center_3d.z()) < kBridgeEps
        && std::abs(a.width - b.width) < kBridgeEps
        && std::abs(a.height - b.height) < kBridgeEps;
}

}  // namespace

QJsonArray berthCornersJson(const usv::Berth &berth)
{
    const double rad = berth.angle * M_PI / 180.0;
    const double cosA = std::cos(rad);
    const double sinA = std::sin(rad);
    const double hw = berth.w * 0.5;
    const double hl = berth.l * 0.5;

    const double localX[4] = {-hw, hw, hw, -hw};
    const double localY[4] = {-hl, -hl, hl, hl};

    QJsonArray corners;
    for (int i = 0; i < 4; ++i) {
        QJsonObject pt;
        pt.insert(QStringLiteral("x"), berth.cx + localX[i] * cosA - localY[i] * sinA);
        pt.insert(QStringLiteral("y"), berth.cy + localX[i] * sinA + localY[i] * cosA);
        corners.append(pt);
    }
    return corners;
}

void logBerthResultIfChanged(const usv::BerthMeasureResult &res)
{
    if (!SystemFileLogger::instance().isEnabled())
        return;

    BerthLogState &state = berthState();

    if (!res.is_detected) {
        if (state.initialized && state.detected) {
            QJsonObject data;
            data.insert(QStringLiteral("timestamp"), res.timestamp);
            data.insert(QStringLiteral("detected"), false);
            SystemFileLogger::instance().logEvent(
                SystemLogLevel::Info, "BerthDetection", "berth_result", data);
        }
        state.initialized = true;
        state.detected = false;
        state.berths.clear();
        return;
    }

    bool changed = !state.initialized || !state.detected
        || state.berths.size() != res.expanded_berths.size();
    if (!changed) {
        for (size_t i = 0; i < res.expanded_berths.size(); ++i) {
            if (!berthStableForLog(res.expanded_berths[i], state.berths[i])) {
                changed = true;
                break;
            }
        }
    }
    if (!changed)
        return;

    QJsonObject data;
    data.insert(QStringLiteral("timestamp"), res.timestamp);
    data.insert(QStringLiteral("detected"), true);
    data.insert(QStringLiteral("berth_count"),
               static_cast<int>(res.expanded_berths.size()));

    QJsonArray berths;
    for (size_t i = 0; i < res.expanded_berths.size(); ++i) {
        const usv::Berth &b = res.expanded_berths[i];
        QJsonObject item;
        item.insert(QStringLiteral("index"), static_cast<int>(i + 1));
        item.insert(QStringLiteral("cx"), b.cx);
        item.insert(QStringLiteral("cy"), b.cy);
        item.insert(QStringLiteral("w"), b.w);
        item.insert(QStringLiteral("l"), b.l);
        item.insert(QStringLiteral("angle_deg"), b.angle);
        item.insert(QStringLiteral("kind"), static_cast<int>(b.kind));
        item.insert(QStringLiteral("opening_edge"), b.opening_edge);
        item.insert(QStringLiteral("corners"), berthCornersJson(b));
        berths.append(item);
    }
    data.insert(QStringLiteral("berths"), berths);

    SystemFileLogger::instance().logEvent(
        SystemLogLevel::Info, "BerthDetection", "berth_result", data);

    state.initialized = true;
    state.detected = true;
    state.berths = res.expanded_berths;
}

void logBridgeResultIfChanged(const usv::BridgeMeasureResult &result)
{
    if (!SystemFileLogger::instance().isEnabled())
        return;

    BridgeLogState &state = bridgeState();
    usv::BridgeMeasureResult last;
    last.is_detected = state.detected;
    last.average_center_3d = Eigen::Vector3d(state.cx, state.cy, state.cz);
    last.width = state.width;
    last.height = state.height;

    if (state.initialized && bridgeStableForLog(result, last))
        return;

    QJsonObject data;
    data.insert(QStringLiteral("timestamp"), result.timestamp);
    data.insert(QStringLiteral("detected"), result.is_detected);
    if (result.is_detected) {
        data.insert(QStringLiteral("center_x"), result.average_center_3d.x());
        data.insert(QStringLiteral("center_y"), result.average_center_3d.y());
        data.insert(QStringLiteral("center_z"), result.average_center_3d.z());
        data.insert(QStringLiteral("width_m"), result.width);
        data.insert(QStringLiteral("height_m"), result.height);
    }

    SystemFileLogger::instance().logEvent(
        SystemLogLevel::Info, "BridgeDetection", "bridge_result", data);

    state.initialized = true;
    state.detected = result.is_detected;
    state.cx = result.average_center_3d.x();
    state.cy = result.average_center_3d.y();
    state.cz = result.average_center_3d.z();
    state.width = result.width;
    state.height = result.height;
}

}  // namespace system_log

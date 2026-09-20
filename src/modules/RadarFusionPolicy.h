#pragma once

#include <QStringList>

#include <cmath>

namespace radar_fusion_policy {

/** 参与主点云界面显示的雷达。201/213/214 只进入显示输出。 */
inline const QStringList &displayRadarIds()
{
    static const QStringList ids = {
        QStringLiteral("210"),
        QStringLiteral("211"),
        QStringLiteral("201"),
        QStringLiteral("213"),
        QStringLiteral("214"),
    };
    return ids;
}

/** 参与泊位检测和 SLAM 算法融合的雷达。 */
inline const QStringList &algorithmRadarIds()
{
    static const QStringList ids = {
        QStringLiteral("210"),
        QStringLiteral("211"),
    };
    return ids;
}

inline bool isAlgorithmRadar(const QString &id)
{
    return algorithmRadarIds().contains(id);
}

inline bool isFrameSynchronized(double frameTimestamp,
                                double masterTimestamp,
                                double maxToleranceSec)
{
    return std::isfinite(frameTimestamp)
        && std::isfinite(masterTimestamp)
        && std::isfinite(maxToleranceSec)
        && maxToleranceSec >= 0.0
        && std::abs(frameTimestamp - masterTimestamp) <= maxToleranceSec;
}

} // namespace radar_fusion_policy

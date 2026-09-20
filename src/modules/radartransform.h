#ifndef RADARTRANSFORM_H
#define RADARTRANSFORM_H

#include "CalibrationConfig.h"

#include <QMatrix4x4>
#include <QMap>
#include <QString>

struct RadarConfig {
    QString id;
    QMatrix4x4 transform;
    uint8_t intensity;
};

class RadarTransform {

public:
    static RadarConfig getConfig(const QString &radarId)
    {
        static QMap<QString, RadarConfig> configCache;
        if (configCache.isEmpty()) {
            initConfigs(configCache);
        }
        return configCache.value(radarId, {radarId, QMatrix4x4(), 0});
    }

private:
    static void initConfigs(QMap<QString, RadarConfig> &cache)
    {
        cache.clear();
        const QMap<QString, LidarCalibrationEntry> entries =
            CalibrationConfig::instance().allLidarEntries();
        for (auto it = entries.constBegin(); it != entries.constEnd(); ++it) {
            const LidarCalibrationEntry &entry = it.value();
            cache.insert(entry.id, {entry.id, entry.T_to_210, entry.display_intensity});
        }
    }
};
#endif  // RADARTRANSFORM_H

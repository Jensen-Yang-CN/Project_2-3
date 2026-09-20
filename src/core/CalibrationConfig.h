#ifndef CALIBRATIONCONFIG_H
#define CALIBRATIONCONFIG_H

#include <QMap>
#include <QMatrix4x4>
#include <QJsonArray>
#include <QString>

#include <Eigen/Geometry>

struct BridgeCameraCalibration {
    double intrinsic_k[9] = {};
    double distortion[5] = {};
    double rotation_r[9] = {};
    double translation_t[3] = {};
};

struct LidarCalibrationEntry {
    QString id;
    QMatrix4x4 T_to_210;
    uint8_t display_intensity = 0;
};

/** 传感器外参配置：从 calibration.json 加载，失败时使用内置默认值 */
class CalibrationConfig
{
public:
    static CalibrationConfig &instance();

    bool load(const QString &filePath = QString());

    QString calibrationFilePath() const { return m_filePath; }
    QString referenceFrame() const { return m_referenceFrame; }

    Eigen::Isometry3d matrix210To211() const;
    QMatrix4x4 matrix210To211Q() const;
    QMatrix4x4 matrix211To210Q() const;
    Eigen::Matrix4f matrix210To211Eigen4f() const;

    Eigen::Isometry3d matrix211ToBody() const;
    Eigen::Isometry3d matrix210ToBody() const;

    LidarCalibrationEntry lidarEntry(const QString &id) const;
    QMap<QString, LidarCalibrationEntry> allLidarEntries() const;

    BridgeCameraCalibration bridgeCamera() const { return m_bridgeCamera; }
    BridgeCameraCalibration rightCamera() const { return m_rightCamera; }

private:
    CalibrationConfig();

    void initDefaults();
    static bool parseMatrix4RowMajor(const QJsonArray &arr, Eigen::Isometry3d &out);
    static QMatrix4x4 toQMatrix4x4(const Eigen::Isometry3d &iso);

    QString m_filePath;
    QString m_referenceFrame = QStringLiteral("210");

    Eigen::Isometry3d m_T_210_to_211 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d m_T_211_to_body = Eigen::Isometry3d::Identity();
    QMap<QString, LidarCalibrationEntry> m_lidars;
    BridgeCameraCalibration m_bridgeCamera;
    BridgeCameraCalibration m_rightCamera;
};

#endif  // CALIBRATIONCONFIG_H

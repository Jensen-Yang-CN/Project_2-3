#include "CalibrationConfig.h"

#include "AppConfig.h"

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>

namespace {

void setIsoFromRowMajor16(const double values[16], Eigen::Isometry3d &out)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix(row, col) = values[row * 4 + col];
        }
    }
    out = Eigen::Isometry3d(matrix);
}

bool parseDoubleArray(const QJsonValue &value, int expectedSize, double *out)
{
    if (!value.isArray())
        return false;
    const QJsonArray arr = value.toArray();
    if (arr.size() != expectedSize)
        return false;
    for (int i = 0; i < expectedSize; ++i) {
        if (!arr[i].isDouble())
            return false;
        out[i] = arr[i].toDouble();
    }
    return true;
}

}  // namespace

CalibrationConfig &CalibrationConfig::instance()
{
    static CalibrationConfig cfg;
    return cfg;
}

CalibrationConfig::CalibrationConfig()
{
    initDefaults();
}

void CalibrationConfig::initDefaults()
{
    m_referenceFrame = QStringLiteral("210");

    const double t210To211[16] = {
        0.6416347543, -0.7668231513, -0.0169439298, -0.2457911542,
        0.7667985142, 0.6418201544, -0.0093235223, -0.1280878914,
        0.0180244484, -0.0070102843, 0.9998129701, -0.0123589146,
        0.0, 0.0, 0.0, 1.0};
    setIsoFromRowMajor16(t210To211, m_T_210_to_211);

    const double t211ToBody[16] = {
        -0.526062435, 0.850443972, 0.00183432608, 8.0,
        0.85028448, 0.525919301, 0.0206201755, 0.3,
        0.0165715965, 0.0124071987, -0.999785699, 1.5,
        0.0, 0.0, 0.0, 1.0};
    setIsoFromRowMajor16(t211ToBody, m_T_211_to_body);

    m_lidars.clear();
    auto addLidar = [this](const char *id, const double m[16], uint8_t intensity) {
        LidarCalibrationEntry entry;
        entry.id = QString::fromLatin1(id);
        entry.display_intensity = intensity;
        Eigen::Isometry3d iso;
        setIsoFromRowMajor16(m, iso);
        entry.T_to_210 = toQMatrix4x4(iso);
        m_lidars.insert(entry.id, entry);
    };

    const double t210Identity[16] = {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const double t213To210[16] = {
        0.8988148388, 0.4289420519, -0.0902252833, -1.0231772568,
        -0.4148311055, 0.8989062232, 0.1410062254, -1.0126236883,
        0.1415875682, -0.0893102337, 0.9858887577, -0.4920311602,
        0.0, 0.0, 0.0, 1.0};
    const double t201To210[16] = {
        -0.2642025071, 0.9466835498, 0.1843564259, -4.2391828707,
        -0.9639074400, -0.2656920444, -0.0170347986, -12.7556322301,
        0.0328554721, -0.1822031671, 0.9827118213, 4.9636308346,
        0.0, 0.0, 0.0, 1.0};
    const double t214To210[16] = {
        -0.8696290412, -0.4628442850, 0.1718153040, 0.8920096402,
        0.4735037367, -0.8804419331, 0.0248236531, -2.1405130560,
        0.1397839124, 0.1029425580, 0.9848163724, -0.4774459018,
        0.0, 0.0, 0.0, 1.0};

    addLidar("210", t210Identity, 250);
    {
        LidarCalibrationEntry entry;
        entry.id = QStringLiteral("211");
        entry.display_intensity = 150;
        entry.T_to_210 = toQMatrix4x4(m_T_210_to_211.inverse());
        m_lidars.insert(entry.id, entry);
    }
    addLidar("213", t213To210, 100);
    addLidar("201", t201To210, 50);
    addLidar("214", t214To210, 200);

    m_bridgeCamera = {};
    const double kDefault[9] = {
        1121.996820617591, 0.0, 1015.384051830748,
        0.0, 1121.713320959861, 497.0475695528236,
        0.0, 0.0, 1.0};
    std::copy(std::begin(kDefault), std::end(kDefault), std::begin(m_bridgeCamera.intrinsic_k));
    m_bridgeCamera.distortion[0] = -0.110955866103800;
    m_bridgeCamera.distortion[1] = 0.095194442475660;
    m_bridgeCamera.distortion[2] = 0.001555726425174;
    m_bridgeCamera.distortion[3] = 0.0004240877681588755;
    m_bridgeCamera.distortion[4] = 0.0005282108284090351;
    const double rDefault[9] = {
        0.96907, 0.246661, 0.00788997,
        0.00876667, -0.00245628, -0.999959,
        -0.246632, 0.969099, -0.00454271};
    std::copy(std::begin(rDefault), std::end(rDefault), std::begin(m_bridgeCamera.rotation_r));
    m_bridgeCamera.translation_t[0] = -2.33679;
    m_bridgeCamera.translation_t[1] = 1.96762;
    m_bridgeCamera.translation_t[2] = 7.9897;

    m_rightCamera = m_bridgeCamera;
    const double rightKDefault[9] = {
        1123.956601732128, 0.0, 1012.661344967900,
        0.0, 1124.007223450208, 522.1319076047985,
        0.0, 0.0, 1.0};
    std::copy(std::begin(rightKDefault), std::end(rightKDefault),
              std::begin(m_rightCamera.intrinsic_k));
    m_rightCamera.distortion[0] = -0.111741370458708;
    m_rightCamera.distortion[1] = 0.096569118218854;
    m_rightCamera.distortion[2] = -0.0001114834487810880;
    m_rightCamera.distortion[3] = 0.0005205286147479465;
    m_rightCamera.distortion[4] = -0.004293509544096;
}

bool CalibrationConfig::parseMatrix4RowMajor(const QJsonArray &arr, Eigen::Isometry3d &out)
{
    double values[16];
    if (!parseDoubleArray(arr, 16, values))
        return false;
    setIsoFromRowMajor16(values, out);
    return true;
}

QMatrix4x4 CalibrationConfig::toQMatrix4x4(const Eigen::Isometry3d &iso)
{
    const Eigen::Matrix4d matrix = iso.matrix();
    return QMatrix4x4(
        static_cast<float>(matrix(0, 0)), static_cast<float>(matrix(0, 1)),
        static_cast<float>(matrix(0, 2)), static_cast<float>(matrix(0, 3)),
        static_cast<float>(matrix(1, 0)), static_cast<float>(matrix(1, 1)),
        static_cast<float>(matrix(1, 2)), static_cast<float>(matrix(1, 3)),
        static_cast<float>(matrix(2, 0)), static_cast<float>(matrix(2, 1)),
        static_cast<float>(matrix(2, 2)), static_cast<float>(matrix(2, 3)),
        static_cast<float>(matrix(3, 0)), static_cast<float>(matrix(3, 1)),
        static_cast<float>(matrix(3, 2)), static_cast<float>(matrix(3, 3)));
}

bool CalibrationConfig::load(const QString &filePath)
{
    initDefaults();

    const QString path = filePath.isEmpty()
        ? AppConfig::instance().calibrationFilePath()
        : AppConfig::resolvePath(filePath);
    m_filePath = path;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "[CalibrationConfig] 未找到外参配置文件，使用默认值:" << path;
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        qWarning() << "[CalibrationConfig] 外参配置文件格式无效:" << path;
        return false;
    }

    const QJsonObject root = doc.object();
    if (root.contains(QStringLiteral("reference_frame")))
        m_referenceFrame = root.value(QStringLiteral("reference_frame")).toString(m_referenceFrame);

    const QJsonObject transforms = root.value(QStringLiteral("transforms")).toObject();
    const QJsonObject t210To211 = transforms.value(QStringLiteral("T_210_to_211")).toObject();
    if (t210To211.contains(QStringLiteral("matrix_4x4"))) {
        if (!parseMatrix4RowMajor(t210To211.value(QStringLiteral("matrix_4x4")).toArray(), m_T_210_to_211)) {
            qWarning() << "[CalibrationConfig] T_210_to_211 解析失败，保留默认值";
        }
    }
    const QJsonObject t211ToBody = transforms.value(QStringLiteral("T_211_to_body")).toObject();
    if (t211ToBody.contains(QStringLiteral("matrix_4x4"))) {
        if (!parseMatrix4RowMajor(t211ToBody.value(QStringLiteral("matrix_4x4")).toArray(), m_T_211_to_body)) {
            qWarning() << "[CalibrationConfig] T_211_to_body 解析失败，保留默认值";
        }
    }

    const QJsonObject lidars = root.value(QStringLiteral("lidars")).toObject();
    for (auto it = lidars.begin(); it != lidars.end(); ++it) {
        if (!it.value().isObject())
            continue;
        const QJsonObject obj = it.value().toObject();
        if (!obj.contains(QStringLiteral("T_to_210")))
            continue;

        Eigen::Isometry3d iso;
        if (!parseMatrix4RowMajor(obj.value(QStringLiteral("T_to_210")).toArray(), iso))
            continue;

        LidarCalibrationEntry entry;
        entry.id = it.key();
        entry.T_to_210 = toQMatrix4x4(iso);
        entry.display_intensity = static_cast<uint8_t>(
            obj.value(QStringLiteral("display_intensity")).toInt(
                m_lidars.value(entry.id).display_intensity));
        m_lidars.insert(entry.id, entry);
    }

    if (m_lidars.contains(QStringLiteral("211"))) {
        LidarCalibrationEntry entry211 = m_lidars.value(QStringLiteral("211"));
        entry211.T_to_210 = toQMatrix4x4(m_T_210_to_211.inverse());
        m_lidars.insert(QStringLiteral("211"), entry211);
    }

    const QJsonObject bridge = root.value(QStringLiteral("bridge_camera")).toObject();
    if (!bridge.isEmpty()) {
        parseDoubleArray(bridge.value(QStringLiteral("intrinsic_K")), 9, m_bridgeCamera.intrinsic_k);
        parseDoubleArray(bridge.value(QStringLiteral("distortion")), 5, m_bridgeCamera.distortion);
        parseDoubleArray(bridge.value(QStringLiteral("R_211_to_camera")), 9, m_bridgeCamera.rotation_r);
        parseDoubleArray(bridge.value(QStringLiteral("T_211_to_camera")), 3, m_bridgeCamera.translation_t);
    }

    const QJsonObject rightCamera = root.value(QStringLiteral("right_camera")).toObject();
    if (!rightCamera.isEmpty()) {
        parseDoubleArray(rightCamera.value(QStringLiteral("intrinsic_K")), 9,
                         m_rightCamera.intrinsic_k);
        parseDoubleArray(rightCamera.value(QStringLiteral("distortion")), 5,
                         m_rightCamera.distortion);
    }

    qInfo() << "[CalibrationConfig] 已加载" << path
            << "reference=" << m_referenceFrame
            << "lidars=" << m_lidars.size();
    return true;
}

Eigen::Isometry3d CalibrationConfig::matrix210To211() const
{
    return m_T_210_to_211;
}

QMatrix4x4 CalibrationConfig::matrix210To211Q() const
{
    return toQMatrix4x4(m_T_210_to_211);
}

QMatrix4x4 CalibrationConfig::matrix211To210Q() const
{
    return matrix210To211Q().inverted();
}

Eigen::Matrix4f CalibrationConfig::matrix210To211Eigen4f() const
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    const Eigen::Matrix4d source = m_T_210_to_211.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix(row, col) = static_cast<float>(source(row, col));
        }
    }
    return matrix;
}

Eigen::Isometry3d CalibrationConfig::matrix211ToBody() const
{
    return m_T_211_to_body;
}

Eigen::Isometry3d CalibrationConfig::matrix210ToBody() const
{
    return m_T_211_to_body * m_T_210_to_211;
}

LidarCalibrationEntry CalibrationConfig::lidarEntry(const QString &id) const
{
    return m_lidars.value(id);
}

QMap<QString, LidarCalibrationEntry> CalibrationConfig::allLidarEntries() const
{
    return m_lidars;
}

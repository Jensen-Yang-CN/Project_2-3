#include "AppConfig.h"
#include "CalibrationConfig.h"
#include "RadarFusionPolicy.h"

#include <Eigen/Core>

#include <QMatrix4x4>
#include <QStringList>
#include <QVector3D>

#include <filesystem>
#include <cctype>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace {

int g_failures = 0;

void check(bool condition, const std::string &message)
{
    if (!condition) {
        ++g_failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void testDisplayAndAlgorithmRadarMembership()
{
    const QStringList displayIds =
        radar_fusion_policy::displayRadarIds();
    const QStringList algorithmIds =
        radar_fusion_policy::algorithmRadarIds();

    check(displayIds == QStringList({"210", "211", "201", "213", "214"}),
          "显示点云必须包含 201/210/211/213/214");
    check(algorithmIds == QStringList({"210", "211"}),
          "算法点云必须继续只包含 210/211");
    check(displayIds.contains(QStringLiteral("201"))
              && !algorithmIds.contains(QStringLiteral("201")),
          "201 必须进入显示输出，但不得进入算法输出");
    check(!radar_fusion_policy::isAlgorithmRadar("213")
              && !radar_fusion_policy::isAlgorithmRadar("214"),
          "213/214 不得进入 SLAM 或泊位检测使用的算法点云");
}

void testRadarFramesMustRespectSynchronizationWindow()
{
    constexpr double master_ts = 100.0;
    constexpr double tolerance_sec = 0.10;

    check(radar_fusion_policy::isFrameSynchronized(
              master_ts + 0.099, master_ts, tolerance_sec),
          "a radar frame inside the synchronization window must be accepted");
    check(!radar_fusion_policy::isFrameSynchronized(
              master_ts + 0.101, master_ts, tolerance_sec),
          "a stale radar frame outside the synchronization window must be rejected");
    check(!radar_fusion_policy::isFrameSynchronized(
              std::numeric_limits<double>::quiet_NaN(),
              master_ts, tolerance_sec),
          "a radar frame with a non-finite timestamp must be rejected");
}

Eigen::Matrix4d expectedRestored210To211()
{
    Eigen::Matrix4d expected;
    expected <<
        0.6416347543, -0.7668231513, -0.0169439298, -0.2457911542,
        0.7667985142, 0.6418201544, -0.0093235223, -0.1280878914,
        0.0180244484, -0.0070102843, 0.9998129701, -0.0123589146,
        0.0, 0.0, 0.0, 1.0;
    return expected;
}

void checkCurrentCalibration(const std::string &context)
{
    const CalibrationConfig &calibration =
        CalibrationConfig::instance();
    const Eigen::Matrix4d actual =
        calibration.matrix210To211().matrix();
    const Eigen::Matrix4d expected =
        expectedRestored210To211();
    check((actual - expected).cwiseAbs().maxCoeff() < 1e-9,
          context + ": 210→211 外参必须恢复为更新前矩阵");

    const Eigen::Matrix4d round_trip =
        actual.inverse() * actual;
    check((round_trip - Eigen::Matrix4d::Identity())
              .cwiseAbs().maxCoeff() < 1e-9,
          context + ": 210→211 与 211→210 必须闭环为单位阵");

    const QVector3D point210(3.25f, -1.5f, 0.75f);
    const QVector3D point211 =
        calibration.matrix210To211Q().map(point210);
    const QVector3D restored =
        calibration.matrix211To210Q().map(point211);
    check((restored - point210).length() < 1e-5f,
          context + ": 点坐标正反变换后必须恢复原值");

    const LidarCalibrationEntry entry211 =
        calibration.lidarEntry(QStringLiteral("211"));
    const QVector3D restored_by_fusion =
        entry211.T_to_210.map(point211);
    check((restored_by_fusion - point210).length() < 1e-5f,
          context + ": 五雷达融合的 211→210 必须由新外参求逆");
}

void testRestoredExtrinsicIsTheDefaultAndRuntimeCalibration()
{
    checkCurrentCalibration("内置默认值");

    const std::filesystem::path source_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path config_path =
        source_root / "config" / "calibration.json";
    check(CalibrationConfig::instance().load(
              QString::fromStdString(config_path.string())),
          "必须能加载源码目录中的 calibration.json");
    checkCurrentCalibration("calibration.json");
}

void testRuntimeNavigationSynchronizationWindow()
{
    const std::filesystem::path source_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path config_path =
        source_root / "config" / "config.json";
    check(AppConfig::instance().load(
              QString::fromStdString(config_path.string())),
          "the source config.json must load");
    check(std::abs(AppConfig::instance().slam().max_gnss_sync_sec - 0.25)
              < 1e-12,
          "GNSS/INS synchronization must be limited to 250 ms");
}

void testMainWindowSeparatesDisplayAndAlgorithmConsumers()
{
    const std::filesystem::path source_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(source_root / "src" / "gui" / "mainwindow.cpp");
    const std::string source((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
    std::string compact;
    compact.reserve(source.size());
    for (const unsigned char ch : source) {
        if (!std::isspace(ch))
            compact.push_back(static_cast<char>(ch));
    }

    check(compact.find(
              "&RadarFusionManager::displayCloudReady,ui->openGLWidget,"
              "&MultiLidarWidget::updateFusedCloud") != std::string::npos,
          "display output must connect only to the point-cloud widget");
    check(compact.find(
              "&RadarFusionManager::algorithmCloudReady,this,"
              "&MainWindow::onFusedCloudToBus") != std::string::npos,
          "algorithm output must connect to the SLAM/perception bus entry");
    check(compact.find("RadarFusionManager::fusedCloudReady")
              == std::string::npos,
          "the old shared fusion signal must not remain connected");
}

} // namespace

int main()
{
    testDisplayAndAlgorithmRadarMembership();
    testRadarFramesMustRespectSynchronizationWindow();
    testRestoredExtrinsicIsTheDefaultAndRuntimeCalibration();
    testRuntimeNavigationSynchronizationWindow();
    testMainWindowSeparatesDisplayAndAlgorithmConsumers();
    if (g_failures != 0) {
        std::cerr << g_failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All radar update tests passed\n";
    return 0;
}

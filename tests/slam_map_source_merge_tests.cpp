#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path sourceRoot()
{
    std::filesystem::path path = std::filesystem::current_path();
    if (std::filesystem::exists(path / "src" / "slam" / "DloSlamNode.cpp"))
        return path;
    if (std::filesystem::exists(path.parent_path() / "src" / "slam"
                                / "DloSlamNode.cpp")) {
        return path.parent_path();
    }
    return path;
}

std::string readFile(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    std::ostringstream content;
    content << input.rdbuf();
    return content.str();
}

bool contains(const std::string &text, const std::string &needle,
              const char *message, int &failures)
{
    if (text.find(needle) != std::string::npos)
        return true;
    ++failures;
    std::cerr << "FAIL: " << message << " (missing: " << needle << ")\n";
    return false;
}

void testZipDynamicFilterWasMerged(int &failures)
{
    const std::filesystem::path root = sourceRoot();
    const std::string config = readFile(
        root / "src" / "code-slam" / "LidarGnssSlamConfig.h");
    const std::string header = readFile(
        root / "src" / "code-slam" / "LidarGnssSlam.h");

    contains(config, "kDynamicSorMeanK",
             "zip dynamic-filter configuration must be present", failures);
    contains(config, "kDynamicSpeedThreshold",
             "zip dynamic speed threshold must be present", failures);
    contains(header, "getLastStaticCloud",
             "SLAM must expose the static cloud used for mapping", failures);
    contains(header, "getLastNavigationReference",
             "current geographic reference API must be preserved", failures);
}

void testStaticCloudFeedsMapOutputs(int &failures)
{
    const std::string dlo = readFile(
        sourceRoot() / "src" / "slam" / "DloSlamNode.cpp");
    contains(dlo, "algorithm_->getLastStaticCloud()",
             "DLO must consume the algorithm static cloud", failures);
    contains(dlo, "createKeyframe(",
             "keyframes must be built from static points", failures);
    contains(dlo, "bundle->timestamp, pose_state.pose, static_points",
             "keyframes must pass the static cloud", failures);
    contains(dlo, "world_scan.reserve(static_cast<int>(static_points.size()))",
             "live map output must use the static cloud", failures);
    contains(dlo, "algorithm_->getLastNavigationReference()",
             "keyframes must preserve navigation references", failures);
    contains(dlo, "slam_map_io::saveArchive",
             "the DLO save entry point must use the current archive format",
             failures);
    if (dlo.find("keyframes_poses.csv") != std::string::npos) {
        ++failures;
        std::cerr << "FAIL: DLO save entry point must not create the legacy "
                     "directory map export\n";
    }
}

void testLegacyMapLoadPathWasMerged(int &failures)
{
    const std::string task = readFile(
        sourceRoot() / "src" / "slam" / "SlamMapLoadTask.cpp");
    const std::string window = readFile(
        sourceRoot() / "src" / "gui" / "mainwindow.cpp");
    contains(task, "suffix().compare(QStringLiteral(\"pcd\")",
             "map loading must recognize the archive legacy .pcd format",
             failures);
    contains(task, "source_format_major = 1",
             "legacy maps must be wrapped with an explicit source version",
             failures);
    contains(window, "SLAM 地图 (*.slammap *.pcd)",
             "GUI must allow selecting legacy .pcd maps", failures);
    contains(window, "QStringLiteral(\".slammap\")",
             "new map saving must remain .slammap based", failures);
}

void testCurrentGeoSyncGuardWasPreserved(int &failures)
{
    const std::string types = readFile(
        sourceRoot() / "common" / "slam_types.h");
    const std::string config = readFile(
        sourceRoot() / "config" / "config.json");
    contains(types, "max_gnss_sync_sec = 0.25",
             "current map geo-reference sync guard must remain strict",
             failures);
    if (config.find("\"max_gnss_sync_sec\": 2.0") != std::string::npos) {
        ++failures;
        std::cerr << "FAIL: legacy 2.0 second sync threshold must not replace "
                     "the current map guard\n";
    }
}

void testRuntimeMapTopicsAndBerthControl(int &failures)
{
    const std::filesystem::path root = sourceRoot();
    const std::string dlo = readFile(root / "src" / "slam" / "DloSlamNode.cpp");
    const std::string window = readFile(root / "src" / "gui" / "mainwindow.cpp");
    const std::string windowHeader = readFile(root / "src" / "gui" / "mainwindow.h");

    contains(dlo, "getStateTopic<SlamKeyframe>(\"slam/keyframe\"",
             "DLO must publish keyframes to the topic consumed by the UI", failures);
    contains(dlo, "getStateTopic<SlamScanCloudMessage>(\"slam/scan_cloud\"",
             "DLO must publish live scans to the topic consumed by the UI", failures);
    contains(dlo, "slam_keyframe_topic_->publish(",
             "generated keyframes must actually be published", failures);
    contains(dlo, "slam_scan_topic_->publish(",
             "generated live scans must actually be published", failures);
    contains(windowHeader, "QToolButton *berthDetectBtn;",
             "the toolbar must expose an independent berth button", failures);
    contains(window, "berthDetectBtn->setText(tr(\"泊位检测\"))",
             "the independent berth button must be clearly labelled", failures);
    contains(window, "void MainWindow::onBerthDetectionClicked()",
             "the independent berth button must have its own handler", failures);
}

}  // namespace

int main()
{
    int failures = 0;
    testZipDynamicFilterWasMerged(failures);
    testStaticCloudFeedsMapOutputs(failures);
    testLegacyMapLoadPathWasMerged(failures);
    testCurrentGeoSyncGuardWasPreserved(failures);
    testRuntimeMapTopicsAndBerthControl(failures);
    if (failures != 0)
        return 1;
    std::cout << "All SLAM map source merge tests passed\n";
    return 0;
}

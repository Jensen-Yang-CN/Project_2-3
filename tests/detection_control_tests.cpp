#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

std::string readCompact(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    const std::string source((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
    std::string compact;
    compact.reserve(source.size());
    for (const unsigned char ch : source) {
        if (!std::isspace(ch))
            compact.push_back(static_cast<char>(ch));
    }
    return compact;
}

} // namespace

int main()
{
    int failures = 0;
    const std::filesystem::path root =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::string header = readCompact(root / "src" / "gui" / "mainwindow.h");
    const std::string source = readCompact(root / "src" / "gui" / "mainwindow.cpp");
    const std::string widgetHeader =
        readCompact(root / "src" / "gui" / "MultiLidarWidget.h");
    const std::string widgetSource =
        readCompact(root / "src" / "gui" / "MultiLidarWidget.cpp");
    const std::string bridgeHeader =
        readCompact(root / "src" / "bus" / "BridgeDetectionNode.h");
    const std::string bridgeSource =
        readCompact(root / "src" / "bus" / "BridgeDetectionNode.cpp");
    const std::string berthHeader =
        readCompact(root / "src" / "bus" / "BerthDetectionNode.h");
    const std::string berthSource =
        readCompact(root / "src" / "bus" / "BerthDetectionNode.cpp");
    const std::string commonTypes =
        readCompact(root / "src" / "bus" / "common_types_extended.h");
    const std::string pointTypes =
        readCompact(root / "common" / "pointxyz.h");
    const std::string dispatcherHeader =
        readCompact(root / "src" / "network" / "UdpDispatcher.h");
    const std::string dispatcherSource =
        readCompact(root / "src" / "network" / "UdpDispatcher.cpp");

    check(header.find("QToolButton*bridgeDetectBtn;") != std::string::npos,
          "the toolbar must expose a dedicated bridge detection button", failures);
    check(header.find("QToolButton*berthDetectBtn;") != std::string::npos,
          "the toolbar must expose a dedicated berth detection button", failures);
    check(header.find("m_bridgeDetectionEnabled") != std::string::npos
              && header.find("m_berthDetectionEnabled") != std::string::npos,
          "bridge and berth detection must have independent state", failures);
    check(header.find("m_bridgeDetectionAllowed") != std::string::npos
              && header.find("DetectionNodeScheduler") != std::string::npos,
          "manual bridge permission must be separate from effective scheduling state",
          failures);
    check(header.find("m_perceptionDetectEnabled") == std::string::npos,
          "the legacy shared perception switch must be removed", failures);
    check(source.find("onBridgeDetectionClicked") != std::string::npos
              && source.find("onBerthDetectionClicked") != std::string::npos,
          "each detection button must have its own slot", failures);
    check(source.find("bridgeDetectBtn->setText(tr(\"桥梁检测\"))")
              != std::string::npos
              && source.find("berthDetectBtn->setText(tr(\"泊位检测\"))")
              != std::string::npos,
          "button labels must make the two responsibilities explicit", failures);
    check(source.find("m_bridgeDetectionEnabled") != std::string::npos
              && source.find("tr(\"停止桥检\")") != std::string::npos
              && source.find("tr(\"桥梁检测\")") != std::string::npos,
          "桥梁按钮文字必须跟随开关状态", failures);
    check(source.find("m_berthDetectionEnabled") != std::string::npos
              && source.find("tr(\"停止泊检\")") != std::string::npos
              && source.find("tr(\"泊位检测\")") != std::string::npos,
          "泊位按钮文字必须跟随开关状态", failures);
    check(source.find("m_bridgeDetectionEnabled.load(") != std::string::npos,
          "camera and bridge inputs must be gated by the bridge switch", failures);
    check(source.find("m_berthDetectionEnabled.load(") != std::string::npos,
          "fused berth input must be gated by the berth switch", failures);
    check(source.find("m_bridgeDetectionEnabled.load(std::memory_order_relaxed)"
                      "&&ui->openGLWidgetLeft") != std::string::npos
              && source.find("m_bridgeDetectionEnabled.load(std::memory_order_relaxed)"
                             "&&ui->openGLWidgetRight") != std::string::npos,
          "late camera bridge overlays must be rejected after bridge detection is disabled",
          failures);
    check(source.find("m_berthDetectionEnabled.load(std::memory_order_relaxed)"
                      "&&ui->openGLWidget") != std::string::npos,
          "late berth overlays must be rejected after berth detection is disabled",
          failures);
    check(source.find("m_perceptionDetectEnabled") == std::string::npos,
          "mainwindow implementation must not retain the shared switch", failures);
    check(source.find("processBerthResultForScheduling") != std::string::npos
              && source.find("BerthDetectionSource::Realtime")
                     != std::string::npos
              && source.find("!berth.is_anchor_recovered")
                     != std::string::npos,
          "automatic scheduling must use fresh realtime berths, not blind recall",
          failures);
    check(source.find("setBridgePermission(enabled)") != std::string::npos
              && source.find("applyBridgeComputeState") != std::string::npos,
          "bridge button must act as the hard permission for automatic scheduling",
          failures);
    check(source.find("startBaselineComputeNodes") != std::string::npos
              && source.find("initSlamNode()") != std::string::npos,
          "berth and SLAM must be started as baseline compute nodes", failures);
    check(bridgeHeader.find("setProcessingEnabled") != std::string::npos
              && bridgeHeader.find("processing_enabled_") != std::string::npos,
          "bridge node must expose a reusable processing gate", failures);
    check(bridgeSource.find("processing_enabled_.load(") != std::string::npos
              && bridgeSource.find("sync_queue_.clear()") != std::string::npos
              && bridgeSource.find("playback_timeline_generation_.fetch_add(")
                     != std::string::npos,
          "bridge standby must gate inputs, clear work, and invalidate late tasks",
          failures);
    check(bridgeHeader.find("left_vision_pending_generation_")
                  != std::string::npos
              && bridgeHeader.find("right_vision_pending_generation_")
                     != std::string::npos
              && bridgeSource.find("compare_exchange_strong(")
                     != std::string::npos,
          "camera pending ownership must be generation-aware across rapid toggles",
          failures);
    check(commonTypes.find("uint64_ttimeline_generation=1;")
                  != std::string::npos
              && berthHeader.find("playback_timeline_generation_")
                     != std::string::npos
              && berthSource.find("res.timeline_generation=timeline_generation;")
                     != std::string::npos
              && berthSource.find("timeline_generation!=playback_timeline_generation_.load(")
                     != std::string::npos,
          "late berth results must carry and validate their playback generation",
          failures);
    check(header.find("m_expectedBerthTimelineGeneration")
                  != std::string::npos
              && source.find("result.timeline_generation!=m_expectedBerthTimelineGeneration")
                     != std::string::npos,
          "the UI scheduler must reject queued berth results from an old timeline",
          failures);
    check(source.find("resetSlamForTimelineChange") != std::string::npos
              && source.find("stopSlamNode();") != std::string::npos
              && source.find("initSlamNode();") != std::string::npos,
          "SLAM state must be rebuilt when the input timeline changes", failures);
    check(header.find("m_sensorTimelineGeneration") != std::string::npos
              && pointTypes.find("quint64timeline_generation=1;")
                     != std::string::npos
              && source.find("cloudReadyWithGeneration") != std::string::npos,
          "sensor frames must carry a timeline generation through the lidar pipeline",
          failures);
    check(dispatcherHeader.find("setAcceptedTimelineGeneration")
                  != std::string::npos
              && dispatcherHeader.find("dispatchWithGeneration")
                     != std::string::npos
              && dispatcherSource.find("!=m_acceptedTimelineGeneration.load(")
                     != std::string::npos,
          "UDP dispatch must reject packets from an obsolete source timeline",
          failures);
    check(source.find("PcapngReader::udpPacketWithGeneration")
                  != std::string::npos
              && source.find("RadarFusionManager::algorithmCloudReady,this,"
                             "&MainWindow::onFusedCloudToBus,Qt::DirectConnection")
                     != std::string::npos,
          "source generations must reach dispatch and fused output must not linger in the UI queue",
          failures);
    check(header.find("m_pclPerceptionBusy") != std::string::npos
              && header.find("m_pclPerceptionGeneration") != std::string::npos
              && header.find("m_pclPreview") != std::string::npos,
          "PCL perception must keep one persistent algorithm and busy-drop state",
          failures);
    check(source.find("QtConcurrent::run") != std::string::npos
              && source.find("schedulePclPerception") != std::string::npos,
          "fused point-cloud perception must execute in a background task",
          failures);
    check(source.find("resetPclPerception") != std::string::npos
              && source.find("clearPclClusterPreview") != std::string::npos,
          "bridge disable and playback seek must reset stale PCL state",
          failures);
    check(source.find("result.bridgeVerticalSpanMeters") != std::string::npos
              && source.find("result.bridgePointCount") != std::string::npos
              && source.find("pier.centerDistanceMeters") != std::string::npos,
          "PCL bridge logs must include vertical span, highlighted points and pier center distance",
          failures);
    check(widgetHeader.find("updatePclClusterPreview") != std::string::npos
              && widgetHeader.find("clearPclClusterPreview") != std::string::npos,
          "the point-cloud widget must expose PCL overlay update and clear slots",
          failures);
    check(widgetSource.find("drawPclClusterPreview") != std::string::npos
              && widgetSource.find("drawOverlayArrays") != std::string::npos,
          "PCL semantics must render through the existing overlay VBO path",
          failures);
    check(widgetSource.find("glBegin(") == std::string::npos,
          "PCL overlay must not use removed immediate-mode OpenGL", failures);

    if (failures != 0)
        return 1;
    std::cout << "All detection control tests passed\n";
    return 0;
}

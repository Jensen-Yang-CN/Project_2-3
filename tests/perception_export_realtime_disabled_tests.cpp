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

std::string readFile(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
}

}  // namespace

int main()
{
    int failures = 0;
    const std::string source =
        readFile(std::string(__FILE__).substr(
            0, std::string(__FILE__).find_last_of("\\/"))
                 + "/../src/bus/PerceptionExportNode.cpp");
    const std::string mainwindow_source =
        readFile(std::string(__FILE__).substr(
            0, std::string(__FILE__).find_last_of("\\/"))
                 + "/../src/gui/mainwindow.cpp");

    check(!source.empty(), "perception export implementation should be readable",
          failures);
    check(source.find("kEnableRealtimeMapExport = true") != std::string::npos,
          "realtime map export must be enabled for offline replay delivery",
          failures);
    check(source.find("!kEnableRealtimeMapExport") != std::string::npos,
          "realtime map paths must consult the hard-disable policy", failures);
    check(source.find("历史重叠虚拟关键帧") != std::string::npos,
          "historical overlap map export must remain available", failures);
    check(source.find("sendBerth") != std::string::npos,
          "berth export path must remain available", failures);
    check(source.find("kHistoricalMapResolutionM = 0.05")
              != std::string::npos,
          "historical map packets should use the denser 0.05 m grid", failures);
    check(source.find("historyState.origin_gnss = historyGnss")
              != std::string::npos,
          "historical berths must use the same history ENU anchor as 0xFC", failures);
    check(source.find("不要先把它伪装成当前\n    // 雷达二维坐标") != std::string::npos,
          "historical berths must avoid the lidar extrinsic round-trip", failures);
    check(source.find("kRealtimeBerthCacheMatchM = 6.0") != std::string::npos,
          "realtime berth delivery must not merge nearby berths", failures);
    check(source.find("realtime_map_to_enu_") != std::string::npos,
          "realtime map packets must use a fixed map-to-ENU transform",
          failures);
    check(source.find("realtime_map_anchor_") != std::string::npos,
          "realtime map packets must use a fixed geographic anchor", failures);
    check(mainwindow_source.find("node->feedImu(copy)") != std::string::npos,
          "realtime export must receive IMU directly from the playback path",
          failures);
    check(source.find("subscribeSlamBus()") != std::string::npos,
          "realtime export must subscribe to live SLAM topics", failures);
    check(source.find("slam/keyframe") != std::string::npos,
          "realtime export must forward live keyframes", failures);

    check(source.find("sendStaticBerthLibraryIfDue(true)") == std::string::npos,
          "starting realtime delivery must not send the fixed berth layer",
          failures);
    check(source.find("const std::size_t static_count") == std::string::npos,
          "realtime berth batches must not reserve slots for fixed berths",
          failures);
    check(source.find("overlaps_fixed_berth") == std::string::npos,
          "realtime berth batches must not filter against fixed berths",
          failures);
    check(source.find("trySendHistoricalBerths();") != std::string::npos,
          "historical map loading must retain the historical berth path",
          failures);
    check(source.find("sendStaticBerthLibraryIfDue();") != std::string::npos,
          "historical berth delivery must retain the fixed berth library",
          failures);
    const std::string fixedSendCall = "sendStaticBerthLibraryIfDue();";
    const std::size_t firstFixedSend = source.find(fixedSendCall);
    check(firstFixedSend != std::string::npos,
          "historical berth path should call the fixed berth sender", failures);
    if (firstFixedSend != std::string::npos) {
        check(source.find(fixedSendCall, firstFixedSend + fixedSendCall.size())
                  == std::string::npos,
              "realtime GNSS updates must not resend the fixed berth layer",
              failures);
    }

    if (failures != 0)
        return 1;
    std::cout << "Realtime map export policy tests passed\n";
    return 0;
}

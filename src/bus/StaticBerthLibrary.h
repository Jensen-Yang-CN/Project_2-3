#pragma once

#include "PerceptionUdpProtocol.h"
#include "common_types_extended.h"
#include "common/slam_types.h"

#include <QString>

#include <vector>

namespace static_berth_library {

struct LoadResult {
    std::vector<BerthUdpUnit> units;
    // 与 UDP 单元同源的本地绘制结果；不参与协议编码。
    std::vector<usv::Berth> display_berths;

    // JSON 的 east_m/north_m 以报告生成时的局部 ENU 原点为准。
    // 记录一个带经纬度的参考点，供本机历史地图显示层换算到地图锚点。
    struct DisplayCoordinateReference {
        bool valid = false;
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        double altitude_m = 0.0;
        double projected_east_m = 0.0;
        double projected_north_m = 0.0;
    } display_reference;
    bool display_berths_rebased = false;
};

/**
 * 读取泊位地理信息 JSON，筛选最终 library 记录并按 ENU 中心去重。
 * 返回的单元仍为主机字节序，交给 berth_udp::buildBerthPackets 编码。
 */
bool load(const QString &path, LoadResult &result, QString *error = nullptr);

// 仅平移本地绘制泊位到目标 ENU 锚点；UDP units 保持原始协议坐标不变。
bool rebaseDisplayBerthsToAnchor(
    LoadResult &result,
    const usv::SlamGeoAnchor &target_anchor,
    QString *error = nullptr);

}  // namespace static_berth_library

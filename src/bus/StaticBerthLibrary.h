#pragma once

#include "PerceptionUdpProtocol.h"

#include <QString>

#include <vector>

namespace static_berth_library {

struct LoadResult {
    std::vector<BerthUdpUnit> units;
};

/**
 * 读取泊位地理信息 JSON，筛选最终 library 记录并按 ENU 中心去重。
 * 返回的单元仍为主机字节序，交给 berth_udp::buildBerthPackets 编码。
 */
bool load(const QString &path, LoadResult &result, QString *error = nullptr);

}  // namespace static_berth_library

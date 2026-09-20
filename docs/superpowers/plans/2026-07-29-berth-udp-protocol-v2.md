# 泊位 UDP 投递协议 V2 实现计划

> **面向 AI 代理的工作者：** 使用 executing-plans 在当前会话逐任务实现；当前目录不是 Git 仓库，不能使用 worktree 或逐任务 commit，改以备份、测试和构建结果作为检查点。

**目标：** 把 `0xFB` 泊位投递更新为 `bowei2.h` 的 20 字节包头、72 字节单元和每包最多 10 个泊位的批次分包协议。

**架构：** 将纯字节编码和分包逻辑从 `PerceptionExportNode` 提取到可独立测试的 `BerthUdpPacketBuilder`。节点仍负责 GNSS/INS、泊位几何和特征点排序，组包器只接收已排序的 LiDAR 局部点并生成网络字节序报文。

**技术栈：** C++17、Qt5 Core/Network、CTest。

---

## 文件结构

- 创建 `src/bus/BerthUdpPacketBuilder.h`：定义协议点、泊位单元编码和分包 API。
- 创建 `src/bus/BerthUdpPacketBuilder.cpp`：实现高度编码、单元填充、网络字节序和分包。
- 创建 `tests/berth_udp_protocol_tests.cpp`：验证协议尺寸、字段、分包和 LiDAR 局部坐标。
- 修改 `src/bus/PerceptionUdpProtocol.h`：更新 V2 线协议结构和常量。
- 修改 `src/bus/PerceptionExportNode.cpp`：改用组包器逐包发送。
- 修改 `src/bus/PerceptionExportNode.h`：增加 16 位泊位批次号。
- 修改 `CMakeLists.txt`：把组包器加入主目标并注册测试。

### 任务 1：建立失败的协议测试

- [ ] 在 `tests/berth_udp_protocol_tests.cpp` 编写断言：

```cpp
static_assert(sizeof(BerthUdpHeader) == 20);
static_assert(sizeof(BerthUdpUnit) == 72);
check(buildBerthPackets(100, 7, oneUnit).front().size() == 92,
      "一个泊位的包长必须是 92");
check(buildBerthPackets(100, 7, elevenUnits).size() == 2,
      "11 个泊位必须拆为两包");
```

- [ ] 在 `CMakeLists.txt` 增加 `berth_udp_protocol_tests` 目标和 `berth_udp_protocol` CTest。
- [ ] 运行：

```powershell
cmake --build build-radar-slam --target berth_udp_protocol_tests -j 2
```

预期：因为 V2 字段或 `BerthUdpPacketBuilder` 尚不存在而失败，证明测试覆盖新行为。

### 任务 2：更新线协议结构和纯组包器

- [ ] 将 `BerthUdpHeader` 改为：

```cpp
struct BerthUdpHeader {
    uint8_t prefix[4];
    uint32_t total_len;
    int32_t timestamp;
    uint16_t frame_id;
    uint16_t total_count;
    uint8_t id;
    uint8_t berth_num;
    uint8_t packet_index;
    uint8_t packet_count;
};
```

- [ ] 在 `BerthUdpUnit` 的四个保留字节之后增加：

```cpp
uint8_t deck_height;
uint8_t clear_height;
uint8_t clear_depth;
uint8_t reserve4;
```

- [ ] 增加常量：

```cpp
constexpr std::size_t kMaxBerthsPerPacket = 10;
constexpr std::size_t kMaxBerthPacketsPerBatch = 255;
constexpr std::size_t kMaxBerthsPerBatch = 2550;
```

- [ ] 实现 `encodeDeckHeight`：忽略开口边和无效高度，排序后取中值，按米乘 10 编码并限制到 `0..255`。
- [ ] 实现 `makeBerthUdpUnit`：经纬高与 LiDAR 局部 `x/y/z` 按现有比例编码。
- [ ] 实现 `buildBerthPackets`：

```cpp
const std::size_t total = std::min(units.size(), kMaxBerthsPerBatch);
const uint8_t packetCount =
    static_cast<uint8_t>((total + kMaxBerthsPerPacket - 1)
                         / kMaxBerthsPerPacket);
```

每包写入相同的时间戳、`frameId`、`totalCount` 和 `packetCount`，并转换全部多字节字段为大端。

- [ ] 再次构建并运行：

```powershell
cmake --build build-radar-slam --target berth_udp_protocol_tests -j 2
ctest --test-dir build-radar-slam -R berth_udp_protocol --output-on-failure
```

预期：测试通过。

### 任务 3：接入投递节点

- [ ] 在 `PerceptionExportNode::sendBerth` 保留现有 GNSS/INS 换算和开口点排序。
- [ ] 将每个已排序角点填入 `BerthProtocolPoint`，其中局部坐标使用：

```cpp
point.x_m = corner.p_lidar.x();
point.y_m = corner.p_lidar.y();
point.z_m = corner.p_lidar.z();
```

不得继续使用 `corner.world_pt` 填写 `x/y/z`。

- [ ] 对每个检测结果执行一次：

```cpp
const quint16 frameId = ++berth_frame_id_;
const auto packets = buildBerthPackets(timestamp, frameId, units);
for (const QByteArray &packet : packets)
    sendUdpPayload(packet, "0xFB");
```

- [ ] 若泊位数超过 2550，日志报告截断；`totalCount` 表示实际投递数量。
- [ ] 构建主程序：

```powershell
cmake --build build-radar-slam --target PointCloud -j 2
```

预期：退出码 0。

### 任务 4：协议回归验证

- [ ] 运行：

```powershell
ctest --test-dir build-radar-slam --output-on-failure
```

- [ ] 确认 `berth_recall` 仍验证点 1、4 为开口侧。
- [ ] 检查源码不再存在旧包长：

```powershell
rg -n "16 \\+ berth_num|sizeof\\(BerthUdpUnit\\).*68|reserve1 = 0.*kBerthId" src tests
```

预期：没有旧 `0xFB` 组包逻辑残留；全部测试通过。

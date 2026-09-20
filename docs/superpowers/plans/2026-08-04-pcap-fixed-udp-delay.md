# PCAP 固定 UDP 包间隔慢进实现计划

> **面向实现者：** 在当前源代码快照内按 TDD 顺序执行；该目录不是 Git 仓库，因此不执行 commit 步骤。

**目标：** 增加通过 `config.json` 控制的固定 UDP 包间隔慢进模式，同时保留原始时间戳回放。

**架构：** `AppConfig` 负责读取和约束配置，`MainWindow` 在创建读取器时注入配置，`PcapngReader` 在有效 UDP 包投递后进行可中断等待。自动化测试使用运行时生成的最小 PCAP 验证真实读取链路。

**技术栈：** C++17、Qt 5、libpcap/Npcap、CMake/CTest。

---

### 任务 1：建立失败测试

**文件：**
- 创建：`tests/pcap_playback_tests.cpp`
- 修改：`CMakeLists.txt`

- [x] 编写配置读取、延时范围、UDP 过滤、固定等待、停止响应和原始时间戳模式测试。
- [x] 运行 `cmake --build build-radar-slam --target pcap_playback_tests -j2`，确认因缺少 `PcapPlaybackConfig`、`pcapPlayback()` 和 `setPlaybackConfig()` 失败。

### 任务 2：实现配置模型

**文件：**
- 修改：`src/core/AppConfig.h`
- 修改：`src/core/AppConfig.cpp`
- 修改：`config/config.json`

- [x] 增加 `PcapPlaybackConfig`，默认关闭固定延时并保留 2 ms 默认值。
- [x] 读取 `pcap_playback`，将延时约束到 `0..1000`。
- [x] 在项目源配置中启用 2 ms 固定延时。

### 任务 3：实现读取器慢进模式

**文件：**
- 修改：`src/network/PcapngReader.h`
- 修改：`src/network/PcapngReader.cpp`
- 修改：`src/gui/mainwindow.cpp`

- [x] 增加配置注入接口和读取器成员。
- [x] 固定延时启用时跳过原始时间戳限速。
- [x] 增加 IPv4 的 UDP 协议检查。
- [x] 在有效 UDP 载荷投递后分段等待并检查停止标志。
- [x] 创建读取器时注入 `AppConfig::pcapPlayback()`。

### 任务 4：验证

**文件：**
- 验证：`tests/pcap_playback_tests.cpp`
- 验证：完整 `PointCloud` 目标与现有 CTest。

- [x] 构建并运行 `pcap_playback_tests`，全部断言通过。
- [x] 构建 `PointCloud`，退出码为 0。
- [x] 运行 `ctest --test-dir build-radar-slam --output-on-failure`，12 项测试全部通过。
- [x] 核对构建目录中的 `config.json` 已包含 2 ms 慢进配置。

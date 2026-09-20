# PCAP 完整播放器调度器实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 用统一的抓包时间戳调度替换逐 UDP 包固定延时，提供播放/暂停和 1x/2x/4x 快进，同时保证视频、点云、导航及泊位检测继续接收完整且时间戳不变的数据。

**架构：** 新增线程安全的 `PcapPlaybackController`，由 PCAP 读取线程在读取每个抓包记录后调用。控制器用条件变量实现暂停、恢复、倍速变化和立即停止；界面按钮只调用线程安全控制接口，不把控制任务排进正忙于读取的工作线程事件队列。启动时不再自动恢复历史 SLAM 地图，手动加载入口保持不变。

**技术栈：** C++17、Qt 5、libpcap、CMake/CTest、`std::mutex`、`std::condition_variable`

---

## 文件结构

- 创建 `src/network/PcapPlaybackController.h/.cpp`：独立、可测试的播放器时钟与状态机。
- 修改 `src/network/PcapngReader.h/.cpp`：移除逐 UDP 延时，接入统一时间戳调度器并暴露线程安全控制接口。
- 修改 `src/gui/mainwindow.h/.cpp`：增加播放/暂停、倍速控件和状态同步；关闭历史地图启动自动恢复。
- 修改 `src/core/AppConfig.h/.cpp`、`config/config.json`：删除有问题的逐包延时配置，保留播放器默认倍速配置。
- 修改 `tests/pcap_playback_tests.cpp`：覆盖原始时间戳、1x/2x/4x、暂停/恢复及停止唤醒。
- 修改 `CMakeLists.txt`：让程序和测试目标编译新的调度器。

### 任务 1：播放器状态机测试

- [ ] 在 `tests/pcap_playback_tests.cpp` 编写测试：1x 等待约等于抓包间隔、2x/4x 按比例缩短；暂停期间不放行下一包；恢复继续；停止能立即唤醒。
- [ ] 运行 `cmake --build build-radar-slam --target pcap_playback_tests -j 4`，确认测试因 `PcapPlaybackController` 尚不存在而失败。

### 任务 2：实现时间戳调度器并替换固定延时

- [ ] 实现 `PcapPlaybackController::waitForPacket(double)`：首包立即放行，后续按相邻抓包时间差和当前倍速等待，暂停时间不计入播放时间。
- [ ] 实现 `setPaused`、`setSpeed`、`stop`、`reset`，所有状态修改均持锁并通知条件变量。
- [ ] 在 `PcapngReader::readLoop()` 的协议过滤之前统一调度抓包记录，发给下游的 `currentPacketTime` 保持原值。
- [ ] 运行 `pcap_playback_tests`，确认上述测试通过。

### 任务 3：界面播放器控制

- [ ] 在工具栏增加 `播放/暂停` 按钮和 `倍速 1x/2x/4x` 按钮；未播放离线文件时禁用。
- [ ] 播放期间点击暂停直接调用线程安全接口；恢复从下一抓包记录继续；倍速按钮按 1x→2x→4x→1x 循环。
- [ ] 读取完成、手动停止或加载新文件时重置按钮状态；不改动 `pathPlanBtn` 和算法总线连接。

### 任务 4：屏蔽历史 SLAM 自动恢复

- [ ] 删除构造函数中的 `QTimer::singleShot(...restoreLastSlamMap)` 自动调用。
- [ ] 保留 `onLoadSlamMapClicked()`、地图保存、最近地图记录及手动切换功能。

### 任务 5：完整验证

- [ ] 重新配置并构建主程序和 `pcap_playback_tests`。
- [ ] 运行 `ctest --test-dir build-radar-slam --output-on-failure`，确认全量测试无失败。
- [ ] 检查源代码中不再存在 `fixed_udp_delay_enabled`、`udp_packet_delay_ms` 或 `waitAfterUdpPacket` 的生产逻辑。
- [ ] 核对播放器只调节发包时机，不改动原始时间戳、UDP 分流、视频解码及泊位检测总线。

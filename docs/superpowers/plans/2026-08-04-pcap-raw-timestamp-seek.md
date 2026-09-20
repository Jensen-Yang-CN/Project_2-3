# PCAP 原始时间戳跳转实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 给现有 PCAP 播放器增加按原始 Unix 时间戳跳转，并通过 2 秒隐藏预热保证视频、融合点云和泊位检测在目标位置继续工作。

**架构：** `PcapngReader` 接收值类型播放请求，在工作线程中扫描范围、快速跳包、预热并恢复时间戳调度。`MainWindow` 负责输入验证、停止/重启读取任务、冻结绘制和屏蔽检测；解析器及检测节点提供线程安全的时间线重置入口。

**技术栈：** C++17、Qt 5、libpcap、FFmpeg、CMake/CTest

**执行环境说明：** 当前目录不是 Git 仓库，无法建立 worktree 或执行逐任务 commit；每个任务完成后以目标测试和主程序构建作为检查点。

---

## 文件结构

- 创建 `src/network/PcapPlaybackRequest.h`：单次播放/跳转请求值对象。
- 修改 `src/network/PcapngReader.h/.cpp`：范围扫描、快速定位、预热状态和当前时间信号。
- 修改 `tests/pcap_playback_tests.cpp`：范围、目标包、预热边界、超范围及既有播放回归测试。
- 修改 `src/protocol/RsLidarParser.h/.cpp`：请求式雷达时间轴重置。
- 修改 `src/modules/VideoWorker.h/.cpp`：在线程内重建视频解码器。
- 修改 `src/modules/radarfusionmanager.h/.cpp`：清除旧时间线融合缓存。
- 修改 `src/bus/ThreadSafeQueue.h`、`src/bus/BerthDetectionNode.h/.cpp`：清队列并在首个新时间线任务前重置泊位算法状态。
- 修改 `src/gui/mainwindow.h/.cpp`：时间输入、会话重启、绘制冻结、检测屏蔽及状态栏显示。
- 修改 `CMakeLists.txt`：把新请求头加入程序和测试目标。

### 任务 1：定义跳转请求与读取器行为测试

**文件：**
- 创建：`src/network/PcapPlaybackRequest.h`
- 修改：`tests/pcap_playback_tests.cpp`

- [ ] **步骤 1：先写失败测试**

测试使用合成 PCAP，时间戳依次为 `100.0、100.5、101.0、101.5、102.0`，期望接口如下：

```cpp
PcapPlaybackRequest request;
request.files = {pcapPath};
request.playback_speed = 4.0;
request.seek_enabled = true;
request.target_timestamp_sec = 101.5;
request.preroll_sec = 1.0;
request.scan_range = true;

reader.startPlayback(request);
```

断言：范围为 `100.0~102.0`；快速扫描不发送 `100.0`；预热发送 `100.5、101.0`；目标及后续发送 `101.5、102.0`；目标完成实际时间为 `101.5`；所有 UDP 载荷和原始时间戳不变。

- [ ] **步骤 2：运行红灯验证**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;'+$env:Path
cmake --build build-radar-slam --target pcap_playback_tests -j 4
```

预期：因 `PcapPlaybackRequest.h` 或 `PcapngReader::startPlayback` 尚不存在而失败。

### 任务 2：实现范围扫描、快速定位和预热

**文件：**
- 创建：`src/network/PcapPlaybackRequest.h`
- 修改：`src/network/PcapngReader.h`
- 修改：`src/network/PcapngReader.cpp`
- 修改：`CMakeLists.txt`
- 测试：`tests/pcap_playback_tests.cpp`

- [ ] **步骤 1：实现请求值对象**

```cpp
struct PcapPlaybackRequest {
    QStringList files;
    double playback_speed = 1.0;
    bool seek_enabled = false;
    double target_timestamp_sec = 0.0;
    double preroll_sec = 2.0;
    bool scan_range = true;
};
```

- [ ] **步骤 2：实现读取器信号和统一入口**

```cpp
void startPlayback(const PcapPlaybackRequest &request);

void captureRangeReady(double firstTimestamp, double lastTimestamp);
void playbackTimestampChanged(double timestamp);
void seekPrerollChanged(bool active, double targetTimestamp);
void seekCompleted(double requestedTimestamp, double actualTimestamp);
void playbackError(const QString &message);
```

`startAnalysis(files)` 继续存在，并转换为普通 `PcapPlaybackRequest` 调用统一入口。

- [ ] **步骤 3：实现扫描和三阶段读取**

- 范围扫描只调用 `pcap_next_ex` 并记录首末时间，不构造 UDP 载荷。
- `timestamp < target-preroll` 时快速跳过且不等待、不发包。
- 预热阶段重置播放器为 4x，按原顺序发送完整 UDP。
- 首个 `timestamp >= target` 的包到达前结束预热，按请求倍速重置时钟并发出 `seekCompleted`。
- 每 200 ms 墙钟时间最多发送一次 `playbackTimestampChanged`。
- 文件结束仍未找到目标时发送 `playbackError`，并确保预热状态关闭。

- [ ] **步骤 4：运行绿灯验证**

```powershell
.\build-radar-slam\pcap_playback_tests.exe
```

预期：范围、预热、目标包、超范围和原有暂停/倍速/高包率测试全部通过。

### 任务 3：重置跨时间线解析和检测状态

**文件：**
- 修改：`src/protocol/RsLidarParser.h`
- 修改：`src/protocol/RsLidarParser.cpp`
- 修改：`src/modules/VideoWorker.h`
- 修改：`src/modules/VideoWorker.cpp`
- 修改：`src/modules/radarfusionmanager.h`
- 修改：`src/modules/radarfusionmanager.cpp`
- 修改：`src/bus/ThreadSafeQueue.h`
- 修改：`src/bus/BerthDetectionNode.h`
- 修改：`src/bus/BerthDetectionNode.cpp`

- [ ] **步骤 1：添加线程安全重置入口**

```cpp
// RsLidarParser：只设置原子标记，由点云处理线程重置时间戳累计器。
void requestPlaybackTimelineReset();

// VideoWorker：在所属解码线程执行。
void resetDecoder();

// RadarFusionManager：在 GUI 所属线程清理同步队列与定时器。
void resetForPlaybackSeek();

// BerthDetectionNode：清空待处理队列，在首个新任务前重置算法与世界库。
void requestPlaybackTimelineReset();
```

- [ ] **步骤 2：保证泊位状态只在消费线程重置**

`requestPlaybackTimelineReset()` 调用 `sync_queue_.clear()` 并设置原子标志；`processLoop()` 在处理首个新任务前依次调用 `u_shape_algo_.init()`、`line_algo_.init()`、`resetWorldRecallState()` 和清空日志状态，避免跨线程直接修改检测器。

- [ ] **步骤 3：构建受影响目标**

```powershell
cmake --build build-radar-slam --target PointCloud -j 4
```

预期：解析器、融合和检测节点的新接口全部编译通过。

### 任务 4：接入时间跳转界面和隐藏预热

**文件：**
- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`

- [ ] **步骤 1：增加会话状态和按钮**

增加 `timestampSeekBtn`，点击后用 `QInputDialog::getText` 获取原始时间戳；保存：

```cpp
QStringList m_currentPcapFiles;
bool m_captureRangeValid = false;
double m_captureFirstTimestamp = 0.0;
double m_captureLastTimestamp = 0.0;
bool m_seekPending = false;
bool m_seekInProgress = false;
bool m_seekPrerollActive = false;
double m_pendingSeekTimestamp = 0.0;
```

- [ ] **步骤 2：实现输入和重启协调**

- 仅接受可转换为有限正数且位于已知范围内的输入。
- 保存跳转请求后调用线程安全的 `reader->stopReading()`。
- `finished` 信号处理器发现 `m_seekPending` 时不恢复普通空闲界面，而是重置解析状态并把带目标的请求投递到读取线程。
- 跳转失败或普通停止时统一解除预热冻结、恢复按钮。

- [ ] **步骤 3：冻结绘制并屏蔽检测**

- 预热开始时对实时点云、左右视频和红外视频控件调用 `setUpdatesEnabled(false)`，清除旧检测框。
- 视频、雷达、融合和 IMU 仍接收预热包。
- `onBridgeCloud210/211`、相机检测提交及 `publishFusedSensors` 的检测参数在 `m_seekPrerollActive` 时关闭。
- 将泊位/桥梁结果连接改为经 MainWindow 检查预热状态的 lambda，防止旧任务结果重新覆盖界面。
- 预热结束时重新启用绘制并调用 `update()`。

- [ ] **步骤 4：状态栏显示范围和当前时间**

范围和当前值均用 `QString::number(value, 'f', 6)`，跳转完成日志同时显示请求值和实际首包值。

### 任务 5：完整验证

**文件：**
- 验证：`build-radar-slam/PointCloud.exe`
- 验证：全部 CTest 目标

- [ ] **步骤 1：构建播放器测试和主程序**

```powershell
$env:Path='D:\msys64\ucrt64\bin;'+$env:Path
cmake --build build-radar-slam --target pcap_playback_tests PointCloud -j 4
```

- [ ] **步骤 2：运行全量测试**

```powershell
ctest --test-dir build-radar-slam --output-on-failure -j 4
```

预期：所有测试 0 失败。

- [ ] **步骤 3：静态需求核对**

确认启动构造函数没有调用 `restoreLastSlamMap`；检测开关逻辑没有被暂停、倍速或跳转按钮改写；预热前包不进入 `udpPacket`；目标包时间戳保持原值。

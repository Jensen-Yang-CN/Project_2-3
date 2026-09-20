# PCAP USV 时间跳转、0.5x 回放与暂停快照实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让 PCAP 播放器支持按 USV 面板本地时间跳转、0.5x 慢放，并在点击暂停时异步保存 210/211 原始点云及左右相机原图。

**架构：** 将本地时间解析和暂停快照收集/写盘拆成独立、可测试组件；`PcapngReader` 只提供精确的最后放行时间戳，`MainWindow` 负责协调暂停、300 ms 收尾和后台保存。现有时间跳转、UDP 分流、视频/雷达解析及检测流水线保持不变。

**技术栈：** C++17、Qt 5 Core/Gui/Concurrent、libpcap、CMake/CTest、PCD binary、PNG

**执行环境说明：** 当前目录不是 Git 仓库，无法建立 worktree 或提交 commit；每个任务完成后使用对应测试目标与主程序构建作为检查点。

---

## 文件结构

- 创建 `src/network/PcapPlaybackTime.h/.cpp`：解析 USV 本地时间或 Unix 秒，格式化显示值和快照文件名。
- 修改 `src/network/PcapPlaybackController.cpp`、`src/core/AppConfig.cpp`、`src/network/PcapngReader.h/.cpp`：允许 0.5x，并记录最后放行抓包时间。
- 创建 `src/debug/PauseSnapshot.h/.cpp`：维护四路最新帧/暂停候选，校验同步完整性并写出 binary PCD 与 PNG。
- 修改 `src/gui/MainWindow.h`、`src/gui/mainwindow.cpp`：时间输入、0.5x 按钮循环、暂停收集和后台保存。
- 修改 `tests/pcap_playback_tests.cpp`：时间转换、0.5x 和精确放行时间测试。
- 创建 `tests/pause_snapshot_tests.cpp`：候选选择、完整性及四文件落盘测试。
- 修改 `CMakeLists.txt`：加入新源文件和 `pause_snapshot_tests`。

### 任务 1：USV 本地时间与 Unix 时间转换

**文件：**
- 创建：`src/network/PcapPlaybackTime.h`
- 创建：`src/network/PcapPlaybackTime.cpp`
- 修改：`tests/pcap_playback_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：编写失败测试**

在 `tests/pcap_playback_tests.cpp` 中增加：

```cpp
void testUsvLocalTimeAndUnixTimestampParsing()
{
    const QTimeZone shanghai("Asia/Shanghai");
    const auto local = pcap_playback_time::parseTimestamp(
        QStringLiteral("2026-06-10 14:28:34.145"), shanghai);
    check(local.ok, "USV local display time must parse");

    const QDateTime expected(
        QDate(2026, 6, 10), QTime(14, 28, 34, 145), shanghai);
    check(std::abs(local.unix_seconds
                   - expected.toMSecsSinceEpoch() / 1000.0) < 1e-6,
          "USV local time must map to the same Unix timeline");

    const auto raw = pcap_playback_time::parseTimestamp(
        QString::number(local.unix_seconds, 'f', 6), shanghai);
    check(raw.ok && std::abs(raw.unix_seconds - local.unix_seconds) < 1e-6,
          "legacy Unix timestamp input must remain supported");
    check(pcap_playback_time::snapshotBaseName(local.unix_seconds, shanghai)
              == QStringLiteral("20260610_142834_145"),
          "snapshot filename must use a Windows-safe USV time");
    check(!pcap_playback_time::parseTimestamp(
               QStringLiteral("2026-02-30 12:00:00.000"), shanghai).ok,
          "invalid calendar date must be rejected");
}
```

- [ ] **步骤 2：运行测试验证红灯**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;'+$env:Path
cmake --build build-radar-slam --target pcap_playback_tests -j 4
```

预期：编译失败，提示 `PcapPlaybackTime.h` 或 `pcap_playback_time` 不存在。

- [ ] **步骤 3：实现最小时间工具**

`src/network/PcapPlaybackTime.h` 定义：

```cpp
namespace pcap_playback_time {
struct ParseResult {
    bool ok = false;
    double unix_seconds = 0.0;
    QString error;
};

ParseResult parseTimestamp(
    const QString &text,
    const QTimeZone &zone = QTimeZone::systemTimeZone());
QString formatLocal(
    double unixSeconds,
    const QTimeZone &zone = QTimeZone::systemTimeZone());
QString snapshotBaseName(
    double unixSeconds,
    const QTimeZone &zone = QTimeZone::systemTimeZone());
}
```

实现先尝试有限正数 Unix 秒，再严格解析 `yyyy-MM-dd HH:mm:ss.zzz`；本地时间通过传入 `QTimeZone` 转为 epoch。`formatLocal` 输出毫秒格式，`snapshotBaseName` 输出 `yyyyMMdd_HHmmss_zzz`。

- [ ] **步骤 4：运行测试验证绿灯**

运行 `build-radar-slam\pcap_playback_tests.exe`，预期时间解析测试与既有测试全部通过。

### 任务 2：0.5x 播放和最后放行时间戳

**文件：**
- 修改：`src/network/PcapPlaybackController.cpp`
- 修改：`src/network/PcapPlaybackController.h`
- 修改：`src/core/AppConfig.cpp`
- 修改：`src/network/PcapngReader.h`
- 修改：`src/network/PcapngReader.cpp`
- 修改：`tests/pcap_playback_tests.cpp`

- [ ] **步骤 1：编写 0.5x 和精确时间失败测试**

```cpp
void testHalfSpeedDoublesCaptureInterval()
{
    const qint64 atHalf = measureInterval(0.5, 0.16);
    check(atHalf >= 260 && atHalf < 650,
          "0.5x must replay 160 ms of capture time in about 320 ms");
}

void testReaderReportsLastReleasedPacketTimestamp()
{
    QTemporaryDir dir;
    const QString pcapPath = writeTimedPcap(
        dir.filePath("last-released.pcap"),
        {
            {60.000, makeIpv4Frame(17, QByteArray("first"))},
            {60.125, makeIpv4Frame(17, QByteArray("last"))},
        });
    PcapngReader reader;
    PcapPlaybackRequest request;
    request.files = QStringList{pcapPath};
    request.playback_speed = 4.0;
    request.scan_range = false;
    reader.startPlayback(request);
    check(std::abs(reader.lastReleasedTimestamp() - 60.125) < 1e-6,
          "reader must expose the exact last packet released by its playback clock");
}
```

同时把低速配置测试的预期值从 `1.0` 改为 `0.5`，输入配置为 `0.2`。

- [ ] **步骤 2：运行测试验证红灯**

运行 `pcap_playback_tests`，预期 0.5x 仍被收敛到 1x，或缺少 `lastReleasedTimestamp()`。

- [ ] **步骤 3：实现 0.5x 和原子时间值**

- `PcapPlaybackController::normalizeSpeed` 使用 `std::clamp(speed, 0.5, 4.0)`；
- `AppConfig` 的 `default_speed` 使用相同范围；
- `PcapngReader` 请求速度使用相同范围；
- `PcapPlaybackController` 在 `waitForPacket()` 即将返回 `true` 前，使用现有 `mutex_` 更新 `last_released_timestamp_sec_`，并增加同锁只读接口；这样 GUI 暂停取得控制器锁后不会读到刚放行包之前的旧值。
- `PcapngReader` 的只读接口直接转发控制器值：

```cpp
double lastReleasedTimestamp() const
{
    return m_playbackController.lastReleasedTimestamp();
}
```

每次 `PcapPlaybackController::reset()` 把该值重置为 0；所有成功放行路径都在持锁状态下写入当前抓包时间。

- [ ] **步骤 4：运行播放器测试验证绿灯**

预期 0.5x、配置边界、暂停/继续、时间跳转和最后放行时间全部通过。

### 任务 3：暂停快照候选选择与落盘

**文件：**
- 创建：`src/debug/PauseSnapshot.h`
- 创建：`src/debug/PauseSnapshot.cpp`
- 创建：`tests/pause_snapshot_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：编写候选选择失败测试**

测试构造目标时间 `100.0`，依次喂入 `99.4、99.8、100.1`：

```cpp
pause_snapshot::Collector collector(0.5);
collector.updateLidar210(cloudA, 99.4);
collector.updateLidar210(cloudB, 99.8);
collector.begin(100.0);
collector.updateLidar210(cloudFuture, 100.1);
const auto data = collector.finish();
check(data.lidar210.valid && std::abs(data.lidar210.timestamp - 99.8) < 1e-6,
      "collector must select the closest frame not later than pause target");
```

分别验证：四路有效时 `complete=true`；缺少右图像时不完整；时间差大于 0.5 秒时该路无效。

- [ ] **步骤 2：运行测试验证红灯**

运行构建 `pause_snapshot_tests`，预期因目标或接口不存在而失败。

- [ ] **步骤 3：实现收集数据类型**

`PauseSnapshot.h` 定义四路帧、完整性问题和收集器：

```cpp
namespace pause_snapshot {
struct CloudFrame { bool valid = false; double timestamp = 0.0; QVector<M_PointXYZI> data; };
struct ImageFrame { bool valid = false; double timestamp = 0.0; QImage data; };
struct Snapshot {
    double target_timestamp = 0.0;
    CloudFrame lidar210;
    CloudFrame lidar211;
    ImageFrame camera_left;
    ImageFrame camera_right;
    QStringList problems;
    bool complete() const { return problems.isEmpty(); }
};

class Collector {
public:
    explicit Collector(double maxDifferenceSec = 0.5);
    void updateLidar210(const QVector<M_PointXYZI> &, double);
    void updateLidar211(const QVector<M_PointXYZI> &, double);
    void updateCameraLeft(const QImage &, double);
    void updateCameraRight(const QImage &, double);
    bool begin(double targetTimestamp);
    Snapshot finish();
};
}
```

收集器始终保存四路最新帧；`begin` 用已有最新帧初始化候选；收集期间只接受 `timestamp <= target` 且时间差更小的帧；`finish` 对 0.5 秒容差和空数据生成明确问题列表。

- [ ] **步骤 4：编写 PCD/PNG 失败测试**

使用 `QTemporaryDir` 保存两个各含 2 个点的点云和两张小型 `QImage`，断言：

- 四个文件存在；
- PCD 包含 `FIELDS x y z intensity`、`SIZE 4 4 4 1`、`TYPE F F F U`、`POINTS 2`、`DATA binary`；
- `DATA binary\n` 后载荷大小为 `2 * 13` 字节；
- 两张 PNG 能被 `QImage` 重新加载且尺寸不变。

- [ ] **步骤 5：实现原子写盘**

定义：

```cpp
struct SaveResult {
    bool complete = false;
    QString base_name;
    QStringList written_files;
    QStringList errors;
};

SaveResult saveSnapshot(
    const Snapshot &snapshot,
    const QString &rootDirectory,
    const QTimeZone &zone = QTimeZone::systemTimeZone());
```

使用 `QDir::mkpath` 创建 `210/211/camera_left/camera_right`；PCD 用 `QSaveFile` 写头和逐点 13 字节 binary 数据；PNG 用 `QImage::save(QIODevice*, "PNG")` 后提交 `QSaveFile`。有效的单路即使集合不完整也允许写盘，但 `SaveResult::complete` 只在四路有效且四文件全部成功时为 true。

- [ ] **步骤 6：运行快照测试验证绿灯**

运行：

```powershell
cmake --build build-radar-slam --target pause_snapshot_tests -j 4
.\build-radar-slam\pause_snapshot_tests.exe
```

预期候选、完整性、PCD 和 PNG 测试全部通过。

### 任务 4：接入主窗口播放器交互

**文件：**
- 修改：`src/gui/MainWindow.h`
- 修改：`src/gui/mainwindow.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：接入本地时间输入**

`onTimestampSeekClicked()` 的默认值改为 `formatLocal(suggested)`；提示同时显示本地范围和 Unix 范围。输入通过 `parseTimestamp` 转为 Unix 秒后继续使用现有范围校验和跳转状态机，不修改 `PcapngReader` 的定位规则。

- [ ] **步骤 2：接入 0.5x 按钮循环**

`onPlaybackSpeedClicked()` 使用：

```cpp
if (m_playbackSpeed < 0.75)
    m_playbackSpeed = 1.0;
else if (m_playbackSpeed < 1.5)
    m_playbackSpeed = 2.0;
else if (m_playbackSpeed < 3.0)
    m_playbackSpeed = 4.0;
else
    m_playbackSpeed = 0.5;
```

按钮文本继续由 `QString::number(..., 'g', 2)` 生成，因此显示 `0.5x`。

- [ ] **步骤 3：把四路完整帧送入收集器**

- `onBridgeCloud210` 在检测开关判断前调用 `updateLidar210`；
- `onBridgeCloud211` 在检测开关判断前调用 `updateLidar211`；
- `onBridgeCameraFrame` 计算帧时间后调用 `updateCameraLeft`；
- `onBridgeCameraFrameRight` 计算帧时间后调用 `updateCameraRight`。

这些调用只做隐式共享拷贝，不进行磁盘 I/O。

- [ ] **步骤 4：暂停时启动一次快照**

当 `onPlaybackPauseClicked()` 从播放切换到暂停时：

```cpp
reader->setPaused(true);
const double target = reader->lastReleasedTimestamp();
if (m_pauseSnapshotCollector.begin(target)) {
    m_pauseSnapshotPending = true;
    playbackPauseBtn->setEnabled(false);
    QTimer::singleShot(300, this, &MainWindow::finishPauseSnapshot);
}
```

从暂停切换到继续时不保存。时间跳转预热和在线模式不调用该流程。

- [ ] **步骤 5：300 ms 后后台写盘**

`finishPauseSnapshot()` 取出 `Snapshot`，恢复暂停按钮可用；创建 `QFutureWatcher<pause_snapshot::SaveResult>`，后台调用：

```cpp
pause_snapshot::saveSnapshot(
    snapshot,
    QDir(QCoreApplication::applicationDirPath()).filePath("pause_snapshots"));
```

完成信号在 GUI 线程记录目标时间、四路实际时间、文件列表与错误。成功时状态栏显示根目录；不完整时列出缺失或超差来源。

- [ ] **步骤 6：构建主程序**

运行：

```powershell
cmake --build build-radar-slam --target PointCloud -j 4
```

预期主程序、Qt MOC、Concurrent、PCD/PNG 写入组件全部编译链接成功。

### 任务 5：完整验证与静态验收

**文件：**
- 验证：`build-radar-slam/PointCloud.exe`
- 验证：全部 CTest 目标

- [ ] **步骤 1：运行全量自动测试**

```powershell
$env:Path='D:\msys64\ucrt64\bin;'+$env:Path
ctest --test-dir build-radar-slam --output-on-failure -j 4
```

预期全部测试 0 失败，测试数比当前 12 个至少增加 1 个 `pause_snapshot`。

- [ ] **步骤 2：静态核对需求边界**

确认：

- USV 本地时间与 Unix 输入均进入同一范围校验；
- 0.5x 未被读取器或配置再次收敛成 1x；
- 暂停快照使用 210/211 单雷达帧而非融合点云；
- 保存的是左右原图，检测叠加层不进入 PNG；
- 磁盘写入只在 QtConcurrent 后台任务执行；
- 点击继续、在线模式和跳转预热不触发快照；
- 检测开关状态不被暂停、继续、倍速或快照改写；
- 构造函数仍未调用 `restoreLastSlamMap()`。

- [ ] **步骤 3：交互验收说明**

记录实际操作步骤：加载 PCAP → 输入 USV 时间跳转 → 切换 0.5x → 开启检测 → 点击暂停 → 等待日志 → 用 PCL/图像查看器打开四个文件 → 继续播放。若当前环境没有用户的实际 PCAP 和图形桌面，只报告自动验证结果，并明确把该步骤交给用户现场验收，不虚构运行结果。

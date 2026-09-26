# 实时 SLAM 增量与离线泊位显示优化实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让 PCAP 回放时本地 SLAM 视图持续显示绿色增量点云，让检测按钮明确显示运行状态，并在加载离线 `.slammap` 时显示固定泊位库中的全部泊位。

**架构：** 保留现有 OctoMap、投递节点和检测算法。UI 在 OctoMap 模式下同时订阅关键帧/实时扫描，将关键帧累积到独立实时层；占据栅格单独缓存，不能覆盖历史地图。固定泊位库复用已有 `StaticBerthLibrary` 解析器，仅转换为本地绘制结果，不进入实时投递或保存内容。

**技术栈：** C++17、Qt/QOpenGL、现有 Topic 总线、现有静态泊位 JSON 解析器、仓库内 C++ 静态回归测试。

---

### 任务 1：先写回归测试，锁定三项行为

**文件：**
- 修改：`tests/slam_map_source_merge_tests.cpp`
- 修改：`tests/detection_control_tests.cpp`
- 修改：`tests/static_berth_library_tests.cpp`

- [x] **步骤 1：编写失败测试**

在 `slam_map_source_merge_tests.cpp` 增加检查：

```cpp
contains(window,
         "m_slamKeyframeTopic = comm.getStateTopic<usv::SlamKeyframe>(\"slam/keyframe\", 32);",
         "OctoMap 模式下 UI 仍需订阅关键帧");
contains(window,
         "m_slamScanTopic = comm.getStateTopic<usv::SlamScanCloudMessage>(\"slam/scan_cloud\", 8);",
         "OctoMap 模式下 UI 仍需订阅实时扫描");
contains(widget,
         "|| !m_slamRealtimeMapCloud.isEmpty()",
         "实时关键帧层必须参与 paintGL 就绪判断");
contains(widget,
         "if (m_slamMapIsOccupancy)\n        return;",
         "关键帧不能再被占据栅格模式直接丢弃");
contains(widget,
         "m_slamOccupancyCloud",
         "占据栅格必须拥有独立缓存，不能覆盖历史点云");
```

在 `detection_control_tests.cpp` 增加检查：

```cpp
check(source.find("m_bridgeDetectionEnabled ? tr(\"停止桥检\") : tr(\"桥梁检测\")")
          != std::string::npos,
      "桥梁按钮文字必须跟随开关状态", failures);
check(source.find("m_berthDetectionEnabled ? tr(\"停止泊检\") : tr(\"泊位检测\")")
          != std::string::npos,
      "泊位按钮文字必须跟随开关状态", failures);
```

在 `static_berth_library_tests.cpp` 增加检查：

```cpp
check(result.display_berths.size() == 2,
      "固定泊位解析结果必须同时提供本地绘制记录", failures);
check(result.display_berths[0].cx == 10.0
          && result.display_berths[0].cy == 20.0,
      "本地绘制泊位必须保留 ENU 中心", failures);
```

- [x] **步骤 2：运行测试确认失败**

运行现有对应测试命令；预期新增断言失败，因为当前 OctoMap 分支排除了 UI 关键帧订阅、占据栅格没有独立缓存、按钮没有停止文字、静态解析器没有绘制记录。

### 任务 2：实现实时 SLAM 增量显示

**文件：**
- 修改：`src/gui/mainwindow.cpp:1136-1190`
- 修改：`src/gui/MultiLidarWidget.h:155-190`
- 修改：`src/gui/MultiLidarWidget.cpp:110-230,310-430,470-520,750-815,1370-1400`

- [x] **步骤 1：移除 OctoMap 对 UI 关键帧/扫描订阅的条件排除**

让 `subscribeSlamResultBus()` 无论是否定义 `ENABLE_OCTOMAP` 都注册现有 `slam/keyframe` 和 `slam/scan_cloud` 两个 UI 订阅；OctoMap 自己的订阅保持不变。

- [x] **步骤 2：让关键帧进入独立实时层**

删除 `appendSlamKeyframe()` 中直接返回的占据栅格门控，并把目标层选择改成：已加载历史地图或当前为占据栅格模式时写入 `m_slamRealtimeMapCloud`，否则沿用 `m_slamMapCloud`。

- [x] **步骤 3：隔离占据栅格缓存**

新增 `QVector<M_PointXYZI> m_slamOccupancyCloud`；清空地图、加载新地图时清空它；`updateSlamOccupancyGrid()` 只更新该缓存和线框，不清空实时关键帧/扫描，不重置 `m_hasHistoricalSlamMap`；`rebuildOccupancyWireframe()` 从占据缓存读取数据。

- [x] **步骤 4：修正绘制就绪条件和历史层绘制**

把 `m_slamRealtimeMapCloud` 加入 `paintGL()` 的就绪判断；保留历史点云、实时关键帧、当前扫描的三层绘制，历史地图加载后不被后续占据栅格回调覆盖。

- [x] **步骤 5：运行增量显示回归测试**

运行 `slam_map_source_merge_tests`；预期新增静态断言全部通过。

### 任务 3：实现检测按钮状态文字

**文件：**
- 修改：`src/gui/mainwindow.cpp:1079-1105,1535-1575`

- [x] **步骤 1：在统一同步函数中设置文字**

`syncPathPlanButton()` 在同步勾选状态的同时设置：

```cpp
pathPlanBtn->setText(m_bridgeDetectionEnabled
    ? tr("停止桥检") : tr("桥梁检测"));
berthDetectBtn->setText(m_berthDetectionEnabled
    ? tr("停止泊检") : tr("泊位检测"));
```

- [x] **步骤 2：保持点击和管线停止都经过统一同步**

开启、关闭按钮以及 `stopAlgorithmPipeline()` 继续调用 `syncPathPlanButton()`，不改动节点启停逻辑。

- [ ] **步骤 3：运行按钮回归测试**（当前构建中的既有检测控制断言仍有基线失败）

运行 `detection_control_tests`；预期新增文字断言通过。

### 任务 4：加载离线地图时显示全部固定泊位

**文件：**
- 修改：`src/bus/StaticBerthLibrary.h:8-20`
- 修改：`src/bus/StaticBerthLibrary.cpp:18-245`
- 修改：`src/gui/MultiLidarWidget.h:60-190`
- 修改：`src/gui/MultiLidarWidget.cpp:1-15,110-230,390-410,790-815`
- 修改：`tests/static_berth_library_tests.cpp`

- [x] **步骤 1：让已有 JSON 解析器同时输出绘制记录**

在 `LoadResult` 增加 `std::vector<usv::Berth> display_berths`；解析每条最终记录时保留 ENU 中心、宽长、角度、类型和开口边，按现有 5 米去重结果同步输出。

- [x] **步骤 2：在控件中加载固定泊位覆盖层**

增加 `m_slamFixedBerths` 和一个私有加载函数，按投递端相同的候选路径顺序查找 `泊位检测完整地理信息_20260901.json`：配置路径、程序目录、当前目录。仅在加载 `.slammap` 时读取；找不到时继续沿用 archive 内泊位。

- [x] **步骤 3：绘制固定层并保留实时检测层**

把固定泊位转换为 `BerthMeasureResult` 后先绘制，再绘制同步后的实时泊位；固定层仅用于界面绘制，不写入 `m_slamMapBerthStore`、`m_slamRealtimeBerthStore`，不改变保存文件和 UDP 投递内容。

- [ ] **步骤 4：运行静态泊位测试**（当前 Windows 临时文件测试夹具无法被重复打开）

运行 `static_berth_library_tests`；预期 JSON 解析和绘制记录断言通过。

### 任务 5：收尾验证

**文件：**
- 修改：无（除测试与上述实现文件）

- [ ] **步骤 1：执行全部相关测试**（既有 `detection_control_tests` 与 `slam_berth_overlay_tests` 尚有基线失败）

运行 `detection_control_tests`、`slam_map_source_merge_tests`、`static_berth_library_tests` 及现有 `slam_berth_overlay_tests`。

- [x] **步骤 2：执行格式和差异检查**

运行 `git diff --check`，确认只有本计划涉及的 UI/静态泊位/测试文件发生变化。

- [x] **步骤 3：记录完整构建状态**

尝试当前工程的 PointCloud 构建；若命中已有 GCC8 `std::filesystem` 兼容错误，仅记录该外部阻塞，不修改桥梁模块。

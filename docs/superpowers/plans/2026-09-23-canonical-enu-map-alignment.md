# 固定 ENU 离线地图坐标对齐实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让 `Mergedclouds_enu_optimized.slammap` 作为固定 ENU 离线地图时，PCAP 产生的实时点云和位姿只经过一次固定 `map_to_enu` 变换后与其对齐。

**架构：** 在实时坐标变换辅助头文件中提供固定矩阵；`MultiLidarWidget::loadSlamMap` 仅识别该 canonical 文件名并预置实时对齐状态。现有实时关键帧、里程计、扫描变换路径复用已有状态，历史点云与泊位不变，其他地图继续走原有校正逻辑。

**技术栈：** C++17、Qt、Eigen、现有 SLAM 地图归档与单元测试。

---

### 任务 1：为固定矩阵添加失败回归测试

**文件：**
- 修改：`tests/slam_realtime_map_alignment_tests.cpp`
- 修改：`src/slam/SlamRealtimeMapAlignment.h`

- [x] **步骤 1：编写失败测试**

在测试中调用待新增的 `canonicalMapToEnu()`，断言其平移和旋转元素等于老师提供的 `georeference(1).json` 矩阵，并用一个已知点断言变换结果。

- [x] **步骤 2：运行测试确认失败**

运行：

```powershell
D:\mingw64\bin\g++.exe -std=c++17 -I. -Icommon -Isrc/slam -Isrc/bridge -Ieigen-3.4.1 tests/slam_realtime_map_alignment_tests.cpp -o tests/slam_realtime_map_alignment_tests.exe
```

预期：因 `canonicalMapToEnu()` 尚不存在而失败。

- [x] **步骤 3：实现最少辅助函数**

在 `SlamRealtimeMapAlignment.h` 中增加返回固定 `Eigen::Isometry3d` 的纯内联函数，不改变现有 `computeMapToEnu`、`transformKeyframe` 和 `transformPose` 的行为。

- [x] **步骤 4：运行测试确认通过**

重新编译并运行：

```powershell
tests\slam_realtime_map_alignment_tests.exe
```

预期：进程退出码为 0。

### 任务 2：加载 canonical 地图时启用固定实时对齐

**文件：**
- 修改：`src/gui/MultiLidarWidget.cpp`

- [x] **步骤 1：添加 canonical 文件名判断**

在 `.slammap` 加载分支中读取 `QFileInfo(filePath).fileName()`，只对 `Mergedclouds_enu_optimized.slammap`（大小写不敏感）启用固定矩阵，并跳过普通地图的地理校正。

- [x] **步骤 2：保持历史地图原样**

canonical 分支直接使用归档读取结果，将 `m_historicalMapUsesEnu` 设为 `true`，不修改 `loaded`、`archive.berths` 或历史关键帧点云；其他地图继续调用 `slam_map_geo_correct::correct`。

- [x] **步骤 3：预置一次实时变换**

canonical 分支将 `m_realtimeMapToEnu = slam_realtime_alignment::canonicalMapToEnu()` 并将 `m_hasRealtimeMapAlignment = true`；普通地图仍在函数末尾清零并由首个有效参考捕获。

- [x] **步骤 4：添加一条诊断日志**

在检测信息中输出 canonical ENU 对齐已启用，便于确认实际加载的是固定地图；不改 UDP 消息格式。

### 任务 3：验证既有实时路径未被重复变换

**文件：**
- 复核：`src/gui/MultiLidarWidget.cpp`
- 复核：`src/slam/SlamRealtimeMapAlignment.h`

- [x] **步骤 1：检查关键帧路径**

确认 `appendSlamKeyframe` 只变换 pose，局部 cloud 仍由 `mergeKeyframeIntoMap` 使用该 pose 变换一次。

- [x] **步骤 2：检查里程计和扫描路径**

确认 `updateSlamOdometry`、`updateSlamLiveScan` 仅在 `m_historicalMapUsesEnu && m_hasRealtimeMapAlignment` 时应用一次固定矩阵；普通地图行为不变。

- [x] **步骤 3：运行回归验证**

运行坐标测试、`git diff --check`，并检查变更只涉及设计文档、测试、实时对齐辅助函数和地图加载状态。

### 任务 4：记录变更并准备交付

- [x] **步骤 1：查看差异**

运行：

```powershell
git diff --stat
git diff --check
git status --short
```

- [x] **步骤 2：提交最小变更**

```powershell
git add docs/superpowers/specs/2026-09-23-canonical-enu-map-alignment-design.md docs/superpowers/plans/2026-09-23-canonical-enu-map-alignment.md src/slam/SlamRealtimeMapAlignment.h src/gui/MultiLidarWidget.cpp tests/slam_realtime_map_alignment_tests.cpp
git commit -m "fix: align realtime map to canonical ENU offline map"
```

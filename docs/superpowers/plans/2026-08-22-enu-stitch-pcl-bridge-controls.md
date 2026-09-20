# ENU 地图拼接与近点点云桥梁检测融合实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将近点融合点云桥梁检测安全合并到当前工程，拆分桥梁/泊位按钮，并让地图默认按统一 ENU 地理锚点直接拼接。

**架构：** 地图拼接保留现有 WGS84→ECEF→ENU 换算，但默认只施加锚点平移并按时间裁剪重复关键帧。检测侧以两个独立原子开关门控输入；传统视觉与新 PCL 点云检测共用桥梁开关，泊位 Topic 单独受泊位开关控制，检测节点首次启动后保持运行到程序退出。

**技术栈：** C++17、Qt 5 Widgets/Concurrent/OpenGL、PCL 1.15、Eigen、CMake/CTest、MSYS2 UCRT64。

---

## 文件结构

- 创建 `src/modules/pcl_cluster_preview.h/.cpp`：近点点云聚类、桥梁/桥墩/障碍物识别与跟踪。
- 修改 `src/gui/MultiLidarWidget.h/.cpp`：保存、清空并用现有 overlay VBO 绘制 PCL 语义结果。
- 修改 `src/gui/mainwindow.h/.cpp`：两个按钮、独立门控、后台单任务调度、状态重置与日志。
- 修改 `src/bus/SensorPublisherNode.h/.cpp`：以独立泊位/SLAM开关发布融合点云（维持现有接口语义）。
- 修改 `src/slam/SlamMapStitcher.h/.cpp`：增加 ENU 直接拼接默认模式并保留诊断信息。
- 修改 `CMakeLists.txt`：PCL 感知选项、源文件、宏、链接和测试目标。
- 修改 `tests/slam_map_stitching_tests.cpp`：ENU 直接拼接及重叠裁剪回归测试。
- 创建 `tests/detection_control_tests.cpp`：桥梁/泊位按钮和数据门控静态集成测试。
- 创建 `tests/pcl_cluster_preview_tests.cpp`：船体范围、障碍物距离和空输入基本算法测试。

### 任务 1：锁定 ENU 直接拼接行为

**文件：**
- 修改：`tests/slam_map_stitching_tests.cpp`
- 修改：`src/slam/SlamMapStitcher.h`
- 修改：`src/slam/SlamMapStitcher.cpp`

- [ ] **步骤 1：编写失败的测试**

增加测试：默认 `Options{}` 的 `enable_icp` 和 `enable_temporal_seam` 为 false；部分时间重叠时结果裁剪重复帧，但追加帧姿态只包含 `anchorOffsetEnu`，`correction_translation_m` 与 `correction_yaw_deg` 为零。

- [ ] **步骤 2：运行测试验证失败**

运行：

```powershell
& 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\slam_map_stitching_tests.exe'
```

预期：新增断言失败，因为当前默认开启时间接缝和 ICP。

- [ ] **步骤 3：实现最小修复**

将默认选项改为 ENU 直接拼接；时间部分重叠时仍计算 `trimmed_keyframes`，直接用锚点变换追加 `targetTime.last` 之后的关键帧；日志明确“ENU 锚点直接拼接，未施加二次刚体修正”。

- [ ] **步骤 4：重新构建并验证测试通过**

```powershell
cmake --build 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' --target slam_map_stitching_tests -j 2
& 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\slam_map_stitching_tests.exe'
```

预期：输出 `All SLAM map stitching tests passed`。

### 任务 2：锁定独立检测按钮与门控

**文件：**
- 创建：`tests/detection_control_tests.cpp`
- 修改：`CMakeLists.txt`
- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`

- [ ] **步骤 1：编写失败的静态集成测试**

读取 `mainwindow.h/.cpp` 并断言存在 `bridgeDetectBtn`、`berthDetectBtn`、`m_bridgeDetectionEnabled`、`m_berthDetectionEnabled`；断言桥梁相机提交只受桥梁开关控制，`publishFusedSensors` 的泊位参数只受泊位开关控制，且不再存在 `m_perceptionDetectEnabled`。

- [ ] **步骤 2：构建并验证测试失败**

```powershell
cmake --build 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' --target detection_control_tests -j 2
& 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\detection_control_tests.exe'
```

预期：按钮和独立状态尚不存在，测试失败。

- [ ] **步骤 3：实现按钮和生命周期**

将旧按钮重命名为“桥梁检测”，新增“泊位检测”；实现 `onBridgeDetectionClicked()`、`onBerthDetectionClicked()`、`ensureAlgorithmPipelineStarted()`、`syncDetectionButtons()`。首次开启任一功能时启动总线节点；关闭按钮只门控输入和清层，不停止/取消订阅节点。

- [ ] **步骤 4：验证静态集成测试通过**

重新构建并运行 `detection_control_tests.exe`，预期退出码 0。

### 任务 3：合并近点点云算法

**文件：**
- 创建：`src/modules/pcl_cluster_preview.h`
- 创建：`src/modules/pcl_cluster_preview.cpp`
- 创建：`tests/pcl_cluster_preview_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：先添加算法测试目标和失败测试**

测试空输入返回空结果；合成船体外矩形簇得到非负净距离；船体内点不生成障碍物；配置默认值为 `shipHalfExtentX=6.0f`、`shipHalfExtentY=2.5f`。

- [ ] **步骤 2：验证目标因算法文件缺失或断言不符而失败**

```powershell
cmake --build 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' --target pcl_cluster_preview_tests -j 2
```

预期：目标尚不可构建或默认船体范围断言失败。

- [ ] **步骤 3：语义合并候选算法**

复制算法实现后仅做当前工程必需修正：船体 X/Y 半尺寸、时间轴重置接口、有限值/配置防护；保持同门给出的聚类、桥梁、桥墩、障碍物和时序阈值不变。

- [ ] **步骤 4：验证算法测试通过**

构建并运行 `pcl_cluster_preview_tests.exe`，预期所有断言通过。

### 任务 4：接入后台调度与 OpenGL 语义层

**文件：**
- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`
- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`

- [ ] **步骤 1：扩展失败测试**

在 `detection_control_tests.cpp` 断言融合点云 PCL 调度受桥梁开关控制、使用 busy-drop 原子状态、关闭和 seek 时清除并重置 PCL；断言 `MultiLidarWidget` 通过 `m_overlayVbo` 绘制而不是 `glBegin`。

- [ ] **步骤 2：运行测试确认失败**

构建并运行 `detection_control_tests.exe`，预期新增调度/渲染断言失败。

- [ ] **步骤 3：实现安全单任务调度**

在融合线程用原子 busy 标志抢占任务；深拷贝当前帧后交给 `QtConcurrent::run`；后台持久化算法只串行访问；结果通过 queued invoke 回 GUI。使用代数 generation 丢弃开关关闭或 seek 前的迟到结果。

- [ ] **步骤 4：实现 VBO 渲染**

为 `MultiLidarWidget` 增加 `updatePclClusterPreview`、`clearPclClusterPreview` 和 `drawPclClusterPreview`；桥梁强度 130（红）、桥墩强度 80（绿），障碍物边界用橙色统一颜色。

- [ ] **步骤 5：验证调度与渲染测试通过**

重新构建并运行 `detection_control_tests.exe`，预期退出码 0。

### 任务 5：完整构建与真实数据诊断

**文件：**
- 修改：`CMakeLists.txt`
- 复核：上述全部源文件

- [ ] **步骤 1：重新配置 Debug 构建**

```powershell
cmake -S 'D:\1\yuanshibanben\PointCloud20260514\PointCloud' -B 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' -DENABLE_SLAM=ON -DENABLE_PCL_PERCEPTION=ON -DBUILD_TESTING=ON
```

预期：检测到 PCL 1.15，配置退出码 0。

- [ ] **步骤 2：运行完整测试套件**

```powershell
ctest --test-dir 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' --output-on-failure
```

预期：全部测试通过，0 failures。

- [ ] **步骤 3：完整构建主程序**

```powershell
cmake --build 'D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug' --target PointCloud -j 2
```

预期：`PointCloud.exe` 链接成功，退出码 0。

- [ ] **步骤 4：诊断三张现有地图**

运行 `slam_map_stitching_tests.exe --diagnose-real-maps` 加三张 maps 目录文件；预期第二张地图模式为 ENU 锚点裁剪，额外 correction 平移/偏航均为 0。

- [ ] **步骤 5：压缩包 PCAP 人工验收**

加载 `qiao2.pcap`，分别验证：桥梁开/泊位关、桥梁关/泊位开、两者同时开、两者同时关；检查视频持续、点云持续、检测层按职责出现/清除、播放暂停和倍速不失效。

### 任务 6：自审与交付记录

**文件：**
- 复核：全部变更
- 更新：`docs/superpowers/plans/2026-08-22-enu-stitch-pcl-bridge-controls.md`

- [ ] **步骤 1：逐项核对规格**

对照设计文档确认 ENU、两个按钮、传统视觉、新 PCL、泊位、seek 重置、后台不积压和构建选项均有实现与验证证据。

- [ ] **步骤 2：检查工作区差异**

工程非 Git 仓库，使用文件哈希、时间戳与针对性 `rg` 检查确认未覆盖候选旧版文件、未改动无关算法参数。

- [ ] **步骤 3：重新运行最终验证命令**

最终再次运行完整 `ctest`、主程序构建和真实地图诊断；只依据本次输出报告结果。

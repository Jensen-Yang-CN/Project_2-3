# SLAM 地图实时泊位框叠加实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将现有实时泊位检测结果按时间戳转换到 SLAM map 坐标，并在 SLAM 地图模式实时绘制，不永久累计或保存。

**架构：** 抽取纯泊位几何变换工具，新增有界的泊位/SLAM 位姿同步器，在 `MultiLidarWidget` 中分别渲染局部结果与世界结果。算法仍只运行一次，SLAM 模式只增加坐标投影和绘制。

**技术栈：** C++17、Qt 5、Eigen、OpenGL、CMake、CTest

---

## 文件结构

- 创建：`src/bus/BerthGeometryTransform.h`——纯泊位/结果几何变换。
- 修改：`src/bus/RealtimeBerthCoordinateLibrary.h`——复用统一几何变换。
- 创建：`src/gui/SlamBerthOverlaySynchronizer.h`——有界时间戳同步和世界结果缓存。
- 修改：`src/gui/MultiLidarWidget.h`——保存同步器并调整绘制接口。
- 修改：`src/gui/MultiLidarWidget.cpp`——接收位姿、投影结果并在 SLAM 模式绘制。
- 创建：`tests/slam_berth_overlay_tests.cpp`——坐标变换、异步到达、过期和清理测试。
- 修改：`CMakeLists.txt`——登记新头文件与测试目标。

### 任务 1：建立几何变换和时间同步红灯测试

**文件：**
- 创建：`tests/slam_berth_overlay_tests.cpp`
- 修改：`CMakeLists.txt`

- [x] **步骤 1：编写 90 度旋转和平移测试**

```cpp
Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
pose.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
pose.translation() = Eigen::Vector3d(10.0, 20.0, 0.0);
const usv::BerthMeasureResult world =
    usv::berth_geometry::transformResult(local, pose);
check(close(world.expanded_berths[0].cx, 8.0)
      && close(world.expanded_berths[0].cy, 21.0),
      "berth center must be transformed into SLAM map coordinates");
```

- [x] **步骤 2：编写开口边、实体边、锚点和来源保持测试**

测试断言 `visible_edges` 与锚点完成同一变换，`opening_edge` 物理含义保持，`BerthDetectionSource::CoordinateLibrary` 不被改写。

- [x] **步骤 3：编写异步到达和过期测试**

```cpp
SlamBerthOverlaySynchronizer sync;
sync.addBerthResult(local);
check(!sync.worldResult().has_value(), "result must wait for a matching pose");
sync.addSlamPose(local.timestamp, pose);
check(sync.worldResult().has_value(), "a later SLAM pose must project the pending result");
check(!sync.visibleAt(local.timestamp + 8.01), "stale overlay must be hidden");
```

- [x] **步骤 4：编写空结果和 reset 清理测试**

空结果必须立即使 `worldResult()` 为空；`resetAll()` 后位姿与结果历史均不可继续产生旧投影。

- [x] **步骤 5：构建并验证红灯**

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake -S D:\1\yuanshibanben\PointCloud20260514\PointCloud -B D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_berth_overlay_tests -j 2
```

预期：因 `BerthGeometryTransform.h` 或 `SlamBerthOverlaySynchronizer.h` 尚不存在而构建失败。

### 任务 2：实现统一几何变换

**文件：**
- 创建：`src/bus/BerthGeometryTransform.h`
- 修改：`src/bus/RealtimeBerthCoordinateLibrary.h`

- [x] **步骤 1：实现基础几何转换**

提供以下接口：

```cpp
DebugLine2D transformLine(const DebugLine2D &, const Eigen::Isometry3d &);
Berth transformBerth(const Berth &, const Eigen::Isometry3d &,
                     BerthDetectionSource);
BerthMeasureResult transformResult(const BerthMeasureResult &,
                                   const Eigen::Isometry3d &);
```

- [x] **步骤 2：完整转换结果字段**

`transformResult` 转换 `expanded_berths`、单个/数组锚点、锚点总线、调试线、`debug_roi/debug_rois` 和 `top_search_roi/top_search_rois`，同时保持时间戳、检测状态及每个泊位的来源。

- [x] **步骤 3：让坐标库复用新工具**

`RealtimeBerthCoordinateLibrary::transformGeometry` 和 `transformDebugLine` 调用 `berth_geometry`，删除原文件中的重复变换实现和边元数据重排函数。

### 任务 3：实现时间同步器

**文件：**
- 创建：`src/gui/SlamBerthOverlaySynchronizer.h`

- [x] **步骤 1：实现有界历史与最近时间匹配**

```cpp
void addSlamPose(double timestamp, const Eigen::Isometry3d &world_from_lidar);
void addBerthResult(const usv::BerthMeasureResult &result);
std::optional<usv::BerthMeasureResult> worldResult() const;
bool visibleAt(double slamTimestamp) const;
void clearResults();
void resetAll();
```

- [x] **步骤 2：实现双向到达顺序**

泊位后到时搜索位姿历史；位姿后到时搜索泊位历史。只接受 `abs(pose_ts-result_ts) <= 0.25` 的配对，并只允许时间更新的匹配覆盖当前世界结果。

- [x] **步骤 3：实现实时清理语义**

空结果立即清除泊位历史和世界结果；`clearResults()` 保留位姿历史供检测重新开启；`resetAll()` 清除全部时间线状态；`visibleAt()` 使用 8 秒过期阈值。

- [x] **步骤 4：构建并运行专项测试验证绿灯**

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_berth_overlay_tests berth_recall_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "slam_berth_overlay|berth_recall" --output-on-failure
```

预期：2/2 通过。

### 任务 4：接入 SLAM 渲染窗口

**文件：**
- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`

- [x] **步骤 1：接入结果和位姿**

`updateBerthResult()` 同时把结果交给同步器；`updateSlamOdometry()` 在更新船体显示位姿前，将 `state.timestamp` 和原始 `state.pose_lidar.pose` 交给同步器。

- [x] **步骤 2：分离局部与 SLAM 绘制结果**

把绘制接口改为：

```cpp
void drawBerthOverlays(const QMatrix4x4 &viewMatrix,
                       const usv::BerthMeasureResult &result,
                       bool useEdgeHeights);
```

普通模式传入 `m_berthResult,true`；SLAM 模式传入同步器世界结果并使用 `false`。

- [x] **步骤 3：实现清理与过期控制**

`clearBerthResult()` 调用 `clearResults()`；`clearSlamMap()` 调用 `resetAll()`；SLAM 模式仅在 `visibleAt(latestSlamTimestamp)` 为真时绘制。

- [x] **步骤 4：保留颜色和 U 型开口**

绘制仍调用 `berthOverlayColor()` 和 `berthOutlineMode()`；继续不绘制 `opening_segment`。

### 任务 5：全量验证与交付

**文件：**
- 验证：`CMakeLists.txt`
- 验证：`D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\PointCloud.exe`

- [x] **步骤 1：运行专项与全量测试**

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

预期：全部测试通过。

- [x] **步骤 2：构建主程序**

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud -j 2
```

预期：退出码 0，生成更新后的 `PointCloud.exe`。

- [x] **步骤 3：静态验收数据流**

确认检测结果仍只由 `BerthDetectionNode` 计算一次；点云模式使用局部结果，SLAM 模式使用 `T_map_lidar` 投影结果；地图保存代码未加入泊位框。

## 执行说明

当前目录没有 `.git`，不能创建专用 worktree 或提交；用户已明确要求在当前项目中直接修改，因此所有变更在当前工作区内联执行。

## 执行结果

- 2026-08-10：先运行新增集成守卫，确认旧实现因 SLAM 模式未接线而出现 5 项预期失败。
- 2026-08-10：完成坐标变换、时间同步、生命周期清理及双模式绘制接入后，专项测试通过。
- 2026-08-10：`PointCloud` 主程序构建成功，全量 CTest 16/16 通过。

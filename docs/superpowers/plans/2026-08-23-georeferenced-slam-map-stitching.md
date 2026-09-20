# 基于逐关键帧地理约束的 SLAM 地图拼接实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让新生成的 `.slammap` 保存逐关键帧 GNSS/ENU 参考，加载时平滑消除独立 SLAM 累计漂移，并在安全小范围配准后对显示点云做三维体素去重。

**架构：** 实时 SLAM 继续生成局部连续位姿，同时把已同步的导航雷达位姿附加到关键帧。`SlamMapGeoCorrector` 在加载阶段根据多点参考平滑修正地图，`SlamMapStitcher` 只做 3 米以内的会话级残差修正，`SlamMapDisplayBuilder` 最后执行 0.3 米三维首点保留去重。

**技术栈：** C++17、Qt 5、Eigen、PCL、CMake、现有无框架可执行测试。

---

## 文件结构

- 修改 `common/slam_types.h`：定义 `SlamGeoPoseReference` 并附加到 `SlamKeyframe`。
- 修改 `src/code-slam/LidarGnssSlam.h/.cpp`：保存并提供最近已处理雷达帧的 GNSS/INS 参考位姿。
- 修改 `src/slam/DloSlamNode.h/.cpp`：创建关键帧时写入同步地理参考。
- 修改 `src/core/SlamMapBinaryIO.h/.cpp`：写 v2.1，兼容读取 v2.0。
- 创建 `src/slam/SlamMapGeoCorrector.h/.cpp`：负责鲁棒逐关键帧漂移校正。
- 修改 `src/slam/SlamMapLoadTask.h/.cpp`：每张输入地图先校正，再拼接，并汇总校正诊断。
- 修改 `src/slam/SlamMapStitcher.h/.cpp`：默认配准范围收紧到 3 米，保留质量门控。
- 修改 `src/slam/SlamMapDisplayBuilder.h/.cpp`：0.3 米 XYZ 体素首点保留去重并返回统计。
- 修改 `src/gui/MultiLidarWidget.h/.cpp`：接收显示构建统计。
- 修改 `src/gui/mainwindow.cpp`：输出格式、地理校正、配准和去重日志。
- 修改 `tests/slam_map_io_tests.cpp`：v2.1 往返、v2.0 兼容、未来次版本拒绝。
- 修改 `tests/slam_pose_tests.cpp`：导航参考有效性与时间戳规则。
- 修改 `tests/slam_map_stitching_tests.cpp`：漂移校正、离群点、受限配准和三维去重。
- 修改 `CMakeLists.txt`：将新组件加入主程序和拼接测试目标。

### 任务 1：v2.1 数据类型与文件格式

**文件：**
- 修改：`common/slam_types.h`
- 修改：`src/core/SlamMapBinaryIO.h`
- 修改：`src/core/SlamMapBinaryIO.cpp`
- 测试：`tests/slam_map_io_tests.cpp`

- [ ] **步骤 1：编写 v2.1 往返失败测试**

给测试关键帧附加有效参考，并断言保存重载后字段一致：

```cpp
source.keyframes[0].geo_reference.valid = true;
source.keyframes[0].geo_reference.timestamp = 12.5;
source.keyframes[0].geo_reference.sync_error_sec = 0.02;
source.keyframes[0].geo_reference.pose_lidar_enu.translation() =
    Eigen::Vector3d(4.0, -3.0, 1.0);
check(loaded.keyframes[0].geo_reference.valid
          && close(loaded.keyframes[0].geo_reference.sync_error_sec, 0.02)
          && close(loaded.keyframes[0].geo_reference.pose_lidar_enu.translation().x(), 4.0),
      "v2.1 keyframe geographic reference must round-trip", failures);
```

- [ ] **步骤 2：增加 v2.0 固定样本和未来次版本测试**

测试内用 `QDataStream` 写一个最小 v2.0 文件，断言仍能加载且 `geo_reference.valid == false`；将次版本改为 `2`，断言加载失败且错误包含“不支持的地图次版本”。

- [ ] **步骤 3：运行红灯测试**

运行：

```powershell
cmake --build . --target slam_map_io_tests -j 2
.\slam_map_io_tests.exe
```

预期：v2.1 地理参考字段不存在或未往返，测试失败。

- [ ] **步骤 4：实现最小 v2.1 格式**

新增：

```cpp
struct SlamGeoPoseReference {
    bool valid = false;
    double timestamp = 0.0;
    double sync_error_sec = 0.0;
    Eigen::Isometry3d pose_lidar_enu = Eigen::Isometry3d::Identity();
};
```

设置 `kFormatMinor = 1`；加载时 `minor == 0` 使用旧布局，`minor == 1` 读取固定地理参考记录，`minor > 1` 明确拒绝。

- [ ] **步骤 5：运行绿灯测试**

运行相同命令，预期输出 `All SLAM map I/O tests passed`。

### 任务 2：关键帧同步导航参考采集

**文件：**
- 修改：`src/code-slam/LidarGnssSlam.h`
- 修改：`src/code-slam/LidarGnssSlam.cpp`
- 修改：`src/slam/DloSlamNode.h`
- 修改：`src/slam/DloSlamNode.cpp`
- 修改：`src/slam/SlamPoseUtils.h`
- 测试：`tests/slam_pose_tests.cpp`

- [ ] **步骤 1：编写参考匹配规则失败测试**

在 `SlamPoseUtils.h` 期望一个纯函数：

```cpp
check(usv::slam_pose::navigationReferenceMatches(
          reference, 100.0, 0.25),
      "matching finite navigation reference must be accepted", failures);
reference.sync_error_sec = 0.3;
check(!usv::slam_pose::navigationReferenceMatches(
          reference, 100.0, 0.25),
      "stale navigation reference must be rejected", failures);
```

- [ ] **步骤 2：运行红灯测试**

运行：

```powershell
cmake --build . --target slam_pose_tests -j 2
.\slam_pose_tests.exe
```

预期：`navigationReferenceMatches` 不存在导致构建失败。

- [ ] **步骤 3：记录最近已处理导航参考**

`LidarGnssSlam` 在 `processLidarFrame()` 成功匹配 `matched_odom` 时保存：

```cpp
SlamGeoPoseReference reference;
reference.valid = has_odom;
reference.timestamp = lidar.timestamp;
reference.sync_error_sec = nearest_source_time_error;
reference.pose_lidar_enu = matched_odom.pose;
```

提供线程安全的 `getLastNavigationReference()`。`DloSlamNode::createKeyframe()` 只在参考通过时间戳、同步误差和有限数值校验时写入关键帧。

- [ ] **步骤 4：运行绿灯测试并构建主对象**

运行：

```powershell
.\slam_pose_tests.exe
cmake --build . --target PointCloud -j 2
```

预期：测试通过，主程序编译通过。

### 任务 3：逐关键帧地理漂移校正

**文件：**
- 创建：`src/slam/SlamMapGeoCorrector.h`
- 创建：`src/slam/SlamMapGeoCorrector.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：编写线性漂移失败测试**

构造 10 个关键帧，SLAM 的 X 从真实值逐步漂移到 `+2 m`，地理参考保持真实值：

```cpp
const slam_map_geo_correct::Result corrected =
    slam_map_geo_correct::correct(archive);
check(corrected.applied, "three or more geo references should correct drift", failures);
check((corrected.archive.keyframes.back().pose.translation()
       - corrected.archive.keyframes.back().geo_reference.pose_lidar_enu.translation()).norm()
          < 0.2,
      "linear SLAM drift should be reduced below 0.2 m", failures);
```

- [ ] **步骤 2：编写 GNSS 跳点和参考不足失败测试**

一个中间参考加入 `+30 m` 跳点，断言其被拒绝且相邻关键帧保持连续；只有两个有效参考时断言 `applied == false` 且位姿不变。

- [ ] **步骤 3：运行红灯测试**

运行：

```powershell
cmake --build . --target slam_map_stitching_tests -j 2
.\slam_map_stitching_tests.exe
```

预期：新校正器头文件或 API 不存在，构建失败。

- [ ] **步骤 4：实现鲁棒校正器**

接口：

```cpp
namespace slam_map_geo_correct {
struct Diagnostic {
    bool applied = false;
    int valid_references = 0;
    int rejected_references = 0;
    double median_translation_m = 0.0;
    double max_translation_m = 0.0;
    double max_yaw_deg = 0.0;
    QString message;
};
struct Result {
    slam_map_io::MapArchive archive;
    Diagnostic diagnostic;
};
Result correct(const slam_map_io::MapArchive &archive);
}
```

实现 SE(2) 残差、5 点滑动中值、5 米/5 度离群门限、时间插值和端点保持。

- [ ] **步骤 5：运行绿灯测试**

预期所有拼接测试通过。

### 任务 4：加载链路集成与安全小范围配准

**文件：**
- 修改：`src/slam/SlamMapLoadTask.h`
- 修改：`src/slam/SlamMapLoadTask.cpp`
- 修改：`src/slam/SlamMapStitcher.h`
- 修改：`src/slam/SlamMapStitcher.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`

- [ ] **步骤 1：编写加载先校正失败测试**

保存带合成漂移的两张 v2.1 地图，经 `loadAndStitch()` 后断言输出关键帧已接近地理参考，并且结果中包含每张地图的校正诊断。

- [ ] **步骤 2：编写 3 米边界失败测试**

构造重复平行结构，使最佳候选超过 3 米，断言模式回退为 `AnchorOnly`，修正平移为零；构造 1 米真实偏置且质量明显改善的地图，断言可接受。

- [ ] **步骤 3：运行红灯测试**

运行拼接测试，预期加载结果缺少校正诊断或默认范围仍为 20 米而失败。

- [ ] **步骤 4：集成校正并收紧默认选项**

在 `loadAndStitch()` 中对每个 archive 调用 `slam_map_geo_correct::correct()` 后再创建 `slam_map_stitch::Input`。调整默认值：

```cpp
double initial_search_translation_step_m = 1.0;
double max_correction_translation_m = 3.0;
double correction_boundary_margin_m = 0.5;
```

继续使用仅平移、双向匹配率、鲁棒 RMSE 和相对改善门控。

- [ ] **步骤 5：运行绿灯测试和真实旧地图诊断**

运行：

```powershell
.\slam_map_stitching_tests.exe
.\slam_map_stitching_tests.exe --diagnose-real-maps .\maps\20260810_193724_167.slammap .\maps\20260810_194048_767.slammap
```

预期：自动化测试通过；旧地图加载成功且不接受超过 3 米的修正。

### 任务 5：显示点云三维体素去重

**文件：**
- 修改：`src/slam/SlamMapDisplayBuilder.h`
- 修改：`src/slam/SlamMapDisplayBuilder.cpp`
- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`
- 测试：`tests/slam_map_stitching_tests.cpp`

- [ ] **步骤 1：编写三维去重失败测试**

构造两个来源：同一 XYZ 体素内各有一点，另有两个 XY 相同但 Z 相差 1 米的点：

```cpp
slam_map_display::BuildStats stats;
const QVector<M_PointXYZI> display = slam_map_display::rebuild(
    archive, ranges, 100, 0.3, &stats);
check(stats.duplicate_points_removed == 1,
      "same 3D voxel should keep the first source point", failures);
check(display.size() == 3,
      "points at different heights must survive XYZ deduplication", failures);
```

- [ ] **步骤 2：运行红灯测试**

预期：新函数参数或统计结构不存在，构建失败。

- [ ] **步骤 3：实现确定性首点保留**

新增 `BuildStats`，用 `(floor(x/0.3), floor(y/0.3), floor(z/0.3))` 三维整数键去重。按关键帧和来源原顺序遍历，重复体素跳过；不修改 archive。

- [ ] **步骤 4：运行绿灯测试**

预期拼接与显示测试全部通过。

### 任务 6：界面诊断、回归与交付构建

**文件：**
- 修改：`src/gui/mainwindow.cpp`
- 修改：`src/gui/MultiLidarWidget.h/.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：增加源码接线回归断言**

断言后台加载仍调用 `loadAndStitch(filePaths)`，启动构造函数仍不调用 `restoreLastSlamMap()`，界面会输出地理校正和体素去重统计。

- [ ] **步骤 2：补充界面日志**

日志包含：格式版本、有效/拒绝地理参考数、校正最大平移/偏航、ICP 接受或拒绝原因，以及三维重复点删除数。

- [ ] **步骤 3：运行完整相关测试**

运行：

```powershell
.\slam_map_io_tests.exe
.\slam_pose_tests.exe
.\slam_geo_anchor_tests.exe
.\slam_map_stitching_tests.exe
```

预期四个测试程序退出码均为 0。

- [ ] **步骤 4：构建主程序**

运行：

```powershell
cmake --build . --target PointCloud -j 2
```

预期输出 `[100%] Built target PointCloud`。

- [ ] **步骤 5：交付核对**

确认：

```text
启动自动地图恢复：禁用
v2.0 地图：可加载，保守兼容
v2.1 地图：逐关键帧地理校正
ICP：最大 3 米且质量门控
显示：0.3 米 XYZ 体素首点保留
原始 archive：不因显示去重丢点
```

此工作区不是 Git 仓库，因此所有任务在当前工作区内联完成，不执行 commit、分支合并或 worktree 清理。

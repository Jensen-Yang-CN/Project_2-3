# 2026-08-06 无调试落盘版泊位检测融合实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将 2026-08-06 交接包的最新版泊位算法规则完整融入当前工程，同时保持现有播放器、雷达、SLAM、桥梁、协议、外参和界面定制。

**架构：** `BerthDetector` 负责单帧和短期恢复算法，`RealtimeBerthRecall` 继续唯一持有 ENU 恢复 ROI、稳定库和世界锚点总线，`BerthDetectionNode` 在检测前注入世界上下文并在检测后回写恢复 ROI。现有 UI 和协议只消费最终 `BerthMeasureResult`，不引入交接包中的旧 GUI、旧协议或第二套世界状态。

**技术栈：** C++17、Qt5、Eigen3、OpenCV、CMake/CTest。

---

## 文件职责

- 修改 `src/berth/BerthDetection.h`：移植新版参数、世界检测上下文、恢复保护和外扩限制，删除停用的调试落盘代码。
- 修改 `src/berth/LineBerthDetection.h`：删除停用的调试落盘代码。
- 创建 `src/berth/BerthRecoveryRules.h`：承载可独立测试的锚点间距、外扩门限和稳定历史覆盖判断。
- 修改 `src/bus/RealtimeBerthRecall.h`：持有 ENU 恢复 ROI，生成检测前上下文并接收检测后 ROI 更新。
- 修改 `src/bus/BerthDetectionNode.cpp`：在 U 型检测前后接通世界上下文和恢复 ROI。
- 修改 `tests/berth_opening_rules_tests.cpp`：补充开口边在后处理后仍不进入可见边的回归断言（使用现有公共几何工具可表达的部分）。
- 创建 `tests/berth_recovery_rules_tests.cpp`：验证 9 m 锚点门限、10 m 外扩门限和稳定历史单锚点排斥。
- 修改 `tests/berth_recall_tests.cpp`：验证世界恢复 ROI 保存、当前位姿投影、无输出保留、检测器清空后同步清空和总线/稳定库上下文投影。
- 修改 `CMakeLists.txt`：注册 `berth_recovery_rules_tests`。

### 任务 1：建立新版恢复规则的失败测试

**文件：**
- 创建：`tests/berth_recovery_rules_tests.cpp`
- 创建：`src/berth/BerthRecoveryRules.h`
- 修改：`CMakeLists.txt`

- [x] **步骤 1：只创建测试目标和失败测试**

测试直接调用预期 API：

```cpp
check(!usv::berth_recovery::validAnchorPairDistance(8.99),
      "anchor pair shorter than 9 m must be rejected");
check(usv::berth_recovery::validAnchorPairDistance(9.01),
      "anchor pair longer than 9 m must be accepted");
check(!usv::berth_recovery::canExpandDimension(10.0, 0.2),
      "a 10 m dimension must not expand further");
check(usv::berth_recovery::canExpandDimension(9.7, 0.2),
      "a dimension remaining below 10 m may expand");
check(usv::berth_recovery::anchorOverlapsStableBerth(
          Eigen::Vector2d(4.9, 0.0), berth, 1.5),
      "single anchor in stable berth margin must be rejected");
```

- [x] **步骤 2：运行测试并确认因 API 缺失失败**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target berth_recovery_rules_tests -j 2
```

预期：编译失败，指出 `BerthRecoveryRules.h` 或预期函数不存在。

- [x] **步骤 3：实现最小规则头文件**

实现三个纯函数，常量固定为交接包值：锚点距离 `> 9.0`、下一扩展步不超过 `10.0`、旋转到稳定泊位局部坐标后按半宽/半长加 `1.5 m` 判断覆盖。

- [x] **步骤 4：运行新测试确认通过**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target berth_recovery_rules_tests -j 2
ctest --test-dir D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug -R berth_recovery_rules --output-on-failure
```

预期：`berth_recovery_rules` 通过。

### 任务 2：移植 U 型检测器新版行为

**文件：**
- 修改：`src/berth/BerthDetection.h`
- 测试：`tests/berth_recovery_rules_tests.cpp`
- 测试：`tests/berth_opening_rules_tests.cpp`

- [x] **步骤 1：增加检测器接口编译断言并确认失败**

在恢复规则测试中实例化 `BerthDetector`，调用：

```cpp
detector.setWorldAnchorBusConstraint(bus);
detector.setStableHistoricalBerths({stable});
detector.setWorldProjectedRecoveryRois({roi});
check(detector.hasRecoveryRoiMemory(),
      "projected world ROI must become detector recovery memory");
detector.clearWorldAnchorBusConstraint();
detector.clearStableHistoricalBerths();
```

运行构建，预期因接口不存在而编译失败。

- [x] **步骤 2：添加世界总线、稳定历史和恢复 ROI 接口**

从交接包移植 `setWorldAnchorBusConstraint()`、`clearWorldAnchorBusConstraint()`、`setStableHistoricalBerths()`、`clearStableHistoricalBerths()`、`setWorldProjectedRecoveryRois()` 和 `hasRecoveryRoiMemory()`。ROI 重投影时同步迁移跟踪恢复标志、高点状态和开口方向状态。

- [x] **步骤 3：应用新版恢复和外扩规则**

把双锚点校验、单锚点历史覆盖校验和逐维外扩判断统一改为调用 `BerthRecoveryRules.h`。把局部线 ROI 外边距调整为 4 m，并确保世界总线约束存在时原始锚点对不能替换它。

- [x] **步骤 4：保持独立开口规则并补充最终开口日志**

继续调用 `berth_opening::selectOpeningEdge()` 和 `countOpeningRayHits()`，记录正常射线、恢复唯一缺口、恢复锚点保持及最终稳定后的开口边来源。不得复制交接包中的第二套射线实现。

- [x] **步骤 5：运行恢复和开口测试**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target berth_recovery_rules_tests berth_opening_rules_tests -j 2
ctest --test-dir D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "berth_(recovery_rules|opening_rules)" --output-on-failure
```

预期：两个测试目标全部通过。

### 任务 3：接通 ENU 世界恢复 ROI 上下文

**文件：**
- 修改：`src/bus/RealtimeBerthRecall.h`
- 修改：`tests/berth_recall_tests.cpp`

- [x] **步骤 1：编写世界上下文失败测试**

测试预期以下接口：

```cpp
const auto context = recall.uShapeDetectionContext(world_from_lidar);
check(context.recovery_rois.size() == 1,
      "saved world ROI must project into current lidar frame");
check(context.has_world_anchor_bus,
      "confirmed world bus must be available before detection");
check(!context.stable_historical_berths.empty(),
      "stable U-shape library must be provided to the detector");
```

并验证：有输出时保存世界 ROI、无输出但检测器有记忆时保留、检测器无记忆时清空、`reset()` 后清空。

- [x] **步骤 2：运行测试并确认因接口缺失失败**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target berth_recall_tests -j 2
```

预期：编译失败，指出 `uShapeDetectionContext` 或 `updateUShapeRecoveryRois` 不存在。

- [x] **步骤 3：实现最小世界上下文和 ROI 生命周期**

在 `RealtimeBerthRecall` 中增加：

```cpp
struct UShapeDetectionContext {
    bool has_world_anchor_bus = false;
    DebugLine2D world_anchor_bus{};
    std::vector<Berth> stable_historical_berths;
    std::vector<Berth> recovery_rois;
};

UShapeDetectionContext uShapeDetectionContext(
    const Eigen::Isometry3d &world_from_lidar) const;

void updateUShapeRecoveryRois(
    const std::vector<Berth> &local_rois,
    const Eigen::Isometry3d &world_from_lidar,
    bool detector_has_memory);

void clearUShapeRecoveryRois();
```

世界 ROI 保存在 `u_shape_recovery_rois_world_`，`reset()` 同步清空。上下文只读取现有世界总线和 U 型稳定库，不创建第二套状态。

- [x] **步骤 4：运行盲区回显和稳定性测试**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target berth_recall_tests berth_world_stability_tests -j 2
ctest --test-dir D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "berth_(recall|world_stability)" --output-on-failure
```

预期：两个测试目标全部通过。

### 任务 4：在节点检测前后接入上下文

**文件：**
- 修改：`src/bus/BerthDetectionNode.cpp`

- [x] **步骤 1：在算法调用前查询一次雷达时间对应位姿**

把 `pose_buffer_.lookupNearest(res.timestamp, 0.25, world_from_lidar)` 移到 U 型算法调用前，保持 0.25 秒严格匹配。

- [x] **步骤 2：检测前注入世界约束**

位姿有效时调用 `uShapeDetectionContext()`，分别设置世界总线、稳定历史泊位和恢复 ROI；位姿无效时清除世界总线和稳定历史，新本地 ROI 无法投影或检测器记忆已清空时调用 `clearUShapeRecoveryRois()` 删除旧世界 ROI。

- [x] **步骤 3：检测后更新 ENU 恢复 ROI**

U 型算法返回后，在结果进入稳定/回显合并前调用 `updateUShapeRecoveryRois()`。有输出时保存本帧 U 型实时 ROI；无输出时依据 `hasRecoveryRoiMemory()` 决定保留或清空。

- [x] **步骤 4：保持现有合并和发布链**

继续执行 U/一字交叉仲裁、`mergeWithPose()`、蓝色回显、系统日志、状态仓库发布和 UI 信号，不引入交接包节点中的第二套确认/稳定状态。

- [x] **步骤 5：编译节点和主目标**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud -j 2
```

预期：主目标构建成功。

### 任务 5：彻底移除泊位调试落盘死代码

**文件：**
- 修改：`src/berth/BerthDetection.h`
- 修改：`src/berth/LineBerthDetection.h`

- [x] **步骤 1：删除两个检测器中 `#if 0` 包围的落盘函数**

删除 `save_debug_artifacts()`、`make_debug_basename()` 以及其中的目录创建、`cv::imwrite`、TXT 输出代码；不得删除内存 ROI、锚点、辅助线、`debug_messages` 或形态学矩阵生成。

- [x] **步骤 2：执行静态扫描**

运行：

```powershell
rg -n "save_debug_artifacts|debug_information|create_directories|cv::imwrite|std::ofstream|\.csv" src/berth
```

预期：无匹配。

- [x] **步骤 3：重新运行泊位专项测试**

运行：

```powershell
ctest --test-dir D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "berth_(recovery_rules|opening_rules|recall|world_stability|udp_protocol)" --output-on-failure
```

预期：泊位专项测试全部通过。

### 任务 6：完整回归验证

**文件：**
- 检查：`src/gui/MultiLidarWidget.cpp`
- 检查：`src/bus/PerceptionExportNode.cpp`
- 检查：`src/bus/GnssInsGeoUtils.cpp`
- 检查：`src/gui/mainwindow.cpp`

- [x] **步骤 1：确认保留项没有被压缩包覆盖**

核对实时/历史颜色、U 型三边绘制、`CoordinateLibrary` 来源、当前协议、`CalibrationConfig` 外参、播放器时间轴重置和暂停快照调用仍存在。

- [x] **步骤 2：运行完整测试集**

运行：

```powershell
ctest --test-dir D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

预期：所有已配置测试通过。

- [x] **步骤 3：执行干净的完整构建**

运行：

```powershell
cmake --build D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud --clean-first -j 2
```

预期：退出码为 0，生成当前 Debug 主程序。

## 版本控制说明

当前工作目录没有 `.git`，因此无法执行计划模板要求的逐任务 commit。每个任务改为以红灯/绿灯测试输出、静态扫描结果和最终完整构建输出作为检查点，不执行仓库初始化、提交或推送。

## 执行结果

- 新增恢复规则测试并完成红—绿循环；恢复规则、开口、回显、世界稳定和 UDP 协议专项测试 5/5 通过。
- 完整 CTest 回归 15/15 通过，覆盖雷达融合、SLAM、桥梁、播放器和暂停快照。
- `src/berth` 静态扫描未发现调试目录创建、`cv::imwrite`、`std::ofstream`、TXT/CSV 落盘函数。
- `PointCloud --clean-first` 全量构建成功，输出位于 `D:/1/yuanshibanben/PointCloud20260514/build-PointCloud-x86_windows_msys_pe_64bit-Debug/PointCloud.exe`。
- 构建仍有工程原有的 `_WIN32_WINNT` 重定义和桥梁匿名命名空间类型警告，本次泊位修改未新增编译错误。

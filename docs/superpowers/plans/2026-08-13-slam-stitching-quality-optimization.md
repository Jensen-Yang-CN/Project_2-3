# SLAM 地图拼接质量优化实现计划

> **面向 AI 代理的工作者：** 使用 executing-plans 在当前会话逐项实施；每项行为以测试先行，并在完成后更新复选框。

**目标：** 通过时间接缝裁剪消除重复关键帧重影，并用真实重叠、多尺度 ICP 和双向质量门限拒绝错误空间配准。

**架构：** `SlamMapStitcher` 在拼接前按关键帧时间排序；有时间重叠的连续地图用轨迹接缝变换且只追加新增关键帧，无时间重叠地图走锚点粗对齐和稳健空间 ICP。配准始终使用降采样副本，归档和渲染保留完整关键帧点云。

**技术栈：** C++17、Qt 5、Eigen、PCL ICP/KdTree、现有自定义测试程序、CMake/MSYS2 UCRT64。

---

## 文件职责

- 修改 `src/slam/SlamMapStitcher.h`：定义时间接缝、稳健 ICP 选项和可观测诊断字段。
- 修改 `src/slam/SlamMapStitcher.cpp`：实现输入排序、时间接缝估计、重复关键帧裁剪、真实重叠检查、多尺度 ICP 和双向质量评价。
- 修改 `src/gui/mainwindow.cpp`：为时间接缝和重复跳过模式显示明确的中文日志标签。
- 修改 `tests/slam_map_stitching_tests.cpp`：覆盖时间接缝、完全重复、padding 假重叠、无改善 ICP 回退及既有保存/显示回归。
- 创建 `D:/1/yuanshibanben/PointCloud20260514/SLAM地图拼接效果分析与优化说明_20260813.md`：记录真实数据证据、改动、验收方法和限制。

### 任务 1：用失败测试固定时间接缝行为

**文件：**

- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：增加可生成多关键帧、指定时间戳和轨迹的测试夹具**

```cpp
slam_map_io::MapArchive makeTrajectoryArchive(
    const usv::SlamGeoAnchor &anchor,
    double startTimestamp,
    int keyframeCount,
    const Eigen::Isometry3d &localFromReference);
```

- [x] **步骤 2：增加“部分时间重叠只追加新增关键帧”测试**

构造目标时间 `0..20`、来源时间 `10..30`。验证结果保留目标 21 帧、来源只追加 `21..30` 的 10 帧，诊断模式为 `TemporalSeam`，重复裁剪数为 11，关键帧 ID 连续。

- [x] **步骤 3：增加“完全包含地图跳过”测试**

构造目标时间 `0..20`、来源时间 `5..15`。验证最终关键帧数不变，诊断模式为 `DuplicateSkipped`，新增关键帧数为 0。

- [x] **步骤 4：运行测试并确认红灯**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_map_stitching_tests -j 4
& D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\tests\slam_map_stitching_tests.exe
```

预期：编译因新的诊断枚举或字段未定义而失败，证明测试确实约束新行为。

### 任务 2：实现时间接缝和重复裁剪

**文件：**

- 修改：`src/slam/SlamMapStitcher.h`
- 修改：`src/slam/SlamMapStitcher.cpp`

- [x] **步骤 1：扩展接口**

加入 `TemporalSeam`、`DuplicateSkipped` 模式以及以下选项：15 秒接缝窗口、至少 5 个对应位姿、中位残差不超过 0.75 米、90% 残差不超过 1.5 米、最大接缝平移 30 米、最大偏航 10 度。诊断记录时间重叠、匹配数、残差、裁剪数和新增数。

- [x] **步骤 2：稳定排序输入**

复制输入并按首关键帧时间戳 `std::stable_sort`，使用最早地图作为基准，保持时间相同输入的原顺序。

- [x] **步骤 3：实现轨迹插值和二维刚性拟合**

在重叠结束前 15 秒选取源关键帧；对目标轨迹按时间线性插值；对锚点粗对齐后的源 XY 与目标 XY 用 Eigen SVD 求解二维刚性变换，随后计算中位和 90% 残差。

- [x] **步骤 4：实现裁剪式追加**

接缝质量通过时，将接缝变换应用到来源关键帧，但只追加时间戳严格晚于当前最大时间的帧；完全包含时跳过来源地图；接缝质量不足时不裁剪并转入空间拼接。

- [x] **步骤 5：运行测试确认时间接缝绿灯**

运行 `slam_map_stitching_tests.exe`，预期时间接缝和完全重复测试通过，已有测试仍通过。

### 任务 3：用失败测试固定稳健 ICP 接受规则

**文件：**

- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：增加 padding 假重叠测试**

构造两个真实 XY 包围盒间隔 5 米、但旧代码在 10 米 padding 后会相交的密集点云。验证诊断模式保持 `AnchorOnly`，消息指明无真实重叠。

- [x] **步骤 2：增加“ICP 未改善不得接受”测试**

构造重复平行线或大面积均匀点云，使 ICP 能收敛但候选双向误差没有相对锚点产生足够改善。验证结果回退 `AnchorOnly`。

- [x] **步骤 3：保留可靠非对称点云恢复测试**

将已有 1.2 米、-0.6 米、2 度变换测试调整为无时间重叠但空间重叠，验证 `AnchorAndIcp` 仍恢复已知变换。

- [x] **步骤 4：运行测试并确认红灯**

预期：padding 假重叠或无改善候选仍被旧 ICP 接受。

### 任务 4：实现真实重叠、多尺度 ICP 和双向质量门限

**文件：**

- 修改：`src/slam/SlamMapStitcher.h`
- 修改：`src/slam/SlamMapStitcher.cpp`

- [x] **步骤 1：修正重叠区计算顺序**

先计算未经 padding 的 XY 交集；无交集直接回退。只有真实交集成立后，才在交集边缘扩展小范围上下文并提取配准点。

- [x] **步骤 2：实现两级二维 ICP**

粗级使用约 1.5 米体素、4 米最大对应距离；细级使用约 0.5 米体素、1.5 米最大对应距离，并以粗级结果作为细级初值。两级均限制迭代次数和总配准点数。

- [x] **步骤 3：实现双向稳健质量统计**

使用 PCL KdTree 计算源到目标、目标到源最近邻；统计双方有效匹配比例，并对各方向距离排序后裁掉最高 10% 异常值，计算对称稳健 RMSE。

- [x] **步骤 4：比较锚点基线和 ICP 候选**

接受条件同时包含：ICP 收敛、双向匹配比例达标、稳健 RMSE 达标、相对锚点至少有绝对或比例改善、平移不超过 6 米、偏航不超过 3 度。任一条件失败均回退锚点，并在消息中列出基线误差、最终误差、匹配率和拒绝原因。

- [x] **步骤 5：运行拼接测试确认绿灯**

预期：假重叠和无改善候选被拒绝，可靠非对称点云仍可恢复。

### 任务 5：真实三地图诊断和参数校准

**文件：**

- 修改（仅当真实指标证明必要）：`src/slam/SlamMapStitcher.h`
- 修改（仅当真实指标证明必要）：`src/slam/SlamMapStitcher.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：运行三份真实地图的拼接诊断**

输入：

```text
20260730_170136_570.slammap
20260810_193724_167.slammap
20260810_194048_767.slammap
```

验证第二份地图进入时间接缝，约 253 帧被裁掉、约 97 帧新增，最终总帧数约 453。

- [x] **步骤 2：检查第三份地图质量门限**

确认原先 `fitness=2.157 m²、平移=4.840 m` 的可疑解不能仅凭旧 fitness 被接受；新日志必须给出双向误差、匹配率和改善量。若调参，必须把真实失败特征转成自动测试后再改默认值。

- [x] **步骤 3：保存和重新加载真实拼接结果**

验证 `.slammap` 输出可重新读取，关键帧 ID 连续，锚点仍为最早基准地图锚点。

### 任务 6：完整回归、构建与文档

**文件：**

- 创建：`D:/1/yuanshibanben/PointCloud20260514/SLAM地图拼接效果分析与优化说明_20260813.md`

- [x] **步骤 1：运行全部自动测试**

运行 `ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure`，预期全部通过。

- [x] **步骤 2：完整构建主程序**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -j 4
```

预期：主程序和测试目标全部成功链接。

- [x] **步骤 3：编写优化说明**

文档写明问题根因、真实数据指标、时间接缝裁剪、多尺度 ICP、质量回退、测试结果、使用方法及“不修复单张地图内部非线性漂移”的边界。

- [x] **步骤 4：核对输出文件**

确认源码、测试、规格、计划和最终优化说明均存在，且没有修改或覆盖用户原始 `.slammap` 文件。

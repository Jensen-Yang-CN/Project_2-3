# SLAM 地图拼接仅平移安全修正实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法跟踪进度。

**目标：** 默认地图拼接只能修正 ENU 下的 X/Y 平移，禁止重复岸线导致的错误整图旋转。

**架构：** 保留现有稳定结构点、初值搜索和双向质量门控；增加仅平移变换估计器，并把默认搜索偏航固定为零。刚性二维 ICP 通过显式选项保留，供诊断测试使用。

**技术栈：** C++17、Eigen、PCL ICP、Qt、现有 `slam_map_stitching_tests`。

---

### 任务 1：建立过旋转回归测试

**文件：**
- 修改：`tests/slam_map_stitching_tests.cpp`

- [ ] 增加 `testDefaultRegistrationNeverChangesEnuYaw`，构造含平移和偏航的重复固定结构，调用默认 `stitch()` 后断言诊断偏航为 `0°`。
- [ ] 将已有 `testBoundedIcpCorrection` 显式设置 `allow_yaw_correction = true`，证明诊断模式仍能恢复已知小角度刚性误差。
- [ ] 将默认大残差和稳定杂点测试调整为纯平移真值，避免默认行为再依赖旋转。
- [ ] 编译运行 `slam_map_stitching_tests.exe`，预期新用例因默认实现仍允许偏航而失败。

### 任务 2：实现仅平移 ICP

**文件：**
- 修改：`src/slam/SlamMapStitcher.h`
- 修改：`src/slam/SlamMapStitcher.cpp`

- [ ] 在 `Options` 增加 `bool allow_yaw_correction = false`。
- [ ] 新增 PCL `TransformationEstimation` 实现，根据当前对应点的 X/Y 位移中位数生成单位旋转矩阵和平移向量。
- [ ] 默认初值搜索仅生成 `yaw = 0` 的候选；只有显式开启选项时才使用原有偏航候选。
- [ ] 粗、细 ICP 默认使用仅平移估计器；显式开启选项时继续使用 `TransformationEstimation2D`。
- [ ] 在接受结果前额外验证默认模式的偏航绝对值接近零，否则安全回退 ENU。
- [ ] 运行全部拼接测试，预期全部通过。

### 任务 3：真实地图与主程序验证

**文件：**
- 修改：`D:/1/yuanshibanben/PointCloud20260514/SLAM地图拼接效果分析与优化说明_20260813.md`

- [ ] 对 `20260810_193724_167.slammap` 和 `20260810_194048_767.slammap` 运行真实地图诊断。
- [ ] 确认第二张地图诊断偏航为 `0°`；若质量不足则允许回退 ENU，不放行旋转结果。
- [ ] 将真实平移、RMSE、匹配率和最终模式写入优化说明文档。
- [ ] 编译并链接 `PointCloud.exe`，确认最新二进制生成成功。

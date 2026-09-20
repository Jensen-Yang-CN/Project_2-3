# 泊位稳定化功能实现计划

> **面向 AI 代理的工作者：** 在当前会话内按测试驱动开发逐项执行；本目录不是 Git 仓库，因此不执行提交步骤。

**目标：** 在现有实时泊位坐标库和 U 型检测跟踪基础上，增加一字泊位参考长度保护、严重缩短时历史框替换，以及 U 型开口边连续三帧锁定。

**架构：** 一字泊位逻辑收敛在 `RealtimeBerthCoordinateLibrary`：正式库更新时保留首次入库长度，合并时仅对短于参考长度 `2/3` 的一字实时框使用历史框替换。U 型开口逻辑收敛在 `BerthDetector`：新增与 `tracked_berths_` 对齐的逐泊位状态，在开口判定完成后、所有派生几何计算前稳定输出。

**技术栈：** C++17、Eigen、OpenCV、CMake/CTest。

---

### 任务 1：一字泊位坐标库稳定化

**文件：**
- 修改：`tests/berth_recall_tests.cpp`
- 修改：`src/bus/RealtimeBerthCoordinateLibrary.h`
- 修改：`src/bus/RealtimeBerthRecall.h`

- [ ] 添加测试：一字泊位正式入库后，后续较短检测不能改写库中参考 `l`。
- [ ] 添加测试：实时长度处于 `[2/3 * 参考长度, 参考长度)` 时继续输出实时框。
- [ ] 添加测试：实时长度小于 `2/3 * 参考长度` 时输出坐标库完整框、来源为 `CoordinateLibrary`，且 `is_using_memory=true`。
- [ ] 运行 `berth_recall_tests`，确认新增测试因功能缺失而失败。
- [ ] 在正式库更新处使用保留一字 `l` 的更新函数。
- [ ] 在坐标库合并处增加短一字框替换，并把替换计入历史回显统计。
- [ ] 重新运行 `berth_recall_tests`，确认全部通过。

### 任务 2：U 型开口三帧锁定

**文件：**
- 创建：`src/berth/OpeningEdgeStabilizer.h`
- 创建：`tests/opening_edge_stabilizer_tests.cpp`
- 修改：`src/berth/BerthDetection.h`
- 修改：`CMakeLists.txt`

- [ ] 添加测试：同一泊位连续两帧相同开口不锁定，第三帧锁定。
- [ ] 添加测试：锁定后单帧开口突变仍输出锁定边。
- [ ] 添加测试：多泊位按中心和方向独立匹配状态。
- [ ] 添加测试：清空状态后重新累计三帧。
- [ ] 运行新测试，确认因稳定器尚不存在而失败。
- [ ] 实现专注、可独立测试的 `OpeningEdgeStabilizer`。
- [ ] 在 `BerthDetector::clear_detection_memory()` 中清空稳定器。
- [ ] 在每帧开口判定之后、船距/边高/可视边计算之前调用稳定器。
- [ ] 重新运行新测试与现有泊位回显测试，确认全部通过。

### 任务 3：集成验证

**文件：**
- 验证：`src/bus/BerthDetectionNode.cpp`
- 验证：`src/bus/PerceptionExportNode.cpp`
- 验证：`src/gui/MultiLidarWidget.cpp`

- [ ] 配置并构建测试目标。
- [ ] 运行所有泊位相关 CTest。
- [ ] 构建主程序目标，确认头文件集成无编译错误。
- [ ] 检查历史替换来源、`is_using_memory`、UI 颜色和协议角点排序的数据链路。
- [ ] 检查没有新增 `Map` 枚举或不相关重构。

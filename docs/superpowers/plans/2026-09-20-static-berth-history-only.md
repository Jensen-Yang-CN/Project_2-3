# 固定泊位图层仅用于历史地图 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让实时 `0xFB` 只发送检测结果，同时保留加载 `.slammap` 时的 11 个固定泊位图层。

**架构：** 固定泊位库继续由历史地图投递函数发送；启动函数不再发送固定库；实时 `sendBerthWithPose()` 不再插入或过滤固定库。其他地图、检测和协议代码不变。

**技术栈：** C++/Qt、现有 CTest 源码策略回归测试。

---

### 任务 1：编写失败的回归测试

**文件：**
- 修改：`tests/perception_export_realtime_disabled_tests.cpp`

- [x] **步骤 1：** 增加断言，要求源码中不存在启动阶段 `sendStaticBerthLibraryIfDue(true)` 调用，并要求实时组包不再出现 `static_count`、`overlaps_fixed_berth`；同时保留 `trySendHistoricalBerths()` 和 `sendStaticBerthLibraryIfDue()` 的历史路径。
- [x] **步骤 2：** 构建并运行该测试，预期因当前实现仍有启动调用、实时固定泊位插入和重叠过滤而失败。

### 任务 2：实施最小生产代码修改

**文件：**
- 修改：`src/bus/PerceptionExportNode.cpp:432-434, 1454-1631`

- [x] **步骤 1：** 删除 `start()` 中的 `sendStaticBerthLibraryIfDue(true)` 调用；不改历史 `trySendHistoricalBerths()` 调用。
- [x] **步骤 2：** 在 `sendBerthWithPose()` 中移除固定泊位数量预留、固定单位插入、重叠过滤和对应的 `static_count` 状态更新；实时结果直接按 `kMaxBerthsPerBatch` 组包。

### 任务 3：验证

**文件：**
- 无新增文件。

- [x] **步骤 1：** 将 `D:/msys64/ucrt64/bin` 加入构建环境 PATH 后，Debug/Release 目标均成功构建。
- [x] **步骤 2：** 运行回归测试和已有泊位协议、固定泊位库测试，确认历史固定层路径仍通过。
- [x] **步骤 3：** 检查差异，确认未修改泊位检测算法、本地界面、历史点云和接收端代码。

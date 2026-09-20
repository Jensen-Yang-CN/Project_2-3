# 泊位实时检测与盲区回显实现计划

> **面向 AI 代理的工作者：** 在当前会话中使用 executing-plans 逐任务实现；本目录不是
> Git 仓库，跳过 worktree 和 commit，所有验证必须保留完整命令输出。

**目标：** 按交付包逻辑把实时泊位检测、GNSS/INS 世界坐标库和蓝色盲区回显完整接入原项目。

**架构：** 新增一个不依赖 UI 的实时回显控制器，负责定位校验、ENU 位姿和坐标库合并；
`BerthDetectionNode` 只负责算法调用和发布。OpenGL 根据结果来源选择黄绿色或蓝色，并使用
算法输出的 U 型几何线段。

**技术栈：** C++17、Qt 5、Eigen3、OpenCV、CMake/CTest、OpenGL

---

## 文件职责

- 创建 `src/bus/RealtimeBerthRecall.h`：定位校验、ENU 位姿更新、坐标库写入与回显合并。
- 创建 `src/gui/BerthOverlayStyle.h`：实时/回显颜色的纯数据样式映射。
- 创建 `tests/berth_recall_tests.cpp`：坐标库确认、回显、实时优先和过期定位回归测试。
- 修改 `src/bus/BerthDetectionNode.h/.cpp`：接入回显控制器和来源日志。
- 修改 `src/gui/MultiLidarWidget.cpp`：清除空结果、按来源配色、按算法线段绘制。
- 修改 `CMakeLists.txt`：登记新文件并增加独立泊位测试目标。

### 任务 1：为盲区回显行为建立红灯测试

**文件：**
- 创建：`tests/berth_recall_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] 编写测试：5 帧确认后空检测产生 `CoordinateLibrary` 回显。
- [ ] 编写测试：实时结果与库结果重复时保留 `Realtime`。
- [ ] 编写测试：GNSS/INS 与点云相差超过 2 秒时不入库。
- [ ] 编写测试：实时样式为黄绿色、回显样式为蓝色。
- [ ] 配置独立测试目标并运行，确认因 `RealtimeBerthRecall.h` 和
  `BerthOverlayStyle.h` 尚不存在而失败。

运行：

```powershell
cmake -S . -B build-berth-recall -G "MinGW Makefiles" -DBUILD_TESTING=ON
cmake --build build-berth-recall --target berth_recall_tests --parallel 4
```

预期：测试目标编译失败，错误明确指向缺少待实现接口。

### 任务 2：实现最小回显控制器和颜色样式

**文件：**
- 创建：`src/bus/RealtimeBerthRecall.h`
- 创建：`src/gui/BerthOverlayStyle.h`

- [ ] 实现 `RealtimeBerthRecall::merge()`：校验定位、更新 ENU、生成
  `T_world_lidar`、仅写入实时结果、合并坐标库回显。
- [ ] 实现 `reset()`，同时清空 ENU 状态和坐标库。
- [ ] 实现 `berthOverlayColor()`：实时返回 `(0.68, 1.0, 0.12)`，坐标库返回
  `(0.10, 0.45, 1.0)`。
- [ ] 重新构建并运行 `berth_recall_tests`，确认全部通过。

运行：

```powershell
cmake --build build-berth-recall --target berth_recall_tests --parallel 4
ctest --test-dir build-berth-recall -R berth_recall --output-on-failure
```

预期：`berth_recall` 通过，退出码为 0。

### 任务 3：接入实时节点

**文件：**
- 修改：`src/bus/BerthDetectionNode.h`
- 修改：`src/bus/BerthDetectionNode.cpp`
- 修改：`CMakeLists.txt`

- [ ] 在节点中持有 `RealtimeBerthRecall`。
- [ ] `Init()` 时重置回显状态。
- [ ] 每帧算法合并后读取 `CommunicationManager` 最新 `GnssInsMessage` 并调用
  `merge()`。
- [ ] 日志稳定比较加入 `source`，输出实时/盲区来源及本帧回显数量。
- [ ] 运行泊位测试，确认节点接入没有破坏坐标库行为。

### 任务 4：修正 OpenGL 回显

**文件：**
- 修改：`src/gui/MultiLidarWidget.cpp`
- 修改：`CMakeLists.txt`

- [ ] `updateBerthResult()` 始终替换缓存；空结果清除可见状态。
- [ ] 绘制每个泊位前通过 `berthOverlayColor()` 设置 RGB：实时黄绿色、回显蓝色。
- [ ] U 型优先绘制 `visible_edges` 和 `opening_segment`；字段缺失时才回退到矩形边。
- [ ] 一字型保持完整矩形。
- [ ] 运行泊位测试并构建主程序。

### 任务 5：完整验证与代码审查

**文件：**
- 检查全部本次修改文件。

- [ ] 运行 `ctest --test-dir build-berth-recall --output-on-failure`。
- [ ] 运行 `cmake --build build-berth-recall --parallel 4`。
- [ ] 检查交付包算法头与主工程算法头 SHA256 仍一致。
- [ ] 请求代码审查，修复 Critical/Important 反馈。
- [ ] 重跑测试和完整构建，以最新输出作为完成依据。


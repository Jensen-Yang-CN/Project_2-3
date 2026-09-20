# 历史地图重叠区投递实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 把已加载历史分块地图完整投递，并在每个实时关键帧到来时重复投递其在历史 ENU 中的重叠区域。

**架构：** 新增独立、只读、可测试的历史 Tile 数据源；投递节点维护历史会话和高低优先级队列；主窗口只传递 manifest 与实时锚点。虚拟关键帧永不进入实时 SLAM。

**技术栈：** C++17、Qt 5、Eigen、现有 `SlamTileMapIO`、现有 UDP `0xFC` 协议。

---

### 任务 1：历史地图只读数据源

**文件：**
- 创建：`src/bus/HistoricalMapExportSource.h`
- 创建：`src/bus/HistoricalMapExportSource.cpp`
- 创建：`tests/historical_map_export_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] 编写测试：临时创建带有效地理锚点的 manifest/Tile，验证自动选择最细 LOD。
- [ ] 运行 `historical_map_export_tests`，确认因类不存在而失败。
- [ ] 实现 `openManifest()`、整图 Tile 顺序读取和有限 Tile 缓存。
- [ ] 编写测试：给定不同实时/历史锚点和关键帧位姿，验证覆盖范围被转换到历史 ENU，且仅返回范围内历史点。
- [ ] 运行测试，确认因重叠提取未实现而失败。
- [ ] 实现 `makeOverlapVirtualKeyframe()`：保留实时时间戳/编号，输出单位位姿和历史 ENU 点。
- [ ] 编写并运行重复调用测试，验证同一区域不会去重，每帧都返回虚拟关键帧。

### 任务 2：投递节点接入历史会话

**文件：**
- 修改：`src/bus/PerceptionExportNode.h`
- 修改：`src/bus/PerceptionExportNode.cpp`

- [ ] 增加加载/清除历史 manifest、更新实时锚点的线程内接口。
- [ ] 为 `0xFC` 构包增加显式历史锚点入口，保持报文长度、ID、分辨率和 chunk 布局不变。
- [ ] 增加历史整图低优先级队列；按一个 Tile 一批生产，达到水位后暂停，不再通过弹出队首丢包。
- [ ] 让实时/重叠包始终优先发送；历史整图在空闲带宽继续发送。
- [ ] 在 `onSlamKeyframe()` 中先保留原实时投递，再对每一帧调用重叠提取并投递虚拟关键帧。
- [ ] 调整清理逻辑：重置实时 SLAM 不清除已加载历史会话；停止整个投递节点才清空所有发送状态。

### 任务 3：界面加载流程接线

**文件：**
- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`

- [ ] 保存当前成功加载的分块 manifest 路径。
- [ ] 地图加载完成且投递正在运行时，立即把 manifest 交给投递节点；先加载后启动投递时也执行同样动作。
- [ ] 选择新地图时清除旧历史投递会话，避免旧新地图混发。
- [ ] 把关键帧与 `DloSlamNode::getGeoAnchor()` 放在同一个排队调用中交给投递节点，确保坐标转换使用对应会话锚点。
- [ ] 保证开始实时 SLAM 时关闭历史显示图层不会关闭历史投递数据源。

### 任务 4：构建与回归验证

**文件：**
- 修改：`CMakeLists.txt`

- [ ] 配置现有可用构建目录并构建 `historical_map_export_tests`，预期编译成功。
- [ ] 运行 `historical_map_export_tests`，预期全部通过。
- [ ] 构建主程序，预期新增源文件、Qt 元对象代码和既有协议均编译通过。
- [ ] 运行现有 `slam_tile_map_tests`、`slam_map_stitching_tests` 与相关投递协议测试，预期无回归。
- [ ] 检查代码路径，确认没有任何 `DloSlamNode` 写入、`keyframes_` 修改或地图保存调用接收虚拟关键帧。


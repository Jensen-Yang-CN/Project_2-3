# 保存并加载泊位图层实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将稳定泊位以独立的 ENU 语义图层写入 `.slammap`，再次加载地图时同步恢复并绘制泊位框。

**架构：** 在 `MapArchive` 中增加可选泊位记录，文件格式从 v2.1 升级为 v2.2；v2.0/v2.1 文件没有泊位段时仍可读取。实时检测结果通过现有 SLAM 时间同步器转换到地图 ENU 坐标，经过位置/尺寸/角度去重后存入界面地图层。加载地图时直接恢复该层，拼接时对泊位应用与点云一致的刚体变换。

**技术栈：** C++17、Qt5 `QDataStream`、Eigen、现有 `SlamBerthOverlaySynchronizer` 与 OpenGL 绘制路径。

---

### 任务 1：扩展地图数据结构和二进制读写

**文件：**
- 修改：`common/slam_types.h`
- 修改：`src/core/SlamMapBinaryIO.h`
- 修改：`src/core/SlamMapBinaryIO.cpp`
- 测试：`tests/slam_map_io_tests.cpp`

- [x] 先添加 v2.2 泊位记录往返测试、旧版无泊位层测试和未知次版本测试。
- [x] 运行 `slam_map_io_tests`，确认在缺少新结构时编译/测试失败。
- [x] 增加 `SlamMapBerth` 和 `MapArchive::berths`，保存/加载可选泊位段并验证字段。
- [x] 将格式次版本升级到 2，保持 v2.0/v2.1 读取兼容。
- [x] 运行测试确认新旧格式均通过。

### 任务 2：收集并稳定化 ENU 泊位

**文件：**
- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`
- 修改：`src/gui/SlamBerthOverlaySynchronizer.h`
- 测试：`tests/slam_berth_overlay_tests.cpp`

- [x] 先添加世界坐标泊位收集和近邻去重的失败测试。
- [x] 让同步器提供最新已投影世界坐标结果，地图控件维护去重后的泊位层。
- [x] 在泊位结果和 SLAM 位姿更新后都尝试收集，避免消息先后顺序造成遗漏。
- [x] 清空实时地图时清空泊位层，加载归档时用归档泊位层替换。

### 任务 3：保存、加载和渲染泊位层

**文件：**
- 修改：`src/gui/mainwindow.cpp`
- 修改：`src/gui/MultiLidarWidget.cpp`
- 修改：`src/slam/SlamMapStitcher.cpp`（拼接时同步变换记录）
- 测试：`tests/slam_map_stitching_tests.cpp`

- [x] 保存按钮从地图控件取得泊位层并写入 `MapArchive`。
- [x] 加载单图和拼接图时传递泊位层，SLAM 视图绘制静态泊位框并保留实时框。
- [x] 拼接源地图时对泊位中心和方向应用同一平移/旋转，按近邻合并重复泊位。
- [x] 运行地图 I/O、拼接和 OpenGL 相关回归测试；源码语法检查通过，完整 Qt Creator 构建仍需使用可用的 MinGW 工具链。

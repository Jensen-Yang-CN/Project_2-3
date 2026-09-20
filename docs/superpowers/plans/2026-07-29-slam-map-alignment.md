# SLAM 地图对齐与失真修正实现计划

> **面向 AI 代理的工作者：** 在当前会话中逐项实现并在每个行为变更后运行对应测试。

**目标：** 修正 SLAM 地图整体方向、船体显示偏移、双雷达错时融合和导航样本丢失导致的地图倾斜。

**架构：** 使用小型无状态辅助函数统一位姿与导航有效性规则；SLAM 后端保留雷达位姿语义，界面边界再转换为船体位姿；融合入口严格执行时间窗。

**技术栈：** C++17、Eigen、Qt5、CMake/CTest、NanoGICP。

---

### 任务 1：锁定雷达同步与配置加载行为

**文件：**
- 修改：`tests/radar_update_tests.cpp`
- 修改：`src/modules/RadarFusionPolicy.h`
- 修改：`src/modules/radarfusionmanager.cpp`
- 修复：`config/calibration.json`

- [x] 在 `radar_update_tests.cpp` 添加 100 ms 内接受、超过 100 ms 拒绝的断言。
- [x] 运行 `radar_update_tests`，确认同步断言或配置加载断言失败。
- [x] 在 `RadarFusionPolicy.h` 实现有限时间戳与绝对时间差判断。
- [x] 让 `RadarFusionManager` 使用该判断并删除强制通过分支。
- [x] 以 UTF-8/Qt 验证 JSON，并重新运行测试确认同步与配置加载均通过。

### 任务 2：锁定地图初始化、船体位姿和导航有效性

**文件：**
- 创建：`src/slam/SlamPoseUtils.h`
- 创建：`tests/slam_pose_tests.cpp`
- 修改：`CMakeLists.txt`
- 修改：`src/code-slam/LidarGnssSlam.cpp`
- 修改：`src/gui/MultiLidarWidget.cpp`

- [x] 添加测试目标，断言初始位姿等于匹配导航雷达位姿。
- [x] 断言 `T_map_lidar * inverse(T_body_lidar)` 能恢复原船体位姿。
- [x] 断言非有限数、非法经纬度和无效姿态标志被拒绝。
- [x] 运行测试并确认在辅助函数尚不存在时失败。
- [x] 实现 `SlamPoseUtils.h`，运行测试确认通过。
- [x] 将 SLAM 三个单位位姿初始化分支替换为匹配导航位姿。
- [x] 界面将雷达位姿转换为船体位姿后再绘制图标和轨迹。

### 任务 3：修正导航消费和时间窗口

**文件：**
- 修改：`src/slam/DloSlamNode.cpp`
- 修改：`config/config.json`
- 修改：`build-radar-slam/config.json`

- [x] 在每次 GICP 等待轮询中调用 `drainImuBus()`。
- [x] 将最大 GNSS/INS 同步时间从 2.0 秒收紧为 0.25 秒。
- [x] 运行 SLAM 位姿测试和已有 CTest，确认无回归。

### 任务 4：完整验证

**文件：**
- 验证：所有上述文件

- [x] 用 PowerShell UTF-8 `ConvertFrom-Json` 校验源目录和运行目录两个配置文件。
- [x] 运行 `ctest --test-dir build-radar-slam --output-on-failure`。
- [x] 运行 `cmake --build build-radar-slam --target PointCloud -j 4`。
- [x] 检查变更清单，确认未删除旧地图且未重新引入 refined 外参。

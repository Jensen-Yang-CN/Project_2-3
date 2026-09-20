# 独立泊位按钮与 SLAM 地图总线修复实现计划

> **面向 AI 代理的工作者：** 使用测试驱动方式执行以下步骤。

**目标：** 恢复独立泊位检测入口并修复 SLAM 关键帧到界面的断链。

**架构：** 主窗口分别控制桥梁和泊位节点；DLO 将地图输出发布到主窗口已经订阅的 Topic。保留既有信号以兼容旧调用者。

**技术栈：** C++17、Qt、现有 CommunicationManager Topic 总线、CMake。

---

- [x] 在 `tests/slam_map_source_merge_tests.cpp` 增加按钮和 Topic 发布契约测试并确认修改前失败。
- [x] 修改 `src/gui/mainwindow.cpp/.h`，增加独立泊位按钮和独立开关状态。
- [x] 修改 `src/slam/DloSlamNode.cpp/.h`，发布关键帧和实时扫描 Topic。
- [x] 重新运行回归测试并编译 `PointCloud` 主目标。

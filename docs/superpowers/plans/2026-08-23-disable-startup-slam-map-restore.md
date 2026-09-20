# 禁用启动时自动恢复 SLAM 地图：实现计划

## 任务 1：建立回归测试

- 修改 `tests/slam_map_stitching_tests.cpp` 中的主窗口地图流程接线测试。
- 将“启动时必须恢复最近地图”改为“启动时不得自动恢复地图”。
- 构建并运行 `slam_map_stitching_tests`，确认测试在旧实现上按预期失败。

## 任务 2：最小化修改启动逻辑

- 从 `src/gui/mainwindow.cpp` 的 `MainWindow` 构造流程中移除 `QTimer::singleShot(0, this, &MainWindow::restoreLastSlamMap)`。
- 保留 `restoreLastSlamMap()` 的声明、实现及最近地图配置；保留所有手动加载、拼接和保存流程。

## 任务 3：验证

- 重新构建并运行 `slam_map_stitching_tests`，要求全部通过。
- 构建 `PointCloud` 主程序，要求编译和链接成功。
- 检查源码，确认自动调用已消失，而手动加载和恢复辅助函数仍在。

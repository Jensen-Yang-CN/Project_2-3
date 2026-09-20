# SLAM 仅显示实时扫描层实现计划

> **面向 AI 代理的工作者：** 在当前会话中按 TDD 顺序实现。

**目标：** 隐藏固定累计关键帧层，只显示实时 SLAM 扫描。

**架构：** 通过一个无状态显示策略定义默认可见层，OpenGL 渲染入口遵守该策略；地图累计和保存逻辑不变。

**技术栈：** C++17、Qt5/OpenGL、CMake/CTest。

---

### 任务 1：显示策略回归测试

**文件：**
- 创建：`tests/slam_display_tests.cpp`
- 创建：`src/gui/SlamDisplayPolicy.h`
- 修改：`CMakeLists.txt`

- [x] 添加测试，要求累计关键帧层不可见、实时扫描层可见。
- [x] 在策略头文件不存在时构建测试，确认红灯。
- [x] 实现最小显示策略并确认测试转绿。

### 任务 2：接入渲染并验证

**文件：**
- 修改：`src/gui/MultiLidarWidget.cpp`

- [x] `renderSlamMap()` 不再提交 `m_slamMapCloud` 绘制。
- [x] `renderSlamMap()` 继续提交 `m_slamLiveScan`。
- [x] 运行全部 CTest。
- [x] 完整构建 `PointCloud` 主程序。

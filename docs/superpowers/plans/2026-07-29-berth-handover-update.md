# 新版实时泊位功能接入实现计划

> **面向 AI 代理的工作者：** 在当前会话内按测试驱动开发逐项执行；本目录不是 Git 仓库，因此不执行提交步骤。

**目标：** 接入新版 U 型检测、8 帧世界稳定和世界锚点总线，保留当前一字型历史能力及 UI 颜色/开口约定。

**架构：** U 型稳定与总线使用世界坐标独立组件，`RealtimeBerthRecall` 负责每帧编排，节点负责按雷达时间戳提供可靠 ENU 位姿，UI 只消费最终雷达坐标结果。

**技术栈：** C++17、Eigen、OpenCV、Qt、CMake/CTest。

---

### 任务 1：世界稳定与锚点总线

**文件：**
- 创建：`src/bus/RealtimeBerthStability.h`
- 创建：`src/bus/RealtimeWorldAnchorBus.h`
- 创建：`tests/berth_world_stability_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] 先写 8 帧稳定、5 帧替换、90° 轴交换、总线 3 帧确认/6 帧锁定、贴线和对侧翻转测试。
- [ ] 构建测试并确认因组件缺失失败。
- [ ] 移植两个交付组件，增加稳定器内部几何校正接口。
- [ ] 修正总线贴线后的开口线和三条 `visible_edges`。
- [ ] 运行测试并确认通过。

### 任务 2：新版 U 型算法和类型

**文件：**
- 修改：`src/berth/BerthDetection.h`
- 修改：`src/bus/common_types_extended.h`
- 删除：`src/berth/OpeningEdgeStabilizer.h`
- 删除：`tests/opening_edge_stabilizer_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] 增加接口编译测试，要求结果包含锚点总线和锚点恢复来源字段。
- [ ] 将新版 `BerthDetection.h` 适配到项目路径。
- [ ] 选择性增加 `is_anchor_recovered/has_anchor_bus/anchor_bus_line`。
- [ ] 移除旧边号稳定器和对应构建目标。
- [ ] 构建算法和世界组件测试。

### 任务 3：回显编排和交叉仲裁

**文件：**
- 修改：`src/bus/RealtimeBerthRecall.h`
- 修改：`src/bus/RealtimeBerthCoordinateLibrary.h`
- 修改：`tests/berth_recall_tests.cpp`

- [ ] 先写 U 型 8 帧入库、异常帧拒绝、稳定库回显、总线贴线、对侧翻转、实时优先和模式关闭测试。
- [ ] 增加相交 U 型候选拒绝测试。
- [ ] 改造回显编排，同时保留现有一字型 5 帧确认和长度保护。
- [ ] 运行所有泊位回显测试。

### 任务 4：GNSS/INS 时间匹配和节点接入

**文件：**
- 创建：`src/bus/BerthPoseBuffer.h`
- 创建：`tests/berth_pose_buffer_tests.cpp`
- 修改：`src/bus/bridge_common_types.h`
- 修改：`src/bus/GnssInsGeoUtils.cpp`
- 修改：`src/bus/BerthDetectionNode.h`
- 修改：`src/bus/BerthDetectionNode.cpp`
- 修改：`CMakeLists.txt`

- [ ] 先写最近时间样本、时间差阈值、质量无效拒绝、时间回退重置测试。
- [ ] 为 `GnssInsMessage` 保留导航状态和姿态有效标志。
- [ ] 订阅 `/sensor/imu/raw`，建立有界历史并按雷达时间匹配。
- [ ] 按新版顺序传递局部锚点总线和世界位姿。
- [ ] 时间轴回退时清空 ENU、稳定库、总线和位姿缓存。

### 任务 5：UI、协议和完整验证

**文件：**
- 修改：`src/gui/MultiLidarWidget.cpp`
- 验证：`src/gui/BerthOverlayStyle.h`
- 验证：`src/bus/BerthProtocolGeometry.h`
- 修改：`tests/berth_recall_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] 显示确认后的锚点总线，保持 U 型三边敞口。
- [ ] 验证实时黄绿色、历史蓝色和协议 P1/P4 开口侧顺序。
- [ ] 配置并构建全部泊位测试。
- [ ] 运行 CTest。
- [ ] 构建完整 `PointCloud`。
- [ ] 启动程序执行冒烟检查。

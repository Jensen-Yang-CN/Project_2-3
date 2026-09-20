# 水面杂波统一滤波实现计划

> **面向 AI 代理的工作者：** 在当前会话中按 executing-plans 与 test-driven-development 顺序执行。项目不是 Git 工作区，因此保留逐任务验证，但跳过 commit 步骤。

**目标：** 为 SLAM 点云地图和 UDP 栅格地图增加统一、自适应、可配置的水面杂波过滤。

**架构：** 独立纯 C++ `WaterSurfaceFilter` 负责平面估计、保护带裁剪、近水面离群点清理和有限历史回退。`DloSlamNode` 在共同输入边界调用一次，`AppConfig` 负责 JSON 配置，CMake 增加独立测试目标。

**技术栈：** C++17、Eigen 3、Qt 5 Core、CMake/CTest、MinGW Makefiles。

---

## 文件结构

- 创建 `src/slam/WaterSurfaceFilter.h`：滤波配置、结果统计和公开接口。
- 创建 `src/slam/WaterSurfaceFilter.cpp`：RANSAC、最小二乘平面拟合、空间哈希离群点过滤和历史回退。
- 创建 `tests/water_surface_filter_tests.cpp`：不依赖传感器的确定性行为测试。
- 修改 `common/slam_types.h`：在 `DloSlamConfig` 中携带水面滤波参数。
- 修改 `src/core/AppConfig.h/.cpp`：读取、校验 `slam.water_filter`。
- 修改 `config/config.json`：写入默认参数。
- 修改 `src/gui/mainwindow.cpp`：将应用配置传给 `DloSlamConfig`。
- 修改 `src/slam/DloSlamNode.h/.cpp`：统一调用滤波器并记录限频统计。
- 修改 `CMakeLists.txt`：编译滤波器并注册 CTest 测试。

### 任务 1：建立失败测试

- [ ] 创建 `tests/water_surface_filter_tests.cpp`，生成水平水面、倾斜水面、真实障碍物、孤立杂波和无可信平面五类合成数据。
- [ ] 修改 `CMakeLists.txt`，在 `BUILD_TESTING AND SLAM_ENABLED` 时创建 `water_surface_filter_tests`。
- [ ] 仅放入接口声明和空实现，使测试可以编译但行为断言失败。
- [ ] 运行：

```powershell
D:\CMake\bin\cmake.exe -S D:\1\yuanshibanben\PointCloud20260514\PointCloud `
  -B D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug `
  -G "MinGW Makefiles"
D:\CMake\bin\cmake.exe --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug `
  --target water_surface_filter_tests --parallel 4
D:\CMake\bin\ctest.exe --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug `
  -R water_surface_filter --output-on-failure
```

预期：测试程序成功构建，CTest 因水面点仍被保留而失败。

### 任务 2：实现最小可用滤波器

- [ ] 在 `WaterSurfaceFilter.cpp` 实现近水平 RANSAC 平面候选评分。
- [ ] 用 Eigen 对最佳内点做协方差特征分解，细化平面。
- [ ] 将平面法向朝向正 Z，按有符号高度删除 `height <= clearance_m` 的点。
- [ ] 实现空间哈希，仅清理 `clearance_m < height <= clearance_m + near_surface_band_m` 内的孤立点。
- [ ] 保存最近可信平面并按 `max_fallback_frames` 限制回退。
- [ ] 运行上一任务的 CTest 命令。

预期：全部水面滤波测试通过。

### 任务 3：接入配置与 SLAM 数据流

- [ ] 在 `SlamAppConfig` 和 `DloSlamConfig` 中加入逐项默认值。
- [ ] 在 `AppConfig::load()` 中读取 `slam.water_filter`，对半径、角度、点数和帧数做范围约束。
- [ ] 在 `mainwindow.cpp::initSlamNode()` 将配置复制到 `DloSlamConfig`。
- [ ] 在 `DloSlamNode` 成员中保存滤波器；`init()` 时应用配置。
- [ ] 在距离裁剪后调用滤波器。过滤后不足以供 SLAM 使用时跳过该帧，并以限频日志输出统计。
- [ ] 在关键坐标系、0.25 m 阈值、回退条件及误删保护处添加中文注释。
- [ ] 运行 CTest，确认集成未改变独立滤波行为。

### 任务 4：全量验证

- [ ] 运行：

```powershell
D:\CMake\bin\cmake.exe --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug `
  --parallel 4
D:\CMake\bin\ctest.exe --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug `
  --output-on-failure
```

- [ ] 检查 `PointCloud.exe` 已更新且构建退出码为 0。
- [ ] 在 Qt Creator 4.11.1 中启动，点击“SLAM建图”，观察至少 30 秒。
- [ ] 检查没有新增 `PointCloud.exe` Windows Application Error 或 CrashDump。
- [ ] 核对配置、测试、滤波实现、SLAM 接入和注释五项均存在。


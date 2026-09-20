# SLAM 地图压缩包源码合并实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将压缩包中与 SLAM 地图生成相关的动态静态点云处理和旧地图兼容能力更新到当前工程，同时保留当前 `.slammap v2.1` 主流程。

**架构：** 以当前工程的 `.slammap v2.1` MapArchive 为唯一新地图保存格式；把压缩包的动态目标剔除放到 LidarGnssSlam 的点云处理链中，把静态点云作为 DLO 关键帧输入；把压缩包的扁平 `.pcd` 作为独立兼容读取路径包装成单关键帧地图。当前地理锚点、导航参考和拼接/瓦片模块不回退。

**技术栈：** C++17、Qt5、Eigen、PCL、NanoGICP、CMake、CTest。

---

### 任务 1：先为旧 `.pcd` 兼容接口建立失败测试

**文件：**
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\tests\slam_map_io_tests.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\core\SlamMapBinaryIO.h`

- [ ] **步骤 1：增加失败测试**

在 `slam_map_io_tests.cpp` 增加真实临时文件测试：调用 `slam_map_io::save()` 写入两个 `M_PointXYZI`，再调用 `slam_map_io::load()` 读取并比较字段；同时写入一个点数与文件大小不匹配的 `.pcd`，确认 `load()` 失败且输出云不被破坏。

- [ ] **步骤 2：运行测试确认接口缺失**

运行：`ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-radar-slam -R "^slam_map_io$" --output-on-failure`

预期：测试目标仍使用旧测试内容，新增调用在重新配置/编译时因 `slam_map_io::save/load` 未声明而失败。

- [ ] **步骤 3：定义兼容接口**

在 `SlamMapBinaryIO.h` 增加：

```cpp
constexpr uint32_t kLegacyMaxPointCount = 5000000;
bool save(const QString &path, const QVector<M_PointXYZI> &cloud,
          QString *error = nullptr);
bool load(const QString &path, QVector<M_PointXYZI> &cloud,
          QString *error = nullptr);
```

- [ ] **步骤 4：运行测试确认仍然失败于实现缺失**

重新配置并构建 `slam_map_io_tests`，确认链接阶段报告 `slam_map_io::save/load` 未定义。

---

### 任务 2：实现旧 `.pcd` 兼容读写并通过红绿测试

**文件：**
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\core\SlamMapBinaryIO.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\tests\slam_map_io_tests.cpp`

- [ ] **步骤 1：实现最小兼容读写**

按压缩包格式写入小端主机上的 `[uint32_t count][count * sizeof(M_PointXYZI)]`，限制最大点数为 `kLegacyMaxPointCount`，检查空点云、短文件、异常点数和精确文件大小；读取失败时保留调用方原有 `QVector` 不变。

- [ ] **步骤 2：运行定向测试**

运行：`cmake --build D:\1\yuanshibanben\PointCloud20260514\build-radar-slam --target slam_map_io_tests --parallel 4`，再运行 `ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-radar-slam -R "^slam_map_io$" --output-on-failure`。

预期：旧 `.pcd` 往返和损坏文件测试通过，已有 `.slammap v2.1` 测试仍通过。

---

### 任务 3：移植压缩包动态静态点云处理

**文件：**
- 创建：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\code-slam\LidarGnssSlamConfig.h`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\code-slam\LidarGnssSlam.h`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\code-slam\LidarGnssSlam.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\CMakeLists.txt`

- [ ] **步骤 1：增加动态过滤配置测试**

在源码合并测试中检查 `LidarGnssSlamConfig.h` 暴露压缩包的 SOR、聚类、匹配、速度和静态重叠阈值，并检查 `LidarGnssSlam.h` 保留 `getLastStaticCloud()` 与当前 `getLastNavigationReference()` 两套接口。

- [ ] **步骤 2：运行测试确认当前代码缺少压缩包能力**

运行源码合并测试，预期在新增头文件和接口尚未加入时失败。

- [ ] **步骤 3：合并实现**

加入压缩包的动态点云状态、聚类跟踪、确认记忆、静态点云输出、预注册和动态点剔除实现；在当前处理流程中重新写入 `SlamGeoPoseReference`，并保留当前导航同步误差指针参数。清空/重锚时同时清空静态点云缓存，避免上一帧静态点被误用于新关键帧。

- [ ] **步骤 4：运行编译验证**

重新配置 CMake，构建 `PointCloud` 目标和源码合并测试。预期 `LidarGnssSlam.cpp` 能在当前 Qt/PCL/NanoGICP 配置下编译。

---

### 任务 4：让关键帧和实时地图使用静态点云

**文件：**
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\slam\DloSlamNode.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\tests\slam_map_source_merge_tests.cpp`

- [ ] **步骤 1：增加失败的源码行为测试**

检查 `DloSlamNode.cpp` 在算法处理完成后调用 `getLastStaticCloud()`，并将同一静态点云同时传给 `createKeyframe()` 和 `postSlamScanCloud()`；测试还要检查 `getLastNavigationReference()` 仍传入关键帧创建逻辑。

- [ ] **步骤 2：运行测试确认当前行为不满足**

运行源码合并测试，预期当前代码因仍使用 `filtered_points` 而失败。

- [ ] **步骤 3：最小修改点云来源**

在算法处理完成后读取 `static_points`，关键帧和实时世界扫描统一使用它；不修改当前的距离过滤、水面过滤、IMU 排空、处理超时和关键帧管理逻辑；当静态云为空时不把原始输入重新写回地图。

- [ ] **步骤 4：运行定向测试和构建**

运行源码合并测试并构建 `PointCloud`，确认静态点云调用、地理参考调用和现有 Qt 信号连接同时成立。

---

### 任务 5：支持压缩包旧 `.pcd` 的加载，但保持 `.slammap` 新格式主流程

**文件：**
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\slam\SlamMapLoadTask.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\gui\mainwindow.cpp`
- 修改：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\tests\slam_map_source_merge_tests.cpp`

- [ ] **步骤 1：增加失败测试**

检查地图加载任务允许 `.pcd` 后缀并将扁平云包装为一个无地理锚点的 `SlamKeyframe`；同时检查保存对话框仍只生成 `.slammap`。

- [ ] **步骤 2：运行测试确认当前实现拒绝 `.pcd`**

运行源码合并测试，预期当前加载任务的“仅支持新版 `.slammap`”分支导致失败。

- [ ] **步骤 3：实现兼容加载**

对 `.slammap` 继续调用 `loadArchive()`；对 `.pcd` 调用兼容 `load()`，构造 identity pose 的单关键帧，设置 `source_format_major=1`、`source_format_minor=0`，anchor 保持无效。多张旧 `.pcd` 不自动伪造地理关系，加载错误信息必须说明其为无地理锚点兼容地图。文件选择对话框同时列出 `*.slammap` 和 `*.pcd`，保存逻辑继续写 `*.slammap`。

- [ ] **步骤 4：运行地图加载相关测试**

运行 `slam_map_io`、`slam_map_stitching` 和源码合并测试，检查新旧格式分支不互相覆盖。

---

### 任务 6：全量验证并记录冲突处理结果

**文件：**
- 检查：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\common\slam_types.h`
- 检查：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\config\config.json`
- 检查：`D:\1\yuanshibanben\PointCloud20260514\PointCloud\src\core\SlamMapBinaryIO.cpp`

- [ ] **步骤 1：确认同步阈值没有被旧配置覆盖**

检查运行时配置仍为当前 `0.25s`，并在源码合并测试中确认没有把压缩包 `2.0s` 写回当前配置。

- [ ] **步骤 2：重新配置和编译**

运行：`cmake -S D:\1\yuanshibanben\PointCloud20260514\PointCloud -B D:\1\yuanshibanben\PointCloud20260514\build-slam-map-merge -G "MinGW Makefiles" -DBUILD_TESTING=ON -DENABLE_SLAM=ON`；随后构建 `PointCloud` 及所有可用 SLAM 测试目标。

- [ ] **步骤 3：运行 CTest**

运行：`ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-slam-map-merge --output-on-failure`，记录退出码和失败测试。

- [ ] **步骤 4：按需求逐项检查**

确认只读取源码与压缩包源码，没有打开工作区已有 `.slammap`；确认新保存仍是 `.slammap v2.1`，旧 `.pcd` 只走兼容路径；确认最终报告列出压缩包与当前生成逻辑的冲突和修正结果。

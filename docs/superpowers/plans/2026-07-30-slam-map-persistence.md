# SLAM 地图持久化与启动恢复实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将地图升级为保存地理锚点与关键帧的版本化 `.slammap`，并在程序启动时自动恢复最近地图、由关键帧重建累计点云。

**架构：** `common/slam_types.h` 定义跨 SLAM、持久化和界面共享的地理锚点；`SlamMapBinaryIO` 负责归档序列化、旧格式兼容及最近地图选择；`DloSlamNode` 从实际送入算法的 GNSS 流捕获锚点；`MultiLidarWidget` 保存/恢复关键帧并完成世界系累计；`MainWindow` 编排保存、手动加载和启动恢复。

**技术栈：** C++17、Qt 5 Core/Widgets、Eigen、CMake/CTest。

---

## 文件结构

- 修改 `common/slam_types.h`：增加 `SlamGeoAnchor` 公共数据类型。
- 修改 `src/core/SlamMapBinaryIO.h`：声明新版归档、格式限制、兼容读取和最近地图 API。
- 修改 `src/core/SlamMapBinaryIO.cpp`：用 `QDataStream` 与 `QSaveFile` 实现显式字段序列化、严格校验和自动恢复候选列表。
- 创建 `tests/slam_map_io_tests.cpp`：验证新版往返、旧格式、损坏文件和最近地图选择。
- 修改 `CMakeLists.txt`：注册 `slam_map_io_tests`。
- 修改 `src/slam/DloSlamNode.h` 与 `src/slam/DloSlamNode.cpp`：捕获和读取首个有效 GNSS 锚点。
- 修改 `src/gui/MultiLidarWidget.h` 与 `src/gui/MultiLidarWidget.cpp`：保留关键帧，保存归档，加载后重建累计地图、轨迹和末帧位姿。
- 修改 `src/gui/mainwindow.h` 与 `src/gui/mainwindow.cpp`：保存 `.slammap`、记录最近地图、手动加载兼容格式、启动自动恢复、新建图前清除历史地图。

### 任务 1：用失败测试锁定新版归档 API 与行为

**文件：**

- 创建：`tests/slam_map_io_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：编写新版往返失败测试**

测试构造两个关键帧和有效锚点，调用尚不存在的新版 API：

```cpp
slam_map_io::MapArchive source;
source.anchor = {true, 12.5, 31.234, 121.456, 4.2};
source.keyframes = {makeKeyframe(7, 12.5, 1.0),
                    makeKeyframe(8, 13.0, 3.0)};
check(slam_map_io::saveArchive(path, source, &error),
      "versioned archive should save", failures);

slam_map_io::MapArchive loaded;
check(slam_map_io::loadArchive(path, loaded, &error),
      "versioned archive should load", failures);
check(loaded.anchor.valid
          && close(loaded.anchor.latitude_deg, 31.234)
          && loaded.keyframes.size() == 2
          && loaded.keyframes[1].cloud.size() == 2,
      "anchor, poses and local clouds must round-trip", failures);
```

- [ ] **步骤 2：编写兼容性和错误失败测试**

同一测试程序加入：

```cpp
writeLegacyMap(legacyPath, legacyCloud);
check(slam_map_io::loadArchive(legacyPath, loaded, &error)
          && loaded.legacy_format
          && loaded.legacy_world_cloud.size() == legacyCloud.size(),
      "legacy point-only maps must remain readable", failures);

writeBytes(truncatedPath, QByteArray("USVSLM2", 7));
check(!slam_map_io::loadArchive(truncatedPath, loaded, &error),
      "truncated versioned maps must be rejected", failures);
```

并用临时目录验证：

```cpp
check(slam_map_io::rememberLastMap(preferred, &error),
      "successful map selection should be remembered", failures);
const QStringList candidates = slam_map_io::automaticRestoreCandidates();
check(!candidates.isEmpty() && candidates.front() == canonical(preferred),
      "remembered map must have startup priority", failures);
```

- [ ] **步骤 3：在 CMake 注册测试**

```cmake
add_executable(slam_map_io_tests
    tests/slam_map_io_tests.cpp
    src/core/SlamMapBinaryIO.h
    src/core/SlamMapBinaryIO.cpp
    src/core/AppConfig.h
    src/core/AppConfig.cpp
    common/slam_types.h
    common/pointxyz.h
)
target_include_directories(slam_map_io_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
)
target_link_libraries(slam_map_io_tests PRIVATE Qt5::Core Eigen3::Eigen)
add_test(NAME slam_map_io COMMAND slam_map_io_tests)
```

- [ ] **步骤 4：运行测试并确认红灯**

运行：

```powershell
cmake -S . -B build-radar-slam -DBUILD_TESTING=ON
cmake --build build-radar-slam --target slam_map_io_tests -j 4
```

预期：编译失败，提示 `MapArchive`、`saveArchive`、`loadArchive`、`rememberLastMap` 和 `automaticRestoreCandidates` 尚未定义。

### 任务 2：实现版本化地图格式和最近地图选择

**文件：**

- 修改：`common/slam_types.h`
- 修改：`src/core/SlamMapBinaryIO.h`
- 修改：`src/core/SlamMapBinaryIO.cpp`
- 测试：`tests/slam_map_io_tests.cpp`

- [ ] **步骤 1：定义共享地理锚点**

在 `common/slam_types.h` 增加：

```cpp
struct SlamGeoAnchor {
    bool valid = false;
    double timestamp = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};
```

- [ ] **步骤 2：定义归档和边界**

在 `SlamMapBinaryIO.h` 增加：

```cpp
struct MapArchive {
    usv::SlamGeoAnchor anchor;
    std::vector<usv::SlamKeyframe> keyframes;
    QVector<M_PointXYZI> legacy_world_cloud;
    bool legacy_format = false;
};

constexpr quint16 kFormatMajor = 2;
constexpr quint16 kFormatMinor = 0;
constexpr quint32 kMaxKeyframeCount = 10000;
constexpr quint32 kMaxPointsPerKeyframe = 5000000;
constexpr quint64 kMaxTotalKeyframePoints = 100000000;

bool saveArchive(const QString &, const MapArchive &, QString * = nullptr);
bool loadArchive(const QString &, MapArchive &, QString * = nullptr);
bool rememberLastMap(const QString &, QString * = nullptr);
QStringList automaticRestoreCandidates();
```

保留旧的点云 `save/load` 函数，仅供旧格式兼容测试和读取。

- [ ] **步骤 3：实现原子保存和显式序列化**

`saveArchive` 使用 `QSaveFile`。头部写入固定魔数 `USVSLM2`、主次版本、ENU 坐标系编号、锚点字段和关键帧数。位姿以三维平移加单位四元数写入；点坐标使用单精度、时间戳和位姿使用双精度；强度使用 `quint8`。

保存前验证：

```cpp
if (archive.keyframes.empty())
    return fail(error, QStringLiteral("没有关键帧可保存"));
if (!validAnchorOrExplicitlyInvalid(archive.anchor)
    || archive.keyframes.size() > kMaxKeyframeCount
    || totalPoints(archive) > kMaxTotalKeyframePoints)
    return fail(error, QStringLiteral("地图数据超出格式限制或包含非法数值"));
```

- [ ] **步骤 4：实现新版加载和旧格式分流**

`loadArchive` 先读取魔数：

```cpp
const QByteArray prefix = file.peek(kMagicSize);
if (prefix == kMagic)
    return loadVersion2(file, archive, error);
return loadLegacyPointCloud(file, archive, error);
```

新版加载到局部 `MapArchive candidate`，全部字段、计数和 EOF 校验通过后才移动给输出参数，确保失败时调用者原数据不变。四元数必须为有限数且范数大于最小阈值，读取后归一化。

- [ ] **步骤 5：实现最近地图记录和候选排序**

`rememberLastMap` 用 `QSaveFile` 写入 `maps/last_slam_map.txt`。`automaticRestoreCandidates`：

1. 若记录指向存在的 `.slammap`，规范化后放在首位；
2. 扫描 `maps/*.slammap`，按修改时间降序、文件名降序排序；
3. 去重后返回，且排除记录文件及旧 `.pcd`。

- [ ] **步骤 6：运行测试确认绿灯**

运行：

```powershell
cmake --build build-radar-slam --target slam_map_io_tests -j 4
ctest --test-dir build-radar-slam -R slam_map_io --output-on-failure
```

预期：`slam_map_io` 通过，输出无失败。

### 任务 3：从实际 GNSS 输入捕获 SLAM 地理锚点

**文件：**

- 修改：`src/slam/DloSlamNode.h`
- 修改：`src/slam/DloSlamNode.cpp`
- 创建：`tests/slam_geo_anchor_tests.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：编写锚点捕获失败测试**

将合法性和“仅首个有效数据生效”抽成无 Qt/PCL 依赖的头文件函数，测试：

```cpp
usv::SlamGeoAnchor anchor;
usv::captureFirstSlamGeoAnchor(invalidGnss(), anchor);
check(!anchor.valid, "invalid zero GNSS must not create an anchor", failures);

usv::captureFirstSlamGeoAnchor(validGnss(31.1, 121.2, 8.0, 10.0), anchor);
usv::captureFirstSlamGeoAnchor(validGnss(32.0, 122.0, 9.0, 11.0), anchor);
check(anchor.valid && close(anchor.latitude_deg, 31.1)
          && close(anchor.longitude_deg, 121.2)
          && close(anchor.timestamp, 10.0),
      "the first valid GNSS supplied to SLAM must remain the anchor", failures);
```

运行：

```powershell
cmake --build build-radar-slam --target slam_geo_anchor_tests -j 4
```

预期：因 `captureFirstSlamGeoAnchor` 不存在而编译失败。

- [ ] **步骤 2：实现锚点捕获并接入所有 GNSS 路径**

创建 `src/slam/SlamGeoAnchorPolicy.h`：

```cpp
inline bool captureFirstSlamGeoAnchor(const GnssInsMessage &gnss,
                                      SlamGeoAnchor &anchor)
{
    if (anchor.valid || !slam_pose::validGnssIns(gnss))
        return false;
    anchor.valid = true;
    anchor.timestamp = gnss.timestamp;
    anchor.latitude_deg = gnss.latitude;
    anchor.longitude_deg = gnss.longitude;
    anchor.altitude_m = gnss.altitude;
    return true;
}
```

`DloSlamNode` 增加受互斥量保护的 `geo_anchor_` 和 `getGeoAnchor()`。`feedGnssIns` 在调用算法前捕获锚点，`drainImuBus` 必须改为调用 `feedGnssIns(**opt_msg)`，避免总线消费路径绕过锚点捕获。

- [ ] **步骤 3：运行锚点测试确认绿灯**

运行：

```powershell
cmake --build build-radar-slam --target slam_geo_anchor_tests -j 4
ctest --test-dir build-radar-slam -R slam_geo_anchor --output-on-failure
```

预期：`slam_geo_anchor` 通过。

### 任务 4：由关键帧保存和恢复界面地图

**文件：**

- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`
- 测试：`tests/slam_map_io_tests.cpp`

- [ ] **步骤 1：增加世界坐标恢复测试并确认红灯**

在 `slam_map_io_tests` 中调用尚不存在的纯函数：

```cpp
const QVector<M_PointXYZI> world =
    slam_map_io::rebuildWorldCloud(loaded.keyframes, 100);
check(world.size() == 4
          && close(world[2].x, expectedTranslatedX),
      "loaded local clouds must be transformed by T_map_lidar", failures);
```

运行测试，预期因 `rebuildWorldCloud` 未定义而编译失败。

- [ ] **步骤 2：实现共享的关键帧累计函数**

在 `SlamMapBinaryIO` 增加：

```cpp
QVector<M_PointXYZI> rebuildWorldCloud(
    const std::vector<usv::SlamKeyframe> &keyframes,
    int max_points,
    uint8_t display_intensity = 90);
```

函数逐点执行 `keyframe.pose * Eigen::Vector3d(p.x, p.y, p.z)`，超过显示上限时按确定性步长抽样，确保启动加载不会无限占用显存和内存。

- [ ] **步骤 3：让控件保存关键帧而不是预览点云**

`MultiLidarWidget` 增加成员：

```cpp
std::vector<usv::SlamKeyframe> m_slamKeyframes;
usv::SlamGeoAnchor m_slamGeoAnchor;
```

`appendSlamKeyframe` 同时保存关键帧并调用现有实时合并；`clearSlamMap` 清空两者；新增 `setSlamGeoAnchor`。`saveCurrentSlamMap` 构造 `MapArchive` 并调用 `saveArchive`，如果只有旧地图预览而无关键帧，返回明确错误。

- [ ] **步骤 4：加载后重建地图、轨迹和末帧位姿**

`loadSlamMap` 先读取局部归档：

```cpp
slam_map_io::MapArchive archive;
if (!slam_map_io::loadArchive(filePath, archive, errorMsg))
    return false;
```

新版：

- 用 `rebuildWorldCloud` 生成新地图；
- 用关键帧位姿生成 `m_slamPath`；
- 用最后一帧位姿恢复 `m_slamPose/m_slamX/m_slamY/m_slamYaw`；
- 保存锚点和关键帧。

旧版：沿用点云显示，不伪造关键帧、轨迹、位姿和锚点。所有新状态准备成功后一次性替换成员。

- [ ] **步骤 5：运行持久化测试确认绿灯**

运行：

```powershell
cmake --build build-radar-slam --target slam_map_io_tests -j 4
ctest --test-dir build-radar-slam -R slam_map_io --output-on-failure
```

预期：关键帧世界系恢复测试通过。

### 任务 5：编排保存、手动加载和启动自动恢复

**文件：**

- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`

- [ ] **步骤 1：增加统一的地图视图激活与启动恢复方法**

在 `MainWindow` 声明：

```cpp
void activateLoadedSlamMapView();
void restoreLastSlamMap();
```

`restoreLastSlamMap` 遍历 `automaticRestoreCandidates()`。候选加载失败只写日志并继续；首个成功候选调用 `activateLoadedSlamMapView()` 并 `rememberLastMap()`；全部失败时不改变当前视图。

- [ ] **步骤 2：在构造完成后调度自动恢复**

在 `AppConfig::load()`、日志控件和总线初始化完成后调用：

```cpp
QTimer::singleShot(0, this, &MainWindow::restoreLastSlamMap);
```

保证窗口和 OpenGL 控件已构造，同时不阻塞其余初始化。

- [ ] **步骤 3：升级保存路径与锚点写入**

保存文件名改为：

```cpp
yyyyMMdd_HHmmss_zzz.slammap
```

保存前若 `slam_node_` 存在：

```cpp
ui->openGLWidget->setSlamGeoAnchor(slam_node_->getGeoAnchor());
```

`saveCurrentSlamMap` 成功后调用 `rememberLastMap`。记录写入失败只报告警告，不将已成功保存的地图误报为失败。

- [ ] **步骤 4：升级手动加载**

文件过滤器改为：

```cpp
tr("新版 SLAM 地图 (*.slammap);;旧版 SLAM 点云 (*.pcd);;所有文件 (*.*)")
```

加载成功统一调用 `activateLoadedSlamMapView()`；仅 `.slammap` 更新最近地图记录，旧 `.pcd` 不参与下次自动恢复。

- [ ] **步骤 5：新建图前隔离历史地图**

`onSlamBuildClicked` 在创建新 `DloSlamNode` 前调用 `clearSlamMap()`，并保持地图视图按钮状态一致，避免已加载历史关键帧与新会话未经配准直接叠加。

- [ ] **步骤 6：编译主程序**

运行：

```powershell
cmake --build build-radar-slam --target PointCloud -j 4
```

预期：主程序编译和链接成功。

### 任务 6：完整回归和交付验证

**文件：**

- 检查所有本计划修改文件。

- [ ] **步骤 1：运行新增测试**

```powershell
ctest --test-dir build-radar-slam -R "slam_map_io|slam_geo_anchor" --output-on-failure
```

预期：新增测试全部通过。

- [ ] **步骤 2：运行全部现有测试**

```powershell
ctest --test-dir build-radar-slam --output-on-failure
```

预期：零失败；若现有环境相关测试无法运行，记录确切测试名和输出，不掩盖失败。

- [ ] **步骤 3：重新构建主程序**

```powershell
cmake --build build-radar-slam --target PointCloud -j 4
```

预期：退出码为 0。

- [ ] **步骤 4：核对交付行为**

逐项确认：

- 新保存文件扩展名为 `.slammap`；
- 文件含有效版本、锚点和关键帧；
- 手动加载新版地图后由关键帧恢复累计点云；
- 旧 `.pcd` 仍可手动显示；
- `last_slam_map.txt` 只在成功保存/加载新版地图后更新；
- 启动恢复失败不阻止程序启动；
- 启动恢复成功后直接处于 SLAM 地图视图；
- 开始新建图时历史地图被清除。

当前目录不是 Git 仓库，因此本计划中的频繁 commit 步骤无法执行；不创建伪提交，也不初始化新仓库。

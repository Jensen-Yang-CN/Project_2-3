# SLAM 离线地图恢复与多会话拼接实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 恢复启动时自动显示最近保存的离线 SLAM 地图，并让现有“加载地图”入口支持多选 `.slammap`、WGS84/ENU 粗对齐、受限二维 ICP、来源分色显示及拼接归档再次保存。

**架构：** 将地理换算、地图拼接、异步文件加载和显示点云构建拆成可独立测试的组件。第一张地图作为基准；其余地图先按地理锚点平移到基准 ENU，再在可靠重叠区执行只估计 X/Y/偏航的 ICP，最后只变换关键帧位姿并合并完整归档。界面只在后台任务全部成功后整体替换当前地图。

**技术栈：** C++17、Qt 5 Widgets/Core/Concurrent、Eigen、PCL registration/filters、OpenGL、CMake、CTest

---

## 文件结构

- 创建：`src/slam/SlamMapGeoUtils.h`、`src/slam/SlamMapGeoUtils.cpp`——WGS84/ECEF/ENU 锚点换算。
- 创建：`src/slam/SlamMapStitcher.h`、`src/slam/SlamMapStitcher.cpp`——输入校验、粗对齐、二维 ICP、关键帧合并及诊断结果。
- 创建：`src/slam/SlamMapLoadTask.h`、`src/slam/SlamMapLoadTask.cpp`——后台读取一个或多个 `.slammap` 并调用拼接器。
- 创建：`src/slam/SlamMapDisplayBuilder.h`、`src/slam/SlamMapDisplayBuilder.cpp`——按来源范围和显示上限重建分色世界点云。
- 创建：`tests/slam_map_stitching_tests.cpp`——地理换算、拼接、ICP、文件任务、分色显示测试及界面接线守卫。
- 修改：`src/core/SlamMapBinaryIO.h`、`src/core/SlamMapBinaryIO.cpp`——移除旧版 PCD 接口与兼容分支，只接受新版归档。
- 修改：`tests/slam_map_io_tests.cpp`——把旧格式兼容测试改成非新版文件拒绝测试。
- 修改：`src/gui/MultiLidarWidget.h`、`src/gui/MultiLidarWidget.cpp`——接收已校验归档、整体更新显示、来源分色和拼接地图居中。
- 修改：`src/gui/mainwindow.h`、`src/gui/mainwindow.cpp`——恢复启动加载、多选入口、后台任务、日志和离线拼接结果保存。
- 修改：`CMakeLists.txt`——登记新组件与测试目标并链接 PCL registration/filters。

当前目录不是 Git 仓库，计划中的阶段结束以测试检查点代替 commit，不运行 `git add`、`git commit` 或 worktree 清理。

### 任务 1：建立基线并将地图格式收紧为 `.slammap`

**文件：**
- 修改：`tests/slam_map_io_tests.cpp`
- 修改：`src/core/SlamMapBinaryIO.h`
- 修改：`src/core/SlamMapBinaryIO.cpp`

- [x] **步骤 1：运行当前完整测试基线**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

预期：现有 16 项测试全部通过。

- [x] **步骤 2：先修改 I/O 测试，要求旧 PCD 内容被拒绝**

删除 `writeLegacyMap()` 和旧版成功加载断言，新增：

```cpp
void testNonArchiveContentRejected(int &failures)
{
    QTemporaryDir dir;
    if (!dir.isValid()) {
        ++failures;
        return;
    }
    const QString path = dir.filePath(QStringLiteral("legacy.pcd"));
    QFile file(path);
    check(file.open(QIODevice::WriteOnly), "fixture should open", failures);
    const uint32_t count = 1;
    const M_PointXYZI point = makePoint(1.0f, 2.0f, 3.0f, 4);
    file.write(reinterpret_cast<const char *>(&count), sizeof(count));
    file.write(reinterpret_cast<const char *>(&point), sizeof(point));
    file.close();

    slam_map_io::MapArchive loaded;
    QString error;
    check(!slam_map_io::loadArchive(path, loaded, &error)
              && error.contains(QStringLiteral("魔数")),
          "legacy point-only maps must be rejected", failures);
}
```

同时把 `main()` 中的旧测试调用替换为 `testNonArchiveContentRejected(failures)`。

- [x] **步骤 3：运行测试验证红灯**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_map_io_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R '^slam_map_io$' --output-on-failure
```

预期：FAIL，旧内容仍被当前 `loadArchive()` 当作旧格式地图成功加载。

- [x] **步骤 4：移除旧地图数据结构和接口**

将 `MapArchive` 收紧为：

```cpp
struct MapArchive {
    usv::SlamGeoAnchor anchor;
    std::vector<usv::SlamKeyframe> keyframes;
};
```

从头文件删除：

```cpp
bool save(const QString &, const QVector<M_PointXYZI> &, QString *);
bool load(const QString &, QVector<M_PointXYZI> &, QString *);
```

从实现删除两个旧格式函数；`loadArchive()` 改为只接受 `USVSLM2`：

```cpp
bool loadArchive(const QString &path, MapArchive &archive, QString *error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法打开文件: %1 (%2)")
                               .arg(path, file.errorString()));
    if (file.peek(kMagic.size()) != kMagic)
        return fail(error, QStringLiteral("地图文件魔数无效，仅支持新版 .slammap"));
    return loadVersion2(file, archive, error);
}
```

删除 `loadVersion2()` 中对 `legacy_format` 的赋值。

- [x] **步骤 5：运行 I/O 测试验证绿灯**

运行任务 1 步骤 3 的构建和测试命令。

预期：`slam_map_io` 通过。

### 任务 2：实现 WGS84 锚点换算与关键帧粗拼接

**文件：**
- 创建：`src/slam/SlamMapGeoUtils.h`
- 创建：`src/slam/SlamMapGeoUtils.cpp`
- 创建：`src/slam/SlamMapStitcher.h`
- 创建：`src/slam/SlamMapStitcher.cpp`
- 创建：`tests/slam_map_stitching_tests.cpp`
- 修改：`CMakeLists.txt`

- [x] **步骤 1：创建拼接测试目标和失败测试**

测试先定义期望接口：

```cpp
#include "SlamMapGeoUtils.h"
#include "SlamMapStitcher.h"

void testAnchorOffsetAndCoarseMerge(int &failures)
{
    usv::SlamGeoAnchor base{true, 1.0, 0.0, 0.0, 3.0};
    usv::SlamGeoAnchor east{true, 2.0, 0.0, 0.0001, 6.0};
    Eigen::Vector3d offset;
    QString error;
    check(slam_map_geo::anchorOffsetEnu(base, east, offset, &error),
          "valid anchors should convert", failures);
    check(close(offset.x(), 11.131949, 0.02)
              && close(offset.y(), 0.0, 0.02)
              && close(offset.z(), 3.0, 0.02),
          "WGS84 offset should be east/north/up", failures);

    slam_map_stitch::Input a{QStringLiteral("a.slammap"), makeArchive(base, 5.0, 0.0)};
    slam_map_stitch::Input b{QStringLiteral("b.slammap"), makeArchive(east, 2.0, 1.0)};
    slam_map_stitch::Options options;
    options.enable_icp = false;
    const auto result = slam_map_stitch::stitch({a, b}, options);
    check(result.success && result.archive.keyframes.size() == 2,
          "coarse stitching should merge both archives", failures);
    check(result.archive.keyframes[0].keyframe_id == 0
              && result.archive.keyframes[1].keyframe_id == 1,
          "merged keyframes should receive unique sequential ids", failures);
    check(close(result.archive.keyframes[1].pose.translation().x(),
                13.131949, 0.03),
          "source keyframe pose should move into base ENU", failures);
}
```

在 `CMakeLists.txt` 的 `if(BUILD_TESTING AND SLAM_ENABLED)` 中登记 `slam_map_stitching_tests`，包含新源文件并链接 `Qt5::Core`、`Eigen3::Eigen`、`${PCL_LIBRARIES}`。

- [x] **步骤 2：构建并验证红灯**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake -S D:\1\yuanshibanben\PointCloud20260514\PointCloud -B D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_map_stitching_tests -j 2
```

预期：构建失败，`SlamMapGeoUtils.h` 和 `SlamMapStitcher.h` 尚不存在。

- [x] **步骤 3：实现地理换算接口**

头文件接口：

```cpp
namespace slam_map_geo {
bool validAnchor(const usv::SlamGeoAnchor &anchor);
bool anchorOffsetEnu(const usv::SlamGeoAnchor &base,
                     const usv::SlamGeoAnchor &source,
                     Eigen::Vector3d &offset,
                     QString *error = nullptr);
}
```

实现使用 WGS84 常数 `a=6378137.0`、`f=1/298.257223563`，经纬高先转 ECEF，再以基准纬度和经度旋转为 ENU。所有输入和输出必须检查 `std::isfinite`。

- [x] **步骤 4：实现拼接公共类型和粗合并**

`SlamMapStitcher.h` 定义：

```cpp
namespace slam_map_stitch {
enum class AlignmentMode { Base, AnchorOnly, AnchorAndIcp };

struct Options {
    bool enable_icp = true;
    double voxel_size_m = 1.0;
    double overlap_padding_m = 10.0;
    int min_overlap_points = 500;
    int max_registration_points = 50000;
    int max_iterations = 50;
    double max_correspondence_m = 5.0;
    double max_correction_translation_m = 10.0;
    double max_correction_yaw_deg = 5.0;
    double max_fitness_m2 = 2.25;
};

struct Input {
    QString source_path;
    slam_map_io::MapArchive archive;
};

struct SourceRange {
    int first_keyframe = 0;
    int keyframe_count = 0;
    uint8_t display_intensity = 90;
};

struct Diagnostic {
    QString source_path;
    AlignmentMode mode = AlignmentMode::Base;
    Eigen::Vector3d anchor_offset_enu = Eigen::Vector3d::Zero();
    int source_overlap_points = 0;
    int target_overlap_points = 0;
    bool icp_converged = false;
    double fitness_m2 = 0.0;
    double correction_translation_m = 0.0;
    double correction_yaw_deg = 0.0;
    QString message;
};

struct Result {
    bool success = false;
    QString error;
    slam_map_io::MapArchive archive;
    QVector<SourceRange> source_ranges;
    QVector<Diagnostic> diagnostics;
};

Result stitch(const QVector<Input> &inputs,
              const Options &options = Options{});
}
```

粗合并阶段校验所有归档和锚点，第一张为基准；对后续关键帧执行 `T_base_from_source * keyframe.pose`，保持局部点云和时间戳，重新分配连续 `keyframe_id`。来源颜色依次使用 `{90, 130, 30, 180}`。

- [x] **步骤 5：运行粗拼接测试验证绿灯**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_map_stitching_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R '^slam_map_stitching$' --output-on-failure
```

预期：地理换算和关闭 ICP 的粗拼接测试通过。

### 任务 3：实现重叠提取和受限二维 ICP

**文件：**
- 修改：`tests/slam_map_stitching_tests.cpp`
- 修改：`src/slam/SlamMapStitcher.cpp`
- 修改：`CMakeLists.txt`

- [x] **步骤 1：编写可恢复小平移和偏航的失败测试**

生成不少于 1200 个非对称 L 形/波浪形 XY 点。地图 B 的点使用已知修正矩阵逆变换，锚点与 A 相同：

```cpp
void testBoundedIcpCorrection(int &failures)
{
    const Eigen::Isometry3d expected = planarTransform(1.2, -0.6, 2.0);
    const auto base = makeSyntheticArchive(makeAnchor(), makeAsymmetricPoints());
    const auto source = makeSyntheticArchive(
        makeAnchor(), transformPoints(makeAsymmetricPoints(), expected.inverse()));
    const auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), base},
        {QStringLiteral("source.slammap"), source}});
    check(result.success
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorAndIcp,
          "reliable overlap should use ICP", failures);
    check(close(result.diagnostics[1].correction_translation_m,
                std::hypot(1.2, 0.6), 0.15)
              && close(result.diagnostics[1].correction_yaw_deg, 2.0, 0.2),
          "ICP should recover the bounded planar error", failures);
}
```

- [x] **步骤 2：编写重叠不足和超界修正回退测试**

```cpp
void testIcpFallbacks(int &failures)
{
    auto sparse = makeSyntheticArchive(makeAnchor(), makeAsymmetricPoints(100));
    auto result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), sparse},
        {QStringLiteral("sparse.slammap"), sparse}});
    check(result.success
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "insufficient overlap should keep anchor alignment", failures);

    slam_map_stitch::Options options;
    options.max_correspondence_m = 20.0;
    auto shifted = makeSyntheticArchive(
        makeAnchor(), transformPoints(makeAsymmetricPoints(),
                                      planarTransform(12.0, 0.0, 0.0).inverse()));
    result = slam_map_stitch::stitch({
        {QStringLiteral("base.slammap"), makeSyntheticArchive(
             makeAnchor(), makeAsymmetricPoints())},
        {QStringLiteral("far.slammap"), shifted}}, options);
    check(result.success
              && result.diagnostics[1].mode
                     == slam_map_stitch::AlignmentMode::AnchorOnly,
          "correction beyond ten metres should be rejected", failures);
}
```

- [x] **步骤 3：运行测试验证红灯**

运行任务 2 步骤 5 的命令。

预期：小误差仍为 `AnchorOnly`，因为 ICP 尚未实现。

- [x] **步骤 4：实现体素降采样、XY 重叠提取和二维 ICP**

实现顺序：

1. 用关键帧位姿重建当前合并 target 和粗对齐 source 世界点；
2. 使用 `pcl::VoxelGrid<pcl::PointXYZ>` 做 1 米体素降采样；
3. 计算 XY 包围盒交集并扩展 10 米；
4. 过滤交集点，每侧少于 500 点则写诊断并回退；
5. 超过 50000 点时等间隔抽样；
6. 使用 `pcl::IterativeClosestPoint`，将 transformation estimation 设置为 `pcl::registration::TransformationEstimation2D`；
7. 从最终矩阵提取 XY 平移、偏航和 fitness；
8. 按 10 米、5°、`2.25 m²` 和有限值条件决定是否接受；
9. 接受时使用 `T_icp * T_coarse` 变换关键帧，拒绝时只用 `T_coarse`。

在 CMake 为拼接源文件和测试目标添加 `${PCL_INCLUDE_DIRS}`、`PCL_SILENCE_MALLOC_WARNING=1` 和 `${PCL_LIBRARIES}`。

- [x] **步骤 5：运行拼接测试验证绿灯**

运行任务 2 步骤 5 的命令。

预期：可靠 ICP、重叠不足回退和超界回退全部通过。

### 任务 4：实现后台文件任务和来源分色显示点云构建

**文件：**
- 创建：`src/slam/SlamMapLoadTask.h`
- 创建：`src/slam/SlamMapLoadTask.cpp`
- 创建：`src/slam/SlamMapDisplayBuilder.h`
- 创建：`src/slam/SlamMapDisplayBuilder.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`
- 修改：`CMakeLists.txt`

- [x] **步骤 1：编写文件任务与分色构建失败测试**

公共接口：

```cpp
namespace slam_map_load {
struct Result {
    bool success = false;
    QString error;
    slam_map_stitch::Result stitched;
};
Result loadAndStitch(const QStringList &paths,
                     const slam_map_stitch::Options &options = {});
}

namespace slam_map_display {
QVector<M_PointXYZI> rebuild(
    const slam_map_io::MapArchive &archive,
    const QVector<slam_map_stitch::SourceRange> &ranges,
    int max_points);
}
```

测试使用 `QTemporaryDir` 保存两张归档，调用 `loadAndStitch()` 并断言成功；再调用 `rebuild()`，断言第一段点强度为 90、第二段为 130、点数不超过上限。

- [x] **步骤 2：构建并验证红灯**

运行拼接测试目标构建。

预期：缺少 `SlamMapLoadTask.h` 或 `SlamMapDisplayBuilder.h` 导致构建失败。

- [x] **步骤 3：实现文件读取任务**

`loadAndStitch()` 必须：

- 拒绝空路径列表；
- 拒绝非 `.slammap` 后缀；
- 依次通过 `loadArchive()` 读取到临时 `Input`；
- 任一读取失败时返回带路径的错误且不产生部分结果；
- 单文件也调用 `stitch()`，得到统一的一段 `SourceRange`；
- 多文件调用同一拼接器。

- [x] **步骤 4：实现按关键帧来源分色的显示构建**

`rebuild()` 按总点数计算全局抽样步长，遍历关键帧并查找其所属 `SourceRange`，执行：

```cpp
const Eigen::Vector3d world = keyframe.pose
    * Eigen::Vector3d(point.x, point.y, point.z);
M_PointXYZI output{};
output.x = static_cast<float>(world.x());
output.y = static_cast<float>(world.y());
output.z = static_cast<float>(world.z());
output.intensity = range.display_intensity;
```

无来源范围的普通单地图使用强度 90。输出不得超过 `max_points`。

- [x] **步骤 5：运行拼接测试验证绿灯**

运行任务 2 步骤 5 的命令。

预期：文件任务与分色构建测试通过。

### 任务 5：让 OpenGL 控件原子显示完整归档

**文件：**
- 修改：`src/gui/MultiLidarWidget.h`
- 修改：`src/gui/MultiLidarWidget.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：增加界面接线源代码守卫并验证红灯**

测试读取 `MultiLidarWidget.cpp`，压缩空白后断言存在：

```cpp
check(compact.find("slam_map_display::rebuild(archive,sourceRanges,kMaxSlamMapPoints)")
          != std::string::npos,
      "widget should rebuild an archive with source colours", failures);
check(compact.find("m_viewPanX=0.5f*(minX+maxX);") != std::string::npos,
      "stitched maps should initially centre on their bounds", failures);
check(compact.find("archive.legacy_format") == std::string::npos,
      "widget must not retain legacy PCD handling", failures);
```

运行拼接测试，预期 FAIL，控件仍自行读取文件且包含旧格式分支。

- [x] **步骤 2：替换文件加载接口为归档显示接口**

头文件公开：

```cpp
bool displaySlamArchive(
    const slam_map_io::MapArchive &archive,
    const QVector<slam_map_stitch::SourceRange> &sourceRanges,
    QString *errorMsg = nullptr);
```

删除 `loadSlamMap(const QString &, ...)`。`displaySlamArchive()` 先在局部变量中完成：

- 分色世界点云重建；
- 关键帧轨迹重建；
- 最后关键帧船体位姿计算；
- XY 包围盒计算。

任一步失败直接返回，成员保持原值；全部成功后一次性替换 `m_slamMapCloud`、`m_slamPath`、`m_slamGeoAnchor`、船体位姿和实时扫描。初始 `m_viewPanX/m_viewPanY` 使用 XY 包围盒中心，不使用最后一帧船位。

- [x] **步骤 3：保持实时关键帧累计路径不变**

`appendSlamKeyframe()`、`mergeKeyframeIntoMap()` 和实时 SLAM 泊位框逻辑继续工作；`clearSlamMap()` 仍清理归档显示、实时扫描、轨迹和泊位同步状态。

- [x] **步骤 4：运行拼接、SLAM 显示和地图 I/O 测试**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target slam_map_stitching_tests slam_display_tests slam_map_io_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R '^(slam_map_stitching|slam_display|slam_map_io)$' --output-on-failure
```

预期：3/3 通过。

### 任务 6：接入启动恢复、多选异步拼接和离线保存

**文件：**
- 修改：`src/gui/mainwindow.h`
- 修改：`src/gui/mainwindow.cpp`
- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：编写 MainWindow 数据流接线守卫并验证红灯**

源代码守卫断言：

```cpp
check(compact.find("QTimer::singleShot(0,this,&MainWindow::restoreLastSlamMap);")
          != std::string::npos,
      "startup should restore the last SLAM map", failures);
check(compact.find("QFileDialog::getOpenFileNames(") != std::string::npos,
      "map loading should allow multiple archives", failures);
check(compact.find("slam_map_load::loadAndStitch(filePaths)")
          != std::string::npos,
      "selected archives should use the background load task", failures);
check(compact.find("m_offlineSlamArchive=result.stitched.archive;")
          != std::string::npos,
      "a successful offline stitch should become saveable", failures);
check(compact.find("*.pcd") == std::string::npos,
      "the map file dialog must not advertise PCD", failures);
```

运行拼接测试，预期 FAIL。

- [x] **步骤 2：增加离线归档和后台任务生命周期成员**

`mainwindow.h` 增加：

```cpp
#include "SlamMapLoadTask.h"
#include <optional>

std::optional<slam_map_io::MapArchive> m_offlineSlamArchive;
QFutureWatcher<slam_map_load::Result> *m_slamMapLoadWatcher = nullptr;
bool m_slamMapLoadInProgress = false;

void startSlamMapLoad(const QStringList &filePaths);
void finishSlamMapLoad(const slam_map_load::Result &result,
                       const QStringList &filePaths);
```

析构时若 watcher 存在，只断开回调并等待任务自然结束；后台函数不得捕获 `this`。

- [x] **步骤 3：恢复启动自动加载**

在构造函数完成 `setupCommBus()` 后加入：

```cpp
QTimer::singleShot(0, this, &MainWindow::restoreLastSlamMap);
```

`restoreLastSlamMap()` 改为通过 `loadArchive()` 读取候选文件并调用 `displaySlamArchive(archive, {{0, count, 90}}, &error)`；成功后设置 `m_offlineSlamArchive`。失败只写日志。

- [x] **步骤 4：把“加载地图”改为多选异步任务**

文件选择器：

```cpp
const QStringList filePaths = QFileDialog::getOpenFileNames(
    this, tr("加载或拼接 SLAM 地图"), dir,
    tr("SLAM 地图 (*.slammap)"));
```

`startSlamMapLoad()` 禁用加载和保存按钮，创建 `QFutureWatcher`，使用：

```cpp
const QFuture<slam_map_load::Result> future = QtConcurrent::run(
    [filePaths]() { return slam_map_load::loadAndStitch(filePaths); });
```

完成后先调用 `displaySlamArchive()`；只有显示成功才赋值 `m_offlineSlamArchive`、切换 SLAM 视图、记录最近地图（仅单文件原路径；多文件尚未保存时不改最近记录）并逐条输出诊断。无论成功失败都恢复按钮并删除 watcher。

- [x] **步骤 5：让保存按钮支持离线拼接归档**

保存数据优先级：

```cpp
slam_map_io::MapArchive archive;
#ifdef ENABLE_SLAM
if (slam_node_ && !slam_node_->getKeyframes().empty()) {
    archive.anchor = slam_node_->getGeoAnchor();
    archive.keyframes = slam_node_->getKeyframes();
} else
#endif
if (m_offlineSlamArchive.has_value()) {
    archive = *m_offlineSlamArchive;
}
```

实时建图开始前清空 `m_offlineSlamArchive`，避免旧离线归档被误保存；手动清空地图时同步清空。

- [x] **步骤 6：运行专项测试验证绿灯**

运行任务 5 步骤 4 的命令。

预期：启动恢复、多选拼接、PCD 移除和离线保存接线守卫通过。

### 任务 7：全量构建、验收和交付

**文件：**
- 修改：`CMakeLists.txt`
- 验证：`D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\PointCloud.exe`

- [x] **步骤 1：确认主目标登记全部新文件**

把以下文件加入 `SLAM_SRCS` 或不依赖 PCL的核心源列表：

```cmake
src/slam/SlamMapGeoUtils.h src/slam/SlamMapGeoUtils.cpp
src/slam/SlamMapStitcher.h src/slam/SlamMapStitcher.cpp
src/slam/SlamMapLoadTask.h src/slam/SlamMapLoadTask.cpp
src/slam/SlamMapDisplayBuilder.h src/slam/SlamMapDisplayBuilder.cpp
```

`SlamMapStitcher.cpp` 设置 PCL include 和 `PCL_SILENCE_MALLOC_WARNING=1`，主目标继续链接 `${PCL_LIBRARIES}`。

- [x] **步骤 2：构建主程序**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud -j 2
```

预期：退出码 0，更新 `PointCloud.exe`。

- [x] **步骤 3：运行完整 CTest**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;' + $env:Path
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

预期：包括新增 `slam_map_stitching` 在内的全部测试通过。

- [x] **步骤 4：静态验收行为边界**

使用 `rg` 确认：

```powershell
rg -n "restoreLastSlamMap|getOpenFileNames|loadAndStitch|m_offlineSlamArchive" src/gui/mainwindow.cpp src/gui/mainwindow.h
rg -n "legacy_format|legacy_world_cloud|旧版 SLAM 点云|\*\.pcd" src tests
```

预期：第一条命中新数据流；第二条无命中。

- [x] **步骤 5：记录人工 PCAP 验收边界**

自动测试和构建通过后明确报告：代码已验证 WGS84、合成点云 ICP、保存重载和界面接线；若本轮没有两张真实相邻 `.slammap`，不得声称已经完成真实地图视觉验收。交付时请用户用两个相邻 PCAP 分别生成地图，多选加载并观察日志中的 ICP/锚点回退结果。

## 规格覆盖自检

- 启动自动恢复：任务 6 步骤 3。
- 只支持 `.slammap`、移除 PCD：任务 1 与任务 6 步骤 4。
- WGS84/ENU 粗对齐：任务 2。
- 重叠区域与受限二维 ICP：任务 3。
- 完整关键帧合并、编号重排和保存重载：任务 2、4、6。
- 来源分色和拼接地图居中：任务 4、5。
- 后台执行、失败不覆盖当前地图：任务 4、5、6。
- 全量测试和主程序构建：任务 7。

计划中没有未定义接口、占位实现或超出规格的栅格地图功能。

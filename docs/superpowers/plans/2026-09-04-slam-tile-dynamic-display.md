# 统一 ENU 离线地图只读分块动态显示实现计划

> **面向 AI 代理的工作者：** 使用测试驱动开发逐任务实现本计划。当前目录不是 Git 仓库，因此保留每个任务的验证检查点，但不执行提交步骤。

**目标：** 将统一 ENU 的 `.slammap` 派生为只读分块缓存，并在界面中根据视野异步加载和渲染可见 Tile，避免每次全量加载和常驻整张点云。

**架构：** 首次加载继续使用现有地理校正与拼接得到统一 ENU `MapArchive`，随后由 Builder 生成版本化 Manifest、轨迹和三级 LOD Tile。后续加载直接打开 Manifest，由 Manager 计算 Active/Prefetch 集合并异步读取 Tile，GUI 线程维护每 Tile OpenGL VBO。

**技术栈：** C++17、Qt 5.12 Core/Concurrent/OpenGL、Eigen、现有 CMake/CTest。

---

## 文件结构

**创建：**

- `src/slam/SlamTileMapTypes.h`：Tile ID、Manifest、点数据、区域集合和哈希。
- `src/slam/SlamTileMapIO.h/.cpp`：Manifest、Tile、轨迹读写与缓存指纹。
- `src/slam/SlamTileMapBuilder.h/.cpp`：从统一 ENU `MapArchive` 构建三级 LOD。
- `src/slam/SlamTileMapManager.h/.cpp`：视口选择、异步加载、请求代号和 RAM LRU。
- `tests/slam_tile_map_tests.cpp`：格式、分块、LOD、缓存失效测试。
- `tests/slam_tile_cache_tests.cpp`：集合、LRU和过期请求测试。

**修改：**

- `src/core/AppConfig.h/.cpp`、`config/config.json`：动态地图参数。
- `src/slam/SlamMapLoadTask.h/.cpp`：缓存命中或一次性构建入口。
- `src/gui/MultiLidarWidget.h/.cpp`：视口通知和每 Tile VBO。
- `src/gui/mainwindow.h/.cpp`：生命周期、日志和只读加载调度。
- `CMakeLists.txt`：生产源文件与测试目标。

## 任务 1 Tile 纯数据模型与空间选择

- [ ] 新建 `tests/slam_tile_cache_tests.cpp`，先定义并测试以下行为：
  - `tileIndex(-0.01, 0, 50)` 返回 `-1`，零和正边界使用半开区间；
  - 视口矩形映射到确定的 Active Tile 集合；
  - Prefetch 为 Active 外扩一圈；
  - 地图实例代号或请求代号不匹配时拒绝结果；
  - Active Tile 在内存预算淘汰中保持 pinned。
- [ ] 在 `CMakeLists.txt` 中添加测试目标并运行，确认因缺少 `SlamTileMapTypes.h` 失败。
- [ ] 创建 `SlamTileMapTypes.h` 与 Manager 的纯算法接口，使用值类型和不可变 `shared_ptr<const TileData>`。
- [ ] 实现最小 Tile 索引、集合扩张、请求验证和按字节 LRU，使测试通过。
- [ ] 运行 `slam_tile_cache_tests` 和已有 `slam_display_tests`。

## 任务 2 Manifest 和 Tile 二进制格式

- [ ] 新建 `tests/slam_tile_map_tests.cpp`，编写失败测试：
  - Manifest JSON 往返保留锚点、边界、LOD和源文件顺序；
  - Tile 二进制往返保留 Tile 局部点、强度和元数据；
  - 错误魔数、错误版本、截断内容和校验值不一致被拒绝；
  - 源文件大小、修改时间、顺序或参数变化导致指纹变化；
  - 读取失败不覆盖调用方已有对象。
- [ ] 添加空实现目标并运行，确认测试因功能缺失失败。
- [ ] 实现 `SlamTileMapIO`：
  - Manifest 使用 `QJsonDocument`；
  - Tile 使用 Little Endian `QDataStream`；
  - 记录使用显式 float/uint8 字段，不直接写结构体内存；
  - 使用 `QCryptographicHash::Sha256` 计算缓存键和内容校验；
  - 使用 `QSaveFile` 写单文件，Builder 使用临时目录后提交。
- [ ] 运行两个新测试目标，确认通过。

## 任务 3 统一 ENU Tile Builder

- [ ] 在 `slam_tile_map_tests.cpp` 添加失败测试：
  - 关键帧局部点按 `pose` 转换成世界 ENU；
  - 正负坐标进入正确 Tile；
  - 同一全局体素重复点只保留第一个；
  - L0、L1、L2 点数随体素变大单调不增加；
  - 转换后 Manifest 锚点与输入归档一致；
  - 构建失败不留下有效 Manifest。
- [ ] 实现 `SlamTileMapBuilder`：
  - 默认 50 米 Tile；
  - LOD 为 1.0、0.3、0.1 米；
  - 逐关键帧流式投影到分块暂存，不创建完整世界点云；
  - 使用全局体素键确定性去重；
  - 写入各 LOD Tile、轨迹和 Manifest；
  - 完成后原子发布缓存目录。
- [ ] 运行 `slam_tile_map_tests`、`slam_map_io_tests` 和 `slam_map_stitching_tests`。

## 任务 4 只读动态加载 Manager

- [ ] 扩展 `slam_tile_cache_tests.cpp`，先写失败测试：
  - Active、Prefetch、Unload 状态转换；
  - 重复请求只读取一次；
  - 新地图代号使旧结果失效；
  - RAM 使用量超过预算时按 LRU 淘汰非 Active Tile；
  - Active 总量超过预算时建议降级 LOD；
  - Tile 失败可在下一代请求中重试。
- [ ] 实现 `SlamTileMapManager`：
  - `openManifest()` 清空旧状态并发布新地图信息；
  - `updateView()` 以 120 ms 防抖计算集合；
  - 使用 `QThreadPool/QtConcurrent` 读取 Tile；
  - 信号携带 map generation、request generation 和 Tile ID；
  - 缓存按字节预算管理并输出统计；
  - 析构和关闭时停止接受结果。
- [ ] 运行 Tile 两个测试目标并执行 50 次重复运行，检查竞态和不稳定失败。

## 任务 5 AppConfig 和缓存加载任务

- [ ] 为 AppConfig 补充配置读取测试或纯值约束测试，先验证缺少接口导致失败。
- [ ] 在 `config/config.json` 添加：Tile尺寸、LOD、预取环、卸载环、RAM预算、GPU点预算和防抖时间。
- [ ] 在 `AppConfig.h/.cpp` 中读取并夹紧安全范围。
- [ ] 扩展 `SlamMapLoadTask`：
  - 根据源文件和参数计算缓存键；
  - 有效缓存直接返回 Manifest 路径；
  - 缓存无效时沿用 `loadAndStitch()`，调用 Builder 后释放完整归档；
  - 结果区分缓存命中、缓存新建和错误阶段。
- [ ] 测试首次构建、第二次命中、源文件变化重建和失败不污染旧缓存。

## 任务 6 MultiLidarWidget 每 Tile GPU 渲染

- [ ] 添加可独立测试的视口 ENU 计算函数，先写失败测试覆盖宽高比、平移和缩放。
- [ ] 修改 `MultiLidarWidget`：
  - 离线 Tile 图层与实时累计层分离；
  - 接收 Manifest、Tile ready 和 Tile remove；
  - GUI/OpenGL线程创建与销毁每 Tile VBO；
  - 每帧仅绘制当前可见 Tile；
  - 总点数超过GPU预算时请求更粗LOD；
  - 滚轮、拖动、窗口尺寸变化后发送视口更新；
  - `clearSlamMap()` 清空旧 Tile 和GPU资源；
  - 保留实时扫描、轨迹、USV和泊位框绘制顺序。
- [ ] 运行 `slam_display_tests`、`slam_berth_overlay_tests` 和新视口测试。

## 任务 7 MainWindow 集成和用户行为

- [ ] 编写加载状态行为测试或静态接线守卫，验证：
  - 新加载开始时立即关闭旧 Manager；
  - 旧 watcher 结果不能更新新地图；
  - Tile 模式不保留完整 `m_offlineSlamArchive`；
  - Tile 模式下保存按钮不会重建完整地图；
  - 启动时仍不调用历史地图恢复。
- [ ] 修改 `MainWindow`：
  - “加载地图”继续支持一张或多张 `.slammap`；
  - 启动缓存加载任务并显示阶段日志；
  - 成功后创建/打开 `SlamTileMapManager`，连接Widget；
  - 新任务开始时取消旧请求并清空旧离线层；
  - 显示缓存命中、LOD、Tile数、RAM和加载耗时；
  - Tile只读模式不向 `m_offlineSlamArchive` 保存完整归档；
  - 实时SLAM保存路径保持原行为。
- [ ] 手工检查加载失败、新旧地图切换、实时/SLAM视图切换和泊位回显。

## 任务 8 全量验证与性能验收

- [ ] 配置并构建 Release `PointCloud.exe`。
- [ ] 运行全部 CTest，记录通过数和失败数。
- [ ] 使用一份已有 `.slammap` 验证首次转换和第二次缓存命中。
- [ ] 记录 Manifest Tile数量、各LOD点数、缓存大小、首次转换时间和再次加载时间。
- [ ] 拖动、缩放并观察 Active/Prefetch/Unload 日志，确认旧请求不混入。
- [ ] 加载第二组地图，确认第一组 Tile 立即清除。
- [ ] 检查常驻 RAM 不超过配置预算，GPU点数不超过150万。
- [ ] 核对源 `.slammap` 大小和修改时间未改变。
- [ ] 对照规格逐项确认，不满足项必须修复后重新执行对应验证。

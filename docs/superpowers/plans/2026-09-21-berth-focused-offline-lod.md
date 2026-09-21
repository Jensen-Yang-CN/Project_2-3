# 泊位区域高密度离线地图投递实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让历史离线地图投递时，泊位相交 Tile 使用 `0.1 m` LOD，其他 Tile 使用 `0.3 m` LOD，同时保持实时地图、泊位检测和 0xFB 不变。

**架构：** Tile 缓存继续生成多级 LOD；`HistoricalMapExportSource` 根据 manifest 中保存的泊位几何，为每个 XY Tile 坐标选择一个发送 LOD。历史整图和实时关键帧重叠区域复用同一选择结果，避免重复逻辑。配置和历史包分辨率统一为 `1.0/0.3/0.1 m` 与 `0.1 m`。

**技术栈：** C++17、Qt 5、Eigen、现有 CMake 单元测试和 UDP 投递节点。

---

### 任务 1：为区域化 Tile 选择编写失败测试

**文件：**
- 修改：`tests/historical_map_export_tests.cpp`
- 修改：`tests/perception_export_realtime_disabled_tests.cpp`

- [x] **步骤 1：扩展历史 Tile 测试夹具**

在现有 manifest 中设置 `lods={{0,0.3},{1,0.1}}`，保留一个位于泊位 Tile `(0,0)` 的 `0.1 m` Tile，并新增一个不与泊位相交的背景 Tile `(1,0)`，只写入 `0.3 m` Tile。把 manifest bounds 扩展到 100 m，并让泊位中心仍为 `(12,6)`、尺寸 `8×4`、角度 `15°`。

- [x] **步骤 2：添加选择行为断言**

在 `testOpenAndInitialStreamUsesFinestLod` 中改为断言：

```cpp
check(source.finestLod() == 1 && source.backgroundLod() == 0,
      "berth and background LODs should be selected", failures);
check(source.berthTileCount() == 1
          && source.backgroundTileCount() == 1
          && source.fullMapTileCount() == 2,
      "one berth tile and one background tile should be streamed", failures);
```

继续读取完整队列，确认高密度泊位点和背景点都出现，且没有重复的同一 XY Tile。

- [x] **步骤 3：更新历史分辨率策略断言**

在 `tests/perception_export_realtime_disabled_tests.cpp` 中把历史分辨率期望从 `0.05` 改为源码字符串 `kHistoricalMapResolutionM = 0.1`，并保留实时地图开关、历史泊位锚点和实时泊位路径断言。

- [x] **步骤 4：运行测试确认红灯**（受本机历史构建工具链缓存限制，改用临时 Qt5 语法检查；实现后策略测试通过）

运行：

```powershell
cmake --build build --target historical_map_export_tests perception_export_realtime_disabled_tests --config Debug
ctest --test-dir build -R "historical_map_export|perception_export_realtime_disabled" --output-on-failure
```

预期：历史 Tile 选择 API 尚不存在，或仍只返回全局最细 Tile；测试失败原因应对应新需求，而不是编译环境错误。

### 任务 2：实现按泊位区域选择历史 Tile

**文件：**
- 修改：`src/bus/HistoricalMapExportSource.h`
- 修改：`src/bus/HistoricalMapExportSource.cpp`

- [x] **步骤 1：增加选择状态和只读诊断接口**

保留旧方法名兼容现有调用，同时增加：

```cpp
int backgroundLod() const noexcept;
int berthTileCount() const noexcept;
int backgroundTileCount() const noexcept;
```

将 `finest_tiles_` 改为 `selected_tiles_`，增加 `berth_lod_`、`background_lod_` 和两个计数器。`fullMapTileCount()` 返回实际选中的 Tile 数量。

- [x] **步骤 2：实现泊位旋转矩形 ROI**

在 `.cpp` 匿名命名空间增加基于 `SlamMapBerth::{x,y,width,length,angle_deg}` 的 XY 包围盒计算：旋转四个角，取最小/最大 X/Y，再增加 `2.0 m` 安全边界。无效尺寸的泊位跳过 ROI，不影响地图加载。

- [x] **步骤 3：实现 LOD 选择**

`openManifest` 找到最小体素尺寸作为泊位 LOD，找到其后第一个更粗的 LOD 作为背景 LOD；若只有一个 LOD，则两者回退到同一个可用级别。按 `(TileId.x, TileId.y)` 聚合 manifest Tile：XY Tile 与任何泊位 ROI 相交就选泊位 LOD，否则选背景 LOD。按 TileId 排序后写入 `selected_tiles_`，避免同一 XY Tile 重复发送。

- [x] **步骤 4：让整图和重叠区域共用选中 Tile**

`takeNextFullMapTile` 从 `selected_tiles_` 读取。`makeOverlapVirtualKeyframe` 遍历 `selected_tiles_`，用 Tile 原点和 `manifest.tile_size_m` 判断与实时重叠 bounds 相交，再读取相应 Tile；不再固定调用全局 `finest_lod_`。

- [x] **步骤 5：运行历史测试确认绿灯**（历史源和测试夹具已通过 Qt5 语法检查；完整链接受本机 Qt/MinGW 不匹配阻塞）

重新运行任务 1 的 CMake/CTest 命令，预期历史测试通过，并确认无泊位 manifest 时只选择背景 LOD。

### 任务 3：统一配置和历史 0xFC 栅格分辨率

**文件：**
- 修改：`config/config.json`
- 修改：`src/slam/SlamTileMapBuilder.h`（默认值已为 `0.1 m`，无需改动）
- 修改：`src/core/AppConfig.h`
- 修改：`src/bus/PerceptionExportNode.cpp`
- 修改：`src/bus/PerceptionExportNode.cpp`（历史地图加载日志）

- [x] **步骤 1：修改默认 LOD 列表**

把 `config/config.json` 的 `lod_voxel_sizes_m` 改为 `[1.0, 0.3, 0.1]`；同步把 `SlamTileMapBuilder::BuildOptions` 默认值改为 `{{0,1.0},{1,0.3},{2,0.1}}`，保证配置缺失时也不会回到旧的 `0.05 m`。

- [x] **步骤 2：修改历史包分辨率**

只把 `kHistoricalMapResolutionM` 从 `0.05` 改为 `0.1`。不得修改 `kDefaultResolutionM`、实时累积地图、实时扫描地图或 0xFB 代码。

- [x] **步骤 3：增加一次性选择日志**

在 `PerceptionExportNode::loadHistoricalMap` 成功打开 manifest 后发出日志，包含泊位 Tile 数、背景 Tile 数、泊位 LOD 和背景 LOD，格式保持中文检测日志风格，例如：

```text
[历史地图投递] LOD选择：泊位Tile=…（0.1 m），背景Tile=…（0.3 m）
```

- [x] **步骤 4：运行源码策略测试**

运行 `perception_export_realtime_disabled_tests`，确认实时投递仍为启用状态、历史泊位路径保留、历史分辨率为 `0.1`，且没有静态泊位层混入实时发送。

### 任务 4：完整验证和提交

**文件：**
- 验证：`src/bus/HistoricalMapExportSource.*`、`src/bus/PerceptionExportNode.cpp`、`src/slam/SlamTileMapBuilder.h`、`config/config.json`、相关测试

- [x] **步骤 1：运行所有相关测试**（策略测试通过；完整 Qt5 链接测试受本机工具链限制）

```powershell
cmake --build build --target historical_map_export_tests slam_tile_cache_tests slam_tile_map_tests perception_export_realtime_disabled_tests --config Debug
ctest --test-dir build -R "historical_map_export|slam_tile_cache|slam_tile_map|perception_export_realtime_disabled" --output-on-failure
```

- [x] **步骤 2：检查差异和范围**

运行 `git diff --check`、`git status --short` 和 `git diff --stat`，确认没有修改实时泊位、坐标转换、Ship 端或无关构建文件。

- [ ] **步骤 3：提交**

```powershell
git add config/config.json src/slam/SlamTileMapBuilder.h src/bus/HistoricalMapExportSource.h src/bus/HistoricalMapExportSource.cpp src/bus/PerceptionExportNode.cpp tests/historical_map_export_tests.cpp tests/perception_export_realtime_disabled_tests.cpp
git commit -m "feat: densify historical map around berths"
```

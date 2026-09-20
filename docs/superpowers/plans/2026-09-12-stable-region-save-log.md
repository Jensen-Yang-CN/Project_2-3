# 保存地图时稳定区域日志实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在 `.slammap` 保存成功后分析跨关键帧反复出现的稳定区域，并把汇总和前 10 个区域的明细输出到“检测信息”，不修改地图内容和 ICP。

**架构：** 在 `SlamMapDisplayBuilder` 中新增一个纯只读分析函数，按照 1.0 m XY 网格统计不同关键帧的重复观测，并用 8 邻域聚合相邻稳定网格。`MainWindow` 仅在保存成功后调用该函数并格式化日志；现有归档写入和 `SlamMapStitcher` 保持原样。

**技术栈：** C++17、Qt、Eigen、现有 SLAM 地图归档结构、现有 CMake 测试目标。

---

### 任务 1：用测试锁定稳定区域统计规则

**文件：**
- 修改：`tests/slam_map_stitching_tests.cpp`

- [x] **步骤 1：编写失败的测试**

新增测试数据，验证同一关键帧内的重复点只算一次观测、跨两个关键帧才成为稳定网格、相邻稳定网格合并、孤立小区域可过滤。

- [x] **步骤 2：运行测试验证失败**

运行：

```powershell
$env:Path='D:\msys64\ucrt64\bin;D:\CMake\bin;'+$env:Path
cmake --build build-radar-slam --target slam_map_stitching_tests -j 2
```

预期：编译失败，提示 `analyzeStableRegions` 或对应结果类型尚未定义。

### 任务 2：实现只读稳定区域分析

**文件：**
- 修改：`src/slam/SlamMapDisplayBuilder.h`
- 修改：`src/slam/SlamMapDisplayBuilder.cpp`

- [x] **步骤 1：声明结果结构和分析接口**

结果包含网格参数、稳定网格数、区域总数，以及每个区域的中心 ENU、XYZ 范围、面积、累计点数、观测关键帧数和重复观测统计。

- [x] **步骤 2：实现最少分析逻辑**

把每个关键帧点变换到地图坐标，按 1.0 m XY 网格累计；每个网格每帧最多增加一次观测。保留至少被 2 个关键帧看到的网格，用 8 邻域合并，过滤不足 5 格的小区域，并按面积从大到小排序。

- [x] **步骤 3：运行测试验证通过**

运行：

```powershell
cmake --build build-radar-slam --target slam_map_stitching_tests -j 2
.\build-radar-slam\slam_map_stitching_tests.exe
```

预期：输出 `All SLAM map stitching tests passed`。

### 任务 3：保存成功后输出检测信息日志

**文件：**
- 修改：`src/gui/mainwindow.cpp`

- [x] **步骤 1：接入保存后的分析调用**

在 `saveArchive` 成功之后调用只读分析接口，固定使用 1.0 m 网格、至少 2 帧、至少 5 格，并最多输出前 10 个区域。

- [x] **步骤 2：输出大白话日志**

先输出稳定网格和区域汇总，再逐区域输出中心 ENU、范围、估算面积、高度、累计点数、观测关键帧数和几何形态候选；明确“候选”仅按几何特征判断。

- [x] **步骤 3：构建完整程序**

运行：

```powershell
cmake --build build-radar-slam --target PointCloud -j 2
```

预期：构建完成且无编译错误。

### 任务 4：边界核对

**文件：**
- 检查：`src/slam/SlamMapBinaryIO.cpp`
- 检查：`src/slam/SlamMapStitcher.cpp`

- [x] **步骤 1：确认无越界修改**

确认本次差异没有改变 `.slammap` 序列化字段，也没有改变 ICP 的稳定点选择、配准参数或接受条件。

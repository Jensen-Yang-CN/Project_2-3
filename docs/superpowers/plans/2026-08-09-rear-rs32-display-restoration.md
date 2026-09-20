# 201 尾部 32 线雷达显示恢复实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 恢复目标 PCAP 中真实存在的 201 尾部 32 线雷达蓝色点云显示，同时保持 SLAM 和泊位检测只使用 210/211。

**架构：** 保持现有解包器、工作线程和外参转换不变，只修正显示融合策略。`RadarFusionManager` 生成包含 201/210/211/213/214 的显示点云，并独立生成只包含 210/211 的算法点云。

**技术栈：** C++17、Qt 5、CMake、CTest、现有 RoboSense 驱动

---

## 文件结构

- 修改：`src/modules/RadarFusionPolicy.h`——定义显示与算法雷达成员集合。
- 修改：`src/modules/radarfusionmanager.h`——同步信号语义注释。
- 修改：`src/modules/radarfusionmanager.cpp`——同步融合输出注释。
- 修改：`tests/radar_update_tests.cpp`——验证显示集合包含 201、算法集合排除 201。
- 修改：`tests/radar_fusion_output_tests.cpp`——验证真实融合输出包含 201 蓝色强度。

### 任务 1：建立 201 显示回归测试

**文件：**
- 修改：`tests/radar_update_tests.cpp`
- 修改：`tests/radar_fusion_output_tests.cpp`

- [x] **步骤 1：修改策略测试的预期集合**

```cpp
check(displayIds == QStringList({"210", "211", "201", "213", "214"}),
      "显示点云必须包含 201/210/211/213/214");
check(!algorithmIds.contains(QStringLiteral("201")),
      "201 只能进入显示输出，不得进入算法输出");
```

- [x] **步骤 2：修改融合输出测试的预期点数与强度**

```cpp
check(display.size() == 5,
      "display output must contain 201/210/211/213/214");
check(intensities(display) == QSet<int>({50, 100, 150, 200, 250}),
      "display output must include rear 201 with blue display intensity");
```

- [x] **步骤 3：运行测试并验证红灯**

运行：

```powershell
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target radar_update_tests radar_fusion_output_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "radar_update|radar_fusion_output" --output-on-failure
```

预期：测试因显示集合仍排除 201 而失败。

### 任务 2：恢复 201 显示成员

**文件：**
- 修改：`src/modules/RadarFusionPolicy.h`
- 修改：`src/modules/radarfusionmanager.h`
- 修改：`src/modules/radarfusionmanager.cpp`

- [x] **步骤 1：把 201 加入显示集合**

```cpp
static const QStringList ids = {
    QStringLiteral("210"),
    QStringLiteral("211"),
    QStringLiteral("201"),
    QStringLiteral("213"),
    QStringLiteral("214"),
};
```

- [x] **步骤 2：保持算法集合不变**

```cpp
static const QStringList ids = {
    QStringLiteral("210"),
    QStringLiteral("211"),
};
```

- [x] **步骤 3：更新输出语义注释**

显示信号注释写明 `201/210/211/213/214`；算法信号注释继续写明 `210/211`。

- [x] **步骤 4：运行专项测试并验证绿灯**

运行：

```powershell
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target radar_update_tests radar_fusion_output_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "radar_update|radar_fusion_output" --output-on-failure
```

预期：2/2 通过。

### 任务 3：回归验证与交付

**文件：**
- 验证：`CMakeLists.txt`
- 验证：`D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug\PointCloud.exe`

- [x] **步骤 1：运行全部自动化测试**

```powershell
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

预期：全部测试通过。

- [x] **步骤 2：构建主程序**

```powershell
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud -j 2
```

预期：退出码为 0，并生成最新 `PointCloud.exe`。

- [x] **步骤 3：核对数据流边界**

静态确认显示集合包含 201，算法集合不包含 201；确认 `mainwindow.cpp` 仍将 `displayCloudReady` 连接到点云窗口、将 `algorithmCloudReady` 连接到算法总线。

## 执行说明

当前工程没有 `.git` 目录，因此不执行 Git commit；所有修改直接落在当前工作区。

## 执行结果

- 指定 PCAP 前 35 秒检测到 `201:6699` 点云包 52,500 个、`201:7788` 配置包 31 个，未检测到 213/214 数据包。
- 红灯：`radar_update` 与 `radar_fusion_output` 均因 201 被排除而失败。
- 绿灯：修复后两项雷达专项测试 2/2 通过。
- 全量回归：CTest 15/15 通过。
- 主程序：`PointCloud` 目标构建退出码 0。

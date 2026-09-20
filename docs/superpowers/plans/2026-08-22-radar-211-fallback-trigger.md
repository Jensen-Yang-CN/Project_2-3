# 211 单雷达自适应融合触发实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让只有 211 雷达的 PCAP 正常输出显示与算法点云，同时避免双雷达数据产生重复融合帧。

**架构：** 保留 210 主触发策略，在 `RadarFusionManager` 内记录最近的 210 采集时间戳。211 仅在从未见过 210或210已落后超过0.5秒时作为备用主雷达触发，210一旦恢复就立即重新成为主触发。

**技术栈：** C++17、Qt 5、CMake、CTest

---

## 文件结构

- 修改：`tests/radar_fusion_output_tests.cpp`——增加 211 单路、避免重复触发、断流回退与 210 恢复测试。
- 修改：`src/modules/radarfusionmanager.h`——保存 210 活跃状态和回退阈值。
- 修改：`src/modules/radarfusionmanager.cpp`——实现 211 自适应回退触发并在播放跳转时复位状态。

### 任务 1：建立失败的融合触发回归测试

- [x] **步骤 1：增加仅 211 输入的测试**

连接两个输出信号，输入一帧 211，等待同步定时器后断言显示点云、算法点云各输出一次且时间戳为 211 时间戳。

- [x] **步骤 2：增加主雷达优先级测试**

先输入并完成一帧 210，再输入时间差小于0.5秒的211，断言输出次数不增加；随后输入时间差大于0.5秒的211，断言211接管输出；最后输入210，断言210恢复输出。

- [x] **步骤 3：运行专项测试验证红灯**

```powershell
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target radar_fusion_output_tests -j 2
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug -R "radar_fusion_output" --output-on-failure
```

预期：新增的211单路和断流回退断言失败，证明现有缺陷可复现。

### 任务 2：实现最小回退触发逻辑

- [x] **步骤 1：增加主雷达状态**

在管理器中增加“是否见过210”、最新210时间戳和0.5秒回退阈值，并在播放跳转复位。

- [x] **步骤 2：修改触发条件**

210始终触发；211仅在未见过210或其采集时间戳比最新210晚超过0.5秒时触发。其他雷达仍只入队，不成为主时钟。

- [x] **步骤 3：运行专项测试验证绿灯**

执行任务1的构建和CTest命令，预期 `radar_fusion_output` 通过。

### 任务 3：完整验证

- [x] **步骤 1：运行全部自动化测试**

```powershell
ctest --test-dir D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --output-on-failure
```

- [x] **步骤 2：构建主程序**

```powershell
cmake --build D:\1\yuanshibanben\PointCloud20260514\build-PointCloud-x86_windows_msys_pe_64bit-Debug --target PointCloud -j 2
```

- [x] **步骤 3：静态核对数据边界**

确认211备用触发仍经过原融合函数，显示输出继续包含显示雷达，算法输出继续只包含210/211。

## 执行说明

当前目录不是 Git 仓库，无法建立Git worktree或逐任务提交；所有修改直接在当前项目中内联执行。

## 执行结果

- 红灯：旧实现下 `radar_fusion_output` 出现5条预期失败断言，确认211单路无输出且无法在210断流后接管。
- 绿灯：实现自适应触发后，`radar_fusion_output` 专项测试通过。
- 全量回归：Debug CTest 19/19通过。
- 主程序：常用Debug目录中的程序因正在运行而无法覆盖；同一源码已在 `build-radar-slam` 独立Debug目录完整编译并链接成功。

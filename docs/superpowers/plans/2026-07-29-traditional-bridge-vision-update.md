# 最新传统视觉桥梁检测接入实现计划

> **面向 AI 代理的工作者：** 使用 executing-plans 在当前会话逐任务实现；当前目录不是 Git 仓库，不能使用 worktree 或逐任务 commit，先保存可恢复备份，并在每个阶段运行测试或构建。

**目标：** 按 `replace.zip/README_桥梁检测节点移植说明.md` 接入左右相机传统视觉桥梁检测、桥墩方位角和 `/perception/traditional_bridge_vision` Topic，同时保留现有泊位、雷达、SLAM 和可选 CUDA/PCL 桥洞检测。

**架构：** 仅复制 `add_or_replace` 的桥梁节点和传统算法主体；`merge_reference` 只提取必要接入点。左右相机各使用独立检测器状态，直接消费相机解码帧；公共类型、标定、相机别名和 UI 连接以最小差异合并。

**技术栈：** C++17、Qt5 Core/Widgets、OpenCV core/imgproc/video、现有 Topic 总线、CTest。

---

## 文件结构

- 新增 `src/bridge/TraditionalBridgeVisionDetector.h`。
- 新增 `src/bridge/traditional_bridge_algorithm.cpp`，仅由上面的头文件包含，不独立加入 CMake。
- 修改 `src/bus/BridgeDetectionNode.h/.cpp`：左右工作器、相机直接提交、视觉 Topic 与左右 overlay。
- 修改 `src/modules/bridge_detection_worker.h/.cpp`：传统算法、方位角、结构化结果和可选 CUDA/PCL 兼容。
- 修改 `src/bus/common_types_extended.h`：只增加传统视觉结果类型，不删除当前泊位字段。
- 修改 `src/core/CalibrationConfig.h/.cpp` 和 `config/calibration.json`：增加右相机标定，更新左相机参数。
- 修改 `config/devices.json`：合并 qiao1/qiao2 相机源 IP 别名；保留当前设备与雷达绑定配置。
- 新增 `config/devices_bridge1.json`、`config/devices_bridge2.json`。
- 修改 `src/gui/mainwindow.h/.cpp`：左右相机提交检测、双路 overlay、PCAP 别名分流。
- 修改 `CMakeLists.txt`：传统算法源展示、桥梁测试和配置部署；`ENABLE_BRIDGE_DETECT` 默认 OFF。
- 新增 `tests/bridge_vision_update_tests.cpp`。

### 任务 1：备份与失败测试

- [ ] 把将修改的桥梁、总线、标定、UI、配置和 CMake 文件压缩到：

```text
backups/bridge_update_before_20260729.zip
```

- [ ] 编写 `tests/bridge_vision_update_tests.cpp`，验证：

```cpp
check(sizeof(usv::TraditionalBridgeVisionBusResult) > 0,
      "传统视觉总线类型必须存在");
cv::Mat blank(720, 1280, CV_8UC3, cv::Scalar::all(0));
const auto result = detector.detect(blank);
check(result.overlay.size() == blank.size(),
      "传统检测必须返回同尺寸 BGRA 结果");
```

- [ ] 增加对左右相机标定读取的断言。
- [ ] 在 CMake 注册 `bridge_vision_update_tests`，但暂不加入生产实现。
- [ ] 构建测试：

```powershell
cmake --build build-radar-slam --target bridge_vision_update_tests -j 2
```

预期：因传统算法或公共类型尚不存在而失败。

### 任务 2：加入算法主体与公共契约

- [ ] 从压缩包精确加入：

```text
add_or_replace/src/bridge/TraditionalBridgeVisionDetector.h
add_or_replace/src/bridge/traditional_bridge_algorithm.cpp
```

- [ ] 在 `common_types_extended.h` 增加 `CameraSide`、`BridgePierObservation` 和 `TraditionalBridgeVisionBusResult`，不得替换整个参考文件。
- [ ] 在 `CalibrationConfig` 增加 `m_rightCamera`、`rightCamera()` 和 `right_camera` JSON 解析；保留当前 210/211/船体外参与雷达条目初始化。
- [ ] 将压缩包 `calibration.json` 的左、右相机内参与畸变参数合并到当前配置，保留当前其他外参。
- [ ] 运行桥梁测试，预期算法和标定契约通过。

### 任务 3：升级桥梁节点与工作器

- [ ] 以压缩包 `add_or_replace` 版本为基准更新 `BridgeDetectionNode` 和 `bridge_detection_worker`。
- [ ] 左右相机各使用独立 `TraditionalBridgeVisionDetector` 实例；每路提交间隔至少 125 ms。
- [ ] 每秒发布一次 `/perception/traditional_bridge_vision`，内容包括相机侧、桥墩像素中心、方位角、桥孔中心方位角、桥墩夹角和桥面倾角。
- [ ] 修正压缩包在 `ENABLE_BRIDGE_DETECT` 分支的提前返回：

```cpp
m_impl->detector.init();
m_impl->visionStatsTimer.start();
m_impl->initialized = true;
```

确保开启 CUDA/PCL 时传统视觉与点云桥洞检测都初始化，关闭时仅运行传统视觉。
- [ ] 不替换 `bridge_common_types.h`，避免删除当前 GNSS/INS 有效性字段；不替换相同的 `bridge_cuda.cu` 和 `LidarExtrinsic210To211.h`。

### 任务 4：接入双路相机、配置别名和 UI

- [ ] 在 `MainWindow::ipPortDispatch()` 为 `pcap_alias_ips` 注册右、左相机源 IP 规则。
- [ ] 把左相机帧提交为 `rightCamera=false`，新增右相机处理槽并提交为 `rightCamera=true`。
- [ ] 在线 RTSP 和离线 H265 两条链路都使用相同左右处理槽。
- [ ] 将 `bridgeOverlayReadyRight` 连接到右侧 `VideoWidget`；现有 `VideoWidget` 已支持带时间戳 overlay，无需整体替换参考文件。
- [ ] 在 `devices.json` 仅合并：

```json
"pcap_alias_ips": ["192.168.1.220", "192.168.58.60"]
```

和：

```json
"pcap_alias_ips": ["192.168.1.223", "192.168.58.61"]
```

保留当前中文流名称、RTSP、雷达和组播绑定。
- [ ] 加入 `devices_bridge1.json`、`devices_bridge2.json` 作为参考配置。

### 任务 5：CMake、构建和回归

- [ ] 将 `ENABLE_BRIDGE_DETECT` 默认值改为 `OFF`，现有 `build-radar-slam` 也保持 OFF。
- [ ] 把 `TraditionalBridgeVisionDetector.h` 加入主目标；不得把 `traditional_bridge_algorithm.cpp` 作为独立翻译单元。
- [ ] 确保传统模式链接 OpenCV core/imgproc/video 且不要求 CUDA。
- [ ] 将两个桥梁参考设备配置加入构建后复制。
- [ ] 构建并测试：

```powershell
cmake --build build-radar-slam --target bridge_vision_update_tests -j 2
ctest --test-dir build-radar-slam -R bridge_vision_update --output-on-failure
cmake --build build-radar-slam --target PointCloud -j 2
ctest --test-dir build-radar-slam --output-on-failure
```

- [ ] 静态核对：

```powershell
rg -n "D:/VScode/WORK_SPACE|traditional_bridge_algorithm.cpp" src CMakeLists.txt
rg -n "192.168.1.220|192.168.58.60|192.168.1.223|192.168.58.61" config src
```

预期：无外部绝对算法路径，算法 `.cpp` 只被头文件相对包含；主程序和全部测试通过。

# PointCloud 感知与 SLAM 系统

这是一个基于 Qt、C++、PCL、Eigen 和 OpenCV 的无人船多传感器感知程序，负责接收雷达、GNSS/INS、相机和 PCAP 回放数据，完成点云显示、泊位检测、SLAM 建图、地图保存/加载以及感知结果 UDP 投递。

## 项目功能

- 在线采集和 PCAP 离线回放。
- 激光雷达点云、双目/视频和导航数据的统一处理与可视化。
- 泊位检测：输出泊位四角、类型、开口方向、尺寸及地理坐标。
- DLO SLAM：根据激光点云和 GNSS/INS 建立 ENU 地图，保存为 `.slammap` 并支持重新加载。
- 历史地图投递：把 `.slammap` 中的点云和泊位通过 UDP 投递到艇载接收端。
- 实时地图投递：将实时 SLAM 点云按 0xFC 协议发送到接收端。
- 泊位结果投递：将泊位检测结果按 0xFB 协议发送到接收端。
- 固定泊位图层：可加载泊位地理信息 JSON，在接收端显示预先配置的泊位范围。
- 可选桥梁、障碍物和占据栅格处理模块。

## 主要目录

```text
PointCloud/
├─ common/          公共数据结构和总线消息定义
├─ config/          标定、设备和运行配置
├─ src/gui/         主窗口、多雷达视图和交互逻辑
├─ src/berth/       泊位检测算法
├─ src/bus/         通信总线、UDP 投递和协议封装
├─ src/slam/        DLO SLAM 节点和地图显示
├─ src/code-slam/   SLAM 算法实现
├─ src/protocol/    感知投递协议
├─ tools/           地图、PCAP 和地理坐标辅助脚本
├─ tests/           单元测试
├─ res/             图标、QML 和其他资源
└─ CMakeLists.txt   CMake 构建入口
```

## 核心文档

为避免多份旧说明重复或互相矛盾，当前推荐优先阅读 `docs/architecture/` 下的四份整理文档：

- [PCAP、视频、点云与泊位检测链路](docs/architecture/pcap_video_pointcloud_berth_flow.md)
- [SLAM 建图、坐标系、保存加载与界面渲染](docs/architecture/slam_map_coordinate_rendering.md)
- [实时/历史地图与泊位投递](docs/architecture/map_export_protocol.md)
- [运行操作、日志判据与代码索引](docs/architecture/operation_and_code_index.md)

这些文档按当前源码和实际演示流程校正了入口函数、关键帧参数、`.slammap` 泊位保存方式、历史 Tile 投递和 Ship 端边界；早期设计稿和实施计划仍保留在 `docs/superpowers/`，用于追溯开发过程，不作为当前运行手册。

## 依赖环境

- Windows 10/11
- Qt 5：Widgets、Core、Network、Concurrent、OpenGL
- CMake 3.x
- MinGW/MSYS2（项目当前使用 MSYS2 UCRT64/MinGW 工具链）
- Eigen 3.4.1
- PCL 1.15（启用 SLAM 或 PCL 感知功能时需要）
- OpenCV（桥梁检测等模块需要）
- 可选：CUDA 和对应编译工具链（启用桥梁检测时使用）

项目已经提供部分第三方头文件、库和运行时资源，但具体构建机仍需根据本机 Qt、PCL、OpenCV 和 MSYS2 安装路径调整 CMake 配置。

## CMake 构建

在项目根目录打开 Qt Creator，选择与依赖库 ABI 一致的 MinGW/MSYS2 Kit，然后配置并构建项目。也可以在终端执行：

```powershell
cmake -S . -B build-release -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Release `
  -DENABLE_SLAM=ON `
  -DENABLE_PCL_PERCEPTION=ON `
  -DENABLE_BRIDGE_DETECT=OFF

cmake --build build-release --config Release -j 4
```

如果本机没有正确发现 PCL、Qt 或 OpenCV，请在 CMake 配置阶段设置对应的 `*_DIR`、工具链或 vcpkg 前缀路径。启用桥梁检测前，应先确认 CUDA、OpenCV 和 PCL 均可被 CMake 找到。

## 配置文件

程序启动时从 `config/` 读取设备、标定和运行参数，常用文件包括：

- `config/config.json`：程序运行参数。
- `config/devices.json`：雷达、相机、IMU 等设备配置。
- `config/calibration.json`：传感器外参和标定参数。
- 泊位地理信息 JSON：用于接收端固定泊位图层，文件路径由程序运行配置或界面流程指定。

发布 Release 版本时，应把上述配置文件与可执行文件放在约定的相对目录中，并同时部署 Qt、PCL、OpenCV、FFmpeg 等运行时 DLL。

## 基本使用流程

### PCAP 回放和泊位检测

1. 启动 `PointCloud.exe`。
2. 点击“加载离线数据回放”，选择一个或多个 `.pcap` 文件。
3. 按需开启“泊位检测”。
4. 在点云窗口检查泊位框、开口方向和检测日志。

### 生成和加载 SLAM 地图

1. 点击“SLAM 建图”开始累积关键帧。
2. 运行一段时间后点击“保存地图”，生成 `.slammap` 文件。
3. 后续可点击“加载地图”加载 `.slammap`，同时恢复地图点云和保存的泊位信息。
4. 如果要开始一次全新的 PCAP 回放，建议先停止当前 SLAM 或重启程序，避免旧地图层与新实时地图叠加。

### 信息投递

1. 在“信息投递”中填写接收端 IP、接收端口、本机绑定 IP 和本机端口。
2. 接收端 `Ship.exe` 先开启监听。
3. PointCloud 端开启“信息投递”。
4. 加载 `.slammap` 时，历史地图点云和地图泊位会进入历史投递队列。
5. 回放 PCAP 并开启泊位检测时，泊位结果按 0xFB 投递；实时 SLAM 点云按 0xFC 投递（具体开关以当前构建版本为准）。

PointCloud 端只负责协议发送，`Ship.exe` 是独立的接收和渲染程序，接收端的地图抽稀、栅格合并、投影和显示效果还取决于 Ship 端实现。

## 数据链路概览

```text
PCAP/在线传感器
      │
      ├─ 雷达点云、GNSS/INS、视频
      │
      ├─ 泊位检测节点 ── /perception/berth_result ── 0xFB ──> Ship
      │
      └─ DLO SLAM
           ├─ /slam/odometry
           ├─ /slam/keyframe
           └─ /slam/scan_cloud ── 0xFC ──> Ship

保存地图：关键帧 + ENU 锚点 + 泊位信息 → .slammap
加载地图：.slammap → 本地渲染 + 历史地图投递队列
```

实时地图和历史地图在 PointCloud 端使用不同的内部图层管理，历史地图通常显示为黄色，实时点云通常显示为绿色。两者是否合并、抽稀或重新投影，取决于具体的地图投递模式和接收端实现。

## 测试

测试目标位于 `tests/`，可在 CMake 配置时一并生成。建议在修改协议、地图存储、坐标转换或泊位几何代码后先运行对应单元测试，再进行 PCAP 回放和 Ship 端联调。

## 本次 GitHub 上传范围

本仓库上传的是“源码包”，用于代码阅读、功能开发和后续构建，不是包含运行环境的完整发布包。

已上传的内容包括：

- `src/`：Qt 主界面、点云处理、泊位检测、通信总线、UDP 投递、SLAM 和协议实现。
- `common/`：公共数据结构、SLAM 消息和共享头文件。
- `config/`：设备、标定和运行配置模板。
- `rs_driver/`：项目使用的雷达驱动头文件。
- `res/`：Qt/QML 资源和界面图片。
- `cmake/`、`CMakeLists.txt`、`CMakePresets.json`：构建配置和运行时依赖复制脚本。
- `tools/`：PCAP 合并、地理参考点云转换等 Python 工具脚本；Python 缓存文件未上传。
- `tests/`：泊位几何、地图拼接、协议和调度等测试代码。
- `docs/`：设计说明、实现计划和地图/投递相关技术文档。
- 根目录中的泊位地理信息 JSON、使用说明 DOCX、测试报告 DOCX 和坐标系说明 DOCX。
- `README.md`：项目结构、功能、构建和使用说明。

明确未上传的内容包括：

- `build/`、`build-*`、`artifacts/`：本地构建目录、目标文件和生成数据。
- `lib/`、`external/`、`Eigen3/`、`eigen-3.4.1/`、`opencv2/`：第三方 SDK、预编译库和外部依赖。
- `.qtcreator/`、`.superpowers/`、`.codex*`、`.docx_*`、`backups/`：本地 IDE、工具缓存、临时目录和备份。
- `*.exe`、`*.dll`、`*.a`、`*.o`、`*.obj`：可执行文件、动态库和编译中间文件。
- `*.pcap`、`*.pcapng`、`*.pcd`、`*.slammap` 以及其他大型点云、抓包和地图数据。

因此，克隆本仓库后仍需要在本机安装 Qt、CMake、MinGW/MSYS2、PCL、Eigen、OpenCV、FFmpeg 等依赖，并根据本机路径完成 CMake 配置。构建产物和数据集应通过单独的 Release 压缩包或 Git LFS 发布，不应直接混入源码仓库。

## 版本发布建议

本仓库适合保存源代码和可审查的项目文档。正式发布可执行版本时，应将 Release 构建目录、Qt/PCL/OpenCV/FFmpeg 运行时 DLL、PCAP/PCD 数据集和 `.slammap` 文件作为独立发布包提供。GitHub 普通 Git 推送不接受超过 100 MB 的单个文件。

## 项目描述（GitHub Description）

基于 Qt/PCL/Eigen/OpenCV 的无人船多传感器感知与 SLAM 系统，支持 PCAP 离线回放、雷达点云可视化、泊位检测、ENU 地图构建与 `.slammap` 保存加载，并通过 0xFB/0xFC UDP 协议向艇载 Ship 端投递泊位结果、实时点云和历史地图。

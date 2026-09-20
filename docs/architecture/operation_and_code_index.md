# 运行操作、日志判据与代码索引

## 1. 最短可复现实验流程

### A. PCAP → 泊位检测 → 保存地图

1. 启动 `PointCloud.exe`。
2. 点击“加载离线数据回放”，选择 `.pcap`。
3. 点击“泊位检测”，确认检测框和日志开始刷新。
4. 点击“SLAM 建图”，等待点云和关键帧累积。
5. 点击“保存地图”，得到时间命名的 `.slammap`。

保存之前至少要有关键帧；只打开检测而没有 SLAM 关键帧，不能得到完整地图。

### B. 历史地图 + 泊位投递

1. 启动 Ship，监听 `127.0.0.1:6789`（跨机时改成实际 IP）。
2. PointCloud“信息投递”填写 Ship IP/端口；本机绑定端口用 `6790`。
3. 点击“信息投递”，确认日志出现 UDP 已启动和订阅信息。
4. 点击“加载地图”，选择刚保存的 `.slammap`。
5. 等待日志出现“已准备 Tile、历史泊位”和“已接入投递节点”。
6. 在 Ship 端观察地图包、泊位包计数和泊位表格/覆盖层。

历史地图投递不需要重新点击“SLAM 建图”。若希望投递实时 PCAP，再单独加载 PCAP、开启泊位检测和 SLAM；不要把历史层和新一轮回放无意叠加。

## 2. 发送端应看到的关键日志

```text
信息投递已启动（订阅 /perception/berth_result 与 slam/*）
UDP 已启动：<local-ip>:<local-port> -> <remote-ip>:<remote-port>
已准备 <N> 个 Tile、<M> 个历史泊位
已接入投递节点: .../manifest.json
```

实时回放时还应看到泊位结果发布或 0xFB 发送记录，以及 `slam/keyframe`/`slam/scan_cloud` 被订阅。只有“界面上看到泊位框”而没有这些日志，不代表投递节点已经收到结果。

## 3. 代码索引

| 功能 | 主要文件/函数 |
| --- | --- |
| PCAP 读取 | `src/network/PcapngReader.cpp`：`startAnalysis`、`readLoop` |
| UDP 分发 | `src/network/UdpDispatcher.cpp`：`dispatch` |
| 视频解码 | `src/modules/VideoWorker.cpp`、`src/modules/AVDecoder.cpp` |
| 雷达融合 | `src/modules/radarfusionmanager.cpp`：同步、外参、融合输出 |
| 总线发布 | `src/bus/SensorPublisherNode.cpp`：`publishFusedSensors` |
| 泊位检测 | `src/bus/BerthDetectionNode.cpp`：同步帧消费、U 形/线段候选、结果发布 |
| SLAM 预处理 | `src/slam/DloSlamNode.cpp` |
| SLAM 位姿 | `src/code-slam/LidarGnssSlam.cpp` |
| 本地地图渲染 | `src/gui/MultiLidarWidget.cpp`：`paintGL`、`loadSlamMap` |
| 保存/加载归档 | `src/core/SlamMapBinaryIO.cpp` |
| 地理校正 | `src/slam/SlamMapGeoCorrector.cpp` |
| 历史 Tile | `src/slam/SlamMapLoadTask.cpp`、`src/slam/SlamTileMapBuilder.cpp` |
| 地图/泊位 UDP | `src/bus/PerceptionExportNode.cpp` |
| 配置默认值 | `config/config.json`、`src/config` |

## 4. 数据格式与部署边界

- `.slammap`：当前格式包含地图锚点、关键帧、地理参考和保存的泊位记录。
- `.pcd`：兼容旧格式的单点云输入，通常没有泊位和完整地理锚点。
- `manifest.json`/`*.tilebin`：历史地图投递缓存，不是新的地图源文件。
- 泊位 JSON：固定泊位库/回退图层，不等同于本次 PCAP 检测结果。
- `Ship.exe`：独立接收端，源码不在本仓库，因此接收端抽稀和最终绘制不能仅凭 PointCloud 代码确认。

仓库中的源码、配置、脚本、测试和文档与 Release 构建包分开维护。PCAP、PCD、SLAMMAP、视频和构建产物不应混入源码提交；大文件应通过 Release 或 Git LFS 提供。

## 5. 发生异常时的最小排查顺序

1. 先看 PointCloud 是否产生 `/perception/berth_result` 或 SLAM 主题日志。
2. 再看投递节点是否绑定了正确的本地端口、远端 IP/端口。
3. 用 Ship 的统计数字判断 0xFB/0xFC 是否真正到达。
4. 若发送端计数增长而 Ship 画面不对，优先检查 Ship 端的投影、抽稀、缓存和四角绘制顺序。
5. 若历史 Tile 显示异常，删除对应 `maps/tiled_cache/<fingerprint>` 后重新加载；不要直接修改 `.slammap` 二进制。

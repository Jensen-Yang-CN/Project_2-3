# 实时/历史地图与泊位投递

## 1. 投递节点和端口

发送端是 `src/bus/PerceptionExportNode.cpp`，接收端 `Ship.exe` 不在当前仓库中。PointCloud 只负责生成协议包和 UDP 发送；接收端如何抽稀、合并栅格、投影和绘制由 Ship 端实现。

同机测试可使用：

```text
Ship 监听地址：127.0.0.1:6789
PointCloud 本地发送绑定端口：6790
PointCloud 远端地址：127.0.0.1:6789
```

本地绑定端口不能和 Ship 的监听端口相同。启动顺序建议是：先启动 Ship 并开始监听，再在 PointCloud 配置“信息投递”并开启投递。

## 2. 两种协议

### 0xFB：泊位结果

0xFB 用于传输泊位四角、类型、开口边、尺寸和地理坐标。来源有两类：

1. 实时泊位检测：订阅 `/perception/berth_result`，由 `onBerthResult` 发送。
2. 历史地图泊位：加载 `.slammap` 后从 Tile `manifest.json`/归档泊位记录发送；若没有可用历史记录，可由固定泊位 JSON 提供静态图层。

当前代码把历史泊位按历史地图的 ENU 锚点发送，不再经过当前船体雷达姿态二次变换；这样可以避免历史框随当前船移动，也避免保存/加载/投递重复翻转开口方向。

### 0xFC：点云地图

0xFC 传输栅格化点云。包头携带地图参考经纬度/高度、方位、块索引和分辨率，块内部按 `32×32×10` 组织占据数据。接收端解码后是否按原分辨率显示、再做一次体素合并或只保留当前视窗，取决于 Ship 实现。

当前发送端的实际行为：

- 实时累计/扫描地图使用默认约 `0.1 m` 分辨率。
- 历史地图 Tile 使用 `config/config.json` 中的离线 LOD；默认 `[1.0, 0.3, 0.1] m`。导出历史地图时，包含泊位区域（含 2 m 缓冲）的 Tile 使用 `0.1 m`，其他 Tile 使用 `0.3 m`，以控制带宽和接收端渲染负载。
- 地图发送由 2 ms 节流定时器驱动，每次最多发送 8 个 UDP 包；历史队列低优先级，实时数据优先。
- 历史 Tile 按需逐块生产并缓存到 `maps/tiled_cache/<fingerprint>/`，缓存命中时不会重新计算。

因此，Ship 端看到的点云比 PointCloud 本地少，不一定是发送端没有点，也可能是接收端抽稀、缩放、视窗裁剪或栅格合并造成的。

## 3. 实时地图投递链路

```text
slam/keyframe、slam/scan_cloud
       └─ PerceptionExportNode::onSlamKeyframe/onSlamScanCloud
            └─ 固定一次 SLAM map → ENU 参考
                 └─ buildMapPacketsFromCloud
                      └─ 0xFC UDP → Ship
```

当前源码中的 `kEnableRealtimeMapExport` 为 `true`。实时参考只在收到第一个有效关键帧时捕获，后续点云都使用同一参考，不应因船体移动而把固定泊位重新跟着船平移。实时检测泊位仍单独通过 0xFB 发送；静态泊位 JSON 主要用于历史地图/固定泊位图层，不应替代实时检测结果。

## 4. 历史地图投递链路

```text
点击“加载地图”选择 .slammap
   └─ MainWindow::onLoadSlamMapClicked
        ├─ MultiLidarWidget::loadSlamMap：本地加载和渲染
        └─ prepareHistoricalMapForExport
             └─ SlamMapLoadTask::loadAndPrepareTiles
                  ├─ manifest.json
                  ├─ trajectory.bin
                  └─ *.tilebin
                       └─ PerceptionExportNode::loadHistoricalMap
                            ├─ 历史 0xFC Tile
                            └─ 历史 0xFB 泊位
```

历史地图投递不需要重新打开 SLAM 建图。日志出现“已准备 N 个 Tile、M 个历史泊位”和“已接入投递节点”后，才说明历史数据已经进入投递节点；Ship 端还应看到地图包计数和泊位计数增长。

`.slammap` 的历史泊位优先使用保存的 `SlamMapBerth`。如果地图只来自旧 `.pcd` 或保存时没有泊位检测，归档中就不会有历史泊位，发送端只能使用固定泊位 JSON 的回退图层。

## 5. 固定泊位 JSON 的定位

`PerceptionExportNode::Config` 目前带有默认静态泊位 JSON 路径，并会在应用目录/当前目录查找回退文件。推荐把部署用 JSON 放在源码包或 Release 包约定的配置目录，不要依赖开发机的 `D:/xwechat_files/...` 绝对路径；路径找不到时日志会明确提示加载失败。

固定泊位图层的作用是“预先已知的泊位范围”，不是当前帧检测结果。它适合在历史地图加载时提供稳定覆盖；实时 PCAP 回放阶段是否发送这层，应由当前构建的策略开关决定，不能把静态图层当成动态检测结果。

## 6. 常见问题定位

| 现象 | 优先检查 |
| --- | --- |
| PointCloud 有地图，Ship 没有 | IP/端口、Ship 是否先监听、UDP 防火墙、发送端日志是否显示 Tile 已入队 |
| Ship 地图有但泊位数为 0 | 是否用 `.slammap` 保存且保存时开启泊位检测；是否看到 0xFB 日志；是否加载了 JSON 回退文件 |
| 泊位随船移动 | 检查是否走了历史 ENU 路径；历史泊位不应再套当前船体位姿 |
| 开口方向相反 | 对比发送端 `opening_edge` 与 Ship 的四角绘制顺序；发送端不应再额外反转 |
| Ship 点云稀疏 | 检查接收端抽稀/栅格参数和当前 Tile LOD；必要时删除对应 `tiled_cache` 后重新加载 |
| 历史地图重复叠加实时地图 | 先停止/清空当前 SLAM 层，再开始新的 PCAP 回放；加载 `.slammap` 本身不会自动拼接下一次回放 |

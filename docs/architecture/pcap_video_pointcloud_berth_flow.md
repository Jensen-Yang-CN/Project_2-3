# PCAP、视频、点云与泊位检测链路

> 本文把历史材料中的“PCAP 到视频帧”“PCAP 到点云”和“泊位检测”合并为一条可核对的现行代码链路。若旧材料与源码不一致，以本文件和当前源码为准。

## 1. 一条数据如何进入程序

PCAP 不是视频文件或点云文件，而是以太网数据包的离线记录。程序按原始时间戳读取每个数据包，解析 Ethernet/IPv4/UDP 头，再按源 IP、源端口和协议类型分发给视频、雷达、GNSS/INS 等模块。

```text
PCAP 文件
  └─ PcapngReader::startAnalysis/readLoop
       └─ UdpDispatcher::dispatch
            ├─ 视频 RTP → RtpParser/RtpParserH264 → VideoWorker → AVDecoder → VideoWidget
            ├─ 雷达 UDP → 对应雷达驱动/解析器 → RadarFusionManager
            └─ GNSS/INS → 导航消息 → SLAM 与泊位地理投影
```

关键入口：

- `src/network/PcapngReader.cpp`：离线打开 PCAP、读取时间戳、发出 UDP 载荷。
- `src/network/UdpDispatcher.cpp`：按地址和端口把数据送到对应解析器。
- `src/modules/VideoWorker.cpp`、`src/modules/AVDecoder.cpp`：RTP/NAL 重组、FFmpeg 解码、YUV 到 RGB 转换。
- `src/modules/radarfusionmanager.cpp`：多雷达时间同步、外参转换和融合输出。

## 2. 视频显示路径

视频数据经 RTP 分片重组后，`VideoWorker` 补齐 Annex-B 起始码并交给 `AVDecoder`。解码器调用 FFmpeg 的 `avcodec_send_packet/avcodec_receive_frame`，再通过 `sws_scale` 转成 Qt 可显示的 RGB 图像。主窗口将左右相机帧送给相应 `VideoWidget`，控件在 `paintEvent` 中绘制最近一帧。

这条路径只负责画面显示，不参与点云坐标转换和泊位几何计算。当前播放器采用“保留最新帧”的策略，回放速度较快时可能丢弃过时视频帧，这是为了避免界面队列无限增长。

代码位置：

- `src/gui/mainwindow.cpp`：`onBridgeCameraFrame` 等相机帧槽函数。
- `src/gui/VideoWidget.*`：帧缓存、缩放和绘制。
- `src/modules/VideoWorker.cpp`：H.264/H.265 NAL 组包和代际过滤。

## 3. 点云融合路径

雷达解析器输出带时间戳的点云后，`RadarFusionManager` 在短时间窗口内缓存多路数据，以 210 雷达时间为主时钟，并把 201/211/213/214 等雷达变换到统一的 210 融合坐标系。融合结果通过总线发布：

```text
RadarFusionManager::fusedCloudReady
  └─ MainWindow::onFusedCloudToBus
       └─ SensorPublisherNode::publishFusedSensors(
              cloud, timestamp, berthDetectionEnabled, true)
```

其中第三个参数只控制是否让泊位检测节点处理当前帧；最后一个 `true` 表示 SLAM 仍接收融合点云。因此关闭泊位检测不会自动关闭 SLAM。

`src/modules/radarfusionmanager.cpp` 负责同步和外参，`src/bus/SensorPublisherNode.cpp` 负责把融合数据送入总线，`src/slam/DloSlamNode.cpp` 负责后续预处理和建图。

## 4. 泊位检测调用和输出

`src/bus/BerthDetectionNode.cpp` 订阅 `/preprocess/synced_sensors`。当前实现优先处理最新帧：旧帧会被丢弃，避免 PCAP 高速回放时检测队列越积越多。算法大致分为：

1. 对同步点云做范围、水面和异常点过滤。
2. 检测 U 形结构，必要时使用线段/边缘算法补充候选。
3. 合并相邻候选、去重，计算四角、长度、宽度、中心和开口边。
4. 用 GNSS/INS 的船体位姿把局部泊位投影到 ENU/地理坐标。
5. 发布 `/perception/berth_result`，同时通过 Qt 信号更新本地界面。

泊位结果中的开口方向不是由 Ship 端重新推断的；发送端产生的四角顺序和 `opening_edge` 会随 0xFB 协议发送。若接收端开口方向或数量异常，应同时检查发送数据和 Ship 端的绘制/过滤逻辑。

## 5. 建议的演示操作顺序

1. 点击“加载离线数据回放”，选择 `.pcap`。
2. 点击“泊位检测”，确认检测开关已打开。
3. 需要建图时再点击“SLAM 建图”。
4. 播放或暂停回放，观察点云和检测框。
5. 若要保存地图，先确认已经产生 SLAM 关键帧，再点击“保存地图”。

程序日志中可重点查看：`/perception/berth_result` 是否已订阅、泊位结果是否有四角/开口边，以及 `publishFusedSensors` 是否持续输出。只打开视频显示不会产生泊位结果；没有同步雷达和导航数据也不会得到稳定的地理泊位。

## 6. 当前实现的边界

- `.pcap` 回放按原始时间戳调度，检测节点只保留最新待处理帧，不保证逐帧输出。
- 视频解码、雷达融合、泊位检测和 SLAM 是并行链路；其中一条链路卡顿，不代表其他链路已经停止。
- 泊位地理投影依赖有效 GNSS/INS 参考；没有有效导航时可以看到局部框，但地理坐标和投递结果可能无效。
- 右侧相机若缺少雷达到相机外参，只能显示画面，不能据此推导正确的点云叠加位置。

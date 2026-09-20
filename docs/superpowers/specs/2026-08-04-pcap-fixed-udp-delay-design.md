# PCAP 固定 UDP 包间隔慢进设计

## 目标

为离线 PCAP 回放增加可配置的固定 UDP 包间隔模式。每个有效 UDP 载荷完成投递后，读取线程等待指定毫秒数，再读取下一包。

## 配置

`config/config.json` 增加：

```json
"pcap_playback": {
  "fixed_udp_delay_enabled": true,
  "udp_packet_delay_ms": 2
}
```

- `fixed_udp_delay_enabled=false`：保留现有的原始抓包时间戳回放。
- `fixed_udp_delay_enabled=true`：固定 UDP 包间隔替代原始时间戳限速。
- `udp_packet_delay_ms` 限制在 `0..1000`；`0` 表示不等待。
- 配置在程序启动时加载，修改后重启生效。

## 数据流

`pcap_next_ex` 读取捕获帧 → 校验 IPv4/UDP → 提取 UDP 载荷 → 发出 `udpPacket` → 可中断等待 → 读取下一帧。

固定延时只作用于成功投递的有效 UDP 包。非 IPv4、非 UDP、截断包和空载荷不计入延时。传给雷达、视频、导航与 SLAM 的抓包时间戳保持不变。

## 线程与停止

等待发生在 `PcapngReader` 所在的专用 `QThread`，不阻塞 UI。长延时拆分为不超过 10 ms 的小段，每段检查 `m_stopRequested`，保证停止操作不必等待完整配置时长。

## 验收

1. 源配置加载后固定延时开启且默认值为 2 ms。
2. 配置值小于 0 时收敛到 0，大于 1000 时收敛到 1000。
3. 含两个 UDP 包和一个 TCP 包的测试 PCAP 只投递两次，并只产生两次固定等待。
4. 固定延时关闭时继续按 PCAP 原始时间戳回放。
5. 等待期间点击停止可以提前结束。


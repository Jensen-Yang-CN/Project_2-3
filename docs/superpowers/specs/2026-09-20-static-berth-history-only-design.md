# 固定泊位图层仅用于历史地图

## 目标

实时投递时只发送泊位检测算法当前结果，不再把 JSON 中的 11 个固定泊位插入实时 `0xFB` 数据包；加载历史 `.slammap` 时仍发送这 11 个固定泊位图层。

## 范围

- 修改 `PerceptionExportNode` 的固定泊位发送入口和实时泊位组包逻辑。
- 保留 `loadHistoricalMap()` → `trySendHistoricalBerths()` 的固定泊位发送路径。
- 不修改泊位检测算法、本地界面、历史点云 `0xFC` 投递或接收端协议。

## 行为

1. 信息投递启动时不主动发送固定泊位图层。
2. 实时泊位结果只按检测结果组装并发送；不为固定泊位预留数量，也不按固定泊位过滤检测结果。
3. `loadHistoricalMap()` 成功后，继续通过历史泊位路径发送固定泊位图层；若地图自身有持久化泊位记录，维持现有优先级逻辑。

## 验证

- 回归测试确认启动路径不调用固定泊位发送，实时组包不包含固定泊位插入/过滤逻辑，历史路径仍保留 `trySendHistoricalBerths()` 和固定泊位库调用。
- 编译并运行 `perception_export_realtime_disabled`、`static_berth_library`、`berth_udp_protocol` 及相关 CTest。

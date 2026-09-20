# 泊位实时检测与盲区回显融合设计

## 目标

把 `D:\1\model\berth_detection_realtime_handoff_20260727` 的实时泊位检测和
`RealtimeBerthCoordinateLibrary` 坐标库逻辑完整接入当前工程。U 型与一字型算法继续
独立运行；实时结果优先，已确认泊位进入雷达盲区或短时漏检后，由世界坐标库回显。

## 数据流

1. `SensorPublisherNode` 将五雷达融合点云发布到
   `/preprocess/synced_sensors`。
2. `BerthDetectionNode` 分别运行 `BerthDetector` 和 `LineBerthDetector`，仅在结果层合并。
3. 节点读取总线中最新 `GnssInsMessage`，校验经纬度、姿态、时间戳和点云时间差。
4. 使用 `GnssInsGeoUtils` 的 ENU 累积位姿及标定外参生成
   `T_world_lidar = T_world_body * T_body_lidar`。
5. 仅把本帧实时结果交给 `RealtimeBerthCoordinateLibrary::updateFromRealtime()`；
   累计命中 5 帧后入库。
6. 调用 `mergeIntoCurrentFrame()` 将已入库泊位变换回当前雷达坐标系，与实时结果按中心
   距离和旋转矩形 IoU 去重。实时结果优先，回显结果标记为
   `BerthDetectionSource::CoordinateLibrary`。
7. 最终结果同时送往 OpenGL 显示、日志和 0xFB 协议输出。

## 显示约定

- 本帧实时泊位框：黄绿色。
- 坐标库盲区回显框：蓝色。
- U 型泊位优先使用算法给出的 `visible_edges` 和 `opening_segment`，不在 UI 中重新猜测
  开口边。
- 一字型泊位使用完整旋转矩形。
- 空结果必须替换旧结果并清除旧框，避免把静态残影误认为盲区回显。
- `is_using_memory=true` 表示当前帧至少包含一个坐标库回显泊位。

## 定位无效处理

以下任一情况发生时，本帧只显示实时检测，不更新坐标库，也不回显：

- 没有 GNSS/INS 状态；
- 经纬度、姿态或时间戳非有限值；
- 经纬度越界或经纬度同时为零；
- GNSS/INS 与点云时间差超过 2 秒。

定位恢复后继续使用同一 ENU 原点和坐标库。进程重启或节点重新初始化时清空 ENU 状态、
待确认候选和已存泊位，避免跨坐标原点复用旧数据。

## 测试与验收

- 连续 5 帧真实检测后，第 6 帧无检测仍返回一个蓝色来源的回显泊位。
- 同一帧既有实时结果又有相同库结果时，只保留实时结果。
- GNSS/INS 过期时不入库、不回显。
- 空结果能清除 OpenGL 中上一帧泊位。
- 源码使用黄绿色实时样式和蓝色回显样式。
- 新建干净构建目录完成配置、泊位测试和主程序编译。


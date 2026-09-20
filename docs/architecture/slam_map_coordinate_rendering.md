# SLAM 建图、坐标系、保存加载与界面渲染

## 1. SLAM 输入和关键帧

融合点云和 GNSS/INS 数据进入 `src/slam/DloSlamNode.cpp`，再由 `src/code-slam/LidarGnssSlam.cpp` 完成位姿估计。当前配置应以 `config/config.json` 和 `src/gui/mainwindow.cpp::initSlamNode` 为准：

- 关键帧平移阈值：`0.5 m`。
- 关键帧旋转阈值：`5°`。
- 点云预处理体素叶大小：`0.5 m`。
- 范围：`min_range=0`、`max_range=200 m`。
- 水面过滤可配置；显示端还有限制最大显示体素数的保护。

每个关键帧保存局部点云、`pose`，以及在有导航数据时的 `geo_reference`。世界点按下式得到：

```text
p_world = keyframe.pose * p_local
```

这里的 `pose` 是 SLAM 地图坐标中的位姿，不能直接当作 ENU 或船体坐标。第一帧建立的 SLAM 原点/方向也不必与 ENU 原点重合。

## 2. 坐标约定

项目采用列向量齐次变换，统一约定为：

```text
p_B = T_A_to_B * p_A
p_A = inverse(T_A_to_B) * p_B
```

主要坐标系：

- 210/融合坐标系：多雷达统一后的点云坐标。
- 船体/INS 坐标系：X 向船艏、Y 向右舷、Z 向下。
- NED：北、东、下。
- ENU：东、北、上；使用第一组有效 GNSS 建立局部原点。
- SLAM map：由 SLAM 第一帧建立的局部地图坐标，默认不等于 ENU。
- WGS84：经纬度和海拔，用于协议头和泊位地理信息。

多雷达外参先把各雷达送到 210，再进入 SLAM。典型关系是：

```text
T_210_to_body = T_211_to_body * T_210_to_211
p_body = T_210_to_body * p_210
```

泊位检测先在局部/210 坐标中生成几何，再结合船体导航姿态转为 ENU 或 WGS84。转换时必须明确源坐标系和目标坐标系，不能因为数值看起来相近就复用另一套外参。

## 3. `.slammap` 实际保存了什么

当前归档格式由 `src/core/SlamMapBinaryIO.h/.cpp` 定义，文件魔数为 `USVSLM2`，当前格式版本为 `2.2`。一个完整的 `.slammap` 包含：

- `SlamGeoAnchor`：地图地理锚点。
- 多个关键帧：时间戳、SLAM 位姿、可选 ENU/GNSS 参考、关键帧点云。
- `SlamMapBerth`：泊位编号、中心、尺寸、角度、类型、`opening_edge`、观测次数和置信度。

保存入口是 `MainWindow::onSaveSlamMapClicked` → `MultiLidarWidget::saveCurrentSlamMap`。保存前会检查是否已有关键帧；没有关键帧时不会生成有意义的地图。泊位信息来自 `updateBerthResult` 写入的 `SlamMapBerthStore`，不是从静态 JSON 自动猜出来的。

遗留 `.pcd` 只被当作单个身份关键帧加载，默认没有保存泊位记录，也没有完整地理锚点；需要泊位随地图保存时应使用 `.slammap`。

## 4. 加载和地理校正

`MultiLidarWidget::loadSlamMap` 加载归档后会：

1. 读取锚点、关键帧和泊位记录。
2. 调用 `slam_map_geo_correct::correct`，利用关键帧中的有效地理参考进行平面校正。
3. 用校正后的关键帧重新构建本地点云。
4. 恢复泊位存储，并在 SLAM 视图中绘制历史泊位框。

`src/slam/SlamMapGeoCorrector.cpp` 会计算类似
`reference.pose_lidar_enu * keyframe.pose.inverse()` 的校正关系，并对异常值做过滤、平滑和插值；泊位中心、角度和开口边也会跟随同一个平面校正变换。这样可避免保存端、加载端和投递端各自重复做一次方向翻转。

多地图拼接才使用 `SlamMapLoadTask::loadAndStitch` 和 `SlamMapStitcher`；单个 `.slammap` 通过 GUI 加载时主要是读取和地理校正，不会自动把新的 PCAP 数据拼到旧地图上。要开始新一次回放，建议先清空/停止当前地图或重启程序。

## 5. 本地软件如何渲染

`MultiLidarWidget` 区分三类内容：

- 历史地图：`m_slamMapCloud` 或离线 Tile，当前显示强度约为 `220`，界面通常呈浅黄色。
- 实时地图/实时扫描：`m_slamRealtimeMapCloud`、`m_slamLiveScan`，通常呈绿色。
- 泊位覆盖层：实时检测结果或保存的 `SlamMapBerth`，按四角和开口边绘制。

`paintGL` 在 SLAM 模式下绘制历史层和泊位层，在实时模式下绘制实时融合点云和实时泊位结果。离线地图加载还会调用 `MainWindow::prepareHistoricalMapForExport`，为投递端构建 Tile 缓存；这一步与本地 OpenGL 渲染是两条相互独立但使用同一份地图数据的路径。

## 6. 什么时候需要重新生成地图

- 只修改 Ship 端显示逻辑：不用重新生成 `.slammap`。
- 修改泊位检测算法、希望保存新的泊位框：需要重新回放 PCAP、开启泊位检测并重新保存 `.slammap`。
- 只想提高历史地图投递密度：先确认 `config/config.json` 的历史 Tile LOD 配置，再删除/重建对应 `maps/tiled_cache`，重新加载 `.slammap`。
- 使用 `.pcd` 作为旧格式输入：不能期待它携带泊位和完整 ENU 锚点。

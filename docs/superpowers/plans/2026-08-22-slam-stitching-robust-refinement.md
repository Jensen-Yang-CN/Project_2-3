# SLAM 离线地图稳健拼接优化实施计划

1. 在 `tests/slam_map_stitching_tests.cpp` 增加大残差恢复、越界拒绝和动态杂点用例，并更新默认策略断言，先确认旧实现无法满足新规格。
2. 在 `src/slam/SlamMapStitcher.cpp` 实现跨关键帧稳定结构点提取，并在稳定点不足时回退到原始地图点云。
3. 在 `src/slam/SlamMapStitcher.cpp` 实现受限二维候选初值搜索，将最佳候选交给粗、细两级 ICP。
4. 在 `src/slam/SlamMapStitcher.h` 调整默认开关、搜索范围、修正上限和质量改善阈值；保持时间重叠去重分支不变。
5. 编译并运行 `slam_map_stitching_tests`，修正所有回归问题。
6. 对 `20260810_193724_167.slammap` 和 `20260810_194048_767.slammap` 运行真实地图诊断，确认自动精配准被接受或给出明确回退依据。
7. 编译项目相关目标，检查最终差异和运行日志说明。

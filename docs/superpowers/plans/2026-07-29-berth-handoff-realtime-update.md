# 2026-07-29 泊位检测实时算法更新实施计划

1. 在现有泊位测试中增加新版开口判断、90° 轴交换和交叉框仲裁用例，先确认旧实现不能满足新规则。
2. 修改 `src/berth/BerthDetection.h`，移植 20 m × 2 m 边中点射线、无 Z 过滤和锚点恢复开口规则。
3. 删除 `src/berth/BerthDetection.h` 与 `src/berth/LineBerthDetection.h` 的每帧文件落盘代码。
4. 修改 `src/bus/RealtimeBerthStability.h`，按新版规则处理跨 180° 的边索引映射。
5. 修改 `src/bus/RealtimeBerthRecall.h`，采用新版实时框置信度和实时优先的回显冲突规则。
6. 运行泊位回显、稳定性、UDP 协议测试并构建主程序，检查源码中不再存在泊位调试落盘调用。

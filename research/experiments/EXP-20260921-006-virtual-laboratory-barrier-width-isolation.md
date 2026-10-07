# EXP-20260921-006-virtual-laboratory-barrier-width-isolation: Experiment Plan

问题：在 005 中央挡板候选仍留下固定相机杯体标签像素后，只增加挡板横向半宽，能否形成
`not_observable_by_nominal_fixed_depth`，同时杯体是否仍在冻结的位置筛查半径内？

阻塞前提：v0 结果哈希、005 的 blocked 结果哈希、候选模型哈希和半可达筛查来源运行哈希均必须匹配。
005 是本隔离测试的观察来源，不是已采纳结论。

唯一变量：`vlab_barrier.half_extents_m[1]` 从 `0.35 m` 改为 `0.46 m`。模型、默认关节、桌面、杯体、
挡板 X 厚度/位置/高度、相机、渲染器、筛查半径和硬件 I/O 均固定。`0.46 m` 只表示本虚拟测试中横向
超过桌面与两台相机的 Y 坐标，不代表实体挡板规格。

预先定义终态：两台固定相机均为零标签像素时为 `not_observable_by_nominal_fixed_depth`；任一相机有标签
像素时为 `barrier_width_candidate_remains_observable_by_nominal_fixed_depth`，保留结果且不在本 run 改动第二个
挡板维度。位置只报告 `inside_virtual_half_reach_screen` 或 `outside_virtual_half_reach_screen`；抓取级可达性始终为
`not_evaluated_no_deterministic_ik_collision_protocol`。

本 run 不代表实体遮挡、相机标定、真实深度感知、碰撞安全、IK、无碰撞轨迹、抓取、控制或 sim-to-real 结论。

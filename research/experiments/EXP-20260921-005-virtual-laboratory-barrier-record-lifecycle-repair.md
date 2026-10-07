# EXP-20260921-005-virtual-laboratory-barrier-record-lifecycle-repair: Experiment Plan

这是 `EXP-20260921-004` 的记录生命周期修复重复。004 在渲染和 assessment 写入后，因预建计划 record
与结果 record 使用同一路径而被记录器拒绝；004 已保留为 failed manifest，其中间产物不作为正式结果。

问题、模型、桌面、杯体、默认关节、挡板、两台固定相机、渲染器、硬件 I/O 和半可达位置筛查半径与
004 完全一致。唯一流程变化是：预执行计划 record 保持不变；完成后的不可变 execution record 写为
`EXP-20260921-005-virtual-laboratory-barrier-record-lifecycle-repair.execution.record.json`。

通过、停止条件和终态与 004 完全相同：两台固定相机均为零杯体 simulator segmentation 像素时记录
`not_observable_by_nominal_fixed_depth`；任一相机有像素时记录
`barrier_candidate_remains_observable_by_nominal_fixed_depth`，并不在本 run 改动挡板。位置只报告
`inside_virtual_half_reach_screen` 或 `outside_virtual_half_reach_screen`。抓取可达性始终为
`not_evaluated_no_deterministic_ik_collision_protocol`，距离筛查不得推演为 IK 或无碰撞抓取。

本 run 不代表实体遮挡、相机标定、真实深度感知、碰撞安全、IK、无碰撞轨迹、抓取、控制或 sim-to-real 结论。

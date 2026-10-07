# EXP-20260921-004-virtual-laboratory-barrier-observation-position-screen: Experiment Plan

问题：保持虚拟实验室 v0 的桌面、杯体、默认关节和两台名义固定相机不变，只在中部加入一块固定挡板后，
两台固定相机是否仍能得到杯体的 simulator segmentation 标签；同时，杯体位置是否仍处在已冻结的候选模型
半可达半径筛查内？

阻塞前提：`EXP-20260921-001-virtual-laboratory-v0-preflight` 的结果哈希，以及
`EXP-20260831-004-so101-mujoco-observation-preflight` 的运行清单哈希必须匹配。后者只提供
`0.23917871793616743 m` 的虚拟位置筛查半径，不提供抓取可达性。

控制项：模型版本、默认关节、桌面、杯尺寸、杯中心、两台固定相机、渲染器和硬件 I/O 均固定。唯一新增
变量是名为 `vlab_barrier` 的静态 box；其位于固定相机与杯体之间，且其 X 最小面在杯体中心的正 X 侧。

语义不变量：前序/来源哈希匹配；杯体与挡板世界位置匹配配置；杯体位于桌面 XY 内；三路相机都输出 RGB、
有限深度和 segmentation 产物；两台固定相机均无杯体标签像素；杯体 XY 半径不大于冻结的半可达筛查半径。

预先定义终态：两台固定相机均为零标签像素时为 `not_observable_by_nominal_fixed_depth`；任一相机有标签像素
时为 `barrier_candidate_remains_observable_by_nominal_fixed_depth`，保留结果且不在本 run 改动挡板。位置筛查
仅报告 `inside_virtual_half_reach_screen` 或 `outside_virtual_half_reach_screen`。无论哪种结果，抓取级可达性均为
`not_evaluated_no_deterministic_ik_collision_protocol`，不得由距离筛查推演。

本 run 不代表实体遮挡、相机标定、真实深度感知、碰撞安全、IK、无碰撞轨迹、抓取、控制或 sim-to-real 结论。

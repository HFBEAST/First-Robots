# EXP-20260923-029-known-cylinder-depth-agent-chain-golden-case: Experiment Plan

目的：在 028 已通过的 known-cylinder depth center fit 后，跑通首个完整虚拟链：Depth Detection/fit 的
Coordinate Agent 发布一个包含两台深度 frame、scene version 和 object-model version 的候选中心；
Coordinator 检查合同完整性；Capability Agent 只检查带半径杯体是否落在桌内；mock Execution Agent 将
内存中 A=`[0.12,0,0.055] m` 写为候选坐标；Reviewer 检查终态相等。

另建缺少 `fixed_depth_02` 的同一坐标消息，必须在 Coordinator 拒绝且不改写 A。通过只证明此固定虚拟
消息合同和 mock 状态链可运行；拟合 B 偏差仅记录，不作 Coordinator 或安全阈值。

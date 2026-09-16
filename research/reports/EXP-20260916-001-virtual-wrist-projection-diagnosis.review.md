# EXP-20260916-001-virtual-wrist-projection-diagnosis: Technical Invalidation Note

manifest 完整性校验通过，但本次运行在技术解释层面无效，不能用于投影、可见性、遮挡、相机或硬件结论。

原因：运行器创建 `virtual_cup` 后没有在编译前设置非零初始位置。虽然它在每个组合中修改了
`model.geom_pos[cup_id]`，产物的 650 条记录均显示 `world_center: [0, 0, 0]`，没有采用输入表中的
10 个杯位。最小独立复现确认：对同一模型，未设置初始 `cup.pos` 的路径在 `mj_forward` 后仍给出原点；
在编译前显式设置 `cup.pos=[0,0,0.055]` 后再修改 `model.geom_pos`，派生 `geom_xpos` 才正确更新。

影响：本次 `behind_camera`、`outside_candidate_depth_clip` 与 `outside_candidate_perspective_frustum`
计数只描述原点处目标，不能描述计划中的 650 个配置—杯位组合。`experiment-management validate`
只验证 manifest/产物完整性，未验证目标几何是否位于输入位置；通过不等于科学有效。

边界：此前 `EXP-20260901-001`、`EXP-20260901-002`、`EXP-20260902-002` 和
`EXP-20260902-003` 的运行器均在编译前显式设置 `cup.pos=[0,0,0.055]`，本说明不自动使其失效。
仍不扩大其候选模型、静态或 raster 观察边界。

处置：保留本 run、manifest 和产物；不覆盖结果。修正以新的 experiment ID 重新创建计划、运行器和
formal run，并保持 `pending_human_review`。

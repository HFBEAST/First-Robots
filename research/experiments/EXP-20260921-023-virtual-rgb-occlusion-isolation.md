# EXP-20260921-023-virtual-rgb-occlusion-isolation: Experiment Plan

022 的 B-10（`[-0.15,0.30,0.055] m`）在两台相机各有一个组件，却有很大的双相机投影差；原图显示
fixed_depth_01 中杯体部分被机器人遮住。本 run 的唯一变量是机器人**渲染可见性**：先保留候选模型原始
外观渲染，再仅把候选模型原有 robot geom 的 alpha 设为零重新渲染。桌与杯 geom 不改变，几何、关节、
相机和 B 都冻结。

对同一相机直接比较人工洋红 RGB mask 的像素数。两种 variant 都要求一组件；记录
`robot_visible / robot_render_disabled` 像素比例，但不预设可接受比例。本实验只区分“视觉遮挡是否足以
解释 B-10 的缺失完整性”，不产生融合规则或运动结论。

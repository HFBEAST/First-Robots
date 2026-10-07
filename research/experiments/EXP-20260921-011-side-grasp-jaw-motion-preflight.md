# EXP-20260921-011-side-grasp-jaw-motion-preflight: Experiment Plan

用户已选择侧面抓取。进入 IK 前，先验证候选模型中夹爪关节的静态运动约定：保持五个臂关节为默认零值，
只在 `gripper` 的两个已声明限位间切换；记录 `fixed_jaw_sph_tip1` 与 `moving_jaw_sph_tip1` 的世界位置和
代表性 tip separation。

通过条件：来源哈希匹配；两状态中五个臂关节不变、命名点位置有限；两种 separation 不相等。较大/较小的
代表性 separation 仅用于标注该模型的候选张开/闭合状态，不等同于物理夹持宽度或接触。

本 run 不加入杯体、不定义末端目标、接近距离、姿态容差或碰撞规则；不运行 IK、轨迹、控制或真机。

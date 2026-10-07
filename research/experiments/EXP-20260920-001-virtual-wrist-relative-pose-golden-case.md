# EXP-20260920-001-virtual-wrist-relative-pose-golden-case: Experiment Plan

问题：在固定候选 SO-101 模型、默认关节配置和当前记录的 MuJoCo 运行环境中，将虚拟杯体中心定义为
`wrist_cam` 相机坐标系的 `[0, 0, -0.20]` m 后，模型是否确实接收该相对位置，且单一 segmentation
渲染在三个独立 Python 进程中是否得到完全一致的非零杯体像素？

阻塞前提：模型来源必须是 Menagerie `da76818e269b82289eba39808e2fb91d679d6994`，并且
`robotstudio_so101/scene.xml` SHA-256 必须为
`65496b0061ca3ddd92b58fe09bd30f6a4a443f137cc42cf8975d5cff23030bfd`。该来源的研究限定见
`docs/decisions/DEC-20260831-001-adopt-virtual-so101-model.md`；历史来源记录见
`EXP-20260831-004-so101-mujoco-observation-preflight`。脚本在每个独立进程中重新核对场景哈希，
哈希不符即停止。

控制项：固定模型、相机 `wrist_cam`、六个默认关节值、圆柱杯体半径 0.035 m、半高 0.055 m、
320×240 MuJoCo geom segmentation、三个独立 Python 进程。唯一输入是已冻结的相机相对位置；本 run
不扫描位置、不改变关节、不使用中心投影或射线代理、不输出 RGB/深度、不做动力学、actuation、IK、
轨迹、控制或硬件 I/O。

语义不变量：每个进程都必须在 `mj_forward` 后验证杯体世界中心等于相机坐标变换得到的输入世界位置，
并验证该世界中心逆变换回 `wrist_cam` 坐标后等于 `[0, 0, -0.20]` m。

预先通过条件：三个进程均通过两项位置不变量、均有至少一个杯体 segmentation 像素，且三次的目标
像素数和完整 segmentation 缓冲区 SHA-256 完全一致。任一子进程失败、任一不变量失败、零像素或三次
结果不一致，均记录为未通过并阻止后续一维位移实验。

解释边界：通过仅说明当前候选模型与当前运行时内的一个固定相对位姿金标准可重复；不复现或解释旧
raster 表，不验证任何视锥/射线代理，也不产生实体相机、感知、安全、可达性、控制、sim-to-real 或
多机器人结论。结果保持 `pending_human_review`。

预期产物：正式 run manifest、三个独立进程的掩码与观测记录、配对结果和 experiment record。仅在
本门禁通过后，才可另建计划研究一个方向的一维相对位移。

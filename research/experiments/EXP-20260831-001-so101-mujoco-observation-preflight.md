# EXP-20260831-001-so101-mujoco-observation-preflight: Experiment Plan / 实验计划

> 状态：`authorized_virtual_adapter_pending_execution`。本文件是正式实验前的计划，不是实验结果、能力证明或人工采纳决定。

## Question and hypothesis / 问题与假设

问题：在单次 run 内冻结模型、渲染器版本和虚拟相机后，是否能在 MuJoCo 中以可重复的三视角布局生成
一台 SO-101 与杯状物体的腕部 RGB、两台固定深度相机帧及相应的离线几何可见性标签？

待验证假设 H-OBS-001：对相同的已固定 MuJoCo、SO-101 候选模型、场景和随机种子，离线渲染可重复，
并为每个试次保存足以复查三视角观测的元数据和哈希。

这不是“当前硬件已完成标定”“相机可以支撑抓取”或“多机器人协作可行”的假设。

## Inputs, variables, and controls / 输入、变量与控制项

- 实验专用拟议配置：[EXP-20260831-001-so101-mujoco-observation-preflight.json](../configs/EXP-20260831-001-so101-mujoco-observation-preflight.json)。
- 经人工授权的候选模型：MuJoCo Menagerie `robotstudio_so101` commit `da76818e269b82289eba39808e2fb91d679d6994`，
  Apache-2.0。其与本机硬件的标定一致性未验证。
- 已知硬件范围：一台 SO-101、一个腕部单 RGB、两台固定深度相机。真机相机型号、内外参、同步、噪声和
  实际视场仍未登记；本 run 使用显式记录的虚拟相机，后续真机校准只能以新 run 调整。
- 工作区：使用候选模型关节限位、固定 seed 的 10,000 点采样估计最大径向可达距离，其一半构成虚拟杯子采样盘。
  这是仿真构造，不是实际安全可达边界。
- 控制项：仅仿真；单臂保持模型默认静态姿态；不发送真机命令；不运行抓取、IK、碰撞规划或多机器人调度。
- 自变量：确定性采样的桌面杯状物体初始位姿（拟议 seeds `0..9`）。此数目仅为工作量估计，不是统计功效或验收阈值。
- 因变量：场景加载、每相机帧可用性、深度有限像素比例、基于仿真几何的可见性标签、相同输入的重复渲染哈希。

## Criteria and metrics / 判据与指标

执行时报告全部 10 个 seed 的逐项结果、失败类别和缺失帧；不得只报告成功样本。相同输入的帧哈希
是否一致只作为运行环境的复现性观测，不预设通过阈值。任何关于传感器覆盖、感知质量、抓取或硬件能力
的阈值和结论须人工确认，不能由本计划自动采纳。

## Baseline or why it is not applicable / 对照组或不适用理由

基线为“同一冻结配置和 seed 的第二次无动作渲染”。它只检验产物的确定性。MRBTP、RoCo 和
TAMPURA 的论文/代码不是该实验的性能基线：它们分别采用 MiniGrid、多个机械臂的 MuJoCo 场景和
独立的部分可观测 TAMP 环境，改变了任务、具身、控制器及评价对象。

## Expected artifacts / 预期产物

- 不可覆盖的 `research/runs/EXP-20260831-001-so101-mujoco-observation-preflight.json` 正式 run manifest；
- `research/artifacts/` 下按 seed 保存的相机元数据、RGB/depth 帧、几何可见性标签和 SHA-256 清单；
- `research/reports/` 下仅陈述观察事实、限制和人工审查状态的结果；
- 运行前后执行 manifest 校验；仅校验通过不等于能力已采纳。

## Preconditions and stop conditions / 前置条件与停止条件

执行前必须冻结并记录候选 SO-101 资产来源/版本/许可证/哈希、MuJoCo 版本、三台**虚拟**相机参数、
采样工作区和模型默认姿态。模型加载、相机创建或任一 seed 产物记录失败时停止并保留失败证据；不重试后
隐去失败。真机采纳另需实际标定、同步、噪声和安全边界证据，并以新 run 验证。

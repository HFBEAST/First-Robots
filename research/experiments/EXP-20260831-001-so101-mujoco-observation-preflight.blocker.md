# EXP-20260831-001-so101-mujoco-observation-preflight: Execution Blocker

记录时间：2026-08-31（Asia/Tokyo）。状态：`blocked_before_execution`。

已核查到候选 MuJoCo 资产：DeepMind MuJoCo Menagerie 的 `robotstudio_so101`，在 commit
`da76818e269b82289eba39808e2fb91d679d6994` 下提供 Apache-2.0 许可的 SO-101 MJCF，README 声明
MuJoCo >= 3.1.3 并记录了相机安装位。它只是候选来源，尚未下载、未冻结，也不能证明与本机 SO-101
的标定或相机布局相同。

正式执行仍缺少只能由本项目硬件记录提供的事实：

- SO-101 的实际模型/标定来源和一个人工确认的安全静止姿态；
- 腕部 RGB、固定深度相机 1、固定深度相机 2 的型号、内参、外参、同步和深度噪声/有效范围；
- 桌面工作区坐标系和可采样边界。

在这些事实到位前，不生成正式 MuJoCo run manifest，不以候选资产或假设相机参数替代它们。

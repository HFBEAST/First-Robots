# DEC-20260831-001: Adopt a provisional virtual SO-101 model

## Decision

项目负责人于 2026-08-31 批准：在本项目的**虚拟研究**中，采用 MuJoCo Menagerie 的
`robotstudio_so101` 作为当前 SO-101 候选模型。

- 来源：`google-deepmind/mujoco_menagerie` commit
  `da76818e269b82289eba39808e2fb91d679d6994`；
- 资产：`robotstudio_so101/scene.xml`，Apache-2.0；
- 用途：MuJoCo 中的单 SO-101 研究、虚拟相机和可复现实验；
- 生效范围：仅限 `research/`，产品运行时不读取该模型。

## Evidence

`EXP-20260831-004-so101-mujoco-observation-preflight` 是已校验的 formal virtual run。它加载了该
模型，在固定输入下生成 10 个 seed 的三路 RGB/depth 产物，并记录了模型哈希、MuJoCo 3.12.0、
虚拟工作区和全部产物哈希。

## Explicit non-adoptions

本决定不采纳、也不声称：

- 模型和本机 SO-101 的几何、动力学、关节零位或安全边界等价；
- 虚拟腕部或固定深度相机等同于实际相机标定；
- 真机可见性、感知、抓取、IK、碰撞、负载、控制、sim-to-real 或多机器人能力。

`EXP-...004` 在候选模型默认姿态下记录腕部相机对采样杯位姿的几何可见性为 0/10；这是一条
待审查的虚拟观察，不能被本决定改写或外推到真机。

## Follow-up and rollback

每个后续 run 必须继续固定模型 commit、资产哈希、MuJoCo 版本和虚拟相机配置。真机标定或替换模型
时创建新配置、新 run 和新决策，不修改既有证据。

若撤销本决定，停止将该模型用于新的研究 run；既有 run、产物和本决策保留以维持可审计性。

# 状态

## 当前阶段

`single_arm_simulation`。当前主目标为单台 SO-101、多相机、自然语言抓取与移动物体。
已实现独立运行的固定目标运动和已知坐标圆柱接触搬运接口；过去多机器人协议规划保留为后续方向。

2026-10-07 的首个运动基线 `EXP-20261007-001-single-arm-reach` 使用已知 world 目标
`[0.25,0,0.20] m`、五臂关节 position-only IK、模型关节/执行器限位、101 点路径穿透检查与每步
实际状态检查。IK 15 次迭代收敛；MuJoCo 位置执行器推进 1,000 步（5 s），目标误差由
`0.148768 m` 降为 `0.000154691 m`。每步记录 qpos/qvel/ctrl/site，保存 GIF 与终态审图。
001/002 的行为结果为 `passed_pending_human_review`，第二进程轨迹与第一完全相等，但正式 provenance
校验因 `dirty_at_start=true` 未通过。两次原记录保留为开发观察；随后 003 从提交代码的独立干净 checkout
重跑取得正式基线。位置精度只是该运动场景观测，不能证明抓取、视觉驱动、语言任务或真机能力。
原有感知/消息实验不能代替机械运动证据。

`EXP-20261007-003-single-arm-reach-committed-baseline` 已在提交 `22746b9efd254ca8481e94bf60d47e1df15329ee`
的独立干净 checkout 实际重跑，正式 manifest 校验通过；得到同样的 15 次 IK 迭代、1,000 步实际运动和
`0.000154691 m` 终态误差。003 是后续运动研究的正式基线，仍为 `pending_human_review`。001/002 的
dirty-worktree 校验失败原样保留。运行不使用随机采样，null seed 的确定性协议例外已写入计划。

下一场景已建立自由圆柱、质量/重力/接触，以及五臂关节的侧夹位姿求解。开发预检的低位后退接近点
未收敛并触及关节限位，已在 004 计划保留，改为水平夹爪从高位下降的接近路径。004 独立验证
物体落地支撑与两个位姿端点，没有评价闭爪和搬运。研究哈希产物已声明 Git `-text`，以避免 Windows
checkout 自动换行转换改变已记录的字节哈希；003 主工作区副本已从原始运行产物恢复并通过配对校验。

`EXP-20261007-004-cylinder-fixture-side-pose` 在提交 `ae5f2b1cde0c870dc00948faf5bf3ab05688f7c4`
的干净 checkout 正式执行并通过 manifest/配对校验。50 g 圆柱由初始 Z=0.060 m 落到约
Z=0.054858 m，终态支持法向力 0.4905 N；两秒轨迹有限。高位水平侧夹接近点 14 次迭代、低位
夹取点 21 次迭代收敛。审图为求解位姿的静态示意，不是抓取运动回放。

005 阶段已连接位置执行器与圆柱接触搬运。开发比较 `DEV-20261007-005-gripper-command-tuning`
只改变闭爪命令：0.5/0.6/0.7 rad 完成任务；0.8 rad 无移动侧夹爪接触，按门禁在闭爪后停止，未抬升。
冻结 0.7 rad 进行正式 005 与独立进程 006 复跑。开发观察的 B 中心误差约 13.61 mm；B 为半径 70 mm
的合成区域，不能把“物体落入区域”说成“精确放到点”。最大夹爪法向力仍约 55.55/60.09 N，未做
真机力学标定。开发 run 记录 dirty checkout，不计为正式 provenance 通过。
11 项行为测试通过，无跳过；包含无双侧夹持不得抬升、初始禁用碰撞不得运动、缩小 B 不得误报成功。
初始碰撞测试曾因样例放在无有效碰撞的位置失败，已移到肩部有效碰撞几何后验证门禁；未放宽门禁。

`EXP-20261007-005-single-arm-pick-place` 已在提交 `df0c89d1b6ce871f63cddad0db011ba0c2101118`
的干净 checkout 正式完成；manifest 与 record/result 配对校验通过。29 s 仿真中圆柱由 A 接触夹持、
抬升、搬运并释放，终态中心约 `[0.311772,0.106830,0.054858] m`，距 B 中心 13.61 mm，名义圆柱半径
完整落入预声明 B 区域。抬升后底面离地约 88.4 mm，两侧有正夹持力、无地面支持；终态两侧夹持力
为零、地面支持力约 0.4905 N。执行中没有重新赋值圆柱自由关节或添加刚性连接。
渲染抬升和终态已审图；绿色标记是回放中的 B 区域，不是接触物体。结果保持 `pending_human_review`。
正式门禁只支持这个已知坐标、固定形状/动力学的虚拟案例；随后 006 将复核独立进程轨迹一致性。

## 既有虚拟感知研究记录

以下保留各阶段当时的证据边界；当前运动/接触新增结果以上方 2026-10-07 记录为准。

研究状态：`optional / active`。已完成 formal virtual run
`EXP-20260831-004-so101-mujoco-observation-preflight`；已采纳 Menagerie `robotstudio_so101`
作为仅限研究旁车的临时虚拟模型，见 `docs/decisions/DEC-20260831-001-adopt-virtual-so101-model.md`。
尚无真机、相机标定、接触抓取或多机器人能力证据；虚拟感知和运动的限定结果见各 run。

最新 formal virtual run `EXP-20260902-002-virtual-wrist-segment-contact-scan` 已验证其 manifest：
在候选模型的默认配置至静态候选样本 166 的 65 个离散关节配置中，全部位于候选关节限位内，且对
10 个固定虚拟杯位均为零 MuJoCo 静态接触，几何可见数最高为 8。该结果保持
`pending_human_review`，不代表连续无碰撞、轨迹、控制、安全或真机能力。

`EXP-20260902-003-virtual-wrist-raster-visibility` 曾记录 40 个组合射线命中杯体、全部无 raster
像素；但 `EXP-20260916-004-virtual-wrist-raster-recheck` 在杯体位置不变量通过的 650 条重检中发现
30 条当前有像素、620 条仍无像素。故原全零表不完全可复现，两个结果都保持 `pending_human_review`，
且均不代表实体腕部 RGB 覆盖。

修正后的 `EXP-20260916-002-virtual-wrist-projection-diagnosis` 已验证 650/650 个杯体中心确实位于
输入位置：228 个位于候选相机后方、421 个位于候选透视视锥外、6 个超出候选深度裁剪范围（条件可
重叠），仅一项中心同时在视锥和裁剪范围内且仍无 raster 像素。该结果保持 `pending_human_review`，
不构成对遮挡、实体相机、感知或安全的结论。

`EXP-20260916-005-virtual-wrist-projection-raster-association` 将位置不变量保护的当前 raster 重检
与中心投影条件重新关联：30 条当前可见中有 29 条的杯体中心仍在候选视锥外。因此中心投影与
`mj_ray` 都不足以单独代表完整杯体的 raster 覆盖；该边界保持 `pending_human_review`。

`EXP-20260920-001-virtual-wrist-relative-pose-golden-case` 在恢复并核对的候选模型来源、默认关节
配置和固定相机相对杯位 `[0, 0, -0.20]` m 下，三次独立 Python 进程均通过杯体世界位置与相机相对
位置不变量，且各自得到 12,090 个 target segmentation 像素和相同缓冲区 SHA-256。该技术门禁只支持
继续研究一个受控相对位置变量；仍为 `pending_human_review`，不构成实体相机、感知或能力结论。

`EXP-20260920-002-virtual-wrist-relative-depth-step` 以 001 的结果哈希为阻塞前提，只将杯体相对
`wrist_cam` 的光轴深度从 0.20 m 变为 0.25 m。前序结果、模型哈希和两项位置不变量均通过，测试位
得到 7,504 个 target segmentation 像素（相对基线 -4,586）。像素差是一个受控候选模型观察，不是
视场覆盖、投影规律、因果或实体能力结论；结果保持 `pending_human_review`。

`EXP-20260920-003-virtual-wrist-relative-x-step` 以 002 的结果哈希为阻塞前提，只将杯体相对
`wrist_cam` 的 X 从 0 变为 +0.05 m，并保持相对深度 0.25 m、Y 与其他控制项不变。前序结果、模型
哈希和两项位置不变量均通过，测试位得到 7,544 个 target segmentation 像素（相对 002 +40）。这只是
一个受控横向位移观察，不是横向视场边界、覆盖规律、因果或实体能力结论；结果保持
`pending_human_review`。

`EXP-20260920-004-virtual-wrist-relative-y-step` 以 003 的结果哈希为阻塞前提，只将杯体相对
`wrist_cam` 的 Y 从 0 变为 +0.05 m，并保持 X=+0.05 m、相对深度 0.25 m 与其他控制项不变。前序
结果、模型哈希和两项位置不变量均通过，测试位得到 7,668 个 target segmentation 像素（相对 003
+124）。这只是一个受控 Y 位移观察，不是二维覆盖、视场边界、因果或实体能力结论；结果保持
`pending_human_review`。该点的 X=+0.05 m，因此不能同 X=0 的负 Y 点比较为正负 Y 关系。

`EXP-20260920-005-virtual-wrist-relative-negative-x-step` 从 002 的中心深度基线分支，只将杯体相对
`wrist_cam` 的 X 改为 -0.05 m，并保持 Y=0、相对深度 0.25 m 与其他控制项不变。前序结果、模型
哈希和两项位置不变量均通过，测试位得到 7,475 个 target segmentation 像素（相对 002 -29）。与
正 X 点的比较只是观察，不是对称性、横向覆盖、因果或实体能力结论；结果保持 `pending_human_review`。

`EXP-20260920-006-virtual-wrist-relative-negative-y-step` 从 002 的中心深度基线分支，只将杯体相对
`wrist_cam` 的 Y 改为 -0.05 m，并保持 X=0、相对深度 0.25 m 与其他控制项不变。前序结果、模型
哈希和两项位置不变量均通过，测试位得到 5,116 个 target segmentation 像素（相对 002 -2,388）。
该值不同于正 Y 单点的 7,668 像素，但本阶段未预设对称性条件，因此它不是失败或原因结论；在二维
组合或解释前，必须补建 X=0 的正 Y 单点；004 与 006 不构成正负 Y 对照。结果保持
`pending_human_review`。

`EXP-20260921-001-virtual-laboratory-v0-preflight` 已构建候选模型内的静态虚拟实验室 v0：名义
桌面顶面与世界 Z=0、杯体中心和三路相机的坐标契约均通过。两台名义固定相机分别渲染到 822、824
个杯体 segmentation 像素；候选腕部相机为 0 像素，按观察保留且不影响本 run 的固定相机通过条件。
该 v0 只是一套可重复的虚拟输入与静态观测场景，不是实体工作区、标定、感知、碰撞安全、控制或
sim-to-real 证据；结果保持 `pending_human_review`。

`EXP-20260921-002-virtual-laboratory-static-observation-task` 以 v0 的结果与状态哈希为阻塞前提，
将“至少一台名义固定相机有杯体 simulator segmentation 像素”定义为静态任务成功。两台固定相机均
提供标签证据，终态为 `observable_by_nominal_fixed_depth`；腕部相机标签为 0。该终态仅是冻结虚拟
状态的评价标签，不是 RGB/深度感知、实体相机或机器人能力结论；结果保持 `pending_human_review`。

`EXP-20260921-003-virtual-laboratory-not-observable-candidate` 只将杯体移动到桌面内的
`[-0.40, 0, 0.055]` m，未形成目标失败终态：两台固定相机仍分别有 477、480 个杯体标签像素，终态为
`candidate_remains_observable_by_nominal_fixed_depth`。该 run 的来源、桌面边界、杯体位置和三路产物
均完整，但任务判据不通过，因此保持 `blocked_pending_human_review`；不得将此点称为固定相机盲区，
也不得未计划地更换位置重试。

`EXP-20260921-004-virtual-laboratory-barrier-observation-position-screen` 在渲染后因预建 plan record 与
结果 record 同路径而被记录器拒绝；其 manifest 为 `failed`，没有正式 result 或 metrics，中间产物不作
实验解释。`EXP-20260921-005-virtual-laboratory-barrier-record-lifecycle-repair` 以完全冻结的同一场景修复
记录生命周期；`EXP-20260921-006-virtual-laboratory-barrier-width-isolation` 又只将挡板横向半宽从
`0.35 m` 改为 `0.46 m`。二者的 segmentation 像素计数随后被 007 的关联检查限定，均不得解释为杯体
RGB 可见性结论。

`EXP-20260921-007-virtual-laboratory-segmentation-association` 在不变几何下赋予杯体红色、挡板蓝色显示
色，发现遮挡边界上按杯体 geom ID 选取的 segmentation 像素并不具有预期红色通道关系。因此，在该挡板
场景中，segmentation 的边界标签不足以单独判定 RGB 杯体可见性；该结果保持
`blocked_pending_human_review`，不自动推翻未遮挡 v0 的其他观察，但阻止以 005/006 的计数推断遮挡。

`EXP-20260921-008-virtual-laboratory-occlusion-rgb-color-probe` 保持 006 的几何不变，只用洋红杯体与
青色挡板的预定义 RGB 色彩签名直接检查栅格：两台固定相机均为 0 个杯体签名像素，终态为
`not_observable_by_nominal_fixed_rgb_color_probe`。`EXP-20260921-010-virtual-laboratory-compiled-geometry-position-audit`
独立编译并比较 006 与 008 的场景，确认桌、杯、挡板、固定相机与默认关节相等；同时杯体距 baseframe
的 XY 半径为 `0.12 m`，小于冻结的虚拟半可达筛查半径 `0.23917871793616743 m`。这只支持该静态、人工
色彩虚拟场景的 RGB 栅格不可观察与位置筛查内；抓取级可达性为
`not_evaluated_no_deterministic_ik_collision_protocol`，不代表 IK、无碰撞路径、抓取、安全或真机能力。

`EXP-20260921-009-virtual-laboratory-barrier-geometry-position-audit` 在运行前发现 008 配置未显式重复
挡板名称/类型，不能以纯配置相等方式证明几何相等，故没有写入 run manifest、artifact 或 result；其
预执行失败记录保留在 `research/experiments/`，并由 010 的编译场景比较替代。

用户已选择侧面抓取。`EXP-20260921-011-side-grasp-jaw-motion-preflight` 在候选模型、五个臂关节固定为
零且无杯体/控制的条件下，只比较 `gripper` 两个限位：代表性固定—活动指尖间距在 `limit_upper` 为
`0.1333585014 m`、在 `limit_lower` 为 `0.0041255556 m`。因此仅对这对候选模型 tip，前者记录为候选
张开状态、后者记录为候选闭合状态；结果为 `passed_pending_human_review`。这不是有效夹持宽度、接触、
末端目标、IK、无碰撞运动、抓取或真机能力证据；侧抓的接近距离、末端姿态、碰撞判据与轨迹仍未定义。

`EXP-20260921-012-world-coordinate-contract-review` 冻结了当前虚拟世界合同：baseframe 与名义桌面顶面
均为世界原点/`Z=0`，杯体中心为 `[0.12, 0, 0.055] m`，挡板中心为 `[0.28, 0, 0.16] m`，两台固定相机为
`[0.45, -0.45, 0.42] m` 与 `[0.45, 0.45, 0.42] m`。全部命名对象、相机位置和两张原始审图均通过技术校验，
但仍为 `pending_human_review`。`EXP-20260921-013-world-coordinate-xy-review-map` 的第一张标注图保留，因
标签重叠/裁切不作为人工确认图；`EXP-20260921-014-world-coordinate-xy-map-layout-repair` 只修复画布与标签
布局，输出无遮挡的 XY 坐标复核图。该图显示默认 gripperframe 为约 `[0.391, -0.001, 0.246] m`，不等同于
侧抓目标。坐标合同与图都不是实体测量、标定、碰撞、IK、抓取或真机能力结论。

`EXP-20260921-015-no-barrier-coordinate-baseline` 从 012 的冻结候选模型建立无挡板基线：编译场景中
不存在 `vlab_barrier`，baseframe、桌、杯 A 与两台固定相机均匹配配置；两张审图已保留。该 run 不移动杯体。
在其上，`EXP-20260921-016-no-barrier-multirange-coordinate-agent-protocol` 以 25 个固定 B 坐标测试合成
多 Agent 消息链，但 table-containment 的 Z 公式将杯中心错误地判作 `0.080 m`，因此 25 个正例全部拒绝；
该失败完整保留，不能称为坐标或协作失败。`EXP-20260921-017-no-barrier-multirange-coordinate-agent-protocol-repair`
只修复该公式，25/25 个 B 均由两台固定相机的 simulator-truth 坐标恢复为同一 world B，随后依次通过
frame/version 协调、桌面包含门禁、mock A→B 状态转换和终态相等复核；未知 frame 与桌外 B 都拒绝且状态未变。
017 是 `passed_pending_human_review`，仅支持该无挡板虚拟消息合同与内存状态适配器；其 Detection Agent 不是
RGB/深度感知，Execution Agent 不是 MuJoCo 机器人动作、IK、碰撞、抓取、规划、安全或真机控制，25 点也不是
可达性或安全范围。

`EXP-20260921-018-coordinate-agent-fault-injection` 以 017 为冻结前提，隔离 Coordinator 的消息拒绝行为：
25 个正确的双 frame world 坐标对均接收；对每个 B，第二条坐标加固定的非零 `[0.001,0,0] m` 反例会以
`camera_world_coordinate_disagreement` 拒绝，第二条旧 scenario version 会以 `scenario_version_mismatch` 拒绝。
两类拒绝都发生在 mock Execution Agent 前，A 状态保持不变。该 1 mm 仅是确定性坏消息示例，不是噪声容差或
已采纳阈值；018 为 `passed_pending_human_review`，不评价相机标定、视觉或真实坐标精度。

用户已授权先完成全虚拟感知研究，并保留未来真实数据可替换的接口。`EXP-20260921-019-virtual-rgb-detection-golden-case`
建立 `RgbFrameSource.capture(camera_name) -> RgbFrame` 合同，Detection Agent 只读取 RGB 帧，不接收 world B、
相机外参或 MuJoCo model/data；两台虚拟 RGB 帧均出现候选像素。人工审图发现原始绝对 RGB 谓词还选到非杯
边缘像素，所以 019 的“正像素”通过条件不能作为隔离结论。`EXP-20260921-020-virtual-rgb-detection-predicate-repair`
只增加红/蓝相对绿色的通道占优条件；两台图各得到一个连续候选组件，均为 `passed_pending_human_review`。
这些颜色条件和单组件条件只属于人工洋红虚拟金样例，不是实体检测指标或已采纳阈值。

`EXP-20260921-021-virtual-rgb-centroid-plane-projection` 只将 020 的 RGB 组件质心沿编译相机射线投到
冻结的杯中心高度平面 `Z=0.055 m`，没有融合、Coordinator 或执行。两台估计相对记录 B 的欧氏偏差分别为
`0.0040200972 m` 与 `0.0037757923 m`，且 y 偏差方向相反；结果为 `passed_pending_human_review`，仅说明
交点有限且在相机前方。尚未定义视觉误差容许、双相机融合/拒绝规则或真实标定，故不得将这些点接入 017 的
严格相等 Coordinator 门禁，也不得称为 world pose、执行或硬件能力。

在用户授权虚拟融合/容许规则前，`EXP-20260921-022-virtual-rgb-multirange-projection-sweep` 先对固定
25 点执行同一 RGB→平面投影流程。仅 18/25 点在两台相机各有一个组件；其余 7 点至少一台产生两个组件。
在 18 个双相机点中，最大单相机评估偏差为 `0.0755010339 m`、最大双相机差为 `0.0902239907 m`，故该 run
为 `blocked_pending_human_review`，不能据 021 的单点表现设定全局融合阈值。对最坏单组件点 B-10 的
`EXP-20260921-023-virtual-rgb-occlusion-isolation` 保持几何不变，只关闭机器人渲染：fixed_depth_01 的
候选像素由 1,353 降至 145（比例约 10.7%），fixed_depth_02 保持 3,812，支持该点的视觉部分遮挡解释。
这仍只是人工色彩虚拟外观隔离，不是物理遮挡率或可用性阈值。

为对齐未来两台固定深度相机，`EXP-20260921-024-virtual-depth-frame-contract-golden-case` 已建立
`DepthFrameSource.capture(camera_name) -> DepthFrame`：两台 MuJoCo depth render 均为有限、正值、非均匀的
`float32` array，原始 `.npy` 和归一化审图已保留，结果为 `passed_pending_human_review`。它只证明虚拟深度
帧的可替换接口，不是 Orbbec 采集、单位/标定、深度分割、融合或硬件质量结论。

`EXP-20260923-025-virtual-depth-background-detection-golden-case` 在无挡板、默认机器人姿态和固定 B
`[0.15,0,0.055] m` 中，对两台虚拟固定深度相机各自比较无杯 reference 与有杯 current frame。两路
`reference-current` 均产生一个前景组件，像素数分别为 3,455 和 3,456；Detection Agent 只读取成对
`DepthFrame`，不读取 RGB、world B、相机外参或 MuJoCo model/data。结果为 `passed_pending_human_review`，
只支持该固定虚拟背景差分金样例，不是实体背景采集政策、物理噪声阈值、姿态或融合结论。

`EXP-20260923-026-virtual-depth-surface-point-projection` 已保留但其投影前提错误：它把 MuJoCo `depth`
误作相机原点到表面的射线长度。`EXP-20260923-027-virtual-depth-plane-projection-repair` 根据 MuJoCo
官方 camera output 定义（`depth` 为距相机平面、`distance` 为距相机原点）只修复公式，复用完全相同的
025 前景数组。两台 depth foreground 都产生有限可见表面点，距杯中心分别约 `0.0276617 m` 和
`0.0278052 m`；这符合表面点不是圆柱中心，不能作为目标 world pose。027 为 `passed_pending_human_review`，
但中心拟合、双相机融合、Coordinator 和执行均仍为 `not_evaluated`。

用户将首个感知目标明确限制为已知尺寸、直立且静置在 `Z=0` 虚拟平面的圆柱。`EXP-20260923-028-known-cylinder-depth-center-fit-golden-case`
在冻结的 025 前景点上，采用已知圆柱半径 `0.035 m`、半高 `0.055 m` 和低于合并点云 Z 中位数的可见侧面点作
代数圆拟合；两个 depth 输入共 6,911 点，拟合中心为约 `[0.1499506, 0.0000673, 0.055] m`，相对该冻结 B 的
仅评估误差约 `0.0000835 m`。028 为 `passed_pending_human_review`：这些数字只描述此单一虚拟已知模型，
不是现实精度、一般物体姿态、阈值或已采纳能力。

`EXP-20260923-029-known-cylinder-depth-agent-chain-golden-case` 将 028 的冻结拟合中心接入首个端到端
虚拟链：Coordinate Agent 发布带 `fixed_depth_01`/`fixed_depth_02`、scenario 和 object-model version 的消息；
Coordinator 接收完整匹配消息；Capability Agent 仅检查带圆柱半径的桌面包含；mock Executor 将内存状态从
A 写为候选坐标；Reviewer 检查终态完全相等。对应缺少 `fixed_depth_02` 的反例在 Coordinator 以
`required_depth_frames_incomplete` 拒绝，mock Executor 不执行且 A 状态不变。029 也是
`passed_pending_human_review`；Coordinator 未验证数值精度，Capability 未验证可达、碰撞或安全，mock Executor
不是 MuJoCo 机械臂动作、IK、抓取、规划或硬件控制。RGB 没有进入此链。

`EXP-20260923-030-known-cylinder-depth-x-step-agent-chain` 以 029 为前提，只将已知圆柱 world X 设为
`0.125`、`0.150`、`0.175 m`，冻结 Y=`0`、Z=`0.055 m`、尺寸、姿态、桌面、相机、默认关节、背景差分、
拟合和消息合同。每点重新渲染两台 depth 的空桌 reference 与有圆柱 current；两路均各有一个前景组件、
有限点云、有限拟合中心、完整双 frame 消息、Coordinator 接收、table-only 门禁、mock A→候选状态转换及
Reviewer 相等。三个记录的拟合中心相对各自配置点的仅评估误差约为 `0.0000779`、`0.0000835`、`0.0000824 m`；
未设置精度阈值。030 的原始观测也记录了每点 `compiled_cup_center_matches_configuration=true`，但该字段最初未
进入通过布尔式，所以不能单独作为“只变 X 已门禁”的依据。

`EXP-20260923-031-known-cylinder-depth-x-step-compiled-scene-audit` 独立补齐并实际执行该遗漏的门禁：三组
新编译场景均无挡板，cup 中心恰为声明 X 序列且 Y/Z 不变；table、两台 camera 位置/朝向/FOV 与默认 robot
qpos 都匹配冻结配置。031 为 `passed_pending_human_review`，是后续引用 030 这三个 X 条件时必须同时关联的
配置审计证据。它未重跑或改变 030 的感知/Agent 结果，也不证明真实深度质量、精度范围、可达性、碰撞、
抓取、规划、安全、MuJoCo 动作或硬件能力。

## 已知风险与阻塞

- 最终机器人数量、型号、安装布局和共享工作区尚未冻结；目前只知道研究从 SO-101 能力开始。
- 两台固定深度相机和腕部相机的型号、标定、同步、噪声与实际覆盖尚未登记。
- `research/hardware/HW-20260903-001-orbbec-astra-pro-observation-stack.json` 已登记用户提供的
  Orbbec Astra Pro/OpenNI2 信息和本机只读预检：SDK 文件、设备功能枚举及指定 Conda 环境的
  导入链可用，但尚未初始化设备或读帧，且未确认它们如何对应两台实体固定深度相机。
- 已采纳候选模型默认姿态下，虚拟腕部相机对本 run 的半可达范围杯位姿为 0/10 几何可见；这是虚拟
  观察，不是实体腕相机覆盖率。
- “多 Agent 优于单调度器”只是待验证假设，不能作为架构事实。
- 多机器人交接需要共同可达抓取位、物体所有权、资源锁和同步执行，目前均未实现。
- 外部论文/开源项目只完成资料筛选，尚未在本项目中复现。

## 下一个可验证步骤

在固定目标运动基线上，确定五臂关节可实现的侧抓约束，建立带自由运动、重力和接触的圆柱场景，
实现固定 A→B 的接近、闭爪、抬升、搬运、释放和终态观察。先提供明确的已知坐标，完成抓取后用
虚拟视觉替换输入并接入语言任务。设备启用后另行执行 Orbbec 读帧和标定。每个已完成阶段单独提交，
保留失败与微调记录；不以全部多机器人协议冻结作为前置条件。

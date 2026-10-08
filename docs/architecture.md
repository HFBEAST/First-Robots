# 架构

## 设计原则

产品运行时以类型化契约连接语言、感知、规划和控制。所有安全相关判断保留在确定性模块中；
Agent 只能查询证据、提出候选动作和解释结果。研究代码通过适配器评价产品接口，产品不读取研究目录。

## 当前单臂执行路径

```text
explicit known target (later: language + camera estimate)
  -> bounded deterministic IK
  -> joint/actuator limits + sampled collision gate
  -> MuJoCo position actuators + per-step state/contact checks
  -> trajectory + observed terminal state
```

`src/first_robots/simulation.py` 提供独立的运动与回放接口；默认模型位于 `assets/so101`，运行配置位于
`config/sim_reach.json`。运行时不依赖 `research/`。研究运行器调用产品公开接口并保存配置、代码与模型
哈希、轨迹、结果和审图。已知坐标输入明确属于控制基线，未来用视觉估计替换。

`src/first_robots/grasping.py` 提供自由圆柱场景、五轴侧夹位姿求解和确定性阶段执行：接近、下降、闭爪、
抬升、搬运、下降、释放、撤离。`config/sim_pick_place.json` 固定当前任务输入。对象是有质量的自由体，
执行中只写关节执行器命令；回放函数才重新赋值记录的物体状态。预期夹爪/支撑接触与禁用接触分开检查；
阶段前提失败停止下游执行。语言层最终调用经过验证的技能接口；当前流程不需要 LLM 或多 Agent。

`src/first_robots/depth.py` 定义产品侧 `DepthFrameSource.capture(camera_name)`、米制前向深度和
右/下/前光学坐标合同，提供名义虚拟采集与 pixel→world 变换。当前只验证观察前提，不生成动作。
硬件适配器将来提供同类帧及独立标定，不调用 MuJoCo。旧 `research/protocols/` 合同只供历史复现，
不作为产品运行时的第二事实源。离屏深度关闭多重采样，单位、frame/version 和无效样本显式拒绝。

## 后续多机器人组件草案

| 组件 | 职责 |
|---|---|
| Shared Belief Store | 保存物体、人员、障碍、机器人和资源的带来源、时间、置信度、版本的事实 |
| Capability Registry | 保存每台机器人的静态规格、动态状态、技能契约和正式证据范围 |
| Grounding Agent | 把自然语言转成符号目标并识别缺失信息 |
| Exploration Planner | 在信息不足时选择受预算约束的观察或试探动作 |
| Team Coordinator | 形成联合任务骨架、分配机器人并预约物体与交接区域 |
| Robot Capability Agent | 查询本机可见性、IK、碰撞、负载、抓取和交接可行性 |
| Robot Executor | 执行通过门禁的类型化技能，并流式报告阶段与后置条件 |
| Evidence Reviewer | 判断成功、重规划、不可观测、不可行、执行失败或无法评估 |
| Memory Curator | 从正式运行提出新的能力版本；不覆盖原始记录或自动批准结论 |

## 依赖与数据流

```text
instruction
  -> Grounding Agent
  -> Shared Belief Store <-> perception / observations
  -> Team Coordinator <-> Capability Registry / per-robot Capability Agents
  -> versioned joint plan + resource reservations
  -> per-robot Executors -> deterministic safety and control
  -> execution events / observations
  -> Evidence Reviewer -> replan or terminal assessment
  -> append-only experience -> reviewed capability update proposal
```

`research/` 可以生成故障场景、对照配置和正式 run，但只能通过产品公开接口观察和调用产品。

## 对外入口

- `scripts/run.ps1`：薄运行入口；暴露状态、虚拟 reach 和 pick-place，调用 `src/first_robots/` 的公开应用接口。
- `scripts/governance.ps1`：薄治理入口，转发到安装的 Experiment-Management CLI。
- `src/first_robots/cli.py`：当前最小产品入口；后续不得承担可复用业务逻辑。

## 首批待冻结协议

`BeliefFact`、`RobotCapability`、`SkillContract`、`PlanStep`、`ExecutionEvent`、
`TerminalAssessment`。这些草案留供后续多机器人研究，当前单臂运动和抓取不依赖全部协议冻结。

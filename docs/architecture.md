# 架构

## 设计原则

产品运行时以类型化契约连接语言、感知、规划和控制。所有安全相关判断保留在确定性模块中；
Agent 只能查询证据、提出候选动作和解释结果。研究代码通过适配器评价产品接口，产品不读取研究目录。

## 计划组件与职责

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

- `scripts/run.ps1`：薄运行入口；当前只暴露项目状态，后续调用 `src/first_robots/` 的公开应用接口。
- `scripts/governance.ps1`：薄治理入口，转发到安装的 Experiment-Management CLI。
- `src/first_robots/cli.py`：当前最小产品入口；后续不得承担可复用业务逻辑。

## 首批待冻结协议

`BeliefFact`、`RobotCapability`、`SkillContract`、`PlanStep`、`ExecutionEvent`、
`TerminalAssessment`。协议版本冻结后才开始多机器人执行器实现。

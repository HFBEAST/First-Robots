# EXP-20260831-002-mrbtp-demo-exploratory: Experiment Plan / 实验计划

> 状态：`planned_not_executed`。这是对上游 MRBTP-demo 自检的 exploratory 执行，和 SO-101 MuJoCo
> 正式预检无关，不能构成项目能力结论。

## Question and hypothesis / 问题与假设

问题：已固定的 MRBTP-demo 源码能否在隔离 Python 环境中按其 `app.verify --both` 自检入口运行，
并生成可检查的日志和依赖清单？

待验证假设 H-MRBTP-CODE-001：该提交在记录的本地运行时可完成其共享信念与关闭共享信念的自检路径。
它不假设两种模式的结果存在性能差异；上游说明单次执行存在随机死锁，并使用重试。

## Inputs, variables, and controls / 输入、变量与控制项

- 配置：[EXP-20260831-002-mrbtp-demo-exploratory.json](../configs/EXP-20260831-002-mrbtp-demo-exploratory.json)。
- 已固定源码：`research/sources/mrbtp-demo`，MIT，commit `3824c1c62bf38c89094c37b77573711e639c1d04`。
- 运行两种上游模式：`--both`；二者均使用上游默认的 2 agents、400 max steps、base seed 6、最多 5 次尝试。
- 控制项：SDL headless；不联网执行；不修改上游源码；不接入 First-Robots 产品模块、MuJoCo 或硬件。

## Criteria and metrics / 判据与指标

记录进程退出码、每种模式的每次尝试、规划摘要、最终 `RESULT`/`SUMMARY` 行、已安装依赖版本和源树状态。
探索性运行的技术通过条件是上游进程以退出码 0 结束；这不是论文指标复现、统计显著性、共享信念优效、
或产品验收条件。失败时保留日志并记录失败，不更换提交或“挑选”成功 seed。

## Baseline or why it is not applicable / 对照组或不适用理由

`--no-share` 是上游提供的消融运行，不是严格性能基线：上游自检只要求 shared 路径决定退出码，且重试会
改变尝试的随机轨迹。该 run 只检查两条路径是否可被调用。

## Expected artifacts / 预期产物

- 不可覆盖的 stdout/stderr 日志和 `pip freeze`；
- 源码 HEAD、状态、依赖和命令记录；
- `research/runs/EXP-20260831-002-mrbtp-demo-exploratory.json` manifest 与配对结果 JSON；
- `pending_human_review` 状态；不生成项目源码改动或能力采纳。

# 项目概览

## 目标

First-Robots 研究并实现一个多机器人协同系统：机器人共享带不确定性和来源的世界状态，
根据各自可验证的运动、负载、抓取和感知能力分工；信息不足时主动探索；执行过程中验证
后置条件并更新经验；有限尝试后能够有证据地区分不可观测、不可行、执行失败和无法评估。

代表任务是“把杯子给 B”：若机器人 1 能抓杯子、机器人 2 能到达 B，但二者无法直接交接，
系统应判断机器人 3 是否能形成可验证的接力路径，并协调观察、抓取、交接和交付。

## 用户与边界

- 主要用户：机器人研究者、系统开发者和实验人工复核者。
- 当前阶段：项目规划和协议地基；尚无多机器人运行能力或性能结论。
- 当前研究平台目标：先在 MuJoCo 中使用 SO-101 / SO-ARM101 模型验证，再逐级转向真机。
- LLM 用于语言 grounding、方案提议和解释；不直接负责关节控制或安全判定。
- 仿真真值只可用于离线标签和评价，不能成为正式感知方案的隐藏运行输入。
- 研究旁车是可选证据支持；删除它不能破坏产品测试或运行入口。

## 如何运行与验证

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run.ps1 status
py -3 -m unittest discover -s tests -v
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/governance.ps1 validate-index .
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/governance.ps1 verify-detachable .
```

最后两条依赖已安装的 `experiment-management` v0.5 CLI 包。验证通过只证明项目索引和研究
可拆卸性，不代表任何机器人研究结论已经成立。

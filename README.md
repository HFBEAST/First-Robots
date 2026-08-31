# First-Robots

面向多机器人共享记忆、能力约束、主动探索与多 Agent 协同的开发优先项目。

首次进入项目时依次阅读：

1. `project.json`
2. `docs/overview.md`
3. `docs/architecture.md`
4. `docs/status.md`

查看当前状态：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run.ps1 status
```

执行产品验证：

```powershell
py -3 -m unittest discover -s tests -v
```

研究是 `research/` 中可整体拆卸的证据旁车。产品源码、运行配置、脚本和测试不得读取它。
正式研究结论必须拥有通过校验的 run，并保持人工采纳状态独立。

# 项目概览

## 目标

First-Robots 的当前目标是在现实桌面场景中，用一台 SO-101、两台固定深度相机和一台腕部 RGB 相机，
根据自然语言指令定位、抓取并移动目标物体。首个任务是将一个已知直立圆柱从固定 A 放到固定 B。

先完成简单 MuJoCo 场景中的机械运动与接触抓取，再接入虚拟视觉和语言，扩展受控变量，最后进行真机
标定与受控任务验证。多机器人接力和协同留作后续方向，不作为当前单臂任务的前置条件。

## 用户与边界

- 主要用户：机器人研究者、系统开发者和实验人工复核者。
- 当前阶段：单臂仿真运动基线；已有固定目标位置执行器运动入口，抓取、视觉驱动和语言任务仍待接入。
- 当前研究平台目标：先在 MuJoCo 中使用 SO-101 / SO-ARM101 模型验证，再逐级转向真机。
- LLM 用于语言 grounding、方案提议和解释；不直接负责关节控制或安全判定。
- 控制基线可以显式使用已知目标坐标；视觉任务必须用相机估计替换该输入，并保留真值用于独立评价。
- 研究旁车是可选证据支持；删除它不能破坏产品测试或运行入口。

## 如何运行与验证

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run.ps1 status
py -3 -m unittest discover -s tests -v
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/governance.ps1 validate-index .
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/governance.ps1 verify-detachable .
```

仿真功能需在安装 `requirements-simulation.txt` 的 Python 环境中运行：

```powershell
$env:PYTHONPATH = 'src'
python -m first_robots.cli reach --config config/sim_reach.json
```

`reach` 输出实际运动和 IK 结果；数值设置属于虚拟开发基线，真实相机、关节零位与动力学需另行验证。

最后两条依赖已安装的 `experiment-management` v0.5 CLI 包。验证通过只证明项目索引和研究
可拆卸性，不代表任何机器人研究结论已经成立。

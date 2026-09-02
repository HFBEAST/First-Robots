# EXP-20260902-001-virtual-wrist-segment-contact-scan: Execution Failure Note

日期：2026-09-02

状态：`failed_before_manifest`，未产生任何候选模型观察或能力结论。

运行器在创建 `research/artifacts/EXP-20260902-001-virtual-wrist-segment-contact-scan/config.json`
后，于第一个配置评估前抛出 `KeyError: 'cups'`。原因是它把
`EXP-20260901-002` 的接触表当作杯位来源；该表只保留逐姿态接触结果，杯位仍在其上游
`EXP-20260901-001` coverage 表中。

该运行器手动进入 `RunRecorder` 而未使用上下文管理器，因此异常没有生成正式 run manifest。
保留本失败说明和已写出的配置产物（SHA-256
`453a572b64485aff6cbec5db24f18f184b4766fc5d70acfe8eb584a4fec84f15`），不伪造 manifest。
后续修正使用新的 experiment ID，并改为两个明确输入和上下文管理器，以便未来未处理异常也会
写入失败 manifest。

这不是对 MuJoCo、SO-101、碰撞、可见性或真实硬件的结论。

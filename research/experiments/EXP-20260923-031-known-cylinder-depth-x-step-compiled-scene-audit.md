# EXP-20260923-031-known-cylinder-depth-x-step-compiled-scene-audit: Experiment Plan

030 的每个观测已记录 `compiled_cup_center_matches_configuration=true`，但该字段没有进入通过布尔式。为避免把
未门禁的字段当成前提，本 run 不重跑感知或 Agent 链，只隔离审计每个 X 条件的重新编译场景。

在 030 结果和候选模型哈希冻结后，逐一编译配置的三组 cup X，检查无挡板、cup/table/camera/robot state 与配置
相等，且实际 cup Y/Z 保持冻结、X 顺序等于声明值。全部审计条件必须进入通过谓词。它若失败，后续不得引用
030 的 X-only 结论；若通过，只补齐虚拟场景配置门禁，不能证明感知、精度、可达性或执行能力。

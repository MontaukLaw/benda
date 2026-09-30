# CH5 三分钟快照分析

结论：CH5 在已卸载、Signal 为零时仍长期停留 FLOATING；两次 1.5 N 按压之后重复出现 FALLING → FLOATING，而未进入松手恢复。CSV 的真实上报状态已异常，不只是界面标签问题。

## 数据完整性

- 固定快照导出时间：2026-09-20 09:33:37.290390 +08:00。
- 实际传感器范围：09:30:37.294—09:33:37.290，179.996 秒。
- 26965 条传感器样本，全部为 debug；未将 run_force、channel_4567_statistics 和 temperature 重复计入。
- 历史缓存完整，最大传感器采样间隔 33 ms，无长时间采样中断。
- recent_3min.csv 与保留的同次导出副本 SHA256 均与元数据一致：`c2fc722b779a0226a38ec25241552009be91a4aed218703b54eb7f021f7052ca`。
- 分析脚本绑定同次导出的归档 CSV，避免后续转存覆盖 recent_3min.csv 后误用新数据。
- 压力与电机交叉证据来自 force_press_20260920_092626_016.csv；已将同时间范围保存为 motor_snapshot.csv。以下压力范围只统计压力样本年龄不超过 500 ms 的记录。

## 确认的异常

CH5 全窗口 Baseline 固定为 2359；26965 条样本中 FLOATING 26001 条、RISING 581 条、FALLING 383 条，没有任何 IDLE、SLOW_BACK、WAIT_STABLE、RECOVER 或 ZERO_REBASE 样本。

| 卸载时段 | 压力计 | 电机位置 | CH5 Raw | Raw−Baseline | Signal | 状态 |
|---|---|---|---|---|---|---|
| 09:30:37.294—09:32:48.500 | 0.01–0.05 N | 3 step | 2832–2852 | 473–493 | 始终 0 | 始终 FLOATING |
| 09:32:59—09:33:10 | 0.03–0.04 N | 4 step | 2868–2913 | 509–554 | 始终 0 | 始终 FLOATING |
| 09:33:24—09:33:37.290 | 0.03–0.04 N | 3 step | 2874–2916 | 515–557 | 始终 0 | 始终 FLOATING |

第一段卸载超过 131 秒。作为对照，最初 120 秒 CH4、CH6 均为 IDLE。

两次松手路径：

1. 09:32:56.800 开始释放，09:32:58.132 电机回到 4 step、压力 0.03 N。CH5 于 09:32:57.114 进入 FALLING，09:32:58.389 回到 FLOATING，此时 Signal=0，Raw=2931。
2. 09:33:20.847 开始释放，09:33:22.693 电机回到 3 step、压力 0.03 N。CH5 于 09:33:21.176 进入 FALLING，09:33:22.471 回到 FLOATING，此时 Signal=0，Raw=2950。

## 当前源码解释及证据边界

- channel_state_machine.c:1091：分类使用 Raw 减锁定的 activation_baseline，不读取补偿后的 Signal。
- channel_state_machine.c:549：松手候选首帧要求 Fast K 达到 −250 %/s，并同时满足峰值回落条件。普通下降可以进入 FALLING，却不一定触发 SLOW_BACK。
- channel_state_machine.c:830：另一条活动退出条件依赖差值足够低，或已降到历史活动峰值的 12% 以下且稳定。
- channel_state_machine.c:851：FLOATING 未满足退出阈值时继续维持；普通活动状态稳定后也可再次进入 FLOATING。
- baseline_tracker.c:16：基线只在 IDLE、RECOVER、ZERO_REBASE 跟随；FLOATING、RISING、FALLING 均冻结。
- sensor_algorithm.c:1969：FLOATING 锁住进入时的 Signal，并更新蠕变补偿；锁定值允许为 0。因此 Signal 已归零不意味着状态机的压力差值已归零。

这些条件存在恢复缺口：未识别松手 → 继续活动状态 → 基线冻结 → 残差长期偏大 → 空闲退出条件难以满足；Signal 补偿却可以继续给出零。记录与该机制一致。

不能将上报基线直接当作已观测到的内部 activation_baseline，也不能把窗口内最大 Raw−Baseline 当作完整历史 peak_pressure。CSV 未直接记录这两个内部值、临时零点与蠕变补偿值；稀疏上报的 Fast K 也不能证明固件每个内部采样都未经过 −250 %/s。因此这里是源码路径解释，并非内部状态逐帧复现。运行固件与当前源码的一致性也未通过本次快照单独验证。

快照第一帧已是 FLOATING，无法仅凭这三分钟确定最初触发时间，或把约 500 counts 偏移唯一归因于材料残余、漂移或此前参考变化。

## 修正建议

优先补足真实状态机的松手恢复路径：验证当前一轮回落幅度、下降持续时间与后续稳定情况，避免把所有松手都绑定在单一瞬时 −250 %/s 门槛上；同时审查活动参考与临时零点的关系。应覆盖慢松手、带残余的释放、连续轻压、长时间稳定保持，防止把仍在保持的轻压吸收成零点。

所有状态转换仍由 channel_state_machine.c 的 commit_state() 提交。不能直接用 Signal=0 改为空闲，也不能由上位机或压力计覆盖通道状态。

本次只分析并生成证据文件，未修改固件、未构建 EXE。

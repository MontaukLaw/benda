# 慢释放与回弹恢复修正

修改 `channel_state_machine.c/.h`，仅对DEVICE_MATRIX20启用。40/32 counts激活/空闲门槛及上一版IDLE慢基线系数保留。

1. 慢释放：当压力差值回落至峰值25%以内、快慢斜率稳定并完成空闲确认，进入真实WAIT_STABLE/RECOVER流程；无需必须先出现快速下降脉冲。持续按压和仅部分卸载不因FLOATING持续时间被清零。
2. 恢复中再次按压：同时超过恢复前可信参考、当前Baseline和局部参考40 counts，并满足200 counts/s快速上升。局部低谷回升、尚未高于原基线的情况不再触发RISING。真实快速再按仍可中断恢复。
3. 恢复稳定性：完整慢窗口斜率绝对值不超过5 counts/s，快速窗口峰峰值不超过12 counts，并持续确认500 ms。替换矩阵原有的“每帧变化≤2”及2秒后无条件恢复。RECOVER中重新发生运动则返回WAIT_STABLE，保持基线冻结；SLOW_BACK确认释放后允许稳定残余进入恢复，不强求Raw自行回到冻结基线附近。
4. IDLE遇到超过40 counts的负向动态形变，进入公开的WAIT_STABLE状态并冻结基线，回弹稳定后再恢复。不会把负向形变持续积累进空闲零点。
5. 新增的恢复参考只属于同一个通道状态机，不是隐藏状态机。运行时current_state写入仍只在commit_state中，初始化和重校准仍调用ChannelState_Reset。

## 验证

- 最新日志从21:01:54开始，95 Hz内部时基，修改前精确复现CH3/8/13/16/18五路FLOATING。修改后15191帧回放末尾20路均IDLE、Signal全0，`--assert-released`通过。结果：`replay_fixed95.json`。
- 旧日志从20:29:01开始，115 Hz内部时基，修改后末尾20路均IDLE、Signal全0，`--assert-released`通过。结果：`replay_oldlog_fixed115.json`。
- 固件模拟测试覆盖74/95/200 Hz慢释放、低谷回升不能激活、恢复中真实再次按压；87/115/200 Hz轻压保持、部分卸载、快速松手、过期下降证据；20路独立基线、长期漂移与重校准。
- 95 Hz、±3 counts量化噪声专项测试通过，避免严格Fast斜率导致WAIT_STABLE无法结束。
- 最终构建重新核验20路三参数映射及DFU入口保留RAM。全部通过模拟器，不打开真实串口。

编译更新INDEX_FINGER/UART HEX与签名CYACD2，SHA256见 `fixed_firmware_manifest.json`。未执行DFU/SWD，未打包EXE。启动锁定时基与实际帧率差异仍未在本次更改，实板释放时间需要用户烧录后复核；回放通过不替代实板验证。恢复期间慢于200 counts/s的真实再按可能延迟识别，这是为抑制材料回弹采用的恢复阶段门槛；普通IDLE仍保留原20 counts/s正向门槛。

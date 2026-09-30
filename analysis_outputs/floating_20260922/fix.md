# 矩阵释放残留修正

修改仅作用于20通道矩阵（DEVICE_MATRIX20），其他手指沿用原门槛。

|项目|原值|修正值|
|---|---:|---:|
|激活最小差值|12 counts|40 counts|
|返回空闲差值|10 counts|32 counts|
|IDLE基线Q16跟随系数（160 Hz参考）|327|33，约慢10倍|
|小压力（峰值≤300）释放回落比例|65%|35%|
|小压力释放起始Fast K|−250 %/s|−50 %/s|
|小压力释放连续确认|50 ms|100 ms|

中压力在300～1000 counts之间连续插值，大压力仍保留15%回落、−250 %/s和单帧确认。FLOATING专用进入、退出斜率未改变。

矩阵状态机保存最近500 ms内的快速下降证据，允许下降斜率与回落比例先后达标，避免Raw已经回落但斜率变平后永远漏掉松手。回落比例仍须持续满足，明显回升会清除连续确认；进入释放、空闲或恢复时清除下降证据，重新校准通过ChannelState_Reset清空。不增加隐藏通道状态，运行时切换仍只由commit_state提交。

改动涉及 `baseline_tracker.c`、`channel_state_machine.c/.h`。生成INDEX_FINGER/UART的HEX、CRC签名CYACD2，未连接串口，未执行DFU/SWD，未打包EXE。

## 回放验证

固定输入 `full_debug_snapshot.json.gz` 保存原始采样。脚本 `tools/replay_matrix20_release.py` 从20:29:01参数切换后的已释放区间初始化，随后把所有Raw逐帧送入编译后的真实ARM SensorAlgorithm_Process（Unicorn），并运行真实20通道基线/Signal/状态机链路。

旧固件在115 Hz状态机时基下复现末尾7路FLOATING，通道、Baseline和Signal与原日志一致。修正后115 Hz和87 Hz回放末尾均为20路IDLE、20路Signal为0。87 Hz使用 `--assert-released` 通过断言。回放结果保存在 `replay_old.json`、`replay_final115.json`、`replay_final87.json`；固件SHA256保存在 `firmware_manifest.json`。最终下降证据版本的保持/释放/证据过期测试和DFU入口模拟测试通过。

独立算法用例覆盖87/115/200 Hz：负向200 counts持续1秒后回弹、60 counts轻压持续保持、轻微蠕变、释放残余20 counts、200→150部分卸载保持、随后200→80释放、过期下降证据不能触发之后的慢速卸载、长期基线漂移与重校准。另有20路参数映射及DFU入口保留RAM检查，全部在模拟器执行，不操作实际设备。

阈值修改会降低极轻触灵敏度：不超过40 counts的正差值不会启动新的激活。小压力快速回落35%也比旧版更容易判为释放，应在实板验证轻压、部分卸载和实际松手体验。未增加FLOATING超时强制清零。

此前发现的时基启动锁定偏差不在本次阈值修正中更改；模拟回放分别覆盖原日志推算时基和实际接收帧率。没有把模拟回放当成实板确认。

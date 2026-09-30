# PSoC 4100TP Equivalent Cs 实现说明（V8 已暂停上传）

> 历史计算方法，非现行协议或默认配置依据。当前INDEX启动三参数为Nsub=300、TxDivider=100、RefCDAC=30；生成配置与运行时覆盖须区分。参见[当前实现](CURRENT_SYSTEM_CN.md)和[固件变体](FIRMWARE_VARIANTS_CN.md)。

> 本文保留历史计算方法。UART 协议 V8 已停止发送 Type `0x25`，Python 上位机也已移除 Cs 列；当时Nsub实验观察Raw、Noise、接收FPS和饱和平台；V20/V21 Debug已移除固定Noise字段。

## 历史工程审计（下列为当时配置）

- 构建目标：`APP_CY8CPROTO-041TP`，器件 `CY8C4147LQQ-T493`。
- CAPSENSE Configurator：11.0.0；CAPSENSE Middleware：9.0.0。
- 实际 sensing method：全部 widget 均为 5th-generation LP MSCLP `CSX`，不是 CSD。MatrixButtons0 为 `CSX_MATRIX_BUTTONS`。
- MatrixButtons0 当前为 single-CDAC：生成配置 `cdacConfig=257`，Comp CDAC mode 位为 Disabled，四个 sensor 的 Ccomp 均为 0。
- MatrixButtons0 生成值：Nsub 200、Cref code 96、Tx/SENSE divider 80、Comp divider 16、Ccomp code 0、CDAC dither scale 3。
- `SNS_CLK=16` 是生成配置中的 row sense clock 字段；当前 CSX-RM 公式实际使用 MSCLP `SENSE_DIV` 对应的 Tx clock divider 80。
- CIC2 filter、CIC rate auto 和 CIC2 shift auto 均启用。因此换算使用 Middleware 通过饱和扫描得到的当前 `maxRawCount`，不把 `Nsub * TxClkDiv` 当作固定硬编码 MaxCount。Nsub 动态修改后会重新执行 `Cy_CapSense_InitializeMaxRaw()`。
- CDAC 单位取 Middleware 9.0.0 的 `CY_CAPSENSE_REF_CDAC_LSB_X100=887`，即 8.87 fF/code；固件不另存一份常数。

## 官方 CSX-RM 关系与反算

single-CDAC：

```text
Raw = MaxCount * Cm / (TxClkDiv * (Cref / 2))
Cm  = Raw / MaxCount * TxClkDiv * (Cref / 2)
```

dual-CDAC：

```text
Raw = MaxCount * (Cm - TxClkDiv / CompClkDiv * Ccomp)
                   / (TxClkDiv * (Cref / 2))

Cm  = Raw / MaxCount * TxClkDiv * (Cref / 2)
      + TxClkDiv / CompClkDiv * Ccomp
```

其中 `Cref = RefCDACCode * CDAC_LSB`，`Ccomp = CompCDACCode * CDAC_LSB`。UI 按产品字段名显示为 `Cs [pF]`；对当前 CSX 硬件，其物理含义是等效 sensor mutual-capacitance `Cm`。

## 实时参数来源

`CapSense_GetEquivalentCs(sensor_id)` 每帧读取：

- RAW：`cy_stc_capsense_sensor_context_t.raw`，未使用 CH4..CH7 的应用层经验缩放值；
- Nsub：Middleware 当前 sensor-frame 的 `SNS_SCAN_CTL.NUM_SUB_CONVS + 1`；
- Tx/Sense divider：sensor-frame `SNS_CTL.SENSE_DIV + 1`；
- Compensation divider：sensor-frame `SNS_SCAN_CTL.COMP_DIV + 1`；
- Cref/Ccomp：sensor-frame `SNS_CDAC_CTL.SEL_RE/SEL_CO`；
- MaxCount：对应 widget context 的 `maxRawCount`，由 Middleware 饱和扫描获得；
- CDAC LSB：当前 Middleware 宏 `CY_CAPSENSE_REF_CDAC_LSB_X100`。

参数 pending 更新安排在本帧 RAW/Cs 采集发送之后、下一次扫描之前，避免出现“旧 RAW 配新参数”的单帧错配。

## UART 与 UI

- 保留 Type `0x10` Debug 帧和全部既有字段、长度。
- 新增 Type `0x25`：`float32 cs_pf[channel_count]`，8 通道 payload 为 32 bytes，总帧长 42 bytes。
- Type `0x25` 在 Debug 模式下每 4 个原始 Debug 帧发送一次；旧上位机跳过未知帧，不影响现有数据。该降频仅作用于 Cs，原数据帧率不变。
- 实际使用的 `C:/Kaichen/program/yingfeiling/INDEX_MIDDLE_RING/uart_ui/algo_monitor` 上位机通道表新增 `Cs [pF]` 列，显示三位小数，例如 `12.438 pF`；CSV 同步增加每通道 `cs_pf` 字段。

## 验证状态

- UART 与 I2C Debug 固件均已编译通过；修正版通过 COM5 UART DFU 烧录。
- COM5 实测 Run→Debug：3 秒收到 495 个 Debug、495+495 个力向量、124 个 Cs 和 1 个 ACK，切换后持续输出。
- 实际 `uart_ui/algo_monitor/main.py` 完整连接测试 4 秒：756 个通道帧、约 189.4 FPS，8 个 Cs 标签均更新。
- 下表为无按压、设备静置时实测中位数；测试结束已恢复 Nsub=200、Cref=96。Ccomp 始终为 0。

| Profile | CH | RAW | Baseline | Signal | Cs [pF] | Nsub | Cref | Ccomp |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A | 4 | 1146 | 1146 | 0 | 0.712256 | 200 | 96 | 0 |
| A | 5 | 3283 | 3282 | 0 | 1.697105 | 200 | 96 | 0 |
| A | 6 | 4996 | 4998 | 0 | 2.253829 | 200 | 96 | 0 |
| A | 7 | 4227 | 4228 | 0 | 2.574235 | 200 | 96 | 0 |
| B | 4 | 573 | 572 | 0 | 0.724127 | 100 | 96 | 0 |
| B | 5 | 1624 | 1624 | 0 | 1.706393 | 100 | 96 | 0 |
| B | 6 | 2470 | 2471 | 0 | 2.265133 | 100 | 96 | 0 |
| B | 7 | 2088 | 2088 | 0 | 2.585850 | 100 | 96 | 0 |
| C | 4 | 2227 | 2227 | 0 | 0.691922 | 200 | 48 | 0 |
| C | 5 | 6437 | 6439 | 0 | 1.663580 | 200 | 48 | 0 |
| C | 6 | 9801 | 9800 | 0 | 2.210687 | 200 | 48 | 0 |
| C | 7 | 8302 | 8305 | 0 | 2.527796 | 200 | 48 | 0 |

Nsub 从 200 降到 100 时 RAW 约减半，而 Cs 变化约 0.45%–1.67%；Cref 从 96 降到 48 时 RAW 约翻倍，而 Cs 变化约 -1.80%–-2.86%。

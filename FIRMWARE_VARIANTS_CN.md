# 多手指固件编译、硬件配置与参数差异

> 当前UART为V21，I²C/SensorBus独立。现行功能与历史资料的分工见[当前实现](CURRENT_SYSTEM_CN.md)。烧录或维护Bootloader前须核对[Flash边界及跨工程差异](FLASH_LAYOUT_CN.md)，不能沿用旧App1长度。

> 术语约定：本文及本项目后续提到“三参数”时，固定指CapSense的`Nsub`、`TxDivider`和`RefCDAC`。

2026-10-05通信及构建入口核对：当前`INDEX_FINGER`已是4TX×5RX的20点MatrixButtons0，Makefile默认`RAW_ONLY=0`，使用[V21 DATA协议](UART_PROTOCOL_UNIFIED_V3_CN.md)。Baseline宏默认0，采样发送Type `0x16`（总长50字节）；宏设为1恢复通道算法后Run仅发Signal、其余模式发Debug，不恢复旧四通道合力及FLOATING锁定。启动使用Configurator生成值，本次为`Nsub=200、TxDivider=100、RefCDAC=30`，旧固定参数覆盖未启用。下文旧食指8通道映射、时基及固定参数表是历史配置，不适用于当前矩阵；THUMB/PINKY仍为5个独立Button。

只有显式`RAW_ONLY=1`才使用`raw_stream.c`的46字节采集协议，构建命令和配套脚本见[RAW20说明](RAW20_SERIAL_CN.md)。它仅支持20点食指UART，不能用于原GUI。当前I2C构建使用`app_iic.c`，20通道数据帧90字节、5通道45字节，见[I2C协议](SENSOR_BUS_PROTOCOL_CN.md)。

## 1. 编译选择

同一套 C 源码通过 Make 变量 `FINGER_TYPE` 选择设备。它同时完成两件事：

1. 向 C 编译器传入 `DEVICE_FINGER_TYPE`，由 `device_variant.h` 选择设备名、通道数、时基和能力；
2. 选择与该硬件匹配的 CapSense 生成源码。CapSense 的 `cycfg_*` 符号在两套配置中同名，不能只靠 C 的 `#if` 切换，因此必须在 Make 的源码发现阶段排除另一套配置。

| `FINGER_TYPE` | 设备名 | `device` | 通道数 | CapSense 拓扑 |
| --- | --- | ---: | ---: | --- |
| `INDEX_FINGER` | `INDEX_FINGER` | 1 | 20 | 当前1个4TX×5RX MatrixButtons0；旧8通道见历史映射 |
| `THUMB` | `THUMB` | 2 | 5 | 5个独立Button |
| `PINKY` | `PINKY` | 3 | 5 | 5个独立Button |

编译命令：

```powershell
make build FINGER_TYPE=INDEX_FINGER TRANSPORT_OPT=UART CONFIG=Debug
make build FINGER_TYPE=THUMB TRANSPORT_OPT=UART CONFIG=Debug
make build FINGER_TYPE=PINKY TRANSPORT_OPT=UART CONFIG=Debug
```

也可使用脚本：

```powershell
.\build_firmware.ps1 -Target UART -FingerType INDEX_FINGER
.\build_firmware.ps1 -Target UART -FingerType THUMB
.\build_firmware.ps1 -Target UART -FingerType PINKY
```

输出目录按手指隔离：`build/<FINGER_TYPE>/<TRANSPORT_OPT>/...`。默认V21文件名含手指类型；显式RAW20使用`mtb-example-psoc4-raw20`，同目录可能留有另一构建的旧产物，须核对文件名和构建选项。

## 2. 配置文件位置

| 内容 | 位置 | 维护方式 |
| --- | --- | --- |
| 当前食指20点源配置及生成结果 | `bsps/TARGET_APP_CY8CPROTO-041TP/config/`及`GeneratedSource/` | 用Configurator修改并重新生成；`templates/`的旧8点配置不能代替当前BSP |
| 拇指/小指5通道源配置 | `firmware_variants/thumb_pinky/config_sources/design.modus`、`design.cycapsense` | 来源为 `THUM_PINKY_NEW`，修改后需要重新生成下方源码 |
| 拇指/小指5通道生成源码 | `firmware_variants/thumb_pinky/generated/` | Configurator 自动生成，不手工修改 |
| C层变体入口 | `device_variant.h` | 增加设备类型或分离参数时从这里扩展 |

`THUMB` 和 `PINKY` 目前共用完全相同的5通道引脚和CapSense参数，只通过设备身份区分。真机测试发现两者需要不同参数时，应先在 `device_variant.h` 中建立独立参数宏；只有引脚或CapSense Widget配置不同，才再增加一套生成配置。

## 3. 传感器和引脚映射

### 3.1 INDEX_FINGER历史8通道映射

当前20点矩阵的`CH(4×r+t)`对应MatrixButtons0的`RXr/TXt`（r=0..4，t=0..3），真实引脚以当前BSP生成配置为准。以下只保留旧8点产品映射。

| 逻辑通道 | Widget/Sensor | Rx | Tx | 说明 |
| ---: | --- | --- | --- | --- |
| CH0 | Button8/Rx0 | P0.2 | P0.0 | 独立Button |
| CH1 | Button7/Rx0 | P0.5 | P0.7 | 独立Button |
| CH2 | Button6/Rx0 | P2.4 | P2.7 | 独立Button |
| CH3 | Button5/Rx0 | P2.2 | P2.0 | 独立Button |
| CH4 | MatrixButtons0/Rx0_Tx0 | P1.4 | P1.7 | 矩阵左上 |
| CH5 | MatrixButtons0/Rx0_Tx1 | P1.4 | P1.6 | 矩阵左下 |
| CH6 | MatrixButtons0/Rx1_Tx0 | P1.3 | P1.7 | 矩阵右上 |
| CH7 | MatrixButtons0/Rx1_Tx1 | P1.3 | P1.6 | 矩阵右下 |

### 3.2 THUMB / PINKY（5通道）

| 逻辑通道 | Widget/Sensor | Rx | Tx |
| ---: | --- | --- | --- |
| CH0 | Button0/Rx0 | P2.7 | P3.1 |
| CH1 | Button1/Rx0 | P1.4 | P1.7 |
| CH2 | Button2/Rx0 | P2.1 | P2.4 |
| CH3 | Button3/Rx0 | P0.7 | P0.5 |
| CH4 | Button4/Rx0 | P0.0 | P0.2 |

两种配置均使用 P1.0/P1.1 作为 CapSense 模拟外部电容相关引脚，UART等公共板级引脚来自各自的 `design.modus`。

## 4. 两边参数差异

### 4.1 软件算法时基

| 参数 | INDEX_FINGER | THUMB/PINKY | 原因 |
| --- | ---: | ---: | --- |
| `SENSOR_ALGO_CHANNEL_COUNT` | 8 | 5 | 实际Sensor数量不同 |
| `SENSOR_ALGO_FRAME_RATE_HZ` | 160 Hz | 250 Hz | 参考工程5通道完整扫描更快 |
| Fast K窗口 | 16帧 | 25帧 | 都约100 ms |
| Slow K窗口 | 160帧 | 250帧 | 都约1 s |
| `SENSOR_ALGO_BASELINE_ALPHA_Q16` | 408 | 261 | 都保持约3 s的Baseline时间常数 |

状态阈值、Signal滤波、释放恢复等算法参数目前共享同一份源码，包括轻触调整后的：

- `CHANNEL_STATE_PRESSURE_ON_MIN = 12`
- `CHANNEL_STATE_PRESSURE_ON_NOISE_MULTIPLIER = 3`
- Baseline冻结门槛同样使用 `max(Noise×3, 12)`

这意味着当前不会因为切换手指而偷偷恢复旧的 `20/4` 标准。后续个体微调应优先把确有差异的值提到 `device_variant.h`，不要复制整个算法文件。

### 4.2 CapSense拓扑和采样参数

| 项目 | INDEX_FINGER | THUMB/PINKY |
| --- | --- | --- |
| Widget | Button5～8 + MatrixButtons0 | Button0～4 |
| Sensor总数 | 8 | 5 |
| 全通道运行时Nsub | 300 | 保留各Button配置 |
| 全通道运行时TxDivider | 100 | 保留各Button配置 |
| 全通道运行时Ref CDAC | 30 | 保留各Button配置 |
| 矩阵自动量程 | 禁用，启动默认Ref=30 | 禁用 |
| 5通道运行方式 | 不适用 | 保留每个Button的Configurator参数，不做矩阵统一切档和归一化 |

INDEX_FINGER在首次扫描前由运行时代码把CH0～CH7全部强制同步为`Nsub=300`、`TxDivider=100`、`RefCDAC=30`。同步同时覆盖CapSense中间件配置和每个Sensor Frame硬件镜像，并重新计算CIC2及Max Raw；因此生成配置里的原始默认值不代表启动后的实际值，实际值以`0x97/0x2C`逐通道回读为准。

食指不再执行CH4～CH7自动量程、Ref归一化或切档过渡，算法与V20/V21遥测均使用单一Hardware Raw，不再上传Norm/Candidate Raw。三个手指固件均支持通过`0x8E`易失性统一修改全部逻辑通道的三参数；修改在扫描空闲点应用，并重建CIC2、Max Raw、滤波历史和Baseline，重新上电后恢复各固件的启动默认值。

5通道配置中各Button的主要阈值也不完全相同：

| Widget | Finger Threshold | Noise/NNoise Threshold | Low Baseline Reset |
| --- | ---: | ---: | ---: |
| Button0 | 100 | 40/40 | 30 |
| Button1 | 100 | 30/30 | 255 |
| Button2 | 100 | 30/30 | 255 |
| Button3 | 150 | 50/50 | 255 |
| Button4 | 100 | 40/40 | 255 |

这些是 CapSense 中间件自己的Widget阈值；本工程最终状态判断仍使用自定义 `channel_state_machine` 的Signal/Noise标准，两者不要混为同一个阈值。

其余扫描参数的逐项对比结论如下：

| 参数 | INDEX_FINGER独立Button | THUMB/PINKY独立Button | 结论 |
| --- | --- | --- | --- |
| `SNS_CLK` / `TX_CLK` | 16 / 100 | 16 / 100 | 相同 |
| `CLK_SOURCE` | `SSC_AUTO` | `SSC_AUTO` | 相同 |
| `NUM_SUBCONVERSIONS` | 生成值255，启动后强制300 | 300 | 食指运行值统一为300 |
| `CDAC_REF0` | 生成值96，启动后强制30 | 90 | 食指运行值统一为30，五通道保留生成配置 |
| `CIC_RATE` | Button5为30，其余为64 | Button1为30，其余为64 | 与物理通道布局对应的单路例外被保留 |
| `CDAC_FINE0` / `COMP_DIV` | 通常0 / 8 | Button0为1 / 16，其余0 / 8 | Button0保留参考工程的独立补偿设置 |
| `HYSTERESIS` / `ON_DEBOUNCE` | 10 / 3 | 10 / 3 | 相同 |
| CM/IIR/Median/Average/Jitter/AIIR过滤 | 全部关闭 | 全部关闭 | 相同，滤波主要由本工程算法层完成 |

INDEX_FINGER的`CDAC_FINE0=1`、`COMP_DIV=16`等非档位参数仍来自生成配置；Nsub、TxDivider和Ref CDAC则由启动固定配置覆盖。没有列入上表的大量默认手势、质心和边缘校正字段在两份配置中一致且对当前Button原始计数路径无实际影响；源配置可追溯值以两份`design.cycapsense`为准，运行生效值以`0x2C`回读为准。

### 4.3 运行时参数回读与Raw余量判断

协议V15新增`0x97`查询和`0x2C`响应。上位机连接后会逐通道显示：

- Nsub：中间件配置值 / Sensor Frame硬件值；
- TxDivider：中间件配置值 / Sensor Frame硬件值；
- RefCDAC：中间件配置值 / Sensor Frame硬件值；
- Max Raw：当前CIC2参数对应的理论Raw满量程。

THUMB/PINKY的五个独立Button必须逐路读取，不能继续用Button0代表全部通道。INDEX_FINGER的CH4～CH7共用矩阵Widget参数，但仍逐Sensor核对硬件镜像，以便发现运行时切档只同步了部分扫描槽的问题。

分析Raw时优先计算`Raw占比 = Hardware Raw / Max Raw`。绝对Raw为20000并不等同于饱和；如果Max Raw约为58928，占比约34%，仍有较大正向余量。真正需要调整参数的信号包括：空载已接近量程高水位、按压很快触顶、噪声相对有效增量过大，或同一硬件的空载Raw突然明显偏离历史值。参数正常但空载Signal长期为数百且状态为RISING时，还应检查上电校准期间是否完全卸载、Baseline是否已经稳定，以及结构预紧或接线串扰。

## 5. 设备自动识别

上位机每次成功打开串口后发送广播查询 `0x96`。固件返回 `0x2B` 设备信息帧，内容包括手指类型、通道数、能力位和ASCII设备名。上位机将设备类型显示在监控页最上方的橙色醒目区域，同时写入窗口标题；5通道设备会自动隐藏只适用于矩阵的自动量程和仿射校准面板。

设备身份返回后，上位机继续发送`0x97`并在监控页显示全部通道的当前CapSense参数。该只读诊断区对5通道和8通道设备都可见。

这套查询应答不依赖“串口打开瞬间是否收到了上电广播”，因此自动重连、固件升级后重连和手动连接使用同一流程。

## 6. 上位机合力

固件宏 `SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT` 已关闭。上位机在通道表增加“合力选择”列，每帧按以下方式计算：

```text
SelectedSignalSum[t] = Σ Signal[channel][t]，channel为已勾选且当前设备存在的通道
```

默认选择：食指为CH4～CH7，首次识别5通道设备时为CH0～CH4。该和值是Signal计数的算术和，不等同于已经完成牛顿标定的三维矢量模长。

实时Hardware Raw噪声仍由上位机对最近64帧计算总体标准差，但表格列已移动到最右侧，并改为对当前设备的所有通道计算。

## 7. 增加新手指类型时的检查清单

1. 在 `device_variant.h` 增加类型值、名称、通道数、帧率和能力位。
2. 在 `Makefile` 增加 `FINGER_TYPE`合法值和对应生成源码目录。
3. 用Configurator建立并保存该硬件的`design.modus`与`design.cycapsense`，生成独立`cycfg_*`。
4. 在 `Sensor_ReadRawCounts()` 增加与生成宏一致的逻辑通道顺序。
5. 编译期确认`SENSOR_ALGO_CHANNEL_COUNT == CY_CAPSENSE_SENSOR_COUNT`。
6. 更新设备信息协议枚举和上位机中文显示名。
7. 分别验证Debug帧长度、实时噪声、合力默认勾选和矩阵能力面板显示。
8. 对每个固件变体至少完成一次UART Debug编译，确认HEX和CYACD2产物位于独立目录。

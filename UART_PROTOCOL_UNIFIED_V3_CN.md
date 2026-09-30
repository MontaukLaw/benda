# 多指传感器统一 UART 协议 V21

现行版本：**V21（2026-09-20）**。文件名保留历史V3命名，不表示设备使用版本3。版本协商、界面入口及尚未解决事项见[当前实现](CURRENT_SYSTEM_CN.md)。

V21变更：主法向支持1～30 N自动标定的完整10/20点表，同时兼容48点表；帧布局沿用V20，FCV9偏移和Flash占用不变。食指在新高力表有效、Signal达到首节点且换算达到1 N时切到高力输出，微力表独立保留。接收端兼容V14～V21；新短表须确认设备版本至少V21后写入。

V20变更（2026-09-19）：Debug改为单一Raw、Baseline、Signal、Fast/Slow K、State，载荷为16×通道数；移除Norm/Candidate Raw和固定Noise，停止Slow Baseline计算及0x15上传。详见[单一Raw协议](RAW_PROTOCOL_V20_CN.md)。其余载荷沿用V19，V21继续使用这些布局。


V19变更：低力标定新增最低检测线（Signal和counts，0关闭模拟）；0x99保存载荷24 B，0x2D响应28 B。Type0x02载荷40 B，bit9标记模拟，尾部追加实际换算值及正常优化值。CH4～CH7 Signal和持续严格超过非零检测线、实际换算低于10 g时，持续模拟原始值8.88 g与优化值10.00～10.99 g，不依赖FLOATING。真实数据独立保留。接收端兼容V14～V19，格式严格按版本区分。

V18变更：Type0x02载荷32 B，原28 B布局保留，尾部追加float32主法向优化值；原始值仍在payload偏移4。食指10/20/50/100 g四锚点软吸附，吸附前后值同帧上传。检测线按用户确认降至0.09 N，量程仍小于1 N，检测/超量程按原始值判断。接收端兼容V14～V18；V17及更早Type0x02载荷仍为28 B。

V17变更：新增独立四点手动低力标定，命令0x98/0x99、应答0x2D。食指主法向仅使用此表输出小于1 N的力值，0.1 N以下归零、≥1 N标记超量程，不再依赖旧48点K表。低力表占0x1F500一行，应用镜像止于0x1F4FF；原K表地址不变。接收端兼容V14～V17，低力命令必须使用已确认的V17。

低力命令、Flash格式及Type0x02掩码位见[四点手动低力标定](LOW_FORCE_CALIBRATION_CN.md)；V21高力接管见[自动标定](HIGH_FORCE_CALIBRATION_CN.md)。本节V16～V19变更为历史兼容说明，不能据此认为现行食指只输出1 N以下压力。

V16变更：法向主K表扩展为48节点；FCV9记录移至0x1F600～0x1FBFF；应用禁止写入0x1FD00及之后所有区域。仿射校准停用，0x92～0x94返回不支持。上位机仍解析V14/V15数据，但只有确认V16后才能暂存或保存新K表。

## 1. 协议概述

设备使用同一套固件，通过 UART 命令选择输出模式：

| 输出模式 | 用途 | 每次采样发送的帧 |
| --- | --- | --- |
| 运行模式（Run） | 客户使用 | 通道信号帧 + 三维力帧 + 1 Hz温度帧 |
| 调试模式（Debug） | 内部开发 | 通道调试帧 + Type 0x02力帧 + 可选详细力向量帧 + 1 Hz温度帧 |
| 力标定模式（Force Calibration） | 法向力、切向力数值标定 | 各受力区标定采样帧 + 1 Hz温度帧 |

设备上电后默认进入运行模式。主机发送“设置输出模式”命令后，可在三种模式之间切换。

三种模式互斥：同一次采样只发送其中一组数据。输出模式仅在当前上电周期内有效，设备重新上电后恢复运行模式。

八通道设备在采样帧前另发Type 0x14统计；详细力向量0x11/0x12受编译开关控制，当前默认关闭。监控页自动压力标定消费Debug真实通道状态及Signal，不要求切入旧的Force Calibration输出模式。

## 2. 基本约定

### 2.1 通信参数

| 项目 | 约定 |
| --- | --- |
| 波特率 | `1000000` bps |
| 串口格式 | 8 数据位、1 停止位、无校验、无流控 |
| 字节序 | 小端（little-endian） |
| 浮点数 | 小端 IEEE-754 `float32` |
| 同步字 | ASCII `DATA`（`44 41 54 41`） |
| 协议版本 | 当前`21`（十六进制`0x15`），命令按设备报告版本发送 |

### 2.2 设备类型

| `device` | 设备版本 | 通道数 | 受力区数量 |
| ---: | --- | ---: | ---: |
| `0` | 广播命令 | `0` | 不适用 |
| `1` | 食指 `INDEX_FINGER` | `8` | `2` |
| `2` | 拇指 `THUMB` | `5` | `1` |
| `3` | 小指 `PINKY` | `5` | `1` |

上行帧中的 `device` 必须填写设备自身类型。下行命令中的 `device` 可以填写目标设备类型，也可以填写 `0` 表示广播。

`device` 用于区分硬件版本，不是设备唯一地址。

## 3. 统一帧格式

数据帧、命令帧和确认帧使用相同的 10 字节帧头：

```text
DATA[4] | version[1] | type[1] | device[1] | channel_count[1] | payload_len[2] | payload
```

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0–3 | `magic` | ASCII[4] | 固定为 `DATA` |
| 4 | `version` | `uint8` | 当前固件为`21`；主机根据设备实际报告版本发送命令 |
| 5 | `type` | `uint8` | 帧类型或命令类型 |
| 6 | `device` | `uint8` | 设备类型 |
| 7 | `channel_count` | `uint8` | 数据帧为实际通道数；命令帧和确认帧为 `0` |
| 8–9 | `payload_len` | `uint16` | payload 字节数，小端 |
| 10– | `payload` | 字节数组 | 按 `type` 解析 |

接收端首先在字节流中搜索 `DATA`，随后读取剩余帧头，并根据 `payload_len` 读取完整 payload。协议不包含序号和校验和。

当前上位机兼容V14～V21上行帧及身份查询；V15及以后支持逐通道参数查询。控制命令使用设备报告版本；48点0x84/0x85要求至少V16，主法向10/20点表额外要求至少V21。

### 3.1 帧类型分配

| `type` | 方向 | 名称 |
| ---: | --- | --- |
| `0x01` | 设备 → 主机 | 运行通道信号帧 |
| `0x02` | 设备 → 主机 | 运行三维力帧 |
| `0x03` | 设备 → 主机 | 1 Hz芯片内部温度帧 |
| `0x10` | 设备 → 主机 | 调试通道数据帧 |
| `0x11` | 设备 → 主机 | 主受力区详细力向量帧 |
| `0x12` | 设备 → 主机 | 第二受力区详细力向量帧 |
| `0x13` | 设备 → 主机 | 历史固定通道Signal和值帧；V14固件不再发送 |
| `0x14` | 设备 → 主机 | 固定CH4～CH7的Signal和、Hardware Raw和及总体标准差；仅八通道固件 |
| `0x15` | 历史诊断 | V20已停用；仅保留V15～V19解码兼容 |
| `0x20` | 设备 → 主机 | 力标定采样帧 |
| `0x21` | 设备 → 主机 | 力标定参数帧 |
| `0x22` | 设备 → 主机 | MatrixButtons0 Nsub 状态帧 |
| `0x23` | 设备 → 主机 | MatrixButtons0 CapSense 配置状态帧 |
| `0x24` | 设备 → 主机 | MatrixButtons0 Cref 状态帧 |
| `0x26` | 设备 → 主机 | CH0～CH7 易失性 Nsub 档位状态帧 |
| `0x27` | 设备 → 主机 | 三参数请求值与实际值状态帧 |
| `0x28` | 设备 → 主机 | CH4～CH7 Ref CDAC 自动量程状态帧 |
| `0x29` | 设备 → 主机 | Ref CDAC仿射参数回读帧 |
| `0x2A` | 设备 → 主机 | Ref CDAC仿射参数操作结果帧 |
| `0x2B` | 设备 → 主机 | V14设备身份信息帧 |
| `0x2C` | 设备 → 主机 | V15逐通道CapSense当前参数帧 |
| `0x80` | 主机 → 设备 | 重新校准命令 |
| `0x81` | 设备 → 主机 | 命令确认帧 |
| `0x82` | 主机 → 设备 | 设置输出模式命令 |
| `0x83` | 主机 → 设备 | 读取力标定参数命令 |
| `0x84` | 主机 → 设备 | 临时设置力标定参数命令 |
| `0x85` | 主机 → 设备 | 保存力标定参数命令 |
| `0x86` | 主机 → 设备 | 放弃临时力标定参数命令 |
| `0x87` | 主机 → 设备 | 设置 MatrixButtons0 Nsub 命令 |
| `0x88` | 主机 → 设备 | 读取 MatrixButtons0 Nsub 命令 |
| `0x89` | 主机 → 设备 | 读取 MatrixButtons0 CapSense 配置命令 |
| `0x8A` | 主机 → 设备 | 设置 MatrixButtons0 Cref 命令 |
| `0x8B` | 主机 → 设备 | 读取 MatrixButtons0 Cref 命令 |
| `0x8C` | 主机 → 设备 | 易失性设置 CH0～CH7 Nsub 档位 |
| `0x8D` | 主机 → 设备 | 读取 CH0～CH7 Nsub 档位状态 |
| `0x8E` | 主机 → 设备 | 易失性设置三参数；固定档位固件作用于全部逻辑通道 |
| `0x8F` | 主机 → 设备 | 读取三参数请求值与实际值状态 |
| `0x90` | 主机 → 设备 | 读取 Ref CDAC 自动量程状态 |
| `0x91` | 主机 → 设备 | 控制 Ref CDAC 手动校准模式和档位 |
| `0x92` | 主机 → 设备 | 暂存单通道七档仿射参数 |
| `0x93` | 主机 → 设备 | 保存完整28组仿射参数到Flash |
| `0x94` | 主机 → 设备 | 读取单通道七档仿射参数 |
| `0x95` | 主机 → 设备 | 软件复位设备 |
| `0x96` | 主机 → 设备 | V14读取设备身份信息 |
| `0x97` | 主机 → 设备 | V15读取全部逻辑通道当前CapSense参数 |

V8 不再发送旧的 `0x25` Equivalent Cs 帧。旧 `0x87/0x88` 只控制 MatrixButtons0，并且属于历史兼容接口；新的上位机只使用 `0x8C/0x8D`，以免进入旧的 Flash 持久化路径。

### 3.2 V14 设备身份查询

上位机每次打开串口后发送Type `0x96`，`device=0`、`channel_count=0`、`payload_len=0`。使用广播地址是为了让同一上位机无需预先知道手指类型。

设备返回Type `0x2B`，帧头中的`device`和`channel_count`必须是当前固件的真实类型和真实通道数。payload格式：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `info_version` | `uint8` | 当前固定1 |
| 1 | `finger_type` | `uint8` | 必须与帧头`device`一致：1食指、2拇指、3小指 |
| 2 | `capability_flags` | `uint8` | bit0=支持CH4～CH7矩阵自动量程；bit1=支持易失性全通道三参数手动配置 |
| 3 | `name_length` | `uint8` | 后续ASCII设备名长度，不含结尾零字符 |
| 4～ | `device_name` | ASCII | `INDEX_FINGER`、`THUMB`或`PINKY` |

上位机不应从通道数反推手指类型，因为THUMB和PINKY都是5通道；必须以`finger_type/device_name`为准。查询应答方式也用于升级后自动重连。

### 3.3 V15 逐通道CapSense参数回读

上位机发送空payload的Type `0x97`，设备返回Type `0x2C`。返回值覆盖当前固件的全部逻辑通道，既包含中间件配置区的值，也包含真正用于下一次扫描的Sensor Frame硬件镜像值。二者不一致时说明运行时参数只更新了一侧，应作为错误处理。

Type `0x2C`的`channel_count`等于当前设备实际通道数，payload长度为`4 + channel_count × 14`：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `format_version` | `uint8` | 当前固定1 |
| 1 | `entry_size` | `uint8` | 当前固定14 |
| 2 | `flags` | `uint8` | bit0=全部中间件参数读取成功 |
| 3 | `reserved` | `uint8` | 固定0 |
| 4～ | `entries` | `channel_count × 14` | 按CH0、CH1……顺序排列 |

每个14字节通道记录如下：

| 记录内偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0–1 | `configured_nsub` | `uint16` | 中间件当前Nsub |
| 2–3 | `hardware_nsub` | `uint16` | Sensor Frame中实际Nsub |
| 4–5 | `configured_tx_clk_div` | `uint16` | 中间件当前Tx Clock Divider |
| 6–7 | `hardware_tx_clk_div` | `uint16` | Sensor Frame中实际Tx Clock Divider |
| 8–9 | `max_raw_count` | `uint16` | 中间件根据CIC2配置计算的Raw满量程 |
| 10 | `configured_ref_cdac` | `uint8` | 中间件当前Reference CDAC |
| 11 | `hardware_ref_cdac` | `uint8` | Sensor Frame中实际Reference CDAC |
| 12 | `widget_id` | `uint8` | CapSense Widget编号，供诊断映射 |
| 13 | `sensor_id` | `uint8` | Widget内部Sensor编号 |

上位机连接并识别设备后自动查询一次，也可点击“重新读取”。界面以“配置值 / 硬件值”显示Nsub、TxDivider和RefCDAC，另列显示Max Raw；任何不一致项用红色标记。

#### 3.3.1 V15全通道三参数手动配置

本项目术语“三参数”固定指`Nsub`、`TxDivider`和`RefCDAC`。设备信息`capability_flags.bit1=1`表示支持运行时手动配置。

主机使用Type `0x8E`发送5字节payload：`Nsub uint16`、`TxDivider uint16`、`RefCDAC uint8`，整数均为小端序。合法范围为Nsub `1～16383`、TxDivider `4～4096`且必须为偶数、RefCDAC `7～255`。固定档位固件把同一组三参数同步到全部逻辑通道；参数仅写RAM，在扫描空闲点应用，随后重建CIC2、Max Raw、Raw滤波历史、CapSense Baseline和算法Baseline。重新上电后恢复固件启动默认值。

主机使用空payload的Type `0x8F`查询，设备返回Type `0x27`；固定档位固件返回的实际值代表全通道统一值。设置过程中`pending=1`，完成后主机还应读取Type `0x2C`逐通道核对配置值与硬件值一致。

判断Raw是否过大应使用`Raw / Max Raw`而不是绝对值。空载Raw为两万多只说明当前电极电容、Nsub和CDAC组合产生了这一计数；只要远离Max Raw、短时噪声稳定、加载后仍有足够余量，就不必仅因绝对值大而降低灵敏度。

### 3.4 V14 上位机可选通道合力

V14固件不再周期发送旧Type `0x13`固定和值帧。上位机从Type `0x01`或`0x10`得到各通道Signal后，按界面勾选通道计算：

```text
SelectedSignalSum = Σ Signal[勾选通道]
```

因此合力通道组合可以随设备和测试目的改变，且不会增加串口带宽。该值是Signal计数的算术和，并非三维力矢量的模长。

#### 3.4.1 固定四通道Hardware Raw统计（2026-09-12）

八通道固件每次对外发送采样时先发送Type `0x14`统计帧，随后发送当前模式的数据。Run与Debug均可观察，五通道THUMB/PINKY不发送此帧。该统计载荷沿用V15定义，当前帧头版本为V21；接收端应跳过未知帧类型。

帧头`channel_count=4`表示参与统计的CH4～CH7，不能据此改变设备实际通道数。载荷固定12字节：

| 载荷偏移 | 类型 | 字段 | 单位 |
| ---: | --- | --- | --- |
| 0 | uint32 LE | CH4～CH7 Signal和 | counts |
| 4 | uint32 LE | CH4～CH7 Hardware Raw和 | counts |
| 8 | uint32 LE | Hardware Raw总体标准差×100 | 0.01 count |

令`S=ΣHWRaw[i]`、`Q=ΣHWRaw[i]²`，其中`i=4..7`。平均值为`S/4`，总体标准差为`sqrt(4Q-S²)/4`。固件用64位整数计算`sqrt((4Q-S²)*625)`并四舍五入，避免16位全量程平方溢出和高基数小差值的浮点相消；结果在`0..3276750`之间。

输入直接来自同一扫描的Hardware Raw，不减Baseline、不做归一化、不做时间平均，也不因IDLE、SLOW_BACK等状态清零。它表示逐帧四通道之间的差异，不是各通道时间波动的标准差，也不是一分钟均值之间的标准差。已有合力勾选功能不影响该固定四通道统计。

上位机将统计帧与随后到达的通道帧关联，显示均值、总体标准差和固定Signal和。未收到统计帧或采样中断超过0.5秒显示`--`，真实零显示`0.00`。压力测试CSV新增`ch4567_mcu_hardware_raw_mean`、`ch4567_mcu_hardware_raw_stddev`、`ch4567_mcu_signal_sum`；串口完整记录保留独立的`channel_4567_statistics`帧。标定模式的独立统计帧可在串口完整记录中读取。

#### 3.4.2 已停用的慢基线诊断

V20删除Slow Baseline计算、缓存和上传。旧版Type `0x15`的载荷为N个小端uint32 Q16值，长度4×N；仅供旧数据解析，不计为新的传感器采样。正常Baseline跟踪不受影响。

### 3.5 当前食指固定档位与历史自动量程协议

当前V21的INDEX_FINGER在首次扫描前把CH0～CH7统一设置为`Nsub=300`、`TxDivider=100`、`RefCDAC=30`，并同步CapSense中间件配置与全部Sensor Frame硬件镜像。运行期间不执行矩阵自动量程、Ref归一化或切档过渡，Type `0x10`仅上传一份真实Raw。V15～V19曾为兼容布局重复上传三份相同计数，V20起移除重复字段。

设备信息`capability_flags.bit0=0`表示当前固件没有矩阵自动量程能力，`bit1=1`表示允许通过Type `0x8E`易失性修改全部通道的三参数。Type `0x90`、`0x91`仍返回Type `0x81`结果`2`（不支持命令）；Type `0x8F`用于读取统一三参数状态，诊断每个通道则使用V15的`0x97/0x2C`。

以下自动量程、手动切档和仿射校准内容只记录V10～V13历史协议行为，不代表当前食指固件仍会执行这些功能。

#### 3.5.1 V10～V13历史Ref CDAC自动量程与手动校准

V10 固定 MatrixButtons0 的 `Nsub=300`、`TxDiv=100`、`Max Count=58928`，普通运行期间不接受旧命令设置 Nsub、TxDiv 或 Ref CDAC。Ref CDAC 上电从 12 启动，自动量程共 `12、18、27、40、60、90、135` 七档，135 为最高档。Type `0x87`、`0x8A`、`0x8C`、`0x8E` 设置命令返回结果 `2`（不支持命令）；对应读取命令保留用于诊断。

主机发送 payload 为空的 Type `0x90`，设备返回 Type `0x28`。固件也每 50 个有效输出帧主动发送一次：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0–1 | `nsub` | `uint16` | 固定 300 |
| 2–3 | `tx_div` | `uint16` | 固定 100 |
| 4–5 | `max_count` | `uint16` | 固定 58928 |
| 6 | `actual_ref_cdac` | `uint8` | 当前扫描实际使用的 Ref |
| 7 | `tier_index` | `uint8` | 从 0 开始的当前档位 |
| 8 | `tier_count` | `uint8` | 固定 7 |
| 9 | `state` | `uint8` | 0启动、1保持、2扩档等待、3回落等待、4切换、5冷却、6边界、7错误、8手动校准保持 |
| 10 | `flags` | `uint8` | bit0 pending、bit1最低档、bit2最高档、bit3错误、bit4接触、bit5算法Raw限幅、bit6手动校准模式、bit7仿射已应用 |
| 11 | `requested_ref_cdac` | `uint8` | 自动量程或手动校准最近请求的目标Ref |
| 12–13 | `peak_raw` | `uint16` | CH4～CH7 本帧最大 Hardware Raw |
| 14–15 | `filtered_peak_raw` | `uint16` | 用于普通升降档的滤波峰值 |
| 16 | `high_count` | `uint8` | 高水位连续帧数 |
| 17 | `low_count` | `uint8` | 低水位连续帧数 |
| 18 | `cooldown_frames` | `uint8` | 剩余冷却帧数 |
| 19 | `blank_frames` | `uint8` | 切档过渡剩余帧数；前8帧表示最小等待，速度尚未稳定时保持1，Hermite阶段表示剩余8～1帧，完成后为0 |

Type `0x28` 的 `channel_count=4`、`payload_len=20`。

Type `0x91` 的payload固定为2字节：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `manual_mode` | `uint8` | `1`进入或保持手动校准；`0`退出并恢复自动量程 |
| 1 | `ref_cdac` | `uint8` | 手动模式只接受七个固定档位；退出时必须为0 |

合法命令先返回Type `0x81`成功确认，再立即返回Type `0x28`状态。手动模式会暂停所有自动升降档，切换Ref仍在扫描空闲点执行。每次手动切档在新Ref候选数据等待期沿切档前速度继续预测Norm Raw；至少8帧后，四通道最近3个Candidate差分的极差均不超过128 counts/帧并连续满足3帧，才用8帧三次Hermite曲线对齐新候选轨迹。整条切档链最多等待24帧。只有`flags.bit6=1`、`flags.bit0=0`、`blank_frames=0`且请求Ref等于实际Ref时，上位机才应允许保存校准数据。退出手动模式后，自动量程从当前实际档位继续运行。

### 3.6 V11 Ref CDAC仿射参数持久化（已停用，软件复位除外）

本节0x92～0x94及仿射记录为历史格式，当前不接受保存或应用；0x95软件复位仍有效。不能把以下历史事务当作当前K表写入流程。

V11只完成28组参数的下发、Flash保存、软件复位和启动回读验证。参数虽然加载到RAM，但尚未接入Hardware Raw到Normalized Raw的计算路径。

参数固定覆盖CH4～CH7四个通道和`12、18、27、40、60、90、135`七档Ref。每组参数包含：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `gain_q16` | `int32` | 仿射增益`a × 65536`四舍五入值 |
| `offset` | `int32` | 仿射偏移`b`四舍五入值，单位counts |

整组参数ID使用FNV-1a 32位算法。按CH4到CH7顺序，先加入物理通道号；每档依次加入Ref值、`gain_q16`小端4字节和`offset`小端4字节。固件在保存前重新计算参数ID，防止串口中断、错序或内容损坏形成混合记录。

#### 3.6.1 Type `0x92`：暂存单通道参数

`channel_count=0`，`payload_len=64`：

| 偏移 | 类型 | 说明 |
| ---: | --- | --- |
| 0–3 | `uint32` | 完整28组参数的`calibration_id` |
| 4 | `uint8` | 物理通道号，只允许4～7 |
| 5 | `uint8` | 参数格式版本，固定1 |
| 6 | `uint8` | 档位数，固定7 |
| 7 | `uint8` | 保留，固定0 |
| 8–63 | `7 × (int32 + int32)` | 按固定Ref顺序排列的`gain_q16、offset` |

设备只写RAM暂存区，不修改当前Flash记录，也不应用参数。新的`calibration_id`会清除上一组未完成暂存；相同ID和通道可以安全重发。设备以Type `0x2A`返回操作结果。

#### 3.6.2 Type `0x93`：保存完整参数

`channel_count=0`，`payload_len=4`，payload为`uint32 calibration_id`。只有同一ID的四个通道全部到齐、28组参数范围合法、Ref60保持`gain_q16=65536、offset=0`且固件重算ID一致时才允许保存。

**V16已停用本节仿射功能。** 0x92、0x93、0x94统一返回Type `0x81`结果2，不加载、不应用、不保存旧仿射参数。以下结构仅供维护旧格式；未来重新启用时也只能使用新预留行`0x0001FC00`。

- `0x1F600～0x1FBFF`为FCV9力标定（两个region各占三个Flash row）；
- `0x1FC00～0x1FCFF`预留给停用的仿射记录；
- `0x1FD00`为DFU元数据主行，`0x1FE00`为Bootloader备份行；
- `0x1FD00～0x1FFFF`整体受保护，禁止应用擦写。旧运行配置的0x1FE00保存路径已删除。

记录占252字节，包含魔数`ARAF`、存储版本、设备类型、参数ID、保存代次、七档Ref表、224字节参数和FNV-1a记录校验。固件写入后立即重新从Flash解析并逐字节核对；成功后才返回Type `0x2A`结果0。

#### 3.6.3 Type `0x94` / `0x29`：读取单通道参数

Type `0x94`的payload为一个物理通道号，长度1。设备返回Type `0x29`，`channel_count=4`，`payload_len=72`：

| 偏移 | 类型 | 说明 |
| ---: | --- | --- |
| 0 | `uint8` | 参数格式版本，固定1 |
| 1 | `uint8` | 本帧返回的物理通道号 |
| 2 | `uint8` | 档位数，固定7 |
| 3 | `uint8` | bit0有效、bit1已从Flash加载、bit2存在暂存事务、bit3四通道暂存完整 |
| 4–7 | `uint32` | `calibration_id` |
| 8–11 | `uint32` | Flash成功保存代次，从1开始递增 |
| 12–15 | `uint32` | 最近NVM状态，0表示成功 |
| 16–71 | `7 × (int32 + int32)` | 七档`gain_q16、offset` |

上位机在软件复位后必须确认四个通道的bit0和bit1同时为1、参数ID和代次一致、NVM状态为0，且56个整数与下发内容逐项一致。

#### 3.6.4 Type `0x2A`：仿射参数操作结果

`channel_count=0`，`payload_len=8`：

| 偏移 | 类型 | 说明 |
| ---: | --- | --- |
| 0 | `uint8` | 原命令类型：`0x92/0x93/0x95` |
| 1 | `uint8` | 结果码，含义与Type `0x81`一致 |
| 2 | `uint8` | `0x92`对应通道，其余固定`0xFF` |
| 3 | `uint8` | 保留，固定0 |
| 4–7 | `uint32` | `0x92/0x93`对应参数ID，`0x95`固定0 |

独立结果帧携带原命令类型，避免启动时输出模式命令的通用ACK与多步骤Flash流程互相串扰。

#### 3.6.5 Type `0x95`：软件复位

`channel_count=0`，`payload_len=0`。设备先阻塞发送Type `0x2A`成功结果，确认UART发送完成后调用`NVIC_SystemReset()`。上位机等待设备启动；如果串口因复位消失，则最多自动重连5次，旧仿射流程曾随后通过Type `0x94`依次回读CH4～CH7；该仿射读取现已停用。当前K表保存流程复位后使用`0x83`读取六表，软件复位`0x95`仍有效。

### 3.7 V12 Ref CDAC仿射参数应用（已停用，仅历史格式）

V12将启动时通过Flash完整性、参数ID和范围校验的28组参数接入CH4～CH7的Normalized Raw计算。对物理通道`ch`和当前Ref档位`ref`：

`Norm Raw = round(Hardware Raw × gain_q16[ch][ref] / 65536) + offset[ch][ref]`

固件使用64位中间值计算，小于零的结果限制为零，完整32位Norm Raw仍通过调试协议上传；送入现有算法前保持16位限幅保护。Ref60的参数固定为`gain_q16=65536、offset=0`。

无有效Flash记录、通道非法或Ref不在固定七档表时，固件自动退回V10的`Hardware Raw × Ref / 60`理论比例，不阻断传感器运行。Type `0x28` 的`flags.bit7=1`表示本帧CH4～CH7已实际使用仿射参数；bit7为0表示正在使用理论比例回退。

### 3.8 V13 切档候选Norm Raw诊断（已停用，仅历史格式）

V13在新Ref写入后至少等待8帧Candidate Norm Raw，隔离Hardware Raw暂态；但最终Norm Raw不冻结，而是使用切档前最近4帧差分的中位数速度继续外推。8帧后对每个通道最近3个Candidate差分计算极差；四通道极差均不超过128 counts/帧并连续满足3帧后，才进入8帧Hermite衔接。Hermite目标速度只取已趋稳定的最近3个差分中位数；输出同时限制在“上一帧轨迹速度外推值”和“当前Candidate”形成的包络内，禁止速度项产生软件鼓包。连续紧急扩档共用当前轨迹和24帧候选等待上限，中途再次写入Ref也不重置输出位置和速度。

Type `0x10`在`normalized_raw[]`之后新增`candidate_normalized_raw[]`。`normalized_raw[]`是经过速度预测和Hermite衔接后交给算法及上位机显示的最终值；`candidate_normalized_raw[]`是当前Hardware Raw直接经仿射参数换算、但未经切档过渡处理的候选值。两者同时记录可以区分“物理输入/仿射本身变化”和“软件过渡输出”。

### 3.9 V8 历史易失性全通道 Nsub 控制

当前`0x8C`写入返回Type `0x81`结果2（不支持），`0x8D`只读查询仍保留。以下档位设置规则仅供旧协议参考；现行三参数统一使用`0x8E`写入、`0x97/0x2C`逐通道核验，不写Flash。

CH0～CH3 分别对应 Button8、Button7、Button6、Button5，CH4～CH7 共用 MatrixButtons0。`0x8C` 会在扫描空闲点同时修改这五个 Widget，因此八个逻辑通道使用同一 Nsub。

设置命令 `0x8C` 的 payload 为一个小端 `uint16`，只接受 `100`、`200`、`400`、`800`。该设置只保存在 RAM 和运行时扫描寄存器中，不写 Flash，设备重新上电后恢复工程配置或原有持久化配置。

查询命令 `0x8D` 的 payload 长度为 `0`。设备返回 `0x26`：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0–1 | `requested_nsub` | `uint16` | 最近一次请求值；本次上电尚未设置时为 `0` |
| 2～`2+2N-1` | `actual_nsub[N]` | `uint16[N]` | 当前N个逻辑通道的实际回读值 |
| `2+2N` | `pending` | `uint8` | `1` 表示正在等待应用或首帧初始化 |
| `3+2N` | `result` | `uint8` | `0`成功，`1`非法档位，`2`写入失败，`3`回读失败，`4`当前拓扑不支持矩阵调参 |

`0x26` 的 `channel_count=N`，`payload_len=4+2N`；8通道为20字节，5通道为14字节。只有8通道矩阵变体支持相关设置操作；5通道独立Button变体仅返回只读/不支持状态。

## 4. 运行模式

运行模式只输出客户需要的最终结果。

每次完成传感器采样后，设备按以下顺序发送：

```text
Type 0x01 通道信号帧
Type 0x02 三维力帧
```

### 4.1 Type `0x01`：通道信号帧

帧头：

| 字段 | 值 |
| --- | --- |
| `type` | `0x01` |
| `device` | `1`、`2` 或 `3` |
| `channel_count` | `8` 或 `5` |
| `payload_len` | `channel_count × 2` |

payload 是连续的小端 `uint16` 数组：

```text
signal[0], signal[1], ..., signal[channel_count - 1]
```

`signal[i]` 表示第 `i` 个通道经过算法处理后的最终信号值。

### 4.2 Type `0x02`：三维力帧

三维力帧固定保留两个受力区槽位，每个槽位包含法向力、切向力大小和切向力方向。

帧头：

| 字段 | 值 |
| --- | --- |
| `type` | `0x02` |
| `device` | `1`、`2` 或 `3` |
| `channel_count` | `8` 或 `5` |
| `payload_len` | V19～V21为`40`；V18为`32`；V14～V17为`28` |

payload：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `vector_count` | `uint8` | 有效受力区数量 |
| 1 | `calibration_mask` | `uint8` | bit0：第一受力区已标定；bit1：第二受力区已标定 |
| 2–3 | `component_calibration_mask` | `uint16` | 每个受力区、每个分量独立的标定有效位，见下文 |
| 4–7 | `force1_normal` | `float32` | 第一受力区原始输出，模拟期间为8.88 g对应N值 |
| 8–11 | `force1_tangential` | `float32` | 第一受力区切向力大小 |
| 12–15 | `force1_direction` | `float32` | 第一受力区切向力方向，单位为弧度 |
| 16–19 | `force2_normal` | `float32` | 第二受力区法向力 |
| 20–23 | `force2_tangential` | `float32` | 第二受力区切向力大小 |
| 24–27 | `force2_direction` | `float32` | 第二受力区切向力方向，单位为弧度 |
| 28–31 | `force1_normal_optimized` | `float32` | V18新增：主法向优化输出，模拟期间为10.00～10.99 g对应N值 |
| 32–35 | `measured_normal_force` | `float32` | V19新增：检测线截断前的实际插值，单位N；保留超量程值 |
| 36–39 | `measured_optimized_normal_force` | `float32` | V19新增：正常检测线及软吸附结果，未被模拟替换，单位N |

不同设备的槽位含义：

| 设备版本 | `vector_count` | 第一受力区 | 第二受力区 |
| --- | ---: | --- | --- |
| 食指版本 | `2` | 主受力区/指尖 | 第二受力区/指腹 |
| 拇指/小指版本 | `1` | 主受力区 | 三个字段固定填 `0.0` |

`component_calibration_mask` 的 bit0～bit2 依次表示第一受力区的法向、切向和方向有效，bit3～bit5 依次表示第二受力区的法向、切向和方向有效。V17起bit6表示主法向低力模式、bit7表示超量程、bit8表示低力表有效。V19起bit9表示双输出为模拟值；其余位为0。低力模式bit0表示本帧输出有效，不足以单独判定为实测；未完成校准或超量程时bit0为0。五通道新增低力字段填0且无低力模式位。切向力算法未启用时，切向和方向位保持0。

主机使用法向力时应检查对应受力区的法向分量位。有效法向力单位为牛顿（N）。`calibration_mask` 保留原有的整区标志，只有同一区域的法向、切向和方向三张表全部有效时才置位。

V21食指在CapSense校准完成后，先检查有效10/20点高力表：Signal和/1000达到首个Raw节点且插值力≥1 N时，原始、优化、实际测量和实际优化四字段均输出该高力值，bit0=1并清除bit6/7/9；bit8仍独立表示微力表有效性。没有满足高力接管条件时，执行原微力路径。

微力路径要求四点表有效，将CH4～CH7 Signal和按四点表插值。正常原始值低于0.09 N时两路均为0，≥1 N且没有高力接管时输出占位0并置低力超量程标志。V19引入的最低检测线模拟是低于10 g时的显式例外，bit9置位时原始/优化字段被替换，实际测量仍在偏移32/36保留。旧48点表不会触发食指的新高力接管。`ACTIVE_FLOATING`不是运行换算或模拟输出的门槛；它是自动标定的采集条件。五通道原K表路径仍用Signal和/1000，首节点以下用虚拟(0,0)插值。

V16已停用此前的整数N软吸附；例如K表插值为4.30 N，Type `0x02`仍输出4.30 N。这样低力段0.1 N节点不会被整数聚拢曲线扭曲。

运行方向范围为 `[0, 2π)`，`+Fx逻辑=0`，从 `+Fx逻辑` 朝 `+Fy逻辑` 为正。

### 4.3 Type `0x03`：芯片内部温度帧

温度帧独立于输出模式，每累计约一秒的CapSense有效扫描触发一次非阻塞SAR注入转换；转换完成后排入UART发送队列。转换等待期间不会阻塞CapSense扫描或UART发送。

| 字段 | 值 |
| --- | --- |
| `type` | `0x03` |
| `device` | `1`、`2` 或 `3` |
| `channel_count` | `0` |
| `payload_len` | `2` |
| 帧总长度 | `12` 字节 |

payload只有一个小端有符号`int16`：`temperature_tenth_celsius`，单位为`0.1 °C`。例如`253`表示`25.3 °C`，`-55`表示`-5.5 °C`。协议沿用已经预留的V15 Type `0x03`格式，没有修改其他帧的版本、长度或发送频率。

## 5. 调试模式

调试模式输出当前内部开发所需的完整算法数据。

食指版本每次采样发送：

```text
Type 0x10  8 通道调试数据
Type 0x02  带标定有效位的法向/切向力
[可选] Type 0x11  主受力区详细力向量
[可选] Type 0x12  第二受力区详细力向量
```

拇指/小指版本每次采样发送：

```text
Type 0x10  5 通道调试数据
Type 0x02  带标定有效位的法向/切向力
[可选] Type 0x11  主受力区详细力向量
```

Type `0x11/0x12` 受 `SENSOR_ALGO_FORCE_VECTOR_OUTPUT` 编译开关控制，当前默认关闭；Type `0x02` 的四通道法向合力不受该开关影响。拇指/小指版本没有第二受力区，因此不发送 Type `0x12`。

### 5.1 Type `0x10`：通道调试帧

帧头：

| 字段 | 值 |
| --- | --- |
| `type` | `0x10` |
| `device` | `1`、`2` 或 `3` |
| `channel_count` | `8` 或 `5` |
| `payload_len` | V20/V21为`channel_count × 16`；8通道128字节，5通道80字节 |

设 `N = channel_count`，payload 按以下数组顺序组成：

```text
hardware_raw[0..N-1]
baseline[0..N-1]
signal[0..N-1]
fast_k_percent_x100[0..N-1]
slow_k_percent_x100[0..N-1]
state[0..N-1]
```

| 数组 | 说明 |
| --- | --- |
| `hardware_raw[]` | 小端 `uint16`，未经量程换算的硬件计数，用于饱和判断 |
| `baseline[]` | 当前基线值 |
| `signal[]` | 处理后的信号值 |
| `fast_k_percent_x100[]` | 小端 `int32`，快速归一化斜率 `%/s × 100` |
| `slow_k_percent_x100[]` | 小端 `int32`，慢速归一化斜率 `%/s × 100` |
| `state[]` | 通道状态 |

`state` 的取值：

| 值 | 状态 | 含义 |
| ---: | --- | --- |
| `0` | `IDLE` | 空闲 |
| `1` | `ACTIVE_RISING` | 压力正在上升 |
| `2` | `ACTIVE_HOLD` | 有效按压保持 |
| `3` | `ACTIVE_FALLING` | 压力正在下降但尚未确认松手 |
| `4` | `SLOW_BACK` | 已确认松手，Signal 强制为零并等待材料回弹 |
| `5` | `ZERO_REBASE` | SLOW_BACK 超时后受控重建零点 |
| `6` | `ACTIVE_FLOATING` | 稳定按压期间锁定 Signal，抑制材料蠕变 |
| `7` | `RECOVER` | Raw已稳定，baseline以约0.02系数快速恢复 |
| `8` | `WAIT_STABLE` | baseline冻结，等待Raw稳定或确认新的按压 |

全部状态由同一个通道状态机管理，baseline模块不再维护隐藏阶段。每帧先提交状态，再按该状态更新baseline，UART和快照直接读取真实状态；RECOVER进入帧即使用0.02，退出到IDLE的当帧使用0.005。

普通按压或SLOW_BACK结束后先进入WAIT_STABLE。正向Raw−baseline不超过12 counts，且连续80帧满足相邻Raw差绝对值≤2 counts时，误差大于1 count则进入RECOVER，否则直接回IDLE。任何超出稳定门槛的帧都会清零稳定计数。RECOVER内误差≤1 count后回IDLE。80帧在160 Hz下为0.5秒，在250 Hz下为0.32秒。

IDLE、WAIT_STABLE、RECOVER始终保留新按压检测：正向压力>12 counts时立即冻结baseline，Fast Raw斜率达到20 count/s并连续确认3帧后进入RISING；确认期间状态为WAIT_STABLE。ZERO_REBASE完成后直接回IDLE，不再重复稳定等待和恢复。

基线执行规则：IDLE约0.005；RECOVER约0.02；ZERO_REBASE约0.02，但单帧Raw正向变化>2 counts时暂停；WAIT_STABLE、所有按压状态、SLOW_BACK及未知状态均冻结。系数按采样帧应用；未完成校准、活动抑制和切档期间由原校准/切档流程管理baseline。

UART Debug沿用V16的uint16 state数组；V20精简其他字段后帧长度按本节16×N计算，快照/GetStates接口同步返回真实的7或8。Python统一状态名称和分类，CSV保留数值7、8；自动采样的空闲判定将两者纳入原IDLE语义，FLOATING判定不变。IDLE、WAIT_STABLE、RECOVER均将Signal清零，Slow K未计算；V20已删除独立的slow_baseline诊断。

V8 已从 Debug 帧删除 `raw_k_q16` 和 `signal_k_q16`；上位机也不再计算或显示 `Norm K`。这三个数据和 Equivalent Cs 当前均不参与本轮 Nsub 量程实验。

### 5.2 Type `0x11` 和 `0x12`：详细力向量帧

Type `0x11` 和 Type `0x12` 的 payload 格式相同：

| 字段 | 值 |
| --- | --- |
| `channel_count` | `8` 或 `5` |
| `payload_len` | `56` |
| payload | 14 个小端 `float32` |

payload 字段顺序：

| 顺序 | 字段 | 说明 |
| ---: | --- | --- |
| 1 | `theta` | 切向力方向角，单位为弧度 |
| 2 | `tangential_force` | MCU按Flash切向标定表换算后的切向力；有效标定时单位为N |
| 3 | `confidence` | 方向和力向量可信度 |
| 4 | `normal_force` | 四通道法向合力经Flash法向K表换算后的值；有效标定时单位为N |
| 5–6 | `cop_x`、`cop_y` | 归一化压心坐标 |
| 7–8 | `diff_x`、`diff_y` | 左右、上下压力差 |
| 9–10 | `force_x`、`force_y` | 按切向分段表等比例缩放后的切向分量，单位与切向标定表一致 |
| 11–14 | `p00`、`p01`、`p10`、`p11` | 2 × 2 区域输入值 |

调试模式会先发送 Type `0x02`。上位机应使用其中的 `calibration_mask`
判断标定是否有效，再把 Type `0x02` 或本详细帧中的力值作为牛顿值使用。

对应的受力区和输入通道：

| 设备版本 | `type` | 受力区 | `p00` | `p01` | `p10` | `p11` |
| --- | ---: | --- | --- | --- | --- | --- |
| 食指版本 | `0x11` | 主受力区/指尖 | CH6 | CH7 | CH4 | CH5 |
| 食指版本 | `0x12` | 第二受力区/指腹 | CH1 | CH0 | CH3 | CH2 |
| 拇指/小指版本 | `0x11` | 主受力区 | CH0 | CH3 | CH1 | CH4 |

## 6. 力数值标定

### 6.1 标定原则

方向标定采用上位机统一逻辑 XY 坐标：`+Fx逻辑=0°`，从 `+Fx逻辑` 朝 `+Fy逻辑` 为正。KWR75 原始 `Fx/Fy` 必须先转换到该逻辑坐标，再通过 `atan2(Fy逻辑,Fx逻辑)` 得到方向真值。手指固件的方向表直接映射到逻辑方向，不得重复应用 KWR75 的 XY 变换矩阵。

法向力、切向力和方向各自保存一张独立的分段线性标定表：

```text
normal_N     = piecewise_linear(normal_raw, normal_raw_points[], normal_force_points[])
tangential_N = piecewise_linear(tangential_raw, tangential_raw_points[], tangential_force_points[])
direction_deg = wrap360(piecewise_linear(direction_raw_deg, direction_raw_points[], direction_points[]))
```

其中：

- `normal_raw` 和 `tangential_raw` 是力算法输出的标定前数值。
- `normal_N` 和 `tangential_N` 是标定后的牛顿值。
- 三张表分别下发，更新其中一张不会覆盖另外两张。
- 设备运行帧输出的方向已经处于统一逻辑坐标，上位机不得再次叠加方向 sign/offset。
- 上位机负责采集标准载荷并生成节点表；设备负责插值、应用和保存参数。

### 6.2 受力区与力类型编号

| `region` | 物理受力区 |
| ---: | --- |
| `0` | 主受力区/指尖 |
| `1` | 第二受力区/指腹 |

| `force_type` | 标定表 |
| ---: | --- |
| `0` | 法向力分段标定表 |
| `1` | 切向力分段标定表 |
| `2` | 切向力方向标定表 |

`region` 在所有帧中始终表示物理受力区；`force_type` 只在参数读写帧中选择法向表或切向表。

### 6.3 Type `0x20`：力标定采样帧

设备处于力标定模式时，每次采样为每个有效受力区发送一帧 Type `0x20`。

发送顺序：

```text
食指 IMR：region 0 → region 1
```

帧头：

| 字段 | 值 |
| --- | --- |
| `type` | `0x20` |
| `device` | `1`、`2` 或 `3` |
| `channel_count` | `8` 或 `5` |
| `payload_len` | `24` |

payload：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `region` | `uint8` | 受力区编号 |
| 1 | `flags` | `uint8` | bit0：参数有效；bit1：参数尚未保存 |
| 2–3 | `reserved` | 2 字节 | 固定为 `0` |
| 4–7 | `normal_raw` | `float32` | 标定前法向力算法量 |
| 8–11 | `tangential_raw` | `float32` | 标定前切向力算法量 |
| 12–15 | `direction` | `float32` | 标定前切向力方向，单位为弧度；上位机换算为度后生成方向表 |
| 16–19 | `normal_calibrated` | `float32` | 按当前参数换算后的法向力，单位为 N |
| 20–23 | `tangential_calibrated` | `float32` | 按当前参数换算后的切向力，单位为 N |

该帧同时提供标定前和标定后的数值，因此临时写入参数后，可以在保存前直接检查换算效果。

### 6.4 力标定参数结构

Type `0x83` 每次读取一张表，Type `0x84` 每次只写入一张表。设当前表节点数为 `K`，payload 长度为 `8 + K × 8` 字节：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `region` | `uint8` | 物理受力区：`0` 指尖，`1` 指腹 |
| 1 | `force_type` | `uint8` | `0` 法向表，`1` 切向表，`2` 方向表 |
| 2 | `model` | `uint8` | 固定为 `2`，表示分段线性标定 |
| 3 | `point_count` | `uint8` | 法向容量48、切向容量20、方向固定2；V21下发region 0法向须10/20/48点，V16～V20须48点；无效默认表回读可少于这些数量 |
| 4 | `flags/reserved` | `uint8` | Type `0x21` 为 flags；Type `0x84` 固定为 `0` |
| 5–7 | `reserved` | 3 字节 | 固定为 `0` |
| `8 + 8i`～`11 + 8i` | `raw_points[i]` | `float32` | 当前表第 `i` 个原始量节点 |
| `12 + 8i`～`15 + 8i` | `calibrated_points[i]` | `float32` | 法向/切向为标准力节点（N）；方向为展开后的校正角节点（deg） |

参数帧中的 `flags`：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| bit0 | `valid` | 当前 `(region, force_type)` 标定表有效 |
| bit1 | `dirty` | 当前 `(region, force_type)` 标定表尚未保存到非易失存储 |

设备没有有效保存参数时，法向和切向表使用 `(0,0)`、`(1,1)` 单位映射，方向表使用 `(0°,0°)`、`(360°,360°)` 单位映射，同时保持 `valid = 0`。

非易失记录为 `FCV9`：region 0使用`0x1F600～0x1F8FF`，region 1使用`0x1F900～0x1FBFF`，每区580字节记录占三行768字节，分别预留48个法向节点、20个切向节点和2个方向节点。详见[Flash布局](FLASH_LAYOUT_CN.md)。旧FCV8/FCV7仅兼容读取；旧region 0主表标记无效，需重新完成当前10/20点标定或兼容的旧48点标定，其他分量保留原有效位。写region 1之前，如region 0仍依赖会被覆盖的旧地址，则先迁移region 0。

当前新主K表为1～30 N的10/20个整数目标点，节点力保存压力计实测值，不要求实测值恰好为整数。10/20点表首末实测压力须分别位于1 N/30 N的±10%内；旧1～20 N表不得充当新表。法向数组仍预留48节点，FCV9记录大小和其他字段偏移不变。

兼容的旧48点表为0.1～2 N每0.1 N、3～30 N每1 N；该旧路径使用虚拟原点并自V16停用整数N软吸附。不要将旧表的低力段与独立微力四点表混为同一记录。

当前监控页按10/20个整数目标重复若干轮，全部FLOATING后采样1秒，释放确认后按配置休息（默认30秒，可设0～3600秒），统计每轮实际耗时。可跳过当前剩余休息，仍核验卸载；此配置不改变UART协议。旧48档升降往返、格式4断点记录和19档快速修正仅为兼容功能，不与新采集混用。新结果自动同步K表页，绿色“读取最新标定数据并生成K值表”仍是旧48点入口；详见[当前实现](CURRENT_SYSTEM_CN.md)。

法向/切向表的 raw 节点必须非负且严格递增，输出节点必须非负且单调不减。方向表固定使用两个 raw 节点 `0°、360°`；输出节点为 `offset`、`offset + sign × 360°`，其中 `sign` 只能为 `+1` 或 `-1`。设备插值后环绕到 `[0°,360°)`，再转换为运行帧要求的弧度。

Type `0x21` 的 flags 按当前 `(region, force_type)` 独立返回；运行帧中的总体标定有效位只有在法向、切向和方向三张表都有效时才为 `1`。

### 6.5 底层协议操作参考

本节是分量表的协议操作参考，不是监控页自动标定点位或运动计划。当前1～30 N自动流程见[自动标定说明](HIGH_FORCE_CALIBRATION_CN.md)，不要求同时标定全部三个分量才能使用主法向值。

```text
1. 传感器完全卸载，发送 Type 0x80 重新校准
2. 发送 Type 0x82，mode = 2，进入力标定模式
3. 对目标受力区施加多组已知法向载荷，记录 normal_raw
4. 在稳定法向预载下施加多组已知切向载荷，记录 tangential_raw
5. 上位机根据联合切向标定数据拟合切向幅值表和方向 sign/offset，生成三张节点表
6. 对 region 0 和 region 1 的 force_type 0、1、2 分别执行 Type 0x84 写入，并立即用 Type 0x83/0x21 读回核对
7. 观察 Type 0x20 中的 calibrated 数值，验证误差
8. 验证通过后发送 Type 0x85，保存有修改的区域
9. 发送 Type 0x83 读回参数，确认 valid = 1、dirty = 0
10. 发送 Type 0x82，mode = 0，恢复运行模式
```

法向力和切向力应分别使用多个标定点。切向力标定时保持法向预载稳定，`direction` 只用于确认加载方向，不参与本协议定义的数值拟合。

当前仅校准region 0法向时，可独立暂存、保存该表；同区域切向/方向表无效不会阻止法向保存，其值和有效位原样保留。上位机保存前读回六张表，保存后复位再次读回核对。

## 7. 控制命令

控制命令同样使用 10 字节公共帧头。命令帧的 `channel_count` 固定为 `0`。

主机一次只发送一条命令。写入或动作命令以 Type `0x81` 作为完成确认；读取命令以对应的数据响应帧作为完成确认。

如果命令中的 `device` 不是 `0`，也不是设备自身类型，设备忽略该命令。

### 7.1 Type `0x82`：设置输出模式

帧头和 payload：

| 字段 | 值 |
| --- | --- |
| `type` | `0x82` |
| `channel_count` | `0` |
| `payload_len` | `1` |
| `payload[0]` | `0`：运行模式；`1`：调试模式；`2`：力标定模式 |

模式切换过程：

```text
设备完成当前采样帧组
        ↓
设备发送 Type 0x81 成功确认
        ↓
下一次采样开始使用新模式
```

这样可以保证主机不会收到由两种模式拼成的不完整帧组。

如果目标模式与当前模式相同，设备返回成功，输出模式保持不变。

### 7.2 Type `0x80`：重新校准

| 字段 | 值 |
| --- | --- |
| `type` | `0x80` |
| `channel_count` | `0` |
| `payload_len` | `0` |

设备收到命令后重新校准全部通道，并返回 Type `0x81` 确认帧。重新校准不改变当前输出模式。

### 7.3 Type `0x83`：读取力标定参数

| 字段 | 值 |
| --- | --- |
| `type` | `0x83` |
| `channel_count` | `0` |
| `payload_len` | `2` |
| `payload[0]` | 要读取的物理 `region`：`0` 指尖，`1` 指腹 |
| `payload[1]` | `force_type`：`0` 法向，`1` 切向，`2` 方向 |

设备成功处理后返回所请求 `(region, force_type)` 的 Type `0x21` 参数帧，payload 为第 6.4 节定义的 `8 + K × 8` 字节结构。

参数区无效时，设备改为返回非零结果的 Type `0x81` 确认帧。

### 7.4 Type `0x84`：临时设置力标定参数

| 字段 | 值 |
| --- | --- |
| `type` | `0x84` |
| `channel_count` | `0` |
| `payload_len` | `8 + point_count × 8` |
| payload | 第 6.4 节定义的参数结构 |

设备将参数写入 RAM，校验通过后立即用于力值换算，并返回 Type `0x81` 确认帧。此时参数的 `dirty` 状态为 `1`。

V21的region 0、force_type 0主K表允许10/20/48点；payload分别为88/168/392字节，含帧头共98/178/402字节。V16～V20主表仍要求48点。其他法向表允许2～48点，切向2～20点，方向固定2点，容量独立检查。新短表还须满足第6.4节首末实测压力范围；不会因点数减少而改变Flash地址或记录偏移。

该命令不写非易失存储。设备重新上电后，未保存的参数会丢失。

### 7.5 Type `0x85`：保存力标定参数

| 字段 | 值 |
| --- | --- |
| `type` | `0x85` |
| `channel_count` | `0` |
| `payload_len` | `0` |

设备按region只保存dirty区域；每区三行先擦后写，并逐字节回读完整768字节。通常只修改region 0法向便只写0x1F600、0x1F700、0x1F800。旧地址重叠迁移是额外写入region 0的唯一例外。全部写入和校验成功后返回Type `0x81`成功并清除dirty；失败返回4且保留RAM修改供重试。

这是固定地址记录，没有文件系统或跨行原子提交。断电写坏记录会被校验拒绝，但没有旧表Flash双副本可自动恢复；DFU主记录及备份始终不在应用参数写入范围。

保存后的参数在后续上电时自动加载，并用于运行模式的 Type `0x02` 三维力输出。

### 7.6 Type `0x86`：放弃临时力标定参数

| 字段 | 值 |
| --- | --- |
| `type` | `0x86` |
| `channel_count` | `0` |
| `payload_len` | `0` |

设备放弃全部尚未保存的修改，重新加载非易失存储中的力标定参数，并返回 Type `0x81` 确认帧。

### 7.7 Type `0x21`：力标定参数帧

| 字段 | 值 |
| --- | --- |
| `type` | `0x21` |
| `device` | 设备自身类型 |
| `channel_count` | `0` |
| `payload_len` | `8 + point_count × 8` |
| payload | 第 6.4 节定义的参数结构 |

### 7.8 Type `0x81`：命令确认帧

| 字段 | 值 |
| --- | --- |
| `type` | `0x81` |
| `device` | 设备自身类型 |
| `channel_count` | `0` |
| `payload_len` | `1` |

`payload[0]` 为执行结果：

| `result` | 含义 |
| ---: | --- |
| `0` | 成功 |
| `1` | payload 格式或取值错误 |
| `2` | 不支持的命令 |
| `3` | 不支持的受力区或标定模型 |
| `4` | 非易失存储读写失败 |

确认帧对应主机最近发送并被设备接受的命令。

### 7.9 Type `0x87`：旧MatrixButtons0 Nsub设置（已停用）

当前固件直接返回Type `0x81`结果2（不支持）；以下载荷和执行描述为历史参考。现行统一三参数写入使用`0x8E`。

| 字段 | 值 |
| --- | --- |
| `type` | `0x87` |
| `channel_count` | `0` |
| `payload_len` | `2` |
| `payload[0..1]` | `uint16` 小端 Nsub，合法范围 `1..16383` |

设备只将合法值保存为待处理参数，并返回 Type `0x81`。当前 CapSense 扫描结束后，主循环在下一轮扫描开始前调用 `Cy_CapSense_SetParam()`，随后调用 `Cy_CapSense_GetParam()` 回读，并同步 MatrixButtons0 四个扫描 slot 的 Nsub 字段。非法值返回结果 `1`，不修改当前参数。

### 7.10 Type `0x88` / `0x22`：读取 MatrixButtons0 Nsub

读取命令 Type `0x88` 的 payload 长度为 `0`。设备返回 Type `0x22`，payload 长度为 `12`：

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | `uint16` | `requested_nsub`，最近一次请求值 |
| 2 | `uint16` | `actual_nsub`，`Cy_CapSense_GetParam()` 最近回读值 |
| 4 | `uint32` | `set_status`，`Cy_CapSense_SetParam()` 状态 |
| 8 | `uint32` | `get_status`，`Cy_CapSense_GetParam()` 状态 |

全部多字节字段均为小端。`set_status = 0` 且 `get_status = 0` 表示设置和回读成功。

### 7.11 Type `0x89` / `0x23`：读取 MatrixButtons0 CapSense 配置

Type `0x89` 的 payload 长度为 `0`。设备返回 Type `0x23`，payload 长度为 `54`：

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | `uint16` | Measurement Nsub，`GetParam()` RAM 回读 |
| 2 | `uint16` | 当前 sensor-frame 中将装载到 MSCLP 的 Nsub |
| 4 | `uint16` | Init Nsub，`ptrInternalContext->numFineInitCycles` |
| 6 | `uint8` | Cref，`GetParam()` RAM 回读 |
| 7 | `uint8` | 当前 sensor-frame 中将装载到 MSCLP 的 Cref |
| 8 | `uint8[4]` | 四个 sensor 的 Ccomp RAM 回读 |
| 12 | `uint8[4]` | 四个 sensor-frame 中将装载到 MSCLP 的 Ccomp |
| 16 | `uint16` | Sense/Tx clock divider RAM 回读 |
| 18 | `uint16` | 当前 sensor-frame 中将装载到 MSCLP 的 divider |
| 20 | `uint16` | MaxCount |
| 22 | `uint16` | MatrixButtons0 sensor 0 对应全局 CH4 RAW |
| 24 | `uint16` | CH4 baseline |
| 26 | `uint16` | CH4 signal |
| 28 | `uint8` | SmartSense enabled |
| 29 | `uint8` | CSX auto calibration enabled |
| 30 | `uint8` | CapSense busy 状态 |
| 31 | `uint8` | 保留，固定 `0` |
| 32 | `uint32` | 最近一轮完整 CapSense scan time，单位 us |
| 36 | `uint32` | 根据 scan time 计算的 FPS × 1000 |
| 40 | `uint32` | 所有 `GetParam()` 状态的按位或结果 |
| 44 | `uint32` | 最近一次 CapSense 配置 Flash 写入状态，`0` 为成功 |
| 48 | `uint8` | 当前参数是否已从/已写入有效持久化记录 |
| 49 | `uint32` | 最近一次按上电流程执行全部 filter/baseline 初始化的状态 |
| 53 | `uint8` | 是否等待新参数首轮扫描完成后更新 baseline |

偏移44/48仅为兼容旧诊断结构；现行运行参数不加载或保存CSRT记录，`persistent_config_loaded=0`。不能凭偏移44的0值判断三参数已经写入Flash。当前逐通道参数核验优先使用`0x97/0x2C`。

RAM/frame 两组数值用于确认 SetParam 回读值是否已经同步到下一轮扫描使用的 sensor frame。参数更新后固件会清除 repeat-scan 和 WD-scan 状态，强制下一轮 `ScanAllSlots()` 重新装载 sensor frame。

### 7.12 Type `0x8A`：旧MatrixButtons0 Cref设置（已停用）

当前固件直接返回Type `0x81`结果2（不支持）；以下单矩阵写入规则仅供旧协议参考。现行`0x8E`同步全部逻辑通道的三参数，不能恢复本节旧写入路径。

| 字段 | 值 |
| --- | --- |
| `type` | `0x8A` |
| `channel_count` | `0` |
| `payload_len` | `1` |
| `payload[0]` | Cref，合法范围 `1..255` |

命令仅保存 pending 值。设备在 CapSense `NOT_BUSY` 且下一轮扫描启动前执行 `Cy_CapSense_SetParam()`、`Cy_CapSense_GetParam()`，并只同步 MatrixButtons0 四个 slot 的 `SEL_RE` 字段，不调用 `Cy_CapSense_Enable()`。

### 7.13 Type `0x8B` / `0x24`：读取 MatrixButtons0 Cref

Type `0x8B` 的 payload 长度为 `0`。Type `0x24` 的 payload 长度为 `10`：

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | `uint8` | `requested_cref` |
| 1 | `uint8` | `actual_cref` |
| 2 | `uint32` | `set_status` |
| 6 | `uint32` | `get_status` |

### 7.14 Type `0x25`：已停用

Equivalent Cs 绝对电容帧从 V8 起停止发送。帧号 `0x25` 保留，不得复用；如后续重新启用，应再次升级协议版本。

## 8. 命令示例

以下示例均以已确认为V21的食指设备`device=1`为目标，版本字节`15`是十六进制（十进制21）。连接其他兼容版本时使用其实际版本，不能照抄字节。

### 8.1 切换到运行模式

```text
44 41 54 41  15 82 01 00  01 00  00
 D  A  T  A  ver type dev  N   len   mode
```

### 8.2 切换到调试模式

```text
44 41 54 41  15 82 01 00  01 00  01
 D  A  T  A  ver type dev  N   len   mode
```

### 8.3 进入力标定模式

```text
44 41 54 41  15 82 01 00  01 00  02
 D  A  T  A  ver type dev  N   len   mode
```

### 8.4 读取第一受力区方向标定参数

```text
44 41 54 41  15 83 01 00  02 00  00 02
 D  A  T  A  ver type dev  N   len   region force_type
```

### 8.5 重新校准

```text
44 41 54 41  15 80 01 00  00 00
 D  A  T  A  ver type dev  N   len
```

### 8.6 成功确认

```text
44 41 54 41  15 81 01 00  01 00  00
 D  A  T  A  ver type dev  N   len   result
```

## 9. 输出模式与底层标定命令流程

下图说明设备输出模式切换。当前监控页自动压力标定在Debug模式采集，流程见[自动标定](HIGH_FORCE_CALIBRATION_CN.md)；它不等同于下图底层Force Calibration模式示例。

```text
设备上电
  ↓
默认进入运行模式
  ↓
每次采样发送 Type 0x01 + Type 0x02；每秒另发 Type 0x03
  ↓
主机是否要求调试？
  ├─ 普通运行：继续发送 Type 0x01 + Type 0x02；每秒另发 Type 0x03
  ├─ 内部调试：发送 Type 0x82，mode = 1
  │              ↓
  │            后续采样发送 Type 0x10 + Type 0x02 [+ Type 0x11/0x12]；每秒另发 Type 0x03
  └─ 力值标定：发送 Type 0x82，mode = 2
                 ↓
               后续采样发送 Type 0x20；每秒另发 Type 0x03
                 ↓
               临时写入、验证并保存标定参数
                 ↓
               发送 Type 0x82，mode = 0，恢复运行模式
```

## 附录 A：帧长度

所有帧的总长度均为 `10 + payload_len`。

| 帧 | 食指版本 | 拇指/小指版本 |
| --- | ---: | ---: |
| Type `0x01` | `26` 字节 | `20` 字节 |
| Type `0x02` | V19～V21为`50`字节，V18为`42`字节，更早`38`字节 | V19～V21为`50`字节，V18为`42`字节，更早`38`字节 |
| Type `0x03` | `12` 字节/秒 | `12` 字节/秒 |
| Type `0x10` | `218` 字节 | `140` 字节 |
| Type `0x11` | `66` 字节 | `66` 字节 |
| Type `0x12` | `66` 字节 | 不发送 |
| Type `0x20` | `34` 字节/受力区 | `34` 字节/受力区 |
| Type `0x21` | `30` 字节 | `30` 字节 |
| 一组运行模式数据 | `64` 字节 | `58` 字节 |
| 一组调试模式数据 | `350` 字节 | `206` 字节 |
| 一组力标定模式数据 | `68` 字节 | `34` 字节 |

| 命令 | 总长度 |
| --- | ---: |
| Type `0x80` 重新校准 | `10` 字节 |
| Type `0x82` 设置模式 | `11` 字节 |
| Type `0x83` 读取标定参数 | `11` 字节 |
| Type `0x84` 临时设置标定参数 | `30` 字节 |
| Type `0x85` 保存标定参数 | `10` 字节 |
| Type `0x86` 放弃临时参数 | `10` 字节 |
| Type `0x87` 设置 MatrixButtons0 Nsub | `12` 字节 |
| Type `0x88` 读取 MatrixButtons0 Nsub | `10` 字节 |
| Type `0x22` MatrixButtons0 Nsub 状态 | `22` 字节 |
| Type `0x89` 读取 MatrixButtons0 配置 | `10` 字节 |
| Type `0x23` MatrixButtons0 配置状态 | `64` 字节 |
| Type `0x8A` 设置 MatrixButtons0 Cref | `11` 字节 |
| Type `0x8B` 读取 MatrixButtons0 Cref | `10` 字节 |
| Type `0x24` MatrixButtons0 Cref 状态 | `20` 字节 |
| Type `0x8C` 易失性设置全通道 Nsub | `12` 字节 |
| Type `0x8D` 读取全通道 Nsub | `10` 字节 |
| Type `0x26` 全通道 Nsub 状态 | `14 + 2 × channel_count`字节；8通道30字节，5通道24字节 |
| Type `0x28` 自动量程状态 | `30` 字节 |
| Type `0x29` 仿射参数回读 | `82` 字节 |
| Type `0x2A` 仿射参数操作结果 | `18` 字节 |
| Type `0x90` 查询自动量程状态 | `10` 字节 |
| Type `0x91` 控制Ref手动校准模式 | `12` 字节 |
| Type `0x92` 暂存单通道仿射参数 | `74` 字节 |
| Type `0x93` 保存28组仿射参数 | `14` 字节 |
| Type `0x94` 查询单通道仿射参数 | `11` 字节 |
| Type `0x95` 软件复位 | `10` 字节 |
| Type `0x96` 查询设备身份 | `10` 字节 |
| Type `0x2B` 设备身份信息 | `14 + name_length`字节；INDEX_FINGER为26字节，THUMB/PINKY为19字节 |
| Type `0x81` 确认帧 | `11` 字节 |

## 附录 B：与现有 v2 实现的对应关系

| 现有 v2 | 统一 v3 | 说明 |
| --- | --- | --- |
| 帧头偏移 6 为 `channel_count` | 帧头偏移 6 为 `device` | 新增设备类型 |
| 帧头偏移 7 为 `reserved` | 帧头偏移 7 为 `channel_count` | 使用原保留字节 |
| Type `1` Signal | Type `0x01` | payload 不变 |
| Type `2` Debug | Type `0x10` | v13在最终Norm Raw后增加Candidate Norm Raw |
| Type `3` 主受力区力向量 | Type `0x11` | payload 不变 |
| Type `4` 第二受力区力向量 | Type `0x12` | payload 不变 |
| 无精简三维力帧 | Type `0x02` | 从现有力向量结果组合生成 |
| 编译期开关选择输出 | Type `0x82` 运行时切换 | 一套固件支持两种模式 |
| ASCII `RECAL`，无确认 | Type `0x80` + Type `0x81` | v3 使用统一命令帧 |

v2 和 v3 中的 Type `0x02` 含义不同。接收端必须先检查 `version`，再解释帧头和 `type`。

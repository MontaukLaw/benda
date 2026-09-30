# 状态机、Ref CDAC 动态量程与仿射校准移植说明

> 历史资料：本文保留旧版本协议与实验背景，不是当前操作规范。现行UART为V21，沿用V20 Raw布局；统一入口见[当前实现](CURRENT_SYSTEM_CN.md)，命令见[现行协议](UART_PROTOCOL_UNIFIED_V3_CN.md)，地址见[Flash布局](FLASH_LAYOUT_CN.md)。旧帧编号、仿射流程以及0x1FE00/0x1FF00写入地址不得用于现行固件。

## 1. 文档目的与基准

本文用于把当前工程中的以下能力完整移植到另一份固件或另一块同类硬件：

1. 单通道 7 状态精确判定；
2. Baseline 冻结、恢复和零点重建；
3. CH4～CH7 固定采样参数下的 Ref CDAC 七档自动量程；
4. 每通道、每档位的 28 组仿射参数采集、拟合、下发和 Flash 持久化；
5. 切档时的 Candidate Norm Raw 稳定检查、速度预测和 Hermite 平滑衔接；
6. UART V14 协议、设备身份识别及 Python 上位机中的校准、回读和连续性测试。

本文保存 **2026-09-04、V14时期** 的源码说明；正文中的“当前”仅指该历史基准，不代表现行V21。现行移植应先查[当前实现](CURRENT_SYSTEM_CN.md)、[状态机](MACHINE_STATE.md)及[Flash布局](FLASH_LAYOUT_CN.md)，不得恢复已经停用的自动量程或旧参数写入路径。

本文只整理与状态机、CapSense 参数、动态量程、仿射校准、协议和上位机验证直接相关的提交；电机界面配色、普通 DFU、三维力算法等无关提交不属于本功能移植范围。

## 2. 首先统一几个概念

### 2.1 状态数量与压力点数量无关

- 每个传感器通道独立运行一套状态机，固定有 7 个状态：`IDLE`、`RISING`、`ACTIVE/HOLD`、`FALLING`、`SLOW_BACK`、`ZERO_REBASE`、`FLOATING`。
- P0、30 g、40 g、500 g、1 kg、3 kg 等是仿射校准的“压力点”。压力点数量可以增加或删除，但不会增加状态机状态。
- 压力点的重量数值当前不参与拟合公式；它只用于人工辨认。拟合真正需要的是：在同一个物理压力不变时，七个 Ref 档位各自测得的 CH4～CH7 Hardware Raw。

### 2.2 档位方向

本文统一使用下面的说法，避免“高档”和“高灵敏度”混淆：

- **扩档/升档**：Ref CDAC 增大，例如 `12 → 18`。灵敏度下降，可测电容量程增大，Hardware Raw 降低。
- **回落/降档**：Ref CDAC 减小，例如 `60 → 40`。灵敏度提高，可测电容量程减小，Hardware Raw 升高。
- Ref=12 是第 1 档，灵敏度最高、量程最小；Ref=135 是第 7 档，灵敏度最低、量程最大。

### 2.3 四种 Raw 的含义

| 名称 | 位宽 | 含义 | 主要用途 |
| --- | ---: | --- | --- |
| Hardware Raw | `uint16` | CapSense 当前 Ref 下的原始硬件计数 | 饱和判断、自动量程、仿射采集 |
| Candidate Norm Raw | `uint32` | Hardware Raw 经当前档位仿射换算后的候选值，尚未经过切档过渡 | 判断仿射/硬件是否连续，诊断切档暂态 |
| Final Norm Raw | `uint32` | Candidate 再经过速度预测和 Hermite 衔接后的最终值 | 上位机显示、CSV、算法输入源 |
| Algorithm Raw | `uint16` | Final Norm Raw 限幅到 16 位后的值 | 现有状态机、Baseline、Signal 算法 |

因此，`Norm Raw` 是统一到 Ref=60 计量域的“等效计数”，不是未经进一步物理标定就能直接解释为 pF 的绝对电容值。若要得到 pF，还需要单独建立 Norm Raw 到实际电容的物理标定关系。

## 3. 功能演进与对应提交

| 顺序 | 提交 | 主要作用 | 移植时保留的最终结论 |
| ---: | --- | --- | --- |
| 1 | `408d28b` | 状态机重构前备份 | 仅作为历史回退点，不移植旧判定 |
| 2 | `770fd93` | 拆分 `channel_state_machine`、`baseline_tracker`、单通道处理并扩展上位机 | 建立独立模块和唯一状态提交入口 |
| 3 | `c37c4b6` | 精确限制 `IDLE → RISING`，完善 Fast/Slow K | IDLE 激活同时要求压力和正向 Raw 斜率 |
| 4 | `24ed8b1` | 增加 `SLOW_BACK` 超时与 `ZERO_REBASE` | 材料慢回弹不会永久锁死状态 |
| 5 | `256723e` | 增加 `ACTIVE_FLOATING` 蠕变锁定及退出迟滞 | 固定压力下锁定 Signal，吸收材料缓慢蠕变 |
| 6 | `06fba7b` | UART V8、全通道 Nsub 在线控制和 K 字段精简 | 为后续固定采样参数与 Raw 诊断建立协议基础 |
| 7 | `a97defa` | 同步 CH4～CH7 运行时配置与噪声统计 | 确认实际参数回读和上位机监测路径 |
| 8 | `7a22a25` | 修复在线参数切换后的 CIC2 重新初始化，约束 Ref 范围 | 明确 Nsub、TxDiv、Ref 与 CIC2 Max Count 的关系 |
| 9 | `2bdd4c3` | 增加 CIC2 Max Count 预览及测试参数化 | 固定参数前完成量程/帧率取舍 |
| 10 | `4b335ac` | V9/V10 动态量程、手动七档校准、项目化采集与质量分析 | 固定 7 档、自动升降档、压力点采集和 28 组拟合 |
| 11 | `47ccaed` | V11 参数暂存、Flash 保存、复位和回读验证 | 28 组参数以完整事务持久化 |
| 12 | `37ef198` | 忽略临时文件和校准导出 | `.arcal.json` 是工作项目；CSV 只是快照，不纳入版本库 |
| 13 | `6b35354` | V12 应用仿射参数并增加双向档位连续性测试 | Norm Raw 实际使用 28 组参数，增加连续性报告 |
| 14 | 当前工作区 | V13 Candidate Raw与切档连续性；V14多手指身份、可变通道数和上位机合力 | 本文描述的最终切档、状态和设备变体策略 |

## 4. 总体数据流和模块边界

```text
CapSense 扫描完成
    │
    ├─ 读取 CH0～CH7 Hardware Raw
    │
    ├─ capsense_autorange
    │    ├─ CH4～CH7 最大 Hardware Raw → 自动量程判断
    │    ├─ 仿射参数/理论比例 → Candidate Norm Raw
    │    ├─ 速度预测 + Hermite → Final Norm Raw
    │    └─ 16 位限幅 → Algorithm Raw
    │
    ├─ sensor_algorithm / process_channel_sample
    │    ├─ channel_state_machine → 7 状态、Fast K、Slow K
    │    ├─ baseline_tracker → Baseline 冻结/恢复/重建
    │    └─ Signal 与力算法
    │
    └─ UART V14
         ├─ Hardware Raw
         ├─ Final Norm Raw
         ├─ Candidate Norm Raw
         ├─ 状态、Baseline、Signal、Fast K、Slow K、Noise
         └─ 自动量程/仿射参数状态
```

主要文件职责如下：

| 文件 | 职责 |
| --- | --- |
| `channel_state_machine.c/.h` | 单通道 Raw 窗口、K 值、7 状态分类、消抖和唯一提交入口 |
| `baseline_tracker.c/.h` | Baseline 的 IDLE 跟随、活动冻结、稳定等待、恢复和零点重建 |
| `process_channel_sample.c/.h` | 单通道每帧处理顺序和算法操作接口 |
| `sensor_algorithm.c/.h` | 状态机接入、Signal/FLOATING 处理、协议命令与数据帧 |
| `capsense_autorange.c/.h` | 七档自动量程、候选归一化和切档平滑 |
| `capsense_affine_calibration.c/.h` | 28 组参数校验、Flash 记录和定点仿射换算 |
| `capsense_nsub_control.c/.h` | CapSense 运行时参数写入、实际值回读和扫描空闲点更新 |
| `main.c` | 读取 Hardware Raw、汇总 CH4～CH7 状态、串联自动量程与算法 |
| `UART_PROTOCOL_UNIFIED_V3_CN.md` | 当前 UART V14 字段与设备身份定义 |
| `PY_SRIPT/algo_monitor/autorange_calibration_project_widget.py` | 项目管理、自动压力点采集、Flash 下发和回读验证 |
| `PY_SRIPT/algo_monitor/autorange_calibration_widget.py` | 仿射拟合和质量分析公共逻辑 |
| `PY_SRIPT/algo_monitor/autorange_continuity_test_widget.py` | 双向切档连续性自动测试和 CSV 报告 |
| `PY_SRIPT/algo_monitor/frame_parser.py` | V14 数据、设备身份、自动量程及仿射状态解析 |
| `PY_SRIPT/algo_monitor/main_window.py` | 实时状态、Raw、档位、校准与“未计算”提示 |

## 5. 固定 CapSense 采样参数

### 5.1 当前固定值

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| Nsub | `300` | CH4～CH7 共用 MatrixButtons0 固定值 |
| Tx Clock Divider | `100` | 固定值，不再由上位机普通运行时下发 |
| Max Count | `58928` | CIC2 理论最大计数 |
| 状态机基准帧率 | `160 Hz` | 所有“帧数→时间”换算基准 |
| 上电 Ref | `12` | 从最高灵敏度档起步，由紧急扩档保护饱和 |
| Norm 参考档 | `60` | 所有档位最终换算到 Ref=60 计量域 |

### 5.2 Max Count 推导

按当前 CIC2 规则：

```text
Nsub × TxDiv = 300 × 100 = 30000
DecimationRate = min(255, floor(30000 / 3)) = 255
CIC2 Samples = floor(30000 / 255) - 1 = 116
Hardware Divider = 128
Max Count = floor(255² × 116 / 128) = 58928
```

`58928` 是 Nsub、TxDiv 和 CIC2 配置共同决定的计数上限，不随 Ref 档位变化。Ref 只改变转换增益：Ref 越小 Raw 越大、灵敏度越高；Ref 越大 Raw 越小、量程越大。

### 5.3 为什么固定 Nsub 和 TxDiv

- 避免切量程时同时改变积分时间、帧率、噪声和 CIC2 Max Count。
- 让所有状态机帧数阈值具有稳定的时间含义。
- 量程只由 Ref CDAC 控制，切档问题可以被拆成“硬件暂态、档间增益误差、输出连续性”三个相对独立的问题。
- Ref 变化时只更新 Ref 及相关 sensor-frame，不重复初始化固定的 CIC2 Rate、Shift 和 MaxRaw。

如果移植后的真实算法帧率不再约为 160 Hz，所有时间型帧数必须按比例缩放，不能只改 `CHANNEL_STATE_SAMPLE_RATE_HZ` 而保留原帧数。

## 6. 七档 Ref CDAC 量程

| 档位索引 | 人工编号 | Ref CDAC | 相对 Ref=12 的近似量程倍率 | 特性 |
| ---: | ---: | ---: | ---: | --- |
| 0 | 1 | 12 | 1.00× | 最高灵敏度、最小量程、上电起始档 |
| 1 | 2 | 18 | 1.50× | 低压过渡档 |
| 2 | 3 | 27 | 2.25× | 低中压过渡档 |
| 3 | 4 | 40 | 3.33× | 中间档 |
| 4 | 5 | 60 | 5.00× | Norm Raw 参考档，单位映射 |
| 5 | 6 | 90 | 7.50× | 高压档 |
| 6 | 7 | 135 | 11.25× | 最大实测压力所需的最高量程档 |

档位采用约 1.5 倍级差，在档位数量、重叠范围、校准工作量和突发压力保护之间折中。Ref=135 来自最大压力实测，而不是使用 8 位 Ref 的硬件极限 255。

## 7. 单通道 7 状态机

### 7.1 状态定义

| 值 | 固件枚举 | 上位机名称 | Signal 行为 | Baseline 行为 | 视为有效接触 | 允许矩阵降档 |
| ---: | --- | --- | --- | --- | --- | --- |
| 0 | `SENSOR_ALGO_STATE_IDLE` | IDLE | 空闲输出 | 小系数跟随或恢复 | 否 | 是 |
| 1 | `SENSOR_ALGO_STATE_ACTIVE_RISING` | RISING | 正常计算 | 冻结 | 是 | 否 |
| 2 | `SENSOR_ALGO_STATE_ACTIVE_HOLD` | ACTIVE | 正常计算 | 冻结 | 是 | 否 |
| 3 | `SENSOR_ALGO_STATE_ACTIVE_FALLING` | FALLING | 正常计算 | 冻结 | 是 | 否 |
| 4 | `SENSOR_ALGO_STATE_SLOW_BACK` | SLOW BACK | 强制为 0 | 冻结 | 否 | 是 |
| 5 | `SENSOR_ALGO_STATE_ZERO_REBASE` | ZERO REBASE | 强制为 0 | 受控快速跟随 Raw | 否 | 否 |
| 6 | `SENSOR_ALGO_STATE_ACTIVE_FLOATING` | FLOATING | 锁定并补偿蠕变 | 冻结 | 是 | 否 |

矩阵降档要求 CH4～CH7 **每一路分别** 为 `IDLE` 或 `SLOW_BACK`。允许 `IDLE + SLOW_BACK` 的任意组合，例如 `IDLE/IDLE/SLOW_BACK/IDLE` 可以降档。`ZERO_REBASE` 不在允许集合中。

### 7.2 压力、峰值和动态阈值

```text
Pressure = max(Raw - ActivationBaseline, 0)
PressureOn = max(Noise × 3, 12 counts)
PressureOff = max(Noise × 2, 10 counts)
NormalizationFloor = max(Noise × 10, 20 counts)
NormalizationReference = max(PeakPressure, NormalizationFloor)
```

- `ActivationBaseline` 在 IDLE 中跟随当前 Baseline；离开 IDLE 后冻结为本次活动起点。
- `PeakPressure` 是本轮活动期间最大的 Pressure。
- `PressureOn > PressureOff` 构成压力迟滞，避免阈值附近摆动。
- K 值分母至少为 `max(Noise×10,20)`，防止低压力下百分比斜率被过度放大。

### 7.3 Fast/Slow 窗口和 K 值

| 项目 | 当前值 | 160 Hz 下时间 |
| --- | ---: | ---: |
| Fast Raw 窗口 | 16 帧 | 100 ms |
| Slow Raw 窗口 | 160 帧 | 1 s |
| K 遥测刷新 | 每 8 帧 | 50 ms |

窗口使用线性回归斜率：

```text
SlopePerSample = (N×Σxy - Σx×Σy) / (N×Σx² - (Σx)²)
RawSlopePerSecond = SlopePerSample × 160
NormalizedK(%/s) = RawSlopePerSecond / NormalizationReference × 100
K_x100 = NormalizedK(%/s) × 100
```

固定回归参数：

| 窗口 | N | Σx | 分母 `N×Σx²-(Σx)²` |
| --- | ---: | ---: | ---: |
| Fast | 16 | 120 | 5440 |
| Slow | 160 | 12720 | 54611200 |

通信中的 `2000` 表示 `+20.00 %/s`，`-25000` 表示 `-250.00 %/s`。

### 7.4 状态阈值总表

| 条件 | 当前值 |
| --- | ---: |
| 普通状态候选确认 | 3 帧，约 18.75 ms |
| 活动态返回 IDLE 确认 | 16 帧，约 100 ms |
| IDLE 激活 Fast Raw 斜率 | `>= +20 count/s` |
| 活动态进入 RISING | Fast K `>= +20 %/s` |
| 活动态进入 FALLING | Fast K `<= -30 %/s` 或 Slow K `<= -8 %/s` |
| Fast 稳定 | `abs(Fast K) <= 15 %/s` |
| Slow 稳定 | `abs(Slow K) <= 5 %/s` |
| FLOATING 专用 Slow 稳定 | `abs(Slow K) <= 10 %/s` |
| 进入 FLOATING 前最短活动时间 | 160 帧，约 1 s |
| FLOATING → RISING | Fast K `>= +80 %/s` 连续 8 帧 |
| FLOATING → FALLING | Fast K `<= -100 %/s` 连续 8 帧 |
| 快速松手斜率 | Fast K `<= -250 %/s` |
| 快速松手峰值跌幅 | `>= 15%` |
| 接近释放低位 | Pressure `<= PeakPressure × 12%` |
| SLOW_BACK 重新按压斜率 | Fast Raw `>= +80 count/s` |
| SLOW_BACK 稳定 Raw 斜率 | `abs(Fast Raw slope) <= 20 count/s` |
| SLOW_BACK → IDLE 确认 | 64 帧，约 400 ms |
| SLOW_BACK 超时 | 4800 帧，约 30 s |

### 7.5 状态判定优先级

每帧不是并行随意判断，而是按以下优先级执行：

1. 未完成校准或 `suppress_activity=true`：强制进入 `IDLE`。
2. 当前为 `IDLE`：只检查 `IDLE → RISING`。
3. 当前为 `SLOW_BACK`：依次检查重新按压、自然恢复到 IDLE、30 秒超时进入 ZERO_REBASE。
4. 当前为 `ZERO_REBASE`：依次检查重新按压、Baseline 重建完成进入 IDLE。
5. 其余活动状态：快速松手优先级最高；之后检查返回 IDLE；再处理 FLOATING 专用退出；最后依次判断 RISING、FALLING、FLOATING、HOLD。

这种顺序很重要。例如 Fast K 同时满足普通 FALLING 和快速松手条件时，必须直接进入 `SLOW_BACK`，不能先进入 `FALLING`。

### 7.6 具体切换条件

#### IDLE → RISING

```text
Pressure > PressureOn
且 FastRawSlope >= +20 count/s
且连续 3 帧
```

即使 Raw 已高于压力阈值，只要没有明确上升趋势，也不会直接进入 HOLD。

#### 活动态 → SLOW_BACK

适用于 RISING、HOLD、FALLING 和 FLOATING：

```text
PeakPressure > 0
且 FastK <= -250 %/s
且 (PeakPressure - Pressure) / PeakPressure >= 15%
```

满足后立即切换，不经过普通 3 帧消抖；当帧 Signal 清零。

#### 普通活动状态 → IDLE

```text
Pressure <= PressureOff
或
(
    Pressure <= PeakPressure × 12%
    且 abs(FastK) <= 15 %/s
    且 abs(SlowK) <= 5 %/s
)
```

上述条件必须连续 16 帧；中间任一帧不满足就清零计数。

#### 普通活动状态之间

- RISING：Fast K `>= +20 %/s`，连续 3 帧。
- FALLING：Fast K `<= -30 %/s`，或者 Slow 窗口已填满且 Slow K `<= -8 %/s`，连续 3 帧。
- FLOATING：活动至少 160 帧，Fast K 绝对值不超过 15%，Slow K 绝对值不超过 10%，连续 3 帧。
- HOLD：更高优先级条件均不成立，Fast K 绝对值不超过 15%，Slow K 绝对值不超过 5%，连续 3 帧。

Slow 窗口未填满时，普通 Slow 稳定条件暂按成立处理；但 `Slow K <= -8%/s` 只有窗口填满后才能参与 FALLING 判定。

#### FLOATING 退出

- Fast K `>= +80 %/s` 连续 8 帧才进入 RISING。
- Fast K `<= -100 %/s` 连续 8 帧才进入 FALLING。
- 快速松手仍然可以立即进入 SLOW_BACK。

FLOATING 使用比普通活动状态更宽的退出阈值和更长的确认，防止低压力通道因 1 count 量化波动来回切换。

#### SLOW_BACK → RISING

SLOW_BACK 内持续记录局部低点：

```text
SlowBackReferenceRaw = min(SlowBackReferenceRaw, Raw)
RetriggerDelta = max(Raw - SlowBackReferenceRaw, 0)
```

重新按压条件：

```text
RetriggerDelta > PressureOn
且 FastRawSlope >= +80 count/s
且连续 3 帧
```

正式进入 RISING 后重新建立 ActivationBaseline、PeakPressure 和活动持续时间，不沿用上一轮压力峰值。

#### SLOW_BACK → IDLE

```text
Pressure <= PressureOff
且 abs(FastRawSlope) <= 20 count/s
且连续 64 帧
```

若 4800 帧内始终不能自然恢复，则立即进入 ZERO_REBASE。

#### ZERO_REBASE → IDLE / RISING

- 重新按压条件与 SLOW_BACK 相同。
- Baseline 与 Raw 的绝对误差不超过 `PressureOff`，且 Fast Raw 绝对斜率不超过 20 count/s，连续 64 帧后进入 IDLE。

### 7.7 唯一状态写入口和消抖

只有 `commit_state()` 可以修改 `current_state`：

- `SLOW_BACK`、`ZERO_REBASE`、`IDLE` 为立即提交状态；它们在分类阶段已经有专用确认或必须立即屏蔽输出。
- RISING、HOLD、FALLING、FLOATING 的普通变化需要候选连续 3 帧。
- FLOATING 到 RISING/FALLING 使用 8 帧专用确认。
- 候选中途变化时重新计数；候选重新等于当前状态时清除未完成计数。

## 8. Baseline Tracker

Baseline Tracker 与通道状态机分开，包含 5 个内部阶段：

| 阶段 | 行为 |
| --- | --- |
| `IDLE` | `Baseline += (Raw-Baseline) × 327/65536`，约 0.499%/帧 |
| `FROZEN` | 活动或慢回弹期间 Baseline 不动 |
| `WAIT_STABLE` | 回到 IDLE 后先等待 Raw 连续稳定 80 帧 |
| `RECOVER` | `Baseline += (Raw-Baseline) × 1311/65536`，约 2.0004%/帧 |
| `ZERO_REBASE` | SLOW_BACK 超时后以同样约 2%/帧受控重建零点 |

关键条件：

```text
提前冻结压力 = max(Noise × 3, 12)
单帧稳定范围 = max(Noise, 2 counts/frame)
恢复完成误差 = max(Noise, 1 count)
WAIT_STABLE = 80 帧，约 500 ms
```

即使状态机的 3 帧激活消抖尚未完成，只要 Raw 已明显离开 Baseline，也要提前冻结 Baseline，防止把新按压吸收到基线中。

ZERO_REBASE 中若 `RawDelta > max(Noise,2)`，本帧暂停 Baseline 跟随，避免把新的正向按压吸收掉。Raw 稳定时 2%/帧跟随约 0.93 秒可消除约 95% 的固定残余偏移。

## 9. 自动量程核心策略

### 9.1 判断量：四通道最大 Hardware Raw

```text
PeakRaw = max(HardwareRaw[CH4], ..., HardwareRaw[CH7])
```

必须使用最大值，不能用和或平均值：

- 饱和是单通道风险，只要任一路接近满量程就必须扩档。
- 使用和会让四通道共同偏高时过早扩档，阈值不再具有“满量程百分比”含义。
- 使用平均值会掩盖单路先饱和。
- 自动量程判断始终使用 Hardware Raw，不能使用已经仿射或平滑的 Norm Raw。

普通升降档使用一阶滤波峰值，Q8 内部实现：

```text
FilteredPeak += (PeakRaw - FilteredPeak) / 4
```

也就是每帧吸收当前误差的 1/4。紧急扩档绕过滤波，直接看当帧 PeakRaw。

### 9.2 普通扩档

```text
FilteredPeak >= 48321（58928 的 82%）
且连续 3 帧
```

满足时只升一级，例如 `27 → 40`。约 18.75 ms 的连续确认用于抑制单帧噪声。

### 9.3 紧急扩档

```text
PeakRaw >= 55982（58928 的 95%）
```

单帧立即触发，并允许在切档过渡期或 16 帧冷却期内再次触发。目标不是固定只升一级，而是估算把 Raw 压回约 70% 满量程所需的最低 Ref：

```text
RequiredRef = ceil(CurrentRef × PeakRaw / 41250)
```

从当前档以上选择第一个 `Ref >= RequiredRef` 的档位，必要时可以跨级直达更高量程；最高限制为 Ref=135。

从 Ref=12 上电不会牺牲保护性，因为突发大压力可由 95% 单帧紧急扩档快速跨级处理。

### 9.4 降档/回落

必须同时满足：

1. CH4～CH7 每一路分别处于 `IDLE` 或 `SLOW_BACK`，四路可以任意混合；
2. `FilteredPeak <= 20625`，即当前 Max Count 的 35%；
3. 上述低水位连续 32 帧，约 200 ms；
4. 当前不是最低档；
5. 预测降低一级后的 Hardware Raw 不超过 38303，即 65% Max Count。

预测公式向上取整：

```text
PredictedPeak = ceil(FilteredPeak × CurrentRef / LowerRef)
PredictedPeak <= 38303
```

每次只降低一级。任一路处于 RISING、HOLD、FALLING、FLOATING 或 ZERO_REBASE，或者低水位中断，`low_count` 都会清零。

### 9.5 防止档位来回摆动的六层措施

1. 扩档阈值 82%，降档当前阈值 35%，天然形成大死区。
2. 降档还要预测下一低档不超过 65%，避免一降档立刻重新触发 82% 扩档。
3. 普通扩档需连续 3 帧，降档需连续 32 帧。
4. 降档只允许在四路均已进入 IDLE/SLOW_BACK 安全集合时发生。
5. 每次自动切档后冷却 16 帧，约 100 ms；普通升降档均暂停。
6. 切档过渡期间只允许紧急扩档，不允许普通升档或降档。

### 9.6 应用 Ref 的时机

档位判定只产生 pending 请求，真正的 `CapSenseMatrixRefCdac_Apply()` 必须在 CapSense 扫描空闲点调用。写入后必须回读实际 Ref，成功后才更新 `actual_ref_cdac` 和 `tier_index`；失败则进入 ERROR。

`requested_ref_cdac` 表示控制器最近请求的目标，`actual_ref_cdac` 表示硬件当前实际值。上位机下拉框保存用户待切换值，不能用周期状态帧把它强制覆盖回实际值。

## 10. 仿射校准

### 10.1 根本目的

Ref 改变后 Hardware Raw 的比例并非严格等于理论 Ref 比例，并且四个通道各不相同。若只做：

```text
Norm = Hardware Raw × Ref / 60
```

同一物理压力在相邻档位会得到不同 Norm Raw，静态切档就会形成台阶。仿射校准的目的，是让每个通道、每个档位都映射到同一个 Ref=60 计量域：

```text
Norm(ch, ref) = a(ch, ref) × HardwareRaw(ch, ref) + b(ch, ref)
```

最终目标是同一压力下升档和降档后的稳态 Norm Raw 尽量重合。仿射解决的是“稳态档间比例和偏移”；速度预测/Hermite 解决的是“动态切档期间的时间连续性”，二者不能相互替代。

### 10.2 校准样本组织

- P0 固定为空载。
- 其他压力点应从低压到极限压力覆盖有效 Raw 范围。
- 每个压力点必须保持物理压力不变，依次测完七个 Ref。
- 每档保存最近 16 个稳定 Debug 帧，四通道分别取中位数，并保存总体标准差、最小值、最大值和时间。
- Raw `>=48321` 的通道/档位已经进入自动扩档边界，保留作记录但排除拟合。
- 压力点不是越多越好；至少保证每个“通道×档位”有 6 组不饱和、分布足够宽的有效配对。重复堆积同一小范围样本，价值低于补足缺失的低、中、高压力覆盖。

当前项目文件为：

`PY_SRIPT/algo_monitor/calibration_projects/autorange_calibration_20260902_154948_485242.arcal.json`

它包含 15 个压力点、28 组系数，当前 `coverage` 和 `recommendations` 均为空。此文件是可追溯的工作项目；`calibration_exports` 下的 CSV 只是时间戳快照，不应作为唯一项目存档。

### 10.3 自动采集顺序

自动采集单个压力点时使用对称序列：

```text
135 → 90 → 60 → 40 → 27 → 18 → 12
    → 18 → 27 → 40 → 60 → 90 → 135
```

这样每个非中心档同时取得正向和反向结果，可识别材料慢漂移、压力变化和切档方向相关误差。

采集门控：

1. 先在 Ref=135 检查 2 秒慢趋势；第一四分之一与最后四分之一中位数的变化，逐通道不得超过 `max(20 counts, 末值×0.2%)`。
2. 预检查总超时 15 秒；不稳定时重新开始检查。
3. 每个档位切换最长等待 5 秒。
4. 只有手动模式标志有效、无 pending、`blank_frames=0`、requested Ref 等于 actual Ref 时才开始采样。
5. 收集 16 帧，逐通道短时噪声 σ 必须不超过 `max(5 counts, Raw×0.1%)`。
6. 切档超时或噪声超限自动重试 1 次；再次失败则中止，本压力点旧数据保持不变。
7. 全部 13 步成功后，正反向值取平均作为该 Ref 最终样本，并原子替换旧压力点数据；旧数据移入 history。

正反向差值超过以下限制时认为可能存在压力慢漂移：

```text
abs(ReverseRaw - ForwardRaw) > max(100 counts, 相对P0的有效信号增量 × 1%)
```

完整自动点如果自身的双向差和噪声合格，但与旧手动点的全局拟合不一致，只给出“复核旧数据/模型”的警告，不再反复要求重测这个已经自洽的新点。

### 10.4 相邻档拟合与路径组合

不是对所有档位直接独立拟合到 Ref60，而是先拟合相邻且有共同有效压力范围的边：

```text
12 → 18 → 27 → 40 → 60
90 → 60
135 → 90 → 60
```

每条边使用普通最小二乘：

```text
y = a × x + b
```

若有两段：

```text
y = a1×x + b1
z = a2×y + b2
```

则组合结果为：

```text
z = (a2×a1)×x + (a2×b1+b2)
```

按路径逐段组合即可得到任意 Ref 到 Ref60 的最终 `a、b`。Ref60 固定为：

```text
a = 1
b = 0
gain_q16 = 65536
offset_int = 0
```

最终定点参数：

```text
gain_q16 = round(a × 65536)
offset_int = round(b)
```

Python 还使用各档直接到 Ref60 的有效配对计算最终 RMSE、最大误差和 R²，用于评价组合路径，而不是用这些直接配对替换相邻路径系数。

### 10.5 质量判定

| 检查 | 当前规则 |
| --- | --- |
| 饱和排除 | 源档或 Ref60 任一 Raw `>=48321`，该配对不参与拟合 |
| 最低覆盖 | 每通道、每档至少 6 个有效配对 |
| 单档短时噪声 | σ `<=max(5, Raw×0.1%)` |
| 单点采集跨度 | 超过 30 s 警告；超过 60 s 时普通压力点建议整点重测，P0 只警告复核空载漂移 |
| 双向慢漂移 | 差值 `>max(100, Signal×1%)` 建议重测 |
| 留一验证容许误差 | `max(100 counts, 有效信号×1%)` |
| 留一误差比 | `>2` 警告，`>5` 建议重测；完整自洽自动点只做全局警告 |
| 多通道同向偏差 | 同一 Ref 至少 3 通道同方向且误差比中位数 `>4`，怀疑压力慢漂移 |

留一验证只在训练样本足够且待验证 Raw 位于训练范围内时进行，避免用外推误差错误否定极端压力点。

### 10.6 固件定点应用

```text
normalized_q16 = HardwareRaw × gain_q16 + offset_int × 65536
NormRaw = round(normalized_q16 / 65536)
```

- 使用 64 位有符号中间值。
- 结果 `<=0` 时输出 0。
- 正数加 32768 后右移 16 位实现四舍五入。
- 超过 `uint32` 时饱和到 `UINT32_MAX`。
- 送入旧算法前再限幅到 65535；V13 Debug 仍保留完整 32 位值。
- 无有效 Flash 参数、通道非法或 Ref 不在固定表时，回退到 `round(HardwareRaw×Ref/60)`。
- Type `0x28 flags.bit7=1` 表示本帧实际使用了仿射参数；bit7=0 表示理论比例回退。

### 10.7 当前项目的 28 组参数快照

以下参数来自当前 `.arcal.json`，用于同一硬件的复现和回读核对。它们与传感器、结构、装配和环境相关；移植到另一只样机时应重新标定，不能默认直接复用。

| CH | Ref | a | b | gain_q16 | offset | RMSE | 最大误差 | R² | 有效对数 | 拟合路径 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| CH4 | 12 | 0.237141323 | -57.386 | 15541 | -57 | 91.198 | 279.103 | 0.997594 | 10 | 12→18→27→40→60 |
| CH4 | 18 | 0.320395003 | 20.553 | 20997 | 21 | 94.306 | 272.050 | 0.999220 | 11 | 18→27→40→60 |
| CH4 | 27 | 0.470580468 | 12.931 | 30840 | 13 | 76.142 | 179.068 | 0.999837 | 12 | 27→40→60 |
| CH4 | 40 | 0.678586503 | -0.158 | 44472 | 0 | 59.092 | 153.780 | 0.999958 | 13 | 40→60 |
| CH4 | 60 | 1.000000000 | 0.000 | 65536 | 0 | 0.000 | 0.000 | 1.000000 | 14 | identity |
| CH4 | 90 | 1.495166583 | -39.112 | 97987 | -39 | 75.048 | 204.048 | 0.999967 | 14 | 90→60 |
| CH4 | 135 | 2.213671466 | -67.630 | 145075 | -68 | 120.443 | 338.272 | 0.999914 | 14 | 135→90→60 |
| CH5 | 12 | 0.219421876 | 24.664 | 14380 | 25 | 156.004 | 357.352 | 0.996779 | 7 | 12→18→27→40→60 |
| CH5 | 18 | 0.318862074 | 10.449 | 20897 | 10 | 111.805 | 228.923 | 0.999372 | 10 | 18→27→40→60 |
| CH5 | 27 | 0.468354989 | 14.617 | 30694 | 15 | 74.736 | 147.755 | 0.999719 | 10 | 27→40→60 |
| CH5 | 40 | 0.673891847 | 17.052 | 44164 | 17 | 45.600 | 117.630 | 0.999954 | 11 | 40→60 |
| CH5 | 60 | 1.000000000 | 0.000 | 65536 | 0 | 0.000 | 0.000 | 1.000000 | 13 | identity |
| CH5 | 90 | 1.490117763 | -28.443 | 97656 | -28 | 49.225 | 94.687 | 0.999983 | 13 | 90→60 |
| CH5 | 135 | 2.205263480 | -38.471 | 144524 | -38 | 91.707 | 257.839 | 0.999940 | 13 | 135→90→60 |
| CH6 | 12 | 0.231535822 | -107.462 | 15174 | -107 | 129.113 | 274.623 | 0.997695 | 10 | 12→18→27→40→60 |
| CH6 | 18 | 0.321688995 | 0.520 | 21082 | 1 | 100.072 | 217.518 | 0.999319 | 11 | 18→27→40→60 |
| CH6 | 27 | 0.471109461 | 3.738 | 30875 | 4 | 71.839 | 144.618 | 0.999861 | 12 | 27→40→60 |
| CH6 | 40 | 0.678228067 | -5.387 | 44448 | -5 | 50.362 | 101.593 | 0.999966 | 13 | 40→60 |
| CH6 | 60 | 1.000000000 | 0.000 | 65536 | 0 | 0.000 | 0.000 | 1.000000 | 14 | identity |
| CH6 | 90 | 1.495045723 | -40.271 | 97979 | -40 | 55.134 | 151.385 | 0.999981 | 14 | 90→60 |
| CH6 | 135 | 2.217171274 | -72.322 | 145305 | -72 | 83.509 | 207.505 | 0.999957 | 14 | 135→90→60 |
| CH7 | 12 | 0.218771976 | 15.876 | 14337 | 16 | 92.497 | 174.151 | 0.999241 | 6 | 12→18→27→40→60 |
| CH7 | 18 | 0.318066117 | 4.161 | 20845 | 4 | 63.621 | 97.000 | 0.999802 | 9 | 18→27→40→60 |
| CH7 | 27 | 0.468163264 | 8.641 | 30682 | 9 | 61.646 | 152.138 | 0.999847 | 10 | 27→40→60 |
| CH7 | 40 | 0.674663602 | 9.385 | 44215 | 9 | 33.452 | 76.518 | 0.999977 | 11 | 40→60 |
| CH7 | 60 | 1.000000000 | 0.000 | 65536 | 0 | 0.000 | 0.000 | 1.000000 | 13 | identity |
| CH7 | 90 | 1.488805718 | 1.100 | 97570 | 1 | 47.790 | 114.949 | 0.999986 | 13 | 90→60 |
| CH7 | 135 | 2.205506214 | -25.809 | 144540 | -26 | 86.268 | 269.383 | 0.999953 | 13 | 135→90→60 |

## 11. Flash 持久化

### 11.1 记录布局

**历史布局，禁止在现行固件使用：** 当时仿射记录位于`0x0001FF00`，总长252字节。现行该地址属于禁止应用擦写的保护区；仿射功能已停用，仅在`0x1FC00～0x1FCFF`预留一行。下面字段只供识别旧记录，不是现行写入规范：

| 偏移 | 内容 |
| ---: | --- |
| 0～3 | 魔数 `ARAF` |
| 4 | 存储版本 1 |
| 5 | 设备类型 1 |
| 6 | 通道数 4 |
| 7 | 档位数 7 |
| 8～11 | `calibration_id` |
| 12～15 | 保存代次 `generation` |
| 16～22 | 七档 Ref 表 |
| 23 | 对齐/保留 |
| 24～247 | 28×(`gain_q16 int32` + `offset int32`)，共 224 字节 |
| 248～251 | 前 248 字节的 FNV-1a 校验 |

移植到其他芯片时，`0x1FF00` **绝不能直接照搬**。必须根据目标芯片 Flash 容量、行大小、链接脚本、应用镜像、Bootloader/IAP 元数据和其他配置区重新分配，并保留编译期“记录不超过一行”的检查。

### 11.2 参数事务与校验

- 四个通道使用同一个 32 位 `calibration_id`。
- ID 按 CH4→CH7 顺序计算；先加入物理通道号，每档再加入 Ref、`gain_q16` 小端 4 字节和 `offset` 小端 4 字节。
- 新 ID 到达会清除旧的未完成 staging，防止新旧参数混合。
- 只有四通道全部到齐、固件重算 ID 一致、所有范围合法时才能写 Flash。
- `gain_q16` 必须大于 0 且不超过 `16×65536`；`abs(offset)` 不超过 1,000,000。
- Ref60 必须严格为 `65536, 0`。
- 保存后立即重新从 Flash 解析，并逐项核对；启动时再次校验魔数、版本、设备、维度、档位表、参数 ID 和记录校验。

### 11.3 上位机完整验证顺序

1. 用 `0x92` 依次暂存 CH4、CH5、CH6、CH7。
2. 每包等待 `0x2A` 操作结果。
3. 用 `0x93 + calibration_id` 保存完整记录。
4. 等待保存成功和写后回读验证完成。
5. 用 `0x95` 请求软件复位。
6. 串口消失时最多自动重连 5 次。
7. 用 `0x94` 依次读取四通道。
8. 确认四路 bit0/bit1 均为 1、ID 和 generation 相同、NVM 状态为 0、56 个整数逐项一致。

只有完成复位后回读，才能证明“写入成功”和“冷启动可加载”同时成立。

## 12. 切档连续性策略

### 12.1 为什么仿射后还需要动态过渡

仿射参数能减小静态档间误差，但 Ref 写入后仍存在硬件暂态；动态按压时，压力本身也在切档等待期间继续变化。历史方案先后经历：

1. 固定冻结 8 帧：会形成水平平台，最后一次释放积累变化。
2. “稳定 3 帧后释放”：动态压力下可能等到 24 帧上限，形成更宽平台。
3. 8 帧冻结 + 8 帧线性偏移衰减：静态台阶改善，但动态按压仍出现约 71 ms 平台和等幅台阶。
4. 当前方案：等待期间继续按速度预测，再用 Hermite 同时对齐位置和速度。

因此当前实现不能退回“冻结旧 Norm Raw”的旧逻辑。

### 12.2 当前过渡阶段

切档过渡分为三态：

1. `IDLE`：直接输出 Candidate Norm Raw。
2. `PREDICT`：新 Ref 候选数据尚未稳定，Final Norm Raw 沿切档前速度继续预测。
3. `HERMITE`：用 8 帧三次曲线把预测轨迹平滑并入实时 Candidate 轨迹。

### 12.3 PREDICT 阶段

- 切档前保存每通道最近 4 个 Final Norm Raw 差分。
- 以 4 个差分的中位数作为初始速度。
- 速度限制在 `±8192 counts/frame`。
- 每一帧：`PredictedPosition += Velocity`，不再输出水平平台。
- 新 Ref 至少等待 8 帧，约 50 ms。
- 同时记录每个通道的 Candidate 差分。

最短 8 帧结束后，对 CH4～CH7 分别取最近 3 个 Candidate 差分：

```text
max(delta[3]) - min(delta[3]) <= 128 counts/frame
```

四个通道必须同帧全部满足，并连续满足 3 帧，才允许进入 Hermite。任一路失败，连续计数清零。

候选等待整条连续切档链最多 24 帧，约 150 ms。达到上限即强制进入 Hermite，避免异常输入导致永久冻结。注意：24 帧是候选等待上限，后面仍有最多 8 帧 Hermite，因此完整过渡理论上最多约 32 帧，即 200 ms。

### 12.4 Hermite 阶段

每个通道记录：

```text
PositionError = 当前预测位置 - 当前Candidate
VelocityError = 当前预测速度 - 最近3个稳定Candidate差分的中位数
```

在 8 帧内令 `t` 从 0 走到 1，使用：

```text
h00(t) = 2t³ - 3t² + 1
h10(t) = t³ - 2t² + t
Correction = h00×PositionError + h10×8×VelocityError
Final = Candidate + Correction
```

这样起点位置和速度连续，终点的位置误差与速度误差都归零。为防止三次速度项造成软件鼓包，最终输出还必须限制在以下包络内：

```text
下界 = min(当前Candidate, 上一轨迹位置+上一轨迹速度)
上界 = max(当前Candidate, 上一轨迹位置+上一轨迹速度)
```

连续紧急扩档不会重新从旧值开始，而是保留当前 Final 位置和速度，只清空新档位 Candidate 历史并重新进入 PREDICT；24 帧候选等待预算由整条连续切档链共享。

### 12.5 `blank_frames` 的语义

- 最少等待阶段：显示剩余 8～1 帧。
- 已满 8 帧但 Candidate 尚未连续稳定：保持为 1。
- Hermite 阶段：显示剩余 8～1 帧。
- 完成：0。

上位机只有在 `blank_frames=0`、无 pending 且实际 Ref 等于请求 Ref 时，才能认为档位稳定并保存数据。

## 13. 切档期间状态机隔离

Final Norm Raw 在 PREDICT/Hermite 中是合成轨迹，不能被状态机误认为真实压力斜率。因此当前 CH4～CH7 做以下隔离：

### 13.1 过渡期间

- 保持切档前已确认的通道状态。
- 不把合成 Final Norm Raw 加入 Fast/Slow 斜率窗口。
- 清除尚未完成的普通状态候选。
- 清零 `idle_confirm_count` 和 `slow_back_stable_count`，不在切档期间累计释放确认。
- Fast K、Slow K 遥测标记为未计算。

### 13.2 过渡结束首帧

- 清空 Fast/Slow Raw 历史、样本数、滚动和及回归分子。
- 以当前真实 Candidate 终点作为新窗口第一个样本。
- 保持原状态，本帧只完成同步，从下一帧恢复分类。
- 若原状态为 SLOW_BACK，以当前 Raw 重建局部低点，防止跨档差值被误认为重新按压。

Fast K 至少需要重新积累 16 帧；Slow K 需要重新积累 160 帧，约 1 秒。上位机应显示 `-- 未计算`，不能显示数值 0，因为 0 会被误解为“已经计算且斜率为零”。界面注释应说明：CH4～CH7 切档过渡期间暂停 K 计算；过渡完成后 Slow K 需重新积累约 160 帧。

## 14. UART V14 移植要点

### 14.1 自动量程状态 `0x28`

payload 固定 20 字节：

| 偏移 | 字段 | 类型 |
| ---: | --- | --- |
| 0～1 | nsub=300 | `uint16` |
| 2～3 | tx_div=100 | `uint16` |
| 4～5 | max_count=58928 | `uint16` |
| 6 | actual_ref_cdac | `uint8` |
| 7 | tier_index | `uint8` |
| 8 | tier_count=7 | `uint8` |
| 9 | autorange state 0～8 | `uint8` |
| 10 | flags | `uint8` |
| 11 | requested_ref_cdac | `uint8` |
| 12～13 | peak_raw | `uint16` |
| 14～15 | filtered_peak_raw | `uint16` |
| 16 | high_count | `uint8` |
| 17 | low_count | `uint8` |
| 18 | cooldown_frames | `uint8` |
| 19 | blank_frames | `uint8` |

flags：bit0 pending、bit1 最低档、bit2 最高档、bit3 错误、bit4 有效接触、bit5 Algorithm Raw 被限幅、bit6 手动校准、bit7 仿射已应用。

自动量程状态值：0 STARTUP、1 HOLD、2 UP_PENDING、3 DOWN_PENDING、4 SWITCHING、5 COOLDOWN、6 LIMIT、7 ERROR、8 MANUAL_HOLD。

### 14.2 手动校准 `0x91`

payload 2 字节：

```text
manual_mode uint8
ref_cdac    uint8
```

- `manual_mode=1` 时 Ref 只接受 12、18、27、40、60、90、135。
- `manual_mode=0` 时 Ref 必须为 0；退出后从当前实际档位继续自动控制。
- 手动模式暂停所有自动升降档，但仍使用相同的 PREDICT/Hermite 切档过渡。

### 14.3 参数持久化命令

| 命令/帧 | 方向 | payload |
| --- | --- | ---: |
| `0x92` 暂存单通道 | 主机→设备 | 64 字节 |
| `0x93` 保存四通道 | 主机→设备 | 4 字节 ID |
| `0x94` 查询单通道 | 主机→设备 | 1 字节通道号 |
| `0x95` 软件复位 | 主机→设备 | 0 |
| `0x29` 参数回读 | 设备→主机 | 72 字节 |
| `0x2A` 操作结果 | 设备→主机 | 8 字节 |

完整字段布局以 `UART_PROTOCOL_UNIFIED_V3_CN.md` 为准。

### 14.4 V13 Debug `0x10`

每通道由 22 字节增加到 26 字节，8 通道 payload 为 208 字节。数组顺序必须保持一致：

```text
hardware_raw[0..N-1]              uint16
normalized_raw[0..N-1]            uint32  最终值
candidate_normalized_raw[0..N-1]  uint32  未经过渡候选值
baseline[0..N-1]                  uint16
signal[0..N-1]                    uint16
fast_k_percent_x100[0..N-1]       int32
slow_k_percent_x100[0..N-1]       int32
noise[0..N-1]                     uint16
state[0..N-1]                     uint8
```

固件、Python `PROTOCOL_VERSION`、payload 长度检查、解析偏移、DataBuffer 和 CSV 表头必须同时升级，否则最常见的表现不是“只少一列”，而是从 Candidate 字段之后的全部数组错位。

## 15. Python 项目与导出约定

- 标定工作文件：单个 `.arcal.json`，包含项目 ID、压力点固定 ID、样本、历史重测、28 组参数和质量分析。
- 增删压力点不重新编号；P0 固定保护。
- 每次采样、重测、增删或重新拟合后自动更新同一个项目文件。
- CSV 是显式快照，每次导出在独立时间戳目录生成 `samples.csv`、`coefficients.csv` 和 `quality_report.csv`。
- `calibration_exports/` 和 `tmp/` 已加入 `.gitignore`，不要依赖其中某个“最新 CSV”恢复项目。
- 档位连续性测试每次只生成一个 CSV，其中依次包含 summary、逐切档/通道 detail 和完整 sample 轨迹。

## 16. 档位连续性自动测试

### 16.1 默认测试参数

```text
序列：12→18→27→40→60→90→135→90→60→40→27→18→12
循环：3 轮
每档稳定采样：24 帧
警告阈值 absolute_limit：200 counts
```

每次切档按通道计算：

- 切档前后稳态均值；
- 有符号台阶和绝对台阶；
- 台阶占 Max Count 百分比；
- 前后合并噪声；
- 过渡最大偏差和额外过冲；
- 连续 3 帧进入稳定带所需时间；
- 同一 Ref 的升/降方向漂移；
- Ref12 完整往返漂移。

核心指标公式：

```text
Step = PostNormMean - PreNormMean
AbsoluteStep = abs(Step)
StepFullScalePercent = AbsoluteStep / 58928 × 100%
CombinedNoise = hypot(PreStddev, PostStddev)
PeakDeviation = max(abs(过渡及新稳态样本 - PostNormMean))
Overshoot = max(0, PeakDeviation - AbsoluteStep)
SettlingTolerance = max(10 counts, 3 × PostStddev)
```

稳定时间从切档命令时刻开始计算，第一次出现连续 3 帧都落在 `PostNormMean ± SettlingTolerance` 内时结束。

当前默认判定：

- 任一端 Hardware Raw `>=48321`：标记“无效/接近饱和”，不作为仿射失败。
- 绝对稳态台阶 `>200`：警告。
- 绝对稳态台阶 `>400`：失败。
- 过冲 `>200`：警告。
- 合并噪声 `>max(30, 200/3)`：警告。
- 找不到连续 3 帧稳定：警告。

`absolute_limit` 可在界面调整。固定压力刚施加后材料仍在慢变时，过早测试可能产生警告；延长静置后警告消失，通常说明压力/材料尚未稳态，不一定是仿射参数错误。

### 16.2 建议覆盖的测试场景

1. 空载双向切档：检查 b 偏移和 P0 连续性。
2. 低压力：重点覆盖 Ref12、18、27 的有效区间。
3. 中压力：覆盖最常用切档边界。
4. 500 g、1 kg：检查高档和材料稳定等待。
5. 2～3 kg 极限压力：确认 Ref135 不饱和，低档饱和应被正确标记无效。
6. 动态缓慢加压：比较 Hardware、Candidate、Final，确认没有宽平台。
7. 动态快速加压：验证 95% 紧急扩档可跨级且不丢失轨迹。
8. 缓慢卸载和快速松手：验证 SLOW_BACK、降档门控和 Signal 清零。
9. 长时固定压力：验证 FLOATING 蠕变锁定和退出迟滞。
10. 档位阈值附近反复波动：确认 82%/35%/65%、确认帧和 cooldown 不会来回摆档。
11. Flash 复位回读：确认 28 组参数真正跨重启保存。
12. 多台样机：确认当前系数是否仅适用于单台装配，并决定量产校准策略。

## 17. 推荐移植顺序

为避免一次引入太多耦合，建议严格按以下阶段移植，每阶段通过后再继续：

### 阶段 A：状态机与 Baseline

1. 先移植状态枚举，确保协议状态值 0～6 不变。
2. 移植 `channel_state_machine` 的窗口、阈值、优先级和唯一提交入口。
3. 移植 `baseline_tracker`，接入单通道每帧处理顺序。
4. 用固定单一 Ref 回放 Raw，验证 7 状态和 Signal/Baseline 行为。

### 阶段 B：固定采样域

1. 固定 MatrixButtons0 为 Nsub=300、TxDiv=100。
2. 验证 CIC2 Decimation=255、Samples=116、Divider=128、Max Count=58928。
3. 确认实测算法帧率接近 160 Hz；否则统一缩放所有时间型阈值。
4. 上电设置并回读 Ref=12。

### 阶段 C：只做自动量程，先用理论比例

1. 接入 Hardware Raw 最大值和 1/4 峰值滤波。
2. 实现 82%×3 普通扩档和 95% 单帧紧急扩档。
3. 实现 IDLE/SLOW_BACK 任意组合、35%×32、预测 65% 的降档。
4. 在扫描空闲点写 Ref，加入 16 帧 cooldown。
5. 暂用 `HardwareRaw×Ref/60`，确认档位控制方向正确。

### 阶段 D：手动校准和 Python 采集

1. 加入 `0x90/0x28` 状态查询和 `0x91` 手动档位。
2. 接入 Hardware Raw 独立上传。
3. 移植 `.arcal.json` 项目、压力点 Tab、16 帧采样和自动双向序列。
4. 完成有效覆盖、噪声、饱和、漂移和留一质量分析。

### 阶段 E：28 组参数与 Flash

1. 先离线生成 28 组 Q16 参数。
2. 加入 `0x92～0x95`、`0x29/0x2A`。
3. 为目标芯片重新分配 Flash 行。
4. 完成暂存、保存、软件复位、重连和 56 个整数回读核对。
5. 此阶段仍可不应用参数，以隔离 Flash 问题和算法问题。

### 阶段 F：应用仿射

1. 在 Candidate Norm Raw 路径应用 64 位定点公式。
2. 保留理论比例回退和 bit7 状态。
3. 用固定压力手动双向切档，先只比较稳态 Candidate。
4. Candidate 的稳态档间误差合格后再进入动态平滑阶段。

### 阶段 G：动态平滑与状态隔离

1. 加入 Final 输出 4 差分速度历史。
2. 加入最少 8 帧预测、Candidate 最近 3 差分极差 128、连续 3 帧确认和 24 帧上限。
3. 加入 8 帧 Hermite 和 Candidate/预测包络钳制。
4. 连续紧急扩档保留轨迹并共享等待预算。
5. 切档期间暂停 CH4～CH7 状态分类，结束后重建 Fast/Slow 窗口。
6. 升级 UART V13 并上传 Candidate Norm Raw。
7. 上位机 Slow K 显示“未计算”。

### 阶段 H：系统回归

执行第 16 节的静态、动态、释放、长时、饱和、复位和多压力测试，并同时检查通用录制 CSV、连续性 CSV、状态帧和 Flash 回读。

## 18. 常见移植错误

1. 用四路平均值或和判断饱和，而不是最大 Hardware Raw。
2. 用 Norm Raw 判断量程，导致仿射和平滑反过来影响硬件安全边界。
3. 把“全部 IDLE 或全部 SLOW_BACK”的旧条件照搬，导致三路 IDLE、一路 SLOW_BACK 时永不降档。
4. 把 ZERO_REBASE 也当作降档安全状态，切档扰动零点重建。
5. 修改 Ref 时重新初始化固定 CIC2，额外制造长暂态。
6. 切档时冻结 Final Norm Raw，动态压力下重新形成宽平台。
7. Candidate 只等固定 8 帧，不检查速度稳定，导致 Hermite 目标速度被硬件暂态污染。
8. Hermite 不做包络限制，产生软件鼓包或小毛刺。
9. 切档合成轨迹继续进入状态机斜率窗口，导致 CH4～CH7 状态异常。
10. 把过渡期 Slow K 的 0 当作真实计算结果，而不是“未计算”。
11. Ref60 不固定单位映射，导致整套计量域漂移。
12. 把当前样机的 28 组系数直接复制到不同结构或不同传感器批次。
13. 只确认 Flash 写成功，不做软件复位后的回读。
14. 固件升级到 V13，但 Python 仍按 V12 的 176 字节 Debug payload 解析。
15. 只修改 `actual_ref` 显示，忽略 `requested_ref` 和 pending，导致采样落在错误档位。
16. 实际帧率改变后仍沿用原帧数，所有状态和切档时间常数随之改变。

## 19. 最小验收清单

移植完成至少应逐项满足：

- [ ] Nsub=300、TxDiv=100、Max Count=58928 实际回读一致。
- [ ] 上电 Ref=12，七档表顺序完全一致。
- [ ] 普通扩档、紧急扩档和降档阈值/连续帧数符合本文。
- [ ] CH4～CH7 的 IDLE/SLOW_BACK 任意组合可降档，其他状态会阻止降档。
- [ ] 每次 Ref 只在扫描空闲点写入，requested/actual/pending 可区分。
- [ ] 7 状态编号、状态优先级、普通 3 帧和 FLOATING 8 帧消抖一致。
- [ ] SLOW_BACK 当帧 Signal 清零，30 秒后可进入 ZERO_REBASE。
- [ ] 28 组参数能暂存、保存、复位加载并逐项回读。
- [ ] Ref60 四通道均为 `65536,0`。
- [ ] `flags.bit7` 能正确区分仿射和理论回退。
- [ ] V14设备身份可查询，Debug同时上传 Hardware、Candidate 和 Final Norm Raw。
- [ ] Candidate 稳态档间误差合格，Final 动态切档无宽平台和明显鼓包。
- [ ] 切档期间状态保持，Fast/Slow K 显示未计算；结束后窗口重新积累。
- [ ] 连续性测试覆盖空载、低压、中压、高压和极限压力。
- [ ] 饱和样本被标记无效，而不是误判为仿射失败。
- [ ] Python 项目使用单一 `.arcal.json`，CSV 只作为导出快照。
- [ ] 固件、协议文档、Python 解析器的版本号、字段长度和数组顺序完全一致。

## 20. 后续修改同步规则

以后修改下列任一项目时，应在同一提交中同步更新本文、`CHANGE.md`、UART 协议和 Python：

- 状态枚举、状态阈值、判定优先级或消抖帧数；
- Nsub、TxDiv、Max Count、帧率或七档 Ref 表；
- 82%/95%/35%/65% 阈值、确认帧数或 cooldown；
- 仿射公式、参数顺序、Flash 地址/布局或 FNV 计算顺序；
- Candidate 稳定窗口、128 counts/frame、24 帧上限、速度限幅或 Hermite 帧数；
- `0x10`、`0x28`、`0x29`、`0x2A`、`0x90～0x95` 的协议字段；
- Python 项目 schema、采样帧数、质量阈值或连续性测试判据。

现行状态机以[MACHINE_STATE.md](MACHINE_STATE.md)为准；UART协议以[V21协议说明](UART_PROTOCOL_UNIFIED_V3_CN.md)和源码为准，本文V14内容仅用于历史追溯。

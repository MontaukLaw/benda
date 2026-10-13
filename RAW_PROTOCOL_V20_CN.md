# 单一Raw数据链路（V20布局，V21继续沿用）

> V21 继续使用本文的 V20 数据帧布局，增加主法向 1～30 N 的 10/20 点 K 表读写资格；新功能见 [自动压力标定](HIGH_FORCE_CALIBRATION_CN.md)。当前固件版本为 V21，配套上位机兼容解析 V20/V21。

2026-10-05核对：本文主要定义启用通道算法后的Type `0x10`。当前默认`RAW_ONLY=0`、`SENSOR_ALGO_BASELINE_ENABLED=0`，实际采样发送V21 Type `0x16`，只包含`N`个小端uint16 Raw，payload为`2*N`字节；当前20通道食指总长50字节，5通道总长20字节。Baseline/Signal占位0、状态-1及K为NaN均由上位机补齐，不是上传测量值，详见[主协议](UART_PROTOCOL_UNIFIED_V3_CN.md)第3.10节。独立`RAW_ONLY=1`的46字节RAW20另见[专项文档](RAW20_SERIAL_CN.md)。

2026-09-19：当前固定三参数固件使用硬件 Raw 直接驱动算法。删除 Norm Raw、Candidate Norm Raw 的运行时副本及 UART Debug 字段，删除独立 Slow Baseline 的计算、缓存和上传。正常 Baseline 跟踪和通道状态机保持原有行为。

## UART 布局

帧头仍为 `DATA + version:u8 + type:u8 + device:u8 + channel_count:u8 + payload_len:u16`，共10字节，小端序。V20固件的`version=20`，当前V21固件为`version=21`，Debug的`type=0x10`；不能因载荷相同而忽略版本。

设通道数为 N，以下各数组依次排列，每组包含 N 个值：

| 数组 | 类型 | 含义 |
| --- | --- | --- |
| raw | uint16 | 同一扫描的真实硬件计数 |
| baseline | uint16 | 正常算法基线 |
| signal | uint16 | 算法信号 |
| fast_k_percent_x100 | int32 | 快速斜率，解码后单位 %/s |
| slow_k_percent_x100 | int32 | 慢速斜率，解码后单位 %/s |
| state | uint16 | 状态机提交的真实通道状态 |

Type `0x10`载荷长度为`16*N`：当前食指20通道320字节，历史食指8通道128字节，拇指/小指5通道80字节；含帧头分别为330、138、90字节。不能再使用旧版`26*N`的长度解析V20/V21，也不能把此Debug长度用于Type `0x16`纯Raw帧。

20点矩阵恢复通道算法后，Run仅发Type `0x01`，Debug及Force Calibration均发Type `0x10`；不上传旧四通道合力和Type `0x02/0x20`力采样。旧压力标定流程不能仅靠恢复Baseline宏直接套用。

删除固定 Noise 数组。内部算法仍使用固定噪声参数 1，不把它当作实测噪声。Type `0x15` Slow Baseline 在 V20 停用，编号保留、不复用。

其他命令和帧的载荷沿用 V19：设备信息、温度、三参数、力换算、标定表等保留。V20 不更改独立的 I²C/SensorBus 布局，包括其兼容用途的固定噪声查询。

## 上位机与记录

- 界面仅保留一份 Raw 曲线及数值，另外保留 Baseline、Signal、Fast/Slow K、真实状态和 Raw 背景状态来源单选。
- 实时 Noise σ64 仍由上位机计算，公式为最近最多 64 帧真实 Raw 的总体标准差，不从固件固定 Noise 字段读取。
- 自动测试、压力标定、砝码标定、手动回弹和长期测试均读取真实 Raw。新 CSV/SQLite 不写 Norm Raw、Candidate Norm Raw、固定 Noise 或 Slow Baseline 字段。
- 内部接口中的 `raw` 和 `hardware_raw` 均指同一份真实计数；后者保留命名以兼容既有硬件计数记录列。
- 上位机仍识别 V14～V19 的旧帧，按旧长度跳过已停用字段，防止错读 Baseline、K 和 State。旧 Slow Baseline 帧只保留解析兼容，不进入当前显示和新记录。
- 既有 CSV、JSON、SQLite 不就地改写。离线回弹分析优先读取旧数据库中的 hardware_raw；只有缺少该列的旧档案才回退读取 normalized_raw，此类历史计数不能视作新的硬件 Raw 标定。
- 当前升级使用V21固件及配套上位机源码；V21沿用V20数据布局，但旧上位机不一定接受V21版本号。新10/20点K表须确认V21后写入。固件文件生成不等于已烧录设备；Flash边界和Bootloader差异见[Flash布局](FLASH_LAYOUT_CN.md)。

验证入口：`tools/test_raw_protocol_v20.py` 从固件提取实际组帧函数，在 ARM 仿真中覆盖三种设备，并检查逐字节解析、正负 K、真实状态、旧帧、错误长度、新记录及 σ64。

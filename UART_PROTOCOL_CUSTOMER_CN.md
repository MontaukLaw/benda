# 中指/无名指传感器 UART 协议 v3

> 历史资料：本文保留旧版本协议与实验背景，不是当前操作规范。现行UART为V21，沿用V20 Raw布局；统一入口见[当前实现](CURRENT_SYSTEM_CN.md)，命令见[现行协议](UART_PROTOCOL_UNIFIED_V3_CN.md)，地址见[Flash布局](FLASH_LAYOUT_CN.md)。旧帧编号、仿射流程以及0x1FE00/0x1FF00写入地址不得用于现行固件。

## 1. 通信参数

| 项目 | 参数 |
| --- | --- |
| 波特率 | `1000000` bps |
| 串口格式 | 8 数据位、1 停止位、无校验、无流控 |
| 字节序 | 小端（little-endian） |
| 浮点数 | IEEE-754 `float32`，小端 |
| 协议版本 | `3` |

设备上电后持续输出 Run 数据。每次采样依次发送一帧通道信号和一帧三维力结果，共 64 字节；每秒额外发送一帧芯片温度。

## 2. 公共帧格式

```text
DATA[4] | version[1] | type[1] | device[1] | channel_count[1] | payload_len[2] | payload
```

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0～3 | `magic` | ASCII[4] | 固定为 `DATA`，十六进制 `44 41 54 41` |
| 4 | `version` | `uint8` | 固定为 `3` |
| 5 | `type` | `uint8` | 帧类型 |
| 6 | `device` | `uint8` | 固定为 `1` |
| 7 | `channel_count` | `uint8` | Run 帧固定为 `8`；温度帧为 `0` |
| 8～9 | `payload_len` | `uint16` | payload 长度，小端 |
| 10～ | `payload` | 字节数组 | 按帧类型解析 |

帧总长度为 `10 + payload_len`。协议不包含校验和。

## 3. Type `0x01`：通道信号帧

| 项目 | 值 |
| --- | --- |
| `type` | `0x01` |
| `payload_len` | `16` |
| 帧总长度 | `26` 字节 |

帧头：

```text
44 41 54 41 03 01 01 08 10 00
```

payload 为 8 个小端 `uint16`：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0～1 | `signal[0]` | `uint16` | CH0 信号值 |
| 2～3 | `signal[1]` | `uint16` | CH1 信号值 |
| 4～5 | `signal[2]` | `uint16` | CH2 信号值 |
| 6～7 | `signal[3]` | `uint16` | CH3 信号值 |
| 8～9 | `signal[4]` | `uint16` | CH4 信号值 |
| 10～11 | `signal[5]` | `uint16` | CH5 信号值 |
| 12～13 | `signal[6]` | `uint16` | CH6 信号值 |
| 14～15 | `signal[7]` | `uint16` | CH7 信号值 |

## 4. Type `0x02`：三维力帧

| 项目 | 值 |
| --- | --- |
| `type` | `0x02` |
| `payload_len` | `28` |
| 帧总长度 | `38` 字节 |

帧头：

```text
44 41 54 41 03 02 01 08 1C 00
```

payload：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `vector_count` | `uint8` | 固定为 `2`，依次为指尖和指腹 |
| 1 | `calibration_mask` | `uint8` | bit0 为 1 表示力值已经标定 |
| 2～3 | `component_calibration_mask` | `uint16` | bit0表示指尖法向标定有效；其余分量位见统一协议 |
| 4～7 | `fingertip_normal_force` | `float32` | 指尖法向力；已标定时单位为 N |
| 8～11 | `fingertip_tangential_force` | `float32` | 指尖切向力大小；已标定时单位为 N |
| 12～15 | `fingertip_direction` | `float32` | 指尖切向力方向，单位为弧度 |
| 16～19 | `finger_pad_normal_force` | `float32` | 指腹法向力；已标定时单位为 N |
| 20～23 | `finger_pad_tangential_force` | `float32` | 指腹切向力大小；已标定时单位为 N |
| 24～27 | `finger_pad_direction` | `float32` | 指腹切向力方向，单位为弧度 |

使用指尖法向力前，应检查 `component_calibration_mask.bit0`。该位为1时，MCU每个运行采样周期都汇总CH4～CH7当前Signal，经过Flash法向K表换算和1～20 N整数点临时软吸附后，持续通过 `fingertip_normal_force` 上传牛顿值；FLOATING只表示Signal已进入稳定锁定状态，不再是力值换算的门槛。法向输入低于首个实测K表节点时，MCU在虚拟 `(0 counts, 0 N)` 原点与首节点之间线性插值，因此四通道Signal合计为0时严格输出 `0.0 N`。每个整数点使用左右各 `0.5 N` 的窗口，窗口边界恢复K表原值；它不改变原始Signal和Flash参数，也不参与运动或安全控制。`calibration_mask.bit0`仍表示指尖区域的法向、切向和方向三张表全部有效。

## 5. Type `0x03`：芯片温度帧

| 项目 | 值 |
| --- | --- |
| `type` | `0x03` |
| `channel_count` | `0` |
| `payload_len` | `2` |
| 帧总长度 | `12` 字节 |
| 上报周期 | 约 `1` 秒 |

帧头：

```text
44 41 54 41 03 03 01 00 02 00
```

payload：

| 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0～1 | `temperature` | `int16` | 芯片温度，小端有符号整数，单位为 `0.1 °C` |

例如 `temperature = 253` 表示 `25.3 °C`。

## 6. 接收说明

UART 是连续字节流，一次串口读取可能得到半帧或多帧。接收端应：

1. 搜索同步字 `DATA`。
2. 读取完整的 10 字节帧头。
3. 根据 `payload_len` 接收完整 payload。
4. 按 `type` 解析帧。

正常 Run 输出顺序为：

```text
Type 0x01（26 字节） → Type 0x02（38 字节） → 重复
```

每累计约 1 秒的采样帧后，追加一帧 Type `0x03`（12 字节），随后继续上述 Run 输出。

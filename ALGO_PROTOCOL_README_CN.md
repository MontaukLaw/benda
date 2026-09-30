# 串口通信协议

> 历史资料：本文保留旧版本协议与实验背景，不是当前操作规范。现行UART为V21，沿用V20 Raw布局；统一入口见[当前实现](CURRENT_SYSTEM_CN.md)，命令见[现行协议](UART_PROTOCOL_UNIFIED_V3_CN.md)，地址见[Flash布局](FLASH_LAYOUT_CN.md)。旧帧编号、仿射流程以及0x1FE00/0x1FF00写入地址不得用于现行固件。

本文档描述嵌入式固件与 `PY_SRIPT/algo_monitor` 上位机之间的 UART 数据和控制协议。协议实现位于 `sensor_algorithm.c`、`sensor_algorithm.h` 与 `tuner.c`。

## 传输参数

| 项目 | 当前值 |
| --- | --- |
| 波特率 | `1000000` bps |
| 字节序 | little-endian（小端） |
| 帧同步字 | ASCII `DATA`（`44 41 54 41`） |
| 协议版本 | `2` |
| 默认通道数 | `8` |

数据帧没有校验和、序号或应答字段。接收端应按 `DATA` 同步字查找帧头，并使用 `payload_len` 确定完整帧长度；若数据错位，可丢弃数据直到找到下一组同步字。

## 帧格式

所有设备到上位机的数据帧均为：

```text
+0                 +4       +5     +6            +7       +8          +10
| magic (4 bytes) | version | type | channel_count | reserved | payload_len | payload
|      DATA       |    2    |  1-4 |      N        |    0     | uint16 LE   | ...
```

帧头固定为 10 字节。

| 偏移 | 字段 | 长度 | 说明 |
| ---: | --- | ---: | --- |
| 0 | `magic` | 4 | 固定为 `DATA` |
| 4 | `version` | 1 | 固定为 `2` |
| 5 | `type` | 1 | 帧类型，见下表 |
| 6 | `channel_count` | 1 | 普通通道数据的通道数 `N`；当前为 `8` |
| 7 | `reserved` | 1 | 当前固定为 `0` |
| 8 | `payload_len` | 2 | payload 字节数，`uint16` 小端 |
| 10 | `payload` | 可变 | 按帧类型解释 |

## 帧类型与 payload

| type | 名称 | payload 长度 | payload 内容 |
| ---: | --- | ---: | --- |
| `1` | Signal | `N × 2` | `signal[0..N-1]`，每项 `uint16` |
| `2` | Debug | `N × 5 × 2` | `raw[] + baseline[] + signal[] + noise[] + state[]`，每项 `uint16` |
| `3` | Fingertip force vector | `56` | 指尖区域的 14 个 `float32` |
| `4` | Finger-pad force vector | `56` | 指腹区域的 14 个 `float32` |

### Type 1：Signal

当 `SENSOR_ALGO_DEBUG_OUTPUT == 0` 时，普通通道帧使用 type `1`：

```text
payload = signal[0], signal[1], ..., signal[N - 1]
```

每个 `signal[i]` 为小端 `uint16_t`，代表经算法处理和滤波后的正向信号。

### Type 2：Debug

当 `SENSOR_ALGO_DEBUG_OUTPUT == 1` 时，普通通道帧使用 type `2`：

```text
payload = raw[0..N-1]
        + baseline[0..N-1]
        + signal[0..N-1]
        + noise[0..N-1]
        + state[0..N-1]
```

全部元素均为小端 `uint16_t`。字段含义如下：

- `raw`：CapSense 原始计数。
- `baseline`：当前基线计数。
- `signal`：`max(0, raw - baseline)` 经算法处理后的输出。
- `noise`：校准得到的噪声估计。
- `state`：通道状态。

| `state` | 名称 | 含义 |
| ---: | --- | --- |
| `0` | `IDLE` | 空闲 |
| `1` | `WEAK_ACTIVE` | 弱触发 |
| `2` | `RISING_ACTIVE` | 上升触发 |
| `3` | `ACTIVE` | 稳定触发 |
| `4` | `FALLING_ACTIVE` | 下降/释放 |

### Type 3 与 Type 4：力向量

当 `SENSOR_ALGO_FORCE_VECTOR_OUTPUT == 1` 时，固件会发送两帧力向量。两种类型的 payload 格式完全相同，均为 14 个小端 IEEE-754 `float32`，共 56 字节：

```text
theta, tangential_force, confidence, normal_force,
cop_x, cop_y, diff_x, diff_y, force_x, force_y,
p00, p01, p10, p11
```

| 字段 | 含义 |
| --- | --- |
| `theta` | 切向力方向角，单位为弧度 |
| `tangential_force` | 切向力大小 |
| `confidence` | 方向/力向量可信度 |
| `normal_force` | 2×2 区域的总法向力 |
| `cop_x`、`cop_y` | 归一化压心坐标 |
| `diff_x`、`diff_y` | 归一化前的左右、上下压力差 |
| `force_x`、`force_y` | 滤波后的切向力代理分量 |
| `p00`、`p01`、`p10`、`p11` | 参与该 2×2 计算的四个输入值 |

在当前 8 通道布局中，输入通道映射为：

| 帧类型 | 区域 | `p00` | `p01` | `p10` | `p11` |
| ---: | --- | --- | --- | --- | --- |
| `3` | 指尖 | CH6 | CH7 | CH4 | CH5 |
| `4` | 指腹 | CH1 | CH0 | CH3 | CH2 |

## 当前默认发送次序

当前默认配置为 `SENSOR_ALGO_DEBUG_OUTPUT = 1`、`SENSOR_ALGO_FORCE_VECTOR_OUTPUT = 1`。每完成一次传感器扫描，固件依次发送：

```text
type 2（8 通道 Debug） -> type 3（指尖力向量） -> type 4（指腹力向量）
```

关闭调试输出后，第一个帧会变为 type `1`；关闭力向量输出后，仅发送该普通通道帧。接收端不应依赖固定总长度或固定帧组，而应独立解析每一帧的帧头和长度。

## 主机到设备命令

### 重新校准

直接发送 5 个 ASCII 字节，不带帧头、换行或校验：

```text
RECAL
```

十六进制为 `52 45 43 41 4C`。固件在接收字节流中匹配该连续序列后调用 `SensorAlgorithm_RequestRecalibration()`，使全部通道重新进入 baseline/noise 校准流程。该命令没有串口确认帧。

### DFU 重启

固件还保留 CapSense Tuner 命令包中的 DFU 重启检测：当命令包的命令码和计数器字段分别为 `0xAB` 与 `0xCD` 时，设备进入 DFU 应用。该命令包使用 CapSense Tuner 的既有定义，本文档不定义其完整包格式；上位机仅进行算法监测或重新校准时无需发送它。

## 示例

3 通道 Signal 帧，`signal = [11, 22, 33]`：

```text
44 41 54 41  02 01 03 00  06 00  0B 00 16 00 21 00
 D  A  T  A  ver type  N rsv  len       payload
```

该帧总长度为 `10 + 6 = 16` 字节。

## 上位机实现参考

Python 解析器位于 `PY_SRIPT/algo_monitor/frame_parser.py`。它按 `<4sBBBBH` 解析帧头，按 `<H` 解析普通通道值，按 `<14f` 解析力向量。`CHANNEL_COUNT = 8` 只是 UI 启动默认值，实际解析以帧头的 `channel_count` 为准。

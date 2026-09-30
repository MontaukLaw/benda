# 统一 UART 协议 v3

> 历史提案，不是当前已实现协议。V21现状、帧格式和Flash边界见[当前实现](CURRENT_SYSTEM_CN.md)、[现行UART协议](UART_PROTOCOL_UNIFIED_V3_CN.md)及[Flash布局](FLASH_LAYOUT_CN.md)。下文版本3和示例只供追溯，不用于当前设备收发。

## 协议参数

| 项目 | 约定 |
| --- | --- |
| 波特率 | `1000000` bps |
| 字节序 | 小端（little-endian） |
| 同步字 | ASCII `DATA`（`44 41 54 41`） |
| 协议版本 | `3` |
| 上行帧顺序 | Type 1 通道信号 → Type 2 三维力 |
| 对外数据范围 | 通道信号与三维力 |

主机在字节流中搜索 `DATA`，读取固定 10 字节帧头后，再按 `payload_len` 读取完整 payload。

## 设备标识

| `device` | 设备版本 | `channel_count` | 三维力数量 |
| ---: | --- | ---: | ---: |
| `1` | 中三指 | `8` | `2` |
| `2` | 拇指/尾指 | `5` | `1` |
| `0` | 广播命令 | `0` | 不适用 |

## 公共帧头

所有帧均使用以下固定 10 字节帧头：

```text
DATA[4] | version[1] | type[1] | device[1] | channel_count[1] | payload_len[2]
```

| 字节偏移 | 字段 | 类型 | 固定值/说明 |
| ---: | --- | --- | --- |
| 0–3 | `magic` | ASCII[4] | 固定为 `DATA` |
| 4 | `version` | `uint8` | 固定为 `3` |
| 5 | `type` | `uint8` | 帧类型，见下表 |
| 6 | `device` | `uint8` | 设备标识 |
| 7 | `channel_count` | `uint8` | 通道数量 |
| 8–9 | `payload_len` | 小端 `uint16` | 后续 payload 的字节长度 |

## Type 1：通道信号帧

用于上报每个通道的最终信号值。

| 帧头字段 | 值 |
| --- | --- |
| `type` | `1` |
| `device` | `1` 或 `2` |
| `channel_count` | 对应设备的 `8` 或 `5` |
| `payload_len` | `channel_count × 2` |

| payload 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| `i × 2` | `signal[i]` | 小端 `uint16` | 第 `i` 个通道的处理后信号值，`i = 0 .. channel_count - 1` |

## Type 2：三维力帧

帧内固定保留两组三维力槽位，每组由**法向力、切向力、方向**组成。方向的单位为弧度。

| 帧头字段 | 值 |
| --- | --- |
| `type` | `2` |
| `device` | `1` 或 `2` |
| `channel_count` | 对应设备的 `8` 或 `5` |
| `payload_len` | `28` |

| payload 偏移 | 字段 | 类型 | 说明 |
| ---: | --- | --- | --- |
| 0 | `vector_count` | `uint8` | 有效三维力数量：`1` 或 `2` |
| 1–3 | `reserved` | 3 字节 | 固定为 `0` |
| 4–7 | `force1_normal` | 小端 `float32` | 第一组法向力 |
| 8–11 | `force1_tangential` | 小端 `float32` | 第一组切向力 |
| 12–15 | `force1_direction` | 小端 `float32` | 第一组方向（弧度） |
| 16–19 | `force2_normal` | 小端 `float32` | 第二组法向力 |
| 20–23 | `force2_tangential` | 小端 `float32` | 第二组切向力 |
| 24–27 | `force2_direction` | 小端 `float32` | 第二组方向（弧度） |

| 设备版本 | `vector_count` | 第一组三维力 | 第二组三维力 |
| --- | ---: | --- | --- |
| 中指/无名指 | `2` | 指尖 | 指腹 |
| 拇指/尾指 | `1` | 主受力区 | 固定填 `0.0` |

## Type 0x80：重新校准命令

主机发送该帧后，设备重新校准全部通道。

| 字段 | 值 |
| --- | --- |
| `type` | `0x80` |
| `device` | `0`（广播）或指定设备标识 |
| `channel_count` | `0` |
| `payload_len` | `0` |

## Type 0x81：命令确认帧

| 字段 | 值 |
| --- | --- |
| `type` | `0x81` |
| `payload_len` | `1` |
| payload[0]：`result` | `0` 表示成功；非 `0` 表示失败 |

## 发送流程

```text
每次传感器采样
  ├─ Type 1：通道信号帧
  └─ Type 2：三维力帧
```

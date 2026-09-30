# Algorithm UART Protocol

> 历史资料：本文保留旧版本协议与实验背景，不是当前操作规范。现行UART为V21，沿用V20 Raw布局；统一入口见[当前实现](CURRENT_SYSTEM_CN.md)，命令见[现行协议](UART_PROTOCOL_UNIFIED_V3_CN.md)，地址见[Flash布局](FLASH_LAYOUT_CN.md)。旧帧编号、仿射流程以及0x1FE00/0x1FF00写入地址不得用于现行固件。

This document describes the UART protocol between the embedded firmware and
the Python `algo_monitor` tool.

## Transport

- UART baud rate: `1000000`
- Byte order: little-endian
- Numeric payload values: unsigned 16-bit integers (`uint16_t`)

## Data Frame

All sensor data frames use a self-describing header:

```text
magic[4] version[1] type[1] channel_count[1] reserved[1] payload_len[2] payload[payload_len]
```

Header fields:

| Field | Size | Value | Description |
| --- | ---: | --- | --- |
| `magic` | 4 | `DATA` | Frame sync word |
| `version` | 1 | `2` | Protocol version |
| `type` | 1 | `1`, `2`, or `3` | Payload type |
| `channel_count` | 1 | `1..255` | Number of sensor channels in this frame |
| `reserved` | 1 | `0` | Reserved for future use |
| `payload_len` | 2 | bytes | Payload length after the header |

`payload_len` is little-endian `uint16_t`.

## Frame Types

### Type 1: Release Signal Frame

Used when `SENSOR_ALGO_DEBUG_OUTPUT == 0`.

```text
type = 1
payload_len = channel_count * 2
payload = signal[channel_count]
```

Payload layout:

```text
signal[0], signal[1], ... signal[channel_count - 1]
```

Each `signal[i]` is a little-endian `uint16_t`.

### Type 2: Debug Full Frame

Used when `SENSOR_ALGO_DEBUG_OUTPUT == 1`.

```text
type = 2
payload_len = channel_count * 5 * 2
payload = raw[] + baseline[] + signal[] + noise[] + state[]
```

Payload layout:

```text
raw[0..N-1]
baseline[0..N-1]
signal[0..N-1]
noise[0..N-1]
state[0..N-1]
```

Where `N = channel_count`.

Each value is a little-endian `uint16_t`.

State values:

| Value | Name |
| ---: | --- |
| `0` | `IDLE` |
| `1` | `WEAK_ACTIVE` |
| `2` | `RISING_ACTIVE` |
| `3` | `ACTIVE` |
| `4` | `FALLING_ACTIVE` |

### Type 3: Force Vector Frame

Used when `SENSOR_ALGO_FORCE_VECTOR_OUTPUT == 1`.

This frame is sent after the normal type `1` or type `2` data frame.

```text
type = 3
payload_len = 56
payload = theta + tangential_force + confidence + normal_force + cop_x + cop_y + diff_x + diff_y + force_x + force_y + p00 + p01 + p10 + p11
```

Payload layout:

```text
theta[float32]
tangential_force[float32]
confidence[float32]
normal_force[float32]
cop_x[float32]
cop_y[float32]
diff_x[float32]
diff_y[float32]
force_x[float32]
force_y[float32]
p00[float32]
p01[float32]
p10[float32]
p11[float32]
```

Each value is a little-endian IEEE-754 32-bit float.

Field meanings:

- `normal_force`: total normal load from the 2x2 force patch.
- `cop_x`, `cop_y`: normalized center-of-pressure coordinates.
- `diff_x`, `diff_y`: left-right and top-bottom pressure differences before normalization.
- `force_x`, `force_y`: filtered tangential-force proxy components used to derive `theta`.
- `p00`, `p01`, `p10`, `p11`: the four sensor values used in the 2x2 force calculation.

The current firmware calculates the force vector from this input order:

```c
force_vector_cal(
    outputs[5].signal,
    outputs[4].signal,
    outputs[7].signal,
    outputs[6].signal);
```

## Firmware Mode Switch

The firmware mode is controlled by this macro in `sensor_algorithm.h`:

```c
#ifndef SENSOR_ALGO_DEBUG_OUTPUT
#define SENSOR_ALGO_DEBUG_OUTPUT (1u)
#endif
```

- `1`: debug mode, sends type `2` frames with all channel data.
- `0`: release mode, sends type `1` frames with signal only.

Tangential force output is controlled by:

```c
#ifndef SENSOR_ALGO_FORCE_VECTOR_OUTPUT
#define SENSOR_ALGO_FORCE_VECTOR_OUTPUT (0u)
#endif
```

- `1`: calculate force vector and send a type `3` frame after each normal data frame.
- `0`: disable force-vector calculation and transmission.

Example release build override:

```sh
make build DEFINES='CY_APP_ID=1 SENSOR_ALGO_DEBUG_OUTPUT=0'
```

## Recalibration Command

The Python tool can request a full-channel recalibration by sending this ASCII
command to the embedded device:

```text
RECAL
```

The command is exactly 5 bytes:

```text
0x52 0x45 0x43 0x41 0x4C
```

When the firmware receives this byte sequence, it calls:

```c
SensorAlgorithm_RequestRecalibration();
```

This re-enters global calibration for every channel.

## Python Compatibility

`PY_SRIPT/algo_monitor` parses `channel_count` from every `DATA` frame. The
initial `CHANNEL_COUNT = 8` in `config.py` is only the startup default UI size.
If the firmware sends `channel_count = 5`, the Python monitor automatically
parses and displays 5 channels.

## Example Frames

### Release, 3 Channels

Signals:

```text
11, 22, 33
```

Frame:

```text
44 41 54 41  02 01 03 00  06 00  0B 00 16 00 21 00
 D  A  T  A  ver type ch rsv len   signal payload
```

### Debug, 2 Channels

Payload values:

```text
raw:      1000, 1001
baseline: 900, 901
signal:   100, 101
noise:    3, 4
state:    0, 3
```

Payload order:

```text
1000, 1001, 900, 901, 100, 101, 3, 4, 0, 3
```

Frame header:

```text
magic = DATA
version = 2
type = 2
channel_count = 2
payload_len = 20
```

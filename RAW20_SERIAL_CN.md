# 4×5阵列纯Raw采集

源码核对日期：2026-10-05。RAW20是显式指定`RAW_ONLY=1`才启用的独立采集构建，入口是`raw_stream.c`，仅支持`INDEX_FINGER`和UART。
当前Makefile默认`RAW_ONLY=0`，使用`main.c`的[V21协议](UART_PROTOCOL_UNIFIED_V3_CN.md)，其纯Raw采样是`DATA + Type 0x16`、20通道总长50字节，不是本文46字节协议。
使用当前 BSP 中 Configurator 生成的单个 MatrixButtons0：4个TX、5个RX，共20点。
每次完整扫描后直接读取 `cy_capsense_tuner.sensorContext[0..19].raw`。

运行期间不调用 `Cy_CapSense_ProcessAllWidgets()`，不更新baseline，不做软件滤波、Signal、压力计算、状态机、自动量程、温度采样或Flash参数加载。接收端仅保留原有 `AB CD` 软件复位升级指令，其余旧协议命令不处理。
保留标准 `Cy_CapSense_Init/Enable` 和扫描中断；`Enable` 内部的一次基线/滤波器初始化仍由库完成，后续发送不依赖这些值。
三参数采用Configurator配置，不再被旧代码的固定参数覆盖。
一次扫描完成并发送结束后才启动下一次扫描，因此每帧20点来自同一轮扫描；帧率由配置的扫描时间和串口发送时间共同决定。

## 运行接收脚本

在项目根目录打开命令行：

```powershell
py -3 -m pip install pyserial
py -3 PY_SRIPT/raw20_serial.py --list-ports
py -3 PY_SRIPT/raw20_serial.py COM3
```

将 `COM3` 替换为实际串口。默认 **1000000波特率、8N1、无流控**，沿用原板UART引脚。
每行只打印20个十进制raw值，按以下顺序排列：

```text
RX0/TX0 RX0/TX1 RX0/TX2 RX0/TX3 RX1/TX0 ... RX4/TX0 RX4/TX1 RX4/TX2 RX4/TX3
```

这是扫描索引顺序；物理阵列方向由实际PCB布线决定。按 `Ctrl+C` 关闭串口并退出。
脚本仅接收本文46字节帧，无需握手或启动命令，不导入原GUI程序。原`algo_monitor` GUI只解析`DATA`协议，不能接收RAW20；本脚本也不能接收V21的`0x16`帧。`--baud`可更改接收速率，但必须与固件一致。
连接状态和错误输出到stderr，数据行输出到stdout。打开后无数据时持续等待，不打印虚假零值。

## 固定46字节协议

| 字节偏移 | 内容 |
| --- | --- |
| 0～3 | 固定帧头 `AA 55 AA 55` |
| 4～43 | 20个无符号16位raw，每个低字节在前 |
| 44～45 | 40字节载荷的字节累加和，uint16小端 |

无帧类型、时间戳、序号或尾标记。校验用于识别常见串口错位和损坏，不保证检测所有错误。
脚本支持半帧、连续帧以及校验失败后重新搜索帧头。

## 构建及恢复

ModusToolbox环境中执行 `make build FINGER_TYPE=INDEX_FINGER TRANSPORT_OPT=UART RAW_ONLY=1`。
本机PowerShell也可直接使用：

```powershell
$env:PATH = 'C:/Infineon/Tools/ModusToolbox/tools_3.7/modus-shell/bin;' + $env:PATH
& 'C:/Infineon/Tools/ModusToolbox/tools_3.7/modus-shell/bin/make.exe' build 'CY_TOOLS_PATHS=C:/Infineon/Tools/ModusToolbox/tools_3.7' 'FINGER_TYPE=INDEX_FINGER' 'TRANSPORT_OPT=UART' 'RAW_ONLY=1'
```

产物位于 `build/INDEX_FINGER/UART/APP_CY8CPROTO-041TP/Debug/`：

- `mtb-example-psoc4-raw20.hex`
- `mtb-example-psoc4-raw20_crc.cyacd2`

这是沿用现有Flash布局的应用固件，应用起始地址为 `0x3600`；HEX不包含Bootloader。
RAW20通过UART接收中断识别连续 `AB CD`，写入原有IAP请求，再调用 `Cy_DFU_ExecuteApp(0u)` 选择Bootloader并软件复位，兼容原DFU工具的升级入口。不能只调用 `NVIC_SystemReset()`：DFU库在保留RAM中记录的App1编号会使软件复位直接重进应用。扫描、发送和CapSense异常停采期间均可响应升级指令；发送出的raw数据不会进入本机接收解析。
首版RAW20未保留此入口，因此刷入首版后不能靠串口命令再次升级。后续仅补回接收但仍使用普通软件复位的版本，也会因启动编号保持App1而回到应用。恢复时可通过SWD重新写入本次修复后的应用；调试器恢复DFU需要同时将DFU的保留应用编号切换为App0，再执行软件复位，不能只复位。单纯断电重启不保证进入DFU：当前Bootloader源码会在非软件复位且应用有效时直接启动应用。
进行DFU前先关闭占用同一COM口的raw接收脚本或串口工具；串口参数使用1000000、8N1。
本次仅更新应用产物，未修改Bootloader，亦未执行烧录。

算法源码保留但从RAW20构建中排除。`RAW_ONLY=0`（当前默认）回到`main.c`的20通道V21入口，当前20点BSP可直接用于该构建，无须恢复旧8点配置。默认Baseline宏为0，上传Type `0x16`；设为1恢复通道Signal/Debug，仍不恢复旧四通道压力输出。
THUMB/PINKY默认保持V21构建。RAW20仅支持当前20点BSP与UART，错误组合在构建阶段明确拒绝。
同一构建目录可能保留旧文件，请使用以上带 `raw20` 名称的产物。

离线协议验证：`py -3 -m unittest discover -s tools -p test_raw20_serial.py -v`。

# CAN 收发与传感器丢字节实机对照

2026-09-17 16:30 开始，上位机关闭，电机静止，COM4 以 1,000,000 baud 持续接收。只读电机系统状态 0x43 和温度 0x39，未发送运动、使能、清零或传感器配置命令。原始串口数据先存内存，全部采集结束后才解析和写盘。

| 每阶段约 20 秒 | 传感器 debug 帧 | 状态非法帧 | 串口 CE_OVERRUN 次数 | 成功读取电机状态 |
|---|---:|---:|---:|---:|
| CAN 关闭 | 2896 | 0 | 0 | 0 |
| CAN 打开，不发送查询 | 2897 | 0 | 0 | 0 |
| CAN 在同进程独立线程查询 | 2896 | 38 | 82 | 199 |
| CAN 在独立进程查询 | 2901 | 35 | 82 | 196 |
| CAN 再次关闭 | 2897 | 0 | 0 | 0 |

错误标志全部为 0x0002（CE_OVERRUN），没有 CE_RXOVER、CE_FRAME 或 CE_RXPARITY。Microsoft 对 CE_OVERRUN 的定义是字符缓冲区溢出、丢失字符；它和上位机输入缓冲区满的 CE_RXOVER 是不同标志：
https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-clearcommerror

线程阶段 Windows 接收队列最大仅 95 字节，接收循环处理间隔最大 0.282 ms；独立进程阶段最大队列 1063 字节、处理间隔最大 1.309 ms。故障在没有 GUI、没有实时解析、没有采集期文件写入，并且 CAN 独立进程运行时仍然复现。原始字节中可以找到 debug 帧提前进入下一 run_force 帧的现象，和此前快照吻合。

已确认：CAN 主动查询期间，CH340 接收链路报告溢出并发生字节丢失；原解析器随后按声明长度吞入下一帧，制造 Signal 尖刺。未发现 CAN 字节被送入传感器解析器。拆分 Python 线程/进程不能消除此故障。

Windows PnP 拓扑显示：CH340 位于 USBROOT(0)/USB(2)/USB(4)；candleLight 位于同一 USB(2) Hub 下的 USB(1)/USB(1)。共享祖先为 USB VID_05E3 PID_0610 Hub。

后续用户实机复测确认：“把can换到单独的usb口，直插pc，就没有问题了”。结合上述原始采集，触发条件已定位到原来的共享 USB Hub 接入链路：CAN 查询时 CH340 接收溢出、丢字节，解析器把下一帧内容拼入本帧，造成尖刺。处理方式为保留 USB-CAN 直插 PC 的接法，避免与高速传感器串口共用原来的 Hub 链路。

换口后的恢复由用户观察确认，本目录原始数据仅来自换口前的五阶段采集；没有把它描述成换口后的零错误统计。此结果不区分 Hub 调度、驱动交互等更底层机制，也不等于证明 Hub 硬件损坏。CAN 关闭清理警告是另一个已修复的软件问题。

各子目录保留 serial.bin（解析前原始数据）、chunks.json（每次读取的时间和字节偏移）、can_events.json（CAN 查询时间）及 report.json（错误、帧统计和 SHA256）。summary.json 为全部阶段汇总。诊断脚本：tools/diagnose_can_serial.py。

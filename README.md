# 多通道压力传感器与电机标定工具

本工程包含PSoC固件、传感器协议及Python监控上位机。**当前功能和文档入口见[CURRENT_SYSTEM_CN.md](CURRENT_SYSTEM_CN.md)**，核对日期为2026-09-22；源码版本不代表板上已烧录的版本。

## 当前使用流程

`one_step`分支的“One Step学习”Tab现使用**逐轮连续轨迹学习**：终点、空行程速度、减速起点、受力速度及加速/减速档位共同调整，运动中保持绝对终点并变速，中间不停车。目标1秒内到位且压力峰值和观察窗满足原容差；每轮记录差距与下一轮调整，禁止重复失败参数，成功即记录，每点最多20轮。接触位置可输入并勾选直接使用，跳过10次搜索；空载确认、提前接触保护及异常释放保留。新表版本3逐轮写入`one_step_cycles/`，30个按钮复用逐点轨迹；旧记录兼容。详见[One Step学习与测试说明](ONE_STEP_LEARNING_CN.md)。

真实峰值与稳态欠压冲突时降低受力速度并调整终点/减速参数；压力合格而运动超时则优化速度和加速。容差不会自动放宽。当前仅离线验证，实际1秒表现待实机确认。

1. 运行源码上位机，连接传感器、压力计及所选电机。
2. 在“压力反馈控制”确认释放点，完成接触点及0.05～30 N学习。已保存的完整学习表可在重启后通过新鲜卸载反馈核验恢复，无须仅因重启重新学习。
3. 监控页“按力下压”共用压力反馈页的参数、学习表和闭环逻辑；旧“电机位置/压力标定”Tab已隐藏。
4. 在监控页可折叠的“自动压力标定”设置10/20整数点、正式轮数、点间休息（默认30秒）和验收误差（默认±2%，至少±0.05 N）。先30 N热身至少5次：到位后恒力5秒、采样确认、释放后休息10秒；未收敛最多15次。随后正式标定，再针对历史/本次误差大的点各补测3次并独立全点验收，最多3批。热身休息不可跳过，正式/复测休息仍可跳过。显示阶段、逐轮耗时和验收误差。
5. 只有验收通过才更新“传感器标定K值表”，失败先释放并保留原表与全部记录。节点使用同一次按压的Signal/实测压力配对。核对后手动写入MCU Flash；新10/20点表要求设备协议至少V21。1 N以下沿用独立的微力手动四点参数。

电机学习表是电脑上的压力—行程参考；传感器K表是Signal—压力映射，两者不同。K表页面启动读取电脑latest，不能据此判断MCU已保存。绿色“读取最新标定数据并生成K值表”目前仍读取旧48点记录，新流程完成后的自动同步不需要该按钮，详见[自动标定说明](HIGH_FORCE_CALIBRATION_CN.md)。

中断或重启后，自动标定模块会显示最近未完成记录与下一压力点。保持装夹、材料和三参数不变，连接设备并等待释放点/学习表恢复后，点击“继续标定”可保留已完成采样、接续剩余内容。独立验收重采完整新批次，续跑另存记录，每轮耗时累计且排除关程序的间隔。详情见[续跑规则](HIGH_FORCE_CALIBRATION_CN.md#继续未完成的标定)。

自动标定先连续确认停止状态；仍异常时回原释放点，保留进度等待恢复后重试当前点，不再因失败3次结束整场。从第3次失败起增加30秒不可跳过的恢复冷却，反馈/卸载恢复后再按原配置休息、重新按压。“停止并释放”保留自动回位，“紧急停止”取消包括回位在内的运动；无可信反馈、故障、过流或坐标失效时不盲目驱动。详细规则见[自动标定说明](HIGH_FORCE_CALIBRATION_CN.md)。

## 源码运行与目录

安装上位机所需的Python依赖后，从仓库根目录运行：

```powershell
python PY_SRIPT/algo_monitor/main.py
```

| 位置 | 用途 |
|---|---|
| `PY_SRIPT/algo_monitor/` | 当前GUI、解析器、电机控制与标定 |
| `sensor_algorithm.c/.h` | 传感器数据处理和UART协议 |
| `channel_state_machine.c/.h` | 真实通道状态判定与唯一运行时提交入口 |
| `sensor_algorithm.c`、`low_force_calibration.c/.h` | FCV9分段力表与独立微力表的换算、协议及Flash记录 |
| `app_flash_layout.h`、`app_flash.c` | Flash边界及擦写保护 |
| `tools/` | 离线验证工具 |

快照分析优先使用`PY_SRIPT/algo_monitor/recordings/recent_1min.csv`、`recent_3min.csv`或`recent_5min.csv`及同名`.meta.json`。它们是手动转存的固定历史快照，并非持续更新的实时文件；须核对导出时间、采样范围、间断及SHA256。

## 协议与配置

当前算法UART为**V21**，沿用V20单一硬件Raw布局，上位机兼容V14～V21。完整帧与命令以[UART协议](UART_PROTOCOL_UNIFIED_V3_CN.md)为准；文件名中的V3是历史命名。I²C/SensorBus使用[独立协议](SENSOR_BUS_PROTOCOL_CN.md)，其版本号不跟随UART递增。`ALGO_PROTOCOL_README*.md`和`UART_PROTOCOL_CUSTOMER_CN.md`只供历史参考。

INDEX_FINGER启动三参数为`Nsub=300、TxDivider=100、RefCDAC=30`。这里“三参数”固定指这三项；手动修改仅在RAM。THUMB/PINKY硬件配置与食指不同，见[固件变体](FIRMWARE_VARIANTS_CN.md)。通道状态只能由`channel_state_machine.c`中的`commit_state()`提交，其他模块消费真实状态。

## Flash与构建

应用区为`0x03600～0x1F4FF`；微力表位于`0x1F500`，FCV9两个region位于`0x1F600～0x1FBFF`，停用仿射预留`0x1FC00`，`0x1FD00～0x1FFFF`禁止应用擦写。新10/20点沿用原FCV9记录和容量，不另开一套地址。

**相邻PSOC4_Bootloader工程仍存在App1长度不一致：其范围至0x1FBFF，覆盖现行参数区。** 本轮未修改或烧录Bootloader，也未确认板上版本。地址、擦写粒度、断电限制和跨工程差异统一见[Flash布局](FLASH_LAYOUT_CN.md)，不能从“应用内部无重叠”推断整个升级链路已一致。

在配置好ModusToolbox的环境中，按实际设备构建，例如：

```powershell
make build FINGER_TYPE=INDEX_FINGER TRANSPORT_OPT=UART CONFIG=Debug
```

也可使用`build_firmware.ps1`，变体选择、输出路径和验证要求见[固件构建说明](FIRMWARE_VARIANTS_CN.md)。修改固件后应更新HEX/CYACD2；构建成功不代表已经烧录或通过实机试验。

上位机打包入口为`PY_SRIPT/algo_monitor/build_exe.bat`。只有用户在当前请求中明确要求打包EXE时才执行；源码修改、测试或文档维护不隐含打包授权。

## 文档维护

当前功能以[文档总入口](CURRENT_SYSTEM_CN.md)所列现行文档为准。历史协议、旧功能说明和带日期的实验报告保留当时数据，并标明适用范围；不要把历史“当前”值抄回现行配置。具体变更按日期记录在[CHANGE.md](CHANGE.md)。

# 共享电机驱动接口与旧位置扫描格式

> 旧“电机位置/压力标定”Tab现已隐藏；本文驱动接口仍服务于当前共享电机后端，旧扫描说明不代表可见操作入口。当前按压与学习见[压力反馈控制](../PRESSURE_FEEDBACK_CONTROL_CN.md)，K表采集见[自动标定](../HIGH_FORCE_CALIBRATION_CN.md)。下述`force_position_mapping_*.json`及`force_position_maps/`属于旧位置扫描数据，不是当前`pressure_feedback_config_<namespace>.learning.json`学习表。

标定页通过 `motor_mapping_interface.py` 调用电机，由 `motor_connection_config.json` 的 `motor_model` 选择驱动，修改后重启上位机。支持 `zdt_x42s`（当前实机使用的 Emm 固件）和 `pd42s1`。配置错误时停用通信，不自动连接另一种电机。

```json
{
  "motor_model": "zdt_x42s",
  "zdt_x42s": {
    "interface": "gs_usb",
    "channel": 0,
    "bitrate": 500000,
    "node_id": 1,
    "timeout": 0.2,
    "temperature_positive_sign": 0,
    "steps_per_turn": 3200
  }
}
```

`steps_per_turn` 必须与实机步距角、指令细分一致：1.8°、16细分为3200 step/圈。此参数用于反馈角度与step换算，不修改电机细分。Emm速度缩小10倍功能应关闭。新电机测试页与标定页共用原有CAN连接、唯一后台线程、互斥任务和停止事件。

## 界面与单位

- 位置、步长、行程和极差显示为step；1 step为一个细分后的指令脉冲，不是编码器计数。ZDT反馈使用 `position_deg × steps_per_turn / 360` 换算，发送运动指令直接使用整数step。
- ZDT速度使用RPM，加速度使用协议的0～255档。PD42S1保留其速度档位和r/s²加速度，通过适配器调用原驱动；其逻辑位置单位为51200 step/圈。
- ZDT默认下压32 step、10 RPM，释放24 RPM；默认最大行程按旧机械转数换算为6144 step。实际行程和压力上限应由操作员按机构设置。
- 压力位置标定页允许扫描到30 N，绝对压力保护上限为35 N：达到35 N禁止继续下压，运动中达到35 N立即发出停止指令。每轮达到设定扫描上限后正常结束并释放。
- ZDT Emm的“最大电流”是堵转最大电流，正常运行相电流可能超过此值。设置不保存至驱动器非易失存储。

## 将当前位置设为0

电机静止、状态新鲜且无其他任务时，点击“当前位置设为0（角度清零）”。协议依据厂家《ZDT_X42S第二代闭环步进电机用户手册V1.0.5》第48页5.2.3：逻辑命令 `Addr 0A 6D 6B`，成功应答 `Addr 0A 02 6B`。地址1经CAN发送扩展ID `0x100`、数据 `0A 6D 6B`。

该命令将当前坐标改为0，不寻找机械原点。收到应答后读取位置确认归零；超时不会重发，因为驱动器可能已执行。清零尝试会作废释放位置、旧查找表及压力反馈学习表；随后先确认释放位置，再重新学习。传感器K表与电机坐标是不同数据。碰撞回零、编码器校准同样会撤销旧坐标资格。旧扫描归档文件保留。

## 数据与接口

旧位置扫描功能的驱动配置保存为 `force_position_mapping_zdt_x42s_<地址>_<每圈步数>.json`；查找表及CSV保存在 `force_position_maps/zdt_x42s_<地址>_<每圈步数>/`。新JSON的schema_version为3，包含 `motor_model`、`position_unit=step` 和 `steps_per_turn`，位置字段使用 `*_steps`，下压步长为 `press_step`，速度为 `*_speed_rpm`，加速度为 `acceleration_level`。

读取时校验驱动及单位；旧PD42S1查找表不会被新电机加载或覆盖。为兼容已有插值和查表算法，内存中历史 `*_counts` 键名仍保留，其数值单位由所选接口统一为step；文件读写边界负责字段转换。

`MappingMotor` 的可调用接口包括 `read_position_steps()`、`read_feedback()`、`move_relative_steps()`、`prepare_motion()`、`set_current_limit_ma()` 和 `stop()`。新增驱动时实现适配器与 `MappingProfile`，在 `create_mapping_control()` 注册；界面和扫描算法无需新增厂商协议分支。任务必须提交至所选连接的后台服务，禁止另开CAN连接。

## 验证

`tools/test_mapping_motor_interface.py` 覆盖CAN清零/电流报文、清零超时不重发、旧坐标作废、步数换算、两轮扫描、JSON单位隔离、压力超限阻止下压和任务停止。结合 `test_zdt_motor.py`、`test_motor_selection.py`、`test_force_position_mapping_zero.py` 检查既有驱动和零点等待行为。均为模拟通信验证，实际机构动作需在实机验证。

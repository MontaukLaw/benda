# 4×5 压力阵列触摸调试应用

本应用根据同目录 `prompt.txt` 实现，使用 PySide6。源码位于仓库实际已有的
`PY_SRIPT/new_all_on_python`（仓库目录名为 `PY_SRIPT`）。保留现有监控程序，
复用 `algo_monitor/frame_parser.py` 及 `raw20_serial.py` 的解析逻辑。

## 启动

在仓库根目录执行：

```powershell
py -3.14 -m pip install -r PY_SRIPT/new_all_on_python/requirements.txt
py -3.14 PY_SRIPT/new_all_on_python/main.py
```

也可以从其他目录使用 `main.py` 的绝对路径启动。应用使用 Python 3.10 以上语法，
本机验证环境为 Python 3.14。源码启动不需要打包 EXE。

没有硬件时：

```powershell
py -3.14 PY_SRIPT/new_all_on_python/main.py --simulate
```

等待空载基线就绪后，鼠标左键按住阵列拖动，松开模拟抬手。模拟采样为100Hz；
可勾选“模拟慢回弹”观察残留影响。模式切换需先 Stop，防止模拟数据混入串口采样。

## 串口接入

1. 保持阵列释放/空载，关闭占用同一串口的旧监控程序。
2. 正常启动自动依次尝试 COM4、COM3，使用当前配置的协议和波特率
   （默认DATA、1000000，8N1，无流控）。打开失败或2秒内没有有效20通道Raw时，
   先关闭当前端口，再尝试下一候选；收到有效Raw即保留连接。
3. 自动连接后等待至少50帧且持续至少0.5秒的稳定空载数据建立本地 baseline。
   若两个候选均失败，界面显示原因，可手动选择其他 COM Port 后点击 Connect。
   自动尝试只在启动时进行一轮；普通Stop/关闭会取消待执行的回退，断开后不会自行重连。
   `--simulate` 启动跳过串口自动连接。
4. 若初始基线在按压下建立，释放后点击 Reset Baseline。该按钮重新采集稳定窗口，
   不会把当前单帧直接设成基线。稳定窗口允许波动默认80 counts，可以在 Signal 页调节。

| Protocol | 设备输出 | 接入方式 |
|---|---|---|
| DATA（默认） | V21 Type `0x16` 纯Raw；V14～V21 Type `0x10` Debug | 复用 `AlgoFrameParser` |
| RAW20 | `RAW_ONLY=1` 固定46字节 `AA 55 AA 55` 帧 | 复用 `FrameDecoder` 并校验累加和 |

DATA 的 `0x01` Signal-only 帧没有 Raw，不能反推出真实 Raw。收到20通道 Signal-only 时，
应用按该有效帧的实际版本发送一次 `0x82` Debug模式请求，然后等待 Raw。
该请求只修改本次上电周期的输出模式；应用不发送传感器校准、参数写入、Flash或复位命令。
RAW20 不发送命令。只有包含20通道真实 Raw 的采样进入算法；温度、参数回读、
状态诊断不重复计为传感器采样。其他通道数会被拒绝并显示。

未上传的固件状态不伪造；Debug帧的真实状态在CSV中以 `firmware_state0..19` 保存。
本应用的 `TOUCH_DOWN/MOVING/TOUCH_UP` 是整个触摸板的事件，与固件逐通道状态独立。
本地 baseline/signal 用于本次触摸算法调试，不回写或替换设备的 baseline/signal/state。

## 读取algo_monitor标定结果

在右侧“Raw Gain / 20通道增益”中点击“读取标定记录…”选择`PY_SRIPT/algo_monitor/raw_gain_calibrations/raw_gain_*.json`，或点击“读取最近一次标定”。读取后20通道增益先填入输入框，算法仍使用原增益。文件须有全部20通道合格原始采集；程序复用标定模型重新计算总增益，不依赖缓存结果或旧连接是否失效。来源文件不会被修改。

确认仍是同一阵列、接触面及三参数（Nsub、TxDivider、RefCDAC）未变，勾选确认框；完全移开砝码和手指，点击“应用 / 保存增益并重建基线”。本应用不自动核对标定硬件参数。增益会替换当前20路总增益，不会再次乘以原增益；保留标定四位小数精度，不需要手工抄写。

算法链路为`设备原始Raw → Raw×标定增益 → 本地空载基线与Signal → MAX/ROI/质心/滤波`。旧标定空载值不恢复，应用后须重新等待至少50帧及0.5秒稳定空载。设备Raw与固件真实通道状态保持原样；CSV仍保存原始Raw、增益和调整后的Raw。

应用后的增益与来源信息保存在`touch_config.json`，包括标定质量、目标变化量、文件路径及SHA256；录制开始和配置改变事件也保存这些信息。下次启动直接恢复保存的增益，外部标定记录更新后需重新读取并应用，避免运行中静默更换算法输入。手动修改已应用增益后会清除原标定来源声明。损坏、不完整、CV不合格或超出本应用0～100范围的记录会拒绝导入；保存失败不改变正在运行的增益和基线。

增益归一化不能修复原始Raw固定、饱和、串扰、慢回弹或ROI切换问题，效果仍需静态独立验证及滑动测试。

## 文件与算法入口

| 文件 | 作用 |
|---|---|
| `main.py` | 启动入口，支持 `--simulate` |
| `main_window.py` | 参数界面、来源切换、固定调试Label、配置及记录操作 |
| `sensor_widget.py` | QPainter绘制4行×5列、MAX/ROI/Active、质心和轨迹 |
| `serial_reader.py` | QThread接收、模式兼容、断开与重连 |
| `protocol_bridge.py` | 隔离导入既有解析器，补充边界/长度/校验/超时诊断 |
| `calibration_import.py` | 读取algo_monitor标定记录，复用原始采集校验并计算20通道总增益 |
| `sensor_processor.py` | 稳定基线建立、逐通道冻结与缓慢更新、绝对噪声门限 |
| `touch_tracker.py` | 独立 `TouchTracker.update(raw_values, timestamp)` → `TouchResult` |
| `processing_engine.py` | 逐帧算法调用、快照、断流保护、FPS、轨迹、记录调度 |
| `simulation.py` | 鼠标高斯压力与可选慢回弹模拟 |
| `data_recorder.py` | 有界后台CSV队列，关闭时补写尾部，错误可见 |
| `config.py` | `SENSOR_POSITIONS`、`AppConfig`默认值及JSON验证 |
| `sample_clock.py` | 采样/模拟/新鲜度共用高精度单调主机时钟 |
| `tests/` | 算法、协议、CSV、Qt和串口回环验证 |

真实数据路径是：

```text
SerialReader.run()
  → ProtocolDecoder.feed()
  → frame["raw"]
  → ProcessingEngine.update(raw_values, timestamp, frame)
  → TouchTracker.update(raw_values, timestamp)
  → 逐通道 adjusted_raw = Raw × raw_gains（浮点、不截断）
  → SensorProcessor：baseline / signal / noise threshold
  → MAX / ROI / relative threshold / power centroid / filter / dx,dy
```

`TouchTracker`、`SensorProcessor` 和 `config.py` 不依赖 Qt、串口或 NumPy。
输入必须为20个0～65535整数，时间戳为严格递增的单调秒数。内部计算使用浮点数。
右侧 `Raw Gain / 20通道增益` 为CH0～CH19分别设置乘数，默认1.0，范围0～100。
输入值不即时生效；点击“应用 / 保存增益并重建基线”统一应用并保存至 `touch_config.json`，
然后清除旧轨迹、重新采集至少50帧且至少0.5秒稳定数据。请先松手再应用，等待基线就绪。
“全部恢复1.0”只恢复输入框，随后点击应用/保存生效。0可屏蔽相应通道。
旧配置没有 `raw_gains` 时自动补齐20个1.0；加载含新增益的配置也自动重建基线。
界面R显示增益后的计算Raw，S和基线都在该增益尺度下计算；增益不会改变设备数据或固件状态。
新CSV的 `raw0..19` 仍保存设备原始整数，新增 `adjusted_raw0..19` 和 `raw_gain0..19`，
基线/Signal对应adjusted_raw。分析非默认增益的记录时，应使用adjusted_raw减baseline。
幂权重使用 `(signal / max_signal) ** power`，质心与 `signal ** power` 完全等价，
同时缩小数值范围。开启ROI过渡时，再乘该通道的ROI成员权重（0～1）。
导数权重按100Hz参考差分归一化并限幅，不放大批量收包的近零dt。
`on_touch_down`、`on_touch_move`、`on_touch_up` 为后续 UI 的回调扩展入口。

修改通道物理位置只需修改 `SENSOR_POSITIONS`：

```text
CH0  CH4  CH8  CH12 CH16
CH1  CH5  CH9  CH13 CH17
CH2  CH6  CH10 CH14 CH18
CH3  CH7  CH11 CH15 CH19
```

X向右增加，Y向下增加。Invert X/Y 仅影响 dx/dy 输出，阵列和实际质心坐标保持物理方向。
初次 TOUCH_DOWN 和 TOUCH_UP 输出 dx=dy=0；触摸结束后不继续由衰减数据计算位移。
Reset Tracker 清除位移参考、历史signal、触摸状态及轨迹，保留 baseline。

3x3目标ROI仍以真实MAX为中心，默认开启30ms的ROI平滑过渡。
当MAX改变时，旧ROI成员权重渐减、新ROI权重渐增，按实际经过时间线性推进，
不是按帧数延迟。若目标不再改变，最迟在设定时间内完成；快速反复改变目标时继续
从当前成员权重推进。过渡中有效ROI可能覆盖旧、新两块区域，旧残留只短暂参与，
蓝虚线亮度表示成员权重，绿框表示真正参与质心的通道。
此参数会增加局部跟随延迟，默认30ms作为起点；可在Position页关闭或设为0恢复硬切。
All Sensors模式不使用ROI过渡。第一次DOWN、UP、Reset和断流恢复均不会沿用旧ROI历史。
只有新位置有信号时，跨阵列移动仍可立即到达新位置；旧位置仍有强残留时可能需要过渡时间。

若 Raw 持续至少0.5秒低于本地 baseline，且负差超过
`max(Freeze Threshold, Noise Threshold, 稳定窗口允许波动, 1)`，界面以橙色虚线和
带符号的 `Δ` 标出相关通道，并显示持续负差提示；CSV 的 `below_baseline_indices`
保存这些通道。这可能是负向耦合/回弹，也可能是基线偏高，提示本身不改写状态或自动重建基线。
大幅负差被冻结后不会自行吸收；持续负差的通道不能仅凭 Signal=0 当作基线正常。

## 默认值及建议调参顺序

| 参数 | 默认值 |
|---|---:|
| Noise Threshold | 100 |
| Baseline Update Rate（每帧） | 0.001 |
| Baseline Freeze Threshold | 600 |
| Touch ON / OFF Threshold | 1000 / 600 |
| ROI Mode | 3x3 |
| ROI 平滑过渡 | ON |
| ROI 过渡时间 | 30 ms |
| Relative Threshold Ratio | 0.30 |
| Weight Power | 2.0 |
| Derivative Weight | OFF |
| Positive / Negative Gain | 1.0 / 1.0 |
| Minimum Decay Weight | 0.20 |
| Position Filter / Alpha | ON / 0.40 |
| DX / DY Deadzone | 0.01 / 0.01 |
| X / Y Gain | 1.0 / 1.0 |
| Trail Length | 100点，最多保留最近1秒 |
| Visualization Max Signal | 10000 |

以上幅值参数是起始值，需要依据真实Signal调节。

1. 空载观察Raw波动，正确建立 baseline，然后调整 Noise Threshold。
2. 比较空载残留与最轻有效按压的 MAX Signal，设置 OFF < ON。
   GUI修改一侧阈值越界时自动修正另一侧。
3. 用双向 Freeze Threshold 防止受压或负向下冲污染基线：仅当
   `abs(raw - baseline) < Freeze Threshold` 时更新。正向受压和负向大幅下冲均冻结；
   小幅空载漂移仍双向追踪，Baseline Update Rate 控制每帧更新速度。
   若空载环境永久偏移超过该阈值，需在确认释放且稳定后 Reset Baseline。
4. 观察 ROI、Active 和 Raw centroid，先调 Relative Threshold Ratio、Weight Power。
   比较ROI过渡开启/关闭的同一动作；默认30ms，可在20～40ms内比较，越长通常越平滑且延迟越大。
5. 再调 Position Alpha 和 Deadzone：滤波越强越容易延迟，过大死区会吞掉慢速移动。
6. 若回拉仍明显，再打开 Derivative Weight 实验，比较同一动作的记录。

所有算法参数在输入提交/值改变时立即生效，无需重启。数字输入框只显示数字，单位在标签中。
右侧分组点击展开，Debug使用固定Label；Last DOWN / UP Event 保留最近的触摸边沿，
避免50Hz GUI漏看仅持续一个采样帧的事件。

Save Config 保存到同目录 `touch_config.json`，下次启动自动载入。
Load Config 可选择其他JSON，完整验证通过后一次性生效；再点击 Save Config 可设为启动配置。
连接中的串口配置改动只在下次连接生效。baseline和触摸历史不作为配置保存。

## 记录、帧率及断流

点击 Start Recording 后CSV保存到：

```text
PY_SRIPT/new_all_on_python/recordings/touch_YYYYMMDD_HHMMSS_ffffff.csv
```

每个传感器采样保存 timestamp、monotonic_timestamp、来源/协议/配置修订号、
raw0..19、baseline0..19、signal0..19、MAX、原始/滤波XY、dx/dy、触摸事件、
有效性、Active/ROI、Total Signal及可用的真实固件状态。CSV的 baseline/signal 是PC本地计算值；
基线采集中的帧 `baseline_ready=False`，该阶段signal为零，不能当作正常传感器信号。
同名 `.events.jsonl` 保存初始参数/坐标、实时参数修改、重置、来源切换、断流和最终记录统计。
新记录的 `recording_start.baseline_update_rule` 标记双向保护规则版本，便于区分旧算法记录。
`roi_weight_rule` 标记ROI过渡规则，CSV同时保存目标ROI、有效ROI、是否过渡中以及
`roi_weight0..19`。历史记录缺少这些参数时，离线复现应明确使用当时的硬切模式，
不把新默认值混进原结果。
模拟记录明确标记 Simulation。

录制期间可点击“标记接触 (F7)”和“标记松手 (F8)”，或在应用窗口有焦点时使用快捷键，
在实际动作发生后立即标记。它们仅保存人为观察，不触发 DOWN/UP、不重建基线。
同名事件文件的 `user_observation` 包含 `contact/released`、高精度主机标记时间、
最近采样时间/距标记时间、已处理采样计数、数据来源和当时算法状态。
按键有人为反应延迟，适合定位明显回拉和松手后移动，不是硬件同步的抬手时刻。
未开始录制时按钮/快捷键禁用；记录可在开始按压前启动，以保留空载参考。

时间戳是主机接收时间，使用本机时区（本机为UTC+8）并保留偏移；协议未提供硬件采样时间。
采样和新鲜度使用同一 `perf_counter` 高精度单调时钟。旧版 Windows/Python 的
`monotonic` 可能仅约15.6ms分辨率，不能用来严格区分100Hz逐帧时间；
批量帧按真实逐帧处理时刻保留，不人为补造100Hz时间线。
批量到达的帧不会伪造为均匀100Hz。FPS为过去1秒收到的有效Raw采样数，启动时会渐增，
不计参数/温度等帧。算法逐帧计算，GUI仅50Hz绘图；CSV不会按绘图频率降采样。
记录器后台每秒flush，Stop Recording和正常关闭会补写队列尾部。
5000项有界队列满或文件写入失败时停止记录并显示错误，不静默丢帧。

两种协议都没有帧序号，无法精确计算 Dropped Frame Count，界面显示 N/A。
Parse Error Count 统计非法帧头/采样长度/RAW20校验/不完整帧超时；Discarded RX Bytes 统计重同步丢弃字节。
超过250ms无有效采样计一次 Stream Gap，并清零位移输出、清除历史参考和轨迹；
恢复后的首次接触仍为零位移。断开后允许重连，每次新连接重新建立 baseline。

## 衰减阶段的坐标可信度

右侧 `Filter / 位置滤波` 新增“衰减坐标保护”，默认开启，可随时关闭作对照。
旧配置缺少此项时使用新默认值；已有通道增益、ON/OFF和Position Alpha不变。
默认观察窗口40ms，明显衰减比例0.35（下降35%，输入框填写0.35）。

仅在以下条件同时满足时暂停输出坐标：近期MAX和总Signal都明显下降，新位置附近
没有足够的增长支持，而且位置突然回转或偏离近期方向超过60度。近期静止时则检查
是否出现超过0.03个通道间距的突变。运动方向只从强响应或有新响应支持的阶段更新，
避免把衰减尾部学成新的真实方向；正常沿原方向继续运动不直接触发暂停。
局部增长门槛为 `max(Noise Threshold, 当前MAX × 0.10, 1)`。

暂停期间粉色圈保持最后输出位置，DX/DY为0，白色空心圈仍显示未滤波质心。
新的局部响应支持持续20ms后恢复正常滤波；使用真实单调时间，不按帧数假造时长。
释放、重触、断流、Reset Tracker、Reset Baseline、增益变化或保护设置变化都会清除
相应历史。暂停本身不修改Raw、Signal、Baseline、MAX、ROI或PC的ON/OFF触摸事件，
也不改写固件上报状态。界面会显示“衰减保护：保持最后坐标”或“确认新响应”。

CSV末尾增加 `position_held`、`position_confidence`；状态值为 `tracking`、`decay_hold`、
`recovering`、`disabled`、`inactive`。`position_valid=True`只表示仍有可显示的输出位置，
不保证本帧接受了新坐标，应同时检查`position_held`。事件文件的
`position_confidence_rule=global_decay_direction_local_growth_guard_v1`记录规则版本。
离线复现旧记录时应显式关闭此保护，不能把新增默认值当成历史运行配置。

这是减少衰减尾部回勾的启发式保护，不是物理抬手检测，也不是空间定位标定。
轻微移动或减力同时转弯可能暂时暂停，需要用真实轻触、换向与长按验证；原始坐标的
固定偏置、稳定残留超过OFF、材料零点恢复等问题仍需另外处理。

## 验证及实测边界

在仓库根目录执行：

```powershell
$env:PYTHONPATH = (Resolve-Path PY_SRIPT).Path
$env:QT_QPA_PLATFORM = 'offscreen'
py -3.14 -m unittest discover -s PY_SRIPT/new_all_on_python/tests -v
```

离线验证覆盖：20通道方向与映射、中心和边缘ROI、非线性质心、MAX相邻切换、
远处/弱残留排除、静止零位移、滤波/增益/反转、ON/OFF滞回、抬手/重触零位移、
200Hz基线建立、不稳定基线拒绝、断流恢复、无效数据、V14～V21 Debug、V21 Raw、
RAW20校验、半帧/粘包/损坏重同步、CSV尾部、鼠标模拟、PySerial回环连接/关闭/重连、
Signal-only模式协商及串口异常。测试渲染图片保存在 `_verification/`，不参与应用运行。

真实硬件连接和慢回弹性能仍需在指定COM口按实际按压验收。
以下是第一阶段算法本身的边界，不能用离线测试替代实测：

- 稳定按压和稳定空载都能通过稳定窗口；建立基线前必须由操作人员先释放阵列。
- 若抬手后残留仍高于OFF阈值，Raw算法仍判为触摸，不能从Raw唯一推断真实抬手时刻。
  只有判定 TOUCH_UP 后才能保证残留不再输出位移。
- 旧位置若仍比新位置更强，MAX可能仍在旧位置；ROI/平方权重不能保证任意残留下都正确跟踪。
  强残留或ROI切换边界可能导致跳变/漂移，界面与CSV用于定位和决定下一阶段改进。
- 导数实验只调整ROI内权重，不改变MAX选择，也不能独立解决错误ROI。
- 2026-10-07 的两份连续滑动实测已确认上述残留与ROI切换问题。
  双向基线保护只防止负向下冲进一步拉低基线，不消除材料残留；离线回放仍需单独验证
  触点选择和抬手判定，不能把该修正当作慢回弹算法已通过验收。
- ROI过渡的四份历史记录回放隔离了ROI改动，基线和DOWN/UP判定保持相同。
  10:33记录中原始质心单帧跳变超过1格由13次降至1次，最大跳变由1.759降至1.193格；
  最大滤波跳变由0.770降至0.570格。10:05记录超过2格由23次降至4次，但仍有大跳变。
  回放不是有真实位置标注的定位验收，30ms过渡增加的手感延迟仍需实测。

本次未修改固件、未执行烧录、未打包EXE。

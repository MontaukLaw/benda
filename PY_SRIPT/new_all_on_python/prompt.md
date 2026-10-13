我现在有一个 4×5 的压力传感器阵列，共 20 个通道，希望把它做成类似触摸板的二维输入设备，用于后续控制类似 Apple Watch 蜂窝表盘的 UI。
通信协议参考UART_PROTOCOL_UNIFIED_V3_CN.md及6份配套说明
请基于我现有的串口采集程序，实现一个 Python GUI 调试工具。

核心目标是：

```text
串口数据
→ 20通道 raw_val
→ baseline / signal
→ 找最大压力点
→ 局部 ROI
→ 加权质心
→ x / y
→ dx / dy
→ GUI 中显示类似光标滑动
```

第一阶段先不要做真正的 Apple Watch UI，只验证触摸位置和 dx/dy 是否稳定、连续、没有明显回拉和跳动。

---

# 1. 输入数据

现有程序已经可以通过串口持续接收数据。

每帧最终可以解析得到：

```python
raw_val[20]
```

即：

```python
raw_val = [
    ch0, ch1, ch2, ..., ch19
]
```

每个通道范围：

```text
0 ~ 65535
```

压力越大，数值越大。

采样频率约：

```text
100 Hz
```

即约：

```text
10 ms / frame
```

请优先复用现有串口通信和解析逻辑。

不要无必要修改原来的通信协议。

只需要在获得：

```python
raw_val[20]
```

以后增加触摸算法模块。

---

# 2. 实际传感器排列

注意：

这是：

```text
4行 × 5列
```

不是5行×4列。

实际物理排列如下：

```text
        Col0   Col1   Col2   Col3   Col4

Row0    CH0    CH4    CH8    CH12   CH16
Row1    CH1    CH5    CH9    CH13   CH17
Row2    CH2    CH6    CH10   CH14   CH18
Row3    CH3    CH7    CH11   CH15   CH19
```

二维坐标定义：

```text
x = 0 ~ 4
y = 0 ~ 3
```

其中：

```text
x增加 = 向右
y增加 = 向下
```

通道坐标：

```text
CH0  = (0,0)
CH1  = (0,1)
CH2  = (0,2)
CH3  = (0,3)

CH4  = (1,0)
CH5  = (1,1)
CH6  = (1,2)
CH7  = (1,3)

CH8  = (2,0)
CH9  = (2,1)
CH10 = (2,2)
CH11 = (2,3)

CH12 = (3,0)
CH13 = (3,1)
CH14 = (3,2)
CH15 = (3,3)

CH16 = (4,0)
CH17 = (4,1)
CH18 = (4,2)
CH19 = (4,3)
```

当前排列满足：

```python
x = channel // 4
y = channel % 4
```

但是：

不要在算法代码中到处写 `/4` 和 `%4`。

请独立定义坐标映射：

```python
SENSOR_POSITIONS = {
    0:  (0, 0),
    1:  (0, 1),
    2:  (0, 2),
    3:  (0, 3),

    4:  (1, 0),
    5:  (1, 1),
    6:  (1, 2),
    7:  (1, 3),

    8:  (2, 0),
    9:  (2, 1),
    10: (2, 2),
    11: (2, 3),

    12: (3, 0),
    13: (3, 1),
    14: (3, 2),
    15: (3, 3),

    16: (4, 0),
    17: (4, 1),
    18: (4, 2),
    19: (4, 3),
}
```

以后 PCB 通道映射发生变化，只需要修改这张表。

---

# 3. GUI框架

优先使用：

```text
PySide6
```

如果现有项目已经使用：

```text
PyQt5
PyQt6
Tkinter
```

则优先沿用现有框架，不要为了换框架而大改工程。

---

# 4. 软件模块结构

不要把所有逻辑写进一个文件。

建议结构：

```text
main.py

serial_reader.py
    串口接收
    数据解析
    输出raw_val[20]

sensor_processor.py
    baseline
    signal
    noise threshold

touch_tracker.py
    max sensor
    ROI
    relative threshold
    weighted centroid
    filtering
    touch state
    dx/dy

sensor_widget.py
    4x5阵列显示
    光标
    ROI
    trail

main_window.py
    GUI主界面
    参数面板

config.py
    默认参数
```

如果现有工程已经有自己的结构：

以现有结构为主。

不要为了拆文件而进行大规模无意义重构。

---

# 5. 核心算法必须与GUI解耦

请实现一个独立算法类：

```python
class TouchTracker:
    def update(self, raw_values, timestamp):
        ...
        return result
```

输出类似：

```python
TouchResult(
    touched=True,
    state="MOVING",

    max_index=10,
    max_signal=23000,

    x_raw=2.15,
    y_raw=1.82,

    x=2.11,
    y=1.85,

    dx=0.04,
    dy=-0.02,

    active_indices=[5, 9, 10, 13, 14]
)
```

GUI只负责显示算法输出。

未来我要把算法迁移到：

```text
C
C++
MCU
嵌入式Linux
```

时，可以独立重写算法模块。

---

# 6. baseline

每个通道维护自己的 baseline：

```python
baseline[20]
```

计算：

```python
signal[i] = raw_val[i] - baseline[i]
```

如果小于0：

```python
signal[i] = 0
```

即：

```python
signal[i] = max(raw_val[i] - baseline[i], 0)
```

---

# 7. baseline 更新原则

传感器存在明显的：

```text
慢回弹
蠕变
历史残留
```

所以 baseline 不允许在明显受压时快速追踪 raw。

基本策略：

```text
如果通道没有明显受压：
    baseline缓慢跟踪raw

如果通道明显受压：
    baseline冻结
    或非常缓慢更新
```

推荐使用类似：

```python
baseline += alpha * (raw - baseline)
```

但是只在：

```text
signal < baseline_freeze_threshold
```

时更新。

GUI 参数：

```text
Baseline Update Rate
Baseline Freeze Threshold
```

增加按钮：

```text
Reset Baseline
```

点击以后：

重新使用当前稳定状态建立 baseline。

---

# 8. 绝对噪声门限

baseline subtraction以后：

```python
signal[i] = max(raw[i] - baseline[i], 0)
```

增加噪声门限：

```python
if signal[i] < noise_threshold:
    signal[i] = 0
```

GUI参数：

```text
Noise Threshold
```

---

# 9. 判断是否触摸

每帧寻找：

```python
max_index = argmax(signal)
max_signal = signal[max_index]
```

使用两个不同阈值产生 hysteresis：

```text
Touch ON Threshold
Touch OFF Threshold
```

例如：

```text
ON  = 1000
OFF = 600
```

逻辑：

```text
当前未触摸：

    max_signal > ON
        → TOUCH_DOWN

当前正在触摸：

    max_signal < OFF
        → TOUCH_UP
```

避免在阈值附近：

```text
DOWN
UP
DOWN
UP
```

反复抖动。

---

# 10. Touch状态机

实现：

```text
IDLE
TOUCH_DOWN
MOVING
TOUCH_UP
```

或者内部简化成：

```text
IDLE
TOUCHING
```

但GUI调试信息最好显示：

```text
TOUCH_DOWN
MOVING
TOUCH_UP
```

在第一次 TOUCH_DOWN 时：

```python
previous_x = current_x
previous_y = current_y

dx = 0
dy = 0
```

避免第一次接触产生巨大位移。

---

# 11. 不直接使用20点全局质心

不要把所有20个 signal 直接计算：

```python
x = sum(signal*x) / sum(signal)
```

因为传感器慢回弹会导致：

```text
旧位置还有较大残留

↓

质心被旧位置拖住

↓

手指已经移动
但质心明显滞后
```

甚至手指停下来后：

```text
旧位置继续衰减

↓

质心自己移动

↓

产生假的 dx / dy
```

因此改成：

```text
MAX
→ 局部ROI
→ relative threshold
→ nonlinear weighted centroid
```

---

# 12.寻找MAX点

计算：

```python
max_index = argmax(signal)
max_signal = signal[max_index]
```

GUI中必须高亮：

```text
当前MAX通道
```

---

# 13. ROI

默认使用：

```text
3×3 ROI
```

以MAX所在物理位置：

```python
max_x
max_y
```

为中心。

例如：

```text
MAX = CH10

CH10 = (2,2)
```

它附近的3×3区域为：

```text
CH5    CH9    CH13
CH6    CH10   CH14
CH7    CH11   CH15
```

注意：

不要用：

```python
max_index +/- 1
max_index +/- 4
```

直接找邻居。

必须根据：

```python
channel → (x,y)
```

进行二维坐标判断。

判断条件：

```python
abs(sensor_x - max_x) <= 1
and
abs(sensor_y - max_y) <= 1
```

边缘位置自动裁剪。

例如：

```text
MAX = CH0
```

附近只有：

```text
CH0 CH4
CH1 CH5
```

而不是强行凑9个点。

GUI参数：

```text
ROI Mode
```

选项：

```text
3x3
All Sensors
```

默认：

```text
3x3
```

---

# 14. Relative Threshold

ROI内不要所有传感器都参与质心。

计算：

```python
relative_threshold = max_signal * relative_threshold_ratio
```

例如默认：

```text
ratio = 0.30
```

只有：

```python
signal[i] >= max_signal * ratio
```

才参与质心。

目的：

抑制旧位置慢回弹产生的弱残留。

GUI参数：

```text
Relative Threshold Ratio
```

范围建议：

```text
0.0 ~ 1.0
```

默认：

```text
0.30
```

---

# 15. nonlinear weighted centroid

不要使用：

```python
weight = signal
```

而是：

```python
weight = signal ** power
```

默认：

```text
power = 2.0
```

计算：

```python
sum_weight = sum(weight)

x_raw = sum(weight[i] * sensor_x[i]) / sum_weight
y_raw = sum(weight[i] * sensor_y[i]) / sum_weight
```

GUI参数：

```text
Weight Power
```

范围建议：

```text
1.0 ~ 4.0
```

默认：

```text
2.0
```

---

# 16. 为什么使用平方权重

例如：

```text
当前真正受压：
30000

旧位置残留：
5000
```

线性权重：

```text
6 : 1
```

平方权重：

```text
36 : 1
```

可以明显降低弱残留对质心的影响。

---

# 17. Derivative Weight实验功能

传感器旧位置可能出现：

```text
12000
11800
11600
11400
```

即：

```text
signal仍然很大
但正在下降
```

而当前新位置可能：

```text
1000
5000
14000
25000
```

即：

```text
正在快速上升
```

因此增加可选的：

```text
Derivative Weight
```

计算：

```python
ds[i] = signal[i] - previous_signal[i]
```

默认：

```text
OFF
```

打开时：

正在上升的通道：

```text
权重提高
```

正在下降的通道：

```text
权重降低
```

基础权重：

```python
base_weight = signal[i] ** power
```

可以实现类似：

```python
if ds[i] >= 0:
    derivative_factor = 1.0 + positive_gain * normalized_ds
else:
    derivative_factor = max(
        minimum_decay_weight,
        1.0 + negative_gain * normalized_ds
    )

weight = base_weight * derivative_factor
```

具体实现要：

```text
简单
稳定
数值不会爆炸
```

GUI参数：

```text
Derivative Weight Enable

Positive Gain
Negative Gain

Minimum Decay Weight
```

默认：

```text
OFF
```

因为第一阶段优先验证：

```text
MAX
+
ROI
+
Relative Threshold
+
Power Centroid
```

---

# 18. centroid结果

算法输出：

```python
x_raw
y_raw
```

坐标范围：

```text
x_raw = 0.0 ~ 4.0
y_raw = 0.0 ~ 3.0
```

这是连续浮点坐标。

例如：

```text
x = 2.37
y = 1.64
```

不要把光标限制在某一个传感器中心。

---

# 19. 位置滤波

对：

```python
x_raw
y_raw
```

增加一阶低通：

```python
x_filtered += alpha * (x_raw - x_filtered)
y_filtered += alpha * (y_raw - y_filtered)
```

GUI参数：

```text
Position Filter Enable

Position Alpha
```

默认：

```text
ON

alpha = 0.4
```

范围：

```text
0.0 ~ 1.0
```

其中：

```text
alpha大：
响应快
但抖动更明显

alpha小：
更平滑
但延迟更大
```

---

# 20. dx / dy

在触摸状态下：

```python
dx_raw = x_filtered - previous_x
dy_raw = y_filtered - previous_y
```

之后更新：

```python
previous_x = x_filtered
previous_y = y_filtered
```

---

# 21. Deadzone

增加小位移死区：

```python
if abs(dx_raw) < dx_deadzone:
    dx_raw = 0

if abs(dy_raw) < dy_deadzone:
    dy_raw = 0
```

GUI参数：

```text
DX Deadzone
DY Deadzone
```

默认：

```text
0.01
0.01
```

---

# 22. Gain

最终：

```python
dx = dx_raw * x_gain
dy = dy_raw * y_gain
```

GUI参数：

```text
X Gain
Y Gain
```

默认：

```text
1.0
1.0
```

---

# 23. 坐标反转

增加：

```text
Invert X
Invert Y
```

如果开启：

```python
dx = -dx
```

或：

```python
dy = -dy
```

方便适配实际安装方向。

---

# 24. 当前阶段不要做累计虚拟鼠标坐标

当前 GUI 中主要显示：

```text
实际 centroid / cursor
```

即：

```text
x_filtered
y_filtered
```

不要把 dx/dy 无限累计成一个完全独立的鼠标坐标。

第一阶段主要观察：

```text
真实手指位置
与
算法估计位置
```

是否一致。

同时显示：

```text
dx
dy
```

供后续 Apple Watch UI 使用。

---

# 25. GUI整体布局

界面大致：

```text
+---------------------------------------------------------------+
|                                                               |
|           4×5 SENSOR AREA             |    PARAMETERS         |
|                                       |                       |
|                                       | Serial                |
|                                       | Signal                |
|                                       | Touch                 |
|                                       | Position              |
|                                       | Filter                |
|                                       | Gesture               |
|                                       | Visualization         |
|                                       | Debug                 |
|                                                               |
+---------------------------------------------------------------+
```

左侧占主要面积。

右侧是参数配置。

---

# 26. 左侧4×5显示

严格显示成：

```text
┌────────┬────────┬────────┬────────┬────────┐
│  CH0   │  CH4   │  CH8   │ CH12   │ CH16   │
├────────┼────────┼────────┼────────┼────────┤
│  CH1   │  CH5   │  CH9   │ CH13   │ CH17   │
├────────┼────────┼────────┼────────┼────────┤
│  CH2   │  CH6   │ CH10   │ CH14   │ CH18   │
├────────┼────────┼────────┼────────┼────────┤
│  CH3   │  CH7   │ CH11   │ CH15   │ CH19   │
└────────┴────────┴────────┴────────┴────────┘
```

即：

```text
4 rows
5 columns
```

---

# 27. 每个Cell显示

每个传感器格显示：

```text
CH编号
Raw
Signal
```

例如：

```text
CH10

R: 23104
S: 12543
```

字体不要太大。

---

# 28. 压力强度可视化

每个格子的背景亮度根据：

```python
signal
```

变化。

压力越大：

```text
显示越亮
```

不要使用过于刺眼的彩虹色。

最好使用单色渐变。

需要注意：

不要直接用：

```text
0~65535
```

作为固定显示范围。

因为实际 signal 可能只有：

```text
0~10000
```

可以根据：

```text
Visualization Max Signal
```

进行归一化。

GUI增加：

```text
Visualization Max Signal
```

---

# 29. MAX显示

当前：

```python
max_index
```

所在格：

使用明显边框。

例如：

```text
黄色粗边框
```

---

# 30. ROI显示

3×3 ROI里的传感器：

使用另一种边框或半透明效果。

---

# 31. Active Sensors显示

真正通过：

```text
relative threshold
```

参与质心计算的通道：

使用第三种显示方式。

例如：

```text
绿色边框
```

因此视觉上可以区分：

```text
MAX
ROI
Active Sensors
```

---

# 32. 虚拟Cursor

在整个4×5阵列上绘制一个圆形 cursor。

位置：

```text
x_filtered
y_filtered
```

例如：

```text
x = 2.35
y = 1.72
```

圆点应该出现在对应连续位置。

不是只能停在20个 sensor 中心。

---

# 33. Raw Centroid

增加一个较小的标记：

```text
Raw Centroid
```

对应：

```text
x_raw
y_raw
```

例如：

```text
空心圆
```

---

# 34. Filtered Cursor

主光标：

```text
Filtered Centroid
```

例如：

```text
实心圆
```

这样可以直接观察滤波引入多少延迟。

---

# 35. Cursor Trail

保存最近一段时间位置：

```text
0.5 ~ 1 秒
```

100Hz 下：

```text
50 ~ 100 points
```

绘制轨迹线。

GUI参数：

```text
Show Trail

Trail Length
```

可以直接观察：

```text
轨迹是否连续
有没有跳变
有没有回拉
有没有震荡
```

---

# 36. dx / dy显示

左侧阵列下方显示：

```text
X:
Y:

DX:
DY:
```

并用较大字体实时更新。

例如：

```text
X  = 2.371
Y  = 1.684

DX = +0.042
DY = -0.016
```

---

# 37. 可选：dx/dy方向箭头

可以增加一个简单箭头：

```text
→
←
↑
↓
```

表示当前主要移动方向。

如果：

```text
abs(dx) > abs(dy)
```

主要显示左右。

否则主要显示上下。

仅用于调试。

---

# 38. 右侧 Serial 面板

包括：

```text
COM Port

Refresh Ports

Baudrate

Connect

Disconnect

Serial Status

Actual FPS
```

串口断开时不要让 GUI 崩溃。

支持重新连接。

---

# 39. Signal参数

右侧：

```text
Noise Threshold

Baseline Update Rate

Baseline Freeze Threshold

Reset Baseline
```

所有参数修改以后：

```text
立即生效
```

---

# 40. Touch参数

```text
Touch ON Threshold

Touch OFF Threshold
```

需要限制：

```text
OFF < ON
```

如果用户设置不合理：

可以提示或自动修正。

---

# 41. Position Algorithm参数

```text
ROI Mode
    3x3
    All

Relative Threshold Ratio

Weight Power

Derivative Weight Enable

Positive Gain

Negative Gain

Minimum Decay Weight
```

---

# 42. Filter参数

```text
Position Filter Enable

Position Alpha
```

---

# 43. Gesture参数

```text
DX Deadzone

DY Deadzone

X Gain

Y Gain

Invert X

Invert Y
```

---

# 44. Visualization参数

```text
Show Raw

Show Signal

Show MAX

Show ROI

Show Active Sensors

Show Raw Centroid

Show Filtered Cursor

Show Trail

Trail Length

Visualization Max Signal
```

---

# 45. Debug区域

实时显示：

```text
FPS

Touch State

MAX Channel
MAX Signal

Active Sensor Count

X Raw
Y Raw

X Filtered
Y Filtered

DX
DY

Total Signal
```

不要把所有内容不断 append 到 QTextEdit。

使用固定 Label 更新。

避免GUI越来越慢。

---

# 46. FPS计算

不要假定串口一定正好100Hz。

根据 timestamp 实际计算：

```text
Actual FPS
```

建议做一个：

```text
1秒滑动平均
```

GUI显示：

```text
98.7 Hz
100.2 Hz
```

之类。

---

# 47. GUI刷新频率

算法处理：

```text
跟串口一样
≈100 Hz
```

GUI没有必要100Hz全刷新。

建议：

```text
30~60 Hz
```

例如：

```text
50 Hz
```

数据算法仍然每帧处理。

GUI只降低绘图刷新频率。

---

# 48. 线程

禁止在 GUI 主线程里阻塞：

```python
serial.read()
```

使用：

```text
QThread
```

或者：

```text
worker thread
```

串口线程主要负责：

```text
read
parse
raw_val[20]
```

然后通过 signal / callback 交给算法。

不要因为串口超时卡死GUI。

---

# 49. 数据记录

增加：

```text
Start Recording
Stop Recording
```

保存 CSV。

每一帧记录：

```text
timestamp

raw0
raw1
...
raw19

baseline0
...
baseline19

signal0
...
signal19

max_index
max_signal

x_raw
y_raw

x_filtered
y_filtered

dx
dy

touch_state

active_sensor_count
```

文件名自动包含时间戳。

---

# 50. 参数实时修改

所有算法参数：

```text
Noise Threshold

Baseline参数

Touch Threshold

ROI

Relative Threshold

Power

Derivative Weight

Alpha

Deadzone

Gain
```

都必须：

```text
运行过程中实时修改
```

不需要重启程序。

---

# 51. 默认参数

第一版默认：

```text
ROI Mode
3x3
```

```text
Relative Threshold Ratio
0.30
```

```text
Weight Power
2.0
```

```text
Derivative Weight
OFF
```

```text
Position Filter
ON
```

```text
Position Alpha
0.40
```

```text
DX Deadzone
0.01
```

```text
DY Deadzone
0.01
```

```text
X Gain
1.0
```

```text
Y Gain
1.0
```

Touch阈值和 baseline threshold：

请根据我的实际 signal 数据通过GUI调节。

不要因为raw最大值65535，就假设正常按压一定接近65535。

---

# 52. 重要问题：慢回弹

我的传感器有比较明显的慢回弹。

例如手指：

```text
A → B
```

手指已经离开A，但A的 signal 仍然可能：

```text
12000
10000
8000
6000
...
```

慢慢下降。

所以重点验证：

```text
手指到了B以后
cursor不能明显被A拉回去
```

尤其关注：

```text
手指停止运动后
cursor是否自己漂移
```

---

# 53. 第一阶段算法顺序

第一阶段一定按照：

```text
RAW
↓
Baseline
↓
Signal
↓
Noise Threshold
↓
MAX
↓
3×3 ROI
↓
Relative Threshold
↓
Power Weight
↓
Centroid
↓
Position Filter
↓
dx/dy
```

先不要加入复杂机器学习算法。

---

# 54. 如果第一版仍有明显回拉

再打开：

```text
Derivative Weight
```

利用：

```text
ds/dt
```

降低正在衰减的旧通道权重。

不要一开始就把所有复杂功能全部打开。

---

# 55. 不建议直接用MAX通道位置

不要使用：

```text
cursor = MAX sensor center
```

作为正式算法。

MAX仅用于：

```text
确定主要触摸区域
确定ROI
```

最终位置依然使用：

```text
ROI内加权质心
```

因为MAX位置只有20个离散点，会导致：

```text
跳格
抖动
位置分辨率低
```

---

# 56. MAX切换时避免不必要的突跳

需要特别观察：

当手指处于两个sensor之间时：

```text
CH9 = 12000
CH10 = 11900
```

下一帧：

```text
CH9 = 11800
CH10 = 12100
```

MAX可能：

```text
CH9 → CH10
```

但是最终 centroid 不应该因此突然跳一整个格子。

ROI + weighted centroid 应该保证：

```text
位置连续
```

如果MAX切换造成明显跳动：

优先修正ROI/权重逻辑。

不要直接用MAX坐标作为输出。

---

# 57. ROI切换边界问题

当MAX从一个sensor切换到相邻sensor时：

3×3 ROI也会变化。

要注意：

```text
某些通道突然进入ROI
某些通道突然离开ROI
```

可能产生小的centroid跳动。

实现时尽量保持连续。

如果有必要，可以增加实验选项：

```text
ROI Padding
```

或者：

```text
Previous ROI Merge
```

但是第一版先不启用复杂逻辑。

---

# 58. GUI视觉目标

GUI不是为了做漂亮产品界面。

主要目标是：

```text
算法调试
```

希望一眼可以看到：

```text
压力分布
MAX在哪里
ROI在哪里
哪些点参与质心
Raw centroid在哪里
Filtered centroid在哪里
轨迹如何
dx/dy如何
```

界面尽量：

```text
简洁
深色
工程调试风格
```

避免复杂动画。

---

# 59. 参数保存

增加：

```text
Save Config
Load Config
```

使用：

```text
JSON
```

保存当前所有算法参数。

例如：

```text
touch_config.json
```

程序启动时：

如果存在上一次配置：

自动载入。

否则使用默认配置。

---

# 60. Reset功能

增加：

```text
Reset Baseline
```

以及：

```text
Reset Tracker
```

Reset Tracker应该清除：

```text
previous x/y
trail
touch state
previous signals
```

但不一定清除baseline。

---

# 61. 串口异常处理

需要处理：

```text
串口断开

非法数据包

数据帧长度不对

解析失败

短暂丢帧
```

错误不能导致GUI退出。

可以显示：

```text
Parse Error Count
Dropped Frame Count
```

---

# 62. 测试模式

除了真实串口以外，增加：

```text
Simulation Mode
```

方便没有硬件时调试GUI。

Simulation Mode可以：

用鼠标在4×5区域移动。

根据鼠标位置生成模拟压力分布。

例如使用二维高斯：

```text
距离鼠标越近的sensor
signal越大
```

这样可以验证：

```text
centroid
cursor
trail
dx/dy
```

是否工作。

Simulation Mode与真实串口模式分开。

不要影响实际串口逻辑。

---

# 63. 性能要求

100Hz数据下：

CPU占用应该合理。

不要：

```text
每帧重建全部GUI控件
```

只更新：

```text
已有widget的内容
```

绘图尽量使用：

```text
QPainter
```

或者轻量自定义Widget。

---

# 64. 数据类型

raw数据：

```python
int
```

baseline：

```python
float
```

signal：

```python
float
```

centroid：

```python
float
```

dx/dy：

```python
float
```

不要因为raw来自uint16，就在中间计算继续使用uint16。

否则：

```text
减法
平方
```

可能发生溢出或截断。

---

# 65. Power Weight数值安全

如果：

```python
signal ** 2
```

需要转换为：

```python
float
```

或者：

```python
float64
```

避免整数溢出。

例如：

```python
weight = float(signal) ** power
```

---

# 66. 无有效点保护

有可能：

```text
max_signal > touch_threshold
```

但是经过：

```text
ROI
relative threshold
```

以后有效点数量异常。

必须保护：

```python
sum_weight <= epsilon
```

此时不要除0。

可以：

```text
保持上一位置
```

或者：

```text
使用MAX位置
```

作为临时 fallback。

但必须保证程序不崩溃。

---

# 67. TOUCH_UP行为

当：

```text
max_signal < OFF threshold
```

时：

进入：

```text
TOUCH_UP
```

此时：

```text
dx = 0
dy = 0
```

不要继续根据正在回弹的 signal 输出移动。

这点非常重要。

一旦判定：

```text
手指已经离开
```

历史残留不能继续推动cursor产生手势。

---

# 68. Touch重新开始

下一次：

```text
TOUCH_DOWN
```

重新：

```python
previous_x = current_x
previous_y = current_y
```

所以：

```text
第一次帧dx=0
dy=0
```

避免新一次触摸产生跨位置跳跃。

---

# 69. 可选的Total Signal

计算：

```python
total_signal = sum(signal)
```

GUI显示。

这个值可以帮助观察：

```text
整体压力大小
```

但暂时不要用它作为主要位置算法。

---

# 70. 后续接口预留

未来这个：

```text
dx
dy
```

会用于控制类似 Apple Watch 的蜂窝图标界面。

请预留：

```python
on_touch_down(x, y)

on_touch_move(dx, dy, x, y)

on_touch_up(x, y)
```

类似接口。

但第一阶段：

只打印/显示即可。

---

# 71. 第一阶段验收标准

程序运行以后：

我用手指在传感器上：

```text
左 → 右
```

GUI中的cursor应该：

```text
平滑左 → 右
```

不能：

```text
大范围跳动
```

---

我用手指：

```text
上 → 下
```

cursor：

```text
上 → 下
```

方向正确。

---

我移动到某个位置后停止：

cursor应该：

```text
基本停止
```

不能由于旧signal慢慢回弹而持续明显漂移。

---

我快速：

```text
A → B
```

cursor应该：

```text
跟到B
```

不能明显被A的历史残留拖回。

---

手指抬起：

```text
TOUCH_UP
```

以后：

```text
dx=0
dy=0
```

不能因为材料继续回弹产生虚假滑动。

---

# 72. 调试重点

请重点检查以下四类问题：

```text
1. MAX频繁切换导致跳动

2. ROI切换导致质心跳动

3. 历史残留导致质心回拉

4. Position Filter过强导致延迟
```

GUI必须能够通过：

```text
MAX显示
ROI显示
Active Sensor显示
Raw centroid
Filtered centroid
Trail
```

让我直观看出问题来自哪里。

---

# 73. 第一版优先级

开发优先级：

```text
1. 串口稳定读取

2. 正确4×5通道映射

3. baseline / signal

4. MAX + ROI

5. weighted centroid

6. cursor显示

7. dx/dy

8. GUI参数实时调节

9. trail

10. CSV记录

11. derivative weighting

12. simulation mode
```

不要为了实现次要功能而影响核心算法稳定性。

---

# 74. 最终要求

请先阅读并理解现有代码结构。

在现有程序基础上修改。

不要凭空重新写一个与原程序无法整合的新项目。

修改完成后：

请检查：

```text
程序是否可以启动

串口是否可以连接

20通道数据是否正确

GUI是否为4行×5列

通道顺序是否严格为：

CH0 CH4 CH8 CH12 CH16
CH1 CH5 CH9 CH13 CH17
CH2 CH6 CH10 CH14 CH18
CH3 CH7 CH11 CH15 CH19

MAX是否正确

ROI是否根据XY坐标计算

centroid是否为连续浮点值

dx/dy是否正确

TOUCH_UP以后是否停止输出位移
```

不要把阵列错误写成：

```text
5行×4列
```

这一点非常重要。

---

# 75. 请在实现后说明

代码完成以后，请简要告诉我：

```text
1. 修改了哪些文件

2. TouchTracker算法入口在哪里

3. 串口raw_val在哪里传入算法

4. 主要默认参数在哪里配置

5. GUI中的哪些参数最值得我优先调

6. CSV保存位置

7. 如何启动程序
```

不要只生成代码而不说明入口。

整个实现优先追求：

```text
稳定
容易调参数
容易观察问题
容易继续修改
```

而不是追求复杂算法。
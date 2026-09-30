"""电机速度始终使用实际RPM；仅协议层换算为整数编码，避免小数被截断。"""
import math


def valid_rpm(value, *, minimum=.1, maximum=3000, step=.1):
    return (type(value) in (int, float) and math.isfinite(value)
            and minimum <= value <= maximum
            and abs(value/step-round(value/step)) <= 1e-7)


def quantize_rpm(value, step):
    ticks = round(value/step)
    return ticks if step == 1 else round(ticks*step, 1)

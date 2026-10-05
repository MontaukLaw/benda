"""柱状图与触摸板共用逐通道Raw增益，不改写接收样本。"""
import math


RAW_GAIN_MAX = 1000.0
RAW_DISPLAY_MAX = int(65535 * RAW_GAIN_MAX)


def normalize_raw_gains(values):
    # 无效配置沿用单路默认值，与增益输入框范围和精度一致。
    gains = [1.0] * 20
    if isinstance(values, (list, tuple)) and len(values) == 20:
        for channel, value in enumerate(values):
            try:
                gain = float(value)
                if math.isfinite(gain) and 0 <= gain <= RAW_GAIN_MAX:
                    gains[channel] = round(gain, 4)
            except (ValueError, TypeError, OverflowError):
                pass
    return tuple(gains)


def apply_raw_gains(raw, gains):
    # 返回独立浮点输入，参考采集和运行时使用同一缩放口径。
    return tuple(value * gain for value, gain in zip(raw, gains))

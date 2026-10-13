"""采样与新鲜度统一使用高精度单调时钟，兼容旧版 Windows/Python。"""

import time


def now():
    # 旧版Windows的monotonic可能只有约15.6ms分辨率，不能作为100Hz逐帧时间戳。
    # perf_counter使用高精度计数器；不人为补出10ms，也不重构设备采样时刻。
    return time.perf_counter()

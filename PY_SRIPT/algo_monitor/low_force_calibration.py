"""四点手动低力标定的输入规则和统一显示语义。"""
import math

MASSES_G = (10, 20, 50, 100)
FORCES_N = tuple(mass * 9.80665 / 1000 for mass in MASSES_G)
DETECTION_N = 0.09


def validate_points(points):
    entries = tuple(points)
    if len(entries) != 4:
        raise ValueError("需要完整的10、20、50、100 g四个标定值")
    missing = [str(mass) for mass, value in zip(MASSES_G, entries) if value is None or isinstance(value, str) and not value.strip()]
    if missing:
        raise ValueError("缺少" + "、".join(missing) + " g的Signal和；已有标定请先点击“读取 MCU Flash”，否则请填写四点标定")
    converted = []
    for mass, value in zip(MASSES_G, entries):
        try:
            converted.append(float(value))
        except (ValueError, TypeError, OverflowError):
            raise ValueError(f"{mass} g的Signal和必须填写有效数字") from None
    values = tuple(converted)
    if len(values) != 4 or any(not math.isfinite(value) or not 0 < value <= 262140 for value in values):
        raise ValueError("四个Signal和必须是大于0且不超过262140的有限数值")
    if any(right <= left for left, right in zip(values, values[1:])):
        raise ValueError("Signal和必须按10、20、50、100 g严格递增")
    return values


def validate_detection_counts(value, points):
    try:
        value = float(value)
    except (ValueError, TypeError, OverflowError):
        raise ValueError("最低检测线必须填写有效数字；0表示关闭模拟") from None
    if not math.isfinite(value) or not 0 <= value < points[0]:
        raise ValueError("最低检测线必须≥0且小于10 g标定点的Signal和；0表示关闭模拟")
    return value


def low_force_display(vector, optimized=False):
    """仅处理固件明确标记的低力模式，旧协议显示保持兼容。"""
    if not vector or not vector.get("low_force_mode"):
        return None
    if not vector.get("low_force_calibration_valid"):
        return "-- N", "低力标定未保存或无效", "invalid"
    if vector.get("low_force_over_range"):
        return "超出低力量程", "换算结果≥1 N", "over_range"
    if not vector.get("normal_calibration_valid"):
        return "-- N", "等待MCU完成基线校准", "waiting"
    value = vector.get("optimized_normal_force" if optimized else "normal_force")
    if optimized and value is None:
        return "-- N", "优化值未上传，需要V18或更新固件", "waiting"
    try:
        force = float(value)
    except (ValueError, TypeError):
        return "-- N", "MCU低力值无效", "invalid"
    if not math.isfinite(force) or force < 0 or force >= 1:
        return "-- N", "MCU低力值无效", "invalid"
    # 检测线只由原始值决定，不重复截断优化结果。
    simulated = bool(vector.get("low_force_simulated"))
    if not optimized and not simulated and force < DETECTION_N:
        force = 0.0
    text = "<1.000 N" if force >= 0.9995 else f"{force:.3f} N"
    if simulated:
        return text, "最低检测线触发模拟输出；实际测量值独立上传并保留", "valid"
    return text, "四点低力标定有效 · 检测线按原始值判断", "valid"

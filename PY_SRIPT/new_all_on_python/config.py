"""默认参数、物理坐标及可验证的 JSON 配置。"""

from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "touch_config.json"
RECORDING_DIR = APP_DIR / "recordings"
RAW_GAIN_LIMITS = (0.0, 100.0, 0.05, 6)
SENSOR_POSITIONS = {
    0: (0, 0), 1: (0, 1), 2: (0, 2), 3: (0, 3),
    4: (1, 0), 5: (1, 1), 6: (1, 2), 7: (1, 3),
    8: (2, 0), 9: (2, 1), 10: (2, 2), 11: (2, 3),
    12: (3, 0), 13: (3, 1), 14: (3, 2), 15: (3, 3),
    16: (4, 0), 17: (4, 1), 18: (4, 2), 19: (4, 3),
}

# 范围同时用于配置验证和 GUI，输入框只呈现数字。
NUMERIC_LIMITS = {
    "noise_threshold": (0, 65535, 1, 0),
    "baseline_update_rate": (0, 1, 0.001, 4),
    "baseline_freeze_threshold": (0, 65535, 10, 0),
    "baseline_stability_limit": (0, 65535, 5, 0),
    "touch_on_threshold": (1, 65535, 10, 0),
    "touch_off_threshold": (0, 65534, 10, 0),
    "relative_threshold_ratio": (0, 1, 0.05, 2),
    "weight_power": (1, 4, 0.1, 2),
    "roi_transition_ms": (0, 200, 5, 0),
    "positive_gain": (0, 10, 0.1, 2),
    "negative_gain": (0, 10, 0.1, 2),
    "minimum_decay_weight": (0, 1, 0.05, 2),
    "position_alpha": (0, 1, 0.05, 2),
    "decay_guard_window_ms": (10, 200, 5, 0),
    "decay_guard_drop_ratio": (0.05, 0.95, 0.05, 2),
    "dx_deadzone": (0, 4, 0.001, 3),
    "dy_deadzone": (0, 3, 0.001, 3),
    "x_gain": (0, 100, 0.1, 2),
    "y_gain": (0, 100, 0.1, 2),
    "trail_length": (1, 1000, 10, 0),
    "visualization_max_signal": (1, 65535, 100, 0),
}


@dataclass
class AppConfig:
    raw_gains: list = field(default_factory=lambda: [1.0] * 20)
    calibration_source: dict = field(default_factory=dict)
    noise_threshold: float = 100
    baseline_update_rate: float = 0.001
    baseline_freeze_threshold: float = 600
    baseline_stability_limit: float = 80
    touch_on_threshold: float = 1000
    touch_off_threshold: float = 600
    roi_mode: str = "3x3"
    roi_transition_enable: bool = True
    roi_transition_ms: float = 30
    relative_threshold_ratio: float = 0.30
    weight_power: float = 2.0
    derivative_weight_enable: bool = False
    positive_gain: float = 1.0
    negative_gain: float = 1.0
    minimum_decay_weight: float = 0.20
    position_filter_enable: bool = True
    position_alpha: float = 0.40
    decay_guard_enable: bool = True
    decay_guard_window_ms: float = 40
    decay_guard_drop_ratio: float = 0.35
    dx_deadzone: float = 0.01
    dy_deadzone: float = 0.01
    x_gain: float = 1.0
    y_gain: float = 1.0
    invert_x: bool = False
    invert_y: bool = False
    show_raw: bool = True
    show_signal: bool = True
    show_max: bool = True
    show_roi: bool = True
    show_active_sensors: bool = True
    show_raw_centroid: bool = True
    show_filtered_cursor: bool = True
    show_trail: bool = True
    trail_length: int = 100
    visualization_max_signal: float = 10000
    baudrate: int = 1000000
    port: str = ""
    protocol: str = "DATA"

    def validate(self):
        if not isinstance(self.raw_gains, list) or len(self.raw_gains) != 20:
            raise ValueError("Raw 增益必须为20通道列表")
        for index, gain in enumerate(self.raw_gains):
            if isinstance(gain, bool) or not isinstance(gain, (int, float)) or not math.isfinite(gain) or not RAW_GAIN_LIMITS[0] <= gain <= RAW_GAIN_LIMITS[1]:
                raise ValueError(f"CH{index} Raw 增益必须为0～100的有限数字")
        if not isinstance(self.calibration_source, dict):
            raise ValueError('标定来源必须为对象')
        if self.calibration_source:
            source = self.calibration_source
            if (source.get('schema_version') != 1 or not isinstance(source.get('path'), str)
                    or not isinstance(source.get('sha256'), str) or len(source['sha256']) != 64
                    or any(c not in '0123456789abcdef' for c in source['sha256'])
                    or source.get('gains') != self.raw_gains):
                raise ValueError('标定来源与当前20通道增益不一致')
            for name in ('mass_g', 'target'):
                value = source.get(name)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError('标定来源的质量或目标无效')
            try:
                json.dumps(source, allow_nan=False)
            except (TypeError, ValueError) as exc:
                raise ValueError('标定来源含无效数据') from exc
        for name, (low, high, _step, _decimals) in NUMERIC_LIMITS.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)):
                raise ValueError(f"{name} 必须为数字")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} 必须在 {low}～{high} 内")
        if not self.touch_off_threshold < self.touch_on_threshold:
            raise ValueError("Touch OFF 必须小于 ON")
        for field in fields(self):
            if isinstance(field.default, bool) and type(getattr(self, field.name)) is not bool:
                raise ValueError(f"{field.name} 必须为布尔值")
        if type(self.trail_length) is not int:
            raise ValueError("Trail Length 必须为整数")
        if type(self.baudrate) is not int or not 1200 <= self.baudrate <= 4000000:
            raise ValueError("波特率必须为1200～4000000的整数")
        if self.roi_mode not in ("3x3", "All Sensors"):
            raise ValueError("ROI Mode 无效")
        if self.protocol not in ("DATA", "RAW20") or not isinstance(self.port, str):
            raise ValueError("串口配置无效")
        return self

    def copy(self):
        return AppConfig(**asdict(self))

    def save(self, path=CONFIG_PATH):
        self.validate()
        path = Path(path)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path=CONFIG_PATH):
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if not isinstance(data, dict):
            raise ValueError("配置文件必须为 JSON 对象")
        unknown = set(data) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("未知配置项：" + ", ".join(sorted(unknown)))
        return cls(**data).validate()

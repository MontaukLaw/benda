"""电机型号选择独立保存，不与旧电机运动参数混用。"""

import json
from pathlib import Path
import sys

from zdt_x42s_can_driver import ZDTConfig


def motor_connection_settings_path():
    root = Path(sys.executable).parent if getattr(sys, "frozen", False) else Path(__file__).parent
    return root / "motor_connection_config.json"


def load_motor_connection_settings(path=None):
    path = Path(path) if path is not None else motor_connection_settings_path()
    data = json.loads(path.read_text(encoding="utf-8-sig")) if path.exists() else {}
    if not isinstance(data, dict):
        raise ValueError(f"{path.name} 必须为 JSON 对象")
    model = data.get("motor_model", "zdt_x42s")
    if model not in ("zdt_x42s", "pd42s1"):
        raise ValueError("motor_model 只能为 zdt_x42s 或 pd42s1")
    config = ZDTConfig(**data.get("zdt_x42s", {}))
    return model, config

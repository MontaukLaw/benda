"""鼠标高斯压力输入，独立100Hz线程，可选慢回弹用于观察局限。"""

import math
import threading
import time
from PySide6 import QtCore
from .config import SENSOR_POSITIONS
from .sample_clock import now


class SimulationReader(QtCore.QThread):
    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._lock = threading.Lock()
        self._position = (2.0, 1.5, False)
        self._slow_rebound = False

    def set_position(self, x, y, pressed):
        with self._lock:
            self._position = (x, y, pressed)

    def set_slow_rebound(self, enabled):
        with self._lock:
            self._slow_rebound = enabled

    def stop(self):
        self.requestInterruption()

    def run(self):
        previous = [0.0] * 20
        deadline = now()
        try:
            while not self.isInterruptionRequested():
                with self._lock:
                    x, y, pressed = self._position
                    slow = self._slow_rebound
                values = [0] * 20
                for i, (sx, sy) in SENSOR_POSITIONS.items():
                    pressure = 14000.0 * math.exp(-((x - sx) ** 2 + (y - sy) ** 2) / (2 * 0.65 ** 2)) if pressed else 0.0
                    if slow:
                        pressure = max(pressure, previous[i] * 0.97)
                    previous[i] = pressure
                    values[i] = int(round(12000 + i * 20 + pressure))
                self.engine.update(values, now(), {"protocol": "Simulation", "raw_only": True})
                deadline += 0.01
                delay = deadline - now()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.1:
                    deadline = now()
        finally:
            self.engine.end_source()

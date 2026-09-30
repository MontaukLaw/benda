import time

from config import GUI_REFRESH_HZ
from qt_compat import QtCore, Signal


class RefreshWorker(QtCore.QThread):
    refresh_requested = Signal()

    def __init__(self, refresh_hz=GUI_REFRESH_HZ, parent=None):
        super().__init__(parent)
        self.refresh_hz = refresh_hz
        self._running = False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        interval = 1.0 / float(self.refresh_hz)
        next_tick = time.perf_counter()

        while self._running:
            self.refresh_requested.emit()
            next_tick += interval
            time.sleep(max(0.0, next_tick - time.perf_counter()))

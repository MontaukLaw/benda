from qt_compat import QtCore, Signal


class RecentDataExportThread(QtCore.QThread):
    completed = Signal(object)
    failed = Signal(str)

    def __init__(self, recorder, snapshot, parent=None):
        super().__init__(parent)
        self.recorder = recorder
        self.snapshot = snapshot

    def run(self):
        try:
            result = self.recorder.write_recent_snapshot(self.snapshot)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)

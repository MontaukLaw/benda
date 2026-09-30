"""自动采集记录保存：目标文件占用时接受已完整落盘的独立副本，不冒充K表发布成功。"""
from pathlib import Path
from force_signal_pressure_calibration import _write_json_atomic, ForceSignalPressureCalibrationError


class CalibrationRecordStore:
    def __init__(self, path):
        self.path = Path(path)
        self.saved_path = None
        self.events = []

    def save(self, document):
        try:
            _write_json_atomic(self.path, document)
            self.saved_path = self.path
            return ''
        except ForceSignalPressureCalibrationError as exc:
            recovery_path = getattr(exc, 'recovery_path', None)
            if not recovery_path:
                raise
            self.saved_path = Path(recovery_path)
            message = f'主记录文件暂无法替换，完整数据已保存独立备份，继续标定：{self.saved_path}'
            self.events.append(dict(primary_path=str(self.path), saved_path=str(self.saved_path), reason=str(exc)))
            return message

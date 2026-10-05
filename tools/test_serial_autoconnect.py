"""隔离串口与配置文件，验证COM3启动连接、等待重试及手动停止。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SMOKE = r'''
from qt_compat import QtCore, QtWidgets, Signal
import sensor_window
import serial

# 测试绝不打开真实设备，模拟串口线程的成功和失败信号。
serial.Serial = lambda *a, **kw: (_ for _ in ()).throw(AssertionError('Real serial opened'))
ports = [('COM4', 'COM4 other device'), ('COM3', 'COM3 sensor')]
sensor_window.SerialPortScanner.list_ports = lambda self: (list(ports), '')
attempts = []
fail = False
class Reader(QtCore.QObject):
    state_changed = Signal(bool, str)
    device_info = Signal(object)
    matrix_config_status = Signal(object)
    capsense_channel_config = Signal(object)
    live_noise = Signal(object)
    command_ack = Signal(int)
    finished = Signal()
    def __init__(self, port, buffer, baud, parent, recorder, **kwargs):
        super().__init__(parent)
        self.port = port
    def start(self):
        attempts.append(self.port)
        if fail:
            self.state_changed.emit(False, 'Port busy')
            self.state_changed.emit(False, 'Disconnected')
            self.finished.emit()
        else:
            self.state_changed.emit(True, 'Connected: ' + self.port)
    def stop(self):
        self.state_changed.emit(False, 'Disconnected')
        self.finished.emit()
    def wait(self, *args): return True
sensor_window.SerialReaderThread = Reader
app = QtWidgets.QApplication([])
errors = []
import sys
sys.excepthook = lambda *args: errors.append(args)
def make_window():
    window = sensor_window.MainWindow()
    # 避免本测试中的延迟参数查询需要模拟协议命令。
    window.send_output_mode_command = lambda: None
    window.request_parameters = lambda: None
    window.show()
    app.processEvents()
    return window

window = make_window()
assert attempts == ['COM3']
assert window.current_port == window.port_combo.currentData() == 'COM3'
assert window._connected and window.connect_button.text() == 'Disconnect'
assert not window._startup_connect_timer.isActive()
window.toggle_connection()
window._try_startup_connection()
assert window.reader is None and not window._connected
assert attempts == ['COM3'], 'Manual disconnect must not reconnect'
window.close()

# 只有COM4时等待COM3，不误连；稍后出现COM3自动连接。
ports[:] = [('COM4', 'COM4 other device')]
window = make_window()
assert window.reader is None and attempts == ['COM3']
assert '等待 COM3' in window.status_label.text()
assert window._startup_connect_timer.interval() == 2000
ports.append(('COM3', 'COM3 sensor'))
window._startup_connect_timer.timeout.emit()
assert attempts == ['COM3', 'COM3'] and window._connected
window.close()

# 端口占用时保留错误原因，退出旧线程后重试，不叠加多个reader。
fail = True
window = make_window()
assert window.reader is None and not window._connected
assert 'Port busy' in window.status_label.text()
assert window._startup_connect_timer.isActive()
fail = False
window._startup_connect_timer.timeout.emit()
assert window._connected and not window._startup_connect_timer.isActive()
count = len(attempts)
window._try_startup_connection()
assert len(attempts) == count
window.close()

# 手动选择其他端口并连接会取消启动重试；升级中不创建采集线程。
ports[:] = [('COM4', 'COM4 other device')]
window = make_window()
window.dfu_thread = object()
window._startup_connect_timer.timeout.emit()
assert window.reader is None
window.dfu_thread = None
window.toggle_connection()
assert attempts[-1] == 'COM4' and window._connected
assert not window._startup_connect_pending
window.close()
window = make_window()
window.close()
assert not window._startup_connect_timer.isActive()
count = len(attempts)
window._try_startup_connection()
assert len(attempts) == count
assert not errors, errors
print('COM3 auto-connect: startup, late plug, busy retry, manual disconnect, override, DFU and close passed')
'''


class AutoConnectTests(unittest.TestCase):
    def test_startup_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT / 'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path, directory)
            script = Path(directory) / '_autoconnect_smoke.py'
            script.write_text(SMOKE, encoding='utf-8')
            env = dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONIOENCODING='utf-8')
            result = subprocess.run([sys.executable, str(script)], cwd=directory, env=env,
                                    capture_output=True, text=True, encoding='utf-8', timeout=40)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""隔离验证触摸板UI及逐帧处理，系统鼠标API使用替身，不打开实际串口。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import time
import json
import os
from unittest.mock import patch
from pathlib import Path
from qt_compat import QtWidgets, QtGui, QtCore
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports = lambda self: ([], '')
from sensor_window import MainWindow
app = QtWidgets.QApplication([])
for name in ('msyh.ttc','segoeui.ttf'):
    font = Path('C:/Windows/Fonts')/name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
window = MainWindow()
window.show()
app.processEvents()
window.refresh_timer.stop()
panel = window.touchpad
window.control_tabs.setCurrentWidget(panel)
window.plot_tabs.setCurrentWidget(panel.canvas)
assert not panel.enabled_box.isChecked()
assert not panel.system_mouse.isChecked()
for w in (panel.sensitivity, panel.threshold, panel.cutoff):
    assert not w.prefix() and not w.suffix()
panel.enabled_box.setChecked(True)
panel.reference_button.click()
assert '已开始' in panel.reference_status_label.text()
assert not panel.reference_button.isEnabled()
assert panel.reference_cancel_button.isEnabled()
assert panel.reference_progress.value() == 0
panel.process_samples([], now=panel.not_before+2)
assert '未收到' in panel.reference_detail.text()
base = [8000+123*i for i in range(20)]
clock = [time.perf_counter()+1]
logs = []
panel.log_callback = logs.append

def sample(x=None, state=0):
    clock[0] += .01
    values = [0]*20 if x is None else [1000*max(0,1-abs(i//4+.5-x))*max(0,1-abs(i%4+.5-1.5)) for i in range(20)]
    frame = dict(type='debug', device=1, channel_count=20, raw=[a+b for a,b in zip(base,values)],
                 baseline=base, signal=values, state=[state]*20, fast_k_percent=[0]*20, slow_k_percent=[0]*20)
    with patch('data_buffer.time.perf_counter', return_value=clock[0]):
        window.data_buffer.append_frame(frame)

def flush():
    panel.process_samples(window.data_buffer.pointer_samples_after(panel.last_sequence), now=clock[0])

for i in range(130):
    sample()
    if i%10 == 9: flush()
    if i == 49:
        assert 40 <= panel.reference_progress.value() <= 50
        assert '50 帧' in panel.reference_detail.text()
assert panel.tracker.reference == tuple(base)
assert panel.reference_progress.value() == 100
assert '采集完成' in panel.reference_status_label.text()
assert not panel.reference_cancel_button.isEnabled()
assert panel.reference_button.isEnabled()
assert any(row['reference'] == tuple(base) for row in logs)
assert len(logs) == 130
for i in range(100):
    sample(2,2)
    if i%10 == 9: flush()
assert panel.canvas.cursor == [960,540]
for i in range(100):
    sample(2+.5*(i+1)/100,6)
    if i%10 == 9: flush()
assert panel.canvas.cursor[0] > 1080
assert len(logs) == 330
assert '采集完成' in panel.reference_status_label.text()
old = tuple(panel.canvas.cursor)
flush()
assert tuple(panel.canvas.cursor) == old and len(logs) == 330

# 默认预览不调用鼠标；用户开启后只发送整数位移，Esc立即退出。
with patch.object(QtGui.QCursor,'setPos') as move, patch.object(QtGui.QCursor,'pos',return_value=QtCore.QPoint(100,100)), patch.object(window,'isActiveWindow',return_value=True):
    panel.system_mouse.setChecked(True)
    for i in range(100):
        sample(2.5+.3*(i+1)/100,6)
        flush()
    assert move.call_count > 10
    event = QtGui.QKeyEvent(QtCore.QEvent.Type.KeyPress, QtCore.Qt.Key.Key_Escape, QtCore.Qt.KeyboardModifier.NoModifier)
    window.eventFilter(window,event)
    assert not panel.system_mouse.isChecked()
    count = move.call_count
    for i in range(30):
        sample(2.8+.1*(i+1)/30,6)
        flush()
    assert move.call_count == count

# 保存数值设置，但启用状态、鼠标接管和无触摸参考均不持久化。
panel.sensitivity.setValue(450)
panel.cutoff.setValue(3.5)
window.save_display_config()
config = json.loads(window._config_path.read_text(encoding='utf-8'))
assert config['touchpad'] == dict(sensitivity=450,threshold=100,min_cutoff=3.5)
panel.sensitivity.setValue(250)
window._load_display_config()
assert panel.sensitivity.value() == 450

# 参数回读发生变化时旧参考失效，连接切换同样清除参考。
def parameters(nsub):
    return dict(read_ok=True,channels=[dict(channel=i,configured_nsub=nsub,hardware_nsub=nsub,
        configured_tx_clk_div=100,hardware_tx_clk_div=100,configured_ref_cdac=12,hardware_ref_cdac=12,
        max_raw_count=65535) for i in range(20)])
window.on_capsense_channel_config(parameters(300))
window.on_capsense_channel_config(parameters(301))
assert panel.tracker.reference is None
assert not panel.system_mouse.isChecked()
panel.tracker.reference = tuple(base)
window._set_connected(True)
assert panel.tracker.reference is None
window._set_connected(False)
panel.reference_button.click()
panel.reference_cancel_button.click()
assert not panel.tracker.calibrating
assert '取消' in panel.reference_status_label.text()
assert panel.reference_progress.value() == 0
panel.reference_button.click()
for i in range(130):
    sample()
    if i%10 == 9: flush()
assert '采集完成' in panel.reference_status_label.text()
panel.tracker.message = '参考就绪，滑动可移动光标'
panel._display()
if os.environ.get('TOUCHPAD_SCREENSHOT'):
    window.resize(1700,1000)
    app.processEvents()
    window.grab().save(os.environ['TOUCHPAD_SCREENSHOT'])
window.close()
app.processEvents()
print('Touchpad UI: full-rate queue, calibration, pointer output mock, Esc, persistence and invalidation passed')
'''


class TouchpadUiTests(unittest.TestCase):
    def test_ui_and_pointer_lifecycle(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path,directory)
            script = Path(directory)/'smoke_touchpad.py'
            script.write_text(SCRIPT,encoding='utf-8')
            env = dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result = subprocess.run([sys.executable,str(script)],cwd=directory,env=env,
                                    capture_output=True,text=True,encoding='utf-8',timeout=40)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)


if __name__ == '__main__':
    unittest.main()

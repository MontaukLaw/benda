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
import struct
from unittest.mock import patch
from pathlib import Path
from qt_compat import QtWidgets, QtGui, QtCore
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports = lambda self: ([], '')
from sensor_window import MainWindow
from frame_parser import AlgoFrameParser
app = QtWidgets.QApplication([])
for name in ('msyh.ttc','segoeui.ttf'):
    font = Path('C:/Windows/Fonts')/name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
window = MainWindow()
window.show()
app.processEvents()
window.refresh_timer.stop()
panel = window.touchpad
assert panel.source.currentData() == 'host_signal'
panel.source.setCurrentIndex(panel.source.findData('raw_reference'))
window.control_tabs.setCurrentWidget(panel)
window.plot_tabs.setCurrentWidget(panel.canvas)
assert not panel.enabled_box.isChecked()
assert not panel.force_reference_button.isEnabled()
assert not panel.system_mouse.isChecked()
for w in panel.setting_inputs():
    assert not w.prefix() and not w.suffix()
assert panel.release_drop.value()==25 and panel.release_rise.value()==0
panel.load_settings(dict(source='raw_reference',sensitivity=300,threshold=100,min_cutoff=2.5))
assert panel.release_drop.value()==25 and panel.release_rise.value()==0
panel.enabled_box.setChecked(True)
panel.reference_button.click()
assert '已开始' in panel.reference_status_label.text()
assert not panel.reference_button.isEnabled()
assert panel.force_reference_button.isEnabled()
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
assert config['touchpad'] == dict(source='raw_reference',sensitivity=450,threshold=100,min_cutoff=3.5,
                                    raw_release_drop_percent=25,raw_release_rise_percent=0,
                                    position_reference_seconds=.5,position_noise_floor=20,
                                    position_decay_enabled=True,position_decay_ms=80,position_restore_ms=20,position_decay_percent=60)
panel.sensitivity.setValue(250)
window._load_display_config()
assert panel.sensitivity.value() == 450
# 新阈值立即传入算法并中断当前轨迹，保留参考；仅数值修改也会自动保存。
reference=panel.tracker.reference
panel.motion.remainder=[.8,.8]
panel.release_drop.setValue(15)
panel.release_rise.setValue(5)
assert panel.tracker.raw_release_drop_percent==15 and panel.tracker.raw_release_rise_percent==5
assert not panel.tracker.contact and panel.tracker.reference==reference
assert panel.motion.remainder==[0,0]
loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(650,loop.quit);loop.exec()
saved=json.loads(window._config_path.read_text(encoding='utf-8'))['touchpad']
assert saved['raw_release_drop_percent']==15 and saved['raw_release_rise_percent']==5
panel.release_drop.setValue(30);panel.release_rise.setValue(2)
window._load_display_config()
assert panel.release_drop.value()==15 and panel.release_rise.value()==5
panel.release_drop.setValue(25);panel.release_rise.setValue(0)
window.save_display_config()

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
# 普通采集被未上传状态阻挡时，可在旁边切换为强制采集。
panel.reference_button.click()
sample(state=-1);flush()
assert panel.tracker.reference is None
assert '未上传' in panel.reference_detail.text()
panel.system_mouse.setChecked(True)
panel.force_reference_button.click()
assert not panel.system_mouse.isChecked()
assert '强制采集' in panel.reference_status_label.text()
assert not panel.force_reference_button.isEnabled()
assert panel.reference_cancel_button.isEnabled()
for i in range(130):
    sample(state=-1)
    if i%10 == 9: flush()
assert panel.tracker.reference==tuple(base)
assert panel.reference_progress.value()==100
assert panel.reference_status_label.text()=='强制采集完成'
assert panel.force_reference_button.isEnabled()
assert any(row['reference_status']['forced'] for row in logs)
panel.force_reference_button.click()
panel.reference_cancel_button.click()
assert not panel.tracker.reference_forced and not panel.tracker.calibrating
panel.force_reference_button.click()
for i in range(130):
    sample(state=-1)
    if i%10 == 9: flush()
panel.force_reference_button.click()
def raw_values_sample(values):
    clock[0] += .01
    raw = [round(a+b) for a,b in zip(base,values)]
    wire = struct.pack('<4sBBBBH20H', b'DATA', 21, 0x16, 1, 20, 40, *raw)
    frame = AlgoFrameParser().feed(wire)[0]
    with patch('data_buffer.time.perf_counter', return_value=clock[0]):
        window.data_buffer.append_frame(frame)
def raw_sample(x=None, amplitude=1000):
    values = [0]*20 if x is None else [amplitude*max(0,1-abs(i//4+.5-x))*max(0,1-abs(i%4+.5-1.5)) for i in range(20)]
    raw_values_sample(values)
for i in range(130):
    raw_sample()
    if i%10 == 9: flush()
assert panel.tracker.reference==tuple(base)
assert window.data_buffer.latest_sensor_sample()['raw_only']
cursor_before=tuple(panel.canvas.cursor)
for i in range(100):
    raw_sample(2)
    if i%10 == 9: flush()
assert tuple(panel.canvas.cursor)==cursor_before
for i in range(100):
    raw_sample(2+.5*(i+1)/100)
    if i%10 == 9: flush()
assert panel.canvas.cursor[0]>cursor_before[0]+100
assert panel.tracker.contact and 'Raw观察模式' in panel.status.text()
cursor_after=tuple(panel.canvas.cursor)
raw_sample();flush()
assert tuple(panel.canvas.cursor)==cursor_after and not panel.tracker.contact
assert any(row['raw_only'] and row['dx']>0 for row in logs)
# 柱状图增益同步到算法，改增益立即停鼠标、清参考；高度/数据源变化不清参考。
panel.system_mouse.setChecked(True)
window.array_bar_settings.raw_gain_inputs[5].setValue(2)
window.array_bar_settings.raw_gain_inputs[9].setValue(.5)
window.array_bar_settings.raw_gain_inputs[19].setValue(0)
gains=window.array_bar_settings.settings()['raw_gains']
assert panel.tracker.raw_gains==tuple(gains)
assert panel.tracker.reference is None and not panel.system_mouse.isChecked()
assert '增益' in panel.reference_detail.text()
panel.force_reference_button.click()
for i in range(130):
    raw_sample()
    if i%10==9: flush()
assert panel.tracker.reference==tuple(v*g for v,g in zip(base,gains))
assert any(row['reference']==panel.tracker.reference and row['raw_gains']==tuple(gains)
           and row['algorithm_input']=='raw_times_channel_gain' for row in logs)
reference=panel.tracker.reference
window.array_bar_settings.maximum.setValue(12345)
window.array_bar_settings.source.setCurrentIndex(window.array_bar_settings.source.findData('raw'))
assert panel.tracker.reference==reference
window.array_bars.set_sample(dict(channel_count=20,has_raw=True,raw=base,signal=[0]*20,age_seconds=0))
assert window.array_bars.values==panel.tracker.reference

# 释放门控同时清除滤波和像素小数尾量；系统鼠标与画布在卸载第一帧保持位置。
def unloading_response(main):
    values=[0]*20
    values[5]=values[9]=main
    values[4]=1400
    return values
with patch.object(QtGui.QCursor,'setPos') as move, patch.object(QtGui.QCursor,'pos',return_value=QtCore.QPoint(100,100)), patch.object(window,'isActiveWindow',return_value=True):
    panel.system_mouse.setChecked(True)
    for _ in range(100):
        raw_values_sample(unloading_response(8000));flush()
    assert panel.tracker.contact
    old=tuple(panel.canvas.cursor)
    move.reset_mock()
    panel.motion.remainder=[.8,.8]
    for main in (2000,1000,500,100,0):
        raw_values_sample(unloading_response(main));flush()
        assert panel.tracker.release_blocked and not panel.tracker.contact
        assert panel.tracker.motion_block_reason=='coherent_release_tail'
        assert tuple(panel.canvas.cursor)==old
        assert panel.motion.remainder==[0,0]
        assert all(axis.filtered is None for axis in panel.tracker.axes)
    assert not move.called
panel.stop_mouse()
window.save_display_config()
window.array_bar_settings.raw_gain_inputs[5].setValue(3)
window._load_display_config()
assert panel.tracker.raw_gains==tuple(gains)
assert panel.tracker.reference is None
panel.force_reference_button.click()
raw_sample();flush()
window.array_bar_settings.raw_gain_inputs[5].setValue(4)
assert not panel.tracker.calibrating and panel.tracker.reference is None
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

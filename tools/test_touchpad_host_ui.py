"""从真实 Raw 队列经过实验状态机到触摸板，验证派生质心的逐帧位移。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import json, math, os, time, traceback, sys
from pathlib import Path
from unittest.mock import patch
from qt_compat import QtWidgets, QtGui, QtCore
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports = lambda self: ([], '')
from sensor_window import MainWindow
from centroid_filter import signal_centroid
app = QtWidgets.QApplication([])
for name in ('msyh.ttc', 'segoeui.ttf'):
    font = Path('C:/Windows/Fonts') / name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
errors = []
def hook(*args):
    errors.append(str(args)); traceback.print_exception(*args)
sys.excepthook = hook
window = MainWindow(); window.show(); app.processEvents()
window.refresh_timer.stop(); window._startup_connect_timer.stop()
panel, engine = window.touchpad, window.host_state_machine
window.plot_tabs.setCurrentWidget(panel.canvas)
window.control_tabs.setCurrentWidget(panel)
assert panel.source.currentData() == 'host_signal'
assert not panel.enabled_box.isChecked() and not panel.system_mouse.isChecked()
assert panel.reference_group.isHidden() and not panel.reference_button.isEnabled()
assert window.waveform_source.currentData() == 'firmware'
clock = [time.perf_counter() + 1]
logs, derived, motion_events = [], {}, []
panel.log_callback = logs.append
panel.motion_sample.connect(motion_events.append)
window.array_bar_settings.raw_gain_inputs[5].setValue(2.)
panel.enabled_box.setChecked(True)

def append(x=None, y=1.5, run=False):
    clock[0] += .01
    values = [0] * 20 if x is None else [2000 * max(0, 1-abs(i//4+.5-x)) * max(0, 1-abs(i%4+.5-y)) for i in range(20)]
    frame = dict(type='debug', device=1, channel_count=20, raw=[10000+v for v in values],
                 baseline=[0]*20, signal=[0]*20, state=[-1]*20, raw_only=True,
                 fast_k_percent=[0]*20, slow_k_percent=[0]*20)
    if run: frame = dict(type='signal', device=1, channel_count=20, signal=[65000]*20)
    with patch('data_buffer.time.perf_counter', return_value=clock[0]):
        window.data_buffer.append_frame(frame)

original_feed = engine.feed
def feed_and_capture(sample):
    frame = original_feed(sample)
    if frame is not None: derived[frame['sequence']] = frame
    return frame
engine.feed = feed_and_capture

def flush():
    with patch('sensor_window.time.perf_counter', return_value=clock[0]), patch('touchpad_view.time.perf_counter', return_value=clock[0]):
        window.refresh_plots()

def phase(frames, x=None, y=1.5):
    for index in range(frames):
        append(x, y)
        if index % 10 == 9: flush()

with patch.object(QtGui.QCursor, 'setPos') as move, patch.object(QtGui.QCursor, 'pos', return_value=QtCore.QPoint(100,100)), patch.object(window,'isActiveWindow', return_value=True):
    phase(340)
    assert all(c.state == 0 for c in engine.channels)
    assert panel.tracker.reference is None and not panel.tracker.calibrating
    assert not panel.tracker.contact and len(logs) == 340
    phase(100, 2.)
    assert panel.tracker.contact
    assert math.dist(panel.canvas.cursor, (960., 540.)) < .1
    assert panel.tracker.raw_position == signal_centroid([c.signal for c in engine.channels], local_2x2_enabled=False)
    assert engine.channels[5].signal > engine.channels[9].signal
    previous_cursor = tuple(panel.canvas.cursor)
    for index in range(100):
        append(2. + .4 * (index+1) / 100)
        if index % 10 == 9: flush()
    assert len(logs) == 540 and panel.last_sequence == engine.last_sequence == 540
    assert panel.canvas.cursor[0] > previous_cursor[0] + 60
    assert not move.called
    # 每个触摸板采样使用同序号的实验 Signal，不能把末帧重复用于整个批次。
    moving = logs[-100:]
    assert len({row['raw_xy'] for row in moving if row['raw_xy']}) > 90
    for row in logs:
        assert row['source'] == 'host_signal_centroid' and row['algorithm_input'] == 'host_signal'
        frame = derived.get(row['source_sample_sequence'])
        if frame is not None:
            assert row['host_signal'] == tuple(frame['signal'])
            assert row['host_state'] == tuple(frame['state'])
        if row['contact']:
            assert row['host_adjacent_pairs']
            assert row['raw_xy'] == signal_centroid(row['host_signal'], local_2x2_enabled=False)
        assert math.isclose(row['dx'], row['centroid_dx'] * panel.sensitivity.value(), abs_tol=1e-9)
        assert math.isclose(row['dy'], row['centroid_dy'] * panel.sensitivity.value(), abs_tol=1e-9)
    assert window.data_buffer.latest_sensor_sample()['signal'] == (0.,) * 20
    assert window.data_buffer.latest_sensor_sample()['state'] == (-1,) * 20
    assert any(event['contact'] and event['dx'] > 0 and not event['cancel'] for event in motion_events)
    # 只有明确勾选系统鼠标才发送整数位移；双轴输入都经过同一质心差路径。
    panel.system_mouse.setChecked(True)
    phase(20, 2.4)
    for index in range(100):
        append(2.4, 1.5 + .3 * (index+1)/100)
        if index % 10 == 9: flush()
    assert move.called
    assert any(row['dy'] > 0 for row in logs[-100:])
    assert panel.canvas.cursor[1] > 550
    # 整体卸载时保留画布位置，立即清掉坐标和像素滤波尾量。
    panel.motion.remainder = [.8, .8]
    frozen = None
    for _ in range(30):
        append(); flush()
        if panel.tracker.motion_block_reason in ('no_adjacent_activate_pair', 'coherent_release_tail'):
            if frozen is None:
                frozen = (tuple(panel.canvas.cursor), panel.tracker.position, move.call_count)
            assert tuple(panel.canvas.cursor) == frozen[0]
            assert panel.tracker.position == frozen[1]
            assert move.call_count == frozen[2]
    assert frozen is not None
    old = tuple(panel.canvas.cursor)
    assert not panel.tracker.contact
    assert tuple(panel.canvas.cursor) == old
    assert panel.motion.remainder == [0,0]
    assert all(axis.filtered is None for axis in panel.tracker.axes)
    assert any(row['motion_block_reason'] == 'no_adjacent_activate_pair' for row in logs[-30:])
    assert panel.tracker.centroid_held
    assert '保持' in panel.coordinates.text()
    panel.stop_mouse()
    # 原会话内的新落指重新锚定，不补发抬指前后的位移。
    resume_start = len(logs)
    phase(100, 3.2)
    assert panel.tracker.contact
    first_contact = next(row for row in logs[resume_start:] if row['contact'])
    assert first_contact['dx'] == first_contact['dy'] == 0
    # 基线恢复后的加载形状可稍有变化，但不得补发旧点到新点约一格的差值。
    assert math.dist(panel.canvas.cursor, old) < 5
    old = tuple(panel.canvas.cursor)
    # Raw 未上传时不能用固件 Signal 作为实验输入。
    append(run=True); flush()
    assert not panel.tracker.contact and panel.tracker.position is None
    assert tuple(panel.canvas.cursor) == old
    # 手动重置、增益变更和断流都断开当前轨迹，并停止已勾选的鼠标输出。
    panel.system_mouse.setChecked(True)
    window._reset_host_state('测试重置')
    assert not panel.system_mouse.isChecked() and panel.tracker.position is None
    phase(340); phase(100, 2.)
    assert panel.tracker.contact
    panel.system_mouse.setChecked(True)
    window.array_bar_settings.raw_gain_inputs[5].setValue(1.)
    assert not panel.system_mouse.isChecked() and not panel.tracker.contact
    phase(340); phase(100, 2.)
    assert panel.tracker.contact
    before_gap = tuple(panel.canvas.cursor)
    clock[0] += .3; flush()
    assert not panel.tracker.contact and panel.motion.remainder == [0,0]
    assert tuple(panel.canvas.cursor) == before_gap
    assert all(axis.filtered is None for axis in panel.tracker.axes)
    panel.system_mouse.setChecked(True)
    panel.source.setCurrentIndex(panel.source.findData('raw_reference'))
    assert not panel.system_mouse.isChecked() and panel.tracker.reference is None
    assert not panel.reference_group.isHidden()
    panel.source.setCurrentIndex(panel.source.findData('host_signal'))
    assert panel.reference_group.isHidden()
    phase(340); phase(100, 2.)
    assert panel.tracker.contact
    # 表盘直接使用格单位质心差，不能叠加触摸板的像素灵敏度。
    window.plot_tabs.setCurrentWidget(window.watch_face)
    window.watch_face.timer.stop()
    phase(20, 2.)
    assert window.watch_face.owner == 'sensor'
    offset = tuple(window.watch_face.model.offset)
    for index in range(80):
        append(2. + .2 * (index + 1) / 80)
        if index % 10 == 9: flush()
    assert window.watch_face.model.offset[0] > offset[0] + 30
window.plot_tabs.setCurrentWidget(panel.canvas)
window.control_tabs.setCurrentWidget(panel)
window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['touchpad']['source'] == 'host_signal'
if os.environ.get('TOUCHPAD_HOST_SCREENSHOT'):
    window.resize(1700,1000); app.processEvents()
    window.grab().save(os.environ['TOUCHPAD_HOST_SCREENSHOT'])
window.close(); app.processEvents()
restarted = MainWindow()
restarted.refresh_timer.stop(); restarted._startup_connect_timer.stop()
assert restarted.touchpad.source.currentData() == 'host_signal'
assert not restarted.touchpad.enabled_box.isChecked() and not restarted.touchpad.system_mouse.isChecked()
assert restarted.touchpad.tracker.reference is None
restarted.close(); app.processEvents()
assert not errors, errors
print('Host Signal touchpad: per-sample centroid, dx/dy, gain, release, retouch, reset, mouse mock and persistence passed')
'''


class HostTouchpadUiTests(unittest.TestCase):
    def test_raw_to_host_signal_to_relative_touchpad(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT / 'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path, directory)
            script = Path(directory) / 'host_touchpad_smoke.py'
            script.write_text(SCRIPT, encoding='utf-8')
            result = subprocess.run([sys.executable, str(script)], cwd=directory,
                env=dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONIOENCODING='utf-8'),
                capture_output=True, text=True, encoding='utf-8', timeout=40)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout)


if __name__ == '__main__':
    unittest.main()

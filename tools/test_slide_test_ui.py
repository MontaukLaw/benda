"""隔离完整窗口，模拟采样与提示时钟，不访问串口或系统鼠标。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import json, os, time, zipfile, struct
from pathlib import Path
from unittest.mock import patch
from qt_compat import QtWidgets, QtGui
from serial_ports import SerialPortScanner
from frame_parser import AlgoFrameParser
SerialPortScanner.list_ports = lambda self: ([], '')
from sensor_window import MainWindow
app = QtWidgets.QApplication([])
for name in ('msyh.ttc', 'segoeui.ttf'):
    font = Path('C:/Windows/Fonts') / name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
window = MainWindow()
window._startup_connect_timer.stop()
window.refresh_timer.stop()
window.show()
app.processEvents()
page = window.slide_test
page.timer.stop()
window.plot_tabs.setCurrentWidget(page)
assert window.plot_tabs.tabText(window.plot_tabs.indexOf(page)) == '滑动测试'
page.start()
assert not page.running and not list(page.directory.glob('slide_test_*'))
window.current_port = 'FAKE'
window._set_connected(True)
clock = [time.perf_counter()]
parser = AlgoFrameParser()
def sample(pressed=False):
    raw = [10000] * 20
    if pressed: raw[5] += 2000; raw[6] += 1800
    wire = struct.pack('<4sBBBBH20H', b'DATA', 21, 0x16, 1, 20, 40, *raw)
    frame = parser.feed(wire)[0]
    window.rolling_log.submit('rx', port='FAKE', data=wire)
    window.rolling_log.submit('frame', port='FAKE', frame=frame)
    window.data_buffer.append_frame(frame)
    window.refresh_plots()
def advance(duration, pressed=False):
    for _ in range(round(duration/.05)):
        clock[0] += .05
        sample(pressed)
def await_save():
    deadline = time.monotonic()+5
    while not page.capture.status()['done']:
        assert time.monotonic()<deadline
        time.sleep(.01)
    page.tick()
    assert page.start_button.isEnabled()
    with zipfile.ZipFile(page.capture.status()['archive']) as z:
        return json.loads(z.read('manifest.json')), [json.loads(line) for line in z.read('records.jsonl').splitlines()]
with patch('slide_test_view.time.perf_counter', side_effect=lambda: clock[0]), patch.object(QtGui.QCursor, 'setPos') as mouse:
    advance(3.5)
    assert all(c.state == 0 for c in window.host_state_machine.channels)
    page.horizontal_count.setValue(2)
    page.vertical_count.setValue(1)
    page.horizontal_distance.setValue(25)
    page.notes.setText('固定起终点')
    assert not window.touchpad.enabled_box.isChecked()
    page.start()
    assert page.running and window.touchpad.enabled_box.isChecked()
    assert page.plan[0]['phase'] == 'baseline'
    assert not page.settings_group.isEnabled()
    assert not window.touchpad.system_mouse.isChecked()
    if os.environ.get('SLIDE_TEST_SCREENSHOT'):
        window.resize(1700,1000); app.processEvents()
        window.grab().save(os.environ['SLIDE_TEST_SCREENSHOT'])
    seen = []
    while page.running:
        phase = dict(page.plan[page.index])
        seen.append((phase['axis'], phase['trial'], phase['phase']))
        while clock[0] < page.deadline:
            advance(.05, phase['phase'] == 'slide')
        page.tick()
    meta, records = await_save()
    assert meta['outcome'] == 'completed', meta
    assert not window.touchpad.enabled_box.isChecked()
    slides = [p for p in meta['phases'] if p['phase'] == 'slide']
    assert [(p['axis'], p['trial']) for p in slides] == [('horizontal',1),('horizontal',2),('vertical',1)]
    assert all(p['counts']['sensor_samples'] and p['counts']['touchpad'] and p['counts']['host_state_machine'] for p in slides)
    assert meta['context']['protocol']['horizontal_distance_mm'] == 25
    assert meta['context']['protocol']['notes'] == '固定起终点'
    assert any(r['kind'] == 'touchpad' and r['frame']['dx'] == 0 for r in records)
    assert not mouse.called
    # 中途停止仍保留 ZIP 和全部已采集数据。
    advance(5)
    page.start(); advance(.1); page.stop('用户停止')
    meta, _ = await_save()
    assert meta['outcome'] == 'aborted' and meta['reason'] == '用户停止'
    assert not window.touchpad.enabled_box.isChecked()
    # 断线和参数变化不能伪装成完成。
    page.start(); advance(.1); window._set_connected(False)
    meta, _ = await_save()
    assert meta['outcome'] == 'aborted' and '串口' in meta['reason']
    window._set_connected(True); advance(3.5)
    page.start(); advance(.1); window.touchpad.sensitivity.setValue(500); page.tick()
    meta, _ = await_save()
    assert meta['outcome'] == 'aborted' and '变化' in meta['reason']
    advance(.6); page.start()
    assert page.running
    clock[0] += 2
    page.tick()
    meta, _ = await_save()
    assert meta['outcome'] == 'aborted' and '采样中断' in meta['reason']
window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['slide_test']['horizontal_count'] == 2
window._set_connected(False)
window.close()
while window.rolling_log.is_alive(): time.sleep(.01)
app.processEvents()
restarted = MainWindow()
restarted._startup_connect_timer.stop(); restarted.refresh_timer.stop()
assert restarted.slide_test.horizontal_count.value() == 2
assert restarted.slide_test.horizontal_distance.value() == 25
assert restarted.slide_test.notes.text() == '固定起终点'
assert not restarted.slide_test.running
restarted.close(); app.processEvents()
print('Slide test: ordered prompts, same-frame capture, ZIP, abort, disconnect, configuration, stale sample and persistence passed')
'''


class SlideTestUiTests(unittest.TestCase):
    def test_guided_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path, directory)
            script = Path(directory)/'slide_test_smoke.py'
            script.write_text(SCRIPT, encoding='utf-8')
            result = subprocess.run([sys.executable, str(script)], cwd=directory,
                env=dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONIOENCODING='utf-8'),
                capture_output=True, text=True, encoding='utf-8', timeout=45)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            print(result.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)

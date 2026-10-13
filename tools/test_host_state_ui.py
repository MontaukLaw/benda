"""隔离上位机实验 UI，保留真实固件字段且不访问串口或鼠标。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import json
import math
import os
import sys
import time
from unittest.mock import patch
from pathlib import Path
from qt_compat import QtWidgets, QtGui, QtCore
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports = lambda self: ([], '')
from sensor_window import MainWindow, CHANNEL_TABLE_COLUMN_INDEX
from host_state_view import PARAMETER_GROUPS, JSON_ONLY_PARAMETERS
app = QtWidgets.QApplication([])
paint_errors = []
def report_paint_error(*args):
    import traceback
    paint_errors.append(args)
    traceback.print_exception(*args)
sys.excepthook = report_paint_error
for name in ('msyh.ttc', 'segoeui.ttf'):
    font = Path('C:/Windows/Fonts') / name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
window = MainWindow()
window.show()
app.processEvents()
window.refresh_timer.stop()
window._startup_connect_timer.stop()
panel = window.host_state_settings
engine = window.host_state_machine
window.control_tabs.setCurrentWidget(panel)
assert window.control_tabs.tabText(window.control_tabs.indexOf(panel)) == '基线与状态机'
assert set(panel.inputs) == set(engine.parameters.settings()) - JSON_ONLY_PARAMETERS
assert JSON_ONLY_PARAMETERS == {'normalization_floor', 'gap_seconds'}
groups = panel.findChildren(QtWidgets.QGroupBox)
assert [group.title() for group in groups] == [title for title, _ in PARAMETER_GROUPS]
for group, (_, keys) in zip(groups, PARAMETER_GROUPS):
    assert all(group.isAncestorOf(panel.inputs[key]) for key in keys)
assert not any('上位机实验 · 20 通道独立状态' in label.text() for label in panel.findChildren(QtWidgets.QLabel))
assert not panel.findChildren(QtWidgets.QTableWidget)
assert window.table.rowCount() == 20
assert panel.settings()['input_alpha'] == .2
assert panel.settings()['signal_alpha'] == .1
assert not panel.signal_filtered_box.isChecked()
assert panel.settings()['initial_seconds'] == 3
assert panel.inputs['slowback_tolerance_percent'].value() == 10
assert panel.inputs['slowback_close_frames'].value() == 3
assert panel.inputs['reload_reference_margin'].value() == 300
assert all(not w.prefix() and not w.suffix() for w in panel.inputs.values())
# 手动 JSON 设置的隐藏值在之后的应用、保存和重启中必须保留。
panel.load_settings(dict(panel.settings(), normalization_floor=7.5, gap_seconds=.35))
assert not window.touchpad.enabled_box.isChecked()
assert panel.apply_button.text() == panel.bottom_apply_button.text() == '确认生效并保存'
reason_col = CHANNEL_TABLE_COLUMN_INDEX['activation_reason']
assert reason_col == window.table.columnCount() - 1
assert not window.table.isColumnHidden(reason_col)
clock = [time.perf_counter() + 1]

def sample(value=10000, raw_only=False):
    clock[0] += .01
    frame = dict(type='debug', device=1, channel_count=20, raw=[value]*20,
                 baseline=[9000]*20, signal=[value-9000]*20, state=[6]*20,
                 fast_k_percent=[123]*20, slow_k_percent=[456]*20, raw_only=raw_only)
    if raw_only:
        frame.update(baseline=[0]*20, signal=[0]*20, state=[-1]*20)
    with patch('data_buffer.time.perf_counter', return_value=clock[0]):
        window.data_buffer.append_frame(frame)

def refresh():
    with patch('sensor_window.time.perf_counter', return_value=clock[0]):
        window.refresh_plots()

window.array_bar_settings.raw_gain_inputs[0].setValue(2)
for index in range(340):
    sample()
    if index % 20 == 19: refresh()
assert engine.channels[0].input == 20000
assert engine.channels[0].state == 0
assert len(engine.histories['raw'][0]) == 340
assert engine.last_sequence == 340
before = engine.version
refresh()
assert engine.version == before
for index in range(40):
    sample(14000)
    if index % 20 == 19: refresh()
assert engine.channels[0].state == 1
assert engine.channels[0].baseline == 20000
firmware = window.data_buffer.snapshot()
assert firmware[0][0][-1] == 14000 and firmware[1][0][-1] == 9000
assert firmware[5][0] == 6
assert window.value_labels['state'][0].text() != 'activate'
assert window.value_labels['activation_reason'][0].text() == '--'
# 柱状图独立选择派生 Signal，无需改变波形来源，也不再次乘 Raw 增益。
bars = window.array_bars
bar_panel = window.array_bar_settings
bar_panel.source.setCurrentIndex(bar_panel.source.findData('host_signal'))
assert window.waveform_source.currentData() == 'firmware'
assert bars.values == tuple(c.signal for c in engine.channels)
assert bars.values[0] == engine.channels[0].input - 20000
assert '上位机计算 Signal' in bars._message
assert 'host_signal' not in window.data_buffer.latest_sensor_sample()
assert bar_panel.maximum.value() == 20000
# 在快照返回后模拟接收线程立即追加一帧，显示仍必须使用已处理的同帧结果。
import threading
sample(14000)
captured_sequence = window.data_buffer.latest_sensor_sample()['sequence']
original_snapshot = window.data_buffer.display_snapshot_after
injections = []

def receive_after_snapshot(sequence):
    result = original_snapshot(sequence)
    writer = threading.Thread(target=lambda: sample(14000))
    writer.start();writer.join(1)
    assert not writer.is_alive()
    injections.append(True)
    return result

with patch.object(window.data_buffer, 'display_snapshot_after', side_effect=receive_after_snapshot):
    refresh()
assert injections
assert window.data_buffer.latest_sensor_sample()['sequence'] == captured_sequence + 1
assert bars._sample['sequence'] == engine.last_sequence == captured_sequence
assert bars.values is not None and max(bars.values) > 0
assert bars.values == tuple(c.signal for c in engine.channels)
refresh()
assert engine.last_sequence == bars._sample['sequence'] == captured_sequence + 1
bar_panel.maximum.setValue(100000)
assert bars.height_limit() == 100000
bar_panel.source.setCurrentIndex(bar_panel.source.findData('raw'))
assert bar_panel.maximum.value() == 65535
assert bars.values[0] == 28000
bar_panel.source.setCurrentIndex(bar_panel.source.findData('signal'))
assert bar_panel.maximum.value() == 20000
assert bars.values[0] == 5000
bar_panel.source.setCurrentIndex(bar_panel.source.findData('host_signal'))
assert bar_panel.maximum.value() == 100000
bar_panel.auto_range.setChecked(True)
assert bars.height_limit() >= max(bars.values)
bar_panel.auto_range.setChecked(False)
assert bars.height_limit() == 100000
panel.waveform_button.click()
refresh()
assert window.waveform_source.currentData() == 'host'
assert not window.host_state_plot.isVisible()
assert window.value_labels['state'][0].text() == 'activate'
reason_text = window.value_labels['activation_reason'][0].text()
reason_detail = window.value_labels['activation_reason'][0].toolTip()
assert 'idle' in reason_text and '差值触发' in reason_text and '3帧' in reason_text
assert 'input' in reason_detail and 'baseline' in reason_detail and '3/3' in reason_detail
assert window.curves['raw'][0].getData()[1][-1] == engine.channels[0].input
assert window.curves['baseline'][0].getData()[1][-1] == 20000
assert window.curves['signal'][0].getData()[1][-1] == engine.channels[0].signal
# 全窗口包含初始化的无效速度和按压上升沿；极值只统计有效采样。
window.raw_plot.setXRange(0, len(engine.histories['fast'][0]) - 1, padding=0.)
app.processEvents()
valid_speeds = [v for v in engine.histories['fast'][0] if math.isfinite(v)]
assert max(valid_speeds) > 0 and min(valid_speeds) == 0
for name, expected in (('最大', max(valid_speeds)), ('最小', min(valid_speeds))):
    line = window.speed_extrema_lines[name]
    assert line.isVisible() and abs(line.value() - expected) < 1e-8
    assert line.pen.style() == QtCore.Qt.PenStyle.DashLine
    assert 'CH0' in line.label.format and '%/s' in line.label.format
# 缩放到按压前的空载部分，旧上升峰必须离开统计窗口。
window.raw_plot.setXRange(40, 120, padding=0.)
app.processEvents()
assert all(line.isVisible() and line.value() == 0 for line in window.speed_extrema_lines.values())
window.plot_checkboxes['fast'][0].setChecked(False)
assert all(not line.isVisible() for line in window.speed_extrema_lines.values())
window.plot_checkboxes['fast'][0].setChecked(True)
app.processEvents()
assert all(line.isVisible() for line in window.speed_extrema_lines.values())
# 可见范围完全没有采样点时隐藏，不沿用旧极值。
window.raw_plot.setXRange(1000, 1100, padding=0.)
app.processEvents()
assert all(not line.isVisible() for line in window.speed_extrema_lines.values())
window.auto_range()
app.processEvents()
# 显示开关同时切换波形、表格及柱状图，但不重算状态或触摸参考。
before_generation = engine.generation
before_version = engine.version
before_touchpad_sequence = window.touchpad.last_sequence
before_state = tuple(c.state for c in engine.channels)
panel.signal_filtered_box.setChecked(True)
refresh()
assert engine.generation == before_generation and engine.version == before_version
assert window.touchpad.last_sequence == before_touchpad_sequence
assert tuple(c.state for c in engine.channels) == before_state
assert bars.values == tuple(c.signal_filtered for c in engine.channels)
assert '低通后' in bars._message
assert window.curves['signal'][0].getData()[1][-1] == engine.channels[0].signal_filtered
# 极小的真实滤波尾值仍保留，自动量程与手动缩放不能让刻度间距下溢成零。
saved_signal_data = window.curves['signal'][0].getData()
signal_view = window.signal_plot.getViewBox()
signal_axis = window.signal_plot.getAxis('left')
for tail in ([0., 0.], [5e-324, 1e-322], [2.5e-323, 2.5e-323], [0., 12000.]):
    window.curves['signal'][0].setData([0, 1], tail)
    window.auto_range()
    signal_view.updateAutoRange()
    app.processEvents()
    low, high = signal_view.viewRange()[1]
    assert math.isfinite(low) and math.isfinite(high) and high - low >= 1.
    assert all(spacing > 0 for spacing, _ in signal_axis.tickValues(low, high, 200))
    assert list(window.curves['signal'][0].getData()[1]) == tail
    window.grab() # 强制真实 Qt 绘图，异常须由 excepthook 捕获。
signal_view.setYRange(0., 1e-322, padding=0.)
app.processEvents()
assert signal_view.viewRange()[1][1] - signal_view.viewRange()[1][0] >= 1.
assert not paint_errors, paint_errors
window.curves['signal'][0].setData(*saved_signal_data)
window.auto_range()
assert window.value_labels['signal_value'][0].text() == f'{engine.channels[0].signal_filtered:.0f}'
assert '低通后' in window.table.horizontalHeaderItem(CHANNEL_TABLE_COLUMN_INDEX['signal_value']).text()
assert window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['host_signal_filtered_display'] is True
panel.signal_filtered_box.setChecked(False)
refresh()
assert bars.values == tuple(c.signal for c in engine.channels)
assert window.curves['signal'][0].getData()[1][-1] == engine.channels[0].signal
assert '低通前' in bars._message
assert window.table.horizontalHeaderItem(CHANNEL_TABLE_COLUMN_INDEX['signal_value']).text() == 'Sig Val'
assert math.isfinite(window.curves['fast'][0].getData()[1][-1])
assert window.host_state_curve.getData()[1][-1] == 1
assert 'Input' in window.table.horizontalHeaderItem(3).text()
assert window._state_regions
# 两个观察入口及原背景入口均必须联动四条曲线和状态，且不重置算法。
before_generation = engine.generation
before_sequence = engine.last_sequence
panel.observation_channel.setCurrentIndex(19)
refresh()
assert window.host_observation_channel.currentData() == 19
assert window.state_background_channel_combo.currentData() == 19
for key in ('raw','baseline','signal','fast'):
    assert [i for i,c in enumerate(window.curves[key]) if c.isVisible()] == [19]
assert window.curves['raw'][19].getData()[1][-1] == engine.channels[19].input
assert window.host_state_curve.getData()[1][-1] == engine.channels[19].state
app.processEvents()
assert all('CH19' in line.label.format for line in window.speed_extrema_lines.values()), [(line.isVisible(), line.label.format) for line in window.speed_extrema_lines.values()]
assert engine.generation == before_generation and engine.last_sequence == before_sequence
window.state_background_channel_combo.setCurrentIndex(12)
refresh()
assert panel.observation_channel.currentData() == 12
assert window.curves['fast'][12].isVisible() and not window.curves['fast'][19].isVisible()
window.host_observation_channel.setCurrentIndex(7)
refresh()
assert panel.observation_channel.currentData() == 7
assert window.state_background_channel_combo.currentData() == 7
assert window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['host_observation_channel'] == 7
# 老配置中的自定义列顺序须保留，新诊断列追加在最右侧。
legacy = json.loads(window._config_path.read_text(encoding='utf-8'))
legacy_order = [key for key in legacy['order'] if key != 'activation_reason']
legacy_order.remove('signal_value')
legacy_order.insert(0, 'signal_value')
legacy['order'] = legacy_order
legacy['columns'].pop('activation_reason')
window._config_path.write_text(json.dumps(legacy), encoding='utf-8')
panel.observation_channel.setCurrentIndex(1)
window._load_display_config()
refresh()
assert window.column_order == legacy_order + ['activation_reason']
assert window.table.horizontalHeader().visualIndex(reason_col) == window.table.columnCount() - 1
assert panel.observation_channel.currentData() == 7
assert window.curves['signal'][7].isVisible() and not window.curves['signal'][1].isVisible()
panel.observation_channel.setCurrentIndex(0)
refresh()
for _ in range(15): sample(8000)
refresh()
assert engine.channels[0].state == 3
assert bars.values == tuple(c.signal for c in engine.channels)
assert bars.values == (0.,) * 20
assert '激活0/20' in bars._message
assert window.curves['signal'][0].getData()[1][-1] == 0
assert window.value_labels['state'][0].text() == 'recover'
assert window.host_state_curve.getData()[1][-1] == 3
assert math.isfinite(window.curves['fast'][0].getData()[1][-1])
assert window.value_labels['fast_k_value'][0].text() == f'{engine.channels[0].rate:.2f}'
app.processEvents()
assert window.speed_extrema_lines['最小'].value() < 0
assert window.speed_extrema_lines['最大'].value() > 0
assert window.value_labels['activation_reason'][0].text() == reason_text
assert window.value_labels['activation_reason'][0].toolTip() == reason_detail
panel.signal_filtered_box.setChecked(True)
refresh()
assert engine.channels[0].signal == 0 and engine.channels[0].signal_filtered > 0
assert bars.values == tuple(c.signal_filtered for c in engine.channels)
assert window.value_labels['state'][0].text() == 'recover'
assert window.curves['signal'][0].getData()[1][-1] == engine.channels[0].signal_filtered
panel.signal_filtered_box.setChecked(False)
refresh()
window.waveform_source.setCurrentIndex(0)
refresh()
assert not window.host_state_plot.isVisible()
assert all(not line.isVisible() for line in window.speed_extrema_lines.values())
assert window.curves['raw'][0].getData()[1][-1] == 8000
assert window.curves['baseline'][0].getData()[1][-1] == 9000
assert window.data_buffer.snapshot()[5][0] == 6
assert window.value_labels['activation_reason'][0].text() == '--'
assert '固件协议未上报' in window.value_labels['activation_reason'][0].toolTip()
# 真正重新按压后，监控原因须显示新增加的卸载前参考及余量证据。
panel.waveform_button.click()
for _ in range(15): sample(16000)
refresh()
assert engine.channels[0].state == 1
assert engine.channels[0].last_activation['reason'] == 'reload'
assert engine.channels[0].last_activation['release_reference'] == 20000
reload_detail = window.value_labels['activation_reason'][0].toolTip()
assert '卸载前参考基线' in reload_detail and '超出参考门限 300.000' in reload_detail
panel.inputs['release_threshold'].setValue(300)
before = engine.generation
assert not panel.apply()
assert engine.generation == before and engine.parameters.release_threshold == 50
assert '未应用' in panel.message.text()
panel.inputs['release_threshold'].setValue(50)
panel.inputs['drop_rate'].setValue(15)
panel.inputs['slowback_frames'].setValue(200)
panel.inputs['slowback_tolerance_percent'].setValue(7.5)
panel.inputs['slowback_close_frames'].setValue(4)
sample() # 尚未被GUI消费的旧帧不得进入新初始化。
assert panel.apply()
assert not engine.histories['raw'][0]
assert bars.values is None
assert all(not line.isVisible() for line in window.speed_extrema_lines.values())
refresh()
assert engine.channels[0].started is None
assert engine.parameters.drop_rate == 15
assert engine.parameters.slowback_frames == 200
assert engine.parameters.slowback_tolerance_percent == 7.5
assert engine.parameters.slowback_close_frames == 4
# apply返回时必须已经写盘，无需等自动保存定时器。
assert not window._save_timer.isActive()
assert '已生效并保存' in panel.message.text()
assert panel.message.text() == panel.bottom_message.text()
saved = json.loads(window._config_path.read_text(encoding='utf-8'))
assert saved['host_state_machine']['drop_rate'] == 15
assert saved['host_state_machine']['slowback_frames'] == 200
assert saved['host_state_machine']['slowback_tolerance_percent'] == 7.5
assert saved['host_state_machine']['slowback_close_frames'] == 4
# 最后一个仍在编辑的文本也要被提交。
panel.inputs['drop_rate'].lineEdit().setText('30')
panel.apply_button.click()
assert engine.parameters.drop_rate == 30
assert json.loads(window._config_path.read_text(encoding='utf-8'))['host_state_machine']['drop_rate'] == 30
window._load_display_config()
assert engine.parameters.drop_rate == 30
assert panel.inputs['drop_rate'].value() == 30
# 原子替换失败不能宣称成功、不能应用新参数、不能覆盖旧配置。
before_parameters = engine.parameters
before_generation = engine.generation
before_file = window._config_path.read_text(encoding='utf-8')
panel.inputs['drop_rate'].setValue(15)
panel.inputs['window'].setValue(40)
panel.inputs['reload_threshold'].setValue(800)
panel.inputs['reload_reference_margin'].setValue(650)
panel.inputs['signal_alpha'].setValue(.25)
with patch.object(Path, 'replace', side_effect=PermissionError('test write denied')):
    assert not panel.apply()
assert engine.parameters == before_parameters and engine.generation == before_generation
assert window._config_path.read_text(encoding='utf-8') == before_file
assert '未保存' in panel.bottom_message.text() and 'test write denied' in panel.bottom_message.text()
assert panel.inputs['window'].value() == 40
panel.bottom_apply_button.click()
assert engine.parameters.drop_rate == 15 and engine.parameters.window == 40
assert engine.parameters.reload_threshold == 800
assert engine.parameters.reload_reference_margin == 650
assert engine.parameters.signal_alpha == .25
assert '已生效并保存' in panel.bottom_message.text()
assert '实验参数已保存' in window.status_label.text() and '失败' not in window.status_label.text()
confirmed_settings = panel.settings()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['host_state_machine'] == confirmed_settings
panel.signal_filtered_box.setChecked(True)
sample()
refresh()
assert engine.channels[0].input == 20000
assert bars.values == (0.,) * 20 and '初始化' in bars._message
sample(14000)
window.array_bar_settings.raw_gain_inputs[0].setValue(3)
assert bars.values is None
refresh()
assert engine.channels[0].started is None
sample()
refresh()
assert engine.channels[0].input == 30000
assert engine.channels[0].state == -1
sample(raw_only=True)
refresh()
assert engine.channels[0].input == 30000
assert window.data_buffer.snapshot()[5][0] == -1
window.waveform_source.setCurrentIndex(1)
clock[0] += .01
with patch('data_buffer.time.perf_counter', return_value=clock[0]):
    window.data_buffer.append_frame(dict(type='signal', device=1, channel_count=20, signal=[20]*20))
refresh()
assert not engine.histories['raw'][0]
assert 'Run' in engine.reason
assert bars.values is None and 'Debug' in bars._message
sample()
refresh()
assert engine.channels[0].input == 30000 and engine.channels[0].state == -1
assert bars.values == (0.,) * 20
clock[0] += engine.parameters.gap_seconds + .01
refresh()
assert bars.values is None and '中断' in bars._message
assert all(not line.isVisible() for line in window.speed_extrema_lines.values())
sample()
refresh()
assert bars.values == (0.,) * 20
# 模拟真正的连接切换，核对配置记录可写且固件数据不变。
events = []
with patch.object(window.rolling_log, 'submit', side_effect=lambda kind, **kw: events.append((kind,kw))):
    window._set_connected(True)
    sample()
    refresh()
    window._set_connected(False)
assert any(kind == 'host_state_config' for kind,kw in events)
assert any(kind == 'host_state_machine' for kind,kw in events)
assert engine.channels[0].started is None
assert window.data_buffer.snapshot()[0][0][-1] == 10000
if os.environ.get('HOST_STATE_SCREENSHOT'):
    window.resize(1700,1000)
    window._apply_font_size(16)
    window.control_tabs.setCurrentWidget(panel)
    panel.waveform_button.click()
    for index in range(340):
        sample()
        if index % 20 == 19: refresh()
    for index in range(40):
        sample(14000)
        if index % 20 == 19: refresh()
    app.processEvents()
    window.grab().save(os.environ['HOST_STATE_SCREENSHOT'])
    panel.verticalScrollBar().setValue(panel.verticalScrollBar().maximum())
    app.processEvents()
    window.grab().save(os.environ['HOST_STATE_SCREENSHOT'].replace('.png','_states.png'))
    window.control_tabs.setCurrentIndex(0)
    window.table.scrollToItem(window.table.item(0, 0))
    window.table.horizontalScrollBar().setValue(window.table.horizontalScrollBar().maximum())
    app.processEvents()
    window.grab().save(os.environ['HOST_STATE_SCREENSHOT'].replace('.png','_reasons.png'))
    window.plot_tabs.setCurrentWidget(bars)
    app.processEvents()
    window.grab().save(os.environ['HOST_STATE_SCREENSHOT'].replace('.png','_bars.png'))
window.close()
app.processEvents()
restarted = MainWindow()
restarted.refresh_timer.stop()
restarted._startup_connect_timer.stop()
assert restarted.host_state_settings.settings() == confirmed_settings
assert confirmed_settings['normalization_floor'] == 7.5
assert confirmed_settings['gap_seconds'] == .35
assert restarted.host_state_settings.inputs['window'].value() == 40
assert restarted.host_state_settings.inputs['reload_threshold'].value() == 800
assert restarted.host_state_settings.inputs['reload_reference_margin'].value() == 650
assert restarted.host_state_settings.inputs['signal_alpha'].value() == .25
assert restarted.host_state_settings.signal_filtered_box.isChecked()
restarted._show_host_waveforms()
app.processEvents()
assert restarted.host_state_plot.isHidden()
assert restarted.signal_plot.getViewBox().state['limits']['yRange'][0] == 1.
assert restarted.host_state_settings.inputs['slowback_tolerance_percent'].value() == 7.5
assert restarted.host_state_settings.inputs['slowback_close_frames'].value() == 4
assert restarted.array_bar_settings.source.currentData() == 'host_signal'
assert restarted.array_bar_settings.maximum.value() == 100000
assert restarted.array_bars.values is None
restarted.close()
app.processEvents()
assert not paint_errors, paint_errors
print('Host UI: full-rate processing, independent firmware data, plots, config, gain/session reset passed')
'''


class HostStateUiTests(unittest.TestCase):
    def test_state_machine_ui(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT / 'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path, directory)
            script = Path(directory) / 'smoke_host_state.py'
            script.write_text(SCRIPT, encoding='utf-8')
            result = subprocess.run([sys.executable, str(script)], cwd=directory,
                                    env=dict(os.environ, QT_QPA_PLATFORM='offscreen', PYTHONIOENCODING='utf-8'),
                                    capture_output=True, text=True, encoding='utf-8', timeout=40)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout)


if __name__ == '__main__':
    unittest.main()

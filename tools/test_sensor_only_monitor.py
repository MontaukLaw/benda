"""隔离运行纯传感器窗口，验证无电机/CAN加载和20通道V21显示。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SMOKE = r'''
import importlib.abc
import sys
import struct
import os
from pathlib import Path
class NoMotorCan(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0].startswith(('can', 'motor_', 'zdt_', 'pressure_meter_', 'pressure_feedback_', 'force_press_', 'one_step_')):
            raise AssertionError('Unexpected hardware module: '+fullname)
sys.meta_path.insert(0, NoMotorCan())
from main_window import MainWindow, CHANNEL_TABLE_COLUMN_INDEX
from serial_ports import SerialPortScanner
from qt_compat import QtCore, QtGui, QtWidgets
from frame_parser import AlgoFrameParser
from centroid_view import CHANNEL_GRID
import serial
serial.Serial = lambda *a, **kw: (_ for _ in ()).throw(AssertionError('Startup must not open any port'))
SerialPortScanner.list_ports = lambda self: ([('COM4','COM4 TEST')], '')
app = QtWidgets.QApplication([])
for name in ('msyh.ttc','segoeui.ttf'):
    font = Path('C:/Windows/Fonts')/name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
errors=[]
def report_exception(*args):
    import traceback
    errors.append(args)
    traceback.print_exception(*args)
sys.excepthook=report_exception
window=MainWindow()
window.show()
app.processEvents()
assert [window.control_tabs.tabText(i) for i in range(window.control_tabs.count())] == ['监控','标定','波形设置','三参数','串口日志','固件升级','触摸板','表盘设置','柱状图设置','基线与状态机']
graphs = window.main_splitter.widget(0)
assert [window.plot_tabs.tabText(i) for i in range(window.plot_tabs.count())] == ['波形图','阵列质心','触摸板光标','滑动测试','圆形表盘','3D柱状图']
assert window.plot_tabs.currentWidget() is window.watch_face
assert window.centroid_view.centroid is None
assert CHANNEL_GRID == ((0,4,8,12,16),(1,5,9,13,17),(2,6,10,14,18),(3,7,11,15,19))
assert window.centroid_scale_slider.value() == 70
assert window.control_tabs.isAncestorOf(window.centroid_scale_slider)
assert window.centroid_threshold_input.value() == 100
assert window.centroid_local_2x2_checkbox.isChecked()
assert not window.centroid_raw_checkbox.isChecked()
assert not window.centroid_raw_difference_checkbox.isChecked()
assert window.control_tabs.isAncestorOf(window.centroid_raw_difference_checkbox)
assert window.control_tabs.isAncestorOf(window.centroid_raw_checkbox)
assert window.control_tabs.isAncestorOf(window.centroid_local_2x2_checkbox)
assert window.centroid_view.position_filter.signal_threshold == 100
assert window.control_tabs.widget(0).isAncestorOf(window.centroid_threshold_input)
window.centroid_scale_slider.setValue(100)
full_grid,_=window.centroid_view.grid_geometry()
window.centroid_scale_slider.setValue(50)
small_grid,_=window.centroid_view.grid_geometry()
assert abs(small_grid.width()*2-full_grid.width()) < 1e-6
assert small_grid.center() == full_grid.center()
assert window.centroid_scale_label.text() == '50%'
window.save_display_config()
import json
assert json.loads(window._config_path.read_text(encoding='utf-8'))['centroid_scale'] == 50
window.centroid_scale_slider.setValue(70)
window._load_display_config()
assert window.centroid_scale_slider.value() == window.centroid_view.scale_percent == 50
window.centroid_scale_slider.setValue(70)
assert all(window.slide_test.isAncestorOf(widget) for widget in graphs.findChildren(QtWidgets.QSpinBox))
assert [widget for widget in graphs.findChildren(QtWidgets.QComboBox) if not window.slide_test.isAncestorOf(widget)] == [window.waveform_source, window.host_observation_channel]
assert window.control_tabs.isAncestorOf(window.port_combo)
assert window.control_tabs.isAncestorOf(window.history_input)
assert window.control_tabs.isAncestorOf(window.nsub_input)
assert window.reader is None and window.table.rowCount() == 20
from font_scale import scaled_font_size
assert window._ui_font_px == 13
original_height = window.table.rowHeight(0)
for _ in range(5): window.font_larger_button.click()
app.processEvents()
assert window._ui_font_px == window.live_status.font().pixelSize() == 18
assert window.table.rowHeight(0) > original_height
assert scaled_font_size(window.array_bars,12) == 17
assert window.raw_plot.getAxis('left').style['tickFont'].pixelSize() == 18
assert window.raw_plot.getAxis('left').labelText == 'count'
assert window.raw_plot.getAxis('bottom').labelText == '采样帧'
window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['ui_font_px'] == 18
window.font_smaller_button.click()
window._load_display_config()
assert window._ui_font_px == 18
for _ in range(5): window.font_smaller_button.click()
app.processEvents()
assert window._ui_font_px == 13 and window.table.rowHeight(0) == original_height
window.save_display_config()
assert len(window.table_row_buttons) == 20
assert 'force_sum_select' not in CHANNEL_TABLE_COLUMN_INDEX
assert len(window.curves['signal']) == 20
for widget in window.findChildren(QtWidgets.QSpinBox):
    assert not widget.prefix() and not widget.suffix()
# 首尾通道都必须显示实际采样；Run缺少诊断时显示未上传。
payload=struct.pack('<20H',*range(2000,2020))+struct.pack('<20H',*range(1900,1920))
payload+=struct.pack('<20H',*([100]*20))+struct.pack('<20i',*([-125]*20))+struct.pack('<20i',*([250]*20))
payload+=struct.pack('<20H',*([6]*20))
wire=struct.pack('<4sBBBBH',b'DATA',21,0x10,1,20,len(payload))+payload
frame=AlgoFrameParser().feed(wire)[0]
window.data_buffer.append_frame(frame)
window.data_recorder.record(frame)
window.state_background_buttons[19].setChecked(True)
window.refresh_plots()
assert window.value_labels['hardware_raw_value'][19].text() == '2019'
assert window.value_labels['baseline_value'][19].text() == '1919'
assert window.value_labels['signal_value'][19].text() == '100'
assert window.value_labels['state'][19].text() == 'FLOATING'
assert window.state_background_channel_combo.currentData() == 19
window.table_row_buttons['channel_19'].setChecked(False)
assert window.table.isRowHidden(19)
window.table_row_buttons['channel_19'].setChecked(True)
window._reorder_column('signal_value','state',False)
assert window.table.horizontalHeader().logicalIndex(1) == CHANNEL_TABLE_COLUMN_INDEX['signal_value']
window.plot_checkboxes['raw'][19].setChecked(True)
window.refresh_plots()
assert window.curves['raw'][19].getData()[1][-1] == 2019
# 编辑输入框不提前应用或被其他设置的自动保存带入；设置按钮才应用并持久化。
window.centroid_threshold_input.setValue(250)
window.save_display_config()
assert window.centroid_view.position_filter.signal_threshold == 100
assert json.loads(window._config_path.read_text(encoding='utf-8'))['centroid_signal_threshold'] == 100
window.centroid_threshold_button.click()
assert window.centroid_view.centroid is None
assert window.centroid_view.position_filter.threshold_hidden
assert window.centroid_view.raw_centroid == (1.0,1.0)
assert json.loads(window._config_path.read_text(encoding='utf-8'))['centroid_signal_threshold'] == 250
window.centroid_threshold_input.setValue(300)
window._load_display_config()
assert window.centroid_threshold_input.value() == 250
assert window.centroid_view.position_filter.signal_threshold == 250
window.centroid_threshold_input.setValue(100)
window.centroid_threshold_button.click()
# 用实际数据缓冲刷新验证方向、加权、隐藏通道和Run模式；全零及断流隐藏红点。
assert window.centroid_view.centroid == (1.0,1.0)
window.table_row_buttons['channel_19'].setChecked(False)
for channel, position in ((0,(.5,.5)),(16,(4.5,.5)),(3,(.5,3.5)),(19,(4.5,3.5))):
    values=[0]*20; values[channel]=100
    window.data_buffer.append_frame(dict(frame,signal=values))
    window.refresh_plots()
    assert window.centroid_view.raw_centroid == position
values=[0]*20; values[13]=100; values[18]=300
window.data_buffer.append_frame(dict(frame,type='signal',signal=values))
window.refresh_plots()
assert window.centroid_view.raw_centroid == (4.25,2.25)
assert window.centroid_view.centroid != window.centroid_view.raw_centroid
from unittest.mock import patch
# 新采样记录一次原始/滤波坐标；GUI重复刷新不重复滤波或重复记录。
window._connected=True
with patch.object(window.rolling_log,'submit') as log:
    window.data_buffer.append_frame(dict(frame,type='signal',signal=values))
    window.refresh_plots()
    displayed=window.centroid_view.centroid
    window.refresh_plots()
    assert window.centroid_view.centroid == displayed
    assert log.call_count == 1
    assert log.call_args.args == ('centroid',)
    recorded=log.call_args.kwargs['frame']
    assert recorded['raw_xy'] == (4.25,2.25)
    assert recorded['selected_channels'] == (13,14,17,18)
    assert recorded['selected_sum_signal'] == 400
    assert recorded['filter']['centroid_algorithm'] == 'max_sum_2x2'
    assert recorded['filtered_xy'] == displayed
    assert recorded['signal'] == tuple(values)
    assert recorded['signal_threshold'] == 100
    assert recorded['max_signal'] == 300
    assert recorded['source_sample_sequence'] == window.data_buffer.latest_sensor_sample()['sequence']
window._connected=False
window.centroid_smoothing_checkbox.setChecked(False)
window.refresh_plots()
assert window.centroid_view.centroid == (4.25,2.25)
window.centroid_smoothing_checkbox.setChecked(True)
window.refresh_plots()
# 切换同一采样立即重算，日志与保存配置必须反映所选模式。
comparison = [0]*20; comparison[0]=100; comparison[19]=300
window.data_buffer.append_frame(dict(frame,type='signal',signal=comparison))
window.refresh_plots()
window._connected=True
with patch.object(window.rolling_log,'submit') as log:
    window.centroid_local_2x2_checkbox.setChecked(False)
    assert window.centroid_view.centroid == (3.5,2.75)
    assert '全部20通道' in window.centroid_view._message
    assert log.call_count == 1
    assert log.call_args.kwargs['frame']['filter']['centroid_algorithm'] == 'all_20'
    assert log.call_args.kwargs['frame']['selected_channels'] == tuple(range(20))
    window.refresh_plots()
    assert log.call_count == 1
window._connected=False
window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['centroid_local_2x2'] is False
window.centroid_local_2x2_checkbox.setChecked(True)
assert window.centroid_view.centroid == (4.5,3.5)
assert '最强2×2' in window.centroid_view._message
window._load_display_config()
assert not window.centroid_local_2x2_checkbox.isChecked()
assert not window.centroid_view.position_filter.local_2x2_enabled
assert window.centroid_view.centroid == (3.5,2.75)
window.centroid_local_2x2_checkbox.setChecked(True)
# Raw开关立即重算同一帧并保存来源；Run不含Raw时隐藏，Debug恢复后重新显示。
window.centroid_local_2x2_checkbox.setChecked(False)
raw_values=[1000]*19+[3000]
window.data_buffer.append_frame(dict(frame,signal=comparison,raw=raw_values))
window.refresh_plots()
window._connected=True
with patch.object(window.rolling_log,'submit') as log:
    window.centroid_raw_checkbox.setChecked(True)
    actual=window.centroid_view.centroid
    assert abs(actual[0]-59000/22000)<1e-10 and abs(actual[1]-47000/22000)<1e-10
    assert 'Raw Val' in window.centroid_view._message
    assert log.call_count == 1
    recorded=log.call_args.kwargs['frame']
    assert recorded['raw'] == tuple(raw_values)
    assert recorded['filter']['centroid_source'] == 'raw'
    assert recorded['selected_sum_weight'] == 22000
    assert recorded['selected_sum_signal'] == 400
window._connected=False
window.save_display_config()
assert json.loads(window._config_path.read_text(encoding='utf-8'))['centroid_raw'] is True
window.centroid_raw_checkbox.setChecked(False)
assert window.centroid_view.centroid == (3.5,2.75)
window._load_display_config()
assert window.centroid_raw_checkbox.isChecked()
assert window.centroid_view.position_filter.raw_enabled
window.data_buffer.append_frame(dict(frame,type='signal',signal=comparison))
window.refresh_plots()
assert window.centroid_view.centroid is None
assert 'Debug' in window.centroid_view._message
window.data_buffer.append_frame(dict(frame,signal=comparison,raw=raw_values))
window.refresh_plots()
assert window.centroid_view.centroid is not None
# 均值差开关与Raw互斥，同帧立即到达角点；日志包含实际权重及计算方式。
window._connected=True
with patch.object(window.rolling_log,'submit') as log:
    window.centroid_raw_difference_checkbox.setChecked(True)
    assert not window.centroid_raw_checkbox.isChecked()
    assert window.centroid_view.centroid == (4.5,3.5)
    assert '均值差归一化' in window.centroid_view._message
    assert log.call_count == 1
    record=log.call_args.kwargs['frame']
    assert record['raw_mean'] == 1100
    assert record['normalized_weights'] == (0,)*19+(1,)
    assert record['filter']['centroid_source'] == 'raw'
    assert record['filter']['centroid_weighting'] == 'positive_mean_difference_normalized'
window._connected=False
window.save_display_config()
saved=json.loads(window._config_path.read_text(encoding='utf-8'))
assert saved['centroid_raw_difference'] is True and saved['centroid_raw'] is False
window.centroid_raw_checkbox.setChecked(True)
assert not window.centroid_raw_difference_checkbox.isChecked()
assert not window.centroid_view.position_filter.raw_difference_enabled
window._load_display_config()
assert window.centroid_raw_difference_checkbox.isChecked()
assert not window.centroid_raw_checkbox.isChecked()
assert window.centroid_view.position_filter.raw_difference_enabled
assert window.centroid_view.centroid == (4.5,3.5)
window.data_buffer.append_frame(dict(frame,type='signal',signal=comparison))
window.refresh_plots()
assert window.centroid_view.centroid is None and 'Debug' in window.centroid_view._message
window.data_buffer.append_frame(dict(frame,signal=comparison,raw=raw_values))
window.refresh_plots()
assert window.centroid_view.centroid == (4.5,3.5)
window.centroid_raw_difference_checkbox.setChecked(False)
assert window.centroid_view.centroid == (3.5,2.75)
window.centroid_raw_checkbox.setChecked(False)
window.centroid_local_2x2_checkbox.setChecked(True)
# 消失冻结独立于平滑开关；原始Signal、真实状态和原始质心依然保留。
window.centroid_smoothing_checkbox.setChecked(False)
window.data_buffer.append_frame(frame)
window.refresh_plots()
held_xy = window.centroid_view.centroid
tail=[0]*20; tail[0]=20
window.data_buffer.append_frame(dict(frame,signal=tail,state=[3]*20))
window.refresh_plots()
assert window.centroid_view.centroid == held_xy
assert window.centroid_view.position_filter.holding_position
assert window.centroid_view.raw_centroid == (.5,.5)
assert window.centroid_view.position_filter.threshold_hidden
assert window.value_labels['signal_value'][0].text() == '20'
assert window.value_labels['state'][0].text() == 'FALLING'
window.centroid_scale_slider.setValue(45)
window.refresh_plots()
assert window.centroid_view.centroid == held_xy
# 相同串口采样不再更新时，GUI时钟仍须让冻结点到时消失，并只记一次超时事件。
old_sample=dict(window.data_buffer.latest_sensor_sample(),age_seconds=.15)
window._connected=True
with patch.object(window.data_buffer,'latest_sensor_sample',return_value=old_sample), patch.object(window.rolling_log,'submit') as log, \
        patch('sensor_window.time.perf_counter',return_value=old_sample['sample_timestamp']+.15):
    window.refresh_plots()
    assert window.centroid_view.centroid is None
    assert not window.centroid_view.position_filter.holding_position
    assert log.call_count == 1
    assert log.call_args.kwargs['frame']['update_reason'] == 'hold_timeout'
    window.refresh_plots()
    assert log.call_count == 1
window._connected=False
window.centroid_scale_slider.setValue(70)
window.centroid_smoothing_checkbox.setChecked(True)
window.data_buffer.append_frame(frame)
window.refresh_plots()
stale=dict(window.data_buffer.latest_sensor_sample(),age_seconds=1)
with patch.object(window.data_buffer,'latest_sensor_sample',return_value=stale), \
        patch('sensor_window.time.perf_counter',return_value=stale['sample_timestamp']+1):
    window.refresh_plots()
assert window.centroid_view.centroid is None
for values in ([0]*20, [100]*19, [float('nan')]+[100]*19):
    window.data_buffer.append_frame(dict(frame,type='signal',channel_count=len(values),signal=values))
    window.refresh_plots()
    assert window.centroid_view.centroid is None
window.data_buffer.append_frame(frame)
window.refresh_plots()
window.table_row_buttons['channel_19'].setChecked(True)
window.plot_tabs.setCurrentIndex(0)
app.processEvents()
window.plot_tabs.setCurrentWidget(window.centroid_view)
# 三参数命令及20路配置/硬件回读。
class Reader:
    def __init__(self): self.sent=[]
    def set_matrix_config(self,*args): self.sent.append(args); return True
    def get_matrix_config(self): return True
    def get_capsense_channel_config(self): return True
    def send_output_mode(self,*args): return True
reader=Reader(); window.reader=reader; window._set_connected(True)
window.nsub_input.setValue(400);window.tx_clk_div_input.setValue(120);window.ref_cdac_input.setValue(15)
generation = window.host_state_machine.generation
window.apply_parameters()
assert reader.sent == [(400,120,15)]
assert window.host_state_machine.generation > generation
generation = window.host_state_machine.generation
window.on_matrix_config_status(dict(pending=False,result=0,actual_nsub=400,actual_tx_clk_div=120,actual_ref_cdac=15))
assert window.host_state_machine.generation > generation
channels=[dict(channel=i,configured_nsub=400,hardware_nsub=400,configured_tx_clk_div=120,hardware_tx_clk_div=120,
               configured_ref_cdac=15,hardware_ref_cdac=15,max_raw_count=60000) for i in range(20)]
window.on_capsense_channel_config(dict(channels=channels,read_ok=True))
assert window.capsense_parameter_table.rowCount() == 20
assert window.capsense_parameter_table.item(19,1).text() == '400 / 400'
window.tx_clk_div_input.setValue(121);window.apply_parameters()
assert len(reader.sent) == 1
window.reader=None;window._set_connected(False)
for minutes in (1,3,5):
    window.export_recent(minutes)
    worker=window._recent_export_worker
    loop=QtCore.QEventLoop();worker.finished.connect(loop.quit)
    QtCore.QTimer.singleShot(5000,loop.quit);loop.exec()
    app.processEvents()
    assert (window.data_recorder.output_dir/f'recent_{minutes}min.csv').exists()
if os.environ.get('MATRIX20_SCREENSHOT'):
    window.data_buffer.append_frame(frame)
    window.refresh_plots()
    window.resize(1920,1080)
    app.processEvents()
    window.grab().save(os.environ['MATRIX20_SCREENSHOT'])
    window.control_tabs.setCurrentIndex(next(i for i in range(window.control_tabs.count()) if window.control_tabs.tabText(i)=='波形设置'))
    app.processEvents()
    window.grab().save(os.environ['MATRIX20_SCREENSHOT'].replace('.png','_settings.png'))
window.save_display_config()
# 写盘被阻塞时关闭窗口仍须立刻返回，由后台完成后自动关闭。
import threading, time
entered, release = threading.Event(), threading.Event()
original_write = window.rolling_log._write
def slow_write(batch):
    entered.set(); release.wait(3); original_write(batch)
window.rolling_log._write = slow_write
window.rolling_log.submit('frame', port='COM_TEST', frame=frame)
start = time.monotonic()
window.close()
assert time.monotonic() - start < .5
assert entered.wait(2)
assert window.isVisible()
release.set()
deadline = time.monotonic() + 3
while window.isVisible() and time.monotonic() < deadline:
    app.processEvents(); time.sleep(.01)
assert not window.isVisible()
assert not window.rolling_log.is_alive()
# 连接切换及三参数请求/同步各记录实验配置，关闭时必须一起落盘。
assert window.rolling_log.status()['written'] == 5, window.rolling_log.status()
assert not errors, errors
assert not any(name.split('.')[0].startswith(('can','motor_','zdt_')) for name in sys.modules)
print('Sensor-only monitor: no CAN/motor imports or startup port opens; 20-channel UI, parameters and snapshots passed')
'''


class SensorOnlyTests(unittest.TestCase):
    def test_window_without_motor_or_can(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path,directory)
            script=Path(directory)/'_sensor_smoke.py'
            script.write_text(SMOKE,encoding='utf-8')
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result=subprocess.run([sys.executable,str(script)],cwd=directory,env=env,capture_output=True,text=True,encoding='utf-8',timeout=40)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)


if __name__=='__main__':
    unittest.main(verbosity=2)

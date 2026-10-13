"""隔离验证右侧标定、逐帧采集、保存事务与参考重建，不访问串口或电机。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=r'''
import json, os, time
from pathlib import Path
from unittest.mock import patch
from qt_compat import QtWidgets, QtGui
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports=lambda self:([], '')
from sensor_window import MainWindow
app=QtWidgets.QApplication([])
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():QtGui.QFontDatabase.addApplicationFont(str(font))
w=MainWindow();w.show();w.refresh_timer.stop();w._startup_connect_timer.stop()
p=w.raw_gain_calibration;w.control_tabs.setCurrentWidget(p);app.processEvents()
assert w.control_tabs.tabText(w.control_tabs.indexOf(p))=='标定'
assert p.inputs['mass_g'].value()==100 and p.inputs['repeats'].value()==5
assert p.inputs['target'].value()==0
assert all(not v.prefix() and not v.suffix() for v in p.inputs.values())
assert not p.apply_button.isEnabled()
w.current_port='TEST_ONLY';w._connected=True
p.inputs['repeats'].setValue(3);p.inputs['settle_seconds'].setValue(.2);p.inputs['window_seconds'].setValue(.3)
clock=[time.perf_counter()+1]
def sample(raw):
    clock[0]+=.01
    frame=dict(type='debug',device=1,channel_count=20,raw_only=True,raw=raw,baseline=[0]*20,signal=[0]*20,state=[-1]*20,
               fast_k_percent=[float('nan')]*20,slow_k_percent=[float('nan')]*20)
    with patch('data_buffer.time.perf_counter',return_value=clock[0]):w.data_buffer.append_frame(frame)
def refresh():
    with patch('sensor_window.time.perf_counter',return_value=clock[0]):w.refresh_plots()
def click(button):
    with patch('time.perf_counter',return_value=clock[0]):button.click()
def burst(raw):
    for n in range(60):
        sample(raw)
        if n%10==9:refresh()
sample([10000]*20);refresh()
for ch in range(20):
    p.select_channel(ch)
    for repeat in range(3):
        click(p.empty_button);burst([10000+ch*30]*20)
        assert p.engine.phase=='await_load',p.status.text()
        click(p.loaded_button);raw=[10000+ch*30]*20;raw[ch]+=1000+ch*100;burst(raw)
    assert len(p.engine.trials[ch])==3
assert p.engine.results()['ready'] and p.apply_button.isEnabled()
assert p.report_path.exists()
saved=json.loads(p.report_path.read_text(encoding='utf-8'))
assert saved['settings']['mass_g']==100 and saved['results']['ready']
assert saved['trials'][19][0]['response']==2900
assert w.array_bar_settings.settings()['raw_gains']==[1.0]*20
# 原采样始终保留设备Raw，标定不能改写固件占位状态。
assert w.data_buffer.latest_sensor_sample()['raw'][19]==13470
assert w.data_buffer.latest_sensor_sample()['state']==(-1,)*20
gains=p.engine.results()['gains']
gen=w.host_state_machine.generation
before=w.array_bar_settings.settings();config_path=w._config_path
w._config_path=Path('missing_parent')/'config.json'
ok,msg=w._apply_calibrated_gains(gains)
assert not ok and w.array_bar_settings.settings()==before and w.host_state_machine.generation==gen
w._config_path=config_path
# 应用按钮只在明确移开砝码并完成新的稳定空载后生效。
click(p.apply_button);assert w.array_bar_settings.settings()==before
burst([10000]*20)
assert w.array_bar_settings.settings()['raw_gains']==gains
assert list(w.host_state_machine.gains)==gains and list(w.touchpad.tracker.raw_gains)==gains
assert w.host_state_machine.generation>gen and w.touchpad.tracker.position_reference is None
assert json.loads(config_path.read_text(encoding='utf-8'))['array_bars']['raw_gains']==gains
assert json.loads(config_path.read_text(encoding='utf-8'))['raw_gain_calibration']['mass_g']==100
# 相同增益再次应用也必须重建参考。
gen=w.host_state_machine.generation
ok,msg=w._apply_calibrated_gains(gains)
assert ok and w.host_state_machine.generation>gen
# 独立验证不进入标定集，原始Raw不受现有增益影响。
p.mode.setCurrentIndex(1);p.select_channel(0)
click(p.empty_button);burst([18000]*20);click(p.loaded_button)
raw=[18000]*20;raw[0]+=1000;burst(raw)
assert len(p.engine.trials[0])==3 and p.engine.results()['channels'][0]['validation_count']==1
assert abs(p.engine.results()['channels'][0]['validation_error_percent'])<.01
assert p.inputs['target'].isEnabled() and not p.inputs['mass_g'].isEnabled()
# 保存失败不能清空旧项目；正常新建保留历史文件。
old_path=p.report_path
with patch.object(p,'save_report',return_value=False):p.new_session()
assert len(p.engine.trials[0])==3
p.new_session();assert old_path.exists() and not any(p.engine.trials)
# 最后一条撤销后仍保存归档，不能让磁盘误保留有效标定点。
p.select_channel(0);click(p.empty_button);burst([18000]*20)
click(p.loaded_button);raw=[18000]*20;raw[0]+=1000;burst(raw)
one_path=p.report_path;p.undo_button.click()
discarded=json.loads(one_path.read_text(encoding='utf-8'))
assert not any(discarded['trials']) and discarded['discarded'][0]['trial']['response']==1000
p.new_session();assert one_path.exists()
# GUI积压的旧帧不能完成新的空载确认或触发增益应用。
click(p.empty_button);sample([18000]*20)
p.process_samples(w.data_buffer.pointer_samples_after(p.engine.last_sequence),clock[0]+1)
assert p.engine.phase=='empty' and p.engine.baseline is None
# 参数/连接变化使未完成采集作废，已完成结果保留但不能应用。
p.select_channel(0);click(p.empty_button);burst([18000]*20)
click(p.loaded_button);raw=[18000]*20;raw[0]+=1000;burst(raw)
assert len(p.engine.trials[0])==1
w._set_connected(False)
assert p.engine.invalid_reason and len(p.engine.trials[0])==1 and not p.apply_button.isEnabled()
if os.environ.get('RAW_CALIBRATION_SCREENSHOT'):
    p.new_session();p.load_settings({});w.control_tabs.setCurrentWidget(p);app.processEvents()
    w.grab().save(os.environ['RAW_CALIBRATION_SCREENSHOT'])
    p.verticalScrollBar().setValue(p.verticalScrollBar().maximum());app.processEvents()
    w.grab().save(os.environ['RAW_CALIBRATION_SCREENSHOT'].replace('.png','_bottom.png'))
w.close();app.processEvents()
w=MainWindow();w.refresh_timer.stop();w._startup_connect_timer.stop()
assert w.array_bar_settings.settings()['raw_gains']==gains
assert list(w.host_state_machine.gains)==gains
w.close();app.processEvents()
print('Calibration UI: 20 channels, independent validation, raw preservation, failure transaction, reference reset and persistence passed')
'''

HISTORY_SCRIPT=r'''
import json, os, time
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch
from qt_compat import QtWidgets, QtGui, QtCore
from serial_ports import SerialPortScanner
SerialPortScanner.list_ports=lambda self:([], '')
from sensor_window import MainWindow
from raw_gain_calibration import CalibrationSettings, RawGainCalibration
app=QtWidgets.QApplication([])
app.setQuitOnLastWindowClosed(False)
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():QtGui.QFontDatabase.addApplicationFont(str(font))
settings=CalibrationSettings(mass_g=50,repeats=3,settle_seconds=.2,window_seconds=.3)
model=RawGainCalibration(settings)
for ch in range(20):
    for repeat in range(3):
        baseline=[10000.0]*20;loaded=baseline.copy();response=1000+100*ch;loaded[ch]+=response
        trial=dict(channel=ch,response=response,capture_settings=asdict(settings),baseline=baseline,loaded_raw=loaded,
            unloaded=dict(raw=baseline.copy(),noise=[1.0]*20,ranges=[4]*20,timestamp=10+ch,sample_count=64),
            loaded_timestamp=12+ch,loaded_elapsed_seconds=.6,loaded_sample_count=64,loaded_ranges=[4]*20,dominant_channel=ch)
        model.trials[ch].append(trial);model.revision+=1
gains=model.results()['gains'];target=model.results()['target']
validation=dict(model.trials[0][0],gain=gains[0],target=target,normalized_response=1000*gains[0],calibration_revision=model.revision)
model.validations[0].append(validation)
report=model.report();report.update(context=None,invalid_reason='传感器连接已改变',saved_at='2026-10-11T10:00:00+08:00',applied=None)
history=Path('raw_gain_calibrations');history.mkdir()
source=history/'raw_gain_legacy.json';source.write_text(json.dumps(report),encoding='utf-8');original=source.read_bytes()
def window():
    w=MainWindow();w.refresh_timer.stop();w._startup_connect_timer.stop();return w
def close_window(w):
    w.close()
    w.rolling_log.wait(timeout=2)
    w.close()
    loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(150,loop.quit);loop.exec()
def identity(w,connected=True):
    w.current_port='TEST_ONLY';w.current_device_info={'device':1,'version':21}
    w._touchpad_config_signature=tuple((ch,32,8,20) for ch in range(20));w._connected=connected
clock=[time.perf_counter()]
def sample(w,raw):
    clock[0]+=.01
    frame=dict(type='debug',device=1,channel_count=20,raw_only=True,raw=raw,baseline=[0]*20,signal=[0]*20,state=[-1]*20,
               fast_k_percent=[float('nan')]*20,slow_k_percent=[float('nan')]*20)
    with patch('time.perf_counter',return_value=clock[0]):w.data_buffer.append_frame(frame)
def call(function,*args):
    with patch('time.perf_counter',return_value=clock[0]):return function(*args)
def burst(w,raw):
    for n in range(60):
        sample(w,raw)
        if n%10==9:call(w.refresh_plots)
w=window();p=w.raw_gain_calibration
# 无新配置项目指针的升级启动会读取最近记录，保留50g及全部验证数据。
assert p.loaded_path==source.resolve()
assert p.inputs['mass_g'].value()==50 and p.table.item(13,1).text()=='3'
assert p.engine.results()['channels'][0]['validation_count']==1
assert p.engine.results()['gains']==gains and not p.apply_button.isEnabled()
assert p.engine.baseline is None and p.engine.last_sequence==0
assert w.array_bar_settings.settings()['raw_gains']==[1.0]*20
identity(w);sample(w,[10000]*20);call(w.refresh_plots)
assert p.confirm_environment_button.isEnabled()
if os.environ.get('RAW_CALIBRATION_HISTORY_SCREENSHOT'):
    w.show();w.control_tabs.setCurrentWidget(p);app.processEvents()
    path=os.environ['RAW_CALIBRATION_HISTORY_SCREENSHOT']
    w.grab().save(path)
    p.verticalScrollBar().setValue(p.verticalScrollBar().maximum());app.processEvents()
    w.grab().save(path.replace('.png','_bottom.png'))
# 不完整/损坏文件不能覆盖已恢复数据或运行增益。
bad=Path('bad.json');bad.write_text('{',encoding='utf-8');previous=p.engine
assert not call(p.load_report,bad) and p.engine is previous
assert source.read_bytes()==original
call(p.confirm_environment)
assert p.engine.results()['ready'] and p.report_path!=source.resolve()
assert source.read_bytes()==original
saved=json.loads(p.report_path.read_text(encoding='utf-8'))
assert saved['context']['hardware_signature']==[[ch,32,8,20] for ch in range(20)]
assert saved['restored_from']['path']==str(source.resolve())
current=dict(p.get_context());current['port']='ALTERNATE_PORT'
assert p._environment_check(current)[0]=='confirm'
current=dict(p.get_context());current['hardware_signature']=None
assert p._environment_check(current)[0]=='waiting'
# 读取不会应用增益，只有新的明确空载窗口完成后才更新运行配置。
call(p.apply_button.click);assert w.array_bar_settings.settings()['raw_gains']==[1.0]*20
burst(w,[10000]*20);assert w.array_bar_settings.settings()['raw_gains']==gains
# 只重测CH7，其他19通道原始记录保持完整，使用新的本次空载。
p.mode.setCurrentIndex(0);p.select_channel(7);call(p.undo_button.click)
assert len(p.engine.trials[7])==2 and all(len(v)==3 for ch,v in enumerate(p.engine.trials) if ch!=7)
call(p.empty_button.click);burst(w,[19000]*20);call(p.loaded_button.click)
raw=[19000]*20;raw[7]+=1700;burst(w,raw)
assert p.engine.trials[7][-1]['baseline'][7]==19000 and p.engine.results()['ready']
assert source.read_bytes()==original
call(w.save_display_config);working=p.report_path
w._set_connected(False);call(w.save_display_config);close_window(w)
# 重启恢复指定工作副本，不依赖旧窗口内存；相同环境自动核对，序号从新会话开始。
w=window();p=w.raw_gain_calibration
assert p.loaded_path==working.resolve() and len(p.engine.trials[7])==3
assert p.inputs['mass_g'].value()==50 and len(p.engine.discarded)==1
assert w.array_bar_settings.settings()['raw_gains']==gains
identity(w,False);w._set_connected(True);sample(w,[19000]*20);call(w.refresh_plots)
assert p.engine.results()['ready'] and not p._needs_environment_check
assert p.engine.last_sequence==1 and p.engine.baseline is None
# 已知三参数不同，人工确认不能绕过；记录仍可查看。
w._touchpad_config_signature=tuple((ch,64 if ch==0 else 32,8,20) for ch in range(20))
call(w.refresh_plots);assert not p.engine.results()['ready']
call(p.confirm_environment);assert p.engine.invalid_reason and not p.apply_button.isEnabled()
assert len(p.engine.trials[7])==3 and p.table.item(7,1).text()=='3'
assert source.read_bytes()==original
# 当前记录保存失败时不允许加载别的项目覆盖内存。
previous=p.engine
with patch.object(p,'save_report',return_value=False):assert not call(p.load_report,source)
assert p.engine is previous
# 不恢复未完成空载/加载采集，断线后记录环境不会被清成null。
assert json.loads(p.report_path.read_text(encoding='utf-8'))['context']['hardware_signature']==[[ch,32,8,20] for ch in range(20)]
w._set_connected(False);p.new_session();call(w.save_display_config)
assert not any(p.engine.trials) and json.loads(w._config_path.read_text(encoding='utf-8'))['raw_gain_calibration_record']==''
close_window(w)
w=window();p=w.raw_gain_calibration
assert not any(p.engine.trials) and p.loaded_path is None and w.array_bar_settings.settings()['raw_gains']==gains, (p.resume_path(),w.array_bar_settings.settings()['raw_gains'],gains)
close_window(w)
print('History UI: startup restore, legacy confirmation, environment matching, immutable source, fresh apply and single-channel retest passed')
'''


class CalibrationUiTests(unittest.TestCase):
    def test_calibration_ui(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):shutil.copy2(path,directory)
            script=Path(directory)/'calibration_ui_smoke.py';script.write_text(SCRIPT,encoding='utf-8')
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result=subprocess.run([sys.executable,str(script)],cwd=directory,env=env,capture_output=True,text=True,encoding='utf-8',timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)

    def test_history_restore_ui(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):shutil.copy2(path,directory)
            script=Path(directory)/'history_ui_smoke.py';script.write_text(HISTORY_SCRIPT,encoding='utf-8')
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result=subprocess.run([sys.executable,str(script)],cwd=directory,env=env,capture_output=True,text=True,encoding='utf-8',timeout=90)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)


if __name__=='__main__':unittest.main()

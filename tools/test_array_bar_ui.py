"""隔离验证20路柱体的实际采样、布局、量程、旋转与保存恢复。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import json,os,sys,time,traceback
from pathlib import Path
from qt_compat import QtCore,QtGui,QtWidgets
from serial_ports import SerialPortScanner
from sensor_window import MainWindow
from array_bar_view import normalize_settings
from data_buffer import RX_FPS_STALE_SECONDS
SerialPortScanner.list_ports=lambda self:([], '')
app=QtWidgets.QApplication([])
for name in ('msyh.ttc','segoeui.ttf'):
    font=Path('C:/Windows/Fonts')/name
    if font.exists():QtGui.QFontDatabase.addApplicationFont(str(font))
errors=[]
def hook(*args):
    errors.append(str(args));traceback.print_exception(*args)
sys.excepthook=hook
window=MainWindow();window.show();window.refresh_timer.stop()
view=window.array_bars;panel=window.array_bar_settings
window.plot_tabs.setCurrentWidget(view);app.processEvents()
assert window.control_tabs.currentWidget() is panel
assert view.values is None
assert [[b['channel'] for b in view.bars() if b['row']==row] for row in range(4)]==[
    [0,4,8,12,16],[1,5,9,13,17],[2,6,10,14,18],[3,7,11,15,19]]
for widget in panel.findChildren(QtWidgets.QSpinBox):assert not widget.prefix() and not widget.suffix()

signal=[0]*20
for ch,value in {9:14000,5:6800,13:4800,8:4300,10:5200,4:1500,6:1700,12:2100,14:2400}.items():signal[ch]=value
raw=[6000+ch*100 for ch in range(20)]
raw=[a+b for a,b in zip(raw,signal)]
frame=dict(type='debug',device=1,channel_count=20,raw=raw,baseline=[6000+ch*100 for ch in range(20)],
           signal=signal,state=[0]*20,fast_k_percent=[0]*20,slow_k_percent=[0]*20)
window.data_buffer.append_frame(frame);window.refresh_plots();app.processEvents()
assert view.values==tuple(signal)
bars=view.bars();peak=next(b for b in bars if b['channel']==9)
assert (peak['row'],peak['col'])==(1,2)
assert all(b['height']==0 for b in bars if signal[b['channel']]==0)
window.table_row_buttons['channel_9'].setChecked(False)
assert next(b for b in view.bars() if b['channel']==9)['value']==14000
window.table_row_buttons['channel_9'].setChecked(True)
if os.environ.get('ARRAY_BAR_SCREENSHOT'):
    window.grab().save(os.environ['ARRAY_BAR_SCREENSHOT'])

# 源切换立即使用最新帧，分别保存量程，超量程数字保留原值。
panel.maximum.setValue(12000)
assert peak['value']==14000
assert next(b for b in view.bars() if b['channel']==9)['height']==view.MAX_HEIGHT
assert view.values[9]==14000
panel.source.setCurrentIndex(panel.source.findData('raw'))
assert view.values==tuple(raw) and panel.maximum.value()==65535
panel.maximum.setValue(30000)
panel.source.setCurrentIndex(panel.source.findData('signal'))
assert panel.maximum.value()==12000
panel.auto_range.setChecked(True)
assert not panel.maximum.isEnabled() and view.height_limit()>=max(signal)

# Run只更新Signal；选择Raw不能复用上个Debug帧的旧Raw。
window.data_buffer.append_frame(dict(type='signal',device=1,channel_count=20,signal=list(range(20))))
window.refresh_plots()
assert view.values==tuple(range(20))
panel.source.setCurrentIndex(panel.source.findData('raw'))
assert view.values is None and 'Debug' in view._message
window.data_buffer.append_frame(frame);window.refresh_plots()
assert view.values==tuple(raw)
sample=window.data_buffer.latest_sensor_sample()
view.set_sample(dict(sample,age_seconds=RX_FPS_STALE_SECONDS+.01))
assert view.values is None and '中断' in view._message
view.set_sample(dict(sample,channel_count=8))
assert view.values is None
view.set_sample(dict(sample,raw=[float('nan')]*20))
assert view.values is None
window.refresh_plots()

# 鼠标旋转同步到右侧控制项，旋转与数据源保存恢复。
point=QtCore.QPointF(view.width()/2,view.height()/2)
def mouse(kind,pos,button,buttons):
    return QtGui.QMouseEvent(kind,pos,pos,button,buttons,QtCore.Qt.KeyboardModifier.NoModifier)
view.mousePressEvent(mouse(QtCore.QEvent.Type.MouseButtonPress,point,QtCore.Qt.MouseButton.LeftButton,QtCore.Qt.MouseButton.LeftButton))
view.mouseMoveEvent(mouse(QtCore.QEvent.Type.MouseMove,point+QtCore.QPointF(75,-50),QtCore.Qt.MouseButton.NoButton,QtCore.Qt.MouseButton.LeftButton))
view.mouseReleaseEvent(mouse(QtCore.QEvent.Type.MouseButtonRelease,point,QtCore.Qt.MouseButton.LeftButton,QtCore.Qt.MouseButton.NoButton))
assert (view.settings['azimuth'],view.settings['elevation'])==(65,50)
assert (panel.azimuth.value(),panel.elevation.value())==(65,50)
panel.auto_range.setChecked(False)
panel.show_values.setChecked(False)
loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(650,loop.quit);loop.exec()
saved=json.loads(window._config_path.read_text(encoding='utf-8'))['array_bars']
assert saved==panel.settings()
assert saved['source']=='raw' and saved['raw_max']==30000 and saved['signal_max']==12000
assert normalize_settings(dict(signal_max=0,raw_max=float('nan'),azimuth=900,elevation=-5))['raw_max']==65535
window.close();app.processEvents()
window=MainWindow();window.show();window.refresh_timer.stop();app.processEvents()
assert window.array_bar_settings.settings()==saved
assert window.array_bars.settings==saved
assert window.array_bars.values is None
window.plot_tabs.setCurrentWidget(window.array_bars)
window.data_buffer.append_frame(frame);window.refresh_plots();app.processEvents()
if os.environ.get('ARRAY_BAR_SCREENSHOT'):
    window.grab().save(os.environ['ARRAY_BAR_SCREENSHOT'].replace('.png','_raw.png'))
window.close();app.processEvents()
assert not errors,errors
print('Array bars: physical channel mapping, Signal/Raw switching, clipping, stale/Run handling, camera and persistence passed')
'''


class ArrayBarUiTests(unittest.TestCase):
    def test_array_bar_ui(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):
                shutil.copy2(path,directory)
            script=Path(directory)/'array_bar_smoke.py';script.write_text(SCRIPT,encoding='utf-8')
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result=subprocess.run([sys.executable,str(script)],cwd=directory,env=env,capture_output=True,text=True,encoding='utf-8',timeout=40)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)


if __name__=='__main__':unittest.main()

"""隔离验证表盘绘制、鼠标/传感器拖动、异常取消及参数自动保存恢复。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=r'''
import sys,time,json,os,math,traceback
from pathlib import Path
from qt_compat import QtCore,QtGui,QtWidgets
from serial_ports import SerialPortScanner
from sensor_window import MainWindow
from watch_face import WatchFaceModel,DEFAULTS
SerialPortScanner.list_ports=lambda self:([], '')
app=QtWidgets.QApplication([])
for name in ('msyh.ttc','segoeui.ttf'):
    font=Path('C:/Windows/Fonts')/name
    if font.exists():QtGui.QFontDatabase.addApplicationFont(str(font))
errors=[]
def exception_hook(*args):
    errors.append(str(args));traceback.print_exception(*args)
sys.excepthook=exception_hook
window=MainWindow();window.show();app.processEvents()
window.refresh_timer.stop()
canvas=window.watch_face;canvas.timer.stop()
assert window.plot_tabs.currentWidget() is canvas
assert window.control_tabs.currentWidget() is window.watch_settings
assert len(canvas.model.icons())==100
assert set(window.watch_settings.controls)==set(DEFAULTS)
for widget in window.watch_settings.findChildren(QtWidgets.QAbstractSpinBox):
    assert not widget.prefix() and not widget.suffix()

# 不同运动阶段只能有一个放大项，选中切换无需等动画帧就缩小旧项。
def check_sizes(expected=None):
    icons=canvas.model.icons()
    selected=[icon for icon in icons if icon['selected']]
    assert len(selected)==1
    if expected is not None:assert selected[0]['id']==expected
    for icon in icons:
        if not icon['selected']:
            assert icon['size']==canvas.model.settings['small_size']
            assert canvas.icon_diameter(icon)==canvas.model.settings['small_size']
    assert sum(canvas.icon_diameter(icon)>canvas.model.settings['small_size'] for icon in icons)<=1
check_sizes(0)
canvas.animate()
canvas.model.begin(time.perf_counter())
canvas.model.drag(-canvas.model.settings['spacing'],0,time.perf_counter())
check_sizes(1)
canvas.animate()
check_sizes(1)
canvas.model.snap(time.perf_counter(),17)
check_sizes(17)
canvas.animate(time.perf_counter()+.1)
check_sizes(17)
canvas.model=WatchFaceModel()
canvas._sizes.clear()
canvas._focus.clear()

# 鼠标按住拖动，离开原图标也持续跟随，释放后先惯性后吸附。
center,scale=canvas.geometry_info()
def mouse(kind,point,button,buttons):
    return QtGui.QMouseEvent(kind,point,point,button,buttons,QtCore.Qt.KeyboardModifier.NoModifier)
press=mouse(QtCore.QEvent.Type.MouseButtonPress,center,QtCore.Qt.MouseButton.LeftButton,QtCore.Qt.MouseButton.LeftButton)
canvas.mousePressEvent(press)
assert canvas.owner=='mouse' and canvas.model.phase=='drag'
for i in range(1,7):
    point=center+QtCore.QPointF(i*8*scale,i*3*scale)
    canvas.mouseMoveEvent(mouse(QtCore.QEvent.Type.MouseMove,point,QtCore.Qt.MouseButton.NoButton,QtCore.Qt.MouseButton.LeftButton))
    time.sleep(.01)
assert canvas.model.drag_distance>40
canvas.mouseReleaseEvent(mouse(QtCore.QEvent.Type.MouseButtonRelease,point,QtCore.Qt.MouseButton.LeftButton,QtCore.Qt.MouseButton.NoButton))
assert canvas.model.phase=='inertia'
check_sizes()
end=time.perf_counter()+3
canvas.animate(end)
assert canvas.model.phase=='idle' and canvas.model.nearest()['distance']<1e-6

# 按真实触摸板算法发出事件，保证触摸板灵敏度不会叠加到表盘速度上。
canvas.model=WatchFaceModel()
panel=window.touchpad
panel.enabled_box.setChecked(True)
base=[8000+i*123 for i in range(20)]
panel.tracker.reference=tuple(base);panel.tracker.noise=(2.,)*20
clock=time.perf_counter()+.1
sequence=0
def sensor(x, state=2,valid=True):
    global clock,sequence
    clock+=.01;sequence+=1
    values=[0]*20 if x is None else [1000*max(0,1-abs(i//4+.5-x))*max(0,1-abs(i%4+.5-1.5)) for i in range(20)]
    sample=dict(sequence=sequence,sample_timestamp=clock,channel_count=20,has_raw=valid,
                raw=[a+b for a,b in zip(base,values)],signal=values,state=[state]*20)
    panel.process_samples([sample],now=clock)
for _ in range(60):sensor(2)
assert canvas.owner=='sensor'
assert math.hypot(*canvas.model.offset)<1e-6
for i in range(80):sensor(2+.4*(i+1)/80)
assert canvas.model.offset[0]>90
sensor(None,state=4)
assert canvas.owner is None and canvas.model.phase=='inertia'
canvas.animate(clock+.03)
offset=tuple(canvas.model.offset)
window.touchpad.motion_sample.emit(dict(timestamp=clock+.03,contact=True,dx=0.,dy=0.,cancel=False))
assert canvas.model.phase=='drag' and tuple(canvas.model.offset)==offset
sensor(None,valid=False)
assert canvas.model.phase=='idle' and canvas.owner is None

# 系统鼠标开关与表盘输入不叠加；切换页面中断未完成手势。
window.plot_tabs.setCurrentWidget(panel.canvas)
panel.system_mouse.setChecked(True)
window.plot_tabs.setCurrentWidget(canvas)
assert not panel.system_mouse.isChecked()
panel.system_mouse.setChecked(True)
assert not panel.system_mouse.isChecked()
canvas.sensor_sample(dict(timestamp=clock+.1,contact=True,dx=.1,dy=0,cancel=False))
window.plot_tabs.setCurrentWidget(window.centroid_view)
assert canvas.owner is None and canvas.model.phase=='idle'
window.plot_tabs.setCurrentWidget(canvas)

# 自动保存全部设置，重新创建窗口验证从磁盘读取，配置文件仍在临时目录。
controls=window.watch_settings.controls
controls['snap_duration_ms'].setValue(730)
controls['inertia_strength'].setValue(1.15)
controls['drag_speed'].setValue(420)
controls['reverse_y'].setChecked(True)
controls['show_numbers'].setChecked(False)
controls['center_glow_strength'].setValue(95)
controls['inactive_opacity'].setValue(35)
controls['focus_transition_ms'].setValue(180)
controls['large_size'].setValue(78)
controls['small_size'].setValue(32)
canvas.animate()
check_sizes()
loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(650,loop.quit);loop.exec()
saved=json.loads(window._config_path.read_text(encoding='utf-8'))['watch_face']
assert saved==window.watch_settings.settings()
assert saved['snap_duration_ms']==730 and saved['reverse_y'] is True
assert (saved['center_glow_strength'],saved['inactive_opacity'],saved['focus_transition_ms'])==(95,35,180)
assert (saved['large_size'],saved['small_size'])==(78,32)
window.close();app.processEvents()
window=MainWindow();window.show();app.processEvents()
assert window.watch_settings.settings()==saved
assert window.watch_face.model.settings==saved
assert window.watch_face.model.focused()['size']==78
assert {icon['size'] for icon in window.watch_face.model.icons() if not icon['selected']}=={32}
window.watch_settings.reset_defaults()
canvas=window.watch_face
canvas.model=WatchFaceModel()
canvas.animate()
window.resize(1700,1000)
app.processEvents()
if os.environ.get('WATCH_FACE_SCREENSHOT'):
    window.grab().save(os.environ['WATCH_FACE_SCREENSHOT'])
    window.watch_settings.verticalScrollBar().setValue(window.watch_settings.verticalScrollBar().maximum())
    app.processEvents()
    window.grab().save(os.environ['WATCH_FACE_SCREENSHOT'].replace('.png','_layout.png'))
window.close();app.processEvents()
assert not errors,errors
print('Watch UI: 100 vector apps, mouse/sensor gestures, no duplicate cursor control, interruption and full autosave/reload passed')
'''


class WatchFaceUiTests(unittest.TestCase):
    def test_watch_ui_and_persistence(self):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'):shutil.copy2(path,directory)
            script=Path(directory)/'watch_smoke.py';script.write_text(SCRIPT,encoding='utf-8')
            env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8')
            result=subprocess.run([sys.executable,str(script)],cwd=directory,env=env,capture_output=True,text=True,encoding='utf-8',timeout=40)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            print(result.stdout)


if __name__=='__main__':unittest.main()

"""隔离完整窗口，验证型号选择只连接一种协议，且非法配置不回退。"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SMOKE = r'''
import sys
from unittest.mock import patch
from main_window import MainWindow
from pressure_meter_serial_widget import PressureMeterSerialWidget
from motor_control_widget import MotorControlWidget
from qt_compat import QtWidgets, QtGui, QtCore

app = QtWidgets.QApplication([])
# Windows offscreen插件不会自动加载系统字体，显式加载仅用于截图验证。
from pathlib import Path
font_path = Path('C:/Windows/Fonts/msyh.ttc')
if font_path.exists():
    font_id = QtGui.QFontDatabase.addApplicationFont(str(font_path))
    families = QtGui.QFontDatabase.applicationFontFamilies(font_id)
    if families:
        app.setFont(QtGui.QFont(families[0], 9))
PressureMeterSerialWidget._auto_connect_last_working_port = lambda self: None
MainWindow.refresh_ports = lambda self: None
startup_connect = MainWindow._auto_connect_on_startup
MainWindow._auto_connect_on_startup = lambda self: None
window = MainWindow()
window.show()
app.processEvents()
expected = sys.argv[1]
legacy = window.motor_control_widget
if expected == 'zdt_x42s':
    assert window.motor_model == expected
    assert window.zdt_motor_test_widget is not None
    assert window.mapping_motor_control.backend is window.zdt_motor_test_widget
    assert window.force_position_mapping_widget._profile.model == expected
    assert window.force_position_mapping_widget._profile.steps_per_turn == 3200
    assert not legacy._communication_enabled
    assert not legacy.isEnabled()
    assert not legacy._service_thread.is_alive()
    with patch('motor_control_widget.PD42S1Motor') as old_driver:
        legacy.connect_can()
        try:
            legacy._connect_motor()
        except RuntimeError:
            pass
        else:
            raise AssertionError('legacy driver was not blocked')
        old_driver.assert_not_called()
    with patch.object(window.zdt_motor_test_widget, 'connect_can') as new_connect, patch.object(legacy, 'connect_can') as old_connect:
        window._startup_auto_connect_done = False
        startup_connect(window)
        new_connect.assert_called_once()
        old_connect.assert_not_called()
    index = next(i for i in range(window.control_tabs.count()) if window.control_tabs.tabText(i) == '新电机控制测试')
    window.control_tabs.setCurrentIndex(index)
    app.processEvents()
    if len(sys.argv) > 2:
        window.resize(1280, 900)
        app.processEvents()
        window.grab().save(sys.argv[2])
        mapping_index = next(i for i in range(window.control_tabs.count()) if window.control_tabs.tabText(i) == '电机位置/压力标定')
        window.control_tabs.setCurrentIndex(mapping_index)
        app.processEvents()
        window.grab().save(str(Path(sys.argv[2]).with_name('motor_mapping_step.png')))
elif expected == 'pd42s1':
    assert window.zdt_motor_test_widget is None
    assert legacy._communication_enabled
    assert window.force_position_mapping_widget._profile.model == expected
    assert legacy._service_thread.is_alive()
    with patch.object(legacy, 'connect_can') as old_connect:
        window._startup_auto_connect_done = False
        startup_connect(window)
        old_connect.assert_called_once()
else:
    assert window.motor_model is None
    assert window.motor_config_error
    assert window.zdt_motor_test_widget is None
    with patch.object(legacy, 'connect_can') as old_connect:
        startup_connect(window)
        old_connect.assert_not_called()
window.close()
if window.zdt_motor_test_widget:
    assert not window.zdt_motor_test_widget._thread.is_alive()
# 在解释器退出前完成Qt延迟销毁，避免离屏截图后的对象留到Python析构阶段。
window.deleteLater()
QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
app.processEvents()
print('motor selection passed:', expected)
'''


class SelectionTests(unittest.TestCase):
    def test_new_legacy_and_invalid_configuration(self):
        for model in ("zdt_x42s", "pd42s1", "invalid"):
            with self.subTest(model=model), tempfile.TemporaryDirectory() as directory:
                for path in (ROOT / "PY_SRIPT/algo_monitor").iterdir():
                    if path.is_file() and path.suffix in (".py", ".json"):
                        shutil.copy2(path, directory)
                target = Path(directory)
                (target / "motor_connection_config.json").write_text(json.dumps({"motor_model": model}), encoding="utf-8")
                script = target / "_motor_selection.py"
                script.write_text(SMOKE, encoding="utf-8")
                args = [sys.executable, str(script), model]
                if model == "zdt_x42s":
                    screenshot = ROOT / "tmp/zdt_motor_test.png"
                    screenshot.parent.mkdir(exist_ok=True)
                    args.append(str(screenshot))
                result = subprocess.run(args, cwd=directory,
                    env=dict(os.environ, QT_QPA_PLATFORM="offscreen", PYTHONIOENCODING="utf-8"),
                    capture_output=True, text=True, encoding="utf-8", timeout=45)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main(verbosity=2)

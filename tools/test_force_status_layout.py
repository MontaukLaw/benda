"""离屏验证真实按力控制布局，不连接设备、不启动任务、不写入配置或日志。

运行：python tools/test_force_status_layout.py
可用 PYQTGRAPH_QT_LIB 和 QT_SCALE_FACTOR 分别指定 Qt 绑定与界面缩放。
"""

import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PY_SRIPT" / "algo_monitor"))

from qt_compat import QtCore, QtGui, QtWidgets
from force_press_control_widget import ForcePressControlWidget
from wrapped_label import WrappedLabel


class LayoutOnlyForceWidget(ForcePressControlWidget):
    def __init__(self):
        QtWidgets.QGroupBox.__init__(self, "按力下压（压力－位置查表）")
        self.setObjectName("ForcePressControl")
        self._target_buttons = []
        self._motor_control = SimpleNamespace(capture_release_position=lambda: None)
        self._settings_path = ROOT / "tmp" / "unused-layout-settings.json"
        self._build_ui(self._load_settings())

    def _record_event(self, *args, **kwargs):
        pass


class ForceStatusLayoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        if os.name == "nt":
            for name in ("segoeui.ttf", "msyh.ttc"):
                QtGui.QFontDatabase.addApplicationFont("C:/Windows/Fonts/" + name)

    def setUp(self):
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        self.content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.content)
        layout.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetNoConstraint)
        policy = self.content.sizePolicy()
        policy.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Ignored)
        self.content.setSizePolicy(policy)
        self.widget = LayoutOnlyForceWidget()
        layout.addWidget(self.widget)
        layout.addStretch()
        self.scroll.setWidget(self.content)
        # 与主窗口一致：先建立子控件，再应用最终字体样式。
        self.set_font_size(14)
        self.scroll.resize(1000, 750)
        self.scroll.show()
        self.settle()

    def tearDown(self):
        self.scroll.close()
        self.scroll.deleteLater()
        self.app.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)

    def set_font_size(self, pixels):
        self.scroll.setStyleSheet(
            'QWidget {font-family:"Segoe UI", "Microsoft YaHei";'
            f'font-size:{pixels}px;background:#05070a;color:#d9e2ef;}}'
        )

    def settle(self):
        for _ in range(30):
            self.app.processEvents()

    def assert_text_fits(self, label):
        # 独立测量实际绘制区域，而不是只检查控件自己的尺寸提示。
        flags = QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.TextFlag.TextWordWrap
        flags = getattr(flags, "value", flags)
        rect = label.contentsRect().adjusted(
            label.margin(), label.margin(), -label.margin(), -label.margin()
        )
        needed = label.fontMetrics().boundingRect(
            QtCore.QRect(0, 0, rect.width(), 100000), flags, label.text()
        )
        self.assertGreaterEqual(rect.height(), needed.height(), label.objectName())
        self.assertLessEqual(needed.width(), rect.width(), label.objectName())
        self.assertTrue(label.parentWidget().rect().contains(label.geometry()))

    def test_status_changes_width_font_and_collapse(self):
        states = ("waiting", "writing", "verified", "flash_saved", "discarded", "lost", "failed")
        messages = (
            "region 0，类型 0存在节点保存或回读校验失败；请按提示排查，已停止且未改写Flash",
            "逐点回读失败：" + "CH4～CH7连续中文错误说明" * 15,
            "file=" + "very_long_identifier_without_spaces_" * 15 + "\n下一步：重新读取MCU参数",
        )
        label = self.widget.quick_correction_ram_status_label
        for pixels in (14, 18, 24):
            self.set_font_size(pixels)
            for width in (760, 1000, 1400, 760):
                self.scroll.resize(width, 600)
                for index, state in enumerate(states):
                    with self.subTest(pixels=pixels, width=width, state=state):
                        self.widget.set_quick_correction_ram_result(
                            {"state": state, "message": messages[index % len(messages)]}
                        )
                        self.settle()
                        self.assert_text_fits(label)
                        self.assertGreater(self.scroll.verticalScrollBar().maximum(), 0)
        large_height = label.height()
        self.widget.content.hide()
        self.widget.setMaximumHeight(32)
        self.widget.set_quick_correction_ram_result({"state": "verified", "message": "完成"})
        self.scroll.resize(1400, 600)
        self.set_font_size(14)
        self.settle()
        self.widget.setMaximumHeight(16777215)
        self.widget.content.show()
        self.settle()
        self.assert_text_fits(label)
        self.assertLess(label.height(), large_height)

    def test_neighboring_multiline_labels(self):
        self.widget.sweep_plan_label.setText(
            "低力0.1～2 N每0.1 N；高力2～30 N每1 N；共48档 × 升降2遍 × 1轮 = 96次有效采样；先以30.00 N热身5次"
        )
        for width in (760, 1000, 1400, 760):
            self.scroll.resize(width, 600)
            self.settle()
            for label in self.widget.findChildren(WrappedLabel):
                if label.isVisible() and label.text():
                    with self.subTest(width=width, text=label.text()[:30]):
                        self.assert_text_fits(label)


if __name__ == "__main__":
    unittest.main()

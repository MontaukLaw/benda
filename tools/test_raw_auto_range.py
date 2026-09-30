"""固定传感器数据重复刷新，验证Raw自动量程不会被状态标注撑大。"""
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'PY_SRIPT/algo_monitor'))

import pyqtgraph as pg
from qt_compat import QtWidgets
from main_window import MainWindow, NoWheelViewBox


class RawPlotHarness:
    _state_display = MainWindow._state_display
    _update_raw_plot_state_regions = MainWindow._update_raw_plot_state_regions
    _position_raw_state_labels = MainWindow._position_raw_state_labels

    def __init__(self):
        self.raw_plot = pg.PlotWidget(viewBox=NoWheelViewBox())
        self.raw_plot.resize(640, 320)
        self.raw_plot.setXRange(0, 99)
        self.raw_plot.setYRange(0, 8000)
        self.raw_plot.addLegend(offset=(12, 12))
        self.curve = self.raw_plot.plot(name='R0')
        self._raw_state_history_cache = None
        self._raw_state_regions = []
        self._raw_state_region_states = []
        self._raw_state_labels = []
        self.raw_plot.show()


class RawAutoRangeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.harness = RawPlotHarness()
        self.view = self.harness.raw_plot.getViewBox()
        self.app.processEvents()

    def tearDown(self):
        self.harness.raw_plot.close()

    def refresh(self, values, states, count=120):
        for _ in range(count):
            self.harness._update_raw_plot_state_regions(states)
            self.harness.curve.setData(values)
            self.view.updateAutoRange()
            self.app.processEvents()
        return self.view.viewRange()[1]

    def test_fixed_data_stays_bounded_with_state_labels(self):
        self.harness.raw_plot.enableAutoRange(axis='y', enable=True)
        bounds = self.refresh([1000, 1100] * 50, [0] * 50 + [7] * 50)
        self.assertGreater(bounds[0], 900, bounds)
        self.assertLess(bounds[1], 1200, bounds)
        again = self.refresh([1000, 1100] * 50, [0] * 50 + [7] * 50)
        for first, second in zip(bounds, again):
            self.assertAlmostEqual(first, second, places=6)

    def test_range_grows_and_shrinks_with_actual_data(self):
        self.harness.raw_plot.enableAutoRange(axis='y', enable=True)
        self.refresh([1000, 1100] * 50, [0] * 100)
        large = self.refresh([1000, 20000] * 50, [6] * 100)
        self.assertGreaterEqual(large[1], 20000)
        small = self.refresh([1000, 1100] * 50, [8] * 100)
        self.assertGreater(small[0], 900, small)
        self.assertLess(small[1], 1200, small)
        self.harness._update_raw_plot_state_regions([])
        cleared = self.refresh([1000, 1100] * 50, [-1] * 100)
        self.assertLess(cleared[1], 1200, cleared)

    def test_manual_range_stays_fixed(self):
        self.harness.raw_plot.setYRange(0, 60000, padding=0)
        bounds = self.refresh([1000, 1100] * 50, [0] * 50 + [8] * 50)
        self.assertEqual(bounds, [0, 60000])


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""4行5列传感阵列，可用固件上传的Signal或Raw Val计算质心。"""
import math
import time

from qt_compat import QtCore, QtGui, QtWidgets
from font_scale import scaled_font_size
from centroid_filter import CHANNEL_GRID, CentroidFilter


class CentroidView(QtWidgets.QWidget):
    """保持方格比例的轻量绘图控件，无额外采样线程或历史缓存。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.position_filter = CentroidFilter()
        self.smoothing_enabled = True
        self.scale_percent = 70
        self.raw_centroid = None
        self.centroid = None
        self._message = '等待20通道数据'
        self.setMinimumSize(320, 300)

    def set_scale_percent(self, value):
        self.scale_percent = max(30, min(100, int(value)))
        self.update()

    def grid_geometry(self):
        cell = min((self.width() - 48) / 5, (self.height() - 120) / 4) * self.scale_percent / 100
        left = (self.width() - cell * 5) / 2
        top = 56 + (self.height() - 120 - cell * 4) / 2
        return QtCore.QRectF(left, top, cell * 5, cell * 4), cell

    def set_signals(self, signals, timestamp=None, states=None, now=None, raw_values=None):
        timestamp = time.perf_counter() if timestamp is None else timestamp
        self.position_filter.update(signals, timestamp, states, raw_values=raw_values)
        self.position_filter.expire_hold(timestamp if now is None else now)
        filtered = self.position_filter.filtered_xy
        self.raw_centroid = self.position_filter.raw_xy
        centroid = filtered if self.smoothing_enabled else self.raw_centroid
        if self.position_filter.holding_position:
            # 锁定上一次实际显示的坐标，关闭平滑时也不能漂向残留信号。
            centroid = self.centroid
        elif (self.position_filter.release_hidden or self.position_filter.threshold_hidden
              or self.raw_centroid is None):
            centroid = None
        if signals is None or len(signals) != 20 or not all(math.isfinite(v) for v in signals):
            message = '等待20通道数据'
        elif self.position_filter.source_missing:
            message = '等待有效20路Raw Val（需Debug模式）'
        elif self.position_filter.holding_position:
            message = '信号暂失 · 红点位置锁定'
        elif self.position_filter.threshold_hidden:
            message = f'Signal未达到有效触摸条件（阈值{self.position_filter.signal_threshold}） · 质心已隐藏'
        elif self.position_filter.release_hidden:
            message = '释放尾段 · 质心已隐藏'
        elif centroid is None:
            message = '当前数据无有效权重 · 无质心'
        else:
            mode = '最强2×2' if self.position_filter.local_2x2_enabled else '全部20通道'
            source = 'Raw Val' if self.position_filter.raw_enabled else 'Signal'
            if self.position_filter.raw_difference_enabled:
                source = 'Raw均值差归一化'
            message = (f'{source} · {mode}质心 · X/Y独立平滑' if self.smoothing_enabled
                       else f'{source} · {mode}质心 · 平滑关闭')
        if centroid != self.centroid or message != self._message:
            self.centroid, self._message = centroid, message
            self.update()
        return dict(raw_xy=self.raw_centroid, filtered_xy=filtered, displayed_xy=centroid,
                    smoothing_enabled=self.smoothing_enabled,
                    scale_percent=self.scale_percent, **self.position_filter.visibility(),
                    filter=self.position_filter.parameters())

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#080d13'))
        painter.setPen(QtGui.QColor('#d9e7f0'))
        title_font = QtGui.QFont(self.font())
        title_font.setPointSize(scaled_font_size(self, 13))
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.drawText(QtCore.QRectF(24, 16, self.width() - 48, max(28,painter.fontMetrics().height()+4)),
                         QtCore.Qt.AlignmentFlag.AlignCenter, '4 × 5 传感阵列')

        grid, cell = self.grid_geometry()
        left, top = grid.left(), grid.top()
        painter.fillRect(grid, QtGui.QColor('#101d29'))
        painter.setPen(QtGui.QPen(QtGui.QColor('#496476'), 1))
        for col in range(6):
            x = left + col * cell
            painter.drawLine(QtCore.QPointF(x, top), QtCore.QPointF(x, grid.bottom()))
        for row in range(5):
            y = top + row * cell
            painter.drawLine(QtCore.QPointF(left, y), QtCore.QPointF(grid.right(), y))

        label_font = QtGui.QFont(self.font())
        label_font.setPixelSize(scaled_font_size(self, max(6, min(13, int(cell / 4)))))
        painter.setFont(label_font)
        painter.setPen(QtGui.QColor('#9bb7ca'))
        for row, channels in enumerate(CHANNEL_GRID):
            for col, channel in enumerate(channels):
                inset = min(8, cell * .08)
                label = QtCore.QRectF(left + col * cell + inset, top + row * cell + inset,
                                     cell - inset * 2, min(max(24,painter.fontMetrics().height()+4), cell * .5))
                painter.drawText(label, QtCore.Qt.AlignmentFlag.AlignLeft |
                                 QtCore.Qt.AlignmentFlag.AlignVCenter, f'CH{channel}')

        if self.centroid is not None:
            x, y = self.centroid
            painter.setPen(QtGui.QPen(QtGui.QColor('#ffc5c5'), 1.5))
            painter.setBrush(QtGui.QColor('#ff3838'))
            painter.drawEllipse(QtCore.QPointF(left + x * cell, top + y * cell), 8, 8)

        painter.setPen(QtGui.QColor('#9bb7ca'))
        painter.setFont(self.font())
        height = max(28,painter.fontMetrics().height()+4)
        painter.drawText(QtCore.QRectF(16, self.height() - height - 16, self.width() - 32, height),
                         QtCore.Qt.AlignmentFlag.AlignCenter, self._message)

"""固定4行×5列的 QPainter 视图及模拟鼠标输入。"""

from PySide6 import QtCore, QtGui, QtWidgets
from .config import SENSOR_POSITIONS


class SensorWidget(QtWidgets.QWidget):
    simulation_position = QtCore.Signal(float, float, bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config.copy()
        self.result = None
        self.trail = ()
        self.simulation_enabled = False
        self._pressed = False
        self.setMinimumSize(500, 400)
        self.setMouseTracking(True)

    def set_snapshot(self, result, trail, config):
        self.result = result
        self.trail = trail
        self.config = config
        self.update()

    def grid_rect(self):
        available = QtCore.QRectF(38, 38, self.width() - 62, self.height() - 66)
        size = min(available.width() / 5, available.height() / 4)
        return QtCore.QRectF(available.center().x() - size * 2.5,
                             available.center().y() - size * 2, size * 5, size * 4)

    def _point(self, x, y):
        grid = self.grid_rect()
        return QtCore.QPointF(grid.left() + (x + 0.5) * grid.width() / 5,
                             grid.top() + (y + 0.5) * grid.height() / 4)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#111923"))
        grid = self.grid_rect()
        cw, ch = grid.width() / 5, grid.height() / 4
        c, r = self.config, self.result
        p.setFont(QtGui.QFont("Microsoft YaHei", 9))
        p.setPen(QtGui.QColor("#9faec0"))
        for x in range(5):
            p.drawText(QtCore.QRectF(grid.left() + x * cw, grid.top() - 28, cw, 24),
                       QtCore.Qt.AlignmentFlag.AlignCenter, f"Col{x} / X={x}")
        for y in range(4):
            p.drawText(QtCore.QRectF(grid.left() - 38, grid.top() + y * ch, 36, ch),
                       QtCore.Qt.AlignmentFlag.AlignCenter, f"R{y}")
        for index, (sx, sy) in SENSOR_POSITIONS.items():
            rect = QtCore.QRectF(grid.left() + sx * cw + 2, grid.top() + sy * ch + 2, cw - 4, ch - 4)
            signal = r.signal[index] if r else 0
            intensity = max(0.0, min(1.0, signal / c.visualization_max_signal))
            p.fillRect(rect, QtGui.QColor(int(25 + 55 * intensity), int(39 + 150 * intensity), int(53 + 172 * intensity)))
            p.setPen(QtGui.QPen(QtGui.QColor("#344455"), 1))
            p.drawRect(rect)
            if r and r.baseline_ready:
                if c.show_roi and index in r.roi_indices:
                    roi_color = QtGui.QColor("#6999d5")
                    roi_color.setAlpha(round(60 + 195 * (r.roi_weights[index] if r.roi_weights else 1)))
                    p.setPen(QtGui.QPen(roi_color, 2, QtCore.Qt.PenStyle.DashLine))
                    p.drawRect(rect.adjusted(3, 3, -3, -3))
                if c.show_active_sensors and index in r.active_indices:
                    p.setPen(QtGui.QPen(QtGui.QColor("#63e3aa"), 2))
                    p.drawRect(rect.adjusted(7, 7, -7, -7))
                if c.show_max and index == r.max_index and r.max_signal > 0:
                    p.setPen(QtGui.QPen(QtGui.QColor("#ffd166"), 4))
                    p.drawRect(rect.adjusted(1, 1, -1, -1))
                if index in r.below_baseline_indices:
                    p.setPen(QtGui.QPen(QtGui.QColor("#ff9a6b"), 2, QtCore.Qt.PenStyle.DotLine))
                    p.drawRect(rect.adjusted(10, 10, -10, -10))
            p.setPen(QtGui.QColor("#f0f5fa"))
            lines = [f"CH{index}"]
            if c.show_raw:
                value = (r.adjusted_raw or r.raw)[index] if r else None
                lines.append(f"R: {value:.3f}".rstrip("0").rstrip(".") if r else "R: —")
            if c.show_signal:
                lines.append(f"S: {signal:.0f}" if r else "S: —")
            if r and index in r.below_baseline_indices:
                lines.append(f"Δ: {(r.adjusted_raw or r.raw)[index] - r.baseline[index]:.0f}")
            p.drawText(rect.adjusted(8, 12, -8, -10), QtCore.Qt.AlignmentFlag.AlignCenter, "\n".join(lines))
        if c.show_trail and len(self.trail) > 1:
            path = QtGui.QPainterPath()
            path.moveTo(self._point(*self.trail[0][1:]))
            for point in self.trail[1:]:
                path.lineTo(self._point(*point[1:]))
            p.setPen(QtGui.QPen(QtGui.QColor("#f38bb8"), 2))
            p.drawPath(path)
        if r and r.touched and r.position_valid:
            if c.show_raw_centroid and r.x_raw is not None:
                p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
                p.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
                p.drawEllipse(self._point(r.x_raw, r.y_raw), 8, 8)
            if c.show_filtered_cursor and r.x is not None:
                p.setBrush(QtGui.QColor("#f38bb8"))
                p.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
                p.drawEllipse(self._point(r.x, r.y), 12, 12)

    def _emit_mouse(self, position, pressed):
        grid = self.grid_rect()
        x = max(0.0, min(4.0, (position.x() - grid.left()) / (grid.width() / 5) - 0.5))
        y = max(0.0, min(3.0, (position.y() - grid.top()) / (grid.height() / 4) - 0.5))
        self.simulation_position.emit(x, y, pressed)

    def mousePressEvent(self, event):
        if self.simulation_enabled and event.button() == QtCore.Qt.MouseButton.LeftButton and self.grid_rect().contains(event.position()):
            self._pressed = True
            self._emit_mouse(event.position(), True)
            event.accept()

    def mouseMoveEvent(self, event):
        if self.simulation_enabled:
            self._emit_mouse(event.position(), self._pressed)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._pressed = False
            if self.simulation_enabled:
                self._emit_mouse(event.position(), False)

    def leaveEvent(self, event):
        # 拖动出界仍以实际鼠标释放事件结束；Qt按下时自动抓取鼠标。
        if self.simulation_enabled and not self._pressed:
            self.simulation_position.emit(2, 1.5, False)

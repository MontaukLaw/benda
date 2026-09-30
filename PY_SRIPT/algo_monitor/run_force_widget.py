import math

from qt_compat import QtCore, QtGui, QtWidgets
from low_force_calibration import low_force_display


class DirectionArrowWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._theta = 0.0
        self._force = 0.0
        self.setMinimumHeight(150)

    def set_force(self, theta, force):
        self._theta = float(theta)
        self._force = max(0.0, float(force))
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        try:
            antialiasing = getattr(
                QtGui.QPainter,
                "Antialiasing",
                QtGui.QPainter.RenderHint.Antialiasing,
            )
            painter.setRenderHint(antialiasing, True)
            center = QtCore.QPointF(self.width() * 0.5, self.height() * 0.5)
            radius = max(20.0, min(self.width(), self.height()) * 0.32)

            painter.setPen(QtGui.QPen(QtGui.QColor("#344255"), 2))
            painter.drawEllipse(center, radius, radius)
            painter.drawLine(
                QtCore.QPointF(center.x() - radius, center.y()),
                QtCore.QPointF(center.x() + radius, center.y()),
            )
            painter.drawLine(
                QtCore.QPointF(center.x(), center.y() - radius),
                QtCore.QPointF(center.x(), center.y() + radius),
            )

            if self._force <= 0.0:
                return

            length = radius * 0.85
            dx = math.cos(self._theta)
            dy = -math.sin(self._theta)
            tip = QtCore.QPointF(center.x() + dx * length, center.y() + dy * length)
            side_x = -dy
            side_y = dx
            base = QtCore.QPointF(tip.x() - dx * 14.0, tip.y() - dy * 14.0)
            arrow = QtGui.QPolygonF([
                tip,
                QtCore.QPointF(base.x() + side_x * 6.0, base.y() + side_y * 6.0),
                QtCore.QPointF(base.x() - side_x * 6.0, base.y() - side_y * 6.0),
            ])

            pen = QtGui.QPen(QtGui.QColor("#35d6a5"), 4)
            pen.setCapStyle(getattr(QtCore.Qt, "RoundCap", QtCore.Qt.PenCapStyle.RoundCap))
            painter.setPen(pen)
            painter.drawLine(center, tip)
            painter.setBrush(QtGui.QColor("#35d6a5"))
            painter.drawPolygon(arrow)
        finally:
            painter.end()


class RunForceWidget(QtWidgets.QGroupBox):
    def __init__(self, title, parent=None):
        super().__init__(title, parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.arrow = DirectionArrowWidget()
        self.normal_label = QtWidgets.QLabel()
        self.tangential_label = QtWidgets.QLabel()
        self.direction_label = QtWidgets.QLabel()
        layout.addWidget(self.arrow, 1)
        layout.addWidget(self.normal_label)
        layout.addWidget(self.tangential_label)
        layout.addWidget(self.direction_label)
        self.set_vector(None)

    def set_vector(self, vector):
        if vector is None:
            normal_force = 0.0
            tangential_force = 0.0
            normal_display = 0.0
            tangential_display = 0.0
            theta = 0.0
            normal_suffix = ""
            tangential_suffix = ""
        else:
            normal_force = float(vector["normal_force"])
            tangential_force = float(vector["tangential_force"])
            theta = float(vector["theta"])
            normal_display = normal_force
            tangential_display = tangential_force
            normal_suffix = (
                " N" if vector.get("normal_calibration_valid") else " (raw)"
            )
            tangential_suffix = (
                " N" if vector.get("tangential_calibration_valid") else " (raw)"
            )

        self.arrow.set_force(theta, tangential_force)
        self.normal_label.setText(f"Normal: {normal_display:.3f}{normal_suffix}")
        low_display = low_force_display(vector)
        if low_display is not None:
            self.normal_label.setText(f"法向力：{low_display[0]}")
            self.normal_label.setToolTip(low_display[1])
        self.tangential_label.setText(
            f"Tangential: {tangential_display:.3f}{tangential_suffix}"
        )
        self.direction_label.setText(
            f"Direction: {theta:.3f} rad / {math.degrees(theta):.1f}°"
        )

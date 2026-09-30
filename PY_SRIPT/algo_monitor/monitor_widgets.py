from qt_compat import QtCore, QtGui, QtWidgets, Signal


class FlowLayout(QtWidgets.QLayout):
    """根据可用宽度自动换行排列控件。"""

    def __init__(self, parent=None, margin=0, horizontal_spacing=6, vertical_spacing=6):
        super().__init__(parent)
        self._items = []
        self._horizontal_spacing = int(horizontal_spacing)
        self._vertical_spacing = int(vertical_spacing)
        self.setContentsMargins(margin, margin, margin, margin)

    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, index):
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index):
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self):
        orientations_type = getattr(QtCore.Qt, "Orientations", None)
        orientation_type = getattr(QtCore.Qt, "Orientation", None)
        if orientations_type is not None:
            return orientations_type(0)
        if orientation_type is not None:
            return orientation_type(0)
        return QtCore.Qt.Horizontal & QtCore.Qt.Vertical

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._do_layout(QtCore.QRect(0, 0, width, 0), True)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._do_layout(rect, False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QtCore.QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        margins = self.contentsMargins()
        size += QtCore.QSize(
            margins.left() + margins.right(),
            margins.top() + margins.bottom(),
        )
        return size

    def set_widgets(self, widgets):
        while self._items:
            self.takeAt(0)
        for widget in widgets:
            self.addWidget(widget)
        self.invalidate()

    def _do_layout(self, rect, test_only):
        margins = self.contentsMargins()
        effective = rect.adjusted(
            margins.left(),
            margins.top(),
            -margins.right(),
            -margins.bottom(),
        )
        x = effective.x()
        y = effective.y()
        line_height = 0

        for item in self._items:
            hint = item.sizeHint()
            next_x = x + hint.width() + self._horizontal_spacing
            if (
                line_height > 0
                and next_x - self._horizontal_spacing > effective.right() + 1
            ):
                x = effective.x()
                y += line_height + self._vertical_spacing
                next_x = x + hint.width() + self._horizontal_spacing
                line_height = 0
            if not test_only:
                item.setGeometry(QtCore.QRect(QtCore.QPoint(x, y), hint))
            x = next_x
            line_height = max(line_height, hint.height())

        return (
            y
            + line_height
            - rect.y()
            + margins.bottom()
        )


class ColumnToggleButton(QtWidgets.QPushButton):
    """支持点击显隐和拖动排序的数据列表头按钮。"""

    reorder_requested = Signal(str, str, bool)
    MIME_TYPE = "application/x-sensor-table-column"

    def __init__(self, column_key, title, parent=None):
        super().__init__(title, parent)
        self.column_key = str(column_key)
        self._press_position = None
        self.setObjectName("ColumnToggleButton")
        self.setCheckable(True)
        self.setChecked(True)
        self.setAcceptDrops(True)
        open_hand_cursor = getattr(QtCore.Qt, "OpenHandCursor", None)
        if open_hand_cursor is None:
            open_hand_cursor = QtCore.Qt.CursorShape.OpenHandCursor
        self.setCursor(open_hand_cursor)
        self.setToolTip("单击切换显示；拖动改变下方数据表的列顺序")

    @staticmethod
    def _event_position(event):
        if hasattr(event, "position"):
            return event.position().toPoint()
        return event.pos()

    def mousePressEvent(self, event):
        left_button = getattr(QtCore.Qt, "LeftButton", None)
        if left_button is None:
            left_button = QtCore.Qt.MouseButton.LeftButton
        if event.button() == left_button:
            self._press_position = self._event_position(event)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        left_button = getattr(QtCore.Qt, "LeftButton", None)
        if left_button is None:
            left_button = QtCore.Qt.MouseButton.LeftButton
        if (
            self._press_position is None
            or not event.buttons() & left_button
            or (
                self._event_position(event) - self._press_position
            ).manhattanLength() < QtWidgets.QApplication.startDragDistance()
        ):
            super().mouseMoveEvent(event)
            return

        drag = QtGui.QDrag(self)
        mime_data = QtCore.QMimeData()
        mime_data.setData(self.MIME_TYPE, self.column_key.encode("utf-8"))
        drag.setMimeData(mime_data)
        drag.setPixmap(self.grab())
        self.setDown(False)
        self._press_position = None
        move_action = getattr(QtCore.Qt, "MoveAction", None)
        if move_action is None:
            move_action = QtCore.Qt.DropAction.MoveAction
        if hasattr(drag, "exec"):
            drag.exec(move_action)
        else:
            drag.exec_(move_action)

    def mouseReleaseEvent(self, event):
        self._press_position = None
        super().mouseReleaseEvent(event)

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat(self.MIME_TYPE):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event):
        if event.mimeData().hasFormat(self.MIME_TYPE):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        if not event.mimeData().hasFormat(self.MIME_TYPE):
            event.ignore()
            return
        try:
            source_key = bytes(
                event.mimeData().data(self.MIME_TYPE)
            ).decode("utf-8")
        except UnicodeDecodeError:
            event.ignore()
            return
        drop_after = self._event_position(event).x() >= self.width() / 2
        self.reorder_requested.emit(source_key, self.column_key, drop_after)
        event.acceptProposedAction()



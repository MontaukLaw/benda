"""为嵌套滚动布局中的多行提示保留完整的文本高度。"""

from qt_compat import QtCore, QtWidgets


class WrappedLabel(QtWidgets.QLabel):
    """按当前宽度和最终样式更新最小高度，防止父布局压缩多行文字。"""

    def __init__(self, text="", parent=None):
        self._updating_height = False
        super().__init__(text, parent)
        self.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.setWordWrap(True)
        self.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop
        )
        policy = self.sizePolicy()
        policy.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Ignored)
        policy.setVerticalPolicy(QtWidgets.QSizePolicy.Policy.Minimum)
        self.setSizePolicy(policy)

    def _update_text_height(self):
        # 使用QLabel自身的测量，包含字体、换行、边框和样式内边距。
        if self._updating_height or self.width() <= 0:
            return
        self._updating_height = True
        try:
            self.ensurePolished()
            # Qt的heightForWidth会受旧最小高度限制，测量前解除以允许长消息缩回短消息。
            self.setMinimumHeight(0)
            height = max(0, self.heightForWidth(self.width()))
            if self.minimumHeight() != height:
                self.setMinimumHeight(height)
                self.updateGeometry()
        finally:
            self._updating_height = False

    def setText(self, text):
        if text == self.text():
            return
        super().setText(text)
        self._update_text_height()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_text_height()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() in (
            QtCore.QEvent.Type.FontChange,
            QtCore.QEvent.Type.ApplicationFontChange,
            QtCore.QEvent.Type.StyleChange,
            QtCore.QEvent.Type.ContentsRectChange,
        ):
            self._update_text_height()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_text_height()

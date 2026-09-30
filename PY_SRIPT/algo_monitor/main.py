import sys

from main_window import MainWindow
from qt_compat import QtWidgets


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app_exec = app.exec if hasattr(app, "exec") else app.exec_
    return app_exec()


if __name__ == "__main__":
    raise SystemExit(main())

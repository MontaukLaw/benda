"""从任意工作目录启动：py -3.14 PY_SRIPT/new_all_on_python/main.py。"""

import argparse
from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "new_all_on_python"


def main(argv=None):
    parser = argparse.ArgumentParser(description="4×5压力阵列触摸调试工具")
    parser.add_argument("--simulate", action="store_true", help="启动后自动进入鼠标模拟模式")
    args = parser.parse_args(argv)
    try:
        from PySide6 import QtWidgets
        from .main_window import MainWindow
    except ImportError as exc:
        print(f"缺少依赖：{exc}\n请运行 py -3.14 -m pip install -r {Path(__file__).parent / 'requirements.txt'}", file=sys.stderr)
        return 1
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("4×5 Touch Debugger")
    window = MainWindow(auto_connect=not args.simulate)
    window.show()
    if args.simulate:
        window.start_simulation()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

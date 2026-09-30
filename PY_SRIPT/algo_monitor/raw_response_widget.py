from qt_compat import QtCore, QtWidgets


class RawResponseWidget(QtWidgets.QGroupBox):
    def __init__(self, buffer, parent=None):
        super().__init__("四通道施力响应（固定实验参考）", parent)
        self.buffer = buffer
        layout = QtWidgets.QVBoxLayout(self)
        controls = QtWidgets.QHBoxLayout()
        self.capture_button = QtWidgets.QPushButton("采集2秒参考")
        self.capture_button.clicked.connect(self.capture)
        controls.addWidget(self.capture_button)
        clear = QtWidgets.QPushButton("清除参考")
        clear.clicked.connect(lambda: self.buffer.clear_raw_reference())
        controls.addWidget(clear)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        controls.addWidget(self.status, 1)
        layout.addLayout(controls)
        self.metrics = QtWidgets.QLabel()
        self.metrics.setTextFormat(getattr(QtCore.Qt, "TextFormat", QtCore.Qt).RichText)
        self.metrics.setWordWrap(True)
        layout.addWidget(self.metrics)
        grid = QtWidgets.QGridLayout()
        self.cells = {}
        for col, ch in enumerate(range(4, 8), 1):
            grid.addWidget(QtWidgets.QLabel(f"CH{ch}"), 0, col)
        for row, (key, title) in enumerate((("reference", "参考Raw"), ("delta", "ΔRaw（counts）"), ("shares_percent", "增量占比")), 1):
            grid.addWidget(QtWidgets.QLabel(title), row, 0)
            self.cells[key] = []
            for col in range(1, 5):
                label = QtWidgets.QLabel("--")
                label.setStyleSheet("font-size:18px; font-weight:600;")
                self.cells[key].append(label)
                grid.addWidget(label, row, col)
        layout.addLayout(grid)
        hint = QtWidgets.QLabel("先保持当前预紧力不变并采集参考，再施力。参考不跟随baseline；占比表示电信号响应，不能直接当作受力占比。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.setToolTip("参考采集2秒，至少30帧；各通道时间标准差及前后半段均值差均不超过3 counts。平均增量须大于max(2 counts, 3×参考最大时间标准差)，且各增量非负，才显示CV和占比。")
        self.update_sample(None)

    def capture(self):
        self.buffer.start_raw_reference()
        self.update_sample(self.buffer.latest_sensor_sample())

    def update_sample(self, sample):
        state = (sample or {}).get("raw_response") or {}
        result = state.get("result")
        self.capture_button.setEnabled(not state.get("capturing", False))
        self.status.setText(state.get("message", "等待Debug Hardware Raw"))
        for key, labels in self.cells.items():
            values = result.get(key) if result else None
            for i, label in enumerate(labels):
                if values is None:
                    label.setText("--")
                else:
                    label.setText(f"{values[i]:+.2f}" if key == "delta" else f"{values[i]:.2f}" + ("%" if key == "shares_percent" else ""))
        def number(value):
            return f'<span style="font-size:18px; font-weight:600;">{value}</span>'
        if result:
            cv = "--" if result['cv_percent'] is None else f"{result['cv_percent']:.2f}%"
            mean_text = f"{result['mean']:+.2f}"
            std_text = f"{result['stddev']:.2f}"
            self.metrics.setText(
                f"ΔRaw均值 {number(mean_text)} · 增量标准差 {number(std_text)} counts · "
                f"增量CV {number(cv)} · {result['status']}"
            )
        else:
            self.metrics.setText("ΔRaw均值 -- · 增量标准差 -- · 增量CV --")

"""手动输入四个Signal和，独立读写MCU低力表并显示实时低力结果。"""
import math
import secrets
import json
from pathlib import Path

from config import COMMAND_TYPE_READ_LOW_FORCE_CALIBRATION, COMMAND_TYPE_SAVE_LOW_FORCE_CALIBRATION
from low_force_calibration import MASSES_G, FORCES_N, DETECTION_N, validate_points, validate_detection_counts, low_force_display
from qt_compat import QtCore, QtWidgets


class LowForceCalibrationWidget(QtWidgets.QWidget):
    def __init__(self, reader_provider, sample_provider, busy_provider=lambda: False, parent=None, settings_path=None):
        super().__init__(parent)
        self._reader_provider = reader_provider
        self._sample_provider = sample_provider
        self._busy_provider = busy_provider
        self._pending = None
        self._inputs_dirty = False
        self._auto_read_requested = False
        self._last_auto_reader = None
        self._settings_path = Path(settings_path) if settings_path is not None else Path(__file__).with_name("low_force_display_config.json")
        self._display_mode = "both"
        try:
            settings = json.loads(self._settings_path.read_text(encoding="utf-8"))
            mode = settings.get("display_mode") if isinstance(settings, dict) else None
            if mode in ("original", "optimized", "both"):
                self._display_mode = mode
        except (OSError, ValueError):
            pass
        layout = QtWidgets.QVBoxLayout(self)
        hint = QtWidgets.QLabel("依次放置10、20、50、100 g砝码，手动填写CH4～CH7的Signal和（counts）。\n"
                               "F = 质量(g) ÷ 1000 × 9.80665；正常测量低于0.09 N归零，达到1 N提示超出量程。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        display_row = QtWidgets.QHBoxLayout()
        display_row.addWidget(QtWidgets.QLabel("力值显示"))
        self.display_mode_combo = QtWidgets.QComboBox()
        for title, mode in (("显示原始值", "original"), ("显示优化后的值", "optimized"), ("同时显示两个值", "both")):
            self.display_mode_combo.addItem(title, mode)
        self.display_mode_combo.setCurrentIndex(self.display_mode_combo.findData(self._display_mode))
        display_row.addWidget(self.display_mode_combo)
        self.display_settings_status = QtWidgets.QLabel()
        display_row.addWidget(self.display_settings_status)
        display_row.addStretch(1)
        layout.addLayout(display_row)
        self.signal_label = QtWidgets.QLabel("当前Signal和：-- counts")
        self.force_label = QtWidgets.QLabel("-- N")
        self.optimized_force_label = QtWidgets.QLabel("-- N")
        self.signal_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        self.force_label.setStyleSheet("font-size: 24px; font-weight: 700; color: #ffd966;")
        self.optimized_force_label.setStyleSheet("font-size: 24px; font-weight: 700; color: #a9e881;")
        self.force_label.setWordWrap(True)
        self.optimized_force_label.setWordWrap(True)
        layout.addWidget(self.signal_label)
        layout.addWidget(self.force_label)
        layout.addWidget(self.optimized_force_label)
        self.measured_force_label = QtWidgets.QLabel()
        self.measured_force_label.setStyleSheet("font-size: 18px;")
        self.measured_force_label.setWordWrap(True)
        layout.addWidget(self.measured_force_label)
        self.table = QtWidgets.QTableWidget(4, 3)
        self.table.setHorizontalHeaderLabels(["砝码质量 (g)", "对应力 (N)", "CH4～CH7 Signal和 (counts)"])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.table.setMinimumHeight(220)
        self.inputs = []
        for row, (mass, force) in enumerate(zip(MASSES_G, FORCES_N)):
            for column, text in enumerate((str(mass), f"{force:.6f}")):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(row, column, item)
            editor = QtWidgets.QLineEdit()
            editor.setPlaceholderText("手动输入Signal和")
            editor.setStyleSheet("font-size: 18px;")
            self.inputs.append(editor)
            self.table.setCellWidget(row, 2, editor)
            self.table.setRowHeight(row, 42)
        layout.addWidget(self.table)
        detection_row = QtWidgets.QHBoxLayout()
        detection_row.addWidget(QtWidgets.QLabel("最低检测线（Signal和 / counts）"))
        self.detection_input = QtWidgets.QLineEdit("0")
        self.detection_input.setStyleSheet("font-size: 18px;")
        self.detection_input.setToolTip("填写后需点击保存才会更新MCU；0关闭模拟，非零值必须小于10 g标定点。需要V19及更新固件。")
        detection_row.addWidget(self.detection_input)
        layout.addLayout(detection_row)
        buttons = QtWidgets.QHBoxLayout()
        self.read_button = QtWidgets.QPushButton("读取 MCU Flash")
        self.save_button = QtWidgets.QPushButton("保存到 MCU Flash 并应用")
        self.read_button.clicked.connect(lambda: self._request(False))
        self.save_button.clicked.connect(lambda: self._request(True))
        buttons.addWidget(self.read_button)
        buttons.addWidget(self.save_button)
        layout.addLayout(buttons)
        self.status_label = QtWidgets.QLabel("尚未读取标定表。保存成功后立即生效，下次开机自动加载。")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.input_validation_label = QtWidgets.QLabel()
        self.input_validation_label.setWordWrap(True)
        self.input_validation_label.setStyleSheet("color: #ffd966;")
        layout.addWidget(self.input_validation_label)
        for editor in (*self.inputs, self.detection_input):
            editor.textEdited.connect(self._mark_parameters_edited)
        note = QtWidgets.QLabel("10 g约0.0981 N，高于0.09 N检测线。优化值向10、20、50、100 g平滑吸附。\n"
                               "最低检测线设为0关闭模拟。启用后：CH4～CH7 Signal和持续严格超过检测线，\n"
                               "且实际换算低于10 g时，模拟原始值为8.88 g，模拟优化值为10.00～10.99 g。\n"
                               "输出不依赖FLOATING状态；实际换算值仍同帧上传并保留在CSV中。此设置需要V19固件。")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch(1)
        self._timeout = QtCore.QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.timeout.connect(lambda: self.cancel_pending("操作超时，保存结果未知；请读取Flash核对。"))
        self._refresh_timer = QtCore.QTimer(self)
        self._refresh_timer.timeout.connect(self.refresh)
        self._refresh_timer.start(200)
        self.display_mode_combo.currentIndexChanged.connect(self._save_display_mode)
        self.refresh()

    def _mark_parameters_edited(self, _text):
        if self._pending is None:
            self._inputs_dirty = True
            self.status_label.setText("输入已修改，尚未下发MCU。请点击“保存到 MCU Flash 并应用”；四点为空时先读取已有标定，再修改检测线。")
            self._update_input_validation()

    def _update_input_validation(self):
        message = ""
        if self._inputs_dirty:
            try:
                points = validate_points(editor.text().strip() for editor in self.inputs)
                validate_detection_counts(self.detection_input.text().strip(), points)
            except (ValueError, OverflowError) as exc:
                message = f"当前输入不能保存：{exc}。力值显示仍以MCU已应用的标定为准。"
        self.input_validation_label.setText(message)
        self.input_validation_label.setVisible(bool(message))

    def showEvent(self, event):
        super().showEvent(event)
        self._auto_read_requested = True
        self._try_auto_read()

    def _try_auto_read(self):
        if not self.isVisible():
            return
        reader = self._reader_provider()
        if reader is not self._last_auto_reader:
            self._last_auto_reader = reader
            self._auto_read_requested = True
        if not self._auto_read_requested or self._pending is not None:
            return
        if self._inputs_dirty:
            self._auto_read_requested = False
            self.status_label.setText("有尚未保存的修改，已保留输入并跳过自动读取。需要覆盖输入时请手动点击“读取 MCU Flash”。")
            return
        if reader is None or not reader._protocol_version_confirmed:
            self.status_label.setText("等待MCU连接及设备识别，完成后自动读取标定。")
            return
        if reader._protocol_version < 17:
            self._auto_read_requested = False
            self.status_label.setText("当前固件不支持读取低力标定，需要V17或更新固件。")
            return
        if getattr(reader, "_channel_count", 0) == 0 or self._busy_provider():
            self.status_label.setText("等待设备通道信息或其他标定传输完成，随后自动读取。")
            return
        self._auto_read_requested = False
        self._request(False)

    def _save_display_mode(self):
        self._display_mode = self.display_mode_combo.currentData()
        self.refresh()
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            target = QtCore.QSaveFile(str(self._settings_path))
            content = (json.dumps({"version": 1, "display_mode": self._display_mode}, ensure_ascii=False) + "\n").encode("utf-8")
            if not target.open(QtCore.QIODevice.OpenModeFlag.WriteOnly):
                raise OSError(target.errorString())
            if target.write(content) != len(content):
                target.cancelWriting()
                raise OSError("显示配置写入不完整")
            if not target.commit():
                raise OSError(target.errorString())
            self.display_settings_status.setText("显示方式已保存")
        except OSError as exc:
            self.display_settings_status.setText(f"显示方式未保存：{exc}")

    def _set_busy(self, busy):
        self.table.setEnabled(not busy)
        self.detection_input.setEnabled(not busy)
        self.read_button.setEnabled(not busy)
        self.save_button.setEnabled(not busy)

    def cancel_pending(self, message="连接已变化，请重新读取MCU标定表。"):
        self._pending = None
        self._timeout.stop()
        self._set_busy(False)
        self.status_label.setText(message)

    def _request(self, save):
        if self._pending is not None:
            return
        self._auto_read_requested = False
        reader = self._reader_provider()
        if reader is None or not reader._protocol_version_confirmed or reader._protocol_version < 17:
            self.status_label.setText("需要连接并收到V17或更新固件的数据，才能读写低力表。")
            return
        if getattr(reader, "_channel_count", 0) != 8:
            self.status_label.setText("请等待八通道数据；本标定需要完整CH4～CH7。")
            return
        if self._busy_provider():
            self.status_label.setText("其他标定传输正在进行，请完成后重试。")
            return
        try:
            points = validate_points(editor.text().strip() for editor in self.inputs) if save else None
            detection_counts = validate_detection_counts(self.detection_input.text().strip(), points) if save else None
        except (ValueError, OverflowError) as exc:
            self.status_label.setText(f"输入无效：{exc}。本次未下发，MCU参数未改变。")
            return
        if save and reader._protocol_version < 19 and detection_counts != 0:
            self.status_label.setText("最低检测线需要V19或更新固件；旧固件只能保存四点标定。")
            return
        token = secrets.randbits(32)
        operation = COMMAND_TYPE_SAVE_LOW_FORCE_CALIBRATION if save else COMMAND_TYPE_READ_LOW_FORCE_CALIBRATION
        self._pending = (operation, token, points, detection_counts if reader._protocol_version >= 19 else None)
        self._set_busy(True)
        self.status_label.setText("正在保存并校验Flash…" if save else "正在读取Flash并加载到RAM…")
        self._timeout.start(5000)
        args = (operation, token, points, detection_counts or 0.0) if reader._protocol_version >= 19 else (operation, token, points)
        if not reader.low_force_command(*args):
            self.cancel_pending("发送失败，请检查串口连接。")

    def handle_response(self, frame):
        if self._pending is None or (frame.get("operation"), frame.get("token")) != self._pending[:2]:
            return
        operation, token, expected, expected_detection = self._pending
        self._pending = None
        self._timeout.stop()
        self._set_busy(False)
        result = int(frame["result"])
        if result != 0:
            reason = {1: "参数无效", 2: "设备不支持", 3: "缺少CH4～CH7", 4: "Flash写入或回读校验失败"}.get(result, f"错误码{result}")
            self.status_label.setText(f"操作失败：{reason}。输入值已保留，请核对后重试。")
            return
        if not frame["flash_valid"] or not frame["ram_valid"]:
            self.status_label.setText("MCU Flash中没有有效的低力表，RAM未标定；输入值已保留。")
            return
        try:
            points = validate_points(frame["points"])
            detection = frame.get("detection_counts")
            if detection is not None:
                detection = validate_detection_counts(detection, points)
        except (ValueError, TypeError):
            self.status_label.setText("回读参数无效，未确认保存成功。")
            return
        if expected_detection is not None and (detection is None or not math.isclose(detection, expected_detection, rel_tol=1e-6, abs_tol=1e-5)):
            self.status_label.setText("最低检测线回读与输入不一致，未确认保存成功。")
            return
        if expected is not None and any(not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-5) for a, b in zip(points, expected)):
            self.status_label.setText("回读参数与输入不一致，未确认保存成功。")
            return
        for editor, point in zip(self.inputs, points):
            editor.setText(f"{point:.9g}")
        self.detection_input.setText(f"{detection:.9g}" if detection is not None else "0")
        self._inputs_dirty = False
        self._update_input_validation()
        self.status_label.setText("已保存到MCU Flash并更新RAM，开机自动加载。" if expected is not None else "已读取MCU Flash并加载到RAM。")
        if detection is not None:
            self.status_label.setText(self.status_label.text() + f" MCU回读检测线：{detection:.9g} counts。")

    def refresh(self):
        self._try_auto_read()
        self._update_input_validation()
        sample = self._sample_provider() or {}
        signals = sample.get("signal", ())
        if sample.get("age_seconds", float("inf")) <= 0.5 and len(signals) >= 8:
            self.signal_label.setText(f"当前Signal和：{sum(signals[4:8]):.2f} counts")
        else:
            self.signal_label.setText("当前Signal和：-- counts")
        vector = sample.get("mcu_force_vector")
        age = sample.get("mcu_force_age_seconds")
        simulated = bool(vector and age is not None and age <= 0.5 and vector.get("low_force_simulated"))
        self.measured_force_label.setVisible(simulated)
        measured = vector.get("measured_normal_force") if simulated else None
        if measured is not None and math.isfinite(measured) and measured >= 0:
            self.measured_force_label.setText(f"实际换算（截断前）：{measured:.5f} N（{measured * 1000 / 9.80665:.2f} g）")
        else:
            self.measured_force_label.setText("实际换算（截断前）：-- N")
        for optimized, label, title in ((False, self.force_label, "原始"), (True, self.optimized_force_label, "优化")):
            display = low_force_display(vector, optimized=optimized) if age is not None and age <= 0.5 else None
            if display is None:
                if not vector:
                    reason = "未收到MCU力值数据"
                elif age is None:
                    reason = "MCU力值缺少时间信息"
                elif not age <= 0.5:
                    reason = "MCU力值数据已超时"
                else:
                    reason = "MCU未上报低力模式，请核对固件版本和设备"
                display = ("-- N", reason, "waiting")
            text = display[0] if display else "-- N"
            if display and display[2] == "valid":
                # 两路均从同帧MCU数值换算g，不从三位小数的N显示反算。
                force_n = float(vector["optimized_normal_force" if optimized else "normal_force"])
                if not optimized and not simulated and force_n < DETECTION_N:
                    force_n = 0.0
                text += f"（{force_n * 1000.0 / 9.80665:.2f} g）"
            if display[2] != "valid":
                text += f"（{display[1]}）"
            label.setText(f"{title}：{text}")
            label.setToolTip(f"{title}：{display[1]}")
            label.setVisible(self._display_mode == "both" or self._display_mode == ("optimized" if optimized else "original"))

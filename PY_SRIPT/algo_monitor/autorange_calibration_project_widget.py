import csv
import json
import statistics
import struct
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from qt_compat import QtCore, QtWidgets, Signal, Slot
from autorange_calibration_widget import (
    CAPTURE_FRAME_COUNT,
    MANUAL_MODE_FLAG,
    PENDING_FLAG,
    PROJECT_SCHEMA_VERSION,
    PROJECT_SUFFIX,
    REFERENCE_REF_CDAC,
    REF_CDAC_TIERS,
    VALID_RAW_LIMIT,
    analyze_calibration_quality,
    calculate_affine_coefficients,
)


AUTO_CAPTURE_SEQUENCE = (135, 90, 60, 40, 27, 18, 12, 18, 27, 40, 60, 90, 135)
AUTO_PRECHECK_SECONDS = 2.0
AUTO_PRECHECK_TIMEOUT_SECONDS = 15.0
AUTO_STEP_TIMEOUT_SECONDS = 5.0
AUTO_STEP_RETRY_COUNT = 1
AFFINE_ACTIVE_FLAG = 0x80


class AutorangeCalibrationWidget(QtWidgets.QWidget):
    """以单一项目文件管理Ref CDAC标定、质量分析和CSV快照。"""

    control_requested = Signal(bool, int)
    firmware_persist_requested = Signal(object)
    status_message = Signal(str)

    def __init__(self, sensor_sample_provider, parent=None):
        super().__init__(parent)
        self._sensor_sample_provider = sensor_sample_provider
        self._connected = False
        self._device_status = None
        self._syncing_mode_button = False
        self._last_sequence = None
        self._last_status_ref = None
        self._was_device_stable = False
        self._auto_capture = None
        self._firmware_transfer_busy = False
        self._external_busy = False
        self._capture_windows = {
            ref_cdac: deque(maxlen=CAPTURE_FRAME_COUNT)
            for ref_cdac in REF_CDAC_TIERS
        }
        self._project_id = uuid4().hex
        self._project_created_at = datetime.now().isoformat(timespec="seconds")
        self._project_path = self._new_default_project_path()
        self._points = [self._new_point("P0", "空载")]
        self._coefficients = []
        self._quality = analyze_calibration_quality(self._points)
        self._point_widgets = {}
        self._settings = QtCore.QSettings("Infineon", "AlgoMonitor")
        self._build_ui()
        self._restore_last_project()

        self._capture_timer = QtCore.QTimer(self)
        self._capture_timer.setInterval(10)
        self._capture_timer.timeout.connect(self._poll_live_sample)
        self._capture_timer.start()
        self._update_control_states()

    def _new_default_project_path(self):
        directory = Path(__file__).resolve().with_name("calibration_projects")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return directory / f"autorange_calibration_{timestamp}{PROJECT_SUFFIX}"

    def _new_point(self, name, note):
        return {
            "id": uuid4().hex,
            "name": name,
            "note": note,
            "samples": {},
            "history": [],
        }

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(7)

        project_row = QtWidgets.QHBoxLayout()
        self.new_project_button = QtWidgets.QPushButton("新建")
        self.new_project_button.clicked.connect(self._new_project)
        project_row.addWidget(self.new_project_button)
        self.open_project_button = QtWidgets.QPushButton("打开")
        self.open_project_button.clicked.connect(self._open_project)
        project_row.addWidget(self.open_project_button)
        self.save_project_button = QtWidgets.QPushButton("保存")
        self.save_project_button.clicked.connect(self._save_project)
        project_row.addWidget(self.save_project_button)
        self.save_as_button = QtWidgets.QPushButton("另存为")
        self.save_as_button.clicked.connect(self._save_project_as)
        project_row.addWidget(self.save_as_button)
        self.import_csv_button = QtWidgets.QPushButton("导入旧CSV")
        self.import_csv_button.clicked.connect(self._import_legacy_csv)
        project_row.addWidget(self.import_csv_button)
        self.export_button = QtWidgets.QPushButton("导出CSV快照")
        self.export_button.clicked.connect(self._export_csv_snapshot)
        project_row.addWidget(self.export_button)
        self.project_label = QtWidgets.QLabel()
        self.project_label.setToolTip("当前标定项目文件；每次采样后自动更新同一个文件")
        project_row.addWidget(self.project_label, 1)
        root.addLayout(project_row)

        mode_row = QtWidgets.QHBoxLayout()
        self.mode_button = QtWidgets.QPushButton("进入手动校准模式")
        self.mode_button.setCheckable(True)
        self.mode_button.toggled.connect(self._on_mode_toggled)
        mode_row.addWidget(self.mode_button)
        mode_row.addWidget(QtWidgets.QLabel("Ref CDAC"))
        self.ref_combo = QtWidgets.QComboBox()
        for ref_cdac in REF_CDAC_TIERS:
            self.ref_combo.addItem(str(ref_cdac), ref_cdac)
        mode_row.addWidget(self.ref_combo)
        self.apply_ref_button = QtWidgets.QPushButton("切换档位")
        self.apply_ref_button.clicked.connect(self._request_selected_ref)
        mode_row.addWidget(self.apply_ref_button)
        self.save_current_button = QtWidgets.QPushButton("保存当前档位")
        self.save_current_button.clicked.connect(self._save_active_ref)
        self.save_current_button.setToolTip(
            "保存固件实际Ref的16帧稳定窗口，并立即进行质量分析"
        )
        mode_row.addWidget(self.save_current_button)
        mode_row.addStretch(1)
        root.addLayout(mode_row)

        self.device_status_label = QtWidgets.QLabel("串口未连接")
        self.device_status_label.setWordWrap(True)
        root.addWidget(self.device_status_label)

        point_action_row = QtWidgets.QHBoxLayout()
        self.add_point_button = QtWidgets.QPushButton("新增压力点")
        self.add_point_button.clicked.connect(self._add_point)
        point_action_row.addWidget(self.add_point_button)
        self.delete_point_button = QtWidgets.QPushButton("删除当前点")
        self.delete_point_button.clicked.connect(self._delete_current_point)
        point_action_row.addWidget(self.delete_point_button)
        self.auto_capture_button = QtWidgets.QPushButton("自动采集当前压力点")
        self.auto_capture_button.setToolTip(
            "自动执行135→90→60→40→27→18→12→18→27→40→60→90→135双向采样"
        )
        self.auto_capture_button.clicked.connect(self._toggle_auto_capture)
        point_action_row.addWidget(self.auto_capture_button)
        self.calculate_button = QtWidgets.QPushButton("重新分析并计算28组参数")
        self.calculate_button.clicked.connect(self._calculate)
        point_action_row.addWidget(self.calculate_button)
        self.firmware_persist_button = QtWidgets.QPushButton("写入Flash并重启校验")
        self.firmware_persist_button.setToolTip(
            "将28组Q16参数写入固件Flash，软件复位后逐通道回读核对；暂不应用参数"
        )
        self.firmware_persist_button.clicked.connect(self._request_firmware_persist)
        point_action_row.addWidget(self.firmware_persist_button)
        point_action_row.addStretch(1)
        root.addLayout(point_action_row)

        self.workspace_tabs = QtWidgets.QTabWidget()
        self.workspace_tabs.setUsesScrollButtons(True)
        self.workspace_tabs.currentChanged.connect(self._on_workspace_tab_changed)
        root.addWidget(self.workspace_tabs, 1)

        self.global_status_label = QtWidgets.QLabel()
        self.global_status_label.setWordWrap(True)
        root.addWidget(self.global_status_label)
        self._refresh_workspace_tabs(selected_point_id=self._points[0]["id"])
        self._update_project_label()

    def _create_sample_table(self, point, point_result):
        table = QtWidgets.QTableWidget(len(REF_CDAC_TIERS), 8)
        table.setHorizontalHeaderLabels([
            "Ref", "CH4 HW Raw", "CH5 HW Raw", "CH6 HW Raw", "CH7 HW Raw",
            "采样", "质量", "原因",
        ])
        table.verticalHeader().setVisible(False)
        no_edit = getattr(
            QtWidgets.QAbstractItemView,
            "NoEditTriggers",
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        table.setEditTriggers(no_edit)
        fixed_resize = getattr(
            QtWidgets.QHeaderView,
            "Fixed",
            QtWidgets.QHeaderView.ResizeMode.Fixed,
        )
        stretch_resize = getattr(
            QtWidgets.QHeaderView,
            "Stretch",
            QtWidgets.QHeaderView.ResizeMode.Stretch,
        )
        header = table.horizontalHeader()
        for column in range(7):
            header.setSectionResizeMode(column, fixed_resize)
        header.setSectionResizeMode(7, stretch_resize)
        table.setColumnWidth(0, 48)
        for column in range(1, 5):
            table.setColumnWidth(column, 94)
        table.setColumnWidth(5, 62)
        table.setColumnWidth(6, 94)
        table.verticalHeader().setDefaultSectionSize(34)
        scroll_bar_off = getattr(
            QtCore.Qt,
            "ScrollBarAlwaysOff",
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff,
        )
        table.setVerticalScrollBarPolicy(scroll_bar_off)

        for row, ref_cdac in enumerate(REF_CDAC_TIERS):
            sample = point["samples"].get(ref_cdac)
            sample_result = point_result["samples"].get(ref_cdac, {})
            status = sample_result.get("status", "采集中")
            reasons = sample_result.get("reasons", [])
            values = [str(ref_cdac)]
            if sample is None:
                values.extend(["--", "--", "--", "--", "--"])
            else:
                values.extend(f"{value:.1f}" for value in sample["raw"])
                values.append(f"{sample['sample_count']}帧")
            values.extend([self._status_text(status), "；".join(reasons) or "--"])
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if reasons:
                    item.setToolTip("；".join(reasons))
                table.setItem(row, column, item)

        table_height = (
            table.horizontalHeader().sizeHint().height()
            + len(REF_CDAC_TIERS) * table.verticalHeader().defaultSectionSize()
            + table.horizontalScrollBar().sizeHint().height()
            + table.frameWidth() * 2
            + 4
        )
        table.setFixedHeight(table_height)
        return table

    def _create_point_page(self, point):
        point_result = self._quality["points"][point["id"]]
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(4, 6, 4, 4)

        note_row = QtWidgets.QHBoxLayout()
        note_row.addWidget(QtWidgets.QLabel(f"压力点 {point['name']}"))
        note_row.addWidget(QtWidgets.QLabel("备注"))
        note_edit = QtWidgets.QLineEdit(point["note"])
        note_edit.setReadOnly(point["name"] == "P0")
        note_edit.editingFinished.connect(
            lambda point_id=point["id"]: self._save_point_note(point_id)
        )
        note_row.addWidget(note_edit, 1)
        history_label = QtWidgets.QLabel(f"历史记录 {len(point.get('history', []))}次")
        note_row.addWidget(history_label)
        layout.addLayout(note_row)

        table = self._create_sample_table(point, point_result)
        layout.addWidget(table)

        result_row = QtWidgets.QHBoxLayout()
        reasons = "；".join(point_result["reasons"]) or "当前已采样数据未发现异常"
        result_label = QtWidgets.QLabel(
            f"{self._status_text(point_result['status'])}：{reasons}"
        )
        result_label.setWordWrap(True)
        result_row.addWidget(result_label, 1)
        retest_button = QtWidgets.QPushButton("重测建议项")
        retest_button.setEnabled(point_result["status"] == "建议重测")
        retest_button.clicked.connect(
            lambda _checked=False, point_id=point["id"]: self._remeasure_recommended(point_id)
        )
        result_row.addWidget(retest_button)
        layout.addLayout(result_row)

        self._point_widgets[point["id"]] = {
            "page": page,
            "note_edit": note_edit,
            "table": table,
            "result_label": result_label,
            "retest_button": retest_button,
        }
        return page

    def _create_analysis_page(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        self.analysis_tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.analysis_tabs)

        recommendation_page = QtWidgets.QWidget()
        recommendation_layout = QtWidgets.QVBoxLayout(recommendation_page)
        recommendation_layout.setContentsMargins(0, 0, 0, 0)
        self.recommendation_table = QtWidgets.QTableWidget(0, 5)
        self.recommendation_table.setHorizontalHeaderLabels([
            "压力点", "状态", "异常分数", "建议动作", "原因",
        ])
        self.recommendation_table.verticalHeader().setVisible(False)
        self.recommendation_table.horizontalHeader().setStretchLastSection(True)
        recommendation_layout.addWidget(self.recommendation_table)
        self.analysis_tabs.addTab(recommendation_page, "重测建议")

        coefficient_page = QtWidgets.QWidget()
        coefficient_layout = QtWidgets.QVBoxLayout(coefficient_page)
        coefficient_layout.setContentsMargins(0, 0, 0, 0)
        self.coefficient_table = QtWidgets.QTableWidget(0, 11)
        self.coefficient_table.setHorizontalHeaderLabels([
            "CH", "Ref", "a", "b", "Gain Q16", "Offset", "RMSE",
            "最大误差", "R²", "样本", "拟合路径",
        ])
        self.coefficient_table.verticalHeader().setVisible(False)
        self.coefficient_table.horizontalHeader().setStretchLastSection(True)
        coefficient_layout.addWidget(self.coefficient_table)
        self.analysis_tabs.addTab(coefficient_page, "仿射参数")
        self._show_analysis()
        return page

    def _refresh_workspace_tabs(self, selected_point_id=None, select_analysis=False):
        if selected_point_id is None:
            current = self._current_point()
            selected_point_id = None if current is None else current["id"]
        self._quality = analyze_calibration_quality(self._points)
        self._point_widgets = {}
        self.workspace_tabs.blockSignals(True)
        while self.workspace_tabs.count():
            widget = self.workspace_tabs.widget(0)
            self.workspace_tabs.removeTab(0)
            widget.deleteLater()
        selected_index = 0
        for point_index, point in enumerate(self._points):
            point_result = self._quality["points"][point["id"]]
            title = f"{point['name']} {self._tab_symbol(point_result['status'])}"
            self.workspace_tabs.addTab(self._create_point_page(point), title)
            self.workspace_tabs.setTabToolTip(point_index, "；".join(point_result["reasons"]))
            if point["id"] == selected_point_id:
                selected_index = point_index
        self.analysis_page = self._create_analysis_page()
        self.workspace_tabs.addTab(self.analysis_page, "综合分析")
        if select_analysis:
            selected_index = len(self._points)
        self.workspace_tabs.setCurrentIndex(selected_index)
        self.workspace_tabs.blockSignals(False)
        self._on_workspace_tab_changed(selected_index)
        self._show_global_summary()

    def _status_text(self, status):
        return {
            "合格": "✓ 合格",
            "采集中": "… 采集中",
            "警告": "⚠ 警告",
            "建议重测": "✕ 建议重测",
        }.get(status, status)

    def _tab_symbol(self, status):
        return {
            "合格": "✓",
            "采集中": "…",
            "警告": "⚠",
            "建议重测": "✕",
        }.get(status, "")

    def _show_analysis(self):
        if not hasattr(self, "recommendation_table"):
            return
        recommendations = self._quality.get("recommendations", [])
        self.recommendation_table.setRowCount(len(recommendations))
        for row, item in enumerate(recommendations):
            values = (
                item["point"],
                self._status_text(item["status"]),
                f"{item['score']:.2f}",
                item["action"],
                item["reason"],
            )
            for column, value in enumerate(values):
                self.recommendation_table.setItem(
                    row, column, QtWidgets.QTableWidgetItem(str(value))
                )
        self.recommendation_table.resizeColumnsToContents()

        no_edit = getattr(
            QtWidgets.QAbstractItemView,
            "NoEditTriggers",
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        self.recommendation_table.setEditTriggers(no_edit)
        self.coefficient_table.setEditTriggers(no_edit)
        self.coefficient_table.setRowCount(len(self._coefficients))
        for row, coefficient in enumerate(self._coefficients):
            values = (
                f"CH{coefficient['channel']}",
                str(coefficient["ref_cdac"]),
                f"{coefficient['gain']:.9f}",
                f"{coefficient['offset']:.3f}",
                str(coefficient["gain_q16"]),
                str(coefficient["offset_int"]),
                "--" if coefficient["rmse"] is None else f"{coefficient['rmse']:.3f}",
                "--" if coefficient["max_error"] is None else f"{coefficient['max_error']:.3f}",
                "--" if coefficient["r_squared"] is None else f"{coefficient['r_squared']:.6f}",
                f"直接{coefficient['direct_sample_count']} / 相邻≥{coefficient['edge_min_sample_count']}",
                coefficient["fit_path"],
            )
            for column, value in enumerate(values):
                self.coefficient_table.setItem(
                    row, column, QtWidgets.QTableWidgetItem(value)
                )
        if self._coefficients:
            self.coefficient_table.resizeColumnsToContents()

    def _show_global_summary(self):
        failed = sum(
            result["status"] == "建议重测"
            for result in self._quality["points"].values()
        )
        warnings = sum(
            result["status"] == "警告"
            for result in self._quality["points"].values()
        )
        coverage_count = len(self._quality.get("coverage", []))
        self.global_status_label.setText(
            f"自动质检：建议重测{failed}个压力点，警告{warnings}个，"
            f"有效覆盖不足{coverage_count}项；拟合有效Raw上限={VALID_RAW_LIMIT}。"
        )

    def _current_point(self):
        index = self.workspace_tabs.currentIndex()
        if 0 <= index < len(self._points):
            return self._points[index]
        return None

    @Slot(int)
    def _on_workspace_tab_changed(self, _index):
        for window in self._capture_windows.values():
            window.clear()
        self._update_control_states()

    def set_connected(self, connected):
        self._connected = bool(connected)
        if not connected:
            if self._auto_capture is not None:
                self._abort_auto_capture("串口断开，自动采集已中止", request_safe_ref=False)
            self._device_status = None
            self._sync_mode_button(False)
            self.device_status_label.setText("串口未连接")
        self._update_control_states()

    def has_active_operation(self):
        return self._auto_capture is not None or self._firmware_transfer_busy

    def set_external_busy(self, busy):
        self._external_busy = bool(busy)
        self._update_control_states()

    def on_device_status(self, frame):
        self._device_status = dict(frame)
        manual_mode = bool(int(frame["flags"]) & MANUAL_MODE_FLAG)
        self._sync_mode_button(manual_mode)
        actual_ref = int(frame["actual_ref_cdac"])
        requested_ref = int(frame.get("requested_ref_cdac", actual_ref))
        stable = self._is_device_stable()
        if actual_ref != self._last_status_ref or (stable and not self._was_device_stable):
            if actual_ref in self._capture_windows:
                self._capture_windows[actual_ref].clear()
        self._last_status_ref = actual_ref
        self._was_device_stable = stable
        self.device_status_label.setText(
            f"固件模式={'手动校准' if manual_mode else '自动量程'}；"
            f"请求Ref={requested_ref}，实际Ref={actual_ref}；"
            f"Norm={'仿射已应用' if int(frame['flags']) & AFFINE_ACTIVE_FLAG else '理论比例回退'}；"
            f"{'已稳定，可以保存' if stable else '切档或等待稳定中'}"
        )
        self._update_control_states()

    def control_command_failed(self, message):
        if self._auto_capture is not None:
            self._abort_auto_capture(
                f"档位控制失败：{message}", request_safe_ref=False
            )
        self.global_status_label.setText(message)
        if self._device_status is not None:
            manual_mode = bool(int(self._device_status["flags"]) & MANUAL_MODE_FLAG)
            self._sync_mode_button(manual_mode)
        self._update_control_states()

    def _sync_mode_button(self, checked):
        self._syncing_mode_button = True
        try:
            self.mode_button.setChecked(bool(checked))
            self.mode_button.setText(
                "退出手动校准并恢复自动量程" if checked else "进入手动校准模式"
            )
        finally:
            self._syncing_mode_button = False

    @Slot(bool)
    def _on_mode_toggled(self, checked):
        if self._syncing_mode_button:
            return
        if not self._connected:
            self._sync_mode_button(False)
            self.status_message.emit("无法进入校准模式：串口未连接")
            return
        ref_cdac = int(self.ref_combo.currentData()) if checked else 0
        if checked:
            self._capture_windows[ref_cdac].clear()
        self.mode_button.setText(
            "退出手动校准并恢复自动量程" if checked else "进入手动校准模式"
        )
        self.control_requested.emit(bool(checked), ref_cdac)
        self._update_control_states()

    @Slot()
    def _request_selected_ref(self):
        if not self._connected or not self.mode_button.isChecked():
            self.status_message.emit("请先连接设备并进入手动校准模式")
            return
        ref_cdac = int(self.ref_combo.currentData())
        self._capture_windows[ref_cdac].clear()
        self.control_requested.emit(True, ref_cdac)

    def _is_device_stable(self):
        if self._device_status is None:
            return False
        flags = int(self._device_status["flags"])
        return (
            bool(flags & MANUAL_MODE_FLAG)
            and not bool(flags & PENDING_FLAG)
            and int(self._device_status["blank_frames"]) == 0
            and int(self._device_status["actual_ref_cdac"])
            == int(self._device_status.get("requested_ref_cdac", -1))
        )

    @Slot()
    def _poll_live_sample(self):
        self._check_auto_capture_timeout()
        sample = self._sensor_sample_provider()
        if sample is None or not sample.get("has_raw") or sample.get("age_seconds", 1.0) > 0.5:
            self._update_control_states()
            return
        sequence = int(sample["sequence"])
        if sequence == self._last_sequence:
            return
        self._last_sequence = sequence
        if not self._is_device_stable():
            self._update_control_states()
            return
        actual_ref = int(self._device_status["actual_ref_cdac"])
        hardware_raw = tuple(int(value) for value in sample["hardware_raw"][4:8])
        if len(hardware_raw) != 4:
            return
        self._capture_windows[actual_ref].append(hardware_raw)
        if self._auto_capture is not None:
            self._process_auto_capture_sample(actual_ref, hardware_raw)
        else:
            self.device_status_label.setText(
                f"固件模式=手动校准；实际Ref={actual_ref}；"
                f"实时CH4～CH7={hardware_raw}；"
                f"有效窗口{len(self._capture_windows[actual_ref])}/{CAPTURE_FRAME_COUNT}帧"
            )
        self._update_control_states()

    def _update_control_states(self):
        auto_running = self._auto_capture is not None
        controls_busy = auto_running or self._firmware_transfer_busy or self._external_busy
        manual_mode = self.mode_button.isChecked()
        stable = self._is_device_stable()
        actual_ref = None
        if self._device_status is not None:
            actual_ref = int(self._device_status["actual_ref_cdac"])
        self.mode_button.setEnabled(self._connected and not controls_busy)
        self.ref_combo.setEnabled(self._connected and manual_mode and not controls_busy)
        self.apply_ref_button.setEnabled(self._connected and manual_mode and not controls_busy)
        self.save_current_button.setEnabled(
            not controls_busy
            and
            self._current_point() is not None
            and self._connected
            and stable
            and actual_ref in self._capture_windows
            and len(self._capture_windows[actual_ref]) >= CAPTURE_FRAME_COUNT
        )
        current = self._current_point()
        self.delete_point_button.setEnabled(
            not controls_busy and current is not None and current["name"] != "P0"
        )
        self.auto_capture_button.setEnabled(
            not self._firmware_transfer_busy
            and not self._external_busy
            and (auto_running or (self._connected and current is not None))
        )
        self.auto_capture_button.setText(
            "停止自动采集" if auto_running else "自动采集当前压力点"
        )
        for button in (
            self.new_project_button,
            self.open_project_button,
            self.save_project_button,
            self.save_as_button,
            self.import_csv_button,
            self.export_button,
            self.add_point_button,
            self.calculate_button,
        ):
            button.setEnabled(not controls_busy)
        self.firmware_persist_button.setEnabled(
            self._connected
            and not controls_busy
            and len(self._coefficients) == 28
            and not self._quality.get("recommendations")
        )
        self.workspace_tabs.tabBar().setEnabled(not controls_busy)

    @Slot()
    def _toggle_auto_capture(self):
        if self._auto_capture is not None:
            self._abort_auto_capture("用户停止了自动采集")
            return
        self._start_auto_capture()

    def _start_auto_capture(self):
        point = self._current_point()
        if not self._connected or point is None:
            self.status_message.emit("请先连接设备并选择压力点")
            return
        if point["samples"]:
            answer = QtWidgets.QMessageBox.question(
                self,
                "自动重测当前压力点",
                f"{point['name']}已有采样。自动采集全部成功后，旧数据将移入历史记录，是否继续？",
            )
            if not self._is_yes(answer):
                return

        now = time.monotonic()
        self._auto_capture = {
            "point_id": point["id"],
            "point_name": point["name"],
            "sequence": AUTO_CAPTURE_SEQUENCE,
            "step_index": -1,
            "target_ref": 135,
            "phase": "precheck",
            "started_at": datetime.now().isoformat(timespec="milliseconds"),
            "started_monotonic": now,
            "step_started_monotonic": now,
            "precheck_frames": [],
            "captures": {ref_cdac: [] for ref_cdac in REF_CDAC_TIERS},
            "step_retries": {},
            "precheck_restarts": 0,
        }
        self._sync_mode_button(True)
        self._set_ref_combo_value(135)
        self._capture_windows[135].clear()
        self.control_requested.emit(True, 135)
        self.global_status_label.setText(
            f"{point['name']}自动采集：先在Ref135检查2秒慢趋势"
        )
        self._update_control_states()

    def _set_ref_combo_value(self, ref_cdac):
        index = self.ref_combo.findData(ref_cdac)
        if index >= 0:
            self.ref_combo.setCurrentIndex(index)

    def _process_auto_capture_sample(self, actual_ref, hardware_raw):
        state = self._auto_capture
        if state is None or not self._is_device_stable():
            return
        if actual_ref != int(state["target_ref"]):
            return

        if state["phase"] == "precheck":
            self._process_auto_precheck_sample(hardware_raw)
            return
        if state["phase"] != "capture":
            return

        step_index = int(state["step_index"])
        frame_count = len(self._capture_windows[actual_ref])
        self.device_status_label.setText(
            f"{state['point_name']}自动采集 {step_index + 1}/{len(state['sequence'])}；"
            f"Ref={actual_ref}；稳定窗口{frame_count}/{CAPTURE_FRAME_COUNT}帧"
        )
        if frame_count < CAPTURE_FRAME_COUNT:
            return

        frames = list(self._capture_windows[actual_ref])
        capture = self._make_capture_record(frames)
        noisy_channels = []
        for channel_index, channel in enumerate(range(4, 8)):
            noise_limit = max(5.0, capture["raw"][channel_index] * 0.001)
            if capture["stddev"][channel_index] > noise_limit:
                noisy_channels.append(channel)
        if noisy_channels:
            retry_count = state["step_retries"].get(step_index, 0)
            if retry_count < AUTO_STEP_RETRY_COUNT:
                state["step_retries"][step_index] = retry_count + 1
                self.global_status_label.setText(
                    f"Ref{actual_ref}的CH{','.join(str(value) for value in noisy_channels)}"
                    "短时噪声超限，正在自动重试"
                )
                self._request_auto_step(step_index)
                return
            self._abort_auto_capture(
                f"Ref{actual_ref}短时噪声连续超限，原压力点数据保持不变"
            )
            return

        state["captures"][actual_ref].append(capture)
        next_step = step_index + 1
        if next_step >= len(state["sequence"]):
            self._complete_auto_capture()
        else:
            self._request_auto_step(next_step)

    def _process_auto_precheck_sample(self, hardware_raw):
        state = self._auto_capture
        if state is None:
            return
        now = time.monotonic()
        state["precheck_frames"].append((now, hardware_raw))
        frames = state["precheck_frames"]
        elapsed = frames[-1][0] - frames[0][0]
        self.device_status_label.setText(
            f"{state['point_name']}自动采集：Ref135慢趋势检查"
            f" {min(elapsed, AUTO_PRECHECK_SECONDS):.1f}/{AUTO_PRECHECK_SECONDS:.1f}秒"
        )
        if elapsed < AUTO_PRECHECK_SECONDS:
            return

        group_size = max(3, len(frames) // 4)
        start_group = frames[:group_size]
        end_group = frames[-group_size:]
        start_raw = tuple(
            statistics.median(item[1][channel_index] for item in start_group)
            for channel_index in range(4)
        )
        end_raw = tuple(
            statistics.median(item[1][channel_index] for item in end_group)
            for channel_index in range(4)
        )
        unstable = []
        for channel_index, channel in enumerate(range(4, 8)):
            trend_limit = max(20.0, abs(end_raw[channel_index]) * 0.002)
            change = end_raw[channel_index] - start_raw[channel_index]
            if abs(change) > trend_limit:
                unstable.append((channel, change, trend_limit))
        if unstable:
            state["precheck_restarts"] += 1
            state["precheck_frames"] = []
            details = "、".join(
                f"CH{channel}{change:+.0f}"
                for channel, change, _limit in unstable
            )
            self.global_status_label.setText(
                f"压力仍在慢漂移（{details}），重新观察2秒"
            )
            return

        state["precheck_summary"] = {
            "duration_seconds": elapsed,
            "start_raw": start_raw,
            "end_raw": end_raw,
            "restart_count": state["precheck_restarts"],
        }
        self._request_auto_step(0)

    def _request_auto_step(self, step_index):
        state = self._auto_capture
        if state is None:
            return
        target_ref = int(state["sequence"][step_index])
        state["step_index"] = step_index
        state["target_ref"] = target_ref
        state["phase"] = "capture"
        state["step_started_monotonic"] = time.monotonic()
        self._capture_windows[target_ref].clear()
        self._set_ref_combo_value(target_ref)
        self.control_requested.emit(True, target_ref)
        self.global_status_label.setText(
            f"{state['point_name']}自动采集 {step_index + 1}/{len(state['sequence'])}："
            f"正在切换到Ref{target_ref}"
        )

    def _make_capture_record(self, frames):
        return {
            "raw": tuple(
                float(statistics.median(frame[channel_index] for frame in frames))
                for channel_index in range(4)
            ),
            "stddev": tuple(
                float(statistics.pstdev(frame[channel_index] for frame in frames))
                for channel_index in range(4)
            ),
            "minimum": tuple(
                min(frame[channel_index] for frame in frames)
                for channel_index in range(4)
            ),
            "maximum": tuple(
                max(frame[channel_index] for frame in frames)
                for channel_index in range(4)
            ),
            "sample_count": len(frames),
            "saved_at": datetime.now().isoformat(timespec="milliseconds"),
        }

    def _check_auto_capture_timeout(self):
        state = self._auto_capture
        if state is None:
            return
        now = time.monotonic()
        if state["phase"] == "precheck":
            if now - state["started_monotonic"] > AUTO_PRECHECK_TIMEOUT_SECONDS:
                self._abort_auto_capture(
                    "15秒内未达到慢趋势稳定条件，请保持压力稳定后重试"
                )
            return
        if now - state["step_started_monotonic"] <= AUTO_STEP_TIMEOUT_SECONDS:
            return
        step_index = int(state["step_index"])
        retry_count = state["step_retries"].get(step_index, 0)
        if retry_count < AUTO_STEP_RETRY_COUNT:
            state["step_retries"][step_index] = retry_count + 1
            self._request_auto_step(step_index)
            return
        self._abort_auto_capture(
            f"Ref{state['target_ref']}在{AUTO_STEP_TIMEOUT_SECONDS:.0f}秒内未完成稳定采样"
        )

    def _combine_auto_captures(self, state):
        central_capture = state["captures"][12][0]
        effective_time = central_capture["saved_at"]
        samples = {}
        for ref_cdac in REF_CDAC_TIERS:
            captures = state["captures"][ref_cdac]
            if not captures:
                raise ValueError(f"Ref{ref_cdac}没有自动采样数据")
            first = captures[0]
            second = captures[-1]
            raw = tuple(
                (first["raw"][index] + second["raw"][index]) / 2.0
                for index in range(4)
            )
            sweep_delta = tuple(
                second["raw"][index] - first["raw"][index]
                for index in range(4)
            )
            samples[ref_cdac] = {
                "raw": raw,
                "stddev": tuple(
                    max(first["stddev"][index], second["stddev"][index])
                    for index in range(4)
                ),
                "minimum": tuple(
                    min(first["minimum"][index], second["minimum"][index])
                    for index in range(4)
                ),
                "maximum": tuple(
                    max(first["maximum"][index], second["maximum"][index])
                    for index in range(4)
                ),
                "sample_count": sum(item["sample_count"] for item in captures),
                "saved_at": effective_time,
                "capture_mode": "auto_bidirectional",
                "forward_raw": first["raw"],
                "reverse_raw": second["raw"],
                "sweep_delta": sweep_delta,
                "capture_times": [item["saved_at"] for item in captures],
            }
        return samples

    def _complete_auto_capture(self):
        state = self._auto_capture
        if state is None:
            return
        point = self._point_by_id(state["point_id"])
        if point is None:
            self._abort_auto_capture("当前压力点已不存在")
            return
        try:
            new_samples = self._combine_auto_captures(state)
        except ValueError as exc:
            self._abort_auto_capture(str(exc))
            return

        finished_at = datetime.now().isoformat(timespec="milliseconds")
        if point["samples"]:
            point["history"].append({
                "archived_at": finished_at,
                "reason": "双向自动采集成功后替换整组数据",
                "samples": {str(ref): sample for ref, sample in point["samples"].items()},
            })
        point["samples"] = new_samples
        point["last_auto_capture"] = {
            "started_at": state["started_at"],
            "finished_at": finished_at,
            "sequence": list(state["sequence"]),
            "precheck": state.get("precheck_summary", {}),
            "step_retries": state["step_retries"],
        }
        point_id = point["id"]
        point_name = point["name"]
        self._auto_capture = None
        self._coefficients = []
        try:
            self._coefficients = calculate_affine_coefficients(self._points)
        except ValueError:
            pass
        self._refresh_workspace_tabs(selected_point_id=point_id)
        self._autosave_project()
        point_result = self._quality["points"][point_id]
        usable_pair_count = self._count_usable_adjacent_pairs(point)
        if point_result["status"] == "建议重测":
            details = "；".join(point_result["reasons"])
            message = f"{point_name}自动采集完成，但不满足使用条件：{details}"
            QtWidgets.QMessageBox.warning(self, "自动采集完成：建议重测", message)
        elif usable_pair_count == 0:
            message = (
                f"{point_name}自动采集完成，但所有相邻档位均缺少有效配对；"
                "该压力已超出标定范围，请降低压力后重测。"
            )
            QtWidgets.QMessageBox.warning(self, "自动采集完成：不可用于拟合", message)
        elif point_result.get("global_fit_warning"):
            details = "；".join(point_result["reasons"])
            message = (
                f"{point_name}双向采集自检通过，本点无需反复重测；"
                f"但与现有全局拟合不一致：{details}。"
                "请优先自动重测邻近的旧手动压力点，或调整拟合模型。"
            )
            QtWidgets.QMessageBox.information(
                self, "自动采集完成：本点可用，全局一致性警告", message
            )
        else:
            excluded_count = sum(
                not result["usable_channels"]
                for result in point_result["samples"].values()
            )
            suffix = (
                "；部分饱和数据已自动排除拟合"
                if point_result["status"] == "警告" or excluded_count
                else ""
            )
            message = f"{point_name}自动采集完成，满足当前压力点使用条件{suffix}。"
            QtWidgets.QMessageBox.information(self, "自动采集完成：可以使用", message)
        self.global_status_label.setText(message)
        self.status_message.emit(message)
        self._update_control_states()

    def _count_usable_adjacent_pairs(self, point):
        edges = ((12, 18), (18, 27), (27, 40), (40, 60), (90, 60), (135, 90))
        pair_count = 0
        for source_ref, target_ref in edges:
            source = point["samples"].get(source_ref)
            target = point["samples"].get(target_ref)
            if source is None or target is None:
                continue
            for channel_index in range(4):
                if (
                    float(source["raw"][channel_index]) < VALID_RAW_LIMIT
                    and float(target["raw"][channel_index]) < VALID_RAW_LIMIT
                ):
                    pair_count += 1
        return pair_count

    def _abort_auto_capture(self, reason, request_safe_ref=True):
        state = self._auto_capture
        if state is None:
            return
        point_name = state["point_name"]
        self._auto_capture = None
        if request_safe_ref and self._connected:
            self._set_ref_combo_value(135)
            self.control_requested.emit(True, 135)
        message = f"{point_name}自动采集已中止：{reason}；原数据未修改。"
        self.global_status_label.setText(message)
        self.status_message.emit(message)
        self._update_control_states()

    def _next_point_name(self):
        numbers = []
        for point in self._points:
            name = point["name"]
            if name.startswith("P") and name[1:].isdigit():
                numbers.append(int(name[1:]))
        return f"P{max(numbers, default=0) + 1}"

    @Slot()
    def _add_point(self):
        name = self._next_point_name()
        point = self._new_point(name, f"压力点 {name}")
        self._points.append(point)
        self._coefficients = []
        self._refresh_workspace_tabs(selected_point_id=point["id"])
        self._autosave_project()

    @Slot()
    def _delete_current_point(self):
        point = self._current_point()
        if point is None or point["name"] == "P0":
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "删除压力点",
            f"确认删除{point['name']}及其全部采样吗？其他压力点不会重新编号。",
        )
        if not self._is_yes(answer):
            return
        index = self._points.index(point)
        del self._points[index]
        self._coefficients = []
        selected = self._points[max(0, index - 1)]["id"]
        self._refresh_workspace_tabs(selected_point_id=selected)
        self._autosave_project()

    def _save_point_note(self, point_id):
        widgets = self._point_widgets.get(point_id)
        point = self._point_by_id(point_id)
        if widgets is None or point is None:
            return
        if point["name"] == "P0":
            point["note"] = "空载"
            widgets["note_edit"].setText("空载")
            return
        point["note"] = widgets["note_edit"].text().strip() or f"压力点 {point['name']}"
        self._autosave_project()

    def _point_by_id(self, point_id):
        return next((point for point in self._points if point["id"] == point_id), None)

    def _save_current_raw(self, ref_cdac):
        point = self._current_point()
        if point is None:
            self.status_message.emit("请先选择压力点Tab")
            return
        if not self._is_device_stable():
            self.status_message.emit("当前Ref尚未稳定，不能保存")
            return
        actual_ref = int(self._device_status["actual_ref_cdac"])
        if actual_ref != ref_cdac:
            self.status_message.emit(f"当前实际Ref为{actual_ref}，不能保存到Ref{ref_cdac}")
            return
        frames = list(self._capture_windows[ref_cdac])
        if len(frames) < CAPTURE_FRAME_COUNT:
            self.status_message.emit(
                f"稳定样本只有{len(frames)}帧，请等待{CAPTURE_FRAME_COUNT}帧后再保存"
            )
            return

        raw_values = tuple(
            float(statistics.median(frame[channel_index] for frame in frames))
            for channel_index in range(4)
        )
        stddev_values = tuple(
            float(statistics.pstdev(frame[channel_index] for frame in frames))
            for channel_index in range(4)
        )
        previous = point["samples"].get(ref_cdac)
        if previous is not None:
            point["history"].append({
                "archived_at": datetime.now().isoformat(timespec="milliseconds"),
                "reason": f"覆盖重测Ref{ref_cdac}",
                "samples": {str(ref_cdac): previous},
            })
        point["samples"][ref_cdac] = {
            "raw": raw_values,
            "stddev": stddev_values,
            "minimum": tuple(min(frame[index] for frame in frames) for index in range(4)),
            "maximum": tuple(max(frame[index] for frame in frames) for index in range(4)),
            "sample_count": len(frames),
            "saved_at": datetime.now().isoformat(timespec="milliseconds"),
        }
        self._coefficients = []
        point_id = point["id"]
        self._refresh_workspace_tabs(selected_point_id=point_id)
        self._autosave_project()

        point_result = self._quality["points"][point_id]
        result = point_result["samples"][ref_cdac]
        reasons = "；".join(result["reasons"]) or "短时稳定性和量程检查通过"
        message = (
            f"已保存{point['name']}/Ref{ref_cdac}："
            f"本档{self._status_text(result['status'])}，{reasons}；"
            f"压力点{self._status_text(point_result['status'])}"
        )
        if point_result["status"] == "建议重测" and point_result["reasons"]:
            message += "，" + "；".join(point_result["reasons"])
        self.status_message.emit(message)
        self.global_status_label.setText(message)
        if point_result["status"] == "建议重测":
            QtWidgets.QMessageBox.warning(self, "压力点建议重测", message)

    @Slot()
    def _save_active_ref(self):
        if self._device_status is None:
            self.status_message.emit("尚未收到固件量程状态，不能保存")
            return
        self._save_current_raw(int(self._device_status["actual_ref_cdac"]))

    @Slot()
    def _remeasure_recommended(self, point_id):
        point = self._point_by_id(point_id)
        result = self._quality["points"].get(point_id)
        if point is None or result is None:
            return
        refs = list(point["samples"])
        action = "整个压力点"
        if not result["remeasure_whole_point"] and result["recommended_refs"]:
            refs = [ref for ref in result["recommended_refs"] if ref in point["samples"]]
            action = "Ref " + "、".join(str(value) for value in refs)
        if not refs:
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "准备重测",
            f"将{point['name']}的{action}移入历史记录并清空当前值，是否继续？",
        )
        if not self._is_yes(answer):
            return
        archived_samples = {str(ref): point["samples"][ref] for ref in refs}
        point["history"].append({
            "archived_at": datetime.now().isoformat(timespec="milliseconds"),
            "reason": "自动质检建议重测：" + "；".join(result["reasons"]),
            "samples": archived_samples,
        })
        for ref_cdac in refs:
            del point["samples"][ref_cdac]
        self._coefficients = []
        self._refresh_workspace_tabs(selected_point_id=point_id)
        self._autosave_project()

    @Slot()
    def _calculate(self):
        try:
            self._coefficients = calculate_affine_coefficients(self._points)
        except ValueError as exc:
            self._coefficients = []
            self._refresh_workspace_tabs(select_analysis=True)
            self.global_status_label.setText(f"拟合失败：{exc}")
            self.status_message.emit(f"仿射校准拟合失败：{exc}")
            return False
        self._refresh_workspace_tabs(select_analysis=True)
        self.analysis_tabs.setCurrentIndex(1)
        self._autosave_project()
        message = (
            f"已计算{len(self._coefficients)}组参数；Raw≥{VALID_RAW_LIMIT}自动排除。"
            f"综合分析中有{len(self._quality['recommendations'])}条建议。"
        )
        self.global_status_label.setText(message)
        self.status_message.emit(message)
        return True

    def _firmware_coefficient_channels(self):
        coefficient_map = {
            (int(item["channel"]), int(item["ref_cdac"])): (
                int(item["gain_q16"]),
                int(item["offset_int"]),
            )
            for item in self._coefficients
        }
        channels = {}
        for channel in range(4, 8):
            values = []
            for ref_cdac in REF_CDAC_TIERS:
                key = (channel, ref_cdac)
                if key not in coefficient_map:
                    raise ValueError(f"缺少CH{channel}/Ref{ref_cdac}仿射参数")
                values.append(coefficient_map[key])
            channels[channel] = tuple(values)
        return channels

    @staticmethod
    def _firmware_parameter_id(channels):
        checksum = 2166136261
        for channel in range(4, 8):
            checksum ^= channel
            checksum = (checksum * 16777619) & 0xFFFFFFFF
            for ref_cdac, (gain_q16, offset) in zip(
                REF_CDAC_TIERS, channels[channel]
            ):
                checksum ^= int(ref_cdac)
                checksum = (checksum * 16777619) & 0xFFFFFFFF
                for value in struct.pack("<ii", int(gain_q16), int(offset)):
                    checksum ^= value
                    checksum = (checksum * 16777619) & 0xFFFFFFFF
        return checksum

    @Slot()
    def _request_firmware_persist(self):
        if not self._connected:
            self.status_message.emit("无法写入仿射参数：串口未连接")
            return
        if self._auto_capture is not None or self._firmware_transfer_busy:
            self.status_message.emit("当前操作尚未结束，请稍后再写入仿射参数")
            return
        if len(self._coefficients) != 28 and not self._calculate():
            return
        if self._quality.get("recommendations"):
            QtWidgets.QMessageBox.warning(
                self,
                "暂不允许写入Flash",
                "综合分析仍有重测或覆盖建议，请处理后重新计算28组参数。",
            )
            return
        try:
            channels = self._firmware_coefficient_channels()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "参数不完整", str(exc))
            return
        calibration_id = self._firmware_parameter_id(channels)
        answer = QtWidgets.QMessageBox.question(
            self,
            "写入Flash并重启校验",
            f"即将下发28组参数，参数ID=0x{calibration_id:08X}。\n"
            "固件将写入Flash并执行一次软件复位，随后逐通道回读核对。\n"
            "本阶段只验证保存，不会把参数应用到Raw。是否继续？",
        )
        if not self._is_yes(answer):
            return
        self._firmware_transfer_busy = True
        self._update_control_states()
        self.global_status_label.setText("准备下发28组仿射参数……")
        self.firmware_persist_requested.emit({
            "calibration_id": calibration_id,
            "channels": channels,
            "project_path": str(self._project_path),
        })

    def firmware_transfer_progress(self, message):
        self.global_status_label.setText(str(message))
        self.status_message.emit(str(message))

    def firmware_transfer_finished(self, success, message):
        self._firmware_transfer_busy = False
        self._update_control_states()
        self.global_status_label.setText(str(message))
        self.status_message.emit(str(message))
        if success:
            QtWidgets.QMessageBox.information(self, "Flash保存校验成功", str(message))
        else:
            QtWidgets.QMessageBox.warning(self, "Flash保存校验失败", str(message))

    def _project_document(self):
        return {
            "schema_version": PROJECT_SCHEMA_VERSION,
            "project_id": self._project_id,
            "created_at": self._project_created_at,
            "updated_at": datetime.now().isoformat(timespec="milliseconds"),
            "ref_cdac_tiers": list(REF_CDAC_TIERS),
            "reference_ref_cdac": REFERENCE_REF_CDAC,
            "valid_raw_limit": VALID_RAW_LIMIT,
            "capture_frame_count": CAPTURE_FRAME_COUNT,
            "points": self._points,
            "coefficients": self._coefficients,
            "analysis": self._quality,
        }

    def _write_project_file(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = path.with_name(path.name + ".tmp")
        with temporary_path.open("w", encoding="utf-8") as handle:
            json.dump(self._project_document(), handle, ensure_ascii=False, indent=2)
        temporary_path.replace(path)
        self._project_path = path
        self._settings.setValue("calibration/last_project", str(path))
        self._update_project_label()

    def _autosave_project(self):
        try:
            self._write_project_file(self._project_path)
        except OSError as exc:
            self.status_message.emit(f"标定项目自动保存失败：{exc}")

    @Slot()
    def _save_project(self):
        try:
            self._write_project_file(self._project_path)
        except OSError as exc:
            self.status_message.emit(f"标定项目保存失败：{exc}")
            return
        message = f"项目已保存：{self._project_path.name}"
        self.status_message.emit(message)
        self.global_status_label.setText(message)

    @Slot()
    def _save_project_as(self):
        selected_path, _filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "标定项目另存为",
            str(self._project_path),
            "Autorange calibration project (*.arcal.json);;JSON files (*.json)",
        )
        if not selected_path:
            return
        path = Path(selected_path)
        if not path.name.lower().endswith(".json"):
            path = path.with_name(path.name + PROJECT_SUFFIX)
        self._project_path = path
        self._save_project()

    @Slot()
    def _new_project(self):
        if any(point["samples"] for point in self._points):
            answer = QtWidgets.QMessageBox.question(
                self,
                "新建标定项目",
                "当前项目已自动保存。确认新建一个空白标定项目吗？",
            )
            if not self._is_yes(answer):
                return
        self._project_id = uuid4().hex
        self._project_created_at = datetime.now().isoformat(timespec="seconds")
        self._project_path = self._new_default_project_path()
        self._points = [self._new_point("P0", "空载")]
        self._coefficients = []
        self._refresh_workspace_tabs(selected_point_id=self._points[0]["id"])
        self._autosave_project()

    @Slot()
    def _open_project(self):
        selected_path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "打开标定项目",
            str(self._project_path.parent),
            "Autorange calibration project (*.arcal.json *.json)",
        )
        if selected_path:
            self._load_project_file(Path(selected_path), announce=True)

    def _restore_last_project(self):
        value = self._settings.value("calibration/last_project", "")
        if not value:
            return
        path = Path(str(value))
        if path.is_file():
            self._load_project_file(path, announce=False)

    def _load_project_file(self, path, announce):
        try:
            with path.open("r", encoding="utf-8") as handle:
                document = json.load(handle)
            points = self._normalize_loaded_points(document.get("points", []))
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            if announce:
                QtWidgets.QMessageBox.warning(self, "项目读取失败", str(exc))
            return False
        self._project_id = str(document.get("project_id") or uuid4().hex)
        self._project_created_at = str(
            document.get("created_at") or datetime.now().isoformat(timespec="seconds")
        )
        self._project_path = path
        self._points = points
        self._coefficients = []
        try:
            self._coefficients = calculate_affine_coefficients(self._points)
        except ValueError:
            pass
        self._settings.setValue("calibration/last_project", str(path))
        self._refresh_workspace_tabs(selected_point_id=self._points[0]["id"])
        self._update_project_label()
        if announce:
            message = f"已读取项目：{path.name}，共{len(points)}个压力点"
            self.status_message.emit(message)
            self.global_status_label.setText(message)
        return True

    def _normalize_loaded_points(self, loaded_points):
        if not loaded_points:
            raise ValueError("项目中没有压力点")
        points = []
        for index, loaded in enumerate(loaded_points):
            samples = {}
            for key, sample in dict(loaded.get("samples", {})).items():
                ref_cdac = int(key)
                if ref_cdac not in REF_CDAC_TIERS:
                    continue
                raw = tuple(float(value) for value in sample["raw"])
                stddev = tuple(float(value) for value in sample["stddev"])
                minimum = tuple(int(value) for value in sample["minimum"])
                maximum = tuple(int(value) for value in sample["maximum"])
                if not all(len(value) == 4 for value in (raw, stddev, minimum, maximum)):
                    raise ValueError(f"{loaded.get('name', index)}的Ref{ref_cdac}通道数错误")
                normalized_sample = {
                    "raw": raw,
                    "stddev": stddev,
                    "minimum": minimum,
                    "maximum": maximum,
                    "sample_count": int(sample["sample_count"]),
                    "saved_at": str(sample["saved_at"]),
                }
                if sample.get("capture_mode"):
                    normalized_sample["capture_mode"] = str(sample["capture_mode"])
                for field_name in ("forward_raw", "reverse_raw", "sweep_delta"):
                    if sample.get(field_name) is not None:
                        normalized_sample[field_name] = tuple(
                            float(value) for value in sample[field_name]
                        )
                if sample.get("capture_times") is not None:
                    normalized_sample["capture_times"] = [
                        str(value) for value in sample["capture_times"]
                    ]
                samples[ref_cdac] = normalized_sample
            name = str(loaded.get("name") or f"P{index}")
            points.append({
                "id": str(loaded.get("id") or uuid4().hex),
                "name": name,
                "note": "空载" if index == 0 else str(loaded.get("note") or f"压力点 {name}"),
                "samples": samples,
                "history": list(loaded.get("history", [])),
                "last_auto_capture": dict(loaded.get("last_auto_capture", {})),
            })
        points[0]["name"] = "P0"
        points[0]["note"] = "空载"
        return points

    @Slot()
    def _import_legacy_csv(self):
        selected_path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "导入旧版标定样本CSV",
            str(Path(__file__).resolve().with_name("calibration_exports")),
            "CSV files (*.csv)",
        )
        if not selected_path:
            return
        path = Path(selected_path)
        if path.name.endswith("_coefficients.csv"):
            path = path.with_name(
                path.name[:-len("_coefficients.csv")] + "_samples.csv"
            )
        try:
            points = self._read_legacy_samples_csv(path)
        except (OSError, ValueError, KeyError) as exc:
            QtWidgets.QMessageBox.warning(self, "CSV导入失败", str(exc))
            return
        self._project_id = uuid4().hex
        self._project_created_at = datetime.now().isoformat(timespec="seconds")
        self._project_path = self._new_default_project_path()
        self._points = points
        self._coefficients = []
        try:
            self._coefficients = calculate_affine_coefficients(self._points)
        except ValueError:
            pass
        self._refresh_workspace_tabs(selected_point_id=self._points[0]["id"])
        self._autosave_project()
        message = f"已导入{path.name}并转换为单一项目文件，共{len(points)}个压力点"
        self.status_message.emit(message)
        self.global_status_label.setText(message)

    def _read_legacy_samples_csv(self, path):
        if not path.is_file():
            raise ValueError(f"找不到样本文件：{path}")
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            rows = list(csv.DictReader(handle))
        if not rows or "point" not in rows[0] or "ref_cdac" not in rows[0]:
            raise ValueError("所选文件不是有效的*_samples.csv")
        groups = {}
        order = []
        for row in rows:
            name = row["point"].strip()
            if name not in groups:
                groups[name] = {
                    "id": uuid4().hex,
                    "name": name,
                    "note": row.get("note", "") or f"压力点 {name}",
                    "samples": {},
                    "history": [],
                    "is_empty": row.get("is_empty_load", "0") == "1",
                }
                order.append(name)
            ref_cdac = int(row["ref_cdac"])
            groups[name]["samples"][ref_cdac] = {
                "raw": tuple(float(row[f"ch{channel}_hardware_raw"]) for channel in range(4, 8)),
                "stddev": tuple(float(row[f"ch{channel}_stddev"]) for channel in range(4, 8)),
                "minimum": tuple(int(row[f"ch{channel}_minimum"]) for channel in range(4, 8)),
                "maximum": tuple(int(row[f"ch{channel}_maximum"]) for channel in range(4, 8)),
                "sample_count": int(row["sample_count"]),
                "saved_at": row["saved_at"],
            }
        points = [groups[name] for name in order]
        points.sort(key=lambda point: (not point.pop("is_empty"), self._point_sort_key(point["name"])))
        if not points:
            raise ValueError("CSV中没有样本")
        points[0]["name"] = "P0"
        points[0]["note"] = "空载"
        return points

    def _point_sort_key(self, name):
        if name.startswith("P") and name[1:].isdigit():
            return int(name[1:])
        return 1000000

    @Slot()
    def _export_csv_snapshot(self):
        if not self._coefficients:
            try:
                self._coefficients = calculate_affine_coefficients(self._points)
            except ValueError:
                self._coefficients = []
        selected_directory = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "选择CSV快照保存目录",
            str(Path(__file__).resolve().with_name("calibration_exports")),
        )
        if not selected_directory:
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        snapshot_directory = Path(selected_directory) / f"autorange_calibration_{timestamp}"
        try:
            snapshot_directory.mkdir(parents=True, exist_ok=False)
            self._write_samples_csv(snapshot_directory / "samples.csv")
            self._write_coefficients_csv(snapshot_directory / "coefficients.csv")
            self._write_quality_csv(snapshot_directory / "quality_report.csv")
        except OSError as exc:
            self.status_message.emit(f"校准CSV快照导出失败：{exc}")
            return
        message = f"已导出CSV快照：{snapshot_directory}"
        self.status_message.emit(message)
        self.global_status_label.setText(message)

    def _write_samples_csv(self, path):
        fieldnames = [
            "point", "point_id", "is_empty_load", "note", "ref_cdac",
            "sample_count", "saved_at", "capture_mode", "capture_times",
        ]
        for channel in range(4, 8):
            fieldnames.extend([
                f"ch{channel}_hardware_raw",
                f"ch{channel}_stddev",
                f"ch{channel}_minimum",
                f"ch{channel}_maximum",
                f"ch{channel}_forward_raw",
                f"ch{channel}_reverse_raw",
                f"ch{channel}_sweep_delta",
            ])
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for point_index, point in enumerate(self._points):
                for ref_cdac in REF_CDAC_TIERS:
                    sample = point["samples"].get(ref_cdac)
                    if sample is None:
                        continue
                    row = {
                        "point": point["name"],
                        "point_id": point["id"],
                        "is_empty_load": 1 if point_index == 0 else 0,
                        "note": point["note"],
                        "ref_cdac": ref_cdac,
                        "sample_count": sample["sample_count"],
                        "saved_at": sample["saved_at"],
                        "capture_mode": sample.get("capture_mode", "manual"),
                        "capture_times": "|".join(sample.get("capture_times", [])),
                    }
                    for channel_index, channel in enumerate(range(4, 8)):
                        row[f"ch{channel}_hardware_raw"] = sample["raw"][channel_index]
                        row[f"ch{channel}_stddev"] = sample["stddev"][channel_index]
                        row[f"ch{channel}_minimum"] = sample["minimum"][channel_index]
                        row[f"ch{channel}_maximum"] = sample["maximum"][channel_index]
                        row[f"ch{channel}_forward_raw"] = self._optional_channel_value(
                            sample, "forward_raw", channel_index
                        )
                        row[f"ch{channel}_reverse_raw"] = self._optional_channel_value(
                            sample, "reverse_raw", channel_index
                        )
                        row[f"ch{channel}_sweep_delta"] = self._optional_channel_value(
                            sample, "sweep_delta", channel_index
                        )
                    writer.writerow(row)

    def _optional_channel_value(self, sample, field_name, channel_index):
        values = sample.get(field_name)
        if values is None or len(values) <= channel_index:
            return ""
        return values[channel_index]

    def _write_coefficients_csv(self, path):
        fieldnames = [
            "channel", "ref_cdac", "reference_ref_cdac", "a_gain", "b_offset",
            "gain_q16", "offset_int", "rmse_counts", "max_abs_error_counts",
            "r_squared", "direct_sample_count", "edge_min_sample_count", "fit_path",
            "raw_exclusion_limit",
        ]
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for coefficient in self._coefficients:
                writer.writerow({
                    "channel": coefficient["channel"],
                    "ref_cdac": coefficient["ref_cdac"],
                    "reference_ref_cdac": REFERENCE_REF_CDAC,
                    "a_gain": f"{coefficient['gain']:.12f}",
                    "b_offset": f"{coefficient['offset']:.6f}",
                    "gain_q16": coefficient["gain_q16"],
                    "offset_int": coefficient["offset_int"],
                    "rmse_counts": "" if coefficient["rmse"] is None else f"{coefficient['rmse']:.6f}",
                    "max_abs_error_counts": "" if coefficient["max_error"] is None else f"{coefficient['max_error']:.6f}",
                    "r_squared": "" if coefficient["r_squared"] is None else f"{coefficient['r_squared']:.9f}",
                    "direct_sample_count": coefficient["direct_sample_count"],
                    "edge_min_sample_count": coefficient["edge_min_sample_count"],
                    "fit_path": coefficient["fit_path"],
                    "raw_exclusion_limit": VALID_RAW_LIMIT,
                })

    def _write_quality_csv(self, path):
        fieldnames = ["record_type", "point", "ref_cdac", "status", "score", "action", "reason"]
        with path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for point in self._points:
                result = self._quality["points"][point["id"]]
                writer.writerow({
                    "record_type": "point",
                    "point": point["name"],
                    "ref_cdac": "",
                    "status": result["status"],
                    "score": f"{result['score']:.6f}",
                    "action": "重测整个压力点" if result["remeasure_whole_point"] else "重测指定Ref",
                    "reason": "；".join(result["reasons"]),
                })
                for ref_cdac, sample_result in result["samples"].items():
                    if sample_result["status"] == "合格":
                        continue
                    writer.writerow({
                        "record_type": "sample",
                        "point": point["name"],
                        "ref_cdac": ref_cdac,
                        "status": sample_result["status"],
                        "score": "",
                        "action": "重测" if sample_result["status"] == "建议重测" else "检查",
                        "reason": "；".join(sample_result["reasons"]),
                    })
            for item in self._quality.get("coverage", []):
                writer.writerow({
                    "record_type": "coverage",
                    "point": "新增",
                    "ref_cdac": item["ref_cdac"],
                    "status": "警告",
                    "score": "",
                    "action": "增加有效中间压力点",
                    "reason": (
                        f"CH{item['channel']}只有{item['valid_count']}组有效配对，"
                        f"至少需要{item['required_count']}组"
                    ),
                })

    def _update_project_label(self):
        self.project_label.setText(f"项目：{self._project_path.name}")
        self.project_label.setToolTip(str(self._project_path))

    def _is_yes(self, value):
        yes_value = getattr(
            QtWidgets.QMessageBox,
            "Yes",
            QtWidgets.QMessageBox.StandardButton.Yes,
        )
        return value == yes_value

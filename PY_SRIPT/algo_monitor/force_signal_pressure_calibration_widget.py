"""显示并更新传感器标定生成的Signal合力－压力K值表。"""

from __future__ import annotations

from force_signal_pressure_calibration import (
    ForceSignalPressureCalibrationError,
    QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT,
    build_quick_corrected_mapping,
    build_latest_signal_pressure_mapping,
    save_signal_pressure_mapping_document,
    select_firmware_calibration_document_nodes,
)
from force_signal_pressure_lookup import (
    ForceSignalPressureLookupError,
    load_latest_signal_pressure_document,
)
from qt_compat import QtCore, QtWidgets, Signal, Slot


def _qt_enum(owner, legacy_name, enum_group_name, member_name):
    """同时兼容 Qt5 的直接枚举和 Qt6 的作用域枚举。"""
    value = getattr(owner, legacy_name, None)
    if value is not None:
        return value
    enum_group = getattr(owner, enum_group_name)
    return getattr(enum_group, member_name)


class ForceSignalPressureCalibrationWidget(QtWidgets.QWidget):
    status_message = Signal(str)
    mapping_changed = Signal()
    firmware_persist_requested = Signal(object)
    firmware_trial_save_requested = Signal()
    firmware_trial_discard_requested = Signal()
    ram_trial_state_changed = Signal(bool)
    quick_correction_ram_result = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._document = None
        self._connected = False
        self._firmware_transfer_busy = False
        self._document_before_trial = None
        self._trial_document = None
        self._ram_trial_active = False
        self._quick_correction_adjustment_summary = ""
        self._build_ui()
        self._load_existing_mapping()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        description = QtWidgets.QLabel(
            "读取最新一批已经完整结束的传感器标定数据，以每个压力档的"
            "Signal合力中位值和压力计实测压力中位值为节点，生成分段线性K值表。"
        )
        description.setWordWrap(True)
        description.setStyleSheet("color:#b9d7e8;")
        layout.addWidget(description)

        action_row = QtWidgets.QHBoxLayout()
        self.load_button = QtWidgets.QPushButton("读取最新标定数据并生成K值表")
        self.load_button.setObjectName("ForceSignalCalibrationLoadButton")
        self.load_button.clicked.connect(self.load_latest_calibration)
        action_row.addWidget(self.load_button)
        self.flash_button = QtWidgets.QPushButton("写入MCU Flash")
        self.flash_button.setObjectName("ForceSignalCalibrationFlashButton")
        self.flash_button.clicked.connect(self._request_firmware_persist)
        action_row.addWidget(self.flash_button)
        self.save_trial_button = QtWidgets.QPushButton("确认快调并写入Flash")
        self.save_trial_button.setObjectName("ForceSignalCalibrationTrialSaveButton")
        self.save_trial_button.clicked.connect(self._request_trial_save)
        action_row.addWidget(self.save_trial_button)
        self.discard_trial_button = QtWidgets.QPushButton("撤销RAM快调")
        self.discard_trial_button.setObjectName(
            "ForceSignalCalibrationTrialDiscardButton"
        )
        self.discard_trial_button.clicked.connect(self._request_trial_discard)
        action_row.addWidget(self.discard_trial_button)
        action_row.addStretch(1)
        layout.addLayout(action_row)

        self.source_label = QtWidgets.QLabel("数据源：尚未读取")
        self.source_label.setWordWrap(True)
        self.source_label.setTextInteractionFlags(
            _qt_enum(
                QtCore.Qt,
                "TextSelectableByMouse",
                "TextInteractionFlag",
                "TextSelectableByMouse",
            )
        )
        self.source_label.setStyleSheet("color:#8bd5ff;")
        layout.addWidget(self.source_label)

        self.summary_label = QtWidgets.QLabel(
            "分段公式：P = P₁ + K × (Signal − Signal₁)"
        )
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet("color:#d8ff74;font-weight:700;")
        layout.addWidget(self.summary_label)

        self.table = QtWidgets.QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            (
                "分段",
                "压力计实测压力/N",
                "Signal合力",
                "K / N·count⁻¹",
                "1/K / count·N⁻¹",
                "插值公式",
            )
        )
        self.table.setEditTriggers(
            _qt_enum(
                QtWidgets.QAbstractItemView,
                "NoEditTriggers",
                "EditTrigger",
                "NoEditTriggers",
            )
        )
        self.table.setSelectionBehavior(
            _qt_enum(
                QtWidgets.QAbstractItemView,
                "SelectRows",
                "SelectionBehavior",
                "SelectRows",
            )
        )
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        resize_to_contents = _qt_enum(
            QtWidgets.QHeaderView,
            "ResizeToContents",
            "ResizeMode",
            "ResizeToContents",
        )
        stretch = _qt_enum(
            QtWidgets.QHeaderView,
            "Stretch",
            "ResizeMode",
            "Stretch",
        )
        header = self.table.horizontalHeader()
        for column in range(5):
            header.setSectionResizeMode(column, resize_to_contents)
        header.setSectionResizeMode(5, stretch)
        layout.addWidget(self.table, 1)

        self.status_label = QtWidgets.QLabel("等待读取最新传感器标定数据")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color:#f4d35e;font-weight:700;")
        layout.addWidget(self.status_label)

        self.setStyleSheet(
            "QPushButton#ForceSignalCalibrationLoadButton{"
            "background:#6fd5a5;color:#102019;font-weight:700;padding:7px 12px;}"
            "QPushButton#ForceSignalCalibrationFlashButton{"
            "background:#f4d35e;color:#211b00;font-weight:700;padding:7px 12px;}"
            "QPushButton#ForceSignalCalibrationTrialSaveButton{"
            "background:#6fd5a5;color:#102019;font-weight:800;padding:7px 12px;}"
            "QPushButton#ForceSignalCalibrationTrialDiscardButton{"
            "background:#edbbb6;color:#321310;font-weight:800;padding:7px 12px;}"
            "QTableWidget{background:#080c12;alternate-background-color:#101823;"
            "color:#d8edf8;gridline-color:#304556;}"
            "QHeaderView::section{background:#182431;color:#c9e7f5;"
            "padding:6px;border:1px solid #304556;font-weight:700;}"
        )
        self._refresh_firmware_button()

    def _load_existing_mapping(self):
        try:
            document = load_latest_signal_pressure_document()
        except ForceSignalPressureLookupError:
            return
        metadata = document["metadata"]
        self._show_document(
            document,
            metadata.get("source_path", metadata.get("source_csv", "--")),
            "当前 latest K值表",
        )
        self.status_label.setText("已读取当前N单位K值表，可写入MCU Flash")
        self.status_label.setStyleSheet("color:#5be39b;font-weight:700;")

    def _refresh_firmware_button(self):
        self.flash_button.setEnabled(
            self._connected
            and self._document is not None
            and not self._firmware_transfer_busy
            and not self._ram_trial_active
        )
        self.load_button.setEnabled(
            not self._firmware_transfer_busy and not self._ram_trial_active
        )
        trial_action_available = (
            self._connected
            and self._ram_trial_active
            and not self._firmware_transfer_busy
        )
        self.save_trial_button.setEnabled(trial_action_available)
        self.discard_trial_button.setEnabled(trial_action_available)

    def set_connected(self, connected):
        self._connected = bool(connected)
        self._refresh_firmware_button()

    def _publish_quick_correction_ram_result(self, state, message):
        """把RAM写入阶段与最终回读结果同步到按力页面。"""
        self.quick_correction_ram_result.emit(
            {
                "state": str(state),
                "message": str(message),
            }
        )

    @Slot(object)
    def apply_quick_correction_results(self, payload):
        """将两段代表档位的单轮结果变成累计范围受限的RAM试用表。"""
        if self._firmware_transfer_busy or self._ram_trial_active:
            message = "已有力标定传输或RAM快调正在进行，本次结果未写入MCU RAM"
            self.status_message.emit(message)
            self._publish_quick_correction_ram_result("failed", message)
            return
        if not self._connected:
            message = "串口未连接，快速修正结果没有写入MCU RAM"
            self.status_message.emit(message)
            self._publish_quick_correction_ram_result("failed", message)
            return
        try:
            base_document = load_latest_signal_pressure_document()
            result = build_quick_corrected_mapping(
                base_document,
                dict(payload or {}).get("entries") or (),
            )
        except (
            ForceSignalPressureCalibrationError,
            ForceSignalPressureLookupError,
        ) as exc:
            message = f"快速K修正计算失败，未写入MCU RAM：{exc}"
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color:#ff7b72;font-weight:700;")
            self.status_message.emit(message)
            self._publish_quick_correction_ram_result("failed", message)
            return

        changed_node_count = int(result.get("changed_node_count", 0))
        total_node_count = len(result["node_adjustments"])
        limited_node_count = int(result.get("limited_node_count", 0))
        if changed_node_count <= 0:
            message = (
                "快速K修正没有产生有效变化，未写入MCU RAM："
                f"实际改变0/{total_node_count}个K表节点，"
                f"{limited_node_count}/{total_node_count}个节点被相对首次基础表累计"
                f"{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}限制夹回原值"
            )
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color:#ff7b72;font-weight:700;")
            self.status_message.emit(message)
            self._publish_quick_correction_ram_result("failed", message)
            return

        node_change_summary = (
            f"K表Signal节点实际改变 {changed_node_count}/{total_node_count}；"
            f"改变量 {result['minimum_node_change_counts']:+.1f}～"
            f"{result['maximum_node_change_counts']:+.1f} counts；"
            f"最大绝对变化 {result['maximum_absolute_node_change_counts']:.1f} counts"
            f"（{result['maximum_absolute_node_change_ratio'] * 100.0:.1f}%）"
        )
        self._document_before_trial = base_document
        self._trial_document = result["document"]
        self._ram_trial_active = False
        source_path = dict(payload or {}).get("source_path") or "本次单轮短测"
        self._show_document(
            self._trial_document,
            source_path,
            "尚未保存：仅准备写入MCU RAM",
        )
        ratios = ", ".join(
            f"{sample['target_force_n']:.0f}N "
            f"{(sample['applied_ratio'] - 1.0) * 100.0:+.1f}%"
            for sample in result["samples"]
        )
        limited_text = (
            f"；其中{result['limited_sample_count']}个实测档需求超出"
            f"{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}"
            if result["limited_sample_count"]
            else ""
        )
        if result["limited_sample_count"] or limited_node_count:
            self._quick_correction_adjustment_summary = (
                f"{node_change_summary}；本轮有{result['limited_sample_count']}/"
                f"{len(result['samples'])}个实测档的推算修正超出"
                f"{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}；"
                f"{limited_node_count}/{len(result['node_adjustments'])}个K节点受到"
                f"相对首次基础表累计{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}限制"
            )
        else:
            self._quick_correction_adjustment_summary = (
                f"{node_change_summary}；本轮所有实测档和K节点均未触及"
                f"{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}修正上限"
            )
        self._firmware_transfer_busy = True
        self._refresh_firmware_button()
        progress_message = (
            f"{node_change_summary}；单轮修正比例：{ratios}{limited_text}；"
            "正在暂存到MCU RAM并逐点回读，Flash不改写……"
        )
        self.firmware_transfer_progress(progress_message)
        self._publish_quick_correction_ram_result("writing", progress_message)
        base_selection = select_firmware_calibration_document_nodes(
            self._document_before_trial
        )
        selected_indices = tuple(result["firmware_node_indices"])
        nodes = tuple(
            self._trial_document["nodes"][index]
            for index in selected_indices
        )
        base_nodes = base_selection["nodes"]
        self.firmware_persist_requested.emit(
            {
                "region": 0,
                "force_type": 0,
                "raw_points": tuple(
                    float(node["signal_sum_median"]) / 1000.0
                    for node in nodes
                ),
                "calibrated_points": tuple(
                    float(node["measured_pressure_median_n"])
                    for node in nodes
                ),
                "base_raw_points": tuple(
                    float(node["signal_sum_median"]) / 1000.0
                    for node in base_nodes
                ),
                "base_calibrated_points": tuple(
                    float(node["measured_pressure_median_n"])
                    for node in base_nodes
                ),
                "source_created_at": self._trial_document["metadata"].get(
                    "created_at", ""
                ),
                "persist": False,
                "trial_kind": "quick_correction",
            }
        )

    def firmware_transfer_progress(self, message):
        self.status_label.setText(str(message))
        self.status_label.setStyleSheet("color:#f4d35e;font-weight:700;")

    def firmware_transfer_finished(self, success, message):
        quick_correction_pending = (
            self._trial_document is not None and not self._ram_trial_active
        )
        self._firmware_transfer_busy = False
        if quick_correction_pending:
            previous_document = self._document_before_trial
            self._trial_document = None
            self._document_before_trial = None
            self._quick_correction_adjustment_summary = ""
            if previous_document is not None:
                self._show_document(
                    previous_document,
                    previous_document["metadata"].get("source_path", "--"),
                    "当前 latest K值表",
                )
        self._refresh_firmware_button()
        self.status_label.setText(str(message))
        color = "#5be39b" if success else "#ff7b72"
        self.status_label.setStyleSheet(f"color:{color};font-weight:700;")
        self.status_message.emit(str(message))
        if quick_correction_pending:
            detail = str(message)
            if success:
                detail = f"写入流程结束但未完成RAM逐点回读确认：{detail}"
            self._publish_quick_correction_ram_result("failed", detail)

    def firmware_ram_trial_ready(self, message):
        """标记快速修正表已由MCU逐点回读确认并正在RAM生效。"""
        display_message = str(message)
        if self._quick_correction_adjustment_summary:
            display_message = (
                f"{display_message}\n{self._quick_correction_adjustment_summary}"
            )
        self._firmware_transfer_busy = False
        self._ram_trial_active = True
        self._refresh_firmware_button()
        self.ram_trial_state_changed.emit(True)
        self.status_label.setText(display_message)
        self.status_label.setStyleSheet("color:#5be39b;font-weight:700;")
        self.status_message.emit(display_message)
        self._publish_quick_correction_ram_result("verified", display_message)

    def firmware_trial_action_finished(
        self,
        action,
        success,
        message,
        retain_trial=False,
    ):
        """处理RAM试用表保存或撤销后的本地文档状态。"""
        mcu_action_succeeded = bool(success)
        self._firmware_transfer_busy = False
        if success and action == "save" and self._trial_document is not None:
            try:
                saved = save_signal_pressure_mapping_document(
                    self._trial_document,
                    "quick",
                )
            except ForceSignalPressureCalibrationError as exc:
                message = f"{message}；但上位机保存latest K值表失败：{exc}"
                success = False
            else:
                self.source_label.setText(
                    f"数据源：单轮快速K修正\n"
                    f"已保存最新版及历史表：{saved['archive_path']}"
                )
                self.mapping_changed.emit()
        elif success and action == "discard":
            previous_document = self._document_before_trial
            if previous_document is not None:
                self._show_document(
                    previous_document,
                    previous_document["metadata"].get("source_path", "--"),
                    "当前 latest K值表",
                )

        if not retain_trial:
            self._ram_trial_active = False
            self._trial_document = None
            self._document_before_trial = None
            self._quick_correction_adjustment_summary = ""
        else:
            self._ram_trial_active = True
        self._refresh_firmware_button()
        self.ram_trial_state_changed.emit(self._ram_trial_active)
        self.status_label.setText(str(message))
        color = "#5be39b" if success else "#ff7b72"
        self.status_label.setStyleSheet(f"color:{color};font-weight:700;")
        self.status_message.emit(str(message))
        if mcu_action_succeeded and action == "save":
            result_state = "flash_saved"
        elif mcu_action_succeeded and action == "discard":
            result_state = "discarded"
        elif retain_trial:
            result_state = "verified"
        else:
            result_state = "failed"
        self._publish_quick_correction_ram_result(result_state, message)

    def firmware_trial_lost(self, message):
        """串口断开时清除只存在于易失性RAM中的试用状态。"""
        previous_document = self._document_before_trial
        self._firmware_transfer_busy = False
        self._ram_trial_active = False
        self._trial_document = None
        self._document_before_trial = None
        self._quick_correction_adjustment_summary = ""
        if previous_document is not None:
            self._show_document(
                previous_document,
                previous_document["metadata"].get("source_path", "--"),
                "当前 latest K值表",
            )
        self._refresh_firmware_button()
        self.ram_trial_state_changed.emit(False)
        self.status_label.setText(str(message))
        self.status_label.setStyleSheet("color:#ff9f43;font-weight:700;")
        self.status_message.emit(str(message))
        self._publish_quick_correction_ram_result("lost", message)

    @staticmethod
    def _table_item(text, align_right=False):
        item = QtWidgets.QTableWidgetItem(str(text))
        if align_right:
            alignment = _qt_enum(
                QtCore.Qt,
                "AlignRight",
                "AlignmentFlag",
                "AlignRight",
            ) | _qt_enum(
                QtCore.Qt,
                "AlignVCenter",
                "AlignmentFlag",
                "AlignVCenter",
            )
            item.setTextAlignment(alignment)
        return item

    def _show_document(self, document, source_path, archive_path):
        self._document = document
        metadata = document["metadata"]
        nodes = document["nodes"]
        segments = document["segments"]
        self.table.setRowCount(len(segments))
        for row_index, segment in enumerate(segments):
            lower_node = nodes[row_index]
            upper_node = nodes[row_index + 1]
            lower_pressure = float(lower_node["measured_pressure_median_n"])
            upper_pressure = float(upper_node["measured_pressure_median_n"])
            lower_signal = float(lower_node["signal_sum_median"])
            upper_signal = float(upper_node["signal_sum_median"])
            k_value = float(segment["k_n_per_signal_count"])
            values = (
                f"{row_index + 1}",
                f"{lower_pressure:.2f} ～ {upper_pressure:.2f}",
                f"{lower_signal:g} ～ {upper_signal:g}",
                f"{k_value:.9f}",
                f"{1.0 / k_value:.3f}",
                (
                    f"P = {lower_pressure:.2f} + {k_value:.9f}"
                    f" × (S − {lower_signal:g})"
                ),
            )
            for column, value in enumerate(values):
                self.table.setItem(
                    row_index,
                    column,
                    self._table_item(value, align_right=column in (0, 3, 4)),
                )
        self.source_label.setText(
            f"数据源：{source_path}\n"
            f"已保存最新版及历史表：{archive_path}"
        )
        self.summary_label.setText(
            f"共{len(nodes)}个中位值节点、{len(segments)}个线性分段；"
            f"实测压力范围 {nodes[0]['measured_pressure_median_n']:.2f}～"
            f"{nodes[-1]['measured_pressure_median_n']:.2f} N。"
            "分段公式：P = P₁ + K × (Signal − Signal₁)"
        )

        self._refresh_firmware_button()

    @Slot()
    def _request_firmware_persist(self):
        if self._document is None:
            self.firmware_transfer_finished(False, "当前没有可写入的N单位K值表")
            return
        if not self._connected:
            self.firmware_transfer_finished(False, "串口未连接，无法写入MCU Flash")
            return
        try:
            firmware_selection = select_firmware_calibration_document_nodes(
                self._document
            )
        except ForceSignalPressureCalibrationError as exc:
            self.firmware_transfer_finished(False, str(exc))
            return
        firmware_point_count = len(firmware_selection["nodes"])
        yes_button = _qt_enum(
            QtWidgets.QMessageBox,
            "Yes",
            "StandardButton",
            "Yes",
        )
        no_button = _qt_enum(
            QtWidgets.QMessageBox,
            "No",
            "StandardButton",
            "No",
        )
        answer = QtWidgets.QMessageBox.question(
            self,
            "确认写入力标定",
            f"将把{firmware_point_count}节点、{firmware_point_count - 1}分段的"
            "完整Signal合力－压力表原样写入MCU region 0法向标定，不再筛选或压缩节点。\n"
            "写入流程会先回读并保护现有切向、方向和region 1参数，然后保存、复位并再次回读。\n\n"
            "是否继续？",
            yes_button | no_button,
            no_button,
        )
        if answer != yes_button:
            return
        nodes = firmware_selection["nodes"]
        request = {
            "region": 0,
            "force_type": 0,
            "raw_points": tuple(
                float(node["signal_sum_median"]) / 1000.0
                for node in nodes
            ),
            "calibrated_points": tuple(
                float(node["measured_pressure_median_n"])
                for node in nodes
            ),
            "source_created_at": self._document["metadata"].get(
                "created_at", ""
            ),
            "source_point_count": firmware_selection["source_point_count"],
            "maximum_compression_error_n": firmware_selection[
                "maximum_error_n"
            ],
        }
        self._firmware_transfer_busy = True
        self._refresh_firmware_button()
        self.firmware_transfer_progress(
            "正在回读MCU现有6张力标定表，尚未改写Flash……"
        )
        self.firmware_persist_requested.emit(request)

    @Slot()
    def _request_trial_save(self):
        if not self._ram_trial_active or self._trial_document is None:
            self.status_message.emit("当前没有正在MCU RAM生效的快速修正表")
            return
        yes_button = _qt_enum(
            QtWidgets.QMessageBox,
            "Yes",
            "StandardButton",
            "Yes",
        )
        no_button = _qt_enum(
            QtWidgets.QMessageBox,
            "No",
            "StandardButton",
            "No",
        )
        answer = QtWidgets.QMessageBox.question(
            self,
            "确认保存快速修正",
            f"当前{QUICK_CORRECTION_ADJUSTMENT_LIMIT_TEXT}快速修正表正在MCU RAM中试用。\n"
            "确认后将写入Flash、复位MCU并逐表回读验证。\n\n"
            "是否继续？",
            yes_button | no_button,
            no_button,
        )
        if answer != yes_button:
            return
        self._firmware_transfer_busy = True
        self._refresh_firmware_button()
        self.firmware_transfer_progress("正在把RAM快速修正表保存到MCU Flash……")
        self.firmware_trial_save_requested.emit()

    @Slot()
    def _request_trial_discard(self):
        if not self._ram_trial_active:
            self.status_message.emit("当前没有可撤销的RAM快速修正表")
            return
        self._firmware_transfer_busy = True
        self._refresh_firmware_button()
        self.firmware_transfer_progress("正在撤销RAM快速修正并恢复Flash原表……")
        self.firmware_trial_discard_requested.emit()

    @Slot()
    def load_latest_calibration(self):
        self.load_button.setEnabled(False)
        try:
            result = build_latest_signal_pressure_mapping()
            document = load_latest_signal_pressure_document()
        except (
            ForceSignalPressureCalibrationError,
            ForceSignalPressureLookupError,
        ) as exc:
            message = f"生成传感器标定K值表失败：{exc}"
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color:#ff7b72;font-weight:700;")
            self.status_message.emit(message)
            return
        finally:
            self._refresh_firmware_button()
        self._show_document(
            document,
            result["source_path"],
            result["archive_path"],
        )
        message = (
            f"已读取最新完整传感器标定数据并生成"
            f"{len(document['segments'])}段K值表"
        )
        self.status_label.setText(message)
        self.status_label.setStyleSheet("color:#5be39b;font-weight:700;")
        self.status_message.emit(message)
        self.mapping_changed.emit()

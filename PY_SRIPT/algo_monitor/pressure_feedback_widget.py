"""直压机构的压力计反馈试验页，复用既有电机连接和操作互锁。"""

from collections import deque
from dataclasses import asdict
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
import time
import tempfile

import pyqtgraph as pg

from motor_mapping_interface import LEGACY_PROFILE
from pressure_feedback_control import (
    FeedbackControlError, FeedbackSettings, PressureFeedbackController, PressureSamples,
    RELEASE_REFERENCE_TOLERANCE_COUNTS, CONTROL_STRATEGY,
    LearnedPressureMap, pressure_tolerance_n, LOW_FORCE_MAX_N, LOW_FORCE_ZERO_N,
)
from qt_compat import QtCore, QtWidgets, Signal, Slot
from wrapped_label import WrappedLabel
from pressure_learning_store import load_learning, save_learning


def feedback_settings_path(profile):
    """按型号及驱动命名空间隔离参数，ZDT命名空间包含地址和每圈步数。"""
    return Path(__file__).with_name(f"pressure_feedback_config_{profile.namespace or profile.model}.json")


class PressureFeedbackWidget(QtWidgets.QWidget):
    status_message = Signal(str)
    progress = Signal(object)
    finished = Signal(object)
    learning_notice_changed = Signal(str)

    def __init__(self, motor_control, parent=None, output_dir=None, settings_path=None):
        super().__init__(parent)
        self.control = motor_control
        self.profile = getattr(motor_control, "profile", LEGACY_PROFILE)
        self._settings_path = Path(settings_path) if settings_path is not None else feedback_settings_path(self.profile)
        self._learning_path = self._settings_path.with_suffix('.learning.json')
        self._learning_storage_error = ''
        self._settings_dirty = False
        self._loading_settings = True
        self.samples = PressureSamples()
        self.output_dir = Path(output_dir) if output_dir else Path(__file__).with_name("pressure_feedback_tests")
        self._running = False
        self._releasing = False
        self._release_after_stop = False
        self._pending_release = False
        self._learning = False
        self._learned_map = None
        self._learning_suspended_reason = None
        self._learning_suspended_at = None
        self._learning_recovery_samples = []
        self._learning_notice_text = None
        self._learning_notice_level = None
        self._learning_task_notice = None
        self._learning_generation = 0
        self._result = None
        self._points = deque(maxlen=6000)
        self._plot_origin = None
        self._last_plot_at = -math.inf
        self._monitoring = True
        self._plot_target_n = 1.0
        self._external_plot_target = None
        self._plot_tolerance_n = 0.05
        self._settings = None
        self._build_ui()
        self._save_timer = QtCore.QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(300)
        self._save_timer.timeout.connect(self._save_settings)
        self._load_settings()
        self._load_learning()
        self._loading_settings = False
        if self._settings_dirty:
            self._save_timer.start()
        for spin in self.inputs.values():
            spin.valueChanged.connect(self._schedule_settings_save)
            spin.editingFinished.connect(self._save_settings)
        self.progress.connect(self._on_progress)
        self.finished.connect(self._on_finished)
        self.control.control_state_changed.connect(self._control_state_changed)
        self.control.release_position_changed.connect(self._release_reference_changed)
        self.control.worker_position_zeroed.connect(self._coordinates_invalidated)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._refresh)
        self.timer.start()
        self._refresh()

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("压力反馈控制 · 直压试验")
        title.setStyleSheet("font-size:20px;font-weight:700;")
        root.addWidget(title)
        description = QtWidgets.QLabel(
            "以压力计实测值调节位置：自动估算调整步长，偏低下压、偏高回退，容差内也围绕目标微调。"
            "先在无接触位置设置释放点。保持结束后仍可能承压，卸载请点击“停止并释放”。")
        description.setWordWrap(True)
        root.addWidget(description)
        self.live_label = QtWidgets.QLabel("压力计 -- N · 位置 -- step")
        self.live_label.setStyleSheet("font-size:22px;font-weight:700;")
        root.addWidget(self.live_label)
        self.release_label = QtWidgets.QLabel()
        self.release_label.setWordWrap(True)
        root.addWidget(self.release_label)

        form = QtWidgets.QGridLayout()
        self.inputs = {}
        # 参数名、中文标签、单位、范围、小数位；单位放在输入框外。
        fields = [
            ("target_n", "目标压力", "N", .1, 30, 2),
            ("tolerance_percent", "允许误差 ±", "%", .0001, 99.9999, 4),
            ("hold_s", "到位后保持", "秒", 1, 3600, 1),
            # 不设业务上限，使用整数输入控件可表示的最大值；实际运动仍检查行程。
            ("max_step", "每次最大步数", "step", 1, 2**31-1, 0),
            ("speed", "微调速度", self.profile.speed_unit, 1, min(30, self.profile.speed_max), 0),
            ("approach_speed", "快速接近速度", self.profile.speed_unit, 1, min(30, self.profile.speed_max), 0),
            ("settle_s", "到位后等待", "秒", .1, 5, 2),
            ("force_limit_n", "保护压力", "N", .2, 35, 2),
            ("max_travel", "释放点以下最大行程", "step", 1, self.profile.steps_per_turn*12, 0),
            ("max_current_ma", "电流上限", "mA", 1, 3000, 0),
            ("stable_s", "首次到位确认", "秒", .5, 10, 1),
            ("acquire_timeout_s", "寻压超时", "秒", 10, 600, 0),
            ("acceleration", "加速度", self.profile.acceleration_unit, 1, self.profile.acceleration_max, 0),
            ("release_speed", "释放速度", self.profile.speed_unit, 1, min(400, self.profile.speed_max), 0),
            ("plot_window_s", "曲线显示时长", "秒", 1, 600, 0),
        ]
        defaults = FeedbackSettings()
        for index, (name, label, unit, low, high, decimals) in enumerate(fields):
            spin = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
            if decimals:
                spin.setDecimals(decimals)
                spin.setSingleStep(.01 if decimals == 2 else .5)
            spin.setRange(low, high)
            value = (30 if name == "approach_speed" and self.profile.model == "zdt_x42s" else
                     20 if name == "plot_window_s" else defaults.tolerance_n / defaults.target_n * 100
                     if name == "tolerance_percent" else getattr(defaults, name))
            spin.setValue(value if decimals else int(value))
            spin.setMinimumWidth(76)
            spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(self._refresh)
            self.inputs[name] = spin
            if name == "tolerance_percent":
                spin.setToolTip("相对于目标压力的百分比；目标≤0.2 N时有效容差至少±0.01 N，与压力计读数分辨率匹配。")
            elif name == "max_step":
                spin.setToolTip("单次运动步数上限。接触后按实测压力变化自动计算步长；增大上限不会直接增大微调力度。")
            elif name == "approach_speed":
                spin.setToolTip("仅已核验学习表的保守接近段使用；无学习表时仍按微调速度寻压。")
            elif name == "plot_window_s":
                spin.setToolTip("只显示最近这段时间的压力和位置；默认20秒，修改后自动保存，运行中也可调整。")
            row, col = index//3, (index % 3)*3
            form.addWidget(QtWidgets.QLabel(label), row, col)
            form.addWidget(spin, row, col+1)
            form.addWidget(QtWidgets.QLabel(unit), row, col+2)
        root.addLayout(form)
        self.settings_status_label = QtWidgets.QLabel()
        self.settings_status_label.setWordWrap(True)
        self.settings_status_label.setToolTip(str(self._settings_path))
        root.addWidget(self.settings_status_label)
        self.low_force_notice_label = WrappedLabel()
        self.low_force_notice_label.setText("0.1～0.2 N：有效容差至少±0.01 N。学习前在释放点确认±0.02 N内的稳定底数，再以增加0.01～0.02 N确认接触。底数与学习时一致才快速走空行程；接触及受力段≤2 RPM。目标压力仍按原始读数控制。")
        self.low_force_notice_label.setStyleSheet("color:#ffe1a3;background:#3b2a10;border-radius:5px;padding:6px;")
        root.addWidget(self.low_force_notice_label)
        self.learning_notice_label = WrappedLabel()
        self.learning_notice_label.setStyleSheet(
            "color:#ffe1a3;background:#3b2a10;border:2px solid #e7a83e;"
            "border-radius:7px;padding:10px 12px;font-size:15px;font-weight:700;")
        root.addWidget(self.learning_notice_label)
        quick_row = QtWidgets.QGridLayout()
        quick_row.setSpacing(8)
        quick_row.setContentsMargins(0, 4, 0, 4)
        quick_row.addWidget(QtWidgets.QLabel("快捷启动"), 0, 0, 2, 1)
        self.quick_target_buttons = {}
        for index, target in enumerate((0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 20, 25, 30)):
            button = QtWidgets.QPushButton(f"{target} N")
            button.setMinimumWidth(44)
            button.setMinimumHeight(36)
            button.setStyleSheet(
                "QPushButton {background:#111d2a;color:#d9e7f3;border:1px solid #2b4054;"
                "border-radius:8px;padding:6px 10px;font-weight:600;}"
                "QPushButton:hover {background:#1b3043;border-color:#477696;color:#ffffff;}"
                "QPushButton:pressed {background:#24435b;border-color:#64b5dc;}"
                "QPushButton:focus {border-color:#64b5dc;}"
                "QPushButton:disabled {background:#0d151e;border-color:#1c2936;color:#5e7183;}")
            button.setToolTip(f"设置目标压力为{target} N并开始闭环保持，沿用当前保护参数。")
            button.clicked.connect(lambda checked=False, value=target: self.start_target(value))
            self.quick_target_buttons[target] = button
            quick_row.addWidget(button, index//6, index%6+1)
        root.addLayout(quick_row)
        learning_row = QtWidgets.QHBoxLayout()
        learning_row.setSpacing(12)
        learning_row.setContentsMargins(0, 4, 0, 2)
        self.learn_button = QtWidgets.QPushButton("一键学习到 30 N")
        self.learn_button.setMinimumSize(230, 48)
        self.learn_button.setStyleSheet(
            "QPushButton {background:#14543f;color:#f0faf5;border:1px solid #28785b;"
            "border-radius:9px;padding:10px 24px;font-size:15px;font-weight:700;}"
            "QPushButton:hover {background:#1b694e;border-color:#419b76;}"
            "QPushButton:pressed {background:#103e30;border-color:#5aad88;}"
            "QPushButton:focus {border-color:#74c99f;}"
            "QPushButton:disabled {background:#132f26;border-color:#224638;color:#78988a;}")
        self.learn_button.setToolTip("先卸载回释放点，允许±0.02 N内的稳定底数（包括0.01 N）；相对底数增加0.01～0.02 N时确认接触，再记录0.05～30 N共12个加载点，最后自动释放。")
        self.clear_learning_button = QtWidgets.QPushButton("清除学习")
        self.clear_learning_button.setMinimumHeight(36)
        self.clear_learning_button.setStyleSheet(
            "QPushButton {background:transparent;color:#9caebf;border:1px solid #2b3744;"
            "border-radius:8px;padding:6px 16px;}"
            "QPushButton:hover {background:#19232e;color:#e0e9f0;border-color:#506276;}"
            "QPushButton:pressed {background:#243240;}"
            "QPushButton:focus {border-color:#64b5dc;}"
            "QPushButton:disabled {background:transparent;color:#536271;border-color:#1e2934;}")
        self.learn_button.clicked.connect(self.start_learning)
        self.clear_learning_button.clicked.connect(lambda: self._invalidate_learning("已清除学习，请重新学习"))
        learning_row.addWidget(self.learn_button)
        learning_row.addWidget(self.clear_learning_button)
        learning_row.addStretch(1)
        root.addLayout(learning_row)
        self.learning_status_label = QtWidgets.QLabel("尚未学习：当前使用保守寻压；学习结果自动保存，重启核验后恢复快速接近")
        self.learning_status_label.setWordWrap(True)
        self.learning_status_label.setStyleSheet("color:#9baec5;")
        root.addWidget(self.learning_status_label)
        hint = QtWidgets.QLabel(
            "下压方向沿用位置标定页：位置减小为下压。控制使用压力计 N 值，不依赖原位置标定表或传感器状态。"
            "调节时压力反馈超过0.5秒未更新立即停止；每次运动完成并等待后，使用至少3帧新数据调节。"
            "运动中压力进入目标范围会提前停止，等待稳定后再确认。"
            "释放按独立速度一次回到释放点，不因压力归零、负值或压力断流提前停止；到位后检查压力。")
        hint.setWordWrap(True)
        root.addWidget(hint)
        buttons = QtWidgets.QHBoxLayout()
        self.capture_button = QtWidgets.QPushButton("设置当前位置为释放点")
        self.start_button = QtWidgets.QPushButton("开始闭环保持")
        self.release_button = QtWidgets.QPushButton("停止并释放")
        for button, color, hover in (
            (self.capture_button, "#2563eb", "#3b82f6"),
            (self.start_button, "#15803d", "#16a34a"),
            (self.release_button, "#b45309", "#d97706"),
        ):
            button.setStyleSheet(
                f"QPushButton {{background-color:{color};color:white;font-weight:700;"
                "border:1px solid transparent;border-radius:6px;padding:8px 12px;}"
                f"QPushButton:hover {{background-color:{hover};}}"
                "QPushButton:disabled {background-color:#303945;color:#8995a5;}")
            buttons.addWidget(button)
        self.capture_button.clicked.connect(self._capture)
        self.start_button.clicked.connect(self.start_control)
        self.release_button.clicked.connect(self.stop_and_release)
        root.addLayout(buttons)
        self.status_label = QtWidgets.QLabel("未开始")
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)
        self.summary_label = QtWidgets.QLabel("保持统计：--")
        self.summary_label.setWordWrap(True)
        root.addWidget(self.summary_label)
        self.plot_panel = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(self.plot_panel)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        plot_layout.setSpacing(10)
        self.plot = pg.PlotWidget()
        self.plot.setMinimumHeight(180)
        self.plot.setLabel("left", "压力", units="N")
        self.plot.setLabel("bottom", "观察时间", units="s")
        self.plot.showGrid(x=True, y=True, alpha=.25)
        self.plot.addLegend()
        self.force_curve = self.plot.plot(pen=pg.mkPen("#57d5ff", width=2), name="实测压力")
        self.target_curve = self.plot.plot(pen=pg.mkPen("#ffc857", width=1), name="目标压力")
        self.band = pg.LinearRegionItem(orientation="horizontal", movable=False,
                                      brush=pg.mkBrush(90, 210, 130, 30))
        self.plot.addItem(self.band)
        self.position_plot = pg.PlotWidget()
        self.position_plot.setMinimumHeight(160)
        self.position_plot.setLabel("left", "电机位置", units="step")
        self.position_plot.setLabel("bottom", "观察时间", units="s")
        self.position_plot.setXLink(self.plot)
        self.position_curve = self.position_plot.plot(pen=pg.mkPen("#bda6ff", width=2))
        plot_layout.addWidget(self.plot, 2)
        plot_layout.addWidget(self.position_plot, 1)
        root.addWidget(self.plot_panel, 1)
        self.log_label = QtWidgets.QLabel("每次运行自动保存压力、误差、位置和动作记录 CSV。")
        self.log_label.setWordWrap(True)
        root.addWidget(self.log_label)

    def _settings_identity(self):
        return dict(self.profile.metadata(), namespace=self.profile.namespace)

    def _settings_status(self, text, error=False):
        self.settings_status_label.setText(text)
        self.settings_status_label.setStyleSheet("color:#ff7b72;" if error else "color:#9baec5;")

    def _load_settings(self):
        """逐项核验本机型配置；坏文件不覆盖，非法字段保留默认值。"""
        try:
            document = json.loads(self._settings_path.read_text(encoding="utf-8-sig"))
            if not isinstance(document, dict) or document.get("schema_version") not in (1, 2):
                raise ValueError("配置格式或版本不受支持")
            if document.get("profile") != self._settings_identity():
                raise ValueError("电机型号、地址或每圈步数与配置不匹配")
            values = document.get("values")
            if not isinstance(values, dict):
                raise ValueError("配置缺少参数对象")
            migrated = document["schema_version"] == 1
            if migrated:
                # 旧配置的误差单位为N，必须按旧目标换算，不能直接当作百分比读取。
                values = dict(values)
                target = values.get("target_n", FeedbackSettings().target_n)
                tolerance = values.get("tolerance_n", FeedbackSettings().tolerance_n)
                valid = (not isinstance(target, bool) and isinstance(target, (int, float))
                         and .1 <= target <= 30 and not isinstance(tolerance, bool)
                         and isinstance(tolerance, (int, float)) and .01 <= tolerance <= 2)
                values["tolerance_percent"] = tolerance / target * 100 if valid else None
            rejected = []
            for name, spin in self.inputs.items():
                if name not in values:
                    continue
                value = values[name]
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not spin.minimum() <= value <= spin.maximum() or not math.isfinite(value)
                        or isinstance(spin, QtWidgets.QSpinBox) and int(value) != value):
                    rejected.append(name)
                    continue
                spin.setValue(int(value) if isinstance(spin, QtWidgets.QSpinBox) else value)
            if rejected:
                self._settings_status("配置已读取；部分参数无效，使用默认值：" + "、".join(rejected), True)
            elif migrated:
                self._settings_dirty = True
                self._settings_status("旧配置误差已换算为目标压力的百分比，正在自动保存…")
            else:
                self._settings_status("已读取保存的配置；修改后自动保存")
        except FileNotFoundError:
            self._settings_status("首次使用默认配置；修改后自动保存")
        except (OSError, ValueError, TypeError) as exc:
            self._settings_status(f"读取配置失败，使用默认值，原文件保留：{exc}", True)

    def _schedule_settings_save(self, *_args):
        if self._loading_settings:
            return
        self._settings_dirty = True
        self._settings_status("配置已修改，正在自动保存…")
        self._save_timer.start()

    def _save_settings(self):
        """先写临时文件再原子替换，失败保留旧配置并允许后续重试。"""
        self._save_timer.stop()
        if self._loading_settings or not self._settings_dirty:
            return
        temporary = None
        try:
            self._settings_path.parent.mkdir(parents=True, exist_ok=True)
            document = dict(schema_version=2, profile=self._settings_identity(),
                            values={name: spin.value() for name, spin in self.inputs.items()})
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self._settings_path.parent,
                prefix=self._settings_path.stem + "_", suffix=".tmp", delete=False,
            ) as handle:
                temporary = Path(handle.name)
                json.dump(document, handle, ensure_ascii=False, indent=2, allow_nan=False)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self._settings_path)
            temporary = None
            self._settings_dirty = False
            self._settings_status("配置已自动保存，下次启动自动读取")
        except (OSError, ValueError) as exc:
            self._settings_status(f"配置保存失败，尚未保存本次修改：{exc}", True)
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass

    def _flush_settings(self):
        # 提交仍有焦点的输入文本，防止点击开始或关闭窗口时漏存最后一项。
        for spin in self.inputs.values():
            spin.interpretText()
        self._save_settings()

    def take_plot_panel(self):
        """把原有曲线交给主窗口左侧展示，保留曲线对象、数据和刷新连接。"""
        if self.layout().indexOf(self.plot_panel) >= 0:
            self.layout().removeWidget(self.plot_panel)
            self.layout().addStretch(1)
        return self.plot_panel

    @Slot(object)
    def set_pressure_sample(self, sample):
        self.samples.update(sample)

    def control_settings_snapshot(self, commit=False):
        """在界面线程读取同一份控制参数，供监控任务冻结使用。"""
        if commit:
            self._flush_settings()
        values = {name: spin.value() for name, spin in self.inputs.items()}
        values.pop("plot_window_s")
        return values

    def _parameters(self, release_only=False, target_override=None):
        values = self.control_settings_snapshot()
        if target_override is not None:
            values["target_n"] = target_override
        values["tolerance_n"] = pressure_tolerance_n(values["target_n"], values.pop("tolerance_percent"))
        settings = FeedbackSettings(**values)
        settings.validate(self.profile, release_only)
        return settings

    def _status(self, message):
        self.status_label.setText(message)
        self.status_message.emit(message)

    def _capture(self):
        try:
            force, _, _ = self.samples.read(time.monotonic())
            if abs(force) > .1:
                raise FeedbackControlError("设置释放点前请确保压头脱离且压力在 ±0.10 N 内")
            if self._running or self.control.is_operation_busy() or not self.control.is_can_connected():
                raise FeedbackControlError("电机未连接或正在操作")
            self.control.capture_release_position()
            self._status("已记录释放点；与位置标定页共用此坐标")
        except FeedbackControlError as exc:
            self._status(str(exc))

    def start_control(self):
        self._start(False)

    def start_learning(self):
        self._start(False, learning=True)

    def _load_learning(self):
        try:
            table, reason = load_learning(self._learning_path, self.output_dir, self.profile)
            if table is not None:
                self._learned_map = table
                self.learning_status_label.setToolTip(f"已保存释放点：{table.release_position} step\n学习存档：{self._learning_path}")
                self._suspend_learning("已读取上次学习表，等待连接及原释放点卸载核验")
            elif reason:
                self.learning_status_label.setText(reason)
        except (OSError, ValueError, TypeError, KeyError, OverflowError, FeedbackControlError) as exc:
            self._learning_storage_error = f"学习存档读取失败，未启用快速接近：{exc}"
            self.learning_status_label.setText(self._learning_storage_error)

    def _save_learning(self, reason=''):
        try:
            save_learning(self._learning_path, self.profile, self._learned_map, reason)
            self._learning_storage_error = ''
        except (OSError, ValueError, TypeError, FeedbackControlError) as exc:
            self._learning_storage_error = f"学习存档保存失败，本次变更尚未保存：{exc}"

    def _invalidate_learning(self, reason):
        self._learning_generation += 1
        self._learned_map = None
        self._learning_suspended_reason = None
        self._learning_suspended_at = None
        self._learning_recovery_samples.clear()
        self._save_learning(reason)
        self.learning_status_label.setText(reason)
        self.learning_status_label.setToolTip("")
        self.clear_learning_button.setEnabled(False)
        self._update_learning_notice()

    def _suspend_learning(self, reason):
        """运行异常保留完整学习数据，暂停使用并记录恢复检查起点。"""
        if self._learned_map is None:
            return
        self._learning_suspended_reason = str(reason)
        self._learning_suspended_at = time.monotonic()
        self._learning_recovery_samples.clear()
        self.learning_status_label.setText(
            f"已完成学习（末点 {self._learned_map.points[-1][0]:.2f} N），学习表保留；快速接近暂时停用：{reason}")
        self._update_learning_notice()

    @Slot(object)
    def handle_learning_operation_result(self, result):
        """两个操作页面统一处理异常；用户取消但未确认停止时也需要恢复检查。"""
        if (not result.get("ok")
                and (not result.get("cancelled") or not result.get("stop_confirmed"))):
            self._suspend_learning(result.get("message") or result.get("stop_reason") or "操作异常")

    def learning_recovery_pending(self):
        return self._learning_suspended_reason is not None

    def _try_resume_learning(self):
        """重启或异常后通过连续新反馈恢复资格；仅确认参考，不发送运动命令。"""
        if not self.learning_recovery_pending() or self._learned_map is None:
            return
        try:
            if (self._running or self._pending_release or self.control.is_operation_busy()
                    or not self.control.is_can_connected()):
                raise ValueError("电机仍忙或连接无效")
            reference = self.control.release_position_snapshot()
            expected_release = self._learned_map.release_position
            if (reference.get("position_counts") is not None
                    and not self._learned_map.matches(self.profile, reference.get("position_counts"))):
                raise ValueError("释放参考与学习表不一致")
            now = time.monotonic()
            force, pressure_stamp, sequence = self.samples.read(now, allow_negative=True)
            feedback = self.control.latest_motor_feedback() or {}
            motor_stamp = float(feedback["sampled_monotonic"])
            position = float(feedback["position_counts"])
            speed = float(feedback["measured_speed_rpm"])
            current = float(feedback["phase_current_ma"])
            if (not all(math.isfinite(v) for v in (motor_stamp, position, speed, current))
                    or not 0 <= now-motor_stamp <= getattr(self.control, 'feedback_max_age_s', .5)
                    or min(pressure_stamp, motor_stamp) <= self._learning_suspended_at
                    or abs(force) > LOW_FORCE_ZERO_N+1e-9
                    or abs(force-self._learned_map.contact_reference_n) > .01+1e-9 or abs(speed) > 1
                    or int(feedback["running_state"]) not in (0, 1)
                    or not feedback.get("in_position", False)
                    or abs(current) > self.inputs["max_current_ma"].value()
                    or abs(position-expected_release) > self.profile.tolerance(RELEASE_REFERENCE_TOLERANCE_COUNTS)):
                raise ValueError("尚未确认停止、原释放位置和卸载")
        except (FeedbackControlError, KeyError, TypeError, ValueError, OverflowError):
            self._learning_recovery_samples.clear()
            return
        samples = self._learning_recovery_samples
        gap_limit = getattr(self.control, 'feedback_max_age_s', .5)
        if samples and (motor_stamp-samples[-1][1] > gap_limit or pressure_stamp-samples[-1][2] > gap_limit):
            samples.clear()
        if samples and (sequence <= samples[-1][0] or motor_stamp <= samples[-1][1]):
            return
        samples.append((sequence, motor_stamp, pressure_stamp, force))
        if max(s[3] for s in samples)-min(s[3] for s in samples) > .01+1e-9:
            samples.clear()
            return
        if len(samples) >= 3 and min(motor_stamp, pressure_stamp)-max(samples[0][1:3]) >= .2-1e-9:
            if not reference.get('confirmed'):
                restore = getattr(self.control, 'restore_release_position', None)
                if restore is None or not restore(expected_release):
                    samples.clear()
                    return
            self._learning_suspended_reason = None
            self._learning_suspended_at = None
            samples.clear()
            self.learning_status_label.setText(
                f"已确认停止、卸载及原释放点；已恢复{len(self._learned_map.points)}点学习表，无需重新学习（悬停查看位置）")
            self._update_learning_notice()

    def learning_notice_text(self):
        """两页共用学习有效性提示，不改变启动权限或自动执行学习。"""
        return self._learning_notice()[0]

    def begin_learning_task_notice(self, table):
        """仅登记自动标定实际持有的已核验表，供显示使用，不放行任何运动。"""
        self._learning_task_notice = (dict(table=table, phase='starting')
            if table is not None and table is self._learned_map and not self.learning_recovery_pending() else None)
        self._update_learning_notice()

    def update_learning_task_notice(self, phase):
        if self._learning_task_notice is not None:
            self._learning_task_notice['phase'] = phase
            self._update_learning_notice()

    def end_learning_task_notice(self):
        self._learning_task_notice = None
        self._update_learning_notice()

    def _task_feedback_is_fresh(self):
        """显示已恢复之前核对实时反馈；承压时不执行释放点核验或恢复启动权限。"""
        try:
            if not self.control.is_can_connected():
                return False
            now = time.monotonic()
            self.samples.read(now, allow_negative=True)
            sample = self.control.latest_motor_feedback() or {}
            stamp, position, speed, current = (float(sample[k]) for k in
                ('sampled_monotonic', 'position_counts', 'measured_speed_rpm', 'phase_current_ma'))
            return (all(math.isfinite(v) for v in (stamp, position, speed, current))
                    and 0 <= now-stamp <= .5 and abs(current) <= self.inputs['max_current_ma'].value()
                    and int(sample['running_state']) in (0, 1, 2))
        except (FeedbackControlError, KeyError, TypeError, ValueError, OverflowError):
            return False

    def _learning_notice(self):
        """区分本轮表的使用说明与新任务启动资格，提示不参与控制判定。"""
        if self._learning:
            return "正在学习压力位置，请等待学习完成并自动释放。", 'info'
        if self._learning_storage_error:
            return self._learning_storage_error, 'warning'
        if self.learning_recovery_pending():
            task = self._learning_task_notice
            reference = self.control.release_position_snapshot()
            if (task is not None and task['table'] is self._learned_map and reference.get('confirmed')
                    and self._learned_map.matches(self.profile, reference.get('position_counts'))):
                phase = task['phase']
                if phase == 'emergency':
                    return '自动标定已请求紧急停止；学习表保留，停止和释放状态请查看上方标定进度。', 'warning'
                if phase == 'stopping':
                    return '正在停止标定并释放；学习表保留，回位卸载后自动核验，无需重新学习。', 'info'
                if phase == 'storage_wait':
                    return '标定已释放，正在等待记录保存恢复；学习表已保留，无需重新学习。', 'warning'
                if phase in ('recovery', 'recovery_release', 'retry_wait'):
                    return ('自动标定正在恢复，学习表已保留，无需重新学习。\n'
                            '回位、等待及重试进度请看上方自动标定模块。'), 'warning'
                if phase != 'starting' and self._task_feedback_is_fresh():
                    return ('自动标定使用本轮启动时已核验的学习表，无需重新学习。\n'
                            '实时反馈已恢复；新任务的启动资格将在本轮结束、卸载后自动核验。'), 'info'
                return ('自动标定的实时反馈暂不可用，学习表已保留。\n'
                        '请查看上方标定恢复进度；反馈恢复后自动更新提示，无需重新学习。'), 'warning'
            return (f"学习表已保留（末点 {self._learned_map.points[-1][0]:.2f} N），等待新任务启动前核验\n"
                    f"原因：{self._learning_suspended_reason}\n"
                    f"停止在原释放点 {self._learned_map.release_position} step、卸载至±0.02 N且与学习底数一致后自动恢复，无需重新学习。"), 'warning'
        reference = self.control.release_position_snapshot()
        if (self._learned_map is not None and self.control.is_can_connected()
                and reference.get("confirmed")
                and self._learned_map.matches(self.profile, reference.get("position_counts"))):
            return "", 'info'
        return ("建议先学习：当前没有有效学习表\n"
                "请先在“压力反馈控制”页确认释放点，再点击“一键学习到 30 N”。"
                "直接下压将使用较慢的保守寻压，通常无法达到 2～3 秒。"), 'warning'

    def _update_learning_notice(self):
        text, level = self._learning_notice()
        if text != self._learning_notice_text or level != self._learning_notice_level:
            self._learning_notice_text = text
            self._learning_notice_level = level
            self.learning_notice_label.setStyleSheet(
                "color:#91dff9;background:#102635;border:1px solid #365b6f;"
                "border-radius:5px;padding:6px 10px;font-size:13px;font-weight:400;"
                if level == 'info' else
                "color:#ffe1a3;background:#3b2a10;border:2px solid #e7a83e;"
                "border-radius:7px;padding:10px 12px;font-size:15px;font-weight:700;")
            self.learning_notice_label.setText(text)
            if self.learning_recovery_pending():
                self.learning_status_label.setText(text)
            self.learning_notice_label.setVisible(bool(text))
            self.learning_notice_changed.emit(text)

    def _release_reference_changed(self, *_args):
        reference = self.control.release_position_snapshot()
        if (self._learned_map is not None and not reference.get('confirmed')
                and reference.get('position_counts') in (None, self._learned_map.release_position)):
            self._suspend_learning("释放点暂未确认，保留学习表等待新反馈核验")
            self._refresh()
            return
        if (self._learned_map is not None and reference.get("confirmed")
                and self._learned_map.matches(self.profile, reference.get("position_counts"))):
            self._refresh()
            return
        self._invalidate_learning("释放参考已改变；请重新学习")
        if self._running or self._pending_release:
            self.stop()
        self._refresh()

    def _control_state_changed(self, connected, *_args):
        if not connected and (self._learned_map is not None or self._learning):
            if self._learning:
                self._invalidate_learning("学习期间连接异常，学习未完成；请检查后重新学习")
            else:
                self._suspend_learning("连接或实时反馈暂时无效，等待恢复检查")
        self._refresh()

    def start_target(self, target):
        """快捷设置目标并启动，复用原校验和互锁，不自动提高保护压力。"""
        if self._running or self._pending_release or self.control.is_operation_busy():
            return
        self._flush_settings()
        self.inputs["target_n"].setValue(target)
        self.start_control()

    def start_release(self):
        self._start(True)

    def stop_and_release(self):
        # 先停止并确认，再交还唯一电机线程启动释放；重复点击可以取消待释放请求。
        if self._pending_release or self._release_after_stop or (self._running and self._releasing):
            self.stop()
        elif self._running:
            self._release_after_stop = True
            self.control.request_external_stop()
            self._status("正在停止调节；确认停止后自动回释放点，再次点击可取消释放")
            self._refresh()
        else:
            self.start_release()

    def learned_map_snapshot(self):
        """两页共享经当前反馈核验的不可变学习表，待核验期间不提供运动参考。"""
        reference = self.control.release_position_snapshot()
        if (self.learning_recovery_pending() or not self.control.is_can_connected()
                or not reference.get("confirmed") or self._learned_map is None
                or not self._learned_map.matches(self.profile, reference.get("position_counts"))):
            return None
        return self._learned_map

    def _start(self, release_only, learning=False):
        if self._running or self._pending_release or self.control.is_operation_busy():
            return
        self._flush_settings()
        try:
            settings = self._parameters(release_only, target_override=30 if learning else None)
            if not self.control.is_can_connected():
                raise FeedbackControlError("电机未连接")
            release = self.control.release_position_snapshot()
            if not release.get("confirmed"):
                raise FeedbackControlError("请先在无接触位置设置释放点")
            if not release_only and self.learning_recovery_pending():
                raise FeedbackControlError("学习表已保留；请先确认停止并回原释放点卸载，恢复检查通过后再下压")
            if not release_only:
                force, _, _ = self.samples.read(time.monotonic())
                zero_limit = LOW_FORCE_ZERO_N if learning or settings.target_n <= LOW_FORCE_MAX_N else .1
                if abs(force) > zero_limit:
                    raise FeedbackControlError(f"请先释放并确认压力在 ±{zero_limit:.2f} N 内")
        except (FeedbackControlError, ValueError) as exc:
            self._status(str(exc))
            return
        self._running = True
        self._releasing = release_only
        self._learning = learning
        if learning:
            self._invalidate_learning("正在学习：一次加载至30 N，完成后自动释放")
        learning_generation = self._learning_generation
        learned_map = None if learning or release_only else self.learned_map_snapshot()
        self._settings = settings
        self._plot_target_n = settings.target_n
        self._plot_tolerance_n = settings.tolerance_n
        self.band.setRegion((settings.target_n-settings.tolerance_n, settings.target_n+settings.tolerance_n))
        self.summary_label.setText("保持统计：采集中")
        self._result = None

        def task(motor, cancel):
            result = dict(ok=False, message="操作未完成")
            try:
                self.output_dir.mkdir(parents=True, exist_ok=True)
                path = self.output_dir / (datetime.now().strftime("feedback_%Y%m%d_%H%M%S_%f") + ".csv")
                with path.open("x", encoding="utf-8-sig", newline="") as handle:
                    writer = None
                    def record(row):
                        nonlocal writer
                        row = dict(timestamp=datetime.now().astimezone().isoformat(timespec="milliseconds"), **row)
                        row.update(motor_model=self.profile.model, steps_per_turn=self.profile.steps_per_turn,
                                   mode="learn" if learning else "release" if release_only else "force_hold",
                                   control_strategy=CONTROL_STRATEGY,
                                   release_position_steps=release["position_counts"],
                                   release_position_tolerance_steps=self.profile.tolerance(RELEASE_REFERENCE_TOLERANCE_COUNTS),
                                   force_limit_n=settings.force_limit_n, max_step=settings.max_step,
                                   tolerance_percent=(row["tolerance_n"] / row["target_n"] * 100
                                                      if row["target_n"] > 1e-9 else None),
                                   max_travel=settings.max_travel, speed=settings.speed,
                                   approach_speed=settings.approach_speed,
                                   release_speed=settings.release_speed,
                                   speed_unit=self.profile.speed_unit, acceleration=settings.acceleration,
                                   max_current_ma=settings.max_current_ma, settle_s=settings.settle_s,
                                   stable_s=settings.stable_s, hold_s=settings.hold_s,
                                   acquire_timeout_s=settings.acquire_timeout_s)
                        if writer is None:
                            writer = csv.DictWriter(handle, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                        handle.flush()
                        self.progress.emit(row)
                    controller = PressureFeedbackController(
                        motor, self.profile, self.samples, cancel, settings, release["position_counts"],
                        on_record=record, on_motor=self.control.publish_motor_feedback, learned_map=learned_map)
                    result = controller.run(release_only, learning=learning)
                    # 结果也写入CSV，保留失败和用户停止的原因。
                    if writer is None:
                        writer = csv.DictWriter(handle, fieldnames=("timestamp", "phase"))
                        writer.writeheader()
                    final_row = dict(timestamp=datetime.now().astimezone().isoformat(), phase=result["message"])
                    if "stable_arrival_s" in writer.fieldnames:
                        final_row.update(first_in_band_s=result.get("first_in_band_s"),
                                         stable_arrival_s=result.get("stable_arrival_s"))
                    writer.writerow(final_row)
                result["csv_path"] = str(path)
                if result.get("ok") and result.get("stop_confirmed") and result.get("learned_map") is not None:
                    # 归档保留完整原始结果，正式恢复资格在主线程核验代次和参考后另行保存。
                    map_path = path.with_suffix(".learning.json")
                    with map_path.open("x", encoding="utf-8") as handle:
                        json.dump(dict(schema_version=3, created_at=datetime.now().astimezone().isoformat(),
                                       **asdict(result["learned_map"])), handle, ensure_ascii=False, indent=2)
                    result["learning_path"] = str(map_path)
            except Exception as exc:
                result = dict(ok=False, message=f"记录或控制失败：{exc}")
                try:
                    motor.stop()
                except Exception as stop_exc:
                    result["message"] += f"；停止未确认：{stop_exc}"
            result["learning_generation"] = learning_generation
            self.finished.emit(result)

        accepted, reason = self.control.start_external_operation(task, "学习压力位置" if learning else "压力反馈控制" if not release_only else "释放")
        if not accepted:
            self._running = False
            self._learning = False
            self._status(reason)
        else:
            self._status("正在学习到30 N，完成后自动释放" if learning else "正在释放" if release_only else "正在寻压，达到容差并持续确认后开始保持计时")
        self._refresh()

    @Slot(object)
    def _on_progress(self, record):
        if self._learning and record["phase"] == "释放":
            self._releasing = True
        self._plot_target_n = record["target_n"]
        self._plot_tolerance_n = record.get("tolerance_n", self._plot_tolerance_n)
        self.status_label.setText(
            f"{record['phase']} · {record['elapsed_s']:.1f} 秒 · "
            f"压力 {record['force_n']:.2f} N · 误差 {record['error_n']:+.2f} N · "
            f"下压行程 {record['travel_steps']:.0f} step")
        if record.get("hold_remaining_s") is not None:
            self.status_label.setText(self.status_label.text() + f" · 保持剩余 {record['hold_remaining_s']:.1f} 秒")
        if record.get("stable_arrival_s") is not None:
            self.status_label.setText(self.status_label.text() + f" · 稳定到位 {record['stable_arrival_s']:.2f} 秒")

    def set_external_plot_target(self, target):
        """共享曲线显示外部位置测试目标；不修改旧闭环参数或运行状态。"""
        self._external_plot_target = dict(target) if target is not None else None

    def _append_live_point(self, now, force, position, target, tolerance):
        """持续观察共享反馈，最多10 Hz；保留最近6000点，失效通道用断线表示。"""
        if not self._monitoring or now-self._last_plot_at < 0.1:
            return
        if self._plot_origin is None:
            if not (math.isfinite(force) or math.isfinite(position)):
                return
            self._plot_origin = now
        self._last_plot_at = now
        self._points.append(dict(elapsed_s=now-self._plot_origin, force_n=force,
                                 position_steps=position, target_n=target))
        window = self.inputs["plot_window_s"].value()
        end = max(window, now-self._plot_origin)
        start = end-window
        visible = [p for p in self._points if start <= p["elapsed_s"] <= end]
        xs = [p["elapsed_s"] for p in visible]
        self.force_curve.setData(xs, [p["force_n"] for p in visible], connect="finite")
        self.target_curve.setData(xs, [p["target_n"] for p in visible])
        self.position_curve.setData(xs, [p["position_steps"] for p in visible], connect="finite")
        self.plot.setXRange(start, end, padding=0)
        self.position_plot.setXRange(start, end, padding=0)
        self.band.setRegion((target-tolerance, target+tolerance))

    @Slot(object)
    def _on_finished(self, result):
        self._running = False
        was_learning = self._learning
        self._learning = False
        table = result.get("learned_map")
        reference = self.control.release_position_snapshot()
        if (was_learning and result.get("ok") and result.get("stop_confirmed")
                and isinstance(table, LearnedPressureMap) and reference.get("confirmed")
                and self.control.is_can_connected()
                and result.get("learning_generation") == self._learning_generation
                and table.matches(self.profile, reference["position_counts"])):
            self._learned_map = table
            self._save_learning()
            contact_text = (f"底数{table.contact_reference_n:+.2f} N，接触点读数{table.contact_force_n:.2f} N；"
                            if table.contact_travel is not None else "无接触点；")
            self.learning_status_label.setText(f"已学习{len(table.points)}点至约30 N；{contact_text}自动接近再微调（悬停查看位置）")
            estimates = []
            if table.contact_travel is not None:
                estimates.append(f"接触点 {table.contact_force_n:.2f} N ≈ 位置{table.release_position-table.contact_travel:.0f} step")
                estimates.append(f"接触增量 {table.contact_force_n-table.contact_reference_n:.2f} N；最后确认底数{table.contact_reference_n:+.2f} N位置 ≈ {table.release_position-table.contact_zero_travel:.0f} step；高速在该点前预留{self.profile.tolerance(RELEASE_REFERENCE_TOLERANCE_COUNTS)*2} step")
            for target in self.quick_target_buttons:
                travel = table.travel_at(target)
                if target == 30 and travel is None:
                    estimates.append(f"30 N 附近 ≈ 位置{table.release_position-table.points[-1][1]:.0f} step（实测{table.points[-1][0]:.2f} N）")
                if travel is not None:
                    estimates.append(f"{target} N ≈ 位置{table.release_position-travel:.0f} step")
            self.learning_status_label.setToolTip("加载位置估计（不是直接运动终点）：\n" + "\n".join(estimates))
        elif was_learning:
            self._invalidate_learning("学习未完成或参考已失效，未启用学习结果")
        else:
            self.handle_learning_operation_result(result)
        requested_release = self._release_after_stop
        self._release_after_stop = False
        self._pending_release = bool(requested_release and result.get("stop_confirmed")
                                     and (result.get("ok") or result.get("cancelled")))
        self._result = result
        self._status(result["message"])
        rmse = result.get("hold_rmse_n")
        if rmse is not None:
            mean = result.get("hold_mean_error_n")
            mean_text = f"平均误差 {mean:+.3f} N · " if mean is not None else ""
            self.summary_label.setText(
                f"保持段{mean_text}误差 RMS {rmse:.3f} N · 容差内样本 {result['hold_in_band_fraction']:.1%} · "
                f"全程峰值 {result['peak_n']:.2f} N · 调整 {result['move_count']} 次")
        else:
            self.summary_label.setText("未取得恒力保持段统计")
        if result.get("first_in_band_s") is not None:
            timing = f"首次进入容差 {result['first_in_band_s']:.2f} 秒"
            timing += (f" · 稳定到位 {result['stable_arrival_s']:.2f} 秒" if result.get("stable_arrival_s") is not None
                       else " · 尚未确认稳定到位")
            self.summary_label.setText(timing + "\n" + self.summary_label.text())
        if result.get("csv_path"):
            self.log_label.setText("本次记录：" + result["csv_path"])
        if result.get("learning_path"):
            self.log_label.setText(self.log_label.text() + "\n学习数据：" + result["learning_path"])
        if requested_release:
            self._status("已确认停止，等待电机线程空闲后释放" if self._pending_release else
                         result["message"] + "；未自动释放，请检查后重试")
        self._refresh()

    def _refresh(self, *_args):
        # 构建表单时数值变更可能早于按钮创建。
        if not hasattr(self, "start_button"):
            return
        connected = self.control.is_can_connected()
        self._try_resume_learning()
        self._update_learning_notice()
        worker_busy = self.control.is_operation_busy()
        busy = self._running or worker_busy or self._pending_release
        release = self.control.release_position_snapshot()
        confirmed = release.get("confirmed", False)
        self.release_label.setText(
            f"电机 {self.profile.model} · 释放点 {release['position_counts']} step"
            f"（回位允许±{self.profile.tolerance(RELEASE_REFERENCE_TOLERANCE_COUNTS)} step）"
            if confirmed else f"电机 {self.profile.model} · 尚未确认释放点")
        now = time.monotonic()
        try:
            force, _, _ = self.samples.read(now, allow_negative=True)
            valid, zero = True, abs(force) <= .1
            text = f"压力计 {force:.2f} N"
        except FeedbackControlError:
            valid, zero, text = False, False, "压力计 -- N（无有效新数据）"
            force = math.nan
        feedback = self.control.latest_motor_feedback() if connected else None
        position = math.nan
        if feedback:
            try:
                age = now-float(feedback["sampled_monotonic"])
                measured_position = float(feedback["position_counts"])
                if 0 <= age <= getattr(self.control, 'feedback_max_age_s', .5) and math.isfinite(measured_position):
                    position = measured_position
            except (KeyError, TypeError, ValueError, OverflowError):
                pass
        text += f" · 位置 {position:g} step" if math.isfinite(position) else " · 位置 -- step"
        self.live_label.setText(text)
        self.live_label.setStyleSheet(f"font-size:22px;font-weight:700;color:{'#57d5ff' if valid else '#999999'};")
        self.capture_button.setEnabled(connected and not busy and zero)
        self.start_button.setEnabled(connected and not busy and confirmed and zero and not self.learning_recovery_pending())
        for button in self.quick_target_buttons.values():
            button.setEnabled(self.start_button.isEnabled())
        self.learn_button.setEnabled(self.start_button.isEnabled())
        self.clear_learning_button.setEnabled(not busy and self._learned_map is not None)
        self.release_button.setText("停止释放" if self._running and self._releasing else
                                    "取消待释放" if self._release_after_stop or self._pending_release else
                                    "停止并释放")
        self.release_button.setEnabled(self._running or self._pending_release or
                                       (connected and not busy and confirmed))
        for name, spin in self.inputs.items():
            spin.setEnabled(name == "plot_window_s" or not self._running and not self._pending_release)
        target = self._plot_target_n if self._running else self.inputs["target_n"].value()
        tolerance = self._plot_tolerance_n if self._running else pressure_tolerance_n(target, self.inputs["tolerance_percent"].value())
        if not self._running and self._external_plot_target is not None:
            target = self._external_plot_target['target_n']
            tolerance = self._external_plot_target['tolerance_n']
        self._append_live_point(now, force, position, target, tolerance)
        if self._pending_release and not worker_busy:
            self._pending_release = False
            # 释放仍重新检查连接、释放参考及新压力，失败时不保留延迟重试。
            self.start_release()

    def _coordinates_invalidated(self):
        self._invalidate_learning("电机坐标已失效；请重新学习")
        if self._running or self._pending_release:
            self.stop()
        self._refresh()

    def stop(self):
        pending = self._release_after_stop or self._pending_release
        self._release_after_stop = False
        self._pending_release = False
        if self._running:
            self.control.request_external_stop()
            self._status("已请求停止；等待工作线程确认，未自动卸载")
        elif pending:
            self._status("已取消待释放请求，保持停止")
        self._refresh()

    def is_running(self):
        return self._running

    def shutdown(self):
        self._monitoring = False
        self._flush_settings()
        self.timer.stop()
        self.stop()

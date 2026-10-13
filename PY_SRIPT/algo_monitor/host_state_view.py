"""上位机基线实验参数与初始化概况。"""
import math

from qt_compat import QtWidgets, QtCore, Signal
from host_state_machine import PARAMETERS, HostParameters, HOST_STATE_NAMES


# 按用途分组显示；计算保护项仅保存在 JSON，不创建可编辑控件。
JSON_ONLY_PARAMETERS = frozenset(('normalization_floor', 'gap_seconds', 'channel_repress'))
PARAMETER_GROUPS = (
    ('输入与初始化参数', ('input_alpha', 'signal_alpha', 'initial_seconds', 'window')),
    ('idle激活的条件参数', ('idle_alpha', 'activate_threshold', 'activate_frames')),
    ('负向下冲与残余恢复参数', ('negative_threshold', 'negative_frames', 'negative_rebase_seconds')),
    ('负向下冲再次按压参数', ('negative_press_margin', 'negative_press_rate', 'negative_press_frames')),
    ('激活后释放条件参数', ('drop_rate', 'drop_frames', 'release_threshold', 'release_frames')),
    ('slowback到recover条件参数', ('slowback_frames', 'slowback_tolerance_percent', 'slowback_close_frames')),
    ('recover到idle的条件参数', ('recover_alpha', 'recover_tolerance', 'stable_rate', 'recover_frames')),
    ('恢复阶段再次激活条件参数', ('reload_rate', 'reload_threshold', 'reload_reference_margin', 'reload_frames')),
)


def format_activation_reason(evidence):
    if not evidence:
        return '--', '本轮初始化后尚未激活。下一次激活时记录原因，退出 activate 后保留。'
    state = HOST_STATE_NAMES[evidence['from_state']]
    frames = evidence['confirmed_frames']
    summary = f'{state}：' + ('差值触发' if evidence['reason'] == 'idle_threshold' else '重新加载') + f'（{frames}帧）'
    details = [f"第 {evidence['activation_number']} 次激活：{state} → activate",
               f"input {evidence['input_val']:.3f}，baseline {evidence['baseline']:.3f}",
               f"差值 {evidence['difference']:.3f}，激活阈值 {evidence['activate_threshold']:.3f}（判定通过）"]
    if evidence['reason'] == 'negative_recovery_press':
        summary = f'{state}：超过原零点（{frames}帧）'
        if 'negative_source' in evidence:
            source = '空闲负向扰动' if evidence['negative_source'] == 'idle' else '释放下冲'
            details.append(f'本轮负向来源：{source}；后续恢复继续保留原零点保护。')
        details += [f"受力前参考 {evidence['negative_reference']:.3f}，超出参考 {evidence['reference_difference']:.3f}"]
        if 'negative_press_rate' in evidence:
            details += [f"超出参考门限 {evidence['negative_press_margin']:.3f}，同时须超过普通激活阈值",
                        f"上升速度 {evidence['pressure_rate']:.3f} %/s，门限 {evidence['negative_press_rate']:.3f} %/s（判定通过）"]
    if evidence['reason'] == 'reload':
        details += [f"阶段谷值 {evidence['trough']:.3f}，较谷值增加 {evidence['increase']:.3f}",
                    f"增加量阈值 {evidence['reload_threshold']:.3f}（判定通过）",
                    f"上升速度 {evidence['pressure_rate']:.3f} %/s，门限 {evidence['reload_rate']:.3f} %/s（判定通过）"]
        if 'release_reference' in evidence:
            margin = evidence['reload_reference_margin']
            details += [f"卸载前参考基线 {evidence['release_reference']:.3f}，超出参考 {evidence['reference_difference']:.3f}",
                        f"超出参考门限 {margin:.3f}（判定通过）" if margin > 0 else '卸载前基线保护已关闭']
    if evidence.get('parameter_source') == 'channel_override':
        details.append(f"通道独立增量门槛 {evidence['repress_margin']:.3f} count")
    details.append(f"连续确认 {frames}/{evidence['required_frames']} 帧；以上数值固定为触发当帧。")
    return summary, '\n'.join(details)


class CollapsibleParameterGroup(QtWidgets.QWidget):
    """折叠只改变可见性，保留编辑值、校验和保存行为。"""

    def __init__(self, title):
        super().__init__()
        self._title = title
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.toggle = QtWidgets.QToolButton()
        self.toggle.setText(title)
        self.toggle.setCheckable(True)
        self.toggle.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed)
        self.toggle.setToolTip('点击展开或收起参数；收起不会丢失编辑值。')
        layout.addWidget(self.toggle)
        self.body = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(self.body)
        self.form.setContentsMargins(12, 4, 4, 8)
        self.form.setVerticalSpacing(8)
        layout.addWidget(self.body)
        self.toggle.toggled.connect(self.set_expanded)
        self.set_expanded(False)

    def title(self):
        return self._title

    def set_expanded(self, expanded):
        self.toggle.setChecked(expanded)
        self.toggle.setArrowType(QtCore.Qt.ArrowType.DownArrow if expanded else QtCore.Qt.ArrowType.RightArrow)
        self.body.setVisible(expanded)


class HostStatePanel(QtWidgets.QScrollArea):
    settings_changed = Signal()
    waveform_requested = Signal()
    reset_requested = Signal()
    observation_changed = Signal(int)
    signal_display_changed = Signal(bool)

    def __init__(self, engine, save_callback=None):
        super().__init__()
        self.engine = engine
        self.save_callback = save_callback
        self.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        self.setWidget(content)
        root_layout = QtWidgets.QVBoxLayout(content)
        self.config_tabs = QtWidgets.QTabWidget()
        root_layout.addWidget(self.config_tabs)
        common_page = QtWidgets.QScrollArea()
        common_page.setWidgetResizable(True)
        common_content = QtWidgets.QWidget()
        common_page.setWidget(common_content)
        layout = QtWidgets.QVBoxLayout(common_content)
        self.config_tabs.addTab(common_page, '公共参数')
        independent_page = QtWidgets.QWidget()
        self.config_tabs.addTab(independent_page, '通道独立配置')
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        observation_row = QtWidgets.QHBoxLayout()
        observation_row.addWidget(QtWidgets.QLabel('实验观察通道'))
        self.observation_channel = QtWidgets.QComboBox()
        for channel in range(20):
            self.observation_channel.addItem(f'CH{channel}', channel)
        self.observation_channel.setToolTip('切换 input、baseline、Signal、压力变化速度、状态图和背景；不重置算法。')
        self.observation_channel.currentIndexChanged.connect(self.observation_changed.emit)
        observation_row.addWidget(self.observation_channel)
        observation_row.addStretch(1)
        layout.addLayout(observation_row)
        buttons = QtWidgets.QHBoxLayout()
        self.apply_button = QtWidgets.QPushButton('确认生效并保存')
        self.apply_button.setToolTip('校验并立即写入配置文件；保存成功后应用参数并重新初始化，请保持释放。')
        self.reset_button = QtWidgets.QPushButton('释放后重新初始化')
        self.waveform_button = QtWidgets.QPushButton('查看实验波形')
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.reset_button)
        buttons.addWidget(self.waveform_button)
        layout.addLayout(buttons)
        self.message = QtWidgets.QLabel('修改参数后点击“确认生效并保存”；确认成功后请保持释放完成初始化。')
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.signal_filtered_box = QtWidgets.QCheckBox('显示低通后的上位机 Signal（关闭：显示低通前）')
        self.signal_filtered_box.setToolTip('即时切换上位机 Signal 的波形、监控表数值和柱状图，自动保存选择。\n'
            '不改变状态判定或触摸位置计算；低通后松开时显示值会缓降到零。\n'
            'alpha 越小越平滑、延迟越大；1 表示不平滑。修改 alpha 后点击“确认生效并保存”。')
        self.signal_filtered_box.toggled.connect(self.signal_display_changed.emit)
        layout.addWidget(self.signal_filtered_box)
        fold_buttons = QtWidgets.QHBoxLayout()
        self.expand_all_button = QtWidgets.QPushButton('全部展开公共参数')
        self.collapse_all_button = QtWidgets.QPushButton('全部收起公共参数')
        fold_buttons.addWidget(self.expand_all_button)
        fold_buttons.addWidget(self.collapse_all_button)
        fold_buttons.addStretch(1)
        layout.addLayout(fold_buttons)
        self.parameter_groups = []
        self.expand_all_button.clicked.connect(lambda: self._expand_parameters(True))
        self.collapse_all_button.clicked.connect(lambda: self._expand_parameters(False))
        self.inputs = {}
        definitions = {key: (label, default, low, high, decimals)
                       for key, label, default, low, high, decimals in PARAMETERS}
        for title, keys in PARAMETER_GROUPS:
            group = CollapsibleParameterGroup(title)
            self.parameter_groups.append(group)
            form = group.form
            for key in keys:
                label, default, low, high, decimals = definitions[key]
                widget = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
                if decimals:
                    widget.setDecimals(decimals)
                widget.setRange(low, high)
                widget.setValue(default)
                widget.setKeyboardTracking(False)
                widget.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
                widget.setMinimumWidth(130)
                if key == 'reload_reference_margin':
                    widget.setToolTip('上位机增益后的count。input还须超过卸载前保存的基线加此余量才能重新加载；0关闭额外保护。调高更能抑制回弹，也会延迟较轻的重新按压。')
                elif key == 'signal_alpha':
                    widget.setToolTip('每帧：低通后 Signal = 上帧低通值 × (1-alpha) + 本帧原 Signal × alpha。\n'
                        '范围大于0且不超过1；越小越平滑、延迟越大，1表示不平滑。只影响显示。\n'
                        '与其他实验参数一起确认生效并保存，应用时请保持释放完成重新初始化。')
                widget.valueChanged.connect(self._edited)
                self.inputs[key] = widget
                form.addRow(label, widget)
            layout.addWidget(group)
        layout.addStretch(1)
        layout = QtWidgets.QVBoxLayout(independent_page)
        layout.addWidget(QtWidgets.QLabel('通道独立设置 · 恢复阶段再次按压'))
        note = QtWidgets.QLabel('未勾选使用各入口公共参数；勾选仅覆盖恢复再按压增量和确认帧数。作用于空闲负向扰动、释放下冲及 slowback/recover，速度门槛仍用公共值，首次 idle 激活不变。')
        note.setWordWrap(True)
        layout.addWidget(note)
        batch = QtWidgets.QHBoxLayout()
        batch.addWidget(QtWidgets.QLabel('增量（count）'))
        self.batch_margin = QtWidgets.QDoubleSpinBox()
        self.batch_margin.setRange(0, 65535000)
        self.batch_margin.setDecimals(2)
        self.batch_margin.setValue(300)
        batch.addWidget(self.batch_margin)
        batch.addWidget(QtWidgets.QLabel('确认（帧）'))
        self.batch_duration = QtWidgets.QSpinBox()
        self.batch_duration.setRange(1, 10000)
        self.batch_duration.setValue(3)
        batch.addWidget(self.batch_duration)
        fill = QtWidgets.QPushButton('填写所选通道')
        inherit = QtWidgets.QPushButton('所选使用公共参数')
        batch.addWidget(fill)
        batch.addWidget(inherit)
        layout.addLayout(batch)
        self.channel_table = QtWidgets.QTableWidget(10, 8)
        self.channel_table.setHorizontalHeaderLabels(['通道', '参数来源', '增量（count）', '确认（帧）'] * 2)
        self.channel_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectItems)
        self.channel_table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.channel_table.setMinimumHeight(0)
        self.channel_table.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
        self.channel_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.channel_table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.channel_table.verticalHeader().setMinimumSectionSize(1)
        self.channel_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.channel_table.verticalHeader().hide()
        self.channel_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.channel_inputs = []
        self.channel_placeholders = []
        for channel in range(20):
            row, offset = channel % 10, 4 * (channel // 10)
            item = QtWidgets.QTableWidgetItem(f'CH{channel}')
            item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.channel_table.setItem(row, offset, item)
            box = QtWidgets.QCheckBox('使用公共参数')
            margin = QtWidgets.QDoubleSpinBox()
            margin.setRange(0, 65535000)
            margin.setDecimals(2)
            margin.setValue(self.inputs['reload_reference_margin'].value())
            margin.setToolTip('未启用时继承公共恢复余量；各入口按对应公共参数判定。')
            duration = QtWidgets.QSpinBox()
            duration.setRange(1, 10000)
            duration.setValue(3)
            duration.setToolTip('未启用时继承公共帧数；此处值不参与计算。')
            margin.setEnabled(False)
            duration.setEnabled(False)
            self.channel_table.setCellWidget(row, offset + 1, box)
            placeholders = []
            for col, widget in ((2, margin), (3, duration)):
                cell = QtWidgets.QWidget()
                cell_layout = QtWidgets.QVBoxLayout(cell)
                cell_layout.setContentsMargins(0, 0, 0, 0)
                placeholder = QtWidgets.QLabel('使用公共参数')
                placeholder.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                cell_layout.addWidget(placeholder)
                cell_layout.addWidget(widget)
                widget.hide()
                placeholders.append(placeholder)
                self.channel_table.setCellWidget(row, offset + col, cell)
            self.channel_placeholders.append(placeholders)
            box.toggled.connect(lambda enabled, ch=channel: self._toggle_channel(ch, enabled))
            margin.valueChanged.connect(self._edited)
            duration.valueChanged.connect(self._edited)
            self.channel_inputs.append((box, margin, duration))
        layout.addWidget(self.channel_table, 1)
        fill.clicked.connect(self._fill_channels)
        inherit.clicked.connect(self._inherit_channels)
        self.activation_detail = QtWidgets.QLabel()
        self.activation_detail.setWordWrap(True)
        layout.addWidget(self.activation_detail)
        self.bottom_apply_button = QtWidgets.QPushButton('确认生效并保存')
        self.bottom_apply_button.setToolTip(self.apply_button.toolTip())
        layout.addWidget(self.bottom_apply_button)
        self.bottom_message = QtWidgets.QLabel(self.message.text())
        self.bottom_message.setWordWrap(True)
        layout.addWidget(self.bottom_message)
        legend = QtWidgets.QLabel('状态背景：idle 绿 / activate 棕 / slowback 蓝 / recover 紫。\n'
            'Signal 显示低通按每个采样更新，默认 alpha=0.1；开关不重置算法。\n'
            'slowback 达到最长等待或相对基线差值连续达标时进入 recover；快速上升时暂停提前恢复。\n'
            '回弹尚未越过卸载前参考基线时，recover继续跟踪；真正上升候选才冻结基线。\n'
            '初始化取低通 input 的最大值；增益为 0 的通道停用。恢复需差值和速度同时达标。')
        legend.setWordWrap(True)
        layout.addWidget(legend)
        legend.hide()
        self.config_tabs.setCurrentIndex(1)
        self.apply_button.clicked.connect(self.apply)
        self.bottom_apply_button.clicked.connect(self.apply)
        self.reset_button.clicked.connect(self.reset_requested.emit)
        self.waveform_button.clicked.connect(self.waveform_requested.emit)
        self.refresh()

    def _expand_parameters(self, expanded):
        for group in self.parameter_groups:
            group.set_expanded(expanded)

    def _toggle_channel(self, channel, enabled):
        _, margin, duration = self.channel_inputs[channel]
        if enabled:
            margin.setValue(self.inputs['reload_reference_margin'].value())
            duration.setValue(self.inputs['reload_frames'].value())
        box = self.channel_inputs[channel][0]
        box.setText('覆盖公共参数' if enabled else '使用公共参数')
        for placeholder, widget in zip(self.channel_placeholders[channel], (margin, duration)):
            placeholder.setVisible(not enabled)
            widget.setVisible(enabled)
        margin.setEnabled(enabled)
        duration.setEnabled(enabled)
        margin.setToolTip('增益后的 count；未启用时按公共参数判定。')
        duration.setToolTip('连续有效采样帧数；任一加载条件失效即清零，未启用时沿用各入口公共帧数。')
        self._edited()

    def _selected_channels(self):
        return sorted({index.row() + 10 * (index.column() // 4)
                       for index in self.channel_table.selectionModel().selectedIndexes()})

    def _fill_channels(self):
        self.batch_margin.interpretText()
        self.batch_duration.interpretText()
        for channel in self._selected_channels():
            box, margin, duration = self.channel_inputs[channel]
            box.setChecked(True)
            margin.setValue(self.batch_margin.value())
            duration.setValue(self.batch_duration.value())

    def _inherit_channels(self):
        for channel in self._selected_channels():
            self.channel_inputs[channel][0].setChecked(False)

    def _edited(self, *_):
        self._set_message('修改尚未生效或保存；请点击“确认生效并保存”，随后保持释放完成初始化。', '#ffcc78')

    def _set_message(self, text, color=''):
        for label in (self.message, self.bottom_message):
            label.setText(text)
            label.setStyleSheet(f'color:{color};' if color else '')

    def settings(self):
        return self.engine.parameters.settings()

    def apply(self):
        # 提交仍在编辑的文本，避免最后一个输入框未失焦时保存到旧值。
        for widget in self.inputs.values():
            widget.interpretText()
        try:
            # 合并可见编辑项与已加载参数，防止应用时把 JSON 专属值还原为默认值。
            settings = self.engine.parameters.settings()
            settings.update({key: widget.value() for key, widget in self.inputs.items()})
            overrides = {}
            for channel, (box, margin, duration) in enumerate(self.channel_inputs):
                margin.interpretText()
                duration.interpretText()
                if box.isChecked():
                    overrides[str(channel)] = dict(margin=margin.value(), confirm_frames=duration.value())
            settings['channel_repress'] = overrides
            parameters = HostParameters(**settings)
        except ValueError as exc:
            self._set_message(f'未应用、未保存：{exc}', '#ff8d78')
            return False
        if self.save_callback is None:
            self._set_message('未应用、未保存：保存功能尚未连接。', '#ff8d78')
            return False
        saved, detail = self.save_callback(parameters)
        if not saved:
            self._set_message(f'未应用、未保存：{detail}。修改值已保留，请重试。', '#ff8d78')
            return False
        self.engine.configure(parameters)
        self.reset_requested.emit()
        self._set_message(f'已生效并保存；请保持释放完成初始化。\n保存位置：{detail}', '#8be277')
        self.settings_changed.emit()
        return True

    def load_settings(self, settings):
        settings = dict(settings)
        legacy = []
        overrides = dict(settings.get('channel_repress', {}))
        for channel, values in list(overrides.items()):
            if isinstance(values, dict) and 'confirm_ms' in values:
                history = list(self.engine.channels[int(channel)].history) if str(channel).isdigit() and 0 <= int(channel) < 20 else []
                interval = ((history[-1][0] - history[0][0]) / (len(history)-1)
                            if len(history) > 1 else None)
                if interval and interval > 0:
                    overrides[channel] = dict(margin=values['margin'],
                        confirm_frames=min(10000, max(1, math.ceil(values['confirm_ms'] / (interval*1000)))))
                else:
                    overrides.pop(channel)
                legacy.append(str(channel))
        settings['channel_repress'] = overrides
        try:
            parameters = HostParameters.from_settings(settings)
        except (TypeError, ValueError, AttributeError):
            parameters = HostParameters()
        self.engine.configure(parameters)
        for key, widget in self.inputs.items():
            widget.blockSignals(True)
            widget.setValue(getattr(parameters, key))
            widget.blockSignals(False)
        for channel, (box, margin, duration) in enumerate(self.channel_inputs):
            values = parameters.channel_repress.get(str(channel))
            box.setChecked(values is not None)
            if values:
                margin.setValue(values['margin'])
                duration.setValue(int(values['confirm_frames']))
        if legacy:
            self._set_message('旧毫秒配置已迁移：有采样参考时换算为帧，无参考时恢复使用公共参数。涉及CH' + '、CH'.join(legacy) + '；请核对后确认生效并保存。', '#ffcc78')
        else:
            self._set_message('已加载参数；修改后请点击“确认生效并保存”。')

    def refresh(self):
        engine = self.engine
        p = engine.parameters
        public_detail = (f'负向下冲及其后续恢复：超出原零点 {p.negative_press_margin:g} count，连续 {p.negative_press_frames} 帧；'
                         f' slowback/recover：超出卸载前基线 {p.reload_reference_margin:g} count，连续 {p.reload_frames} 帧。'
                         '均须同时满足对应的其他公共加载条件。')
        for box, margin, duration in self.channel_inputs:
            box.setToolTip('勾选仅覆盖恢复再按压增量和确认帧数，速度等条件保持公共设置。\n' + public_detail)
        for placeholders in self.channel_placeholders:
            for placeholder in placeholders:
                placeholder.setToolTip(public_detail)
        channel = self.observation_channel.currentIndex()
        summary, detail = format_activation_reason(engine.channels[channel].last_activation)
        self.activation_detail.setText(f'CH{channel} 最近激活：{summary}')
        self.activation_detail.setToolTip(detail)
        remaining = max((max(0., engine.parameters.initial_seconds - (engine.last_timestamp - c.started))
                         for c in engine.channels if c.state == -1 and c.started is not None), default=0.)
        self.status.setText(f'{engine.reason}' + (f' · 释放初始化剩余 {remaining:.2f} 秒' if remaining else ''))

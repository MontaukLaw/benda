"""上位机基线实验参数与初始化概况。"""
from qt_compat import QtWidgets, Signal
from host_state_machine import PARAMETERS, HostParameters, HOST_STATE_NAMES


def format_activation_reason(evidence):
    if not evidence:
        return '--', '本轮初始化后尚未激活。下一次激活时记录原因，退出 activate 后保留。'
    state = HOST_STATE_NAMES[evidence['from_state']]
    frames = evidence['confirmed_frames']
    summary = f'{state}：' + ('差值触发' if evidence['reason'] == 'idle_threshold' else '重新加载') + f'（{frames}帧）'
    details = [f"第 {evidence['activation_number']} 次激活：{state} → activate",
               f"input {evidence['input_val']:.3f}，baseline {evidence['baseline']:.3f}",
               f"差值 {evidence['difference']:.3f}，激活阈值 {evidence['activate_threshold']:.3f}（判定通过）"]
    if evidence['reason'] == 'reload':
        details += [f"阶段谷值 {evidence['trough']:.3f}，较谷值增加 {evidence['increase']:.3f}",
                    f"增加量阈值 {evidence['reload_threshold']:.3f}（判定通过）",
                    f"上升速度 {evidence['pressure_rate']:.3f} %/s，门限 {evidence['reload_rate']:.3f} %/s（判定通过）"]
        if 'release_reference' in evidence:
            margin = evidence['reload_reference_margin']
            details += [f"卸载前参考基线 {evidence['release_reference']:.3f}，超出参考 {evidence['reference_difference']:.3f}",
                        f"超出参考门限 {margin:.3f}（判定通过）" if margin > 0 else '卸载前基线保护已关闭']
    details.append(f"连续确认 {frames}/{evidence['required_frames']} 帧；以上数值固定为触发当帧，不随当前输入变化。")
    return summary, '\n'.join(details)


class HostStatePanel(QtWidgets.QScrollArea):
    settings_changed = Signal()
    waveform_requested = Signal()
    reset_requested = Signal()
    observation_changed = Signal(int)

    def __init__(self, engine, save_callback=None):
        super().__init__()
        self.engine = engine
        self.save_callback = save_callback
        self.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        self.setWidget(content)
        layout = QtWidgets.QVBoxLayout(content)
        note = QtWidgets.QLabel('上位机实验 · 20 通道独立状态，共用以下可调参数。\n'
            '输入 = Raw × 柱状图逐通道增益，再低通；激活差值 = input − baseline。\n'
            '速度 = 最近窗口按采样时间回归的斜率 ÷ 当前 input × 100，单位 %/s。\n'
            'idle / recover 跟踪基线；activate / slowback 冻结基线。仅 activate 输出 Signal。\n'
            '速度仅在 activate 显示；等待和恢复时用于重新加载及恢复稳定判定。\n'
            '重新加载另需超过卸载前保存的基线及余量，避免下冲后的回弹误激活。\n'
            '通道状态、数值和最近激活原因请在“监控”页查看。\n'
            '实验输出可由柱状图、触摸板选择使用；固件上报数据独立保留。应用参数及重置时请保持释放。')
        note.setWordWrap(True)
        layout.addWidget(note)
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
        group = QtWidgets.QGroupBox('状态切换与基线参数')
        form = QtWidgets.QFormLayout(group)
        self.inputs = {}
        for key, label, default, low, high, decimals in PARAMETERS:
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
            widget.valueChanged.connect(self._edited)
            self.inputs[key] = widget
            form.addRow(label, widget)
        layout.addWidget(group)
        self.bottom_apply_button = QtWidgets.QPushButton('确认生效并保存')
        self.bottom_apply_button.setToolTip(self.apply_button.toolTip())
        layout.addWidget(self.bottom_apply_button)
        self.bottom_message = QtWidgets.QLabel(self.message.text())
        self.bottom_message.setWordWrap(True)
        layout.addWidget(self.bottom_message)
        legend = QtWidgets.QLabel('状态背景：idle 绿 / activate 棕 / slowback 蓝 / recover 紫。\n'
            'slowback 达到最长等待或相对基线差值连续达标时进入 recover；快速上升时暂停提前恢复。\n'
            '回弹尚未越过卸载前参考基线时，recover继续跟踪；真正上升候选才冻结基线。\n'
            '初始化取低通 input 的最大值；增益为 0 的通道停用。恢复需差值和速度同时达标。')
        legend.setWordWrap(True)
        layout.addWidget(legend)
        layout.addStretch(1)
        self.apply_button.clicked.connect(self.apply)
        self.bottom_apply_button.clicked.connect(self.apply)
        self.reset_button.clicked.connect(self.reset_requested.emit)
        self.waveform_button.clicked.connect(self.waveform_requested.emit)
        self.refresh()

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
            parameters = HostParameters(**{key: widget.value() for key, widget in self.inputs.items()})
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
        try:
            parameters = HostParameters.from_settings(settings)
        except (TypeError, ValueError, AttributeError):
            parameters = HostParameters()
        self.engine.configure(parameters)
        for key, value in parameters.settings().items():
            self.inputs[key].blockSignals(True)
            self.inputs[key].setValue(value)
            self.inputs[key].blockSignals(False)
        self._set_message('已加载参数；修改后请点击“确认生效并保存”。')

    def refresh(self):
        engine = self.engine
        remaining = max((max(0., engine.parameters.initial_seconds - (engine.last_timestamp - c.started))
                         for c in engine.channels if c.state == -1 and c.started is not None), default=0.)
        self.status.setText(f'{engine.reason}' + (f' · 释放初始化剩余 {remaining:.2f} 秒' if remaining else ''))

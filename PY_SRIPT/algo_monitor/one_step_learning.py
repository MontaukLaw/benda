"""One Step第一阶段：独立的0～30 N整数压力—电机位置学习。"""
from dataclasses import asdict, replace
from datetime import datetime
import math
from pathlib import Path
import json
import time

from pressure_feedback_control import PressureFeedbackController, FeedbackSettings, FeedbackControlError
from high_force_calibration import CombinedCancel, recover_release
from force_signal_pressure_calibration import _write_json_atomic

DOCUMENT_KIND = 'one_step_integer_pressure_learning'
TARGETS_N = tuple(range(31))


def default_settings():
    return FeedbackSettings(target_n=30, tolerance_n=.05, force_limit_n=35,
        max_step=4, speed=5, approach_speed=5, release_speed=24, acceleration=100,
        max_current_ma=3000, max_travel=6144, settle_s=.1, stable_s=.5,
        hold_s=1, acquire_timeout_s=180)


def validate_settings(settings, profile):
    settings.validate(profile)
    if profile.model != 'zdt_x42s':
        raise ValueError('One Step学习目前仅支持新ZDT电机')
    if settings.target_n != 30 or not 1 <= settings.max_step <= 16:
        raise ValueError('整数学习固定到30 N，每次最多允许1～16 step')
    if not .01 <= settings.tolerance_n <= .2:
        raise ValueError('每个整数点使用绝对容差0.01～0.20 N，避免相邻点重叠')


class IntegerPressureLearningController(PressureFeedbackController):
    """仅替换学习目标计划；保护、静止窗口与串行小步运动使用原闭环。"""
    def __init__(self, *args, on_point=lambda row: None, **kwargs):
        super().__init__(*args, **kwargs)
        self.nodes = []
        self.on_point = on_point

    def _record_point(self, target, force, position):
        if not all(math.isfinite(v) for v in (force, position)) or int(position) != position:
            raise FeedbackControlError('学习压力或位置无效')
        if self.nodes and (position >= self.nodes[-1]['position_steps']
                           or force <= self.nodes[-1]['measured_force_n']):
            raise FeedbackControlError('相邻整数点未形成递增压力与下压行程，保留原始结果并先释放')
        row = dict(target_force_n=target, measured_force_n=force, position_steps=int(position),
            travel_steps=self.release-int(position), error_n=force-target,
            captured_at=datetime.now().astimezone().isoformat())
        self.nodes.append(row)
        self.on_point(dict(row))

    def _learn(self):
        original = self.settings
        sample, force = self._window()
        if (max(abs(v) for v in self._window_force_range) > .02
                or abs(sample['position_counts']-self.release) > 1):
            raise FeedbackControlError('起始点须为稳定零压力，且实际位置与设置起点相差不超过1 step')
        self._record_point(0, force, sample['position_counts'])
        try:
            for target in TARGETS_N[1:]:
                self.settings = replace(original, target_n=target)
                # 每个整数点独立计时，连续加载保留刚度估计，不借用旧学习表。
                self.deadline = self.clock()+original.acquire_timeout_s
                self._last_adjustment_error = self._reversal_cap = None
                self._adjustment_gain = .75
                self._centering_direction = 0
                result = self._regulate(stop_on_stable=True)
                self._record_point(target, result['actual_force_n'], result['actual_position_steps'])
            return dict(ok=True, message='0～30 N共31个位置已采集，准备回起始位置')
        finally:
            self.settings = original


def run_learning(motor, profile, samples, stop, emergency, settings, origin, *,
                 progress=lambda row: None, on_point=lambda row: None,
                 on_motor=lambda row: None, clock=time.monotonic):
    """普通停止及异常总先回起点；只有急停取消自动回位，压力失联不阻止抬升。"""
    validate_settings(settings, profile)
    started = clock()
    controller = IntegerPressureLearningController(motor, profile, samples,
        CombinedCancel(stop, emergency), settings, origin, on_record=progress,
        on_motor=on_motor, clock=clock, on_point=on_point)
    result = controller.run(learning=True)
    progress(dict(phase='return', message='正在停止并回到本次学习的起始位置'))
    released = recover_release(motor, profile, samples, settings, origin, emergency,
        progress=progress, on_motor=on_motor, clock=clock)
    # 回位和压力卸载分别确认，不能把旧文件或急停前的位置当作本次成功。
    unloaded = False
    if released['release_position_confirmed'] and not emergency.is_set():
        deadline, first, previous, count = clock()+3, None, None, 0
        while clock() < deadline and not emergency.is_set():
            try:
                force, stamp, sequence = samples.read(clock(), allow_negative=True)
                if abs(force) > .02:
                    first, count = None, 0
                elif sequence != previous:
                    first = stamp if first is None else first
                    count += 1
                    if count >= 3 and stamp-first >= .1-1e-9:
                        unloaded = True
                        break
                previous = sequence
            except FeedbackControlError:
                first, count = None, 0
            # 同时刷新真实电机反馈，测试与实机都不依赖GUI刷新计时。
            try:
                feedback = motor.read_feedback()
                on_motor(feedback)
            except Exception:
                pass
            emergency.wait(.05)
    complete = bool(result['ok'] and len(controller.nodes) == 31 and unloaded
                    and released['release_position_confirmed'] and not stop.is_set() and not emergency.is_set())
    message = ('学习完成，31个整数压力位置已记录，并已回起点卸载' if complete else
               result['message']+'；'+released['message']+'；学习未完成，不更新有效表')
    return dict(kind=DOCUMENT_KIND, format_version=1,
        created_at=datetime.now().astimezone().isoformat(), profile=profile.metadata(), namespace=profile.namespace,
        origin_position_steps=origin, settings=asdict(settings), complete=complete,
        nodes=controller.nodes, message=message, elapsed_s=clock()-started,
        release_position_confirmed=released['release_position_confirmed'], pressure_unloaded=unloaded,
        cancelled=stop.is_set(), emergency_stopped=emergency.is_set(),
        motion_diagnostics=result.get('motion_diagnostics', []))


def validate_document(document, profile):
    if (document.get('kind') != DOCUMENT_KIND or document.get('format_version') != 1
            or document.get('namespace') != profile.namespace or document.get('profile') != profile.metadata()):
        raise ValueError('不是当前电机的独立One Step学习文件')
    validate_settings(FeedbackSettings(**document['settings']), profile)
    nodes, origin = document['nodes'], document['origin_position_steps']
    if (type(document.get('complete')) is not bool or type(origin) is not int
            or not isinstance(nodes, list) or len(nodes) > 31):
        raise ValueError('学习起点或位置表无效')
    for target, row in enumerate(nodes):
        if (type(row['target_force_n']) is not int or row['target_force_n'] != target
                or type(row['position_steps']) is not int
                or not math.isfinite(row['measured_force_n'])
                or abs(row['measured_force_n']-target) > (.02 if target == 0 else document['settings']['tolerance_n'])+1e-9
                or row['travel_steps'] != origin-row['position_steps']
                or not -1 <= row['travel_steps'] <= document['settings']['max_travel']
                or target == 0 and abs(row['travel_steps']) > 1
                or not math.isfinite(row['error_n'])
                or abs(row['error_n']-(row['measured_force_n']-target)) > 1e-9):
            raise ValueError('整数压力点、实测压力或位置无效')
        if target and (row['position_steps'] >= nodes[target-1]['position_steps']
                       or row['measured_force_n'] <= nodes[target-1]['measured_force_n']):
            raise ValueError('压力与下压行程未严格递增')
    if document.get('complete') and (len(nodes) != 31 or not document.get('release_position_confirmed')
                                   or not document.get('pressure_unloaded')):
        raise ValueError('学习尚未完成全部整数点及回位卸载')
    return document


def latest_path(directory, profile):
    return Path(directory)/f'one_step_learning_latest_{profile.namespace}.json'


def save_document(directory, document, profile, archive_path=None):
    validate_document(document, profile)
    path = Path(archive_path) if archive_path else Path(directory)/(
        'one_step_learning_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    _write_json_atomic(path, document)
    if document['complete']:
        _write_json_atomic(latest_path(directory, profile), document)
    return path


def load_document(directory, profile):
    path = latest_path(directory, profile)
    if not path.exists():
        return None
    document = validate_document(json.loads(path.read_text(encoding='utf-8')), profile)
    if not document['complete']:
        raise ValueError('latest不是完整的One Step学习结果')
    return document

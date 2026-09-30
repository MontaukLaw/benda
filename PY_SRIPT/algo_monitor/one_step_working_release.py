"""确认接触附近的工作释放点；保留绝对坐标及原始机械行程边界。"""
from dataclasses import replace
import math

from pressure_feedback_control import PressureFeedbackController, FeedbackControlError

WORKING_POSITION_TOLERANCE_STEPS = 2


def working_position_matches(actual, target, profile):
    """承压侧保持2步限制；释放侧沿用驱动回位窗口，仍须另外确认静止和连续卸载。"""
    return math.isfinite(actual) and -WORKING_POSITION_TOLERANCE_STEPS <= actual-target <= profile.tolerance(256)


class WorkingReleaseSettlingTimeout(FeedbackControlError):
    """反馈有效但静止窗口尚未形成，可原位有限重建窗口。"""


def working_settings(settings, original, working):
    return replace(settings, max_travel=settings.max_travel+working-original)


def unloaded_window(checker, position, *, phase='工作释放点卸载确认'):
    """连续一秒确认位置静止、压力新鲜及卸载；回位后压力丢失只返回无卸载证据。"""
    checker.phase = phase
    frames, last_motor_stamp, anchor = [], None, None
    checker._unloaded_partial = frames
    checker._unloaded_diagnostics = []
    end = checker.clock()+2
    while checker.clock() < end:
        sample, force, stamp, sequence = checker._poll()
        checker._unloaded_last_sample = sample
        now = checker.clock()
        checker._unloaded_diagnostics.append(dict(
            position_steps=sample['position_counts'], position_error_steps=sample['position_counts']-position,
            speed_rpm=sample['measured_speed_rpm'], in_position=sample.get('in_position'),
            running_state=sample['running_state'], motor_stamp=sample['sampled_monotonic'],
            force_n=force, stamp=stamp, sequence=sequence, elapsed_s=now-(end-2)))
        if (not 0 <= now-sample['sampled_monotonic'] <= .15
                or not math.isfinite(force) or not 0 <= now-stamp <= .15):
            raise FeedbackControlError('工作释放点缺少新鲜电机或压力反馈')
        if (not working_position_matches(sample['position_counts'], position, checker.profile) or abs(sample['measured_speed_rpm']) > 1
                or not sample.get('in_position') or sample['running_state'] not in (0, 1)):
            frames.clear()
            anchor = None
        elif sample['sampled_monotonic'] != last_motor_stamp and (not frames or frames[-1]['sequence'] != sequence):
            if anchor is None or abs(sample['position_counts']-anchor) > 1:
                frames.clear()
                anchor = sample['position_counts']
            if frames and not 0 < stamp-frames[-1]['stamp'] <= .2+1e-9:
                frames.clear()
            frames.append(dict(position_steps=sample['position_counts'], force_n=force, stamp=stamp, sequence=sequence))
            if len(frames) >= 10 and frames[-1]['stamp']-frames[0]['stamp'] >= 1-1e-9:
                return dict(position_steps=position, samples=frames,
                            min_force_n=min(r['force_n'] for r in frames),
                            max_force_n=max(r['force_n'] for r in frames),
                            unloaded=all(abs(r['force_n']) <= .02+1e-9 for r in frames))
        last_motor_stamp = sample['sampled_monotonic']
        checker._wait()
    last = checker._unloaded_diagnostics[-1] if checker._unloaded_diagnostics else {}
    raise WorkingReleaseSettlingTimeout(f'工作释放点静止卸载确认超时：目标{position} step，'
        f'实际{last.get("position_steps")} step，速度{last.get("speed_rpm")} RPM，'
        f'到位标志{last.get("in_position")}，压力{last.get("force_n")} N')


def confirm_unloaded(checker, position, evidence, *, phase='工作释放点卸载复核'):
    """同一门槛连续窗口复核，最多三窗；只观察不发运动命令，完整保留不合格及异常证据。"""
    check = dict(position_steps=position, phase=phase, windows=[], samples=[], unloaded=False,
                 status='checking', started_monotonic=checker.clock())
    evidence.append(check)
    try:
        for attempt in range(1, 4):
            try:
                window = unloaded_window(checker, position, phase=phase)
            except WorkingReleaseSettlingTimeout as exc:
                check['windows'].append(dict(status='settling_timeout', samples=list(checker._unloaded_partial),
                    diagnostics=list(checker._unloaded_diagnostics), error=str(exc)))
                if attempt == 3:
                    raise
                checker.on_record(dict(phase='working_release_recheck', message=
                    f'第{attempt}/3窗尚未形成静止证据，原位重新观察，不发加压命令；{exc}'))
                continue
            check['windows'].append(window)
            check.update(samples=window['samples'], unloaded=window['unloaded'])
            values = [r['force_n'] for w in check['windows'] for r in w['samples']]
            check.update(min_force_n=min(values), max_force_n=max(values))
            if window['unloaded']:
                check['status'] = 'passed' if attempt == 1 else 'recovered'
                check['actual_position_steps'] = window['samples'][-1]['position_steps']
                check['position_error_steps'] = check['actual_position_steps']-position
                if check['position_error_steps'] > WORKING_POSITION_TOLERANCE_STEPS:
                    checker.on_record(dict(phase='working_release_recheck', message=
                        f'实际释放位置{check["actual_position_steps"]} step，比设定{position} step向上偏{check["position_error_steps"]}步；'
                        '已连续1秒确认静止及卸载，保留原坐标继续，无需反向下压纠偏'))
                if attempt > 1:
                    checker.on_record(dict(phase='working_release_recheck', message=
                        f'工作释放点{position} step第{attempt}窗已连续1秒卸载合格，保持原位置继续任务'))
                return check
            checker.on_record(dict(phase='working_release_recheck', message=
                f'工作释放点{position} step第{attempt}/3窗压力{window["min_force_n"]:.3f}～{window["max_force_n"]:.3f} N，'
                + ('原位复核，尚不启动加压' if attempt < 3 else '连续复核未通过，退回原释放点并结束')))
        check['status'] = 'persistent_residual'
        return check
    except Exception as exc:
        check.update(status='error', error=str(exc), error_type=type(exc).__name__,
                     partial_samples=list(getattr(checker, '_unloaded_partial', [])),
                     diagnostics=list(getattr(checker, '_unloaded_diagnostics', [])))
        raise
    finally:
        check['elapsed_s'] = checker.clock()-check['started_monotonic']


class WorkingReleaseController(PressureFeedbackController):
    """只在首次就位时低速接近候选点；未卸载则向原释放方向退，不发送坐标清零。"""
    def __init__(self, *args, contact_position, evidence, **kwargs):
        super().__init__(*args, **kwargs)
        self.contact_position, self.evidence = math.ceil(contact_position), evidence
        self.confirmed = None

    def _fast_approach(self):
        target = self.contact_position
        while target <= self.release:
            self.phase = '低速确认工作释放点'
            self._check()
            sample = self._poll()[0]
            if abs(sample['position_counts']-target) > 1:
                self.motor.prepare_motion()
                self._check()
                self._poll()
                self.motor.move_absolute_steps(target, speed_rpm=2, accel=self.settings.acceleration,
                                               retry_cancelled=self.cancel.is_set)
                self.move_count += 1
                expiry = self.clock()+self.profile.motion_timeout(sample['position_counts']-target, 2, self.settings.acceleration)
                while True:
                    sample, _, pressure_stamp, _ = self._poll()
                    if (self.clock()-pressure_stamp > .15 or self.clock()-sample['sampled_monotonic'] > .15):
                        raise FeedbackControlError('首次就位反馈过期，先回原释放点')
                    if sample['position_counts'] < self.contact_position-2:
                        raise FeedbackControlError('首次就位越过接触候选位置，先回原释放点')
                    if (abs(sample['position_counts']-target) <= 1 and abs(sample['measured_speed_rpm']) <= 1
                            and sample.get('in_position') and sample['running_state'] in (0, 1)):
                        break
                    if self.clock() > expiry:
                        raise FeedbackControlError('首次就位超时，先回原释放点')
                    self._wait()
            check = unloaded_window(self, target)
            self.evidence.append(check)
            if check['unloaded']:
                self.confirmed = check
                return
            self.on_record(dict(phase='working_release_backoff', message=f'{target} step未稳定卸载至±0.02 N，向原释放方向退8 step再确认'))
            if target == self.release:
                raise FeedbackControlError('原释放点也未卸载，停止学习')
            target = min(self.release, target+8)

    def _regulate(self, stop_on_stable=False):
        return dict(ok=True, message='工作释放点已连续一秒确认卸载')


def establish_working_release(motor, profile, samples, cancel, settings, original, contact, evidence,
                              *, progress, on_motor, clock):
    # 首次接近只允许轻触；压力超限、反馈异常或停止统一交由外层回原释放点。
    controller = WorkingReleaseController(motor, profile, samples, cancel,
        replace(settings, target_n=.1, tolerance_n=.01, force_limit_n=.15), original,
        contact_position=contact['median_position_steps'], evidence=evidence,
        on_record=progress, on_motor=on_motor, clock=clock)
    result = controller.run()
    if not result['ok'] or controller.confirmed is None:
        raise FeedbackControlError(result['message'])
    return controller.confirmed

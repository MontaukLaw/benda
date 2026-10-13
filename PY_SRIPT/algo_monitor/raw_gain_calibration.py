"""20通道定载荷Raw增量标定；只消费采样，不修改任何通道状态。"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
import statistics

from raw_gain import RAW_GAIN_MAX


@dataclass(frozen=True)
class CalibrationSettings:
    mass_g: float = 100.0
    repeats: int = 5
    target: float = 0.0
    tolerance: float = 50.0
    settle_seconds: float = 1.0
    window_seconds: float = 0.6
    max_cv_percent: float = 15.0

    def __post_init__(self):
        limits = {'mass_g': (1, 10000), 'repeats': (3, 20), 'target': (0, 1000000),
                  'tolerance': (1, 10000), 'settle_seconds': (.2, 10),
                  'window_seconds': (.3, 5), 'max_cv_percent': (1, 50)}
        for name, (low, high) in limits.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'无效标定参数：{name}')
        if not isinstance(self.repeats, int):
            raise ValueError('重复次数必须为整数')


class RawGainCalibration:
    GAP_SECONDS = .2
    TIMEOUT_SECONDS = 20.0

    def __init__(self, settings=None):
        self.settings = settings or CalibrationSettings()
        self.trials = [[] for _ in range(20)]
        self.validations = [[] for _ in range(20)]
        self.discarded = []
        self.phase = 'empty'
        self.channel = 0
        self.mode = 'calibrate'
        self.last_sequence = 0
        self.last_timestamp = None
        self.invalid_reason = ''
        self.message = '移开砝码，选择通道，然后采集空载。'
        self.baseline = self.noise = None
        self.window = deque()
        self.started = None
        self.revision = 0
        self.raw_limits = (65535,) * 20

    @classmethod
    def from_report(cls, report):
        """读取原始采集记录并重新计算结果，不恢复旧空载窗口或采样序号。"""
        if (not isinstance(report, dict) or type(report.get('schema_version')) is not int
                or report['schema_version'] != 1
                or report.get('method') != 'raw_loaded_minus_per_trial_unloaded_median'):
            raise ValueError('不是支持的20通道Raw增益标定记录')
        try:
            model = cls(CalibrationSettings(**report['settings']))
            revision = report['calibration_revision']
            if type(revision) is not int or revision < 0:
                raise ValueError('标定版本无效')
            def number(value, name, low=0, high=math.inf):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                    raise ValueError(f'记录中的{name}无效')
            def vector(values, name, high=65535):
                if not isinstance(values, list) or len(values) != 20:
                    raise ValueError(f'记录中的{name}须包含20通道')
                for value in values: number(value, name, high=high)
            def trial(value, channel, validation=False):
                if not isinstance(value, dict) or type(value.get('channel')) is not int or value['channel'] != channel:
                    raise ValueError('记录通道与所在行不一致')
                vector(value['baseline'], '空载Raw'); vector(value['loaded_raw'], '加载Raw')
                vector(value['loaded_ranges'], '加载波动范围')
                number(value['response'], '响应', low=100, high=65535)
                expected = value['loaded_raw'][channel] - value['baseline'][channel]
                if not math.isclose(value['response'], expected, rel_tol=1e-9, abs_tol=1e-6):
                    raise ValueError('记录响应与加载、空载Raw不一致')
                unloaded = value['unloaded']
                vector(unloaded['raw'], '空载窗口Raw'); vector(unloaded['noise'], '空载噪声')
                vector(unloaded['ranges'], '空载波动范围')
                if unloaded['raw'] != value['baseline']:
                    raise ValueError('空载窗口与基线记录不一致')
                for capture in (unloaded,):
                    number(capture['timestamp'], '采样时间')
                    if type(capture['sample_count']) is not int or capture['sample_count'] < 20:
                        raise ValueError('稳定窗口采样数量无效')
                number(value['loaded_timestamp'], '加载时间')
                number(value['loaded_elapsed_seconds'], '加载窗口时间')
                if type(value['loaded_sample_count']) is not int or value['loaded_sample_count'] < 20:
                    raise ValueError('加载窗口采样数量无效')
                if type(value['dominant_channel']) is not int or value['dominant_channel'] not in range(20):
                    raise ValueError('最大响应通道无效')
                if 'capture_settings' in value: CalibrationSettings(**value['capture_settings'])
                if validation:
                    number(value['gain'], '验证增益', low=1e-12, high=RAW_GAIN_MAX)
                    number(value['target'], '验证目标', low=1e-12)
                    number(value['normalized_response'], '验证响应', low=1e-12)
                    if not math.isclose(value['normalized_response'], value['response']*value['gain'], rel_tol=1e-9, abs_tol=1e-6):
                        raise ValueError('验证归一化响应与原始响应不一致')
                    if type(value['calibration_revision']) is not int or not 0 <= value['calibration_revision'] <= revision:
                        raise ValueError('验证版本无效')
            for name in ('trials', 'validations'):
                groups = report[name]
                if not isinstance(groups, list) or len(groups) != 20:
                    raise ValueError('记录须包含20个通道分组')
                for channel, values in enumerate(groups):
                    if not isinstance(values, list): raise ValueError('通道记录必须为列表')
                    for value in values: trial(value, channel, name == 'validations')
                setattr(model, name, deepcopy(groups))
            discarded = report.get('discarded', [])
            if not isinstance(discarded, list): raise ValueError('撤销归档无效')
            for item in discarded:
                if not isinstance(item, dict) or item.get('mode') not in ('calibrate', 'validate'):
                    raise ValueError('撤销归档模式无效')
                channel = item['trial']['channel']
                if type(channel) is not int or channel not in range(20): raise ValueError('撤销归档通道无效')
                trial(item['trial'], channel, item['mode'] == 'validate')
            model.discarded = deepcopy(discarded)
            model.revision = revision
            # 历史连接失效原因由页面保存为来源信息；新会话先核对环境。
            model.invalid_reason = '历史记录尚未核对当前设备与三参数'
            model.message = '历史采集记录已读取；请连接传感器，核对设备与三参数。'
            model.results()
            return model
        except (KeyError, TypeError, AttributeError, OverflowError) as exc:
            raise ValueError('标定记录缺少有效的原始采集数据') from exc

    @property
    def capturing(self):
        return self.phase in ('capture_empty', 'capture_loaded')

    def cancel(self, reason='采集已取消，请重新采集空载。'):
        self.phase = 'empty'
        self.baseline = self.noise = None
        self.window.clear()
        self.started = None
        self.message = reason

    def invalidate(self, reason):
        self.cancel(reason + '；已完成记录保留，请新建标定。')
        self.invalid_reason = reason

    def undo_last(self, channel, mode):
        self.cancel()
        trials=self.validations if mode=='validate' else self.trials
        if trials[channel]:
            self.discarded.append(dict(mode=mode,trial=trials[channel].pop()))
            if mode=='calibrate':self.revision+=1
            self.message='已撤销最近一次，原记录保留在文件中；请重新采集空载。'

    def start_empty(self, channel, now, mode='calibrate'):
        if self.invalid_reason:
            raise ValueError(self.invalid_reason + '；请新建标定')
        if channel not in range(20) or mode not in ('calibrate', 'validate', 'apply'):
            raise ValueError('无效通道或采集模式')
        if mode != 'calibrate' and not self.results()['ready']:
            raise ValueError('请先完成20通道合格标定')
        self.cancel()
        self.channel, self.mode, self.started = channel, mode, now
        self.phase = 'capture_empty'
        self.message = '正在采集空载，请完全移开砝码和手指。'

    def start_loaded(self, now):
        if self.phase != 'await_load' or self.mode == 'apply':
            raise ValueError('请先采集本次空载')
        if now-self.empty_capture['timestamp'] > 30:
            self.cancel('空载参考超过30秒，请重新采集空载。')
            raise ValueError(self.message)
        self.started = now
        self.window.clear()
        self.phase = 'capture_loaded'
        self.message = f'正在采集CH{self.channel}，请保持砝码静止。'

    def expire(self, now):
        if self.phase == 'await_load' and (now-self.empty_capture['timestamp'] > 30
                or self.last_timestamp is None or now-self.last_timestamp > self.GAP_SECONDS):
            self.cancel('空载参考过期或采样中断，请重新采集空载。')
        if self.capturing:
            if self.last_timestamp is None or now-self.last_timestamp > self.GAP_SECONDS:
                self.cancel('采样中断，本次采集作废；请重新采集空载。')
            elif now-self.started > self.TIMEOUT_SECONDS:
                self.cancel('稳定采集超时，本次采集作废；检查砝码和振动后重试。')

    def feed(self, sample):
        seq, stamp = sample.get('sequence'), sample.get('sample_timestamp')
        if not isinstance(seq, int) or seq <= self.last_sequence:
            return None
        previous_seq, previous_time = self.last_sequence, self.last_timestamp
        self.last_sequence = seq
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            if self.capturing: self.cancel('采样时间无效，请重新采集空载。')
            return None
        self.last_timestamp = stamp
        if not self.capturing or stamp < self.started:
            return None
        raw = sample.get('raw', ())
        if (not sample.get('has_raw') or sample.get('channel_count') != 20 or len(raw) != 20
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= 65535 for v in raw)):
            self.cancel('需要完整20通道原始Raw，请切换Debug后重试。')
            return None
        if (previous_seq and seq != previous_seq+1 or previous_time is not None
                and (stamp <= previous_time or stamp-previous_time > self.GAP_SECONDS)):
            self.cancel('采样序号或时间不连续，本次采集作废。')
            return None
        if stamp-self.started > self.TIMEOUT_SECONDS:
            self.cancel('稳定采集超时，请重新采集空载。')
            return None
        if stamp-self.started < self.settings.settle_seconds:
            return None
        self.window.append((stamp, tuple(raw)))
        while len(self.window) > 2 and self.window[1][0] <= stamp-self.settings.window_seconds:
            self.window.popleft()
        if len(self.window) < 20 or stamp-self.window[0][0] < self.settings.window_seconds:
            return None
        columns = tuple(zip(*(v for _, v in self.window)))
        medians = tuple(float(statistics.median(c)) for c in columns)
        ranges = tuple(max(c)-min(c) for c in columns)
        if self.phase == 'capture_empty':
            if max(ranges) > self.settings.tolerance:
                self.message = '空载尚未稳定，请保持完全释放；20秒后超时。'
                return None
            self.baseline = medians
            self.noise = tuple(statistics.pstdev(c) for c in columns)
            self.empty_capture = dict(timestamp=stamp, sample_count=len(self.window),
                                      raw=list(medians), noise=list(self.noise), ranges=list(ranges))
            self.phase = 'await_load'
            self.window.clear()
            self.message = f'空载完成。将{self.settings.mass_g:g} g砝码放在CH{self.channel}中心，然后采集按压。'
            return 'empty_ready'
        ch = self.channel
        delta = medians[ch]-self.baseline[ch]
        if ranges[ch] > max(self.settings.tolerance, abs(delta)*.03):
            self.message = '按压尚未稳定，请保持砝码静止；20秒后超时。'
            return None
        if max(columns[ch]) >= min(65500, self.raw_limits[ch]*.995):
            self.cancel('所选通道接近Raw上限，不能用于增益标定。')
            return None
        if delta < max(100., 10*self.noise[ch]):
            self.cancel('所选通道响应过小或信噪比不足，请核对位置并重新采集空载。')
            return None
        changes = [v-b for v, b in zip(medians, self.baseline)]
        trial = dict(channel=ch, response=delta, capture_settings=asdict(self.settings),
                     baseline=list(self.baseline), loaded_raw=list(medians),
                     unloaded=self.empty_capture, loaded_timestamp=stamp,
                     loaded_elapsed_seconds=stamp-self.started, loaded_sample_count=len(self.window),
                     loaded_ranges=list(ranges), dominant_channel=max(range(20), key=lambda i: changes[i]))
        if self.mode == 'validate':
            result = self.results()
            trial.update(gain=result['gains'][ch], target=result['target'],
                         normalized_response=delta*result['gains'][ch], calibration_revision=self.revision)
            self.validations[ch].append(trial)
        else:
            self.trials[ch].append(trial)
            self.revision += 1
        self.cancel('本次采集完成。请移开砝码，下次按压前重新采集空载。')
        return 'trial_ready'

    def results(self):
        channels = []
        for ch, trials in enumerate(self.trials):
            values = [t['response'] for t in trials]
            median = statistics.median(values) if values else None
            cv = statistics.pstdev(values)/statistics.mean(values)*100 if values else None
            channels.append(dict(channel=ch, count=len(values), response=median, cv_percent=cv,
                                 qualified=len(values)>=self.settings.repeats and cv<=self.settings.max_cv_percent))
        qualified = all(c['qualified'] for c in channels)
        ready = qualified and not self.invalid_reason
        target = self.settings.target or (statistics.median(c['response'] for c in channels) if qualified else None)
        gains = [round(target/c['response'], 4) for c in channels] if qualified else None
        error = ''
        if gains is not None and any(not 0 < gain <= RAW_GAIN_MAX for gain in gains):
            error = f'目标值导致增益超出有效范围（大于0且不超过{RAW_GAIN_MAX:g}），请调整目标变化量。'
            ready, gains = False, None
        for c in channels:
            gain = gains[c['channel']] if gains else None
            valid = [v for v in self.validations[c['channel']] if gain is not None
                     and v['calibration_revision']==self.revision and v['gain']==gain and v['target']==target]
            c.update(gain=gain, validation_count=len(valid), validation_error_percent=(
                (statistics.median(v['normalized_response'] for v in valid)/target-1)*100 if valid else None))
        return dict(ready=ready, target=target, gains=gains, channels=channels, error=error)

    def report(self):
        return dict(schema_version=1, method='raw_loaded_minus_per_trial_unloaded_median',
                    settings=asdict(self.settings), calibration_revision=self.revision,
                    invalid_reason=self.invalid_reason, trials=self.trials, validations=self.validations, discarded=self.discarded,
                    results=self.results(), weight_is_user_declared=True)

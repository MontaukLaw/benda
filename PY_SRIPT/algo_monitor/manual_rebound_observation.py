"""手动回弹的只读计时：消费固件真实状态，分别测量IDLE和原基线返回。"""

import math

from channel_states import STATE_IDLE, STATE_FLOATING, STATE_NAMES


CHANNELS = (4, 5, 6, 7)
CONTACT_FORCE_N = 0.05
MAX_SAMPLE_AGE = 0.25
DISCONNECT_SECONDS = 1.0


def sensor_values(sample):
    """Baseline与归一化Raw同量纲；禁止用硬件Raw和归一化Baseline混算。"""
    if not sample or not sample.get('has_raw', False):
        return None
    raw = sample.get('hardware_raw', sample.get('raw', ()))
    baseline, states = sample.get('baseline', ()), sample.get('state', ())
    try:
        if min(len(raw), len(baseline), len(states)) <= max(CHANNELS):
            return None
        values = {c: (float(raw[c]), float(baseline[c]), int(states[c])) for c in CHANNELS}
        if not all(math.isfinite(v) for row in values.values() for v in row):
            return None
        return values
    except (TypeError, ValueError, OverflowError):
        return None


class ReboundObservation:
    def __init__(self, emit):
        self.emit = emit
        self.phase = 'idle'
        self.trial = 0
        self.channels = {}
        self.press_at = self.arrival_at = self.contact_at = self.release_at = None
        self.last_sample_at = self.last_sequence = None
        self.target = self.peak_force = self.release_force = None
        self.tolerance = 2.0
        self.stable_seconds = 0.5
        self.gapped = False

    @property
    def active(self):
        return self.phase in ('pressing', 'holding', 'released')

    def log(self, at, event, message, **fields):
        self.emit(dict(trial=self.trial, at=at, event=event, message=message, **fields))

    def begin(self, at, target, sample, tolerance=2, stable_seconds=0.5):
        values = sensor_values(sample)
        if values is None:
            raise ValueError('需要CH4～CH7的Debug Raw、Baseline和真实状态')
        if self.active:
            self.interrupt(at, '开始下一次按压，上一轮未完成的结果保留为未返回')
        self.trial += 1
        self.phase = 'pressing'
        self.press_at, self.target = at, float(target)
        self.arrival_at = self.contact_at = self.release_at = None
        self.release_force = self.peak_force = None
        self.tolerance, self.stable_seconds = float(tolerance), float(stable_seconds)
        self.last_sequence = sample.get('sequence')
        self.last_sample_at = at-float(sample.get('age_seconds', 0))
        self.gapped = False
        self.channels = {c: dict(raw=v[0], reference=v[1], state=v[2], locked=False,
                                release_state=None, idle_s=None, raw_s=None, raw_since=None)
                         for c,v in values.items()}
        self.log(at, 'press', f'下压发令：目标 {target:.2f} N；开始计时。', target_force_n=target)

    def pressure(self, at, force):
        if not self.active or self.release_at is not None or at < self.press_at:
            return
        if force is None or not math.isfinite(force):
            return
        self.peak_force = force if self.peak_force is None else max(self.peak_force, force)
        if self.contact_at is None and force > CONTACT_FORCE_N:
            self.contact_at = at
            self.log(at, 'contact', f'检测到接触：实测 {force:.2f} N（超过 {CONTACT_FORCE_N:.2f} N）；接触计时开始。')

    def arrived(self, at, force):
        if self.phase != 'pressing':
            return
        self.phase, self.arrival_at = 'holding', at
        text = '--' if force is None else f'{force:.2f}'
        self.log(at, 'arrived', f'下压到位：实测 {text} N；开始到位保持计时。', force_n=force)

    def release(self, at, sample, force):
        if self.phase not in ('pressing', 'holding'):
            return
        values = sensor_values(sample)
        if values is None or float(sample.get('age_seconds', math.inf)) > MAX_SAMPLE_AGE:
            self.interrupt(at, '释放前传感器数据缺失或过期，无法确定释放状态')
            return
        self.release_at, self.release_force, self.phase = at, force, 'released'
        contact_s = None if self.contact_at is None else at-self.contact_at
        hold_s = None if self.arrival_at is None else at-self.arrival_at
        fmt = lambda v: '--' if v is None else f'{v:.3f}'
        states = []
        for c,v in values.items():
            row = self.channels[c]
            # 未观测到基线冻结时采用释放前的Baseline，并明确记录参考来源。
            if not row['locked']:
                row['reference'] = v[1]
                self.log(at, 'reference_fallback', f'CH{c} 未观测到冻结起点，使用释放前Baseline {v[1]:.1f} counts作为参考；Raw返回时间仅供参考。', channel=c, reference=v[1])
            row.update(raw=v[0], state=v[2], release_state=v[2])
            if v[2] == STATE_IDLE:
                row['idle_s'] = 0.0
            states.append(f'CH{c}={STATE_NAMES.get(v[2], str(v[2]))}')
        self.log(at, 'release',
                 f'释放发令：目标 {self.target:.2f} N，释放前实测 {fmt(force)} N，峰值 {fmt(self.peak_force)} N；'
                 f'发令至释放 {at-self.press_at:.3f} s，接触持续 {fmt(contact_s)} s，到位保持 {fmt(hold_s)} s。\n'
                 + '；'.join(states), target_force_n=self.target, release_force_n=force,
                 peak_force_n=self.peak_force, command_duration_s=at-self.press_at,
                 contact_duration_s=contact_s, hold_duration_s=hold_s,
                 references={c:r['reference'] for c,r in self.channels.items()},
                 release_states={c:r['release_state'] for c,r in self.channels.items()})
        if any(r['release_state'] != STATE_FLOATING for r in self.channels.values()):
            self.log(at, 'non_floating_release', '释放时并非全部FLOATING：下列时间按真实释放状态起算，不标成FLOATING→IDLE。')
        self.last_sample_at = at-float(sample.get('age_seconds',0))
        self.last_sequence = sample.get('sequence')

    def sample(self, now, sample):
        if not self.active:
            return
        if not sample or float(sample.get('age_seconds', math.inf)) > MAX_SAMPLE_AGE:
            self.check_timeout(now)
            return
        values = sensor_values(sample)
        if values is None:
            self.interrupt(now, 'Debug Raw或通道数据缺失，观察中断')
            return
        sequence = sample.get('sequence')
        if sequence is None or sequence == self.last_sequence:
            self.check_timeout(now)
            return
        if self.last_sequence is not None and sequence < self.last_sequence:
            self.interrupt(now, '采样序号重置或设备重新连接，观察中断')
            return
        at = now-max(0.0, float(sample.get('age_seconds',0)))
        if at < (self.release_at if self.release_at is not None else self.press_at):
            return
        if self.last_sample_at is not None and at-self.last_sample_at > DISCONNECT_SECONDS:
            self.interrupt(now, '传感器采样中断超过1秒，不能把重连后的IDLE计为恢复')
            return
        if self.last_sample_at is not None and at-self.last_sample_at > MAX_SAMPLE_AGE:
            for row in self.channels.values():
                row['raw_since'] = None
            if not self.gapped:
                self.log(at, 'gap', '采样间断超过0.25秒：Raw连续确认重置，恢复时间标记为首次观测值。')
            self.gapped = True
        self.last_sample_at, self.last_sequence = at, sequence
        for c,(raw,baseline,state) in values.items():
            row = self.channels[c]
            old = row['state']
            row.update(raw=raw, state=state)
            if self.release_at is None:
                if not row['locked']:
                    row['reference'] = baseline
                    if state != STATE_IDLE or raw-baseline > 12:
                        row['locked'] = True
                        self.log(at, 'reference', f'CH{c} 本轮冻结基线参考：{baseline:.1f} counts。', channel=c, reference=baseline)
                continue
            seconds = at-self.release_at
            if state != old:
                self.log(at, 'state', f'释放后 {seconds:.3f} s：CH{c} {STATE_NAMES.get(old,old)} → {STATE_NAMES.get(state,state)}', channel=c, state=state, seconds=seconds)
            if row['idle_s'] is None and state == STATE_IDLE:
                row['idle_s'] = seconds
                origin = STATE_NAMES.get(row['release_state'], row['release_state'])
                qualifier = '（存在采样间断，为首次观测时间）' if self.gapped else ''
                self.log(at, 'idle', f'CH{c} {origin} → IDLE：{seconds:.3f} s{qualifier}；'
                         f'此时Raw距原基线 {raw-row["reference"]:+.1f} counts。', channel=c, seconds=seconds, gapped=self.gapped)
            if row['raw_s'] is None:
                if abs(raw-row['reference']) <= self.tolerance:
                    if row['raw_since'] is None:
                        row['raw_since'] = at
                    if at-row['raw_since'] >= self.stable_seconds:
                        row['raw_s'] = row['raw_since']-self.release_at
                        self.log(at, 'raw_return', f'CH{c} Raw返回原基线±{self.tolerance:g} counts：'
                                 f'{row["raw_s"]:.3f} s（已连续确认 {self.stable_seconds:g} s）；当前 {raw:.1f}，原基线 {row["reference"]:.1f}。',
                                 channel=c, seconds=row['raw_s'], confirmed_seconds=seconds)
                else:
                    row['raw_since'] = None
        if all(r['idle_s'] is not None and r['raw_s'] is not None for r in self.channels.values()):
            self.finish(at, '全部通道的IDLE与Raw返回时间均已记录', 'complete')

    def check_timeout(self, now):
        if self.active and self.last_sample_at is not None and now-self.last_sample_at > DISCONNECT_SECONDS:
            self.interrupt(now, '超过1秒没有新传感器采样，观察中断；已记录结果保留')

    def finish(self, at, reason, phase='interrupted'):
        if not self.active:
            return
        pending = []
        for c,r in self.channels.items():
            missing = [name for key,name in [('idle_s','IDLE'),('raw_s','原基线Raw')] if r[key] is None]
            if missing:
                pending.append(f'CH{c}：未观测到'+ '/'.join(missing))
        self.phase = phase
        self.log(at, phase, reason + ('；'+'；'.join(pending) if pending else ''),
                 elapsed_after_release_s=None if self.release_at is None else at-self.release_at,
                 results={c:dict(r) for c,r in self.channels.items()})

    def interrupt(self, at, reason):
        self.finish(at, reason)

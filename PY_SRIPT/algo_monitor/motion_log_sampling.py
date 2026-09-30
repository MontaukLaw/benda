"""仅控制运动CSV的保存频率；不影响设备接收、界面刷新和真实通道状态。"""
import json
import math


class MotionLogSampling:
    PERIODIC = frozenset(('sensor_sample', 'motor_sample', 'pressure_meter_sample'))

    def __init__(self):
        self.states = None
        self.stable_since = None
        self.last_seen_at = None
        self.last_seen_sequence = None
        self.last_saved_sequence = None
        self.last_saved_at = None
        self.fresh = None
        self.pressure_valid = None
        self.motor_state = None
        self.position = None
        self.busy = False
        self.last_activity_at = None

    def select(self, event, sample, feedback, pressure, busy, now):
        sample = sample or {}
        sequence = sample.get('sequence')
        age = sample.get('age_seconds', math.inf)
        fresh = sequence is not None and age is not None and math.isfinite(age) and 0 <= age <= .5
        states = tuple(sample.get('state') or ())
        valid_states = fresh and len(states) == 8 and all(isinstance(s, int) and 0 <= s <= 8 for s in states)
        previous_states = self.states
        reasons = []
        if self.fresh is not None and fresh != self.fresh:
            reasons.append('sensor_stream_changed')
        self.fresh = fresh
        if valid_states:
            new_frame = sequence != self.last_seen_sequence
            gap = self.last_seen_at is None or now-self.last_seen_at > .5
            restart = self.last_seen_sequence is not None and sequence < self.last_seen_sequence
            if states != self.states:
                if self.states is not None:
                    reasons.append('sensor_state_changed')
                self.stable_since = now
            if gap or restart or self.stable_since is None:
                self.stable_since = now
            if new_frame:
                self.last_seen_at = now
                self.last_seen_sequence = sequence
            self.states = states
        else:
            self.stable_since = None
            self.states = None
            self.last_seen_at = None

        position = feedback.get('position_counts')
        speed = feedback.get('measured_speed_rpm') or 0
        moved = position is not None and self.position is not None and abs(position-self.position)>2
        self.position = position
        if busy or abs(speed)>1 or moved:
            self.last_activity_at = now
        active = bool(busy or self.last_activity_at is not None and now-self.last_activity_at<.5)
        if active:
            self.stable_since = now
        if active and not self.busy:
            reasons.append('motor_activity_started')
        self.busy = active
        valid = bool(pressure.get('accepted')) and not bool(pressure.get('invalidate'))
        if self.pressure_valid is not None and valid != self.pressure_valid:
            reasons.append('pressure_validity_changed')
        self.pressure_valid = valid
        motor_state = feedback.get('running_state')
        if self.motor_state is not None and motor_state != self.motor_state:
            reasons.append('motor_state_changed')
        self.motor_state = motor_state

        quiet = (valid_states and all(state == 0 for state in states)
                 and self.stable_since is not None and now-self.stable_since >= 60 and not active)
        interval = 1.0 if quiet or (not fresh and not active) else .1
        immediate = event not in self.PERIODIC or bool(reasons)
        if not immediate:
            if self.last_saved_at is not None and now-self.last_saved_at < interval-1e-9:
                return None
            if fresh and sequence == self.last_saved_sequence:
                return None
        self.last_saved_at = now
        self.last_saved_sequence = sequence if fresh else None
        return dict(sensor_record_interval_ms=int(interval*1000),
                    sensor_record_reason=','.join(reasons) or ('event' if immediate else 'all_idle_60s' if quiet else 'periodic'),
                    sensor_states_json=json.dumps(states) if valid_states else '',
                    sensor_previous_states_json=json.dumps(previous_states) if 'sensor_state_changed' in reasons else '',
                    sensor_snapshot_fresh=fresh,
                    sensor_record_source_event=event)

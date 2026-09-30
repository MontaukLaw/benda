"""单次按压回弹采集计划；所有时间均为秒，状态只用于记录。"""
import itertools
import math
import random

REBOUND_PROTOCOL = "raw_rebound_v1"
SINGLE_EXP_TEMPLATE = "single_exponential_calibration_v1"
SHORT_PRESS_TEMPLATE = "short_press_60s_recovery_v1"
REPEAT_SHORT_2N_TEMPLATE = "repeat_2n_3s_60s_10rounds_v1"
CONTINUOUS_SHORT_PRESS_TEMPLATES = (SHORT_PRESS_TEMPLATE, REPEAT_SHORT_2N_TEMPLATE)
# 固定2 N短按十轮，仅整组开始前采集一次空载参考。
REPEAT_SHORT_2N_SETTINGS = dict(high_forces_n=[2.0], high_hold_seconds=[3.0],
                               recovery_seconds=[60.0], repeat_count=10,
                               baseline_wait_seconds=10.0, inter_trial_seconds=0.0)
PAIRED_PRESS_TEMPLATE = "large_then_small_5s_60s_v1"
PAIRED_PRESS_SETTINGS = dict(high_forces_n=[20.0], probe_forces_n=[2.0],
    high_hold_seconds=[3.0], probe_hold_seconds=3.0, recovery_seconds=[5.0, 60.0],
    repeat_count=5, baseline_wait_seconds=10.0, inter_trial_seconds=180.0)
SHORT_PRESS_SETTINGS = dict(high_forces_n=[2.0, 10.0, 20.0], high_hold_seconds=[3.0],
                            recovery_seconds=[60.0], repeat_count=5,
                            baseline_wait_seconds=10.0, inter_trial_seconds=0.0)
# 单指数系数采集的默认条件；保持时间变化用于检验积累规律。
SINGLE_EXP_SETTINGS = dict(high_forces_n=[2.0], high_hold_seconds=[2.0, 10.0, 60.0],
                           recovery_seconds=[600.0], repeat_count=3,
                           baseline_wait_seconds=60.0, inter_trial_seconds=0.0)
REBOUND_ZERO_TOLERANCE_N = 0.10
REBOUND_ZERO_WAIT_SECONDS = 10.0
# 本实验按实测力拟合，允许已停止电机的有限位置误差，并记录真实停点。
REBOUND_POSITION_TOLERANCE_COUNTS = 256
# 空载位置越界需由多次新反馈和持续时间共同确认，速度反馈单独跳变只留记录。
REBOUND_MOTION_CONFIRM_SAMPLES = 3
REBOUND_MOTION_CONFIRM_SECONDS = 0.3


def near_rebound_zero(force):
    return force is not None and math.isfinite(float(force)) and abs(float(force)) <= REBOUND_ZERO_TOLERANCE_N


def matches_single_exp_settings(config):
    # GUI列表使用tuple，落盘JSON使用list，比较数值而非容器类型。
    return all(tuple(config.get(k, ())) == tuple(v) if isinstance(v, (list, tuple))
               else config.get(k) == v for k,v in SINGLE_EXP_SETTINGS.items())


def matches_short_press_settings(config):
    settings = (REPEAT_SHORT_2N_SETTINGS if config.get('template_id') == REPEAT_SHORT_2N_TEMPLATE
                else SHORT_PRESS_SETTINGS)
    return all(tuple(config.get(k, ())) == tuple(v) if isinstance(v, (list, tuple))
               else config.get(k) == v for k,v in settings.items())


def build_rebound_plan(config):
    if config.get('template_id') == PAIRED_PRESS_TEMPLATE:
        if not matches_paired_press_settings(config):
            raise ValueError('大压力后测小压力模板条件已改变，请重新选择模板')
        # 两次按压作为一个完整项目；中断后从大压力开始重做，禁止单独补做小压力。
        return [dict(category='paired_rebound', repeat_index=repeat, process_index=process,
            planned_trial_index=(repeat-1)*2+process, high_force_n=20.0,
            high_hold_seconds=3.0, probe_force_n=2.0, probe_hold_seconds=3.0,
            recovery_seconds=delay, pre_observation_seconds=10.0 if repeat==process==1 else 0.0,
            post_observation_seconds=0.0 if repeat==5 and process==2 else 180.0)
            for repeat in range(1, 6) for process, delay in ((1, 5.0), (2, 60.0))]
    if config.get('template_id') in CONTINUOUS_SHORT_PRESS_TEMPLATES and not matches_short_press_settings(config):
        raise ValueError('短按恢复模板条件已改变，请重新选择模板或使用自定义计划')
    if config.get('template_id') == SINGLE_EXP_TEMPLATE and not matches_single_exp_settings(config):
        raise ValueError('回弹系数模板条件已改变，请重新选择模板或使用自定义计划')
    trials = []
    for repeat in range(1, int(config['repeat_count']) + 1):
        block = [dict(category='raw_rebound', repeat_index=repeat,
                      high_force_n=force, high_hold_seconds=hold,
                      pre_observation_seconds=float(config['baseline_wait_seconds']),
                      recovery_seconds=window, probe_force_n=0.0, probe_hold_seconds=0.0)
                 for force, hold, window in itertools.product(
                     config['high_forces_n'], config['high_hold_seconds'], config['recovery_seconds'])]
        random.Random(int(config.get('random_seed', 20260914)) + repeat).shuffle(block)
        trials.extend(block)
    for index, trial in enumerate(trials, 1):
        trial['planned_trial_index'] = index
        if config.get('template_id') in CONTINUOUS_SHORT_PRESS_TEMPLATES:
            # 后续按压直接接续60秒恢复记录，其末10秒兼作下一次按压参考。
            trial['pre_observation_seconds'] = 10.0 if index == 1 else 0.0
            trial['reference_policy'] = 'session_last_10s_before_press'
        if config.get('template_id') == SINGLE_EXP_TEMPLATE:
            # 按整次按压留出末轮数据，续测保留原角色，不随剩余条数重新分组。
            trial['analysis_role'] = 'validation' if trial['repeat_index'] == 3 else 'fit'
            trial['calibration_model'] = SINGLE_EXP_TEMPLATE
    return trials


def matches_paired_press_settings(config):
    return all(tuple(config.get(k, ())) == tuple(v) if isinstance(v, (list, tuple))
               else config.get(k) == v for k, v in PAIRED_PRESS_SETTINGS.items())

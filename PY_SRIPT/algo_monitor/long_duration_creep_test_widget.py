"""Long-duration force-history and creep experiment with SQLite recording."""

from __future__ import annotations

import csv
import itertools
import json
import math
import os
import queue
import random
import sqlite3
import sys
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

from qt_compat import QtCore, QtGui, QtWidgets, Signal, Slot
from wrapped_label import WrappedLabel
from creep_test_safety import CREEP_RECOVERY_SECONDS
from rebound_test_protocol import (REBOUND_PROTOCOL, build_rebound_plan,
                                   SINGLE_EXP_TEMPLATE, SINGLE_EXP_SETTINGS, matches_single_exp_settings,
                                   SHORT_PRESS_TEMPLATE, SHORT_PRESS_SETTINGS, matches_short_press_settings)
from rebound_test_history import read_manifest, saved_tests, saved_reports
from rebound_test_protocol import PAIRED_PRESS_TEMPLATE, PAIRED_PRESS_SETTINGS, matches_paired_press_settings
from rebound_test_protocol import (REPEAT_SHORT_2N_TEMPLATE, REPEAT_SHORT_2N_SETTINGS,
                                   CONTINUOUS_SHORT_PRESS_TEMPLATES)


CHANNEL_COUNT = 8
DEFAULT_HIGH_FORCES = "12,20"
DEFAULT_PROBE_FORCES = "2,6"
DEFAULT_HIGH_HOLDS = "1,60"
DEFAULT_RECOVERY_DELAYS = "1,10,120,360"
DEFAULT_REPEAT_COUNT = 2
DEFAULT_PROBE_HOLD_SECONDS = 3.0
DEFAULT_BASELINE_WAIT_SECONDS = 360.0
DEFAULT_INTER_TRIAL_SECONDS = 10.0
DEFAULT_SPLIT_MIB = 512
CAPTURE_TIMER_MS = 10
REBOUND_RECORD_INTERVAL_SECONDS = 0.100
REBOUND_RECORDING_SETTINGS = {
    'record_interval_ms': 100,
    'capture_policy': 'latest fresh sensor sample every 100ms; no aggregation; pressure every 100ms plus validity changes; observed state transitions and control events retained',
}
RECORDER_QUEUE_MAX_ITEMS = 100000
RECORDER_BATCH_SIZE = 500


def describe_creep_plan(config):
    """根据当前参数解释每轮内容，避免自定义计划仍显示默认的72项。"""
    if config.get("protocol") == REBOUND_PROTOCOL:
        trials = build_rebound_plan(config)
        if config.get('template_id') == PAIRED_PRESS_TEMPLATE:
            return ("大压力之后测小压力：共5轮，每轮按以下顺序做两个过程，共20次按压。\n"
                    "过程1：20 N保持3秒 → 松开并确认卸载 → 等5秒 → 2 N保持3秒 → 松开 → 休息3分钟。\n"
                    "过程2：20 N保持3秒 → 松开并确认卸载 → 等60秒 → 2 N保持3秒 → 松开 → 休息3分钟再开始下一轮。\n"
                    "只在整组开始前记录10秒；最后一轮完成过程2并确认卸载后结束，不再额外等待3分钟。\n"
                    "保持从电机到位计时，间隔从确认卸载计时；不等待IDLE或Raw回原值。电机保持位置，实际力以压力计为准。\n"
                    "全程每100 ms保存最新读数，含休息期间；每次测试单独保存。中断后从未完成过程的20 N重新测。\n"
                    "压力或电流超限、电机故障、无法保存数据时，停止并尝试卸载。修改本固定模板参数会提示不匹配；请选择其他方案来自定义。")
        values = lambda xs: " / ".join(f"{v:g}" for v in xs)
        purpose = (
            "本方案为CH4～7分别采集k（残余比例）和τ（恢复速度）的计算数据，每通道全程使用一组参数。\n"
            "分3轮，每轮各做一次2、10、60秒，轮内顺序打乱。前6次用于计算，后3次检验预测是否准确。\n"
            "按压前记录60秒，以检查读数漂移。实际压力以压力计为准，计算使用本次按压的Raw增量。\n"
            "完成后自动生成已有恢复指标报告；k、τ拟合尚需后续分析，不会自动写入固件。\n"
            if config.get('template_id') == SINGLE_EXP_TEMPLATE else "")
        short_press = config.get('template_id') in CONTINUOUS_SHORT_PRESS_TEMPLATES
        if short_press:
            purpose = ("检查短按后10秒还剩多少：共5轮，每轮2、10、20 N各压3秒，轮内顺序打乱。\n"
                       "只在整组开始前记录10秒；之后松开并确认卸载，记录满60秒就开始下一次，不额外等待。\n"
                       "后续按压参考取前一次恢复记录的最后10秒；报告同时保留相对本组起点的偏移，观察累积残留。\n"
                       "比较1、5、10、30、60秒的残差和按压增量占比；暂不自动判定合格。\n")
        if config.get('template_id') == REPEAT_SHORT_2N_TEMPLATE:
            purpose = ("简单重复测试：每轮目标2 N，保持3秒，松开并确认卸载后记录60秒，共10轮。\n"
                       "整组开始前记录10秒空载参考；轮间不额外等待，第10轮也完整记录60秒恢复。\n"
                       "全程保存芯片温度、压力计实测值及CH0～7的Raw、Baseline、Signal和真实状态。\n")
        process = ("后续每次：电机按压并保持3秒 → 松开并确认卸载 → 记录60秒 → 下一次。\n" if short_press else
                   f"每次：不按压，先记录 {config['baseline_wait_seconds']:g} 秒 → 电机按压并保持 → "
                   f"松开后继续记录 {values(config['recovery_seconds'])} 秒，再开始下一次。\n")
        return purpose + (
            f"共 {len(trials)} 次按压：目标 {values(config['high_forces_n'])} N × "
            f"每次保持 {values(config['high_hold_seconds'])} 秒 × 每个组合重复 {config['repeat_count']} 次。\n"
            + process +
            "保持时间从电机到位开始计算。保持期间电机位置不动，实际压力可能与目标不同、也可能逐渐下降；以压力计实测值为准。\n"
            "全程每100 ms保存一次最新读数，不做平均或中位数汇总；状态变化和控制事件另行保留。\n"
            "松开后即使读数没回到原值，也会完成记录并继续；短暂缺少数据会标记。"
            "压力或电流超过限制、电机故障、无法保存数据时，会停止按压并尝试松开。"
        )
    probes = len(config["probe_forces_n"])
    combinations = (len(config["high_forces_n"]) * len(config["high_hold_seconds"])
                    * len(config["recovery_seconds"]) * probes)
    per_round = combinations + probes * 2
    forces = lambda values: " / ".join(f"{v:g}" for v in values)
    return (
        f"共 {config['repeat_count']} 轮，每轮 {per_round} 项，共 {per_round * config['repeat_count']} 项。\n"
        f"① 每轮开始：做 {probes} 项参考测量。每项先空载 {config['baseline_wait_seconds']:g} 秒，"
        f"再测 {forces(config['probe_forces_n'])} N，作为休息后的对照。\n"
        f"② 做 {combinations} 项组合：先压 {forces(config['high_forces_n'])} N，"
        f"额外保持 {forces(config['high_hold_seconds'])} 秒；卸载后等 "
        f"{forces(config['recovery_seconds'])} 秒，再测 {forces(config['probe_forces_n'])} N，"
        f"额外观察 {config['probe_hold_seconds']:g} 秒。组合打乱顺序，减少先后顺序的影响。\n"
        f"③ 每轮结束：再做 {probes} 项同样的参考测量，看基线和低压力读数是否仍有漂移。"
        f"普通条件之间空载 {config['inter_trial_seconds']:g} 秒。\n"
        "重复多轮用来检查规律能否重复出现。保持时电机位置固定，实际压力可能下降；"
        "记录原始 counts、压力计实测值和温度，用实际时间与压力历史建立补偿模型。\n"
        "故障先回零卸载，恢复360秒后重试；已完成项保留，失败项可补测。"
        "360秒只是统一恢复时间，不代表材料一定已完全恢复。"
    )


def creep_phase_presentation(context):
    """只显示中文动作，区分普通组合和轮前、轮后的参考测量。"""
    high = float(context.get("high_force_n", 0) or 0)
    probe = float(context.get("probe_force_n", 0) or 0)
    has_high = high > 0
    steps = 7 if has_high else 4
    probe_step = 5 if has_high else 2
    phase = context.get("phase", "")
    if context.get('category') == 'paired_rebound':
        paired = {
            'creep_pre_observation': ('整组开始前 · 记录10秒初始读数', '随后开始20 N按压'),
            'creep_recovery_wait': (f"大压力已松开 · 等{context.get('recovery_seconds',0):g}秒", '计时结束后开始2 N按压'),
            'creep_inter_trial_wait': ('小压力已松开 · 休息3分钟', '持续记录；结束后开始下一个过程'),
        }
        if phase in paired:
            return paired[phase]
    if context.get("category") == "raw_rebound":
        rebound = {
            "creep_trial_start": ("开始本次按压测试", "先记录按压前的读数和实际压力"),
            "creep_pre_observation": ("第1/5步 · 不按压，记录初始读数", "计时结束后开始按压"),
            "creep_high_press": (f"第2/5步 · 按目标 {high:g} N开始按压", "电机到位后开始保持计时；实际压力以压力计为准"),
            "creep_high_hold": ("第3/5步 · 保持电机位置", "计时结束后松开，记录实际压力的变化"),
            "creep_high_release": ("第4/5步 · 松开并回到起始位置", "确认电机已松开，压力已接近零"),
            "creep_recovery_wait": ("第5/5步 · 记录松开后的读数", "记录时间结束就保存，读数没回到原值也会继续"),
        }
        if phase in rebound:
            return rebound[phase]
    entries = {
        "creep_starting": ("正在准备测试", "确认连接和释放位置"),
        "creep_initial_release": ("启动前回零卸载", "确认位置和压力归零后开始测试"),
        "creep_trial_start": ("开始当前条件", "准备高压加载" if has_high else "先空载休息，再做参考测量"),
        "creep_high_press": (f"第1/{steps}步 · 加载到 {high:g} N", "下一步：保持电机位置，观察压力变化"),
        "creep_high_hold": (f"第2/{steps}步 · 高压保持", "下一步：回零卸载"),
        "creep_high_release": (f"第3/{steps}步 · 高压后回零卸载", "下一步：空载恢复，观察大压力影响的消退"),
        "creep_recovery_wait": (f"第{4 if has_high else 1}/{steps}步 · 空载恢复", f"下一步：加载到 {probe:g} N，检查测量偏差"),
        "creep_probe_press": (f"第{probe_step}/{steps}步 · 加载到 {probe:g} N 测量", "下一步：保持并记录低压力读数"),
        "creep_probe_hold": (f"第{probe_step + 1}/{steps}步 · 低压测量保持", "下一步：回零卸载，完成本条件"),
        "creep_probe_release": (f"第{steps}/{steps}步 · 测量结束，回零卸载", "确认压力归零后保存本条件结果"),
        "creep_trial_complete": ("本条件完成，正在保存", "继续后续条件；已完成数据保留"),
        "creep_inter_trial_wait": ("条件之间 · 空载等待", "下一步：开始下一个测试条件"),
        "creep_trial_failed": ("当前尝试失败，准备卸载", "保留失败记录，先解除压力"),
        "creep_safety_release": ("异常处理 · 正在回零卸载", "确认卸载前不会继续下压"),
        "creep_safety_released": ("已确认回零并卸载", "根据故障情况恢复重试或保留断点"),
        "creep_retry_recovery": ("恢复测试前 · 空载休息", "下一步：从当前条件开头重新测试"),
        "creep_trial_skipped": ("本条件暂留待补测", "继续下一条件，已完成数据保留"),
    }
    return entries.get(phase, ("等待测试开始", "开始后显示当前步骤和等待倒计时"))


def creep_test_directory():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "long_duration_creep_tests"
    return Path(__file__).with_name("long_duration_creep_tests")


def creep_test_settings_path():
    if getattr(sys, "frozen", False):
        root = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "AlgoMonitor"
        return root / "long_duration_creep_test_config.json"
    return Path(__file__).with_name("long_duration_creep_test_config.json")


def parse_number_list(text, name, minimum, maximum):
    """Parse a comma-separated numeric factor list and reject ambiguous plans."""
    normalized = str(text).replace("，", ",").replace("；", ",").replace(";", ",")
    values = []
    for part in normalized.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            value = float(part)
        except ValueError as exc:
            raise ValueError(f"{name}包含非数字内容：{part}") from exc
        if not math.isfinite(value) or value < minimum or value > maximum:
            raise ValueError(f"{name}必须在{minimum:g}～{maximum:g}之间：{value:g}")
        rounded = round(value, 3)
        if rounded not in values:
            values.append(rounded)
    if not values:
        raise ValueError(f"{name}不能为空")
    return tuple(values)


def build_creep_plan(config):
    """Build balanced blocks with cold baselines around randomized factorial trials."""
    if config.get("protocol") == REBOUND_PROTOCOL:
        return build_rebound_plan(config)
    high_forces = tuple(config["high_forces_n"])
    probe_forces = tuple(config["probe_forces_n"])
    high_holds = tuple(config["high_hold_seconds"])
    recovery_delays = tuple(config["recovery_seconds"])
    repeat_count = int(config["repeat_count"])
    probe_hold = float(config["probe_hold_seconds"])
    baseline_wait = float(config["baseline_wait_seconds"])
    seed = int(config.get("random_seed", 20260910))
    trials = []
    for repeat_index in range(1, repeat_count + 1):
        for baseline_position in ("before",):
            for probe_force in probe_forces:
                trials.append(
                    {
                        "category": f"baseline_{baseline_position}",
                        "repeat_index": repeat_index,
                        "high_force_n": 0.0,
                        "high_hold_seconds": 0.0,
                        "recovery_seconds": baseline_wait,
                        "probe_force_n": probe_force,
                        "probe_hold_seconds": probe_hold,
                    }
                )
        block = [
            {
                "category": "history_response",
                "repeat_index": repeat_index,
                "high_force_n": high_force,
                "high_hold_seconds": high_hold,
                "recovery_seconds": recovery,
                "probe_force_n": probe_force,
                "probe_hold_seconds": probe_hold,
            }
            for high_force, high_hold, recovery, probe_force in itertools.product(
                high_forces,
                high_holds,
                recovery_delays,
                probe_forces,
            )
        ]
        random.Random(seed + repeat_index).shuffle(block)
        trials.extend(block)
        for probe_force in probe_forces:
            trials.append(
                {
                    "category": "baseline_after",
                    "repeat_index": repeat_index,
                    "high_force_n": 0.0,
                    "high_hold_seconds": 0.0,
                    "recovery_seconds": baseline_wait,
                    "probe_force_n": probe_force,
                    "probe_hold_seconds": probe_hold,
                }
            )
    for index, trial in enumerate(trials, start=1):
        trial["planned_trial_index"] = index
    return trials


def completed_trial_indices(manifest, directory):
    """优先从已提交的完成事件恢复断点，同时兼容旧清单。"""
    if int(manifest.get("schema_version", 1)) < 2:
        completed = set(range(1, int(manifest.get("next_trial_index", 1))))
    else:
        completed = set(manifest.get("completed_trial_indices", ()))
    for path in Path(directory).glob("raw_data_part_*.sqlite3"):
        connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
        try:
            for (payload,) in connection.execute(
                "SELECT payload_json FROM events WHERE event IN ('trial_complete','phase_start')"
            ):
                item = json.loads(payload)
                if item.get("phase") == "creep_trial_complete":
                    completed.add(int(item["trial_index"]))
        finally:
            connection.close()
    return completed


def find_resumable_test(root):
    """续测子会话接替父会话，避免完成子会话后又从旧父会话重复测试。"""
    sessions = []
    for path in sorted(Path(root).glob("creep_*/manifest.json"), reverse=True):
        try:
            manifest = json.loads(path.read_text(encoding="utf-8"))
            if manifest.get("status") == "start_rejected":
                continue
            sessions.append((path, manifest))
        except (OSError, ValueError):
            continue
    superseded = {str(Path(m["resumed_from"]).resolve()) for _, m in sessions if m.get("resumed_from")}
    for path, manifest in sessions:
        if str(path.parent.resolve()) in superseded or manifest.get("status") == "completed":
            continue
        done = completed_trial_indices(manifest, path.parent)
        remaining = [t for i, t in enumerate(manifest["trials"], 1) if i not in done]
        if remaining:
            return path, manifest, remaining
    return None


def _adaptive_samples_for_duration(duration_seconds, policy="recovery"):
    """Estimate samples using the phase-specific recorder policy."""
    duration = max(0.0, float(duration_seconds))
    if policy == "inter_trial":
        return duration * 2.0
    if policy == "hold":
        fast = min(duration, 10.0)
        medium = min(max(0.0, duration - 10.0), 50.0)
        slow = max(0.0, duration - 60.0)
        return fast * 100.0 + medium * 20.0 + slow * 5.0
    fast = min(duration, 5.0)
    medium = min(max(0.0, duration - 5.0), 25.0)
    slow = max(0.0, duration - 30.0)
    return fast * 100.0 + medium * 10.0 + slow * 2.0


def estimate_creep_plan(trials, inter_trial_seconds):
    """Estimate wall time, rows and disk before the user starts a long run."""
    trials = tuple(trials)
    motion_overhead_seconds = 0.0
    wait_seconds = 0.0
    estimated_samples = 0.0
    for trial in trials:
        if trial.get('category') == 'paired_rebound':
            wait = sum(float(trial[k]) for k in ('pre_observation_seconds', 'high_hold_seconds',
                       'recovery_seconds', 'probe_hold_seconds', 'post_observation_seconds'))
            motion_overhead_seconds += 16.0
            wait_seconds += wait
            estimated_samples += (wait + 16.0) / REBOUND_RECORD_INTERVAL_SECONDS
            continue
        if trial.get("category") == "raw_rebound":
            wait = sum(float(trial[k]) for k in ("pre_observation_seconds", "high_hold_seconds", "recovery_seconds"))
            motion_overhead_seconds += 8.0
            wait_seconds += wait
            estimated_samples += (wait + 8.0) / REBOUND_RECORD_INTERVAL_SECONDS
            continue
        has_high = float(trial.get("high_force_n", 0.0)) > 0.0
        motion = 24.0 if has_high else 12.0
        high_hold = float(trial.get("high_hold_seconds", 0.0))
        recovery = float(trial.get("recovery_seconds", 0.0))
        probe_hold = float(trial.get("probe_hold_seconds", 0.0))
        motion_overhead_seconds += motion
        wait_seconds += high_hold + recovery + probe_hold
        estimated_samples += motion * 100.0
        estimated_samples += _adaptive_samples_for_duration(high_hold, "hold")
        estimated_samples += _adaptive_samples_for_duration(recovery, "recovery")
        estimated_samples += _adaptive_samples_for_duration(probe_hold, "hold")
    if trials:
        gap_count = sum(t.get('category') != 'paired_rebound' for t in trials[:-1])
        gap_total = gap_count * max(0.0, float(inter_trial_seconds))
        wait_seconds += gap_total
        estimated_samples += sum(
            (inter_trial_seconds / REBOUND_RECORD_INTERVAL_SECONDS if t.get('category') == 'raw_rebound'
             else _adaptive_samples_for_duration(inter_trial_seconds, "inter_trial"))
            for t in trials[:-1] if t.get('category') != 'paired_rebound'
        )
    total_seconds = motion_overhead_seconds + wait_seconds
    estimated_rows = int(round(estimated_samples))
    estimated_bytes = estimated_rows * 760
    return {
        "trial_count": len(trials),
        "total_seconds": total_seconds,
        "estimated_rows": estimated_rows,
        "estimated_bytes": estimated_bytes,
        "motion_overhead_seconds": motion_overhead_seconds,
    }


def format_duration(seconds):
    seconds = max(0, int(round(float(seconds))))
    days, remainder = divmod(seconds, 86400)
    hours, remainder = divmod(remainder, 3600)
    minutes, secs = divmod(remainder, 60)
    parts = []
    if days:
        parts.append(f"{days}天")
    if hours or days:
        parts.append(f"{hours}小时")
    if minutes or hours or days:
        parts.append(f"{minutes}分")
    parts.append(f"{secs}秒")
    return "".join(parts)


BASE_SAMPLE_COLUMNS = (
    ("session_id", "TEXT"),
    ("captured_at", "TEXT"),
    ("sampled_monotonic", "REAL"),
    ("session_elapsed_seconds", "REAL"),
    ("trial_index", "INTEGER"),
    ("planned_trial_index", "INTEGER"),
    ("attempt_index", "INTEGER"),
    ("history_interrupted", "INTEGER"),
    ("trial_total", "INTEGER"),
    ("repeat_index", "INTEGER"),
    ("category", "TEXT"),
    ("phase", "TEXT"),
    ("phase_elapsed_seconds", "REAL"),
    ("high_force_n", "REAL"),
    ("high_hold_seconds", "REAL"),
    ("recovery_seconds", "REAL"),
    ("probe_force_n", "REAL"),
    ("probe_hold_seconds", "REAL"),
    ("sensor_sequence", "INTEGER"),
    ("sensor_sequence_gap", "INTEGER"),
    ("capture_interval_ms", "REAL"),
    ("sensor_age_ms", "REAL"),
    ("sensor_source_monotonic", "REAL"),
    ("sensor_has_raw", "INTEGER"),
    ("die_temperature_celsius", "REAL"),
    ("die_temperature_age_seconds", "REAL"),
    ("pressure_raw_n", "REAL"),
    ("pressure_control_raw_n", "REAL"),
    ("pressure_filtered_n", "REAL"),
    ("pressure_accepted", "INTEGER"),
    ("pressure_sequence", "INTEGER"),
    ("pressure_sampled_monotonic", "REAL"),
    ("pressure_age_ms", "REAL"),
    ("motor_position_counts", "INTEGER"),
    ("motor_speed_rpm", "INTEGER"),
    ("motor_position_speed_counts_s", "REAL"),
    ("motor_phase_current_ma", "INTEGER"),
    ("motor_running_state", "INTEGER"),
    ("motor_sampled_monotonic", "REAL"),
    ("motor_age_ms", "REAL"),
)
CHANNEL_SAMPLE_FIELDS = (
    ("hardware_raw", "INTEGER"),
    ("baseline", "INTEGER"),
    ("signal", "INTEGER"),
    ("fast_k_percent", "REAL"),
    ("slow_k_percent", "REAL"),
    ("state", "INTEGER"),
)
SAMPLE_COLUMNS = BASE_SAMPLE_COLUMNS + tuple(
    (f"ch{channel}_{field}", sql_type)
    for channel in range(CHANNEL_COUNT)
    for field, sql_type in CHANNEL_SAMPLE_FIELDS
)


class SQLiteCreepRecorder:
    """Write wide raw-count rows in batches and rotate bounded SQLite parts."""

    def __init__(self, root, config, max_part_mib=DEFAULT_SPLIT_MIB):
        self.session_id = uuid.uuid4().hex
        self.started_at = datetime.now().astimezone()
        self.session_started_monotonic = time.monotonic()
        stamp = self.started_at.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.session_dir = Path(root) / f"creep_{stamp}_{self.session_id[:8]}"
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.config = dict(config)
        self.max_part_bytes = max(64, int(max_part_mib)) * 1024 * 1024
        self._queue = queue.Queue(maxsize=RECORDER_QUEUE_MAX_ITEMS)
        self._stop_token = object()
        self._thread = None
        self._lock = threading.Lock()
        self._parts = []
        self._sample_count = 0
        self._event_count = 0
        self._dropped_count = 0
        self._error = None
        self._part_index = 0
        self._completed_trial_indices = set()

    def start(self):
        self._thread = threading.Thread(
            target=self._writer_loop,
            name="creep-sqlite-writer",
            daemon=True,
        )
        self._thread.start()
        return self.session_dir

    def enqueue_sample(self, row):
        self._enqueue("sample", dict(row))

    def enqueue_event(self, event, payload=None):
        row = {
            "captured_at": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "sampled_monotonic": time.monotonic(),
            "event": str(event),
            "payload_json": json.dumps(payload or {}, ensure_ascii=False),
        }
        self._enqueue("event", row)

    def _enqueue(self, kind, row):
        try:
            self._queue.put_nowait((kind, row))
        except queue.Full:
            with self._lock:
                self._dropped_count += 1
                self._error = "SQLite队列已满，数据或阶段事件丢失，需卸载并保留断点"

    def snapshot(self):
        with self._lock:
            return {
                "parts": tuple(self._parts),
                "sample_count": self._sample_count,
                "event_count": self._event_count,
                "dropped_count": self._dropped_count,
                "queue_size": self._queue.qsize(),
                "error": self._error,
                "completed_trial_indices": sorted(self._completed_trial_indices),
            }

    def stop(self, timeout_seconds=15.0):
        thread = self._thread
        if thread is None:
            return self.snapshot()
        try:
            self._queue.put(self._stop_token, timeout=1.0)
        except queue.Full:
            with self._lock:
                self._error = "SQLite写入队列已满，结束标记无法排队"
            return self.snapshot()
        thread.join(max(0.1, float(timeout_seconds)))
        if thread.is_alive():
            with self._lock:
                self._error = "SQLite后台写入在15秒内未能结束"
        self._thread = None
        return self.snapshot()

    def _open_part(self):
        self._part_index += 1
        path = self.session_dir / f"raw_data_part_{self._part_index:03d}.sqlite3"
        connection = sqlite3.connect(str(path), timeout=30.0)
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=FULL")
        connection.execute("PRAGMA temp_store=MEMORY")
        definitions = ",".join(
            f'"{name}" {sql_type}' for name, sql_type in SAMPLE_COLUMNS
        )
        connection.execute(
            f"CREATE TABLE samples (id INTEGER PRIMARY KEY AUTOINCREMENT,{definitions})"
        )
        connection.execute(
            "CREATE TABLE events (id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "captured_at TEXT,sampled_monotonic REAL,event TEXT,payload_json TEXT)"
        )
        connection.execute(
            "CREATE TABLE session_metadata (key TEXT PRIMARY KEY,value_json TEXT)"
        )
        metadata = {
            "session_id": self.session_id,
            "part_index": self._part_index,
            "started_at": self.started_at.isoformat(timespec="milliseconds"),
            "config": self.config,
            "schema_version": 3,
            "units": {
                "hardware_raw": "counts",
                "baseline": "counts",
                "signal": "counts",
                "motor_position": "counts",
                "pressure_reference": "N",
                "die_temperature": "degree Celsius",
            },
        }
        connection.executemany(
            "INSERT INTO session_metadata(key,value_json) VALUES (?,?)",
            [
                (key, json.dumps(value, ensure_ascii=False))
                for key, value in metadata.items()
            ],
        )
        connection.execute("CREATE INDEX idx_samples_trial_phase ON samples(trial_index,phase)")
        connection.commit()
        with self._lock:
            self._parts.append(path.name)
        return connection, path

    @staticmethod
    def _part_size(path):
        return sum(
            candidate.stat().st_size
            for candidate in (
                path,
                Path(str(path) + "-wal"),
                Path(str(path) + "-shm"),
            )
            if candidate.exists()
        )

    def _writer_loop(self):
        connection = None
        path = None
        sample_names = tuple(name for name, _sql_type in SAMPLE_COLUMNS)
        sample_sql = (
            "INSERT INTO samples("
            + ",".join(f'"{name}"' for name in sample_names)
            + ") VALUES ("
            + ",".join("?" for _name in sample_names)
            + ")"
        )
        event_sql = (
            "INSERT INTO events(captured_at,sampled_monotonic,event,payload_json) "
            "VALUES (?,?,?,?)"
        )
        stopping = False
        try:
            connection, path = self._open_part()
            while True:
                batch = []
                if not stopping:
                    item = self._queue.get()
                    if item is self._stop_token:
                        stopping = True
                    else:
                        batch.append(item)
                batch_deadline = time.monotonic() + 0.20
                while len(batch) < RECORDER_BATCH_SIZE:
                    try:
                        remaining = batch_deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        item = self._queue.get(timeout=remaining)
                    except queue.Empty:
                        break
                    if item is self._stop_token:
                        stopping = True
                        break
                    batch.append(item)
                sample_rows = [row for kind, row in batch if kind == "sample"]
                event_rows = [row for kind, row in batch if kind == "event"]
                if sample_rows:
                    connection.executemany(
                        sample_sql,
                        [tuple(row.get(name) for name in sample_names) for row in sample_rows],
                    )
                if event_rows:
                    connection.executemany(
                        event_sql,
                        [
                            (
                                row["captured_at"],
                                row["sampled_monotonic"],
                                row["event"],
                                row["payload_json"],
                            )
                            for row in event_rows
                        ],
                    )
                if batch:
                    connection.commit()
                    with self._lock:
                        self._sample_count += len(sample_rows)
                        self._event_count += len(event_rows)
                        for row in event_rows:
                            if row["event"] == "trial_complete":
                                self._completed_trial_indices.add(int(json.loads(row["payload_json"])["trial_index"]))
                if stopping and self._queue.empty():
                    break
                if self._part_size(path) >= self.max_part_bytes:
                    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
                    connection.close()
                    connection, path = self._open_part()
        except Exception as exc:
            with self._lock:
                self._error = f"SQLite写入失败：{exc}"
        finally:
            if connection is not None:
                try:
                    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
                    connection.commit()
                    connection.close()
                except sqlite3.Error:
                    pass


class RunningStatistics:
    def __init__(self):
        self.count = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.minimum = math.inf
        self.maximum = -math.inf

    def add(self, value):
        value = float(value)
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    def result(self):
        if not self.count:
            return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
        return {
            "count": self.count,
            "mean": self.mean,
            "std": math.sqrt(self.m2 / max(1, self.count - 1)),
            "min": self.minimum,
            "max": self.maximum,
        }


class LongDurationCreepTestWidget(QtWidgets.QWidget):
    """Configure, run and persist long-duration creep identification tests."""

    status_message = Signal(str)
    analysis_finished = Signal(object)

    def __init__(
        self,
        force_controller,
        motor_control_widget,
        sensor_sample_provider,
        parent=None,
    ):
        super().__init__(parent)
        self._force_controller = force_controller
        self._motor_control = motor_control_widget
        self._sensor_sample_provider = sensor_sample_provider
        self._running = False
        self._stopping = False
        self._recorder = None
        self._plan = []
        self._config = {}
        self._manifest = {}
        self._latest_pressure = {}
        self._device_configuration = {}
        self._pressure_lock = threading.Lock()
        self._phase_context = {"phase": "idle"}
        self._phase_started_monotonic = time.monotonic()
        self._last_sensor_sequence = None
        self._last_captured_sensor_sequence = None
        self._last_captured_monotonic = 0.0
        self._last_recorded_pressure_monotonic = None
        self._last_recorded_pressure_valid = None
        self._last_observed_states = None
        self._raw_missing_since = None
        self._phase_statistics = {}
        self._current_statistics = {}
        self._summary_path = None
        self._last_manifest_write_monotonic = 0.0
        self._runtime_started_at = None
        self._runtime_reported_at = None
        self._summary_field_names = self._make_summary_fields()
        self._plan_protocol = REBOUND_PROTOCOL
        self._template_id = None
        self._trial_quality_issues = set()
        self._analysis_busy = False
        self._analysis_closing = False
        self._analysis_output = None
        self._history_extra = set()
        self.analysis_finished.connect(self._on_analysis_finished)
        self._build_ui()
        self._capture_timer = QtCore.QTimer(self)
        self._capture_timer.setInterval(CAPTURE_TIMER_MS)
        self._capture_timer.timeout.connect(self._capture_latest_sample)
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._refresh_recorder_status)
        self._runtime_timer = QtCore.QTimer(self)
        self._runtime_timer.setInterval(250)
        self._runtime_timer.timeout.connect(self._render_runtime)
        self._force_controller.operation_progress.connect(self._on_force_progress)
        self._force_controller.operation_finished.connect(self._on_force_finished)
        motion_signal = getattr(self._force_controller, 'manual_observation_event', None)
        if motion_signal is not None:
            motion_signal.connect(self._on_motion_observation)
        self._apply_preset()
        self._load_settings()
        self._update_plan_preview()

        self.refresh_test_history()

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(9, 9, 9, 9)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("按压与恢复测试")
        title.setStyleSheet("font-size:18px;font-weight:700;color:#70e6a4;")
        layout.addWidget(title)
        explanation = WrappedLabel(
            "设置按多大压力、保持多久、松开后记录多久，以及重复几次。\n"
            "自动保存实际压力和传感器原始读数（Raw），测试结束后生成对比报告。"
        )
        explanation.setStyleSheet("color:#d7e6f5;font-size:17px;padding:4px;")
        layout.addWidget(explanation)
        self.experiment_description = WrappedLabel()
        self.experiment_description.setStyleSheet("color:#b8c7d9;font-size:16px;padding:4px;")
        layout.addWidget(self.experiment_description)

        preset_row = QtWidgets.QHBoxLayout()
        preset_row.addWidget(QtWidgets.QLabel("选择测试方案"))
        self.preset_combo = QtWidgets.QComboBox()
        self.preset_combo.addItem("比较不同压力和保持时间：18次，约2小时", "rebound")
        self.preset_combo.addItem("重复5次：2 N，保持10秒，松开后记录10分钟", "repeat_2n")
        self.preset_combo.addItem("测回弹系数：2 N，保持2/10/60秒，各3次", "single_exp")
        self.preset_combo.addItem("短按后恢复：2/10/20 N，各压3秒，松开后记录60秒，共5轮", "short_press")
        self.preset_combo.addItem("简单重复：2 N，压3秒，松开后记录60秒，共10轮", "repeat_short_2n")
        self.preset_combo.addItem("大压力后测小压力：20 N→等5秒/60秒→2 N，各压3秒，共5轮", "paired_press")
        self.preset_combo.addItem("旧方案：大压力之后再测小压力（较少组合）", "screening")
        self.preset_combo.addItem("旧方案：大压力之后再测小压力（较多组合）", "full")
        self.preset_combo.addItem("旧方案：20 N之后再测2 N和6 N", "focused")
        self.preset_combo.addItem("自己设置下面的数值", "custom")
        self.preset_combo.currentIndexChanged.connect(self._apply_preset)
        preset_row.addWidget(self.preset_combo, 1)
        layout.addLayout(preset_row)

        grid = QtWidgets.QGridLayout()
        self._parameter_grid = grid
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        self.high_force_edit = QtWidgets.QLineEdit(DEFAULT_HIGH_FORCES)
        self.probe_force_edit = QtWidgets.QLineEdit(DEFAULT_PROBE_FORCES)
        self.high_hold_edit = QtWidgets.QLineEdit(DEFAULT_HIGH_HOLDS)
        self.recovery_edit = QtWidgets.QLineEdit(DEFAULT_RECOVERY_DELAYS)
        self.repeat_input = QtWidgets.QSpinBox()
        self.repeat_input.setRange(1, 20)
        self.repeat_input.setValue(DEFAULT_REPEAT_COUNT)
        self.probe_hold_input = QtWidgets.QDoubleSpinBox()
        self.probe_hold_input.setRange(0.5, 600.0)
        self.probe_hold_input.setValue(DEFAULT_PROBE_HOLD_SECONDS)
        self.baseline_wait_input = QtWidgets.QDoubleSpinBox()
        self.baseline_wait_input.setRange(10.0, 3600.0)
        self.baseline_wait_input.setValue(DEFAULT_BASELINE_WAIT_SECONDS)
        self.inter_trial_input = QtWidgets.QDoubleSpinBox()
        self.inter_trial_input.setRange(0.0, 3600.0)
        self.inter_trial_input.setValue(DEFAULT_INTER_TRIAL_SECONDS)
        self.split_mib_input = QtWidgets.QSpinBox()
        self.split_mib_input.setRange(64, 4096)
        self.split_mib_input.setValue(DEFAULT_SPLIT_MIB)
        rows = (
            ("高压档位", self.high_force_edit, "N，逗号分隔"),
            ("探针档位", self.probe_force_edit, "N，逗号分隔"),
            ("高压额外保持", self.high_hold_edit, "秒，逗号分隔"),
            ("释放后恢复间隔", self.recovery_edit, "秒，逗号分隔"),
            ("全因子重复", self.repeat_input, "轮"),
            ("探针额外保持", self.probe_hold_input, "秒"),
            ("块前后基线等待", self.baseline_wait_input, "秒"),
            ("条件间空载间隔", self.inter_trial_input, "秒"),
            ("每个数据文件最大大小", self.split_mib_input, "MB（约）"),
        )
        self._plan_inputs = (self.preset_combo,) + tuple(widget for _, widget, _ in rows)
        self._parameter_rows = []
        for row, (label, widget, unit) in enumerate(rows):
            label_widget, unit_widget = QtWidgets.QLabel(label), QtWidgets.QLabel(unit)
            self._parameter_rows.append((label_widget, widget, unit_widget))
            grid.addWidget(label_widget, row, 0)
            grid.addWidget(widget, row, 1)
            grid.addWidget(unit_widget, row, 2)
            if hasattr(widget, "textChanged"):
                widget.textChanged.connect(self._on_plan_edited)
            elif hasattr(widget, "valueChanged"):
                widget.valueChanged.connect(self._on_plan_edited)
        layout.addLayout(grid)

        self.plan_label = WrappedLabel()
        self.plan_label.setObjectName("CreepPlanPreview")
        self.plan_label.setStyleSheet(
            "QLabel#CreepPlanPreview{background:#101923;border:1px solid #365b6f;"
            "border-radius:5px;padding:10px;color:#f4d35e;font-size:17px;font-weight:700;}"
        )
        layout.addWidget(self.plan_label)

        action_row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("开始测试")
        self.start_button.clicked.connect(self.start_test)
        self.stop_button = QtWidgets.QPushButton("停止并松开")
        self.stop_button.clicked.connect(self.stop_test)
        self.stop_button.setEnabled(False)
        self.resume_button = QtWidgets.QPushButton("接着测上次未完成的项目")
        self.resume_button.setToolTip(
            "保留已完成项；先卸载并记录360秒恢复，中断项从头重做。"
            "旧曲线不拼接，360秒不代表材料完全恢复。")
        self.resume_button.clicked.connect(self.resume_last_test)
        action_row.addWidget(self.start_button)
        action_row.addWidget(self.stop_button)
        action_row.addWidget(self.resume_button)
        # 实时状态与操作按钮固定在页顶，查看说明或参数时仍然可见。
        self.runtime_panel = QtWidgets.QFrame()
        self.runtime_panel.setObjectName("CreepRuntimePanel")
        self.runtime_panel.setStyleSheet(
            "QFrame#CreepRuntimePanel{background:#101923;border:1px solid #365b6f;border-radius:8px;}"
        )
        runtime = QtWidgets.QVBoxLayout(self.runtime_panel)
        runtime.setContentsMargins(14, 10, 14, 10)
        runtime.setSpacing(5)
        self.round_label = WrappedLabel("测试尚未开始")
        self.round_label.setStyleSheet("font-size:18px;color:#b8c7d9;font-weight:600;")
        self.stage_label = WrappedLabel("等待开始")
        self.stage_label.setStyleSheet("font-size:26px;color:#70e6a4;font-weight:700;")
        self.countdown_label = WrappedLabel("开始后显示当前步骤和等待倒计时")
        self.countdown_label.setStyleSheet("font-size:28px;color:#ffd166;font-weight:700;")
        self.condition_label = WrappedLabel()
        self.condition_label.setStyleSheet("font-size:16px;color:#d7e6f5;")
        self.next_step_label = WrappedLabel()
        self.next_step_label.setStyleSheet("font-size:16px;color:#9fc3d9;")
        self.elapsed_label = WrappedLabel()
        self.elapsed_label.setStyleSheet("font-size:15px;color:#b8c7d9;")
        for label in (self.round_label, self.stage_label, self.countdown_label,
                      self.condition_label, self.next_step_label, self.elapsed_label):
            runtime.addWidget(label)

        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 1000)
        self.progress_bar.setValue(0)
        self.progress_bar.setMinimumHeight(22)
        self.progress_bar.setStyleSheet("font-size:15px;")
        runtime.addWidget(self.progress_bar)
        self.phase_label = WrappedLabel()
        self.phase_label.setStyleSheet("color:#8bd5ff;font-size:16px;font-weight:600;")
        runtime.addWidget(self.phase_label)
        for button in (self.start_button, self.stop_button, self.resume_button):
            button.setMinimumHeight(40)
            button.setStyleSheet("font-size:16px;font-weight:600;")
        runtime.addLayout(action_row)
        self.recorder_label = WrappedLabel("测试开始后会自动保存数据")
        self.recorder_label.setStyleSheet("color:#b8c7d9;")
        layout.addWidget(self.recorder_label)
        analysis_group = QtWidgets.QGroupBox("测试数据分析")
        analysis_layout = QtWidgets.QVBoxLayout(analysis_group)
        self.auto_analysis_check = QtWidgets.QCheckBox("测试完成后自动计算关键指标")
        self.auto_analysis_check.setChecked(True)
        analysis_layout.addWidget(self.auto_analysis_check)
        analysis_layout.addWidget(WrappedLabel(
            "每次测试单独保存；重新计算会另存一份报告。可选择以前的测试和报告直接查看。"))
        history_row = QtWidgets.QHBoxLayout()
        self.test_history_combo = QtWidgets.QComboBox()
        self.test_history_combo.setMinimumWidth(0)
        self.test_history_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.test_history_combo.setMinimumContentsLength(24)
        self.test_history_combo.currentIndexChanged.connect(self._on_history_selected)
        self.refresh_history_button = QtWidgets.QPushButton("刷新列表")
        self.refresh_history_button.clicked.connect(lambda: self.refresh_test_history())
        self.browse_history_button = QtWidgets.QPushButton("从文件夹读取")
        self.browse_history_button.clicked.connect(self.browse_saved_test)
        history_row.addWidget(QtWidgets.QLabel("选择测试"))
        history_row.addWidget(self.test_history_combo, 1)
        history_row.addWidget(self.refresh_history_button)
        history_row.addWidget(self.browse_history_button)
        analysis_layout.addLayout(history_row)
        report_row = QtWidgets.QHBoxLayout()
        self.report_history_combo = QtWidgets.QComboBox()
        self.report_history_combo.currentIndexChanged.connect(self._on_report_selected)
        report_row.addWidget(QtWidgets.QLabel("选择报告"))
        report_row.addWidget(self.report_history_combo, 1)
        analysis_layout.addLayout(report_row)
        self.analysis_label = WrappedLabel(
            "自动计算实测力、保持时长、四通道固定时刻残差、Raw返回与IDLE时间。"
            "续测会关联旧数据，时间段不拼接；模型系数需另行验证。")
        self.analysis_label.setStyleSheet("font-size:15px;color:#b8c7d9;")
        analysis_layout.addWidget(self.analysis_label)
        analysis_actions = QtWidgets.QHBoxLayout()
        self.analyze_button = QtWidgets.QPushButton("为所选测试生成新报告")
        self.analyze_button.clicked.connect(self.analyze_selected_test)
        self.open_analysis_button = QtWidgets.QPushButton("打开所选报告")
        self.open_analysis_button.setEnabled(False)
        self.open_analysis_button.clicked.connect(self.open_analysis_report)
        analysis_actions.addWidget(self.analyze_button)
        analysis_actions.addWidget(self.open_analysis_button)
        self.open_data_button = QtWidgets.QPushButton("打开数据文件夹")
        self.open_data_button.clicked.connect(self.open_test_directory)
        analysis_actions.addWidget(self.open_data_button)
        analysis_layout.addLayout(analysis_actions)
        layout.addWidget(analysis_group)
        layout.addStretch(1)
        scroll.setWidget(content)
        outer.addWidget(self.runtime_panel)
        outer.addWidget(scroll)

    def _on_plan_edited(self, *_args):
        if self._template_id == PAIRED_PRESS_TEMPLATE:
            self._update_plan_preview()
            return
        self._template_id = None
        if self.preset_combo.currentData() != "custom":
            self.preset_combo.blockSignals(True)
            self.preset_combo.setCurrentIndex(self.preset_combo.findData("custom"))
            self.preset_combo.blockSignals(False)
        self._update_parameter_labels()
        self._update_plan_preview()

    def _apply_preset(self, *_args):
        preset = self.preset_combo.currentData()
        if preset == "custom":
            if self._template_id == PAIRED_PRESS_TEMPLATE:
                self._template_id = None
                self._update_parameter_labels()
                self._update_plan_preview()
            return
        self._template_id = {"single_exp": SINGLE_EXP_TEMPLATE, "short_press": SHORT_PRESS_TEMPLATE,
                             "repeat_short_2n": REPEAT_SHORT_2N_TEMPLATE,
                             "paired_press": PAIRED_PRESS_TEMPLATE}.get(preset)
        self._plan_protocol = REBOUND_PROTOCOL if preset in ("rebound", "repeat_2n", "single_exp", "short_press", "repeat_short_2n", "paired_press") else "history_response_v1"
        widgets = (
            self.high_force_edit,
            self.probe_force_edit,
            self.high_hold_edit,
            self.recovery_edit,
            self.repeat_input,
            self.baseline_wait_input,
            self.inter_trial_input,
            self.probe_hold_input,
        )
        for widget in widgets:
            widget.blockSignals(True)
        if preset == "paired_press":
            self.high_force_edit.setText('20')
            self.probe_force_edit.setText('2')
            self.high_hold_edit.setText('3')
            self.probe_hold_input.setValue(3)
            self.recovery_edit.setText('5,60')
            self.repeat_input.setValue(5)
            self.baseline_wait_input.setValue(10)
            self.inter_trial_input.setValue(180)
        elif preset in ("short_press", "repeat_short_2n"):
            settings = REPEAT_SHORT_2N_SETTINGS if preset == "repeat_short_2n" else SHORT_PRESS_SETTINGS
            self.high_force_edit.setText(','.join(f'{v:g}' for v in settings['high_forces_n']))
            self.high_hold_edit.setText('3')
            self.recovery_edit.setText('60')
            self.repeat_input.setValue(settings['repeat_count'])
            self.baseline_wait_input.setValue(settings['baseline_wait_seconds'])
            self.inter_trial_input.setValue(0)
        elif preset == "single_exp":
            self.high_force_edit.setText('2')
            self.high_hold_edit.setText('2,10,60')
            self.recovery_edit.setText('600')
            self.repeat_input.setValue(SINGLE_EXP_SETTINGS['repeat_count'])
            self.baseline_wait_input.setValue(SINGLE_EXP_SETTINGS['baseline_wait_seconds'])
            self.inter_trial_input.setValue(0)
        elif preset in ("rebound", "repeat_2n"):
            fixed = preset == "repeat_2n"
            self.high_force_edit.setText("2" if fixed else "1,2,10")
            self.high_hold_edit.setText("10" if fixed else "2,10,30")
            self.recovery_edit.setText("600" if fixed else "360")
            self.repeat_input.setValue(5 if fixed else 2)
            self.baseline_wait_input.setValue(30)
            self.inter_trial_input.setValue(0)
        elif preset == "screening":
            self.high_force_edit.setText(DEFAULT_HIGH_FORCES)
            self.probe_force_edit.setText(DEFAULT_PROBE_FORCES)
            self.high_hold_edit.setText(DEFAULT_HIGH_HOLDS)
            self.recovery_edit.setText(DEFAULT_RECOVERY_DELAYS)
            self.repeat_input.setValue(DEFAULT_REPEAT_COUNT)
            self.baseline_wait_input.setValue(DEFAULT_BASELINE_WAIT_SECONDS)
            self.inter_trial_input.setValue(DEFAULT_INTER_TRIAL_SECONDS)
        elif preset == "focused":
            self.high_force_edit.setText("20")
            self.probe_force_edit.setText("2,6")
            self.high_hold_edit.setText("1,10,60")
            self.recovery_edit.setText("1,10,120,360")
            self.repeat_input.setValue(2)
            self.baseline_wait_input.setValue(360.0)
            self.inter_trial_input.setValue(10.0)
        else:
            self.high_force_edit.setText("12,16,20")
            self.probe_force_edit.setText("2,6,10")
            self.high_hold_edit.setText("1,10,60")
            self.recovery_edit.setText("1,10,120,360")
            self.repeat_input.setValue(3)
            self.baseline_wait_input.setValue(360.0)
            self.inter_trial_input.setValue(10.0)
        for widget in widgets:
            widget.blockSignals(False)
        self._update_parameter_labels()
        self._update_plan_preview()

    def _update_parameter_labels(self):
        rebound = self._plan_protocol == REBOUND_PROTOCOL
        paired = self._template_id == PAIRED_PRESS_TEMPLATE
        names = {0: "目标压力" if rebound else "先施加的压力", 1: "之后再测的压力",
                 2: "每次保持多久", 3: "松开后记录多久" if rebound else "松开后等多久再按压",
                 4: "每个组合重复几次", 5: "第二次按压保持多久",
                 6: "每次按压前先记录" if rebound else "每轮前后先休息多久",
                 7: "两次测试之间休息多久", 8: "单文件大小上限"}
        if self._template_id in CONTINUOUS_SHORT_PRESS_TEMPLATES:
            names[6] = "整组开始前先记录"
        if paired:
            names.update({0: '先施加的大压力', 1: '之后施加的小压力', 2: '大压力保持多久',
                          3: '松开大压力后等多久', 4: '两种过程一起重复几轮', 5: '小压力保持多久',
                          6: '整组开始前先记录', 7: '过程之间、轮之间休息'})
        units = {0: "N", 1: "N", 4: "次", 8: "MB"}
        while self._parameter_grid.count():
            self._parameter_grid.takeAt(0)
        visible_index = 0
        for index, widgets in enumerate(self._parameter_rows):
            label, editor, unit_label = widgets
            label.setText(f"{names[index]}（{units.get(index, '秒')}）")
            visible = paired or not rebound or index not in (1, 5, 7)
            label.setVisible(visible)
            editor.setVisible(visible)
            unit_label.hide()
            if visible:
                row, column = divmod(visible_index, 2)
                self._parameter_grid.addWidget(label, row, column * 2)
                self._parameter_grid.addWidget(editor, row, column * 2 + 1)
                visible_index += 1
        self.high_force_edit.setToolTip("填一个或多个目标压力，如2或1,2,10。多个数字用逗号分隔。实际压力以压力计测量为准。")
        self.high_hold_edit.setToolTip("从电机到位开始计时。可填10，也可填2,10,30；期间保持电机位置，实际压力可能变化。")
        self.recovery_edit.setToolTip("确认松开后继续记录的时间。600秒就是10分钟；不会因为读数提前恢复而提前结束。")
        if paired:
            self.recovery_edit.setToolTip('从20 N确认卸载开始计时，到时再施加2 N。过程1等5秒，过程2等60秒。')
        self.repeat_input.setToolTip("每种压力、保持时间、松开后记录时长的组合各做几次。")
        self.split_mib_input.setToolTip("数据文件达到这个大小后自动新建下一个文件，旧数据保留。")

    def _read_config(self):
        config = {
            "protocol": self._plan_protocol,
            "high_forces_n": parse_number_list(
                self.high_force_edit.text(), "目标压力", 0.1, 20.0
            ),
            "probe_forces_n": parse_number_list(
                self.probe_force_edit.text(), "之后再测的压力", 2.0, 20.0
            ),
            "high_hold_seconds": parse_number_list(
                self.high_hold_edit.text(), "每次保持时间", 0.0, 600.0
            ),
            "recovery_seconds": parse_number_list(
                self.recovery_edit.text(), "松开后记录时间", 0.0, 3600.0
            ),
            "repeat_count": self.repeat_input.value(),
            "probe_hold_seconds": self.probe_hold_input.value(),
            "baseline_wait_seconds": self.baseline_wait_input.value(),
            "inter_trial_seconds": self.inter_trial_input.value(),
            "sqlite_split_mib": self.split_mib_input.value(),
            "random_seed": 20260910,
            "capture_policy": (REBOUND_RECORDING_SETTINGS['capture_policy'] if self._plan_protocol == REBOUND_PROTOCOL else (
                "latest-frame sampling, motion <=100Hz; hold 100Hz<=10s, 20Hz<=60s, 5Hz thereafter; "
                "recovery 100Hz<=5s, 10Hz<=30s, 2Hz thereafter; inter-trial 2Hz"
            )),
        }
        if self._plan_protocol == REBOUND_PROTOCOL:
            config.update(REBOUND_RECORDING_SETTINGS)
        if self._template_id == PAIRED_PRESS_TEMPLATE:
            config['template_id'] = PAIRED_PRESS_TEMPLATE
        if (self._template_id in CONTINUOUS_SHORT_PRESS_TEMPLATES
                and matches_short_press_settings(dict(config, template_id=self._template_id))):
            config['template_id'] = self._template_id
            config['observation_goal'] = {
                'checkpoints_seconds': [1,5,10,30,60], 'acceptance_threshold': None,
                'reference_policy': 'session_last_10s_before_press',
                'compare_session_start': True,
            }
        if self._template_id == SINGLE_EXP_TEMPLATE and matches_single_exp_settings(config):
            config['template_id'] = SINGLE_EXP_TEMPLATE
            config['calibration_goal'] = {
                'model': 'R=k*D*(1-exp(-T/tau))*exp(-t/tau)',
                'channels': [4,5,6,7], 'parameters_per_channel': ['k','tau_seconds'],
                'time_zero': 'release_zero_confirmed_monotonic',
                'fit_repeats': [1,2], 'validation_repeats': [3],
                'split_fast_slow': False, 'fit_status': 'not_fitted',
                'raw_reference': 'median of last 10 seconds before press',
                'D': 'median of last min(10s,actual hold) seconds before release minus raw_reference',
            }
        return config

    def _update_plan_preview(self):
        try:
            config = self._read_config()
            trials = build_creep_plan(config)
            estimate = estimate_creep_plan(trials, config["inter_trial_seconds"])
        except ValueError as exc:
            self.plan_label.setText(f"计划无效：{exc}")
            self.experiment_description.setText("请填写有效参数，页面会自动说明每轮的测试内容。")
            self.start_button.setEnabled(False)
            return
        gib = estimate["estimated_bytes"] / (1024 ** 3)
        self.experiment_description.setText(describe_creep_plan(config))
        timing_note = ("已包含计划中的按压前记录时间，另按每次约8秒估算电机动作；实际用时可能略有变化。"
                       if config.get('protocol') == REBOUND_PROTOCOL else
                       "运动到位时间按每个高压/探针组合约24秒估算，实际总时长会随电机、压力计和通信速度变化。")
        count_text = f"共{estimate['trial_count']}次"
        if config.get('template_id') == PAIRED_PRESS_TEMPLATE:
            count_text = '5轮，共10个过程、20次按压'
            timing_note = '已包含9段3分钟休息；另按每次按压与松开约8秒估算动作时间。'
        self.plan_label.setText(
            f"计划：{count_text}，预计约{format_duration(estimate['total_seconds'])}；"
            f"预计保存{estimate['estimated_rows']:,}条读数，约{gib:.2f} GB。"
            + timing_note
        )
        self.start_button.setEnabled(not self._running)

    def _render_runtime(self):
        """刷新展示倒计时，不改动控制器状态或采样阶段。"""
        if not self._running:
            return
        now = time.monotonic()
        context = self._phase_context
        phase = context.get("phase", "")
        index = int(context.get("trial_index", 0) or 0)
        total = len(self._plan)
        repeat = int(context.get("repeat_index", 0) or 0)
        repeats = int(self._config.get("repeat_count", 0))
        attempt = int(context.get("attempt_index", 0) or 0)
        category = {
            "raw_rebound": "按压与松开记录",
            "paired_rebound": "大压力后测小压力",
            "baseline_before": "轮前参考测量", "baseline_after": "轮后参考测量",
            "history_response": "压力历史测试",
        }.get(context.get("category"), "准备测试")
        prefix = f"第 {repeat}/{repeats} 轮 · " if repeat else ""
        if context.get('category') == 'paired_rebound':
            prefix += f"过程{context.get('process_index', 0)} · "
        if context.get("category") == "raw_rebound":
            prefix = ""
        suffix = f" · 第 {attempt} 次尝试" if attempt > 1 else ""
        planned = context.get("planned_trial_index")
        origin = f"（原计划第 {planned} 项）" if planned and int(planned) != index else ""
        self.round_label.setText(f"{prefix}本次第 {index}/{total} 项{origin} · {category}{suffix}")
        title, next_step = creep_phase_presentation(context)
        self.stage_label.setText(title)
        attention = phase in ("creep_trial_failed", "creep_trial_skipped", "creep_safety_release", "creep_retry_recovery")
        color = "#ffd166" if attention or self._stopping else "#70e6a4"
        self.stage_label.setStyleSheet(f"font-size:26px;color:{color};font-weight:700;")
        self.next_step_label.setText(next_step)
        high = float(context.get("high_force_n", 0) or 0)
        probe = float(context.get("probe_force_n", 0) or 0)
        if context.get("category") == "raw_rebound":
            pressure = self._pressure_snapshot()
            actual = pressure.get("raw_force_n")
            sampled_at = pressure.get("sampled_monotonic")
            fresh = (pressure.get("accepted") and actual is not None and sampled_at is not None
                     and 0 <= now-float(sampled_at) <= 2)
            actual_text = f"压力计当前读数 {float(actual):.2f} N" if fresh else "压力计当前读数：等待更新"
            self.condition_label.setText(
                f"目标压力 {high:g} N · 保持 {float(context.get('high_hold_seconds',0)):g} 秒 · "
                f"松开后记录 {float(context.get('recovery_seconds',0)):g} 秒\n{actual_text}"
            )
        elif probe:
            prelude = (f"先压 {high:g} N，保持 {float(context.get('high_hold_seconds', 0)):g} 秒 → "
                       if high > 0 else "参考测量：")
            self.condition_label.setText(
                f"{prelude}空载 {float(context.get('recovery_seconds', 0)):g} 秒 → "
                f"测 {probe:g} N，保持 {float(context.get('probe_hold_seconds', 0)):g} 秒"
            )
        else:
            self.condition_label.setText("")
        duration = context.get("phase_duration_seconds")
        since_report = max(0.0, now - (self._runtime_reported_at or now))
        elapsed = max(0.0, float(context.get("phase_elapsed_seconds", 0) or 0) + since_report)
        if duration is not None:
            duration = max(0.0, float(duration))
            remaining = max(0.0, duration - elapsed)
            if since_report > 3:
                text = "状态更新延迟，等待控制器确认"
            elif remaining <= 0:
                text = "计时已到，等待控制器切换步骤"
            else:
                verb = "保持" if phase in ("creep_high_hold", "creep_probe_hold") else "等待"
                text = f"还需{verb} {format_duration(math.ceil(remaining))}"
            self.countdown_label.setText(text)
            self.next_step_label.setText(
                f"本步已用 {format_duration(min(int(elapsed), int(duration)))} / "
                f"计划 {format_duration(duration)} · {next_step}"
            )
        else:
            self.countdown_label.setText(f"本步已用 {format_duration(int(elapsed))} · 等待动作确认")
        if self._stopping and phase not in ("creep_safety_release", "creep_safety_released"):
            self.stage_label.setText("正在停止 · 等待回零卸载")
            self.countdown_label.setText("确认卸载后结束测试")
            self.next_step_label.setText("正在中止当前条件；已完成数据保留")
        done = int(self._manifest.get("completed_trials", 0))
        pending = len(self._manifest.get("skipped_trial_indices", ()))
        run_elapsed = max(0.0, now - (self._runtime_started_at or now))
        self.elapsed_label.setText(
            f"本次已运行 {format_duration(int(run_elapsed))} · 已保存 {done}/{total} 项 · 待补测 {pending} 项"
        )

    def _save_settings(self):
        try:
            config = self._read_config()
            path = creep_test_settings_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(path)
        except (OSError, ValueError):
            pass

    def _load_settings(self):
        path = creep_test_settings_path()
        try:
            config = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            return
        # 旧页参数不覆盖新的两小时默认计划；旧会话仍可从断点按原计划续测。
        if config.get("protocol") != REBOUND_PROTOCOL:
            return
        mapping = (
            (self.high_force_edit, "high_forces_n"),
            (self.probe_force_edit, "probe_forces_n"),
            (self.high_hold_edit, "high_hold_seconds"),
            (self.recovery_edit, "recovery_seconds"),
        )
        for widget, key in mapping:
            values = config.get(key)
            if isinstance(values, (list, tuple)) and values:
                widget.setText(",".join(f"{float(value):g}" for value in values))
        for widget, key in (
            (self.repeat_input, "repeat_count"),
            (self.probe_hold_input, "probe_hold_seconds"),
            (self.baseline_wait_input, "baseline_wait_seconds"),
            (self.inter_trial_input, "inter_trial_seconds"),
            (self.split_mib_input, "sqlite_split_mib"),
        ):
            if key in config:
                widget.setValue(config[key])
        self.preset_combo.setCurrentIndex(self.preset_combo.findData("custom"))
        if (config.get('template_id') == PAIRED_PRESS_TEMPLATE
                and matches_paired_press_settings(self._read_config())):
            self._template_id = PAIRED_PRESS_TEMPLATE
            self.preset_combo.blockSignals(True)
            self.preset_combo.setCurrentIndex(self.preset_combo.findData('paired_press'))
            self.preset_combo.blockSignals(False)
            self._update_parameter_labels()
            self._update_plan_preview()
        if (config.get('template_id') in CONTINUOUS_SHORT_PRESS_TEMPLATES
                and matches_short_press_settings(dict(self._read_config(), template_id=config['template_id']))):
            self._template_id = config['template_id']
            self.preset_combo.blockSignals(True)
            preset = 'repeat_short_2n' if self._template_id == REPEAT_SHORT_2N_TEMPLATE else 'short_press'
            self.preset_combo.setCurrentIndex(self.preset_combo.findData(preset))
            self.preset_combo.blockSignals(False)
            self._update_parameter_labels()
            self._update_plan_preview()
        if (config.get('template_id') == SINGLE_EXP_TEMPLATE
                and matches_single_exp_settings(self._read_config())):
            # 恢复模板身份而不覆盖用户保存的文件大小等其他选项。
            self._template_id = SINGLE_EXP_TEMPLATE
            self.preset_combo.blockSignals(True)
            self.preset_combo.setCurrentIndex(self.preset_combo.findData('single_exp'))
            self.preset_combo.blockSignals(False)
            self._update_plan_preview()

    @Slot(object)
    def set_pressure_sample(self, sample):
        with self._pressure_lock:
            self._latest_pressure = dict(sample or {})
        if self._running and self._recorder is not None and self._config.get("protocol") == REBOUND_PROTOCOL:
            now = time.monotonic()
            valid = bool((sample or {}).get('accepted')) and not (sample or {}).get('invalidate', False)
            if (self._last_recorded_pressure_monotonic is not None
                    and now - self._last_recorded_pressure_monotonic < REBOUND_RECORD_INTERVAL_SECONDS - 1e-9
                    and valid == self._last_recorded_pressure_valid):
                return
            self._last_recorded_pressure_monotonic = now
            self._last_recorded_pressure_valid = valid
            self._recorder.enqueue_event("pressure_sample", dict(
                sample or {}, trial_index=self._phase_context.get("trial_index"),
                phase=self._phase_context.get("phase"), captured_monotonic=now))

    def _pressure_snapshot(self):
        with self._pressure_lock:
            return dict(self._latest_pressure)

    def _on_motion_observation(self, event):
        if self._running and self._recorder is not None and event.get('mode') == 'creep_test':
            self._recorder.enqueue_event('motor_timing', dict(event,
                trial_index=self._phase_context.get('trial_index'), phase=self._phase_context.get('phase')))

    def record_device_configuration(self, kind, frame):
        """保留固件实际诊断配置，切档时另记事件，不用界面设定值冒充实测参数。"""
        entry = dict(frame=frame, received_monotonic=time.monotonic())
        self._device_configuration[str(kind)] = entry
        if self._running and self._recorder is not None:
            self._recorder.enqueue_event('device_configuration', dict(entry, kind=kind,
                trial_index=self._phase_context.get('trial_index')))

    def _confirm_start(self, estimate, resumed=False):
        gib = estimate["estimated_bytes"] / (1024 ** 3)
        prefix = "续测" if resumed else "测试"
        sample = self._sensor_sample_provider() or {}
        temperature = sample.get("die_temperature_celsius")
        temperature_age = sample.get("die_temperature_age_seconds")
        temperature_available = (
            temperature is not None
            and temperature_age is not None
            and float(temperature_age) <= 3.0
        )
        temperature_text = (
            f"当前芯片温度帧正常（{float(temperature):.1f} °C），将同步记录。"
            if temperature_available
            else "当前固件没有可用芯片温度帧；测试仍可运行，但温漂与蠕变的区分置信度会降低。"
        )
        message = (
            f"本次{prefix}包含 {estimate['trial_count']} 个剩余条件。\n"
            f"预计运行约 {format_duration(estimate['total_seconds'])}，"
            f"保存约 {estimate['estimated_rows']:,} 条读数，占用空间约 {gib:.2f} GB。\n\n"
            f"{temperature_text}\n\n"
            "异常会先回零卸载，空载恢复360秒后重试本条件（最多3次）；失败条件保留待补测。"
            "续测保留已完成项，先回零并记录360秒恢复，再从头重做中断项。"
            "中断前后曲线不拼接；360秒不代表材料完全恢复，空档期间的受力历史未知。\n"
            "实际时长会因电机到位、卸载确认和故障恢复增加。开始后请保持压力计、CAN和传感器串口连接，"
            "并确认磁盘空间与本次设置的力档。是否开始？"
        )
        standard_button = getattr(
            QtWidgets.QMessageBox,
            "StandardButton",
            QtWidgets.QMessageBox,
        )
        yes_button = getattr(QtWidgets.QMessageBox, "Yes", standard_button.Yes)
        no_button = getattr(QtWidgets.QMessageBox, "No", standard_button.No)
        answer = QtWidgets.QMessageBox.question(
            self,
            "确认开始按压与恢复测试",
            message,
            yes_button | no_button,
            no_button,
        )
        return answer == yes_button

    @Slot()
    def start_test(self):
        if self._running:
            return
        try:
            config = self._read_config()
            trials = build_creep_plan(config)
        except ValueError as exc:
            self.phase_label.setText(f"计划无效：{exc}")
            return
        estimate = estimate_creep_plan(trials, config["inter_trial_seconds"])
        if not self._confirm_start(estimate):
            return
        self._start_plan(config, trials, estimate)

    def _start_plan(self, config, trials, estimate, resumed_from=None):
        # 续测创建新的记录段，并明确保存本次软件实际采用的记录频率。
        config = dict(config)
        if config.get('protocol') == REBOUND_PROTOCOL:
            config.update(REBOUND_RECORDING_SETTINGS)
        sample = self._sensor_sample_provider()
        if (
            not isinstance(sample, dict)
            or not sample.get("has_raw")
            or int(sample.get("channel_count", 0)) < CHANNEL_COUNT
            or float(sample.get("age_seconds", math.inf)) > 0.5
        ):
            self.phase_label.setText(
                "无法启动：需要8通道Debug原始帧；请在波形设置中把Output切换为Debug"
            )
            return
        if not self._motor_control.is_can_connected():
            self.phase_label.setText("无法启动：CAN未连接")
            return
        pressure = self._pressure_snapshot()
        if not pressure.get("accepted"):
            self.phase_label.setText("无法启动：压力计没有有效实时读数")
            return
        self._save_settings()
        try:
            recorder = SQLiteCreepRecorder(
                creep_test_directory(), config, config["sqlite_split_mib"]
            )
            output_dir = recorder.start()
        except OSError as exc:
            self.phase_label.setText(f"无法创建测试目录：{exc}")
            return
        self._recorder = recorder
        self._plan = [dict(trial) for trial in trials]
        self._config = dict(config)
        self._summary_path = output_dir / "trial_summary.csv"
        self._write_dataset_readme(output_dir)
        self._manifest = {
            "schema_version": 3,
            "session_id": recorder.session_id,
            "status": "running",
            "created_at": recorder.started_at.isoformat(timespec="milliseconds"),
            "resumed_from": resumed_from,
            "resume_context": ({
                "restart_incomplete_trial": True,
                "first_planned_trial_index": self._plan[0].get("planned_trial_index", 1),
                "recovery_seconds": CREEP_RECOVERY_SECONDS,
                "gap_force_history": "unknown",
                "initial_material_state": "unknown",
                "time_axis_policy": "new_segment_do_not_stitch",
            } if resumed_from else None),
            "config": config,
            "initial_sensor_snapshot": sample,
            "device_configuration": self._device_configuration.copy(),
            "force_controller_settings": dict(getattr(self._force_controller, '_settings_snapshot', {})),
            "estimate": estimate,
            "trials": self._plan,
            "next_trial_index": 1,
            "completed_trials": 0,
            "completed_trial_indices": [],
            "skipped_trial_indices": [],
            "output_parts": [],
            "die_temperature_available_at_start": (
                sample.get("die_temperature_celsius") is not None
                and sample.get("die_temperature_age_seconds") is not None
                and float(sample["die_temperature_age_seconds"]) <= 3.0
            ),
        }
        if not self._write_manifest():
            recorder.stop()
            self._recorder = None
            return
        self._running = True
        for widget in self._plan_inputs:
            widget.setEnabled(False)
        self._stopping = False
        self._phase_context = {"phase": "creep_starting", "trial_index": 0}
        self._phase_started_monotonic = time.monotonic()
        self._runtime_started_at = self._phase_started_monotonic
        self._runtime_reported_at = self._phase_started_monotonic
        self._last_sensor_sequence = None
        self._last_captured_sensor_sequence = None
        self._last_captured_monotonic = 0.0
        self._last_recorded_pressure_monotonic = None
        self._last_recorded_pressure_valid = None
        self._last_observed_states = None
        self._raw_missing_since = None
        self._trial_quality_issues = set()
        self._phase_statistics = {}
        self._current_statistics = {}
        self.start_button.setEnabled(False)
        self.resume_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self._capture_timer.start()
        self._status_timer.start()
        self.progress_bar.setValue(0)
        self.phase_label.setText("")
        self._runtime_timer.start()
        self._render_runtime()
        plan = {
            "protocol": config.get("protocol"),
            "trials": self._plan,
            "repeat_count": config["repeat_count"],
            "inter_trial_seconds": config["inter_trial_seconds"],
            "resumed_from": resumed_from,
        }
        recorder.enqueue_event("session_start", self._manifest)
        if not self._force_controller.start_creep_test(plan):
            reason_provider = getattr(
                self._force_controller,
                "last_status_message",
                None,
            )
            reason = reason_provider() if callable(reason_provider) else ""
            message = (
                f"底层按力控制拒绝启动：{reason}"
                if reason
                else "底层按力控制拒绝启动，但没有返回具体原因"
            )
            self._finish_session("start_rejected", message)
            return
        self.status_message.emit(f"长时蠕变测试已启动：{output_dir}")

    @staticmethod
    def _write_dataset_readme(output_dir):
        text = """# Raw回弹采集数据说明（兼容旧历史效应会话）

先看 manifest.json 的 config.protocol。raw_rebound_v1 表示新单次按压计划：
每次空载记录 → 查表加载一次 → 到位保持 → 固定位置释放 → 完整观察。
high_force_n为目标力，high_hold_seconds为到位保持时长，probe_force_n=0；
pre_observation_seconds为按压前窗口，recovery_seconds为释放确认后的完整观察窗口。
例外：category=paired_rebound是大压力后测小压力，一个项目含两次按压。
high_force_n/high_hold_seconds是大压力条件，probe_force_n/probe_hold_seconds是小压力条件；
recovery_seconds是大压力确认卸载后、再次按压前的5/60秒间隔；post_observation_seconds是
小压力卸载后的休息记录（通常180秒，整组末项为0）。paired_results.high/probe分别保存两次时刻和快照。
repeat_index为轮次、process_index为过程1/2。两次按压及该项目的休息完整结束才提交完成，中断后整项重做。
creep_trial_complete仅表示计划窗口已完成，不表示Raw已恢复。

- manifest.json：配置、随机顺序、断点、初始Raw、设备诊断配置、控制设置与分卷列表。
- raw_data_part_*.sqlite3 的 samples：八通道hardware_raw/baseline/signal/state等，
  实际力、温度、位置/速度/电流、数据年龄与序号。每10ms检查最新帧，每100ms保存一次，重复帧不重复保存；
  新回弹测试全程不按阶段降采样。不是串口逐帧无损采集；实际频率看时间戳与序号。
- samples：新回弹测试全程每100ms直接保存最新有效帧，不求中位数或平均；无新帧不补造记录。
- events：pressure_sample每100ms保存最新到达的压力读数，有效性变化立即保存；即使Raw暂缺仍可记录压力。
  sensor_state_change保留轮询观察到的真实状态变化及源时间，不保证捕获两次轮询之间发生的变化。
  motor_timing保存实际发令事件和发令前快照；phase_start保存各阶段时刻，
  trial_complete保存机械到位、释放发令/到位/零点确认时刻、前后Raw快照与质量标记。
- device_configuration事件保存设备信息、实际三参数和自动量程诊断（以设备实际提供为准）。
  未收到的配置不能假定已知，结合manifest中的初始诊断与后续事件使用。
- raw_gap_start/raw_gap_end、sensor_sequence_reset记录数据缺口；绝不补造Raw=0。
- trial_summary.csv：每次完成后的分阶段统计，包含CH4～7各自的Raw/Baseline/Signal，
  data_quality_issues_json标记缺口。精确固定时刻残余和返回时间应从samples离线计算。

观察期间不等待IDLE或Raw返回，不因实际力与目标偏差而追加施力。
释放位置与连续3个新鲜±0.10 N压力读数用于确认卸载；下压前再次检查零点。
保持时间从机械到位后计时，观察窗口从释放确认后计时；加载和卸载过程也一直录制。
建模的t=0应根据motor_timing、释放时刻和实际力波形选取，不能只按phase标签猜测。
原始帧缺失只标记并继续；空载时压力计缺帧不终止记录，受载/运动期间持续失去压力反馈
仍触发卸载。温度缺失、状态未回IDLE、Raw偏移、力自然松弛都不是停机条件。

使用(session_id,trial_index,attempt_index)标识尝试。planned_trial_index保留原计划编号。
续测在新目录保留resumed_from，旧文件不覆盖；已完成项不重复，中断项从头采集。
resume_context标明新时间段、空档受力历史及初始材料状态未知；不拼接中断曲线。
续测先卸载并记录360秒恢复，再执行完整条件；该等待不证明Raw或材料完全恢复。
跨会话分析使用各自时间戳；不能直接相减不同进程的monotonic或假定空档期间一直零力。
窗口完成与数据完整性分开判断，先检查质量事件，再决定是否用于拟合或需要补测。
SQLite按512MiB默认分卷而不轮转删除；全程异步写入，完成事件以数据库提交为准。

建议从单指数p=1开始，再比较快慢双指数；使用实测F(t)递推，整轮留出验证。
协议与优化步骤见项目根目录 LONG_DURATION_CREEP_TEST_CN.md、RAW_REBOUND_MODEL_CN.md。
旧会话protocol缺失或为history_response_v1时，仍按原来的高压后探针方案及自适应采样解读。
"""
        try:
            (Path(output_dir) / "README_数据说明.md").write_text(
                text,
                encoding="utf-8",
            )
        except OSError:
            pass

    @Slot()
    def stop_test(self):
        if not self._running or self._stopping:
            return
        self._stopping = True
        self.stop_button.setEnabled(False)
        if self._recorder is not None:
            self._recorder.enqueue_event("user_stop_requested", self._phase_context)
        self.phase_label.setText("正在停止：回到标定释放位置并确认压力归零，随后保存数据……")
        self._render_runtime()
        self._force_controller.request_stop()

    @Slot()
    def resume_last_test(self):
        if self._running:
            return
        root = creep_test_directory()
        try:
            candidate = find_resumable_test(root)
        except (OSError, ValueError, sqlite3.Error, KeyError, TypeError) as exc:
            self.phase_label.setText(f"无法安全读取续测断点，请检查记录：{exc}")
            return
        if candidate is not None:
            manifest_path, manifest, remaining = candidate
            config = dict(manifest["config"])
            estimate = estimate_creep_plan(remaining, config["inter_trial_seconds"])
            estimate["total_seconds"] += CREEP_RECOVERY_SECONDS
            if not self._confirm_start(estimate, resumed=True):
                return
            self._start_plan(config, remaining, estimate, resumed_from=str(manifest_path.parent))
            return
        self.phase_label.setText("没有找到可续测的未完成计划")

    @Slot(object)
    def _on_force_progress(self, progress):
        if not self._running:
            return
        progress = dict(progress or {})
        phase = str(progress.get("phase", ""))
        if not phase.startswith("creep_"):
            if "communication" in phase and progress.get("message"):
                self.phase_label.setText(str(progress["message"]))
            return
        self._runtime_reported_at = time.monotonic()
        previous_phase = self._phase_context.get("phase")
        if phase != previous_phase:
            self._finish_phase_statistics(previous_phase)
            self._phase_started_monotonic = time.monotonic()
            self._current_statistics = self._new_phase_statistics()
            if self._recorder is not None and phase != "creep_trial_complete":
                self._recorder.enqueue_event("phase_start", progress)
        if phase == "creep_trial_start":
            # 重试必须清空上一尝试的统计，失败数据仍保留在SQLite中。
            self._phase_statistics = {}
            self._current_statistics = self._new_phase_statistics()
            self._trial_quality_issues = set()
            if self._raw_missing_since is not None:
                self._trial_quality_issues.add('raw_gap')
        self._phase_context = progress
        trial_index = int(progress.get("trial_index", 0) or 0)
        trial_total = int(progress.get("trial_total", len(self._plan)) or len(self._plan))
        overall = self._manifest.get("completed_trials", 0) / max(1, trial_total)
        self.progress_bar.setValue(round(overall * 1000))
        self.phase_label.setText(str(progress.get("message", "")) if phase in (
            "creep_trial_failed", "creep_trial_skipped", "creep_safety_release", "creep_safety_released"
        ) else "")
        self._render_runtime()
        if phase == "creep_trial_complete":
            self._handle_trial_complete(progress)
        elif phase == "creep_trial_skipped":
            self._manifest.setdefault("skipped_trial_indices", []).append(trial_index)
            self._write_manifest()

    @Slot(object)
    def _on_force_finished(self, result):
        result = dict(result or {})
        if not self._running or result.get("mode") != "creep_test":
            return
        status = ("completed_with_gaps" if result.get("creep_skipped_trials") else "completed") if result.get("ok") else "stopped"
        self._manifest["safety_released"] = result.get("safety_released", bool(result.get("ok")))
        message = result.get("message") or result.get("stop_reason") or status
        self._finish_session(status, str(message))
        if result.get("safety_released") is False:
            QtWidgets.QApplication.beep()
            QtWidgets.QMessageBox.critical(self, "自动卸载失败：请立即人工处理", str(message))

    @staticmethod
    def _new_phase_statistics():
        keys = [
            "hardware_raw_sum",
            "baseline_sum",
            "signal_sum",
            "raw_minus_baseline_sum",
            "pressure_raw_n",
            "pressure_filtered_n",
            "motor_position_counts",
            "die_temperature_celsius",
        ]
        keys.extend(
            f"ch{channel}_{field}"
            for channel in range(4, 8)
            for field in ("hardware_raw", "baseline", "signal")
        )
        return {key: RunningStatistics() for key in keys}

    def _finish_phase_statistics(self, phase):
        if phase and self._current_statistics:
            self._phase_statistics[phase] = {
                key: stats.result() for key, stats in self._current_statistics.items()
            }

    def _capture_interval(self, phase_elapsed):
        if getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL:
            return REBOUND_RECORD_INTERVAL_SECONDS
        phase = str(self._phase_context.get("phase", ""))
        if phase in (
            "creep_high_press",
            "creep_high_release",
            "creep_probe_press",
            "creep_probe_release",
            "creep_trial_start",
            "creep_initial_release",
            "creep_safety_release",
        ):
            return 0.0
        if phase in ("creep_high_hold", "creep_probe_hold"):
            if phase_elapsed <= 10.0:
                return 0.0
            if phase_elapsed <= 60.0:
                return 0.05
            return 0.20
        if phase == "creep_inter_trial_wait":
            return 0.50
        if phase_elapsed <= 5.0:
            return 0.0
        if phase_elapsed <= 30.0:
            return 0.10
        return 0.50

    def _capture_latest_sample(self):
        if not self._running or self._recorder is None:
            return
        sample = self._sensor_sample_provider()
        now = time.monotonic()
        if (not isinstance(sample, dict) or sample.get("sequence") is None
                or not sample.get("has_raw")
                or float(sample.get("age_seconds", math.inf)) > 0.5
                or int(sample.get("channel_count", 0)) < CHANNEL_COUNT):
            if getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL:
                if self._raw_missing_since is None:
                    self._raw_missing_since = now
                    self._recorder.enqueue_event('raw_gap_start', dict(self._phase_context,
                        captured_monotonic=now, message='Raw缺帧，保留缺口并继续定时采集'))
                self._trial_quality_issues.add('raw_gap')
                return
            if self._raw_missing_since is None:
                self._raw_missing_since = now
            elif now - self._raw_missing_since > 2.0 and not self._stopping:
                self._recorder.enqueue_event(
                    "raw_frame_lost",
                    {"message": "连续2秒没有Debug原始帧，停止无效测试"},
                )
                self.phase_label.setText("连续2秒没有Debug原始帧，正在安全停止")
                self.stop_test()
            return
        if self._raw_missing_since is not None and getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL:
            self._recorder.enqueue_event('raw_gap_end', dict(self._phase_context,
                captured_monotonic=now, gap_seconds=now-self._raw_missing_since))
        self._raw_missing_since = None
        sequence = int(sample["sequence"])
        if sequence == self._last_sensor_sequence:
            return
        if getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL and self._last_sensor_sequence is not None and sequence < self._last_sensor_sequence:
            self._trial_quality_issues.add('sensor_sequence_reset')
            self._recorder.enqueue_event('sensor_sequence_reset', dict(self._phase_context, captured_monotonic=now))
        if getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL:
            states = tuple(sample.get('state', ()))
            if states and self._last_observed_states is not None and states != self._last_observed_states:
                self._recorder.enqueue_event('sensor_state_change', dict(self._phase_context,
                    captured_monotonic=now, sensor_source_monotonic=now-float(sample.get('age_seconds', 0)),
                    sensor_sequence=sequence, previous_states=self._last_observed_states, states=states))
            if states:
                self._last_observed_states = states
        phase_elapsed = max(0.0, now - self._phase_started_monotonic)
        interval = self._capture_interval(phase_elapsed)
        if self._last_captured_sensor_sequence is not None and now - self._last_captured_monotonic < interval - 1e-9:
            self._last_sensor_sequence = sequence
            return
        self._last_sensor_sequence = sequence
        row = self._build_sample_row(sample, now, phase_elapsed)
        if getattr(self, '_config', {}).get('protocol') == REBOUND_PROTOCOL and (
            not row.get('pressure_accepted') or row.get('pressure_age_ms') is None or row['pressure_age_ms'] > 1000
        ):
            self._trial_quality_issues.add('pressure_gap_or_invalid')
        self._last_captured_sensor_sequence = sequence
        self._last_captured_monotonic = now
        self._recorder.enqueue_sample(row)
        self._update_statistics(row)

    def _build_sample_row(self, sample, now, phase_elapsed):
        context = self._phase_context
        pressure = self._pressure_snapshot()
        motor = self._motor_control.latest_motor_feedback() or {}
        sensor_age_seconds = float(sample.get("age_seconds", 0.0))
        pressure_sampled_monotonic = pressure.get("sampled_monotonic")
        motor_sampled_monotonic = motor.get("sampled_monotonic")
        row = {
            "session_id": self._recorder.session_id,
            "captured_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "sampled_monotonic": now,
            "session_elapsed_seconds": now - self._recorder.session_started_monotonic,
            "trial_index": context.get("trial_index"),
            "planned_trial_index": context.get("planned_trial_index"),
            "attempt_index": context.get("attempt_index"),
            "history_interrupted": int(bool(context.get("history_interrupted", False))),
            "trial_total": context.get("trial_total"),
            "repeat_index": context.get("repeat_index"),
            "category": context.get("category"),
            "phase": context.get("phase"),
            "phase_elapsed_seconds": phase_elapsed,
            "high_force_n": context.get("high_force_n"),
            "high_hold_seconds": context.get("high_hold_seconds"),
            "recovery_seconds": context.get("recovery_seconds"),
            "probe_force_n": context.get("probe_force_n"),
            "probe_hold_seconds": context.get("probe_hold_seconds"),
            "sensor_sequence": sample.get("sequence"),
            "sensor_sequence_gap": (
                None
                if self._last_captured_sensor_sequence is None
                else int(sample["sequence"]) - self._last_captured_sensor_sequence
            ),
            "capture_interval_ms": (
                None
                if self._last_captured_monotonic <= 0.0
                else (now - self._last_captured_monotonic) * 1000.0
            ),
            "sensor_age_ms": sensor_age_seconds * 1000.0,
            "sensor_source_monotonic": now - sensor_age_seconds,
            "sensor_has_raw": 1 if sample.get("has_raw") else 0,
            "die_temperature_celsius": sample.get("die_temperature_celsius"),
            "die_temperature_age_seconds": sample.get(
                "die_temperature_age_seconds"
            ),
            "pressure_raw_n": pressure.get("raw_force_n"),
            "pressure_control_raw_n": pressure.get("control_raw_force_n"),
            "pressure_filtered_n": pressure.get("force_n", pressure.get("filtered_force_n")),
            "pressure_accepted": 1 if pressure.get("accepted") else 0,
            "pressure_sequence": pressure.get("sequence"),
            "pressure_sampled_monotonic": pressure_sampled_monotonic,
            "pressure_age_ms": (
                None
                if pressure_sampled_monotonic is None
                else max(0.0, now - float(pressure_sampled_monotonic)) * 1000.0
            ),
            "motor_position_counts": motor.get("position_counts"),
            "motor_speed_rpm": motor.get("measured_speed_rpm"),
            "motor_position_speed_counts_s": motor.get("position_speed_counts_s"),
            "motor_phase_current_ma": motor.get("phase_current_ma"),
            "motor_running_state": motor.get("running_state"),
            "motor_sampled_monotonic": motor_sampled_monotonic,
            "motor_age_ms": (
                None
                if motor_sampled_monotonic is None
                else max(0.0, now - float(motor_sampled_monotonic)) * 1000.0
            ),
        }
        source_names = {
            "hardware_raw": "hardware_raw",
            "baseline": "baseline",
            "signal": "signal",
            "fast_k_percent": "fast_k_percent",
            "slow_k_percent": "slow_k_percent",
            "state": "state",
        }
        for field, source_name in source_names.items():
            values = tuple(sample.get(source_name) or ())
            for channel in range(CHANNEL_COUNT):
                row[f"ch{channel}_{field}"] = (
                    values[channel] if channel < len(values) else None
                )
        return row

    def _update_statistics(self, row):
        if not self._current_statistics:
            self._current_statistics = self._new_phase_statistics()
        for field in ("hardware_raw", "baseline", "signal"):
            values = [row.get(f"ch{channel}_{field}") for channel in range(4, 8)]
            if all(value is not None for value in values):
                self._current_statistics[f"{field}_sum"].add(sum(values))
                for channel, value in zip(range(4, 8), values):
                    self._current_statistics[f"ch{channel}_{field}"].add(value)
        raw_sum = sum(
            row.get(f"ch{channel}_hardware_raw") or 0 for channel in range(4, 8)
        )
        baseline_sum = sum(
            row.get(f"ch{channel}_baseline") or 0 for channel in range(4, 8)
        )
        self._current_statistics["raw_minus_baseline_sum"].add(
            raw_sum - baseline_sum
        )
        for metric in (
            "pressure_raw_n",
            "pressure_filtered_n",
            "motor_position_counts",
            "die_temperature_celsius",
        ):
            value = row.get(metric)
            if value is not None:
                self._current_statistics[metric].add(value)

    @staticmethod
    def _make_summary_fields():
        base = [
            "trial_index",
            "planned_trial_index",
            "attempt_index",
            "history_interrupted",
            "category",
            "repeat_index",
            "process_index", "post_observation_seconds",
            "high_force_n",
            "high_hold_seconds",
            "recovery_seconds",
            "probe_force_n",
            "probe_hold_seconds",
            "trial_elapsed_seconds",
            "high_measured_force_n",
            "probe_measured_force_n",
            "release_measured_force_n",
            "probe_position_counts",
            "pre_observation_seconds", "completion_basis", "data_quality_issues_json",
        ]
        for phase in ("creep_pre_observation", "creep_high_hold", "creep_recovery_wait", "creep_probe_hold"):
            for metric in (
                "hardware_raw_sum",
                "baseline_sum",
                "signal_sum",
                "raw_minus_baseline_sum",
                "pressure_raw_n",
                "pressure_filtered_n",
                "motor_position_counts",
                "die_temperature_celsius",
            ) + tuple(f'ch{c}_{field}' for c in range(4,8) for field in ('hardware_raw','baseline','signal')):
                for statistic in ("count", "mean", "std", "min", "max"):
                    base.append(f"{phase}_{metric}_{statistic}")
        return tuple(base)

    def _handle_trial_complete(self, progress):
        progress = dict(progress, data_quality_issues=sorted(self._trial_quality_issues),
                        data_valid=not self._trial_quality_issues)
        if self._recorder is not None:
            self._recorder.enqueue_event("trial_complete", progress)
        self._finish_phase_statistics("creep_probe_release")
        result = dict(progress.get("trial_result") or {})
        row = dict(result)
        row['data_quality_issues_json'] = json.dumps(progress['data_quality_issues'])
        for phase, metrics in self._phase_statistics.items():
            for metric, statistics in metrics.items():
                for statistic, value in statistics.items():
                    row[f"{phase}_{metric}_{statistic}"] = value
        if self._summary_path is not None:
            exists = self._summary_path.exists()
            try:
                with self._summary_path.open("a", encoding="utf-8-sig", newline="") as handle:
                    writer = csv.DictWriter(
                        handle,
                        fieldnames=self._summary_field_names,
                        extrasaction="ignore",
                    )
                    if not exists:
                        writer.writeheader()
                    writer.writerow(row)
            except OSError as exc:
                if self._recorder is not None:
                    self._recorder.enqueue_event(
                        "summary_write_error", {"message": str(exc)}
                    )
        self._write_manifest()
        self._phase_statistics = {}
        self._current_statistics = {}

    def _write_manifest(self):
        if self._recorder is None:
            return
        snapshot = self._recorder.snapshot()
        completed = set(snapshot["completed_trial_indices"])
        self._manifest["completed_trial_indices"] = sorted(completed)
        self._manifest["completed_trials"] = len(completed)
        self._manifest["next_trial_index"] = next(
            (i for i in range(1, len(self._plan) + 1) if i not in completed), len(self._plan) + 1
        )
        self._manifest["output_parts"] = list(snapshot["parts"])
        self._manifest["sample_count"] = snapshot["sample_count"]
        self._manifest["dropped_count"] = snapshot["dropped_count"]
        path = self._recorder.session_dir / "manifest.json"
        temporary = path.with_suffix(".json.tmp")
        try:
            temporary.write_text(
                json.dumps(self._manifest, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(path)
            self._last_manifest_write_monotonic = time.monotonic()
            return True
        except OSError as exc:
            self.phase_label.setText(f"清单写入失败：{exc}")
            if self._running and not self._stopping:
                self.stop_test()
            return False

    def _refresh_recorder_status(self):
        if self._recorder is None:
            return
        snapshot = self._recorder.snapshot()
        error_text = "" if not snapshot["error"] else f"｜错误：{snapshot['error']}"
        self.recorder_label.setText(
            f"已保存：{snapshot['sample_count']:,}条读数｜"
            f"{len(snapshot['parts'])}个文件｜待保存{snapshot['queue_size']:,}条｜"
            f"未能保存{snapshot['dropped_count']:,}条{error_text}\n"
            f"目录：{self._recorder.session_dir}"
        )
        if snapshot["error"] and self._running and not self._stopping:
            self.phase_label.setText("数据保存失败，正在停止并松开")
            self.stop_test()
        elif (
            self._running
            and time.monotonic() - self._last_manifest_write_monotonic >= 10.0
        ):
            self._write_manifest()
        if self._running:
            self.progress_bar.setValue(round(1000 * len(snapshot["completed_trial_indices"]) / max(1, len(self._plan))))

    def _finish_session(self, status, message):
        if not self._running and self._recorder is None:
            return
        self._capture_timer.stop()
        self._status_timer.stop()
        self._runtime_timer.stop()
        self._finish_phase_statistics(self._phase_context.get("phase"))
        recorder = self._recorder
        if recorder is not None:
            recorder.enqueue_event(
                "session_finish",
                {"status": status, "message": message},
            )
            snapshot = recorder.stop()
            if snapshot["error"]:
                status = "stopped"
                message = f"{message}；记录异常：{snapshot['error']}"
            self._manifest.update(
                status=status,
                finished_at=datetime.now().astimezone().isoformat(
                    timespec="milliseconds"
                ),
                finish_message=message,
                output_parts=list(snapshot["parts"]),
                sample_count=snapshot["sample_count"],
                event_count=snapshot["event_count"],
                dropped_count=snapshot["dropped_count"],
                recorder_error=snapshot["error"],
            )
            self._write_manifest()
        self._running = False
        self._stopping = False
        for widget in self._plan_inputs:
            widget.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.resume_button.setEnabled(True)
        self.progress_bar.setValue(1000 if status == "completed" else self.progress_bar.value())
        display_status = {
            "completed": "已完成",
            "completed_with_gaps": "已结束，有条件待补测",
            "stopped": "已停止",
            "interrupted": "已中断",
            "start_rejected": "启动失败",
        }.get(status, status)
        self.stage_label.setText(f"测试{display_status}")
        color = "#70e6a4" if status == "completed" else "#ffd166"
        if self._manifest.get("safety_released") is False:
            color = "#ff8b82"
        self.stage_label.setStyleSheet(f"font-size:26px;color:{color};font-weight:700;")
        self.countdown_label.setText(
            "未开始加载" if status == "start_rejected" else
            ("已确认回零卸载" if self._manifest.get("safety_released") else "请确认机构已卸载")
        )
        self.next_step_label.setText(
            "全部条件已完成，数据已保存" if status == "completed" else
            "已完成数据保留；排除问题后可续测未完成项"
        )
        self.phase_label.setText(f"测试{display_status}：{message}")
        self._refresh_recorder_status()
        self.start_button.setEnabled(True)
        self.status_message.emit(f"长时蠕变测试{display_status}：{message}")
        self._recorder = None
        self.refresh_test_history(recorder.session_dir if recorder is not None else None)
        if (status == "completed" and recorder is not None
                and self._config.get("protocol") == REBOUND_PROTOCOL
                and self.auto_analysis_check.isChecked()):
            self._start_analysis(recorder.session_dir)

    def refresh_test_history(self, selected=None):
        selected = str(Path(selected).resolve()) if selected else self.test_history_combo.currentData()
        if selected:
            self._history_extra.add(selected)
        self.test_history_combo.blockSignals(True)
        self.test_history_combo.clear()
        for path, manifest, label in saved_tests(creep_test_directory(), self._history_extra):
            self.test_history_combo.addItem(label, str(path))
            self.test_history_combo.setItemData(self.test_history_combo.count()-1, str(path), QtCore.Qt.ItemDataRole.ToolTipRole)
        index = self.test_history_combo.findData(selected)
        if index >= 0:
            self.test_history_combo.setCurrentIndex(index)
        self.test_history_combo.blockSignals(False)
        self._on_history_selected()

    def _on_history_selected(self, *_args):
        directory = self.test_history_combo.currentData()
        self.report_history_combo.blockSignals(True)
        self.report_history_combo.clear()
        if directory:
            for path, label in saved_reports(directory):
                self.report_history_combo.addItem(label, str(path))
        self.report_history_combo.blockSignals(False)
        self._on_report_selected()
        self.analyze_button.setEnabled(bool(directory) and not self._analysis_busy)
        self.open_data_button.setEnabled(bool(directory))
        if not directory:
            self.analysis_label.setText("没有找到已保存的测试。可从文件夹读取以前保存的测试。")
            return
        try:
            manifest = read_manifest(directory)
        except (OSError, ValueError) as exc:
            self.analyze_button.setEnabled(False)
            self.analysis_label.setText(f"无法读取测试说明：{exc}")
            return
        supported = manifest.get('config', {}).get('protocol') == REBOUND_PROTOCOL
        ended = manifest.get('status') not in ('running', 'start_rejected')
        self.analyze_button.setEnabled(supported and ended and not self._analysis_busy)
        note = "可直接打开已保存的报告，或重新计算并另存新报告。"
        if not supported:
            note = "这是旧测试方案；数据可查看，当前恢复报告计算器不支持此方案。"
        elif not ended:
            note = "此测试尚未结束；保存结束后才能生成报告。可以选择其他历史测试查看。"
        elif not self.report_history_combo.count():
            note = "尚无已保存的报告；点击生成新报告。"
        self.analysis_label.setText(f"{self.test_history_combo.currentText()}\n{note}\n数据文件夹：{Path(directory).name}")
        self.analysis_label.setToolTip(directory)

    def _on_report_selected(self, *_args):
        selected = self.report_history_combo.currentData()
        self._analysis_output = Path(selected) if selected else None
        self.open_analysis_button.setEnabled(bool(selected) and not self._analysis_busy)
        self.report_history_combo.setToolTip(str(selected or '尚无已保存的报告'))

    @Slot()
    def browse_saved_test(self):
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "选择包含 manifest.json 的测试文件夹", str(creep_test_directory()))
        if not directory:
            return
        try:
            read_manifest(directory)
        except (OSError, ValueError) as exc:
            self.analysis_label.setText(f"没有读取到有效测试，请选择包含 manifest.json 的文件夹：{exc}")
            return
        self.refresh_test_history(directory)

    @Slot()
    def open_test_directory(self):
        directory = self.test_history_combo.currentData()
        if directory:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(directory))

    @Slot()
    def analyze_selected_test(self):
        if self._running:
            self.analysis_label.setText("当前正在采集，可直接打开以前保存的报告；重新计算请等本次测试结束。")
            return
        directory = self.test_history_combo.currentData()
        if directory:
            self._start_analysis(Path(directory))

    def _set_history_busy(self, busy):
        for widget in (self.test_history_combo, self.report_history_combo,
                       self.refresh_history_button, self.browse_history_button):
            widget.setEnabled(not busy)

    def _start_analysis(self, directory):
        if self._analysis_busy or self._analysis_closing:
            return
        self.refresh_test_history(directory)
        self._analysis_busy = True
        self._set_history_busy(True)
        self.analyze_button.setEnabled(False)
        self.open_analysis_button.setEnabled(False)
        self.analysis_label.setText(f"正在计算所选测试并另存新报告，已有数据和报告保留……\n{Path(directory).name}")

        def work():
            try:
                from rebound_analysis import analyze_rebound
                result = analyze_rebound(directory)
            except Exception as exc:
                result = {"error": str(exc)}
            if not self._analysis_closing:
                try:
                    self.analysis_finished.emit(result)
                except RuntimeError:
                    # 窗口已销毁时只保留磁盘报告，后台不访问Qt控件。
                    pass

        threading.Thread(target=work, name="rebound-analysis", daemon=True).start()

    @Slot(object)
    def _on_analysis_finished(self, result):
        self._analysis_busy = False
        self._set_history_busy(False)
        self.refresh_test_history()
        if result.get("error"):
            self.analysis_label.setText(f"指标分析失败，原始数据已保留：{result['error']}")
            return
        self._analysis_output = Path(result["output_dir"])
        self.open_analysis_button.setEnabled(True)
        report = result["report"]
        lines = [f"已分析 {report['completed_conditions']} 个条件，{len(report['sessions'])} 段记录。"]
        for channel in range(4, 8):
            rows = [r for r in report["channels"] if r["channel"] == channel]
            returned = sum(r["confirmed_raw_2counts_5s"] is not None for r in rows)
            rising = sum(r["tail_rising"] for r in rows)
            lines.append(f"CH{channel}：曾连续5秒返回±2 counts {returned}/{len(rows)}项；末尾上漂 {rising}项。")
        lines.append("返回后可能再次离带；IDLE、Raw返回、温度和实测力详见报告。")
        lines.append(f"已另存新报告：{self._analysis_output.name}")
        self.analysis_label.setToolTip(str(self._analysis_output))
        self.analysis_label.setText("\n".join(lines))

    @Slot()
    def open_analysis_report(self):
        if self._analysis_output is not None:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(self._analysis_output / "report.html")))

    def shutdown(self):
        self._analysis_closing = True
        if self._running:
            self.stop_test()

    def is_running(self):
        return self._running

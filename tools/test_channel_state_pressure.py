"""在Unicorn中执行真实ARM状态机，回归分段松手和CSV轻压保持记录。

依赖：pip install unicorn pyelftools；也读取tmp/state_test_deps中的隔离安装。
仅生成测试ELF，不构建上位机EXE。CSV丢失帧按sensor_sequence线性插值，
回放是近似验证，不代表获得了日志中没有保存的原始帧。
"""
import argparse
import csv
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tmp/state_test_deps"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_LR, UC_ARM_REG_SP
from unicorn.arm_const import UC_ARM_REG_PC

HARNESS = r'''
/* 引入正式状态机接口，测试不复制状态算法。 */
#include "channel_state_machine.h"
/* 保存回放输入和真实固件输出，供宿主通过模拟RAM读取。 */
uint16_t trace_raw[32768];
/* 保存逐帧状态以及当前遥测快慢斜率。 */
uint8_t trace_states[32768];
int32_t trace_fast[32768];
int32_t trace_slow[32768];
/* 保存活动抑制和切档控制输入，验证候选状态不会跨恢复阶段泄漏。 */
uint8_t trace_flags[32768];
/* 只使用一个隔离的通道实例，保持正式结构布局。 */
static channel_state_machine_t machine;
/* 重置实例并按真实采样顺序处理整个测试波形。 */
void test_run(uint32_t count, uint32_t baseline, uint32_t noise) {
    /* 显式初始化全部输入，避免依赖启动代码清零。 */
    channel_state_input_t input = {0};
    input.baseline = baseline;
    input.noise = noise;
    input.calibrated = true;
    ChannelState_Init(&machine, baseline, baseline);
    /* 每个输入对应一帧真实状态机更新，不绕过窗口和消抖。 */
    for (uint32_t i = 0; i < count; ++i) {
        input.raw = trace_raw[i];
        input.suppress_activity = (trace_flags[i] & 1u) != 0;
        input.autorange_transition_active = (trace_flags[i] & 2u) != 0;
        input.autorange_transition_finished = (trace_flags[i] & 4u) != 0;
        trace_states[i] = ChannelState_Update(&machine, &input);
        trace_fast[i] = ChannelState_GetFastNormalizedKX100(&machine);
        trace_slow[i] = ChannelState_GetSlowNormalizedKX100(&machine);
    }
}
'''


def compiler_path():
    if os.environ.get("ARM_GCC"):
        return os.environ["ARM_GCC"]
    candidates = sorted((Path.home() / "Infineon/Tools/mtb-gcc-arm-eabi").glob("*/gcc/bin/arm-none-eabi-gcc.exe"))
    if not candidates:
        raise RuntimeError("请设置ARM_GCC指向arm-none-eabi-gcc")
    return str(candidates[-1])


def build_model(variant=1, revision=None):
    directory = ROOT / "build/state_tests" / (f"old_{variant}" if revision else f"new_{variant}")
    directory.mkdir(parents=True, exist_ok=True)
    source = ROOT / "channel_state_machine.c"
    if revision:
        for name in ("channel_state_machine.c", "channel_state_machine.h"):
            content = subprocess.check_output(["git", "show", f"{revision}:{name}"], cwd=ROOT)
            (directory / name).write_bytes(content)
        source = directory / "channel_state_machine.c"
    harness = directory / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    elf = directory / "state.elf"
    subprocess.run([compiler_path(), "-mcpu=cortex-m0plus", "-mthumb", "-Os", "-g", "-Wall", "-Wextra", "-Werror",
                    "-std=c99", "-nostartfiles", "--specs=nosys.specs", f"-DDEVICE_FINGER_TYPE={variant}",
                    "-I", str(directory), "-I", str(ROOT), str(harness), str(source),
                    "-Wl,-e,test_run,-Ttext=0x10000,-Tdata=0x20000000", "-o", str(elf)], check=True, capture_output=True)
    return ArmModel(elf)


class ArmModel:
    def __init__(self, path):
        elf = ELFFile(io.BytesIO(path.read_bytes()))
        self.segments = [(s["p_vaddr"], s.data()) for s in elf.iter_segments() if s["p_type"] == "PT_LOAD"]
        self.symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}

    def run(self, raw, baseline=1000, noise=10, flags=None):
        count = len(raw)
        assert 0 < count <= 32768
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x10000, 0x100000)
        uc.mem_map(0x20000000, 0x200000)
        for address, data in self.segments:
            uc.mem_write(address, data)
        uc.mem_write(self.symbols["trace_raw"], struct.pack(f"<{count}H", *raw))
        uc.mem_write(self.symbols["trace_flags"], bytes(flags or [0] * count))
        for reg, value in ((UC_ARM_REG_R0, count), (UC_ARM_REG_R1, baseline), (UC_ARM_REG_R2, noise),
                           (UC_ARM_REG_SP, 0x201FF000), (UC_ARM_REG_LR, 0xF0001)):
            uc.reg_write(reg, value)
        uc.emu_start(self.symbols["test_run"] | 1, 0xF0000, count=100000000)
        if uc.reg_read(UC_ARM_REG_PC) != 0xF0000:
            raise RuntimeError("ARM回放达到指令上限，未正常返回")
        states = list(uc.mem_read(self.symbols["trace_states"], count))
        fast = struct.unpack(f"<{count}i", uc.mem_read(self.symbols["trace_fast"], count * 4))
        slow = struct.unpack(f"<{count}i", uc.mem_read(self.symbols["trace_slow"], count * 4))
        return states, fast, slow


def ramp(start, end, count):
    return [round(start + (end - start) * (i + 1) / count) for i in range(count)]


class StateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {variant: build_model(variant) for variant in (1, 2, 3)}
        cls.legacy = build_model(1, "aca49d8")

    def test_small_and_medium_press_hold_then_release(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            for peak, plateau in [(60, 43), (120, 82), (203, 145), (283, 146), (299, 210), (300, 210),
                                  (301, 210), (600, 480), (999, 900), (1000, 900), (1001, 901), (3000, 2800), (10000, 9500)]:
                with self.subTest(variant=variant, peak=peak):
                    raw = [1000] * hz + ramp(1000, 1000 + peak, hz // 10) + ramp(1000 + peak, 1000 + plateau, hz // 10)
                    hold = len(raw)
                    raw += [1000 + plateau] * (6 * hz)
                    release = len(raw)
                    raw += ramp(raw[-1], 1000, hz // 10) + [1000] * hz
                    states, _, _ = model.run(raw)
                    self.assertTrue(all(s in (1, 2, 3, 6) for s in states[hold:release]))
                    self.assertTrue(all(s == 6 for s in states[release - hz:release]))
                    self.assertTrue(any(s in (0, 4) for s in states[release:release + hz // 4]))
                    self.assertEqual(states[-1], 0)

    def test_old_code_reproduces_small_pressure_spike(self):
        raw = [1000] * 160 + ramp(1000, 1120, 16) + ramp(1120, 1082, 10) + [1082] * 500
        old, _, _ = self.legacy.run(raw)
        new, _, _ = self.models[1].run(raw)
        self.assertIn(4, old[176:])
        self.assertNotIn(4, new)
        self.assertEqual(new[-1], 6)

    def test_large_pressure_release_keeps_original_latency(self):
        for peak in (1000, 1001, 3000, 10000, 50000):
            with self.subTest(peak=peak):
                raw = [1000] * 160 + ramp(1000, 1000 + peak, 16) + [1000 + peak] * 960
                release = len(raw)
                raw += ramp(raw[-1], 1000 + peak // 2, 16) + [1000 + peak // 2] * 160
                old, _, _ = self.legacy.run(raw)
                new, _, _ = self.models[1].run(raw)
                self.assertEqual(old[release:].index(4), new[release:].index(4))

    def test_noise_does_not_activate_and_flat_raw_has_zero_slope(self):
        for variant, model in self.models.items():
            with self.subTest(variant=variant):
                raw = [1000 + [0, 1, -1, 2, -2][i % 5] for i in range(600)] + [1000] * 600
                states, fast, slow = model.run(raw)
                self.assertEqual(set(states), {0})
                active = [1000] * 300 + ramp(1000, 1200, 25) + [1200] * 700
                states, fast, slow = model.run(active)
                self.assertEqual(states[-1], 6)
                self.assertEqual((fast[-1], slow[-1]), (0, 0))

    def test_brief_deep_dip_does_not_clear_small_pressure(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000] * hz + ramp(1000, 1200, hz // 10) + [1200] * (2 * hz)
            start = len(raw)
            raw += [1010] * max(1, hz // 50) + [1200] * hz
            states, _, _ = model.run(raw)
            self.assertNotIn(4, states[start:])
            self.assertNotIn(0, states[start:])

    def test_recorded_light_hold_reaches_floating_until_real_release(self):
        path = ROOT / "tools/fixtures/ch7_light_hold_20260911_112743.csv"
        with path.open(encoding="utf-8") as stream:
            samples = list(csv.DictReader(stream))
        raw, refs = [], []
        previous = int(samples[0]["sensor_sequence"]) - 1
        last_raw = int(samples[0]["ch7_normalized_raw"])
        for row in samples:
            sequence, current = int(row["sensor_sequence"]), int(row["ch7_normalized_raw"])
            raw += ramp(last_raw, current, sequence - previous)
            refs.append((len(raw) - 1, row["timestamp"][11:23]))
            previous, last_raw = sequence, current
        old, _, _ = self.legacy.run(raw, 875, 10)
        new, _, _ = self.models[1].run(raw, 875, 10)
        hold = [i for i, time in refs if "11:27:44.400" <= time <= "11:27:48.450"]
        self.assertEqual({old[i] for i in hold}, {4})
        self.assertEqual({new[i] for i in hold}, {6})
        self.assertFalse(any(new[i] in (0, 4, 5) for i, time in refs if "11:27:43.252" <= time <= "11:27:48.450"))
        self.assertTrue(any(new[i] == 4 for i, time in refs if "11:27:48.500" <= time <= "11:27:48.800"))
        # 记录末尾仍在80帧稳定等待中，真实状态现在明确为WAIT_STABLE。
        self.assertEqual(new[-1], 8)

    def test_slow_release_at_multiple_pressures(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            for noise in (1, 5, 10):
                with self.subTest(variant=variant, noise=noise):
                    peak = max(20, noise * 10) * 2
                    plateau = round(peak * 0.7)
                    raw = [1000] * hz + ramp(1000, 1000 + peak, hz // 5)
                    raw += ramp(raw[-1], 1000 + plateau, hz // 10) + [1000 + plateau] * (2 * hz)
                    release = len(raw)
                    raw += ramp(raw[-1], 1000, 2 * hz) + [1000] * hz
                    states, _, _ = model.run(raw, noise=noise)
                    self.assertEqual(states[release - 1], 6)
                    self.assertEqual(states[-1], 0)

    def test_noise_input_cannot_change_states_or_normalized_slopes(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            for peak in (16, 60, 300, 600, 1000):
                raw = [1000] * hz + ramp(1000, 1000 + peak, hz // 10)
                raw += [1000 + peak] * (2 * hz) + ramp(1000 + peak, 1000, hz // 10) + [1000] * hz
                expected = model.run(raw, noise=1)
                for noise in (0, 5, 10, 65535):
                    with self.subTest(variant=variant, peak=peak, noise=noise):
                        self.assertEqual(model.run(raw, noise=noise), expected)

    def test_activation_requires_strictly_more_than_twelve_counts(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            for delta in (12, 13):
                raw = [1000] * hz + [1000 + delta] * (2 * hz)
                states, _, _ = model.run(raw, noise=65535)
                with self.subTest(variant=variant, delta=delta):
                    if delta == 12:
                        self.assertEqual(set(states), {0})
                    else:
                        # 按压确认期间保持真实IDLE，第三帧直接进入RISING。
                        self.assertEqual(states[hz:hz + 3], [0, 0, 1])
                        self.assertEqual(states[-1], 6)

    def test_idle_over_threshold_without_rising_slope_does_not_enter_recovery(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            # 活动抑制期间填满恒定Raw窗口，随后撤销抑制，仅有偏差而没有上升斜率。
            raw = [1013] * (3 * hz)
            states, fast, slow = model.run(raw, flags=[1] * hz + [0] * (2 * hz))
            with self.subTest(variant=variant):
                self.assertEqual(set(states), {0})
                self.assertEqual(set(fast), {0})
                self.assertEqual(set(slow), {0})

    def test_two_count_jitter_during_post_press_wait_cannot_reactivate(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            # 缓慢释放到高于激活门槛的残余，使通道从活动态进入WAIT_STABLE。
            raw = [1000] * hz + ramp(1000, 2000, hz // 5) + [2000] * (2 * hz)
            raw += ramp(2000, 1070, 2 * hz)
            jitter_start = len(raw)
            # 复现快照中的2 counts窗口波动；旧逻辑会用冻结基线残差和20 counts/s误判再按压。
            jitter = (0, 0, 1, 1, 2, 2, 2, 1, 1, 0, 0, -1, -1, 0, 0, 0)
            raw += [1070 + jitter[i % len(jitter)] for i in range(4 * hz)]
            states, _, _ = model.run(raw)
            wait_index = next(i for i in range(jitter_start, len(states)) if states[i] == 8)
            with self.subTest(variant=variant):
                self.assertIn(7, states[wait_index:])
                self.assertFalse(any(state in (1, 2, 3, 6) for state in states[wait_index:]))

    def test_repress_from_residual_recovers_activity(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000] * hz + ramp(1000, 1200, hz // 10) + [1200] * (2 * hz)
            raw += ramp(1200, 1060, hz // 10) + [1060] * hz
            restart = len(raw)
            raw += ramp(1060, 1180, hz // 10) + [1180] * (2 * hz)
            states, _, _ = model.run(raw)
            self.assertEqual(states[restart - 1], 4)
            self.assertIn(1, states[restart:restart + hz // 4])
            self.assertEqual(states[-1], 6)

    def test_suppression_and_range_reset_cancel_release_candidate(self):
        model = self.models[1]
        raw = [1000] * 160 + ramp(1000, 1200, 16) + [1200] * 320 + [1030] * 5
        start = len(raw)
        raw += [1030] * 16 + [1200] * 400
        flags = [0] * len(raw)
        flags[start:start + 16] = [2] * 16
        flags[start + 16] = 4
        states, _, _ = model.run(raw, flags=flags)
        self.assertNotIn(4, states[start:])
        self.assertEqual(states[-1], 6)
        flags = [0] * len(raw)
        flags[start:start + 16] = [1] * 16
        states, _, _ = model.run(raw, flags=flags)
        self.assertEqual(set(states[start:start + 16]), {0})
        self.assertEqual(states[-1], 6)

    def test_floating_slow_stability_is_not_preempted_by_falling(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000] * hz + ramp(1000, 1200, hz // 10) + ramp(1200, 1164, 2 * hz)
            states, _, slow = model.run(raw)
            self.assertTrue(-1000 <= slow[-1] <= -800)
            self.assertEqual(states[-1], 6)

    def test_two_stage_loading_does_not_lock_on_short_first_plateau(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000] * hz + ramp(1000, 6000, round(hz*.4)) + [6000] * hz
            second = len(raw)
            raw += ramp(6000, 7000, round(hz*.25)) + [7000] * (4*hz)
            states, _, _ = model.run(raw)
            with self.subTest(variant=variant):
                self.assertNotIn(6, states[:second+round(hz*.25)])
                self.assertEqual(states[-1], 6)
                self.assertGreaterEqual(states.index(6), second+round(hz*.25)+round(hz*.3)-1)

    def test_entry_delay_preserves_small_pressure_boundary(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            locks = []
            for pressure in (300, 301):
                raw = [1000]*hz + [1000+pressure]*(6*hz)
                states, _, _ = model.run(raw)
                locks.append(states.index(6))
            with self.subTest(variant=variant):
                self.assertGreaterEqual(locks[1]-locks[0], round(hz*3.27))
                self.assertLessEqual(locks[1]-locks[0], round(hz*3.31))

    def test_floating_entry_stability_restarts_after_new_raw_change(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000] * hz + ramp(1000, 1400, hz//10) + [1400] * (6*hz)
            constant, _, _ = model.run(raw)
            interrupted = constant.index(6)-2
            raw[interrupted:interrupted+5] = [1440]*5
            states, _, _ = model.run(raw)
            with self.subTest(variant=variant):
                self.assertNotIn(6, states[:interrupted+5+round(hz*.3)-1])
                self.assertEqual(states[-1], 6)

    def test_delayed_lock_tracks_late_small_corrections_and_resets_on_release(self):
        for variant, model in self.models.items():
            hz = 160 if variant == 1 else 250
            raw = [1000]*hz
            activations, corrections, releases = [], [], []
            # 两轮按压均须重新等待；补压很小，Fast K不会达到原FLOATING退出门槛。
            for _ in range(2):
                activations.append(len(raw))
                raw += ramp(1000, 6000, round(hz*.4)) + [6000]*round(hz*2.7)
                correction = len(raw)
                raw += ramp(6000, 6100, round(hz*.2)) + [6100]*round(hz*.3)
                raw += ramp(6100, 6150, round(hz*.2))
                corrections.append((correction, len(raw)))
                raw += [6150]*(2*hz)
                releases.append(len(raw))
                raw += ramp(6150, 1000, hz//10) + [1000]*(2*hz)
            states, fast, _ = model.run(raw)
            with self.subTest(variant=variant):
                for active, (first, end), release in zip(activations, corrections, releases):
                    self.assertNotIn(6, states[active:end])
                    self.assertLess(max(fast[first:end]), 8000)
                    lock = next(i for i in range(active, release) if states[i] == 6)
                    self.assertGreaterEqual(lock-active, round(hz*4.3)-2)
                    self.assertLess(lock-active, round(hz*4.5))
                    self.assertEqual(states[release-1], 6)
                    self.assertIn(4, states[release:release+hz//4])
                self.assertEqual(states[-1], 0)


def replay(csv_path, start, end, output):
    samples = []
    with Path(csv_path).open(encoding="utf-8-sig") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            if row["event"] == "sensor_sample" and start <= row["timestamp"][11:23] <= end:
                samples.append((line, row))
    if len(samples) < 2:
        raise ValueError("指定时段不足两条传感器记录")
    baseline = int(samples[0][1]["ch7_baseline"])
    noise = int(samples[0][1]["ch7_noise"])
    raw, refs = [], []
    previous_sequence = int(samples[0][1]["sensor_sequence"]) - 1
    previous_raw = int(samples[0][1]["ch7_normalized_raw"])
    for line, row in samples:
        sequence = int(row["sensor_sequence"])
        gap = sequence - previous_sequence
        if not 1 <= gap <= 160:
            raise ValueError(f"无法可靠插值序号跳变：{gap}")
        current = int(row["ch7_normalized_raw"])
        raw += ramp(previous_raw, current, gap)
        refs.append((len(raw) - 1, line, row))
        previous_sequence, previous_raw = sequence, current
    old, _, _ = build_model(1, "aca49d8").run(raw, baseline, noise)
    new, fast, slow = build_model(1).run(raw, baseline, noise)
    with Path(output).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "source_line", "raw", "baseline", "recorded_signal", "recorded_state", "old_replay_state", "new_replay_state", "new_fast_k", "new_slow_k"])
        for i, line, row in refs:
            writer.writerow([row["timestamp"], line, raw[i], baseline, row["ch7_signal"], row["ch7_state"], old[i], new[i], fast[i] / 100, slow[i] / 100])
    transitions = []
    for i, line, row in refs:
        if not transitions or new[i] != transitions[-1]["state"]:
            transitions.append(dict(time=row["timestamp"][11:23], state=new[i], raw=raw[i], line=line))
    print(json.dumps(dict(samples=len(samples), frames=len(raw), interpolated=len(raw) - len(samples), transitions=transitions), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    if "--replay" in sys.argv:
        parser = argparse.ArgumentParser()
        parser.add_argument("--replay", required=True)
        parser.add_argument("--start", required=True)
        parser.add_argument("--end", required=True)
        parser.add_argument("--output", required=True)
        args = parser.parse_args()
        replay(args.replay, args.start, args.end, args.output)
    else:
        unittest.main(verbosity=2)

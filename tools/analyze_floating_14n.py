"""核对固定三分钟快照，并用真实ARM状态机重放14 N两段加载。"""
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tmp/floating_plot_deps'))
from test_channel_state_pressure import ArmModel, build_model
from test_fixed_noise import Firmware


def read_rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def stamp(row):
    return datetime.fromisoformat(row['timestamp']).timestamp()


def main():
    output = ROOT / 'analysis_outputs/floating_14n_20260920_151246'
    output.mkdir(parents=True, exist_ok=True)
    data = ROOT / 'PY_SRIPT/algo_monitor'
    archive = data / 'recordings/sensor_data_3min_20260920_151246_315895_741bbe1b.csv'
    metadata_path = output / 'source.meta.json'
    if not metadata_path.exists():
        metadata_path = data / 'recordings/recent_3min.meta.json'
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if metadata['archive_file'] != archive.name:
        raise ValueError('固定快照已更新，请使用本次分析保存的元数据核对原归档')
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    assert digest == metadata['csv_sha256']
    (output / 'source.meta.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    all_rows = read_rows(archive)
    rows = [r for r in all_rows if r['type'] in ('debug', 'signal')]
    gaps = [stamp(b)-stamp(a) for a, b in zip(rows, rows[1:])]
    sequence_gaps = [(int(b['sequence'])-int(a['sequence'])) % 65536
                     for a, b in zip(all_rows, all_rows[1:])]
    motor_path = data / 'force_press_tests/zdt_x42s_1_3200/force_press_20260920_150802_544.csv'
    motor = read_rows(motor_path)
    starts = [r for r in motor if r['event'] == 'operation_started'
              and r['control_mode'] == 'press' and r['target_force_n'] == '14.00'
              and stamp(rows[0]) <= stamp(r) <= stamp(rows[-1])]
    old = ArmModel(ROOT / 'build/state_tests/before_floating_entry.elf')
    new = build_model(1)
    summary = dict(source=str(archive.relative_to(ROOT)), sha256=digest,
                   exported_at=metadata['exported_at'], first=rows[0]['timestamp'],
                   last=rows[-1]['timestamp'], sample_count=len(rows), max_gap_s=max(gaps),
                   nonconsecutive_uart_sequences=sum(g != 1 for g in sequence_gaps),
                   motor_source=str(motor_path.relative_to(ROOT)), attempts=[])
    graph_data = None
    for start in starts:
        t0 = stamp(start)
        segment = [r for r in rows if t0-1 <= stamp(r) <= t0+5]
        movement = [r for r in motor if t0 <= stamp(r) <= t0+5 and r['event'] == 'native_force_control']
        attempt = dict(start=start['timestamp'], channels=[])
        traces = []
        for c in range(4, 8):
            raw = [int(r[f'raw_{c}']) for r in segment]
            recorded = [int(r[f'state_{c}']) for r in segment]
            baseline = next(int(r[f'baseline_{c}']) for r in segment if int(r[f'state_{c}']) == 1)
            before, _, _ = old.run(raw, baseline=baseline, noise=1)
            after, _, _ = new.run(raw, baseline=baseline, noise=1)
            i0 = recorded.index(6)
            i1 = after.index(6)
            before_i = before.index(6)
            # 逐个已记录传感器帧回放，不用其他诊断帧凑采样，也不伪造缺失原始帧。
            assert abs(stamp(segment[before_i])-stamp(segment[i0])) <= .08
            assert stamp(segment[i1]) > t0+2.3
            assert after[-1] == 6
            window = [r for r in segment if stamp(segment[i0]) <= stamp(r) <= t0+4]
            tail = window[-1]
            item = dict(channel=c, recorded_lock=segment[i0]['timestamp'],
                        replay_old_lock=segment[before_i]['timestamp'], replay_new_lock=segment[i1]['timestamp'],
                        raw_at_lock=raw[i0], raw_at_4s=int(tail[f'raw_{c}']),
                        raw_increase=int(tail[f'raw_{c}'])-raw[i0],
                        signal_at_lock=int(segment[i0][f'signal_{c}']), signal_at_4s=int(tail[f'signal_{c}']),
                        peak_fast_k=max(float(r[f'fast_k_percent_{c}']) for r in window),
                        new_lock_raw=raw[i1], baseline=baseline,
                        recorded_states_after_lock=sorted(set(int(r[f'state_{c}']) for r in window)))
            attempt['channels'].append(item)
            traces.append((c, segment, before, after))
        summary['attempts'].append(attempt)
        if graph_data is None:
            graph_data = (t0, traces, movement)
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    # 用正式编译的完整算法验证第一轮Signal输出，而不只比较状态机的锁定时刻。
    t0, traces, movement = graph_data
    segment = traces[0][1]
    firmware = Firmware('INDEX_FINGER')
    for _ in range(300):
        firmware.step([int(segment[0][f'raw_{c}']) for c in range(8)])
    signals = {c: [] for c in range(4, 8)}
    for r in segment:
        outputs = firmware.step([int(r[f'raw_{c}']) for c in range(8)])
        for c in signals:
            signals[c].append(outputs[c]['signal'])
    for item in summary['attempts'][0]['channels']:
        c = item['channel']
        at_four = max(i for i, r in enumerate(segment) if stamp(r) <= t0+4)
        item['full_firmware_signal_at_4s'] = signals[c][at_four]
        assert item['full_firmware_signal_at_4s']-item['signal_at_4s'] > 700
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0, traces, movement = graph_data
    fig, axes = plt.subplots(5, 1, figsize=(11, 10), sharex=True)
    axes[0].plot([stamp(r)-t0 for r in movement], [float(r['raw_pressure_n']) for r in movement], 'k.-')
    axes[0].axhline(14, color='gray', ls='--')
    axes[0].set_ylabel('Force (N)')
    axes[0].set_title('14 N two-stage loading: recorded Raw/Signal and replay lock times')
    for ax, (c, segment, before, after) in zip(axes[1:], traces):
        t = [stamp(r)-t0 for r in segment]
        delta = [int(r[f'raw_{c}'])-int(r[f'baseline_{c}']) for r in segment]
        ax.plot(t, delta, label='Recorded Raw - Baseline')
        ax.plot(t, [int(r[f'signal_{c}']) for r in segment], label='Recorded Signal')
        ax.plot(t, signals[c], label='New Signal (full firmware replay)', color='green')
        ax.axvline(t[before.index(6)], color='red', ls='--', label='Old lock (ARM replay)')
        ax.axvline(t[after.index(6)], color='green', ls='--', label='300 ms stable lock (ARM replay)')
        ax.set_ylabel(f'CH{c} counts')
        ax.grid(alpha=.25)
    axes[1].legend(loc='lower right', fontsize=8)
    axes[-1].set_xlabel('Seconds from motor operation start')
    axes[-1].set_xlim(0, 5)
    fig.tight_layout()
    fig.savefig(output / 'two_stage_14n.png', dpi=150)
    plt.close(fig)
    print(f'Validated {len(rows)} sensor frames, {len(starts)} loading attempts, '
          f'{sum(len(a["channels"]) for a in summary["attempts"])} channel replays; outputs: {output}')


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()

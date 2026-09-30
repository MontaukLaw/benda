"""核对固定快照并对齐真实通道状态与电机日志，不修改控制程序。"""
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DATA = ROOT / 'PY_SRIPT/algo_monitor'


def read(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row['t'] = datetime.fromisoformat(row['timestamp']).timestamp()
    return rows


def main():
    meta_path = OUT / 'source.meta.json'
    if not meta_path.exists():
        meta_path.write_bytes((DATA / 'recordings/recent_3min.meta.json').read_bytes())
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['archive_file'] == 'sensor_data_3min_20260920_153336_719764_27162bac.csv'
    source = DATA / 'recordings' / meta['archive_file']
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == meta['csv_sha256']
    all_rows = read(source)
    sensor = [r for r in all_rows if r['type'] in ('debug', 'signal')]
    assert len(sensor) == meta['sensor_sample_count']
    assert len(all_rows) == meta['row_count']
    motor = read(DATA / 'force_press_tests/zdt_x42s_1_3200/force_press_20260920_150802_544.csv')
    gaps = [dict(start=a['timestamp'], end=b['timestamp'], seconds=b['t']-a['t'])
            for a, b in zip(sensor, sensor[1:]) if b['t']-a['t'] > .1]
    starts = [r for r in motor if r['event'] == 'operation_started'
              and r['control_mode'] == 'press' and sensor[0]['t'] <= r['t'] <= sensor[-1]['t']]
    result = dict(exported_at=meta['exported_at'], sha256=digest,
                  sensor_samples=len(sensor), gaps=gaps, presses=[], failed_attempts=[])
    plot_data = []
    for start in starts:
        t0 = start['t']
        op = start['operation_id']
        arrival = next((r for r in motor if r['operation_id'] == op
                        and r['event'] == 'pressure_arrival_timing'), None)
        if arrival is None:
            completion = next(r for r in motor if r['operation_id'] == op
                              and r['event'] == 'operation_complete')
            result['failed_attempts'].append(dict(start=start['timestamp'], reason=completion['message']))
            continue
        end = next((r['t'] for r in motor if r['t'] > t0 and r['event'] == 'operation_started'), sensor[-1]['t'])
        rows = [r for r in sensor if t0 <= r['t'] < end]
        movement = [r for r in motor if t0 <= r['t'] < end]
        assert max(b['t']-a['t'] for a, b in zip(rows, rows[1:])) < .1
        item = dict(operation_id=op, start=start['timestamp'], target_n=float(start['target_force_n']),
                    arrival=arrival['timestamp'], arrival_from_start_s=arrival['t']-t0, channels=[])
        for c in range(4, 8):
            lock = next(r for r in rows if r[f'state_{c}'] == '6')
            post = [r for r in rows if lock['t'] <= r['t'] <= arrival['t']]
            # 只在确有锁定后到位前数据时计算增量；到位后才锁定的通道不虚构该区间。
            tail = post[-1] if post else None
            item['channels'].append(dict(channel=c, lock=lock['timestamp'], lock_from_start_s=lock['t']-t0,
                raw_at_lock=int(lock[f'raw_{c}']), signal_at_lock=int(lock[f'signal_{c}']),
                raw_gain_before_arrival=int(tail[f'raw_{c}'])-int(lock[f'raw_{c}']) if tail else None,
                signal_gain_before_arrival=int(tail[f'signal_{c}'])-int(lock[f'signal_{c}']) if tail else None,
                max_fast_k_before_arrival=max(float(r[f'fast_k_percent_{c}']) for r in post) if post else None,
                states_after_lock=sorted(set(int(r[f'state_{c}']) for r in rows if r['t'] >= lock['t']))))
        result['presses'].append(item)
        plot_data.append((start, arrival, rows, movement, item))
    (OUT / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    sys.path.insert(0, str(ROOT / 'tmp/floating_plot_deps'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(13, 11))
    for pair, (start, arrival, rows, movement, item) in zip(axes, plot_data):
        t0 = start['t']
        # 控制期间的压力计独立事件较稀疏，使用控制循环实际记录的压力样本。
        force = [r for r in movement if r['event'] == 'native_force_control' and r['raw_pressure_n']]
        last_control = force[-1]['t'] if force else t0
        force.extend(r for r in movement if r['event'] == 'pressure_meter_sample'
                     and r['raw_pressure_n'] and r['t'] > last_control)
        pair[0].plot([r['t']-t0 for r in force], [float(r['raw_pressure_n']) for r in force], color='black')
        pair[0].axhline(item['target_n'], color='gray', ls=':')
        pair[0].set_ylabel('Pressure (N)')
        pair[0].set_title(f"{start['timestamp'][11:19]} / {item['target_n']:g} N")
        times = [r['t']-t0 for r in rows]
        pair[1].plot(times, [int(r['raw_5'])-int(r['baseline_5']) for r in rows], label='CH5 Raw - Baseline')
        pair[1].plot(times, [int(r['signal_5']) for r in rows], label='CH5 Signal')
        lock5 = next(c['lock_from_start_s'] for c in item['channels'] if c['channel'] == 5)
        pair[1].set_ylabel('Counts')
        for ax in pair:
            ax.axvspan(min(c['lock_from_start_s'] for c in item['channels']),
                       max(c['lock_from_start_s'] for c in item['channels']), color='orange', alpha=.2,
                       label='CH4-7 lock interval')
            ax.axvline(lock5, color='orange', ls='--', label='CH5 lock')
            ax.axvline(arrival['t']-t0, color='green', ls='--', label='Stable arrival')
            ax.set_xlim(0, 5.5)
            ax.set_xlabel('Seconds from operation start')
            ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=7, loc='lower right')
    axes[0, 1].legend(fontsize=7, loc='lower right')
    fig.suptitle('Recorded states: residual early locking during small force corrections')
    fig.tight_layout()
    fig.savefig(OUT / 'presses.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()

"""用已编译的真实ARM状态机和完整固件回放固定快照的四次按压。"""
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from test_channel_state_pressure import build_model
from test_fixed_noise import Firmware


def main():
    previous = ROOT / 'analysis_outputs/floating_check_20260920_153336'
    metadata = json.loads((previous / 'source.meta.json').read_text(encoding='utf-8'))
    archive = ROOT / 'PY_SRIPT/algo_monitor/recordings' / metadata['archive_file']
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == metadata['csv_sha256']
    with archive.open(encoding='utf-8-sig', newline='') as stream:
        rows = [r for r in csv.DictReader(stream) if r['type'] in ('debug', 'signal')]
    for r in rows:
        r['t'] = datetime.fromisoformat(r['timestamp']).timestamp()
    attempts = json.loads((previous / 'summary.json').read_text(encoding='utf-8'))['presses']
    model = build_model(1)
    results = []
    for attempt in attempts:
        t0 = datetime.fromisoformat(attempt['start']).timestamp()
        segment = [r for r in rows if t0-.5 <= r['t'] <= t0+5.8]
        assert max(b['t']-a['t'] for a, b in zip(segment, segment[1:])) < .1
        result = dict(start=attempt['start'], arrival_s=attempt['arrival_from_start_s'], channels=[])
        firmware = Firmware('INDEX_FINGER')
        for _ in range(300):
            firmware.step([int(segment[0][f'raw_{c}']) for c in range(8)])
        full_signals, full_states = [], []
        for r in segment:
            outputs = firmware.step([int(r[f'raw_{c}']) for c in range(8)])
            full_signals.append([o['signal'] for o in outputs])
            firmware.call('SensorAlgorithm_GetStates', 0x20120000)
            full_states.append(list(firmware.uc.mem_read(0x20120000, 8)))
        for c in range(4, 8):
            baseline = next(int(r[f'baseline_{c}']) for r in segment if r[f'state_{c}'] == '1')
            states, _, _ = model.run([int(r[f'raw_{c}']) for r in segment], baseline, 1)
            lock = states.index(6)
            full_lock = next(i for i, state in enumerate(full_states) if state[c] == 6)
            assert segment[lock]['t']-t0 > attempt['arrival_from_start_s']
            assert segment[full_lock]['t']-t0 > attempt['arrival_from_start_s']
            assert set(states[lock:]) == {6}
            at_arrival = max(i for i, r in enumerate(segment) if r['t']-t0 <= attempt['arrival_from_start_s'])
            # 完整固件应保留原先锁定之后仍发生的真实补压增量。
            gain = full_signals[at_arrival][c]-int(segment[at_arrival][f'signal_{c}'])
            if attempt['operation_id'] in ('15', '19'):
                assert gain >= 75
            result['channels'].append(dict(channel=c, state_model_lock_s=segment[lock]['t']-t0,
                full_firmware_lock_s=segment[full_lock]['t']-t0,
                recorded_signal_at_arrival=int(segment[at_arrival][f'signal_{c}']),
                replay_signal_at_arrival=full_signals[at_arrival][c], signal_gain=gain))
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    (OUT / 'replay.json').write_text(json.dumps(dict(source=metadata, results=results), ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()

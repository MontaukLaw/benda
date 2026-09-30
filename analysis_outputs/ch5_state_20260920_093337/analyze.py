"""只读核对固定三分钟快照，并保存 CH5 状态与压力记录的对照结果。"""
import csv
import hashlib
import io
import json
import os
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding='utf-8')
ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DATA = ROOT / 'PY_SRIPT/algo_monitor'
SOURCE = DATA / 'recordings/sensor_data_3min_20260920_093337_290390_a74be882.csv'
blob = SOURCE.read_bytes()
meta = json.loads(SOURCE.with_suffix('.meta.json').read_text(encoding='utf-8'))
assert hashlib.sha256(blob).hexdigest() == meta['csv_sha256']
all_rows = list(csv.DictReader(io.StringIO(blob.decode('utf-8-sig'))))
rows = [r for r in all_rows if r['type'] in ('debug', 'signal')]
assert len(rows) == meta['sensor_sample_count']
stamp = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in rows])
values = lambda key, ch: np.array([float(r[f'{key}_{ch}']) for r in rows])
epoch = lambda s: datetime.fromisoformat('2026-09-20T' + s + '+08:00').timestamp()
names = ['IDLE', 'RISING', 'HOLD', 'FALLING', 'SLOW_BACK', 'ZERO_REBASE', 'FLOATING', 'RECOVER', 'WAIT_STABLE']
motor_path = DATA / 'force_press_tests/zdt_x42s_1_3200/force_press_20260920_092626_016.csv'
motor_copy = OUT / 'motor_snapshot.csv'
if not motor_copy.exists():
    motor_rows = [r for r in csv.DictReader(io.StringIO(motor_path.read_text(encoding='utf-8-sig')))
                  if rows[0]['timestamp'] <= r['timestamp'] <= rows[-1]['timestamp']]
    with motor_copy.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(motor_rows[0]))
        writer.writeheader()
        writer.writerows(motor_rows)
motor = list(csv.DictReader(motor_copy.open(encoding='utf-8-sig')))
fresh = [r for r in motor if r['raw_pressure_n'] and r['ocr_age_ms'] and 0 <= float(r['ocr_age_ms']) <= 500]
summary = dict(meta=meta, actual_sensor_count=len(rows), actual_span_s=float(stamp[-1]-stamp[0]),
               actual_max_gap_s=float(np.diff(stamp).max()), frame_counts=dict(Counter(r['type'] for r in all_rows)),
               motor_source=str(motor_path.relative_to(ROOT)), channels={}, windows={})
for ch in range(4, 8):
    raw, base, sig, state = [values(k, ch) for k in ('raw', 'baseline', 'signal', 'state')]
    summary['channels'][str(ch)] = dict(raw_range=[int(raw.min()), int(raw.max())],
        baseline_range=[int(base.min()), int(base.max())], signal_range=[int(sig.min()), int(sig.max())],
        transitions=int(np.count_nonzero(np.diff(state))), state_samples=dict(Counter(names[int(s)] for s in state)))
for label, start, end in [('before_press', '09:30:37.294', '09:32:48.500'),
                           ('first_unloaded', '09:32:59', '09:33:10'),
                           ('second_unloaded', '09:33:24', '09:33:37.290')]:
    ix = (stamp >= epoch(start)) & (stamp <= epoch(end))
    pr = [r for r in fresh if epoch(start) <= datetime.fromisoformat(r['timestamp']).timestamp() <= epoch(end)]
    raw, base, sig, state = [values(k, 5)[ix] for k in ('raw', 'baseline', 'signal', 'state')]
    summary['windows'][label] = dict(start=start, end=end, samples=int(ix.sum()),
        raw_range=[int(raw.min()), int(raw.max())], baseline_range=[int(base.min()), int(base.max())],
        residual_range=[int((raw-base).min()), int((raw-base).max())], signal_range=[int(sig.min()), int(sig.max())],
        state_samples=dict(Counter(names[int(s)] for s in state)), pressure_records=len(pr),
        pressure_range=[min(float(r['raw_pressure_n']) for r in pr), max(float(r['raw_pressure_n']) for r in pr)],
        positions=sorted(set(int(r['position_counts']) for r in pr if r['position_counts'])))
state = values('state', 5)
starts = np.r_[0, np.flatnonzero(np.diff(state)) + 1]
summary['ch5_runs'] = [dict(start=rows[int(i)]['timestamp'], state=names[int(state[i])],
    duration_s=round(float(stamp[j]-stamp[i]), 3), raw=int(rows[i]['raw_5']), signal=int(rows[i]['signal_5']))
    for i, j in zip(starts, np.r_[starts[1:], len(rows)-1])]
summary['operations'] = [{k: r.get(k) for k in ('timestamp', 'event', 'target_force_n', 'raw_pressure_n', 'position_counts', 'stop_reason', 'message')}
                         for r in motor if r['event'] in ('operation_started', 'operation_complete')]
(OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: summary[k] for k in ('windows', 'ch5_runs', 'operations')}, ensure_ascii=False, indent=2))

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
sys.path.insert(0, str(DATA))
from qt_compat import QtWidgets, QtGui
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
app.setFont(QtGui.QFont('Microsoft YaHei', 10))
pg.setConfigOptions(antialias=True, background='w', foreground='#233247')
view = pg.GraphicsLayoutWidget()
view.resize(1400, 1100)
view.addLabel('CH5：卸载后 Signal 为零，真实状态仍长期停留 FLOATING', size='18pt', bold=True)
x = stamp - stamp[0]
def panel(title, label):
    view.nextRow()
    p = view.addPlot(title=title)
    p.setLabel('left', label)
    p.getAxis('left').setWidth(125)
    p.setLabel('bottom', '距 09:30:37.294', units='s')
    p.showGrid(x=True, y=True, alpha=.15)
    p.setXRange(0, float(x[-1]), padding=0)
    for s in ('09:32:48.615', '09:33:10.513'):
        p.addItem(pg.InfiniteLine(pos=epoch(s)-stamp[0], pen=pg.mkPen('#bdc4cc', style=pg.QtCore.Qt.PenStyle.DashLine)))
    return p
p = panel('压力计：两次 1.5 N 按压，其他时段接近零', '压力 / N')
p.plot([datetime.fromisoformat(r['timestamp']).timestamp()-stamp[0] for r in fresh],
       [float(r['raw_pressure_n']) for r in fresh], pen=pg.mkPen('#187bb9', width=2))
p = panel('CH5 基线三分钟固定在 2359，卸载 Raw 仍高出约 500 counts', '计数')
p.addLegend()
p.plot(x, values('raw', 5), pen=pg.mkPen('#bf345d', width=2), name='Raw')
p.plot(x, values('baseline', 5), pen=pg.mkPen('#187bb9', width=2), name='Baseline')
p = panel('CH5 Signal：卸载回零，不代表状态机已识别松手', 'Signal / count')
p.plot(x, values('signal', 5), pen=pg.mkPen('#268168', width=2))
p = panel('CH5 状态：始终只有 FLOATING / RISING / FALLING，未进入恢复或空闲', '真实状态')
p.getAxis('left').setTicks([[(i, names[i]) for i in (0, 1, 3, 4, 6, 7, 8)]])
p.setYRange(-.4, 8.4)
p.plot(x, state, pen=pg.mkPen('#863eb7', width=2))
view.show()
app.processEvents()
exporter = ImageExporter(view.scene())
exporter.parameters()['width'] = 1400
exporter.export(str(OUT / 'ch5_state.png'))
view.close()

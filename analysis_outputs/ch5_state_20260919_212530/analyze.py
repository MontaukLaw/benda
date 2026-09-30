"""固定三分钟快照的CH5状态分析；只读传感器/电机记录，回放真实C状态机。"""
import csv
import hashlib
import io
import json
import os
from collections import Counter
from datetime import datetime
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DATA = ROOT/'PY_SRIPT/algo_monitor'
SOURCE = DATA/'recordings/sensor_data_3min_20260919_212530_626844_e70c8d76.csv'
EXPECTED_SHA = 'ed88a4a8a6ff5c580012bb19302dfd0d9f3cdf1fa2562038eecbc3ee60e99557'
blob = SOURCE.read_bytes()
assert hashlib.sha256(blob).hexdigest() == EXPECTED_SHA
meta = json.loads(SOURCE.with_suffix('.meta.json').read_text(encoding='utf8'))
all_rows = list(csv.DictReader(io.StringIO(blob.decode('utf-8-sig'))))
rows = [r for r in all_rows if r['type'] in ('debug', 'signal')]
assert len(rows) == meta['sensor_sample_count']
stamp = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in rows])
values = lambda name, c: np.array([float(r[f'{name}_{c}']) for r in rows])
epoch = lambda s: datetime.fromisoformat('2026-09-19T'+s+'+08:00').timestamp()
names = ['IDLE', 'RISING', 'HOLD', 'FALLING', 'SLOW_BACK', 'ZERO_REBASE', 'FLOATING', 'RECOVER', 'WAIT_STABLE']
tail = stamp >= epoch('21:25:16')
summary = dict(source=str(SOURCE), csv_sha256=EXPECTED_SHA, meta=meta,
               actual_sensor_count=len(rows), actual_span_s=float(stamp[-1]-stamp[0]),
               actual_max_gap_s=float(np.diff(stamp).max()), channels={})
for c in range(4, 8):
    raw, base, sig, state = [values(key, c)[tail] for key in ('raw', 'baseline', 'signal', 'state')]
    summary['channels'][c] = dict(raw_range=[int(raw.min()), int(raw.max())],
        baseline_range=[int(base.min()), int(base.max())], signal_range=[int(sig.min()), int(sig.max())],
        residual_range=[int((raw-base).min()), int((raw-base).max())],
        transitions=int(np.count_nonzero(np.diff(state))),
        state_samples=dict(Counter(names[int(s)] for s in state)))

replay_rows = [r for r in rows if r['timestamp'] >= '2026-09-19T21:25:09']
assert set(r['baseline_5'] for r in replay_rows if r['timestamp'] >= '2026-09-19T21:25:10.336') == {'7591'}
sys.path.insert(0, str(ROOT/'tools'))
from test_channel_state_pressure import build_model
state, fast, slow = build_model().run([int(r['raw_5']) for r in replay_rows], baseline=7591, noise=1)
summary['replay'] = dict(start=replay_rows[0]['timestamp'], end=replay_rows[-1]['timestamp'], frames=len(state),
    fixed_baseline=7591, initial_idle_baseline_range=sorted(set(int(r['baseline_5']) for r in replay_rows)),
    state_matches=sum(s == int(r['state_5']) for s, r in zip(state, replay_rows)),
    fast_k_matches=sum(abs(k/100-float(r['fast_k_percent_5'])) < .001 for k, r in zip(fast, replay_rows)),
    slow_k_matches=sum(abs(k/100-float(r['slow_k_percent_5'])) < .001 for k, r in zip(slow, replay_rows)),
    c_source_sha256=hashlib.sha256((ROOT/'channel_state_machine.c').read_bytes()).hexdigest())

# 电机记录是持续追加文件；只保留快照截止时刻之前的交叉核对数据。
motor_source = DATA/'force_press_tests/zdt_x42s_1_3200/force_press_20260919_212312_322.csv'
motor = [r for r in csv.DictReader(motor_source.open(encoding='utf-8-sig'))
         if rows[0]['timestamp'] <= r['timestamp'] <= rows[-1]['timestamp']]
fresh = [r for r in motor if r['timestamp'] >= '2026-09-19T21:25:16'
         and r['raw_pressure_n'] and r['ocr_age_ms'] and float(r['ocr_age_ms']) < 500]
summary['motor_tail'] = dict(source=str(motor_source), fresh_pressure_records=len(fresh),
    pressure_range=[min(float(r['raw_pressure_n']) for r in fresh), max(float(r['raw_pressure_n']) for r in fresh)],
    position_values=sorted(set(int(r['position_counts']) for r in fresh if r['position_counts'])))
summary['operations'] = [{k:r[k] for k in ('timestamp','event','target_force_n','position_counts','stop_reason','message')}
                         for r in motor if r['event'] in ('operation_started','operation_complete')]
(OUT/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf8')

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['QT_QPA_FONTDIR'] = 'C:/Windows/Fonts'
sys.path.insert(0, str(DATA))
from qt_compat import QtWidgets, QtGui
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
app.setFont(QtGui.QFont('Microsoft YaHei', 10))
pg.setConfigOptions(antialias=True, background='w', foreground='#233247')
view = pg.GraphicsLayoutWidget()
view.resize(1280, 1060)
view.addLabel('CH5：外力已回零，状态仍反复跳变', size='18pt', bold=True)
origin = epoch('21:25:09')
ix = np.flatnonzero(stamp >= origin)
x = stamp[ix]-origin
panels = []
def plot(title, label):
    view.nextRow()
    p = view.addPlot(title=title)
    p.setLabel('left', label)
    p.getAxis('left').setWidth(140)
    p.setLabel('bottom', '从 21:25:09 起', units='s')
    p.showGrid(x=True, y=True, alpha=.15)
    p.setXRange(0, stamp[-1]-origin, padding=0)
    p.addItem(pg.InfiniteLine(pos=7, angle=90, pen=pg.mkPen('#a1a7af', style=pg.QtCore.Qt.PenStyle.DashLine)))
    panels.append(p)
    return p
p = plot('压力计与电机：额外受力结束后，压力保持 0.02 N；电机一直停在 3 step', '压力 / N')
pr = [r for r in motor if r['timestamp'] >= '2026-09-19T21:25:09' and r['raw_pressure_n']
      and r['ocr_age_ms'] and float(r['ocr_age_ms']) < 500]
p.plot([datetime.fromisoformat(r['timestamp']).timestamp()-origin for r in pr],
       [float(r['raw_pressure_n']) for r in pr], pen=pg.mkPen('#187bb9', width=2))
p = plot('CH5 Raw 相对冻结基线的残差：后段一直为 12～20 counts', 'Raw − Baseline / count')
p.addLegend(offset=(10, 10))
p.plot(x, (values('raw', 5)-values('baseline', 5))[ix], pen=pg.mkPen('#bc305e', width=1.5), name='Raw − Baseline')
p.plot([0, x[-1]], [10, 10], pen=pg.mkPen('#ba840c', width=2), name='退出门槛 10 counts')
p = plot('CH5 Signal：后段持续为 0，不能等同于状态已返回空闲', 'Signal / count')
p.plot(x, values('signal', 5)[ix], pen=pg.mkPen('#328456', width=2))
p = plot('CH5 真实上报状态：21:25:16 后，14.6 秒内切换 74 次', '状态')
p.getAxis('left').setTicks([[(i, names[i]) for i in (0, 1, 3, 4, 6)]])
p.plot(x, values('state', 5)[ix], stepMode='left', pen=pg.mkPen('#7042a2', width=1.5))
p.setYRange(-.3, 6.5, padding=0)
view.nextRow()
view.addLabel('快照实际范围 21:23:13.100–21:25:30.619，共 137.519 秒；虚线为 21:25:16。状态及统计均使用完整采样。', size='10pt')
view.show()
app.processEvents()
exporter = ImageExporter(view.scene())
exporter.parameters()['width'] = 1600
exporter.export(str(OUT/'ch5_state.png'))
view.close()
print(json.dumps(dict(channels=summary['channels'], replay=summary['replay'], motor_tail=summary['motor_tail']), ensure_ascii=False))

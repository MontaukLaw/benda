"""固定五分钟快照：核对30 N释放后的Raw下冲回升与状态误触发。"""
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
SOURCE = DATA / 'recordings/sensor_data_5min_20260920_094719_252791_6f3f8b12.csv'
blob = SOURCE.read_bytes()
meta = json.loads(SOURCE.with_suffix('.meta.json').read_text(encoding='utf-8'))
assert hashlib.sha256(blob).hexdigest() == meta['csv_sha256']
all_rows = list(csv.DictReader(io.StringIO(blob.decode('utf-8-sig'))))
rows = [r for r in all_rows if r['type'] in ('debug', 'signal')]
assert len(rows) == meta['sensor_sample_count']
t = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in rows])
ep = lambda s: datetime.fromisoformat('2026-09-20T' + s + '+08:00').timestamp()
v = lambda k, c: np.array([int(r[f'{k}_{c}']) for r in rows])
names = ['IDLE', 'RISING', 'HOLD', 'FALLING', 'SLOW_BACK', 'ZERO_REBASE', 'FLOATING', 'RECOVER', 'WAIT_STABLE']
motor_path = DATA / 'force_press_tests/zdt_x42s_1_3200/force_press_20260920_094220_587.csv'
copy = OUT / 'motor_snapshot.csv'
if not copy.exists():
    m = [r for r in csv.DictReader(io.StringIO(motor_path.read_text(encoding='utf-8-sig')))
         if rows[0]['timestamp'] <= r['timestamp'] <= rows[-1]['timestamp']]
    with copy.open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(m[0]))
        w.writeheader()
        w.writerows(m)
motor = list(csv.DictReader(copy.open(encoding='utf-8-sig')))
fresh = [r for r in motor if r['raw_pressure_n'] and r['ocr_age_ms'] and 0 <= float(r['ocr_age_ms']) <= 500]
tail = [r for r in fresh if ep('09:46:41') <= datetime.fromisoformat(r['timestamp']).timestamp()]
summary = dict(meta=meta, actual_count=len(rows), actual_span_s=float(t[-1]-t[0]),
               actual_max_gap_s=float(np.diff(t).max()), frame_counts=dict(Counter(r['type'] for r in all_rows)),
               temperature_values=sorted(set(r['temperature_celsius'] for r in all_rows if r['type']=='temperature')),
               unloaded_pressure=dict(records=len(tail), minimum=min(float(r['raw_pressure_n']) for r in tail),
               maximum=max(float(r['raw_pressure_n']) for r in tail), positions=sorted(set(r['position_counts'] for r in tail)),
               speeds=sorted(set(r['measured_speed_rpm'] for r in tail))), channels={})
for c in range(4, 8):
    raw = v('raw', c)
    before = float(np.median(raw[(t>=ep('09:46:21')) & (t<ep('09:46:23'))]))
    ix = np.flatnonzero(t>=ep('09:46:36.603'))
    low = ix[np.argmin(raw[ix])]
    summary['channels'][c] = dict(before_median=before, min_after_release=int(raw[low]),
        min_timestamp=rows[int(low)]['timestamp'], end_median=float(np.median(raw[t>=ep('09:47:18')])) )
state = v('state', 5)
ix = np.flatnonzero(t>=ep('09:46:36'))
summary['ch5_transitions'] = [dict(timestamp=rows[int(i)]['timestamp'], state=names[state[i]],
    raw=int(v('raw', 5)[i]), baseline=int(v('baseline', 5)[i]), signal=int(v('signal', 5)[i]))
    for j, i in enumerate(ix) if j==0 or state[i]!=state[ix[j-1]]]
summary['operations'] = [{k:r[k] for k in ('timestamp','event','target_force_n','raw_pressure_n','position_counts','message')}
    for r in motor if r['event'] in ('operation_started','operation_complete') and r['timestamp']>='2026-09-20T09:46:20']
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:summary[k] for k in ('actual_count','actual_span_s','actual_max_gap_s','unloaded_pressure','channels')},ensure_ascii=False,indent=2))

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
sys.path.insert(0,str(DATA))
from qt_compat import QtWidgets, QtGui
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
app.setFont(QtGui.QFont('Microsoft YaHei',10))
pg.setConfigOptions(antialias=True,background='w',foreground='#233247')
view=pg.GraphicsLayoutWidget()
view.resize(1400,1180)
view.addLabel('30 N 释放：CH5 Raw 下冲后回升，基线过早追低导致再次误触发',size='17pt',bold=True)
origin=ep('09:46:20')
def panel(title,label,post=False):
    view.nextRow()
    p=view.addPlot(title=title)
    p.setLabel('left',label)
    p.getAxis('left').setWidth(115)
    p.setLabel('bottom','距 09:46:20',units='s')
    p.setXRange(19 if post else 0, t[-1]-origin,padding=0)
    p.showGrid(x=True,y=True,alpha=.15)
    for at in ('09:46:36.603','09:46:40.198'):
        p.addItem(pg.InfiniteLine(pos=ep(at)-origin,pen=pg.mkPen('#a1a8af',style=pg.QtCore.Qt.PenStyle.DashLine)))
    return p
p=panel('压力计：09:46:36.603开始释放，09:46:40.198回原位；之后0.03～0.04 N','压力 / N')
pr=[r for r in fresh if datetime.fromisoformat(r['timestamp']).timestamp()>=origin]
p.plot([datetime.fromisoformat(r['timestamp']).timestamp()-origin for r in pr],[float(r['raw_pressure_n']) for r in pr],pen=pg.mkPen('#2478b5',width=2))
p=panel('释放后的相对Raw（减去按压前中位数）：CH5从下方回升，CH7也有小幅下冲','相对Raw / count',True)
p.addLegend()
mask=t>=ep('09:46:39')
for c,color in zip(range(4,8),['#197fb1','#c52c61','#31936f','#b77a1f']):
    p.plot(t[mask]-origin,v('raw',c)[mask]-summary['channels'][c]['before_median'],pen=pg.mkPen(color,width=2),name=f'CH{c}')
p.addItem(pg.InfiniteLine(pos=0,angle=0,pen=pg.mkPen('#8b949d',style=pg.QtCore.Qt.PenStyle.DashLine)))
p=panel('CH5：Raw先跌到2274再回升；Baseline追到约2513后冻结','计数',True)
p.addLegend()
for key,color in [('raw','#c52c61'),('baseline','#2478b5')]:
    p.plot(t[mask]-origin,v(key,5)[mask],pen=pg.mkPen(color,width=2),name=key)
p.addItem(pg.InfiniteLine(pos=2865,angle=0,pen=pg.mkPen('#939ba4',style=pg.QtCore.Qt.PenStyle.DashLine)))
p=panel('CH5：09:46:43.294误判RISING；09:46:49.321进入FLOATING并锁定Signal=149','真实状态',True)
p.getAxis('left').setTicks([[(i,names[i]) for i in (0,1,3,4,6,7,8)]])
p.setYRange(-.4,8.4)
p.plot(t[mask]-origin,state[mask],pen=pg.mkPen('#823bb0',width=2))
view.show()
app.processEvents()
exporter=ImageExporter(view.scene())
exporter.parameters()['width']=1400
exporter.export(str(OUT/'ch5_30n_release.png'))
view.close()

"""复核固定五分钟快照并绘制卸载恢复；不连接设备。"""
import csv
import hashlib
import json
import os
from datetime import datetime
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DATA = ROOT / 'PY_SRIPT/algo_monitor'
source = DATA / 'recordings/sensor_data_5min_20260919_190459_985152_a7878246.csv'
meta = json.loads(source.with_suffix('.meta.json').read_text(encoding='utf-8'))
assert hashlib.sha256(source.read_bytes()).hexdigest() == meta['csv_sha256']
rows = []
for row in csv.DictReader(source.open(encoding='utf-8-sig')):
    if row['type'] in ('debug', 'signal'):
        rows.append(row)
assert len(rows) == meta['sensor_sample_count']
stamp = np.array([datetime.fromisoformat(r['timestamp']).timestamp() for r in rows])
raw = np.array([[float(r[f'raw_{c}']) for c in range(8)] for r in rows])
baseline = np.array([[float(r[f'baseline_{c}']) for c in range(8)] for r in rows])
epoch = lambda t: datetime.fromisoformat('2026-09-19T'+t+'+08:00').timestamp()
cycles = [('25 N', '19:02:36', '19:02:57.539', '19:03:00.921', '19:03:43'),
          ('30 N', '19:03:43', '19:04:06.773', '19:04:10.016', '19:04:59.983')]
summary = dict(source=str(source), sha256=meta['csv_sha256'], meta=meta,
               actual_max_gap_s=float(np.diff(stamp).max()), cycles=[])
for label, pre, release, complete, end in cycles:
    pre, release, complete, end = map(epoch, (pre, release, complete, end))
    before = np.median(raw[(stamp >= pre-2) & (stamp < pre)], axis=0)
    selected = np.flatnonzero((stamp >= release) & (stamp <= end))
    result = dict(label=label, channels={})
    for c in range(4, 8):
        minimum_index = selected[np.argmin(raw[selected, c])]
        result['channels'][c] = dict(pre_median=float(before[c]), minimum=float(raw[minimum_index, c]),
            minimum_at=rows[minimum_index]['timestamp'],
            end_median=float(np.median(raw[(stamp > end-2) & (stamp <= end), c])))
    summary['cycles'].append(result)
(OUT/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ.setdefault('QT_QPA_FONTDIR', 'C:/Windows/Fonts')
sys.path.insert(0, str(DATA))
from qt_compat import QtWidgets, QtGui
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
app.setFont(QtGui.QFont('Microsoft YaHei', 11))
pg.setConfigOptions(antialias=True, background='w', foreground='#243247')
view = pg.GraphicsLayoutWidget()
view.resize(1280, 880)
view.addLabel('CH5 卸载后先下冲、再回升：硬件 Raw 对比', size='18pt', bold=True)
view.nextRow()
plot = view.addPlot(title='25 N / 30 N：CH5 相对本轮按压前 Raw 的变化')
plot.addLegend(offset=(650, 20))
plot.setLabel('left', 'Raw 相对按压前变化', units='count')
plot.setLabel('bottom', '电机确认回到释放位置后', units='s')
plot.showGrid(x=True, y=True, alpha=.15)
for (label, pre, release, complete, end), color in zip(cycles, ('#b56809', '#b8295e')):
    pre, complete, end = map(epoch, (pre, complete, end))
    before = np.median(raw[(stamp >= pre-2) & (stamp < pre), 5])
    ix = np.flatnonzero((stamp >= complete) & (stamp <= end))[::10]
    plot.plot(stamp[ix]-complete, raw[ix, 5]-before, pen=pg.mkPen(color, width=2), name=label)
plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#788595', style=pg.QtCore.Qt.PenStyle.DashLine)))
plot.setXRange(0, 50, padding=0)
view.nextRow()
plot2 = view.addPlot(title='30 N 卸载后：CH4 / CH5 / CH6 / CH7（各自减去按压前 Raw）')
plot2.addLegend(offset=(650, 20))
plot2.setLabel('left', 'Raw 相对按压前变化', units='count')
plot2.setLabel('bottom', '电机确认回到释放位置后', units='s')
plot2.showGrid(x=True, y=True, alpha=.15)
pre, complete, end = map(epoch, ('19:03:43', '19:04:10.016', '19:04:59.983'))
before = np.median(raw[(stamp >= pre-2) & (stamp < pre)], axis=0)
ix = np.flatnonzero((stamp >= complete) & (stamp <= end))[::10]
for c, color in zip(range(4, 8), ('#1677b8', '#b8295e', '#318759', '#9465aa')):
    plot2.plot(stamp[ix]-complete, raw[ix, c]-before[c], pen=pg.mkPen(color, width=2), name=f'CH{c}')
plot2.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#788595', style=pg.QtCore.Qt.PenStyle.DashLine)))
plot2.setXRange(0, 50, padding=0)
view.nextRow()
view.addLabel('快照：2026-09-19 18:59:59–19:04:59；按压前参考取启动前 2 秒中位数。曲线抽取显示，统计使用全部采样。', size='10pt')
view.show()
app.processEvents()
exporter = ImageExporter(view.scene())
exporter.parameters()['width'] = 1600
exporter.export(str(OUT/'recovery.png'))
view.close()
print(json.dumps(summary['cycles'], ensure_ascii=False))

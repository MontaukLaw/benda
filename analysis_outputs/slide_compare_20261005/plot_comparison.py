"""导出本轮双模式16毫米测试的距离与轨迹对照图。"""
import json
import os
from pathlib import Path
import sys
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
import pyqtgraph as pg
import pyqtgraph.exporters
from qt_compat import QtWidgets,QtGui

OUT=Path(__file__).parent
reports={m:json.loads((OUT/(m+'.json')).read_text(encoding='utf-8')) for m in ('host_continuous','host_signal')}
app=QtWidgets.QApplication([])
for name in ('segoeui.ttf','msyh.ttc'):QtGui.QFontDatabase.addApplicationFont(str(Path('C:/Windows/Fonts')/name))
app.setFont(QtGui.QFont('Segoe UI',12))
pg.setConfigOptions(background='w',foreground='#16212d',antialias=True)
view=pg.GraphicsLayoutWidget();view.resize(1600,1080)
view.addLabel('16 mm horizontal / vertical | 10 trials per axis per mode | 2026-10-05 14:41 / 14:45',colspan=2)
view.nextRow()
for axis,label in (('horizontal','Horizontal'),('vertical','Vertical')):
    plot=view.addPlot(title=label+' output distance (lower CV means more repeatable)')
    plot.setLabel('bottom','Trial');plot.setLabel('left','Main-axis output','px');plot.addLegend();plot.showGrid(x=True,y=True,alpha=.2)
    for mode,color,name in (('host_continuous','#1478ad','Continuous weights'),('host_signal','#d15d1a','Gated Signal')):
        group=reports[mode]['summary'][axis]
        plot.plot(list(range(1,11)),group['values'],pen=pg.mkPen(color,width=2),symbol='o',symbolBrush=color,
                  name=f"{name} | CV {100*group['cv']:.1f}%")
view.nextRow()
for axis,label in (('horizontal','Horizontal'),('vertical','Vertical')):
    plot=view.addPlot(title=label+' trajectories from each starting cursor position')
    plot.setLabel('bottom','X cursor displacement','px');plot.setLabel('left','Y cursor displacement (positive down)','px')
    plot.invertY(True);plot.addLegend();plot.showGrid(x=True,y=True,alpha=.2)
    for mode,color,name in (('host_continuous','#1478ad','Continuous weights'),('host_signal','#d15d1a','Gated Signal')):
        first=True
        for trial in reports[mode]['trials']:
            if trial['axis']!=axis:continue
            rows=[r for r in trial['traces'] if r['contact']]
            plot.plot([r['cum'][0] for r in rows],[r['cum'][1] for r in rows],pen=pg.mkPen(color,width=1.5),name=name if first else None)
            plot.plot([trial['net'][0]],[trial['net'][1]],pen=None,symbol='o',symbolSize=6,symbolBrush=color,symbolPen=color)
            first=False
view.show();app.processEvents()
exporter=pg.exporters.ImageExporter(view.scene());exporter.parameters()['width']=1800
exporter.export(str(OUT/'comparison.png'));view.close()
print(OUT/'comparison.png')

import json
import os
from pathlib import Path
import sys

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
import pyqtgraph as pg
import pyqtgraph.exporters
from qt_compat import QtWidgets, QtGui

output=Path(__file__).parent
audit=json.loads((output/'audit.json').read_text(encoding='utf-8'))
series=json.loads((output/'series.json').read_text(encoding='utf-8'))
app=QtWidgets.QApplication([])
for name in ('segoeui.ttf','msyh.ttc'):
    font=Path('C:/Windows/Fonts')/name
    if font.exists(): QtGui.QFontDatabase.addApplicationFont(str(font))
app.setFont(QtGui.QFont('Segoe UI',12))
pg.setConfigOptions(background='w',foreground='#16212d',antialias=True)
view=pg.GraphicsLayoutWidget()
view.resize(1600,1050)
view.addLabel('20 sliding trials | 2026-10-05 13:58:38 - 14:01:23 | Physical lengths differ between axes',colspan=2)
view.nextRow()
p1=view.addPlot(title='Distance variation within each direction')
p1.setLabel('bottom','Trial')
p1.setLabel('left','Output / direction mean')
p1.addLegend()
p1.showGrid(x=True,y=True,alpha=.2)
p1.addLine(y=1,pen=pg.mkPen('#aaaaaa',style=pg.QtCore.Qt.PenStyle.DashLine))
p2=view.addPlot(title='Unwanted off-axis displacement')
p2.setLabel('bottom','Trial')
p2.setLabel('left','Off-axis / main-axis','%')
p2.addLegend()
p2.showGrid(x=True,y=True,alpha=.2)
p2.addLine(y=0,pen=pg.mkPen('#aaaaaa'))
for axis,color,label in (('horizontal','#1478ad','Horizontal'),('vertical','#d15d1a','Vertical')):
    trials=[s for s in audit['trials'] if s['axis']==axis]
    x=[s['trial'] for s in trials]
    mean=audit['summary'][axis]['mean_pixels']
    p1.plot(x,[s['main_pixels']/mean for s in trials],pen=pg.mkPen(color,width=2),symbol='o',symbolBrush=color,name=label)
    p2.plot(x,[100*s['cross_pixels']/s['main_pixels'] for s in trials],pen=pg.mkPen(color,width=2),symbol='o',symbolBrush=color,name=label)
view.nextRow()
p3=view.addPlot(title='Horizontal trial 5: centroid discontinuities')
p3.setLabel('bottom','Time after sliding prompt','s')
p3.setLabel('left','Horizontal centroid','cell')
p3.addLegend()
p3.showGrid(x=True,y=True,alpha=.2)
trial=next(s for s in series if s['axis']=='horizontal' and s['trial']==5)
rows=[t for t in trial['traces'] if t['contact']]
p3.plot([r['time'] for r in rows],[r['raw'][0] for r in rows],pen=pg.mkPen('#d15d1a',width=2),name='Current gated Signal')
p3.plot([r['time'] for r in rows],[r['soft'][0] for r in rows],pen=pg.mkPen('#1478ad',width=2),name='Continuous positive input - baseline')
p3.addLine(x=.9203623,pen=pg.mkPen('#707070',style=pg.QtCore.Qt.PenStyle.DashLine))
p4=view.addPlot(title='Offline release-guard comparison (same recorded input)')
p4.setLabel('bottom','Trial')
p4.setLabel('left','Horizontal output','px')
p4.addLegend()
p4.showGrid(x=True,y=True,alpha=.2)
variants=json.loads((output/'variants.json').read_text(encoding='utf-8'))
for key,color,label in (('15','#d15d1a','Current 15%'),('50','#4f9c65','50% (diagnostic only)'),('disabled','#1478ad','Disabled (diagnostic only)')):
    p4.plot(list(range(1,11)),variants[key]['summary']['horizontal']['values'],pen=pg.mkPen(color,width=2),symbol='o',symbolBrush=color,name=label)
view.show()
app.processEvents()
exporter=pg.exporters.ImageExporter(view.scene())
exporter.parameters()['width']=1800
exporter.export(str(output/'comparison.png'))
view.close()
print(output/'comparison.png')

"""用上位机已有pyqtgraph导出同力同保持条件的重复回弹对照图。"""
import argparse
import csv
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from qt_compat import QtWidgets, QtGui
import pyqtgraph as pg
import pyqtgraph.exporters


def plot(directory):
    directory=Path(directory)
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    if os.name == 'nt':
        QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
    app.setFont(QtGui.QFont('Microsoft YaHei',11))
    pg.setConfigOptions(background='w',foreground='#172536',antialias=True)
    with (directory/'recovery_1s.csv').open(encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    w=pg.GraphicsLayoutWidget();w.resize(1500,1000)
    w.addLabel('10 N × 30 s：两次重复的回弹曲线（从确认卸载后第5秒起）',row=0,col=0,colspan=2,size='20pt')
    for j,c in enumerate(range(4,8)):
        p=w.addPlot(row=1+j//2,col=j%2,title=f'CH{c}')
        p.addLegend(offset=(15,15));p.showGrid(x=True,y=True,alpha=.2)
        p.setLabel('bottom','确认卸载后的时间',units='s')
        p.setLabel('left','相对本次按压前Raw的偏差',units='counts')
        for i,color,label in ((1,'#1876b9','第1项 · 实测9.78 N'),(15,'#dc6c24','第15项 · 实测9.82 N')):
            data=[r for r in rows if int(r['planned_trial_index'])==i and int(r['channel'])==c and float(r['seconds'])>=5]
            p.plot([float(r['seconds']) for r in data],[float(r['residual_counts']) for r in data],pen=pg.mkPen(color,width=2.5),name=label)
        p.addLine(y=0,pen=pg.mkPen('#8899aa',width=1))
        p.setXRange(0,360,padding=0)
    w.addLabel('按压前参考不代表充分恢复；后半段上漂与升温同时出现，不能全部解释为按压残余。',row=3,col=0,colspan=2,size='13pt')
    w.show()
    for _ in range(5): app.processEvents()
    export=pg.exporters.ImageExporter(w.scene());export.parameters()['width']=1800
    target=directory/'repeat_10N_30s.png';export.export(str(target));w.close()
    print(target)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('analysis_directory')
    plot(parser.parse_args().analysis_directory)

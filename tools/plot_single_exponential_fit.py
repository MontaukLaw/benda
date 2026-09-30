"""导出单指数留出检验图和逐点预测，使用已保存参数，不重新拟合。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
from fit_rebound_single_exponential import load_curves,model
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from qt_compat import QtWidgets,QtGui
import pyqtgraph as pg
import pyqtgraph.exporters


def plot(out):
    out=Path(out);fit=json.loads((out/'fit.json').read_text(encoding='utf8'))
    report,curves,_=load_curves(Path(fit['input_report']))
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
    app.setFont(QtGui.QFont('Microsoft YaHei',11))
    pg.setConfigOptions(background='w',foreground='#172536',antialias=True)
    def add(w,row,col,c,T,tail=False):
        x=next(x for x in curves if x['channel']==c and x['hold_requested_s']==T and x['role']=='validation')
        f=fit['channels'][str(c)]
        title=f'CH{c} · 保持{T:g}秒 · 第{x["trial"]}次' + (' · 后段放大' if tail else '')
        p=w.addPlot(row=row,col=col,title=title);p.showGrid(x=True,y=True,alpha=.2)
        p.addLegend(offset=(-10,10))
        t=np.arange(600)+.5;y=np.full(600,np.nan);y[(x['t']-.5).astype(int)]=x['y']
        pred=model(t,x['D'],x['T'],f['k'],f['tau_seconds'])
        if tail:
            hit=t>=120;t=t[hit];y=y[hit];pred=pred[hit]
            p.setXRange(120,600,padding=.01)
        else:
            p.setLogMode(x=True);p.setXRange(np.log10(.5),np.log10(600),padding=.01)
            p.getAxis('bottom').setTicks([[(np.log10(v),str(v)) for v in (1,3,10,30,100,300,600)]])
        p.plot(t,y,pen=pg.mkPen('#0072B2',width=2),name='实测残差',connect='finite')
        p.plot(t,pred,pen=pg.mkPen('#D55E00',width=2.5),name='固定系数预测')
        p.addLine(y=0,pen=pg.mkPen('#8899aa'))
        p.setLabel('left','Raw残差',units='counts');p.getAxis('left').setWidth(65)
        p.getAxis('left').enableAutoSIPrefix(False)
        p.setLabel('bottom','确认卸载后时间'+('' if tail else '（对数刻度）'),units='s')
    def export(w,name,width):
        w.show()
        for _ in range(5):app.processEvents()
        exporter=pg.exporters.ImageExporter(w.scene());exporter.parameters()['width']=width
        exporter.export(str(out/name));w.close()
    w=pg.GraphicsLayoutWidget();w.resize(1700,1120)
    f=fit['channels']['5']
    w.addLabel('CH5：前6次拟合，后3次检验同一组系数',row=0,col=0,colspan=2,size='22pt')
    w.addLabel(f'k={f["k"]:.6f}，τ={f["tau_seconds"]:.3f}秒；蓝线实测，橙线预测；没有扣除漂移或另加慢分量',row=1,col=0,colspan=2,size='13pt')
    add(w,2,0,5,2);add(w,2,1,5,10);add(w,3,0,5,60);add(w,3,1,5,60,True)
    w.addLabel('拟合范围为全部有效的一秒窗（约0.5～599.5秒）；横轴对数仅用于看图。目标2 N，实际力随保持时长变化。',row=4,col=0,colspan=2,size='12pt')
    export(w,'validation_ch5.png',1900)
    w=pg.GraphicsLayoutWidget();w.resize(2100,1800)
    w.addLabel('四通道独立检验：每行固定一组k、τ，预测三种保持时间',row=0,col=0,colspan=3,size='22pt')
    for i,c in enumerate((4,5,6,7)):
        for j,T in enumerate((2,10,60)): add(w,i+1,j,c,T)
    w.addLabel('蓝线实测、橙线预测；每列分别保持2、10、60秒。横轴对数，纵轴counts；无漂移扣除，无快慢分段。',row=5,col=0,colspan=3,size='13pt')
    export(w,'validation_all_channels.png',2300)
    with (out/'predictions_1s.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['trial','channel','role','hold_seconds','seconds','observed_counts','predicted_counts','error_counts'])
        for x in curves:
            params=fit['channels'][str(x['channel'])]
            pred=model(x['t'],x['D'],x['T'],params['k'],params['tau_seconds'])
            writer.writerows([x['trial'],x['channel'],x['role'],x['T'],t,y,p,p-y] for t,y,p in zip(x['t'],x['y'],pred))
    print('TRIALS',json.dumps([{k:t[k] for k in ('planned_trial_index','hold_requested_s','hold_actual_s','hold_median_n','load_peak_n','temperature_pre_c','temperature_tail_c','quality_issues')} for t in report['trials']],ensure_ascii=False))
    print('BINS',sum(x['missing_bins'] for x in curves))
    for c in (4,5,6,7):
        items=[e for e in fit['errors'] if e['channel']==c and e['role']=='validation']
        print('SUMMARY',c, 'RMSE pooled',float(np.sqrt(np.mean([e['rmse_counts']**2 for e in items]))),
              'RMSE per T',[round(next(e['rmse_counts'] for e in items if e['hold_requested_s']==T),2) for T in (2,10,60)])
    print('OUTPUT',out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('fit_directory',type=Path);plot(p.parse_args().fit_directory)

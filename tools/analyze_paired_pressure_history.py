"""复现2026-09-15这次大小压力组合测试的专项分析；只读原始数据，另存结果。"""
import argparse
import csv
import hashlib
import json
import os
import sqlite3
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'PY_SRIPT/algo_monitor'))
from channel_states import STATE_NAMES


def analyze(session, output):
    manifest = json.loads((session/'manifest.json').read_text(encoding='utf-8'))
    if manifest['session_id'] != '236036da00164e6499e50979a3be0b6d':
        raise ValueError('此脚本的中文结论仅适用于2026-09-15 17:55这份已核对的会话')
    if manifest['status'] != 'completed':
        raise ValueError('需要已完成并保存的会话')
    reports = sorted((session/'reports').glob('*/analysis.json'))
    report_path = next(p for p in reversed(reports) if (p.parent/'complete.json').exists())
    report = json.loads(report_path.read_text(encoding='utf-8'))
    records, events, hashes = [], [], {}
    for part in manifest['output_parts']:
        path = session/part
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        expected = next(p['sha256'] for p in report['source_files'] if Path(p['path']).name == part)
        if digest != expected:
            raise ValueError('原始文件已改变，需要重新计算基础报告')
        hashes[part] = digest
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
            db.row_factory = sqlite3.Row
            records.extend(dict(row) for row in db.execute('SELECT * FROM samples'))
            events.extend(json.loads(row[0]) for row in db.execute(
                "SELECT payload_json FROM events WHERE event='trial_complete' ORDER BY id"))
    output.mkdir(parents=True, exist_ok=False)
    time_axis = np.array([r['sensor_source_monotonic'] for r in records])
    details = []
    for event in events:
        index = event['planned_trial_index']
        parts = event['trial_result']['paired_results']
        press, release = parts['probe']['press_result'], parts['probe']['release_result']
        start, arrival, command = (press['motion_started_monotonic'], press['arrived_monotonic'],
                                    release['release_command_monotonic'])
        pre_mask = (time_axis>=start-1) & (time_axis<start)
        hold_mask = (time_axis>=arrival) & (time_axis<command)
        pre = [records[i] for i in np.flatnonzero(pre_mask)]
        held = [records[i] for i in np.flatnonzero(hold_mask)]
        for ch in range(4,8):
            source = next(r for r in report['channels'] if r['planned_trial_index']==index and r['channel']==ch)
            row = dict(source)
            # 独立核对报告两个窗口，不把大压力数据或运动数据带入小压力参考。
            raw = np.median([r[f'ch{ch}_normalized_raw'] for r in held])
            reference = np.median([r[f'ch{ch}_normalized_raw'] for r in pre])
            if raw != row['hold_raw_median'] or reference != row['pre_raw']:
                raise ValueError(f'第{index}项CH{ch}与基础报告窗口不一致')
            row.update(pre_sample_count=len(pre), hold_sample_count=len(held),
                hold_signal_median=float(np.median([r[f'ch{ch}_signal'] for r in held])),
                pre_signal_median=float(np.median([r[f'ch{ch}_signal'] for r in pre])),
                pre_states=','.join(STATE_NAMES[int(s)] for s in sorted({r[f'ch{ch}_state'] for r in pre})),
                hold_states=','.join(STATE_NAMES[int(s)] for s in sorted({r[f'ch{ch}_state'] for r in held})),
                final_position_counts=press['final_position_counts'])
            row['residual_before_small_percent_of_response'] = 100*row['residual_before_small_counts']/row['hold_response_counts']
            row['signal_minus_increment_counts'] = row['hold_signal_median']-row['hold_response_counts']
            for sec in (10,60):
                v = row[f'residual_{sec}s']
                row[f'after_small_{sec}s_from_large_reference'] = None if v is None else v+row['residual_before_small_counts']
            details.append(row)
    grouped = []
    for ch in range(4,8):
        for delay in (5,60):
            rows = [r for r in details if r['channel']==ch and r['delay_requested_s']==delay]
            values = lambda key: np.array([r[key] for r in rows if r[key] is not None])
            d = values('hold_response_counts')
            grouped.append(dict(channel=ch, delay_seconds=delay, n=len(rows),
                force_n=float(values('hold_median_n').mean()),
                offset_counts=float(values('residual_before_small_counts').mean()),
                offset_min=float(values('residual_before_small_counts').min()),
                offset_max=float(values('residual_before_small_counts').max()),
                offset_percent=float(values('residual_before_small_percent_of_response').mean()),
                response_counts=float(d.mean()), response_cv_percent=float(100*d.std(ddof=1)/d.mean()),
                signal_counts=float(values('hold_signal_median').mean()),
                after_small_10s=float(values('after_small_10s_from_large_reference').mean()),
                after_small_60s=float(values('after_small_60s_from_large_reference').mean())))
    for name, rows in [('press_details.csv', details), ('group_summary.csv', grouped)]:
        with (output/name).open('w', encoding='utf-8-sig', newline='') as f:
            writer=csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (output/'audit.json').write_text(json.dumps(dict(session=str(session.resolve()),
        report=str(report_path.resolve()),sha256=hashes,sampling=report['sessions'],groups=grouped),
        ensure_ascii=False, indent=2), encoding='utf-8')
    text = ['# 20 N之后再压2 N：残差影响复核', '',
        f"来源：`{session.name}`，{manifest['created_at']} 至 {manifest['finished_at']}。",
        '实际10次小压力按压；前9次有180秒释放后记录，末次释放即结束。加载比较使用10次，释放后比较使用前9次。',
        '原始SQLite SHA256与现有自动报告完全一致，40条通道记录的按压前参考及保持Raw已独立核算一致。', '',
        '## 主要发现', '',
        '1. 5秒间隔下，四通道小压力前Raw仍高于本过程20 N按压前参考43～107 counts；60秒间隔为−35～+31.5 counts。它是观测净偏移，可能同时包含残余、慢漂移，不能全部归为纯材料残余。',
        '2. 这份残留相当于本次小压力Raw增量约1.1%～3.7%（各通道组均值），CH4最明显。60秒后明显减小，但正负偏移会抵消，需同时看范围。',
        '3. 5秒组实测力平均2.319 N，60秒组2.565 N，后者高10.6%；两组力范围不重叠。不能据此直接量化同一个真实2 N的残差误差。',
        '4. 所有小压力按压均有明确的RISING和FLOATING记录；保持Signal约2714～5383 counts，按压前Signal中位值均为0。没有看到残差使这批2 N完全无法识别，但Signal有响应不等于测力无偏差。',
        '5. 每组内部Raw增量的标准差/均值约1.0%～2.0%。重复性较好，也存在稳定的组间差异；该差异混合了实际力、加载历史、位移差异和慢漂移。', '',
        '## 小压力开始前的残留', '',
        '残留＝小压力前最后1秒Raw中位值－本过程大压力前最后10秒Raw中位值。百分比以本次小压力Raw增量为分母，不是测力误差百分比。', '',
        '|通道|间隔s|次数|残留均值 counts|残留范围 counts|占本次Raw增量|Raw增量均值 counts|Signal均值 counts|',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for g in grouped:
        text.append(f"|CH{g['channel']}|{g['delay_seconds']}|{g['n']}|{g['offset_counts']:.1f}|{g['offset_min']:.1f}～{g['offset_max']:.1f}|{g['offset_percent']:.2f}%|{g['response_counts']:.1f}|{g['signal_counts']:.1f}|")
    text += ['', '## 每轮实际压力', '', '|轮次|5秒间隔组 N|60秒间隔组 N|', '|---|---:|---:|']
    for repeat in range(1,6):
        pair = [t for t in report['trials'] if t['repeat']==repeat]
        text.append(f"|{repeat}|{pair[0]['hold_median_n']:.3f}|{pair[1]['hold_median_n']:.3f}|")
    text += ['', '实际到位保持约3.000～3.063秒；大压力保持中位力19.67～19.96 N。',
        '小压力到位位置：5秒组−14133～−14145 counts，60秒组−14152～−14155 counts；均不能单凭目标2 N视为等实际加载。', '',
        '## 为什么报告里有负残差', '',
        '报告的小压力恢复曲线以“小压力按压前”为零。5秒组这个零点本身仍含大压力残余；释放之后残余继续消退，会出现负数。负数并不自动表示负压力或读数低于最初空载值。',
        '例如第1项CH4：大压力前6927，小压力前7021，偏移+94。小压力释放10秒后，自动报告残差−38，对应Raw=6983，相对大压力前仍为+56 counts。',
        '小压力释放10秒后，统一相对本过程大压力前参考的偏移均值如下。这里同时包含两次按压的历史影响，无法拆分贡献：', '',
        '|通道|等5秒后再压：释放10秒后 counts|等60秒后再压：释放10秒后 counts|', '|---|---:|---:|']
    for ch in range(4,8):
        a,b=[g for g in grouped if g['channel']==ch]
        text.append(f"|CH{ch}|{a['after_small_10s']:.1f}|{b['after_small_10s']:.1f}|")
    text += ['', '## 数据质量与结论边界', '',
        '共17464帧，实际平均约8.39 Hz；记录器报告零丢弃，但源采样存在23个超过250 ms的间隔，最大约625 ms。所有40个保持窗口覆盖检查通过。',
        '第5项小压力前1秒仅4帧，且大压力前参考窗口有间断；其参考可信度稍低。剔除第5项后，5秒组残留均值CH4～7为99.0、74.6、51.1、59.0 counts，结论未改变。',
        '5秒组小压力前CH4/5处于SLOW BACK，CH6多为SLOW BACK，CH7已IDLE；60秒组多数IDLE，第2项CH5为ZERO REBASE。状态不同不阻止本次加载，但会影响参考处理。',
        '温度各按压前代表值约34.9～35.35°C，大压力前参考也随轮次变化。没有独立长时间空载对照，不能把基线变化全部归因于温度或残余。',
        '后续若要量化“同样真实2 N”的残差偏差，应先让两组实际压力落入同一窄范围，并保留到位后真实压力全过程；另加未经历20 N的2 N对照。当前两组实际力无重叠，直接除以力或用线性回归扣除都可能受传感器非线性影响。',
        '当前可确认：等待5秒仍有可重复的残留，等待60秒明显减小；这批2 N均被识别，没有足够证据把全部组间响应差异归为残差或固件阈值问题。', '',
        '## 图和可复核明细', '',
        '![小压力释放后的净Raw偏移](recovery.png)', '',
        '`press_details.csv`：40条通道明细，含Signal、状态、参考、增量、原始参考下的10秒/60秒偏移。',
        '`group_summary.csv`：按通道与等待间隔分组统计；`audit.json`：源文件SHA256、报告位置和采样质量。']
    (output/'report.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    plot(report_path.parent/'recovery_1s.csv', details, output/'recovery.png')
    print(output.resolve())
    print(json.dumps(grouped, ensure_ascii=False))


def plot(curve_path, details, target):
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    from qt_compat import QtWidgets, QtGui
    import pyqtgraph as pg
    import pyqtgraph.exporters
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
    app.setFont(QtGui.QFont('Microsoft YaHei',11))
    pg.setConfigOptions(background='w',foreground='#172536',antialias=True)
    with curve_path.open(encoding='utf-8-sig') as f: curves=list(csv.DictReader(f))
    w=pg.GraphicsLayoutWidget(); w.resize(1550,1000)
    w.addLabel('2 N松开后的Raw净偏移：以本过程20 N按压前为参考',row=0,col=0,colspan=2,size='20pt')
    for j,ch in enumerate(range(4,8)):
        p=w.addPlot(row=1+j//2,col=j%2,title=f'CH{ch}')
        p.addLegend(offset=(15,15));p.showGrid(x=True,y=True,alpha=.2)
        p.setLabel('bottom','2 N确认卸载后',units='s');p.setLabel('left','Raw净偏移',units='counts')
        for delay,color in ((5,'#df7923'),(60,'#277ab2')):
            series=[]
            for r in details:
                if r['channel']!=ch or r['delay_requested_s']!=delay: continue
                rows=[x for x in curves if int(x['channel'])==ch and int(x['planned_trial_index'])==r['planned_trial_index']]
                if not rows: continue
                a=np.full(180,np.nan)
                for row in rows:
                    sec=int(float(row['seconds']))
                    if sec>=180 or sec<5 or row['covered']!='True' or not row['residual_counts']: continue
                    a[sec]=float(row['residual_counts'])+r['residual_before_small_counts']
                series.append(a)
                p.plot(np.arange(180)+.5,a,pen=pg.mkPen(color+'55',width=1),connect='finite')
            avg=np.array([np.median(col[np.isfinite(col)]) if np.any(np.isfinite(col)) else np.nan
                          for col in np.array(series).T])
            p.plot(np.arange(180)+.5,avg,pen=pg.mkPen(color,width=3),connect='finite',
                   name=f'大压力后等{delay}秒 · {len(series)}次')
        p.addLine(y=0,pen=pg.mkPen('#89929c',width=1));p.setXRange(5,180,padding=0)
    w.addLabel('细线：各次记录；粗线：各次一秒中位值的组中位数。仅画松开5秒以后；参考不保证已充分恢复。',row=3,col=0,colspan=2,size='12pt')
    w.addLabel('5秒组实测平均2.319 N，60秒组2.565 N；曲线差异不能全部归因于残余。末次无恢复记录。',row=4,col=0,colspan=2,size='12pt')
    w.show()
    for _ in range(5):app.processEvents()
    exporter=pg.exporters.ImageExporter(w.scene());exporter.parameters()['width']=1900
    exporter.export(str(target));w.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'analysis_outputs'/('paired_history_'+datetime.now().strftime('%Y%m%d_%H%M%S')))
    args=parser.parse_args();analyze(args.session,args.output)

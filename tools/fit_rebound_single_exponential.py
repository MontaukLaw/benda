"""对已完成标定报告离线拟合每通道k、tau；按整轮留出检验，不修改原始记录。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from datetime import datetime
from pathlib import Path
import numpy as np


def model(t, D, T, k, tau):
    return k * D * (-np.expm1(-T/tau)) * np.exp(-np.asarray(t)/tau)


def fit_channel(curves, bounds=(.05, 100000.)):
    """固定tau时解析求非负k，再对log(tau)全网格检查并细化所有局部极小值。"""
    if not curves:
        raise ValueError('没有有效拟合曲线')
    def evaluate(log_tau):
        tau=float(np.exp(log_tau))
        ab=bb=0.
        for curve in curves:
            basis=(-np.expm1(-curve['T']/tau))*np.exp(-curve['t']/tau)
            y=curve['y']/curve['D']
            ab+=float(np.mean(basis*y));bb+=float(np.mean(basis*basis))
        k=max(0.,ab/bb) if bb>0 else 0.
        loss=0.
        for curve in curves:
            error=(model(curve['t'],curve['D'],curve['T'],k,tau)-curve['y'])/curve['D']
            loss+=float(np.mean(error**2))
        return loss/len(curves),k,tau
    grid=np.linspace(np.log(bounds[0]),np.log(bounds[1]),1201)
    losses=np.array([evaluate(x)[0] for x in grid])
    candidates=[evaluate(grid[0]),evaluate(grid[-1])]
    minima=np.flatnonzero((losses[1:-1]<=losses[:-2])&(losses[1:-1]<=losses[2:]))+1
    # k=0时目标可能完全平坦，不把任意tau当成可识别的系数。
    if np.ptp(losses)<1e-20:
        loss,k,tau=evaluate(grid[int(np.argmin(losses))])
        return dict(k=k,tau_seconds=tau,normalized_mse=loss,identifiable=False,boundary=False)
    ratio=(np.sqrt(5)-1)/2
    for index in minima:
        lo,hi=grid[index-1],grid[index+1]
        x1=hi-ratio*(hi-lo);x2=lo+ratio*(hi-lo)
        f1=evaluate(x1);f2=evaluate(x2)
        for _ in range(60):
            if f1[0]<f2[0]:
                hi=x2;x2=x1;f2=f1;x1=hi-ratio*(hi-lo);f1=evaluate(x1)
            else:
                lo=x1;x1=x2;f1=f2;x2=lo+ratio*(hi-lo);f2=evaluate(x2)
        candidates.append(evaluate((lo+hi)/2))
    loss,k,tau=min(candidates,key=lambda x:x[0])
    return dict(k=k,tau_seconds=tau,normalized_mse=loss,identifiable=k>0,
                boundary=bool(tau<=bounds[0]*1.001 or tau>=bounds[1]/1.001))


def load_curves(directory):
    report=json.loads((directory/'analysis.json').read_text(encoding='utf8'))
    if report.get('repeated_conditions'):
        raise ValueError('存在重复完整尝试，需明确选取后才能拟合')
    # 核对原始数据库与报告的校验值；不重新生成或覆盖已有报告。
    for source in report['source_files']:
        path=Path(source['path'])
        if not path.is_file():
            raise ValueError(f'原始数据库缺失，无法核对：{path}')
        wal=Path(str(path)+'-wal')
        if wal.exists() and wal.stat().st_size:
            raise ValueError('数据库仍有未合并WAL，需要先核对完整快照')
        with path.open('rb') as f:
            digest=hashlib.file_digest(f,'sha256').hexdigest()
        if digest!=source['sha256']:
            raise ValueError('原始文件SHA256与报告不一致')
    grouped={}
    with (directory/'recovery_1s.csv').open(encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            key=(row['session'],int(row['planned_trial_index']),int(row['channel']))
            grouped.setdefault(key,[]).append(row)
    trials={(x['session'],x['planned_trial_index']):x for x in report['trials']}
    curves=[];excluded=[]
    for x in report['channels']:
        trial=trials[(x['session'],x['planned_trial_index'])]
        if trial.get('analysis_role') not in ('fit','validation'):
            raise ValueError('报告未指定拟合/检验用途，请先明确划分')
        key=(x['session'],x['planned_trial_index'],x['channel'])
        rows=grouped.get(key,[])
        valid=[v for v in rows if v['covered']=='True' and v['residual_counts']]
        reason=None
        if not x['percent_response_valid']: reason='按压Raw增量无效或保持段缺口过大'
        elif not valid: reason='无有效恢复一秒窗'
        if reason:
            excluded.append(dict(session=key[0],trial=key[1],channel=key[2],reason=reason));continue
        curves.append(dict(session=key[0],trial=key[1],channel=key[2],role=trial['analysis_role'],
            repeat=trial['repeat'],T=trial['hold_actual_s'],hold_requested_s=trial['hold_requested_s'],
            D=x['hold_response_counts'],t=np.array([float(v['seconds']) for v in valid]),
            y=np.array([float(v['residual_counts']) for v in valid]),
            missing_bins=len(rows)-len(valid),metrics=x))
    return report,curves,excluded


def errors(curve,fit):
    predicted=model(curve['t'],curve['D'],curve['T'],fit['k'],fit['tau_seconds'])
    e=predicted-curve['y'];baseline=float(np.mean(curve['y']**2))
    result=dict(trial=curve['trial'],channel=curve['channel'],role=curve['role'],
        hold_requested_s=curve['hold_requested_s'],D=curve['D'],valid_bins=len(e),missing_bins=curve['missing_bins'],
        rmse_counts=float(np.sqrt(np.mean(e**2))),mae_counts=float(np.mean(abs(e))),
        maximum_abs_error_counts=float(np.max(abs(e))),rmse_percent_D=float(np.sqrt(np.mean(e**2))/curve['D']*100),
        zero_prediction_rmse_counts=float(np.sqrt(baseline)),
        relative_residual_rmse_percent=float(np.sqrt(np.mean(e**2)/baseline)*100) if baseline else None,
        checkpoints={})
    for sec in (1,5,10,30,60,120,300,600):
        observed=curve['metrics'].get(f'residual_{sec}s')
        pred=float(model(np.array([sec]),curve['D'],curve['T'],fit['k'],fit['tau_seconds'])[0])
        result['checkpoints'][sec]=dict(observed=observed,predicted=pred,error=None if observed is None else pred-observed)
    return result


def run(directory):
    directory=Path(directory).resolve()
    report,curves,excluded=load_curves(directory)
    result=dict(model='R=k*D*(1-exp(-T/tau))*exp(-t/tau)',time_zero='confirmed unload',
        input_report=str(directory),input_hashes={name:hashlib.sha256((directory/name).read_bytes()).hexdigest()
                                               for name in ('analysis.json','recovery_1s.csv')},
        source_files=report['source_files'],sessions=report['sessions'],trials=report['trials'],
        method='equal total weight per trial; MSE of (predicted-observed)/D over covered 1-second bins; analytic k>=0 and profiled tau',
        tau_search_bounds_s=[.05,100000],drift_subtraction=False,fast_slow_split=False,
        excluded_curves=excluded,channels={},errors=[])
    for c in (4,5,6,7):
        train=[x for x in curves if x['channel']==c and x['role']=='fit']
        validate=[x for x in curves if x['channel']==c and x['role']=='validation']
        if len(train)!=6 or len(validate)!=3:
            raise ValueError(f'CH{c}有效曲线数量不满足6次拟合、3次检验：{len(train)}/{len(validate)}')
        fit=fit_channel(train)
        # 只用训练轮次检查系数敏感性，留出轮次不参与搜索或参数选择。
        fit['training_repeat_fits']={str(rep):fit_channel([x for x in train if x['repeat']==rep]) for rep in (1,2)}
        fit['fit_trials']=[x['trial'] for x in train];fit['validation_trials']=[x['trial'] for x in validate]
        result['channels'][c]=fit
        for curve in train+validate: result['errors'].append(errors(curve,fit))
        print('FIT',c,json.dumps(fit,ensure_ascii=False))
        for e in result['errors']:
            if e['channel']==c and e['role']=='validation':
                print('VALIDATION',json.dumps(e,ensure_ascii=False))
    out=directory/('single_exp_fit_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir()
    (out/'fit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    with (out/'coefficients.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['channel','k','tau_seconds','status'])
        for c,fit in result['channels'].items(): writer.writerow([c,fit['k'],fit['tau_seconds'],'exploratory_not_approved_for_firmware'])
    with (out/'curve_errors.csv').open('w',encoding='utf-8-sig',newline='') as f:
        values=[{k:v for k,v in e.items() if k!='checkpoints'} for e in result['errors']]
        writer=csv.DictWriter(f,fieldnames=list(values[0]));writer.writeheader();writer.writerows(values)
    print('OUTPUT',out)
    return out,result,curves


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('analysis_directory',type=Path)
    run(parser.parse_args().analysis_directory)

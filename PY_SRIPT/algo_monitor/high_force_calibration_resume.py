"""校验标定检查点并发现可续跑记录；读取文件不会触发电机运动。"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re

from force_signal_pressure_calibration import high_force_calibration_targets
from high_force_calibration_quality import warmup_convergence, WARMUP_MAX, MAX_RETEST_PASSES, RETEST_REPEATS


def learning_fingerprint(learned):
    return hashlib.sha256(json.dumps(asdict(learned), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def validate_resume(document):
    """只接受顺序完整的已提交采样，不从报错文字或电机位置猜测完成度。"""
    if not isinstance(document, dict) or not isinstance(document.get('metadata'), dict):
        raise ValueError('不是有效的自动标定记录')
    metadata = document['metadata']
    targets = high_force_calibration_targets(metadata['point_count'])
    rounds = metadata['rounds']
    if type(rounds) is not int or not 1 <= rounds <= 100:
        raise ValueError('记录轮数无效')
    if document.get('complete'):
        raise ValueError('该标定已经完成')
    for name in ('warmup_records', 'records', 'retest_records', 'validation_records'):
        rows = document.get(name, [])
        if not isinstance(rows, list):
            raise ValueError('记录格式无效：'+name)
        seen = set()
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError('采样记录格式无效')
            if (row['floating_count'] != 4 or any(not isinstance(row[k], (int, float))
                    or not math.isfinite(row[k]) or row[k] <= 0 for k in ('signal_sum', 'final_force_n'))):
                raise ValueError('记录包含无效采样')
            target, cycle = row['target_force_n'], row['round']
            if type(cycle) is not int or cycle < 1 or target not in ((30,) if name == 'warmup_records' else targets):
                raise ValueError('记录的压力点或轮次无效')
            key = (cycle, target)
            if name == 'warmup_records':
                key = row['point']
            elif name in ('retest_records', 'validation_records'):
                batch = row['pass_index']
                if type(batch) is not int or not 1 <= batch <= MAX_RETEST_PASSES:
                    raise ValueError('复测批次无效')
                if name == 'retest_records' and cycle > RETEST_REPEATS:
                    raise ValueError('复测次数无效')
                key = (batch, cycle, target, row.get('validation_session', 0))
            if key in seen:
                raise ValueError('记录包含重复采样，不能自动续跑')
            seen.add(key)
    rows = document.get('records', [])
    plan = [(cycle, target) for cycle in range(1, rounds+1) for target in targets]
    if [(r['round'], r['target_force_n']) for r in rows] != plan[:len(rows)]:
        raise ValueError('正式采样顺序不完整或与计划不符')
    warmup = document.get('warmup_records', [])
    if len(warmup) > WARMUP_MAX or [r['point'] for r in warmup] != list(range(1, len(warmup)+1)):
        raise ValueError('热身记录顺序无效')
    converged = bool(warmup and warmup_convergence(warmup)['passed'])
    if rows and not converged:
        raise ValueError('缺少收敛热身记录，不能接续正式采样')
    if not converged and len(warmup) >= WARMUP_MAX:
        raise ValueError('热身已达到上限且未收敛，请检查材料后重新开始')
    passes = document.get('quality', {}).get('passes', [])
    if [p['pass_index'] for p in passes] != list(range(1, len(passes)+1)) or len(passes) > MAX_RETEST_PASSES:
        raise ValueError('复测批次顺序无效')
    if (passes or document.get('retest_records') or document.get('validation_records')) and len(rows) != len(plan):
        raise ValueError('正式采样未完成却存在复测记录')
    for row in document.get('retest_records', []):
        batch = row['pass_index']
        if batch > len(passes) or row['target_force_n'] not in passes[batch-1]['retest_targets']:
            raise ValueError('复测记录与保存计划不符')
    for row in document.get('validation_records', []):
        if row['pass_index'] > len(passes) or row['round'] != row['pass_index']:
            raise ValueError('验收记录与保存批次不符')
    if passes and len(passes) == MAX_RETEST_PASSES:
        last = passes[-1]
        # 完成的质量失败是一次终局，不用续跑绕过三批上限。
        if (last.get('validation', {}).get('passed') is False
                or last.get('internal', {}).get('passed') is False or last.get('fit_error')):
            raise ValueError('已完成三批复测且验收失败，请检查原因后重新标定')
    if not converged:
        next_step = f'热身第{len(warmup)+1}次（30 N）'
    elif len(rows) < len(plan):
        cycle, target = plan[len(rows)]
        next_step = f'第{cycle}轮 {target:g} N'
    else:
        next_step = '接续重点复测；独立验收重新采集完整一批'
    return f'{len(targets)}点×{rounds}轮，已完成{len(rows)}/{len(plan)}点；下一步：{next_step}'


def check_resume_context(document, profile, release, learned, current_settings):
    metadata = document['metadata']
    if metadata['namespace'] != profile.namespace or metadata['profile'] != profile.metadata():
        raise ValueError('电机型号或步数配置已变化，不能拼接旧标定')
    if metadata['release_position'] != release:
        raise ValueError('释放点已变化，不能继续旧标定；请使用原释放参考或重新标定')
    saved = metadata.get('learning_fingerprint')
    if saved and saved != learning_fingerprint(learned):
        raise ValueError('学习表已变化，不能拼接旧标定，请重新开始')
    for key in ('force_limit_n', 'max_current_ma', 'max_travel'):
        if metadata['motion_settings'][key] > current_settings[key]:
            raise ValueError('当前保护限制已收紧，续跑不能恢复更高的旧限制：'+key)


def read_resume(path):
    path = Path(path)
    raw = path.read_bytes()
    document = json.loads(raw.decode('utf-8'))
    summary = validate_resume(document)
    match = re.search(r'high_force_\d{8}_\d{6}_\d{6}', path.name)
    root = document['metadata'].get('resume_root_id') or (match.group() if match else path.stem)
    return dict(path=path, document=deepcopy(document), summary=summary, root_id=root,
                sha256=hashlib.sha256(raw).hexdigest())


def discover_resume(directory, namespace):
    """主文件、旧临时文件及新持久备份取同一任务最新版本；完成后不复活旧检查点。"""
    latest = {}
    for path in Path(directory).glob('*high_force_*'):
        if not path.name.endswith(('.json', '.json.tmp')):
            continue
        try:
            doc = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(doc, dict) or not isinstance(doc.get('metadata'), dict):
                continue
            meta = doc['metadata']
            if meta.get('namespace') != namespace:
                continue
            match = re.search(r'high_force_\d{8}_\d{6}_\d{6}', path.name)
            root = meta.get('resume_root_id') or (match.group() if match else path.stem)
            rank = (int(meta.get('resume_generation', 0)), int(doc.get('record_revision', 0)), path.stat().st_mtime_ns)
            if root not in latest or rank > latest[root][0]:
                latest[root] = (rank, path, doc)
        except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
            continue
    for _, path, doc in sorted(latest.values(), key=lambda item: item[0][2], reverse=True):
        if doc.get('complete'):
            continue
        try:
            return read_resume(path)
        except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
            continue
    return None

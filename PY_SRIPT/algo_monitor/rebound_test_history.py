"""轻量读取测试目录和已完成的报告索引，不加载原始采样数据。"""
from __future__ import annotations

import json
from pathlib import Path


def read_manifest(directory):
    manifest = json.loads((Path(directory) / 'manifest.json').read_text(encoding='utf-8'))
    if not isinstance(manifest, dict) or not isinstance(manifest.get('config', {}), dict):
        raise ValueError('测试说明文件格式不正确')
    return manifest


def test_label(directory, manifest):
    status = {'completed': '已完成', 'stopped': '已停止', 'running': '记录中',
              'failed': '未完成', 'start_rejected': '未启动'}.get(manifest.get('status'), '未完成')
    config = manifest.get('config', {})
    values = lambda key: '/'.join(str(x) for x in config.get(key, [])) or '未记录'
    created = str(manifest.get('created_at', Path(directory).name)).replace('T', ' ')[:19]
    protocol = '按压与恢复' if config.get('protocol') == 'raw_rebound_v1' else '旧测试方案'
    suffix = ' · 续测记录' if manifest.get('resumed_from') else ''
    return (f"{created} · {status} · {protocol} · {values('high_forces_n')} N · "
            f"保持 {values('high_hold_seconds')} 秒 · 本段完成 {manifest.get('completed_trials', 0)} 次{suffix}")


def saved_tests(root, extra=()):
    paths = {p.parent.resolve() for p in Path(root).glob('creep_*/manifest.json')}
    paths.update(Path(p).resolve() for p in extra)
    results = []
    for path in paths:
        try:
            manifest = read_manifest(path)
            if manifest.get('status') != 'start_rejected':
                results.append((path, manifest, test_label(path, manifest)))
        except (OSError, ValueError, TypeError):
            continue
    return sorted(results, key=lambda r: str(r[1].get('created_at', r[0].name)), reverse=True)


def saved_reports(directory):
    """新报告仅在完成标记落盘后显示；兼容已经保存的 analysis_v1 报告。"""
    directory = Path(directory)
    reports = []
    for marker in sorted((directory / 'reports').glob('analysis_*/complete.json'), reverse=True):
        try:
            info = json.loads(marker.read_text(encoding='utf-8'))
            if (marker.parent / 'report.html').is_file() and (marker.parent / 'analysis.json').is_file():
                label = (f"{info['generated_at']} · 恢复与残差 · 计算版本 {info['analysis_version']}")
                reports.append((marker.parent, label))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    legacy = directory / 'analysis_v1'
    if (legacy / 'report.html').is_file():
        reports.append((legacy, '以前保存的报告（analysis_v1）'))
    return reports

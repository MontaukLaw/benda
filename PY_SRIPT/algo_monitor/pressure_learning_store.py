"""保存学习资格并读取旧学习归档；加载只返回待核验数据，不授予运动权限。"""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import tempfile

from pressure_feedback_control import LearnedPressureMap


def identity(profile):
    return dict(motor_model=profile.model, namespace=profile.namespace,
                steps_per_turn=profile.steps_per_turn)


def decode_table(document, profile):
    if not isinstance(document, dict) or any(document.get(k) != v for k, v in identity(profile).items()):
        raise ValueError("学习表的电机型号、地址或每圈步数不匹配")
    release = document.get('release_position')
    if isinstance(release, bool) or not isinstance(release, int) or not -(2**31) <= release < 2**31:
        raise ValueError("学习表释放坐标无效")
    points = document.get('points')
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= 128:
        raise ValueError("学习表缺少完整加载点")
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("学习点格式无效")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
            raise ValueError("学习点不是有限数值")
        if not 0 < point[0] <= 35 or not 0 < point[1] <= profile.steps_per_turn*12:
            raise ValueError("学习点超过压力或行程范围")
    contact = {k: document.get(k) for k in ('contact_zero_travel', 'contact_travel', 'contact_force_n')}
    contact['contact_reference_n'] = document.get('contact_reference_n', 0.0)
    for v in contact.values():
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)):
            raise ValueError("学习接触点格式无效")
    return LearnedPressureMap(**identity(profile), release_position=release,
                              points=tuple(tuple(p) for p in points), **contact)


def save_learning(path, profile, table=None, reason=''):
    """原子保存有效表或失效标记，清除后不再从历史归档复活旧表。"""
    path = Path(path)
    document = dict(schema_version=1, profile=identity(profile),
                    table=asdict(table) if table is not None else None, reason=str(reason))
    if table is not None:
        decode_table(document['table'], profile)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=path.stem+'_', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def load_learning(path, archive_dir, profile):
    """优先读取专用存档；首次升级才迁移本机型最新完整归档，不回退到更旧表。"""
    path = Path(path)
    if path.exists():
        document = json.loads(path.read_text(encoding='utf-8-sig'))
        if (not isinstance(document, dict) or document.get('schema_version') != 1
                or document.get('profile') != identity(profile) or 'table' not in document):
            raise ValueError("学习存档格式、版本或电机配置不匹配")
        return (decode_table(document['table'], profile) if document['table'] is not None else None,
                str(document.get('reason', '')))
    for archive in sorted(Path(archive_dir).glob('feedback_*.learning.json'), reverse=True):
        document = json.loads(archive.read_text(encoding='utf-8-sig'))
        if not isinstance(document, dict):
            raise ValueError("历史学习文件格式错误")
        if any(document.get(k) != v for k, v in identity(profile).items()):
            continue
        table = decode_table(document, profile)
        version = document.get('schema_version')
        if (version not in (1, 2, 3) or len(table.points) not in (8, 11, 12)
                or table.points[-1][0] < 27 or (version in (2, 3) and table.contact_travel is None)):
            raise ValueError("最新历史学习表未包含完整30 N学习结果")
        save_learning(path, profile, table)
        return table, ''
    return None, ''

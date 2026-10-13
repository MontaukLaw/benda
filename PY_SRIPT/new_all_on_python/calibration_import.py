"""读取algo_monitor原始标定记录，复用校验与计算，不导入其GUI或改写来源文件。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path

from .config import RAW_GAIN_LIMITS
from .protocol_bridge import PY_ROOT, _load_existing

CALIBRATION_DIR = PY_ROOT / 'algo_monitor' / 'raw_gain_calibrations'
_raw_gain = _load_existing(PY_ROOT / 'algo_monitor' / 'raw_gain.py')
_calibration = _load_existing(PY_ROOT / 'algo_monitor' / 'raw_gain_calibration.py', {'raw_gain': _raw_gain})


def read_calibration(path):
    """从原始采集重新计算20路总增益；旧基线、运行状态和缓存结果不作为输入。"""
    path = Path(path).resolve()
    if path.stat().st_size > 20 * 1024 * 1024:
        raise ValueError('标定记录超过20 MiB')
    data = path.read_bytes()
    def reject_constant(value):
        raise ValueError(f'标定记录含非有限数值：{value}')
    report = json.loads(data.decode('utf-8-sig'), parse_constant=reject_constant)
    model = _calibration.RawGainCalibration.from_report(report)
    # 数值质量与旧连接状态分开：应用前由界面明确确认仍为同一阵列和三参数。
    model.invalid_reason = ''
    result = model.results()
    if not result['ready']:
        qualified = sum(c['qualified'] for c in result['channels'])
        raise ValueError(result['error'] or f'只有{qualified}/20通道标定合格，请在algo_monitor完成或重测后再读取')
    for channel, gain in enumerate(result['gains']):
        if not 0 < gain <= RAW_GAIN_LIMITS[1]:
            raise ValueError(f'CH{channel}增益{gain:g}超出本应用允许范围0～{RAW_GAIN_LIMITS[1]:g}；请调整标定目标，不会自动截断')
    context = report.get('context')
    if context is not None and not isinstance(context, dict):
        raise ValueError('标定设备环境格式无效')
    return dict(schema_version=1, path=str(path), sha256=hashlib.sha256(data).hexdigest(),
                imported_at=datetime.now().astimezone().isoformat(),
                mass_g=model.settings.mass_g, target=result['target'], repeats=model.settings.repeats,
                gains=list(result['gains']), source_saved_at=report.get('saved_at'),
                source_context=context, source_invalid_reason=report.get('invalid_reason', ''),
                channels=result['channels'])


def latest_calibration_path(directory=CALIBRATION_DIR):
    files = list(Path(directory).glob('raw_gain_*.json'))
    if not files:
        raise ValueError('未找到标定记录，请先在algo_monitor完成标定并保存')
    return max(files, key=lambda path: path.stat().st_mtime_ns)

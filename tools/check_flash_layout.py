"""构建前校验应用布局与写入入口，防止触及DFU主记录、备份及其后区域。"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def layout_value(name):
    text = (ROOT / 'app_flash_layout.h').read_text(encoding='utf-8')
    match = re.search(r'#define\s+' + re.escape(name) + r'\s+\((0x[0-9A-Fa-f]+|[0-9]+)u?\)', text)
    if not match:
        raise ValueError(f'Missing literal Flash layout constant: {name}')
    return int(match.group(1), 0)


def check_layout():
    start = layout_value('APP_FLASH_PARAMETER_START')
    protected = layout_value('APP_FLASH_PROTECTED_START')
    row = layout_value('APP_FLASH_ROW_BYTES')
    force = layout_value('APP_FLASH_FORCE_ADDRESS')
    size = layout_value('APP_FLASH_FORCE_REGION_BYTES')
    affine = layout_value('APP_FLASH_AFFINE_ADDRESS')
    assert protected == 0x1FD00 and row == 256
    low_force = layout_value('APP_FLASH_LOW_FORCE_ADDRESS')
    assert start == low_force and low_force + row <= force
    assert force + 2 * size <= affine and affine + row <= protected
    assert all(value % row == 0 for value in (start, protected, force, size, affine))
    linker = (ROOT / 'linker/dfu_cm0p_app1.ld').read_text(encoding='utf-8')
    match = re.search(r'FLASH_app1\s*\(rx\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+),\s*LENGTH\s*=\s*(0x[0-9A-Fa-f]+)', linker)
    assert match and int(match[1], 16) == 0x3600
    assert int(match[1], 16) + int(match[2], 16) == start, 'Linker overlaps parameter region'
    # 上位机可独立运行，因此保留数值常量；构建时防止两端边界漂移。
    host_config = (ROOT / 'PY_SRIPT/algo_monitor/config.py').read_text(encoding='utf-8')
    for name, expected in (('DFU_APPLICATION_START', 0x3600), ('DFU_APPLICATION_END', start),
                           ('DFU_PROTECTED_START', protected), ('DFU_FLASH_ROW_BYTES', row)):
        value = re.search(r'^' + name + r'\s*=\s*(0x[0-9a-fA-F]+|[0-9]+)', host_config, re.MULTILINE)
        assert value and int(value[1], 0) == expected, f'Host Flash boundary mismatch: {name}'
    # 应用根目录中的所有手写模块都只能通过统一入口擦写。
    for path in ROOT.glob('*.c'):
        if path.name == 'app_flash.c':
            continue
        source = path.read_text(encoding='utf-8')
        assert not re.search(r'\bCy_Flash_(?:WriteRow|EraseRow|ProgramRow|StartWrite|StartErase)\s*\(', source), f'Direct Flash write bypass: {path.name}'
    return start, protected


if __name__ == '__main__':
    start, protected = check_layout()
    print(f'Flash layout OK: application ends at 0x{start:X}; all writes stay below 0x{protected:X}')

"""独立HTML报告内的本地筛选、数值排序和CSV导出；不依赖网络资源。"""
import html
import json

SORT_FIELDS = (
    ('planned_trial_index', '原项号'), ('target_n', '目标压力'),
    ('hold_requested_s', '计划保持时长'), ('channel', '通道'),
    ('hold_actual_s', '实际保持时长'), ('hold_median_n', '实测保持压力'),
    ('hold_raw_median', '按压Raw中位值'), ('hold_response_counts', '按压增量'),
    ('hold_raw_window_s', '取值窗口'),
    ('residual_1s', '1s残差'), ('residual_5s', '5s残差'), ('residual_30s', '30s残差'),
    ('residual_1s_percent_of_hold_response', '1s增量占比'),
    ('residual_5s_percent_of_hold_response', '5s增量占比'),
    ('residual_30s_percent_of_hold_response', '30s增量占比'),
    ('repeat', '轮次'),
    ('pre_shift_from_session_start_counts', '按压前相对本段起点偏移'),
    ('residual_10s_from_session_start_counts', '10秒相对本段起点偏移'),
    ('maximum_abs_residual_10_to_60s_counts', '10～60秒最大绝对残差'),
    ('maximum_abs_residual_10_to_60s_percent_of_hold_response', '10～60秒最大绝对残差占比'),
    ('residual_10s', '10s残差'), ('residual_60s', '60s残差'),
    ('residual_300s', '300s残差'), ('tail_residual_counts', '末尾残差'),
    ('residual_600s', '600s残差'),
    ('residual_10s_percent_of_hold_response', '10s增量占比'),
    ('residual_60s_percent_of_hold_response', '60s增量占比'),
    ('residual_300s_percent_of_hold_response', '300s增量占比'),
    ('residual_600s_percent_of_hold_response', '600s增量占比'),
    ('tail_residual_counts_percent_of_hold_response', '末尾增量占比'),
    ('confirmed_idle_1s', 'IDLE确认时间'),
    ('confirmed_raw_2counts_5s', '±2返回确认时间'),
    ('confirmed_raw_5counts_5s', '±5返回确认时间'),
    ('left_band_after_2counts', '±2返回后再离带'), ('tail_rising', '末尾上漂'),
)

CSS = """
.filters{background:#f3f7fb;border:1px solid #bbc8d5;border-radius:8px;padding:14px;margin:16px 0}
.filter-groups,.filter-actions{display:flex;flex-wrap:wrap;gap:12px;align-items:center}
.filters fieldset{border:1px solid #ccd6e0;border-radius:5px;margin:0;padding:8px 12px}
.filters legend{font-weight:600}.filters label{display:inline-flex;gap:5px;align-items:center;margin-right:10px}
.filters input{width:17px;height:17px}.filters button,.filters select{font:inherit;padding:6px 10px}
.filters button{cursor:pointer}.filter-actions{margin-top:12px}#filter-count{font-weight:600}
.filters p{margin-bottom:0}#filter-empty{padding:12px;color:#914000}
.sort-header{font:inherit;font-weight:600;border:0;background:none;color:inherit;cursor:pointer;padding:0}
#recovery-table th[aria-sort=ascending] .sort-header::after{content:' ▲'}
#recovery-table th[aria-sort=descending] .sort-header::after{content:' ▼'}
.table-scroll:has(#recovery-table){max-height:75vh;overflow:auto}
#recovery-table thead th{position:sticky;top:0;z-index:2}
#recovery-table tbody tr:nth-child(even){background:#f7fafc}
"""


def filter_controls(rows):
    groups=[]
    for key,title,unit in (('target_n','目标压力',' N'),('hold_requested_s','计划保持时长',' s'),('channel','通道','')):
        values=sorted({r[key] for r in rows})
        choices=''.join(
            f'<label><input type="checkbox" data-filter="{key}" value="{v:g}" checked>'
            f'{"CH" if key=="channel" else ""}{v:g}{unit}</label>' for v in values)
        groups.append(f'<fieldset><legend>{title}（可多选）</legend>{choices}'
                      f'<button type="button" data-group="{key}" data-select="all">全选</button> '
                      f'<button type="button" data-group="{key}" data-select="none">清空</button></fieldset>')
    options=''.join(f'<option value="{key}">{html.escape(title)}</option>' for key,title in SORT_FIELDS)
    return ('<section class="filters" id="recovery-filters"><div class="filter-groups">'+''.join(groups)+
        '</div><div class="filter-actions"><label>排序 <select id="sort-field">'+options+'</select></label>'
        '<label>方向 <select id="sort-direction"><option value="asc">从小到大</option>'
        '<option value="desc">从大到小</option></select></label>'
        '<button type="button" id="reset-filters">重置筛选与排序</button>'
        '<button type="button" id="export-filtered">导出筛选结果 CSV</button>'
        '<span id="filter-count" role="status" aria-live="polite"></span></div>'
        '<p>同类选项取并集，不同筛选条件同时满足。比较10 N不同保持时间：目标仅保留10 N，保持时长全选，按计划保持时长排序。'
        '点击数值表头也可排序；占比表头按“增量%”排序。每次重复独立保留，不自动平均。筛选仅作用于下方四通道恢复表。</p>'
        '</section><p id="filter-empty" role="status" hidden>没有符合条件的数据，请调整筛选。</p>'
        '<noscript><p>浏览器未启用JavaScript，以下仍显示全部结果，筛选与导出不可用。</p></noscript>')


def interaction_script(rows):
    # 转义小于号，避免会话名等文本提前结束script标签。
    data=json.dumps(rows,ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    return '<script type="application/json" id="recovery-data">'+data+'</script><script>'+SCRIPT+'</script>'


SCRIPT = r"""
(() => {
  'use strict';
  const data = JSON.parse(document.getElementById('recovery-data').textContent);
  const table = document.getElementById('recovery-table');
  const tbody = table.tBodies[0];
  const rowNodes = Array.from(tbody.rows);
  const panel = document.getElementById('recovery-filters');
  const sortField = document.getElementById('sort-field');
  const sortDirection = document.getElementById('sort-direction');
  const exportButton = document.getElementById('export-filtered');
  let visible = [];
  const fields = ['target_n', 'hold_requested_s', 'channel'];
  const isMissing = value => value === null || value === undefined;
  function compare(a, b, key, direction = 1) {
    const x = a[key], y = b[key];
    if (isMissing(x) || isMissing(y)) return isMissing(x) === isMissing(y) ? 0 : isMissing(x) ? 1 : -1;
    return (Number(x) - Number(y)) * direction;
  }
  function update() {
    const selected = Object.fromEntries(fields.map(key => [key, new Set(
      Array.from(panel.querySelectorAll(`input[data-filter="${key}"]:checked`)).map(el => Number(el.value)))]));
    visible = data.map((row, id) => ({row, id})).filter(({row}) => fields.every(key => selected[key].has(row[key])));
    const direction = sortDirection.value === 'asc' ? 1 : -1;
    visible.sort((a, b) => compare(a.row, b.row, sortField.value, direction)
      || compare(a.row, b.row, 'hold_requested_s') || compare(a.row, b.row, 'channel')
      || compare(a.row, b.row, 'planned_trial_index') || a.id - b.id);
    rowNodes.forEach(node => { node.hidden = true; });
    const fragment = document.createDocumentFragment();
    visible.forEach(({id}) => { rowNodes[id].hidden = false; fragment.appendChild(rowNodes[id]); });
    tbody.appendChild(fragment);
    const count = new Set(visible.map(({row}) => row.planned_trial_index)).size;
    document.getElementById('filter-count').textContent = `显示 ${visible.length} / ${data.length} 行，${count} 个原项目`;
    document.getElementById('filter-empty').hidden = visible.length > 0;
    exportButton.disabled = visible.length === 0;
    table.querySelectorAll('th[data-sort]').forEach(th => {
      th.setAttribute('aria-sort', th.dataset.sort === sortField.value ? (direction === 1 ? 'ascending' : 'descending') : 'none');
    });
  }
  panel.addEventListener('change', update);
  panel.querySelectorAll('button[data-group]').forEach(button => button.addEventListener('click', () => {
    panel.querySelectorAll(`input[data-filter="${button.dataset.group}"]`).forEach(input => { input.checked = button.dataset.select === 'all'; });
    update();
  }));
  document.getElementById('reset-filters').addEventListener('click', () => {
    panel.querySelectorAll('input[data-filter]').forEach(input => { input.checked = true; });
    sortField.value = 'planned_trial_index'; sortDirection.value = 'asc'; update();
  });
  table.querySelectorAll('th[data-sort] button').forEach(button => button.addEventListener('click', () => {
    const key = button.parentElement.dataset.sort;
    sortDirection.value = sortField.value === key && sortDirection.value === 'asc' ? 'desc' : 'asc';
    sortField.value = key; update();
  }));
  function csvCell(value) {
    if (isMissing(value)) return '""';
    let text = String(value);
    // 数值负号保持原值；外部文本以公式符号开头时按文本导出。
    if (typeof value === 'string' && /^[=+@\-\t\r\n]/.test(text)) text = "'" + text;
    return '"' + text.replace(/"/g, '""') + '"';
  }
  exportButton.addEventListener('click', () => {
    if (!visible.length) return;
    const keys = Object.keys(data[0]);
    const text = '\uFEFF' + [keys.map(csvCell).join(','), ...visible.map(({row}) => keys.map(key => csvCell(row[key])).join(','))].join('\r\n');
    const url = URL.createObjectURL(new Blob([text], {type: 'text/csv;charset=utf-8'}));
    const link = document.createElement('a'); link.href = url; link.download = 'rebound_filtered.csv';
    document.body.appendChild(link); link.click(); link.remove(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  update();
})();
"""

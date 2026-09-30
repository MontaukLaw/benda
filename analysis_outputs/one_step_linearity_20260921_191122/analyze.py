"""Analyze one completed loading scan; never operate hardware or modify learning data."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
DEPS = ROOT / 'tmp/floating_plot_deps'
if DEPS.exists():
    sys.path.insert(0, str(DEPS))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent
SOURCE = ROOT / 'PY_SRIPT/algo_monitor/one_step_learning/one_step_learning_20260921_191122_616083.json'
raw = SOURCE.read_bytes()
doc = json.loads(raw)
rows = doc['nodes']
assert doc['kind'] == 'one_step_integer_pressure_learning'
assert doc['complete'] and doc['release_position_confirmed'] and doc['pressure_unloaded']
assert [r['target_force_n'] for r in rows] == list(range(31))
x = np.array([r['travel_steps'] for r in rows], dtype=float)
y = np.array([r['measured_force_n'] for r in rows], dtype=float)
assert np.all(np.isfinite(x)) and np.all(np.isfinite(y))
assert np.all(np.diff(x) > 0) and np.all(np.diff(y) > 0)
assert all(r['travel_steps'] == doc['origin_position_steps'] - r['position_steps'] for r in rows)
assert max(abs(r['error_n']) for r in rows) <= 0.05000001

def fit(lo, hi):
    xx, yy = x[lo:hi+1], y[lo:hi+1]
    coefficients = np.polyfit(xx, yy, 1)
    residual = yy - np.polyval(coefficients, xx)
    return dict(range_n=[lo, hi], slope_n_per_step=float(coefficients[0]),
                intercept_n=float(coefficients[1]),
                r_squared=float(1 - np.sum(residual**2) / np.sum((yy-yy.mean())**2)),
                rmse_n=float(np.sqrt(np.mean(residual**2))),
                max_abs_residual_n=float(np.max(np.abs(residual))),
                endpoint_steps_per_n=float((xx[-1]-xx[0])/(yy[-1]-yy[0])))

fits = [fit(lo, hi) for lo, hi in [(1, 30), (1, 10), (10, 20), (20, 30)]]
overall = fits[0]
pred = overall['slope_n_per_step'] * x[1:] + overall['intercept_n']
residual = y[1:] - pred
summary = dict(source=str(SOURCE.relative_to(ROOT)), source_sha256=hashlib.sha256(raw).hexdigest(),
               created_at=doc['created_at'], node_count=len(rows), fits=fits,
               max_learning_error_n=float(np.max(np.abs(y-np.arange(31)))),
               loaded_span_n=float(y[-1]-y[1]),
               max_residual_percent_loaded_span=float(max(abs(residual))/(y[-1]-y[1])*100))
(OUT / 'metrics.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')

font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family': 'Microsoft YaHei', 'font.size': 11,
                     'axes.unicode_minus': False, 'axes.spines.top': False,
                     'axes.spines.right': False, 'axes.titleweight': 'bold',
                     'savefig.facecolor': '#f7f9fc', 'figure.facecolor': '#f7f9fc'})
blue, orange = '#1677b8', '#dc7624'
fig, axs = plt.subplots(2, 2, figsize=(14, 9.7))
fig.subplots_adjust(left=.075, right=.97, top=.86, bottom=.12, hspace=.40, wspace=.23)
fig.suptitle('压力与电机位置：整体弯曲，高压力段更接近直线', fontsize=21, x=.075, ha='left', y=.97)
fig.text(.075, .915, 'One Step 学习 · 2026-09-21 19:05–19:11 · 31 个完整节点 · 使用压力计实测值', fontsize=12, color='#4b5c70')
for ax in axs.flat:
    ax.set_facecolor('white')
    ax.grid(alpha=.18)
    ax.set_axisbelow(True)

ax = axs[0, 0]
ax.plot(x[:2], y[:2], '--', color='#a5adb9', lw=2)
ax.plot(x[1:], y[1:], 'o-', color=blue, ms=4, lw=2)
ax.scatter([0], [0], color='#697687', zorder=3)
ax.annotate('起点 0 N\n到 1 N 共 431 step\n接触点未单独记录', xy=(0, 0), xytext=(70, 13),
            arrowprops=dict(arrowstyle='->', color='#697687'), fontsize=10, color='#4b5c70')
ax.annotate('30.000 N / 1449 step', xy=(1449, 30), xytext=(700, 29), fontsize=10)
ax.set(title='① 完整加载曲线（0～30 N）', xlabel='从起点向下的行程 / step', ylabel='压力计实测压力 / N', ylim=(-1, 33))

ax = axs[0, 1]
ax.plot(x[1:], y[1:], 'o-', color=blue, ms=4, lw=2, label='学习实测')
ax.plot(x[1:], pred, '--', color=orange, lw=2, label='1～30 N 最小二乘直线')
ax.set(title=f'② 加载段直线拟合：R² = {overall["r_squared"]:.4f}', xlabel='从起点向下的行程 / step', ylabel='压力计实测压力 / N')
ax.legend(loc='upper left', frameon=False, fontsize=10)

ax = axs[1, 0]
ax.axhline(0, color='#66758a', lw=1)
ax.fill_between(y[1:], residual, 0, alpha=.12, color=orange)
ax.plot(y[1:], residual, 'o-', color=orange, ms=4)
ax.set(title=f'③ 直线拟合偏差：最大 {max(abs(residual)):.2f} N', xlabel='压力计实测压力 / N', ylabel='实测压力 − 直线预测 / N', ylim=(-2.1, 4.1))
ax.text(.5, .92, f'均方根误差 RMSE = {overall["rmse_n"]:.2f} N', transform=ax.transAxes, ha='center', fontsize=11)

ax = axs[1, 1]
steps_per_n = np.diff(x[1:]) / np.diff(y[1:])
mid = (y[2:] + y[1:-1]) / 2
ax.plot(mid, steps_per_n, 'o-', color=blue, ms=4, lw=1.5, label='相邻学习节点')
for item, color in zip(fits[1:], ['#bb6a25', '#319670', '#8159aa']):
    lo, hi = item['range_n']
    value = item['endpoint_steps_per_n']
    ax.hlines(value, lo, hi, color=color, lw=3, label=f'{lo}～{hi} N 平均：{value:.1f} step/N')
ax.set(title='④ 压力越大，每增加 1 N 所需步数越少', xlabel='相邻节点的实测压力中点 / N', ylabel='每增加 1 N 的行程 / (step/N)', ylim=(0, 85))
ax.legend(frameon=False, fontsize=9, loc='upper right')

fig.text(.075, .055, '坐标说明：起点绝对位置为 2 step；向下行程 = 2 − 电机绝对位置。直线拟合排除 0 N 起点。', fontsize=10, color='#4b5c70')
fig.text(.075, .029, '这是一次小步慢速加载曲线；图中拟合偏差不等于学习误差，也不能代表快速到位后的误差或重复性。', fontsize=10, color='#4b5c70')
fig.savefig(OUT / 'pressure_position_linearity.png', dpi=160)
fig.savefig(OUT / 'pressure_position_linearity.svg')
plt.close(fig)

lines = ['# One Step 压力—位置线性度分析', '',
         f'- 数据文件：`{summary["source"]}`',
         f'- SHA256：`{summary["source_sha256"]}`',
         f'- 保存时间：{doc["created_at"]}；31 个节点完整，已回起点并卸载。',
         '- 学习范围为目标 0～30 N，末点实测 30.000 N；节点目标误差最大 0.050 N。',
         '- 起点绝对位置 2 step；30 N 绝对位置 −1447 step，下压行程 1449 step。', '',
         '## 结论', '',
         '压力与行程严格单调，但全范围存在明显曲率，不能只用一个固定的 step/N 系数。',
         f'排除 0 N 起点后，1～30 N 拟合 R²={overall["r_squared"]:.6f}，RMSE={overall["rmse_n"]:.3f} N，最大绝对偏差={overall["max_abs_residual_n"]:.3f} N（实测加载跨度的 {summary["max_residual_percent_loaded_span"]:.2f}%）。',
         'R² 高不代表误差足以满足压力控制要求。拟合偏差是直线模型误差，不是本次学习节点的目标误差。', '',
         '| 目标区间 | R² | RMSE / N | 最大直线偏差 / N | 端点平均 step/N |',
         '|---|---:|---:|---:|---:|']
for item in fits:
    lo, hi = item['range_n']
    lines.append(f'| {lo}～{hi} N | {item["r_squared"]:.6f} | {item["rmse_n"]:.3f} | {item["max_abs_residual_n"]:.3f} | {item["endpoint_steps_per_n"]:.2f} |')
lines += ['', '拟合使用实测压力为因变量、下压行程为自变量，包含截距，不强制过零。区间按目标压力选取，10 N 和 20 N 是共享边界。',
          f'全段方程：P = {overall["slope_n_per_step"]:.8f} × 行程 {overall["intercept_n"]:+.8f}。仅用于评估线性度，不建议作为控制公式。', '',
          '## 对下一步控制的含义', '',
          '- 建议保留整数节点查表；中间压力在相邻的实测压力—位置节点间插值，再使用压力反馈确认和微调。',
          '- 0～1 N 包含起点到接触后的整个过程，本表没有接触点记录，不能断言 431 step 全是空行程，也不能直接把该段用于微力线性插值。',
          '- 单次慢速逐级加载不能验证回零后快速直达的重复性、动态超调或加载历史影响；后续需要用实际快速到位试验确认。', '',
          '## 全部学习节点', '', '| 目标 / N | 实测 / N | 绝对位置 / step | 下压行程 / step |', '|---:|---:|---:|---:|']
for row in rows:
    lines.append(f'| {row["target_force_n"]} | {row["measured_force_n"]:.3f} | {row["position_steps"]} | {row["travel_steps"]} |')
(OUT / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
print(OUT / 'pressure_position_linearity.png')

"""验证真实Raw批次、连续权重、物理坐标日志、设置保存和测试页参考等待。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from tools.test_touchpad_host_ui import SCRIPT as HOST_SCRIPT
from tools.test_slide_test_ui import SCRIPT as SLIDE_SCRIPT

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=HOST_SCRIPT.split("with patch.object(QtGui.QCursor, 'setPos')")[0]+r'''
panel.source.setCurrentIndex(panel.source.findData('host_continuous'))
panel.position_reference_seconds.setValue(.6)
panel.position_noise_floor.setValue(25)
panel.position_decay_ms.setValue(120)
panel.position_restore_ms.setValue(30)
panel.position_decay_percent.setValue(70)
assert panel.position_decay_enabled.isChecked()
assert not panel.continuous_group.isHidden() and panel.reference_group.isHidden()
with patch.object(QtGui.QCursor,'setPos') as move:
    phase(400)
    assert panel.tracker.position_reference is not None
    assert panel.tracker.position_reference[5] == 20000.
    assert '已就绪' in panel.position_reference_status.text()
    reference=panel.tracker.position_reference
    phase(100,2.)
    assert panel.tracker.contact
    assert panel.tracker.position_reference == reference
    old=tuple(panel.canvas.cursor)
    for i in range(100):
        append(2.+.4*(i+1)/100)
        if i%10==9: flush()
    assert panel.canvas.cursor[0]>old[0]+60
    for row in logs:
        assert row['source']=='host_continuous_centroid'
        assert row['algorithm_input']=='host_input_val_minus_position_reference'
        h=derived[row['source_sample_sequence']]
        assert tuple(row['host_input_val'])==tuple(h['input_val'])
        assert tuple(row['host_baseline'])==tuple(h['baseline'])
        assert row['sensor_pitch_mm']==8. and row['sensor_diameter_mm']==5.
        if row['position_weights'] is not None:
            assert len(row['position_factors'])==20
            assert all(math.isclose(w,u*f,abs_tol=1e-8) for w,u,f in zip(row['position_weights'],row['position_unattenuated_weights'],row['position_factors']))
        if row['raw_xy']:
            assert math.isclose(row['raw_xy_mm'][0],(row['raw_xy'][0]-.5)*8.)
            assert math.isclose(row['centroid_dx_mm'],row['centroid_dx']*8.)
    assert not move.called
    context=window._slide_test_context()
    assert context['position_reference_ready']
    assert context['sensor_geometry']['center_span_mm']==[32.,24.]
    assert tuple(context['channel_grid'][0])==(0,4,8,12,16)
    # 状态机代数变化必须立即让连续位置参考失效。
    window._reset_host_state('test reset')
    assert panel.tracker.position_reference is None and not panel.tracker.contact
window.save_display_config()
saved=json.loads(window._config_path.read_text(encoding='utf-8'))['touchpad']
assert saved['source']=='host_continuous' and saved['position_reference_seconds']==.6
assert saved['position_noise_floor']==25
assert saved['position_decay_enabled'] and saved['position_decay_ms']==120
assert saved['position_restore_ms']==30 and saved['position_decay_percent']==70
if os.environ.get('TOUCHPAD_CONTINUOUS_SCREENSHOT'):
    window.resize(1700,1000); app.processEvents()
    window.grab().save(os.environ['TOUCHPAD_CONTINUOUS_SCREENSHOT'])
panel.position_decay_enabled.setChecked(False)
window.save_display_config()
window.close(); app.processEvents()
restarted=MainWindow(); restarted.refresh_timer.stop(); restarted._startup_connect_timer.stop()
assert restarted.touchpad.source.currentData()=='host_continuous'
assert restarted.touchpad.position_reference_seconds.value()==.6
assert restarted.touchpad.position_noise_floor.value()==25
assert not restarted.touchpad.position_decay_enabled.isChecked()
assert not restarted.touchpad.tracker.position_decay_enabled
assert restarted.touchpad.position_decay_ms.value()==120
assert restarted.touchpad.position_restore_ms.value()==30
assert restarted.touchpad.position_decay_percent.value()==70
assert restarted.touchpad.tracker.position_reference is None
restarted.close(); app.processEvents()
assert not errors,errors
'''


class ContinuousUiTests(unittest.TestCase):
    def run_script(self, script):
        with tempfile.TemporaryDirectory() as directory:
            for path in (ROOT/'PY_SRIPT/algo_monitor').glob('*.py'): shutil.copy2(path,directory)
            target=Path(directory)/'continuous_smoke.py'; target.write_text(script,encoding='utf-8')
            result=subprocess.run([sys.executable,str(target)],cwd=directory,
                env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONIOENCODING='utf-8'),
                capture_output=True,text=True,encoding='utf-8',timeout=45)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_same_frame_pipeline_logging_and_persistence(self): self.run_script(SCRIPT)

    def test_capture_waits_for_initial_reference_and_records_geometry(self):
        script=SLIDE_SCRIPT.replace("page.start()\n    assert page.running and window.touchpad.enabled_box.isChecked()",
            "window.touchpad.source.setCurrentIndex(window.touchpad.source.findData('host_continuous'))\n"
            "    page.start()\n    assert page.running and window.touchpad.enabled_box.isChecked()",1)
        script=script.replace("seen = []",r'''# 模拟参考尚未就绪：倒计时到期不得进入第一趟滑动。
    original_context=page.context
    page.context=lambda: dict(original_context(),position_reference_ready=False)
    advance(3.05); page.tick()
    assert page.running and page.index==0 and '等待' in page.prompt.text()
    page.context=original_context
    advance(.25); page.tick()
    assert page.running and page.index==1
    seen = []''',1)
        script=script.replace("assert meta['outcome'] == 'completed', meta",
            "assert meta['outcome'] == 'completed', meta\n"
            "    assert meta['context']['sensor_geometry']['pitch_mm']==8.\n"
            "    assert any(r['kind']=='touchpad' and r['frame']['position_reference'] for r in records)",1)
        self.run_script(script)


if __name__=='__main__': unittest.main()

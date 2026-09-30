"""在真实Qt WebEngine中验证独立HTML的组合筛选、排序和CSV内容。"""
import copy
import csv
import io
import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('QTWEBENGINE_CHROMIUM_FLAGS', '--disable-gpu')
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWebEngineWidgets import QWebEngineView
from test_rebound_analysis import fixture
from rebound_analysis import analyze_rebound, _write_report


class ReportFilterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        if os.name=='nt': QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')

    def wait_until(self, predicate, timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.app.processEvents()
            if predicate(): return
            time.sleep(.01)
        self.fail('Browser callback timeout')

    def js(self, expression):
        result=[]
        self.view.page().runJavaScript('JSON.stringify('+expression+')', lambda value:result.append(value))
        self.wait_until(lambda:bool(result))
        return json.loads(result[0])

    def set_filter(self, key, values):
        self.js('(() => {const wanted='+json.dumps(values)+';'
                'document.querySelectorAll(\'input[data-filter="'+key+'"]\').forEach(e=>e.checked=wanted.includes(Number(e.value)));'
                'document.getElementById("recovery-filters").dispatchEvent(new Event("change"));return true;})()')

    def visible(self):
        return self.js('Array.from(document.querySelectorAll("#recovery-table tbody tr:not([hidden])"))'
                       '.map(r=>[Number(r.cells[0].textContent),Number(r.cells[1].textContent),r.cells[2].textContent])')

    def test_combined_filters_numeric_sort_export_reset_and_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=fixture(Path(tmp))
            result=analyze_rebound(path); report=result['report']; template=copy.deepcopy(report['channels'])
            report['channels']=[]
            index=0
            for repeat in range(2):
                for force in (1,2,10):
                    for hold in (2,10,30):
                        index+=1
                        for original in template:
                            row=dict(original,planned_trial_index=index,target_n=force,hold_requested_s=hold,
                                     hold_actual_s=hold+.02,residual_10s=(index-9)*10,
                                     confirmed_idle_1s=None if index%3==0 else index)
                            report['channels'].append(row)
            report['completed_conditions']=18
            output=Path(result['output_dir']);_write_report(output,report)
            self.view=QWebEngineView();self.view.resize(1700,950);self.view.show()
            loaded=[];self.view.loadFinished.connect(lambda ok:loaded.append(ok))
            self.view.setUrl(QtCore.QUrl.fromLocalFile(str(output/'report.html')))
            self.wait_until(lambda:bool(loaded));self.assertTrue(loaded[-1])
            self.assertEqual(len(self.visible()),72)
            self.set_filter('target_n',[10]);self.assertEqual(len(self.visible()),24)
            self.set_filter('channel',[4]);self.assertEqual(len(self.visible()),6)
            self.js('(() => {const e=document.getElementById("sort-field");e.value="hold_requested_s";e.dispatchEvent(new Event("change",{bubbles:true}));return true;})()')
            self.assertEqual([r[0] for r in self.visible()],[7,16,8,17,9,18])
            self.set_filter('hold_requested_s',[2,30]);self.assertEqual(len(self.visible()),4)
            self.assertEqual([r[0] for r in self.visible()],[7,16,9,18])
            # 实际点导出按钮，截获Blob检查BOM和CSV，避免测试触发保存对话框。
            self.js('''(() => {
                window.exportText=null;
                URL.createObjectURL=blob=>{blob.arrayBuffer().then(buffer=>{
                    window.exportBytes=Array.from(new Uint8Array(buffer).slice(0,3));
                    window.exportText=new TextDecoder('utf-8',{ignoreBOM:true}).decode(buffer);
                });return 'blob:test';};
                HTMLAnchorElement.prototype.click=function(){};
                document.getElementById('export-filtered').click(); return true;
            })()''')
            self.wait_until(lambda:self.js('window.exportText!==null'))
            self.assertEqual(self.js('window.exportBytes'),[239,187,191])
            exported=list(csv.DictReader(io.StringIO(self.js('window.exportText').lstrip('\ufeff'))))
            self.assertEqual(len(exported),4)
            self.assertEqual([int(r['planned_trial_index']) for r in exported],[7,16,9,18])
            self.assertTrue(all(float(r['target_n'])==10 and int(r['channel'])==4 for r in exported))
            self.assertEqual(float(exported[0]['residual_10s']),-20)
            self.assertIn('residual_10s_percent_of_hold_raw',exported[0])
            self.js('(() => {document.getElementById("reset-filters").click();return true;})()')
            self.assertEqual(len(self.visible()),72)
            self.js('(() => {document.querySelector(\'th[data-sort="residual_10s"] button\').click();return true;})()')
            numbers=self.js('Array.from(document.querySelectorAll("#recovery-table tbody tr:not([hidden])")).map(r=>Number(r.cells[6].textContent))')
            self.assertEqual(numbers,sorted(numbers))
            self.js('(() => {document.querySelector(\'th[data-sort="confirmed_idle_1s"] button\').click();document.querySelector(\'th[data-sort="confirmed_idle_1s"] button\').click();return true;})()')
            first_last=self.js('(() => {const r=document.querySelectorAll("#recovery-table tbody tr:not([hidden])");return [r[0].cells[14].textContent,r[r.length-1].cells[14].textContent];})()')
            self.assertEqual(first_last[0],'17')
            self.assertIn('未确认',first_last[1])
            self.set_filter('channel',[]);self.assertEqual(len(self.visible()),0)
            self.assertTrue(self.js('document.getElementById("export-filtered").disabled'))
            self.assertFalse(self.js('document.getElementById("filter-empty").hidden'))
            self.assertEqual(self.js('document.querySelector("table").tBodies[0].rows.length'),1)
            self.view.close()


if __name__=='__main__': unittest.main()

"""学习结束提示只读真实结果；界面验证不连接硬件。"""
import unittest
from unittest.mock import patch

import test_pressure_feedback as native
from test_one_step_repeatability import TrialUiTests


class StopNoticeTests(unittest.TestCase):
    page = native.FeedbackSettingsUiTests.page
    setUpClass = classmethod(TrialUiTests.setUpClass.__func__)
    setUp = TrialUiTests.setUp

    def stopped_result(self):
        return dict(return_only=True, complete=False,
            message='4 N轨迹学习未达标：未取得连续卸载证据；结束任务，不启动下一轮',
            working_release_check_error='工作释放点静止卸载确认超时',
            points=[dict(target_n=4, attempts=[dict(attempt=3)])],
            last_release_position_steps=-717, release_position_confirmed=True, pressure_unloaded=False)

    def test_failure_reason_is_prominent_and_persists_after_progress_and_return(self):
        w = self.widget
        w._running = w._cycle_running = True
        w._finished(self.stopped_result())
        text = w.learning_notice.text()
        for expected in ('学习已停止', '目标 4 N', '第 3 轮', '静止卸载确认超时', '-717 step', '压力卸载未确认'):
            self.assertIn(expected, text)
        self.assertFalse(w.learning_notice.isHidden())
        self.assertIn('#ff6b75', w.learning_notice.styleSheet())
        w._progress(dict(message='普通状态刷新'))
        w._tick()
        w._finished(dict(return_only=True, message='已回到0度位置并确认静止'))
        self.assertEqual(w.learning_notice.text(), text)

    def test_user_stop_and_emergency_have_distinct_state_and_do_not_claim_release(self):
        w = self.widget
        w._running = w._cycle_running = True
        w.stop()
        self.assertIn('正在停止学习', w.learning_notice.text())
        self.assertIn('尚未确认回位', w.learning_notice.text())
        result = self.stopped_result()
        result.update(cancelled=True, message='用户停止，已回释放点', pressure_unloaded=True)
        w._finished(result)
        self.assertIn('学习已手动停止', w.learning_notice.text())
        self.assertIn('#ffc857', w.learning_notice.styleSheet())
        w._stop.clear()
        w._running = w._cycle_running = True
        with patch.object(w.control, 'request_external_stop'):
            w.emergency_stop()
        self.assertIn('自动回位已取消', w.learning_notice.text())
        w._finished(dict(return_only=True, complete=False, emergency_stopped=True,
                         message='紧急停止', release_position_confirmed=False, pressure_unloaded=False))
        self.assertIn('尚未确认回位', w.learning_notice.text())
        self.assertIn('学习紧急停止', w.learning_notice.text())

    def test_worker_exception_uses_progress_context_and_success_is_green(self):
        w = self.widget
        w._cycle_running = True
        w._progress(dict(target_n=5, attempt=7, message='学习中'))
        w._finished(dict(return_only=True, message='CAN读取失败', release_position_confirmed=False))
        self.assertIn('目标 5 N · 第 7 轮', w.learning_notice.text())
        self.assertIn('CAN读取失败', w.learning_notice.text())
        w._cycle_running = True
        w._finished(dict(return_only=True, complete=True, message='30个压力点学习完成',
                         release_position_confirmed=True, pressure_unloaded=True))
        self.assertIn('学习已完成', w.learning_notice.text())
        self.assertIn('#6fd5a5', w.learning_notice.styleSheet())

    def test_only_accepted_new_learning_clears_previous_notice(self):
        w = self.widget
        w.confirm_table()
        w._set_learning_notice('上次学习停止：卸载未确认')
        with patch.object(w.control, 'start_external_operation', return_value=(False, '任务占用')):
            w.start_cycles()
        self.assertFalse(w.learning_notice.isHidden())
        with patch.object(w.control, 'start_external_operation', return_value=(True, '')):
            w.start_cycles()
        self.assertTrue(w.learning_notice.isHidden())
        w._running = w._cycle_running = False


if __name__ == '__main__':
    unittest.main()

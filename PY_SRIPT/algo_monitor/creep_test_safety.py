"""长时测试的有界等待与故障分类，不直接操作硬件。"""

import time
import sys


class CreepDeadlineEvent:
    """兼容 Event；保持用户停止，同时防止通信重试无限延长受压时间。"""

    def __init__(self, parent, seconds, error_type):
        self.parent = parent
        self.deadline = time.monotonic() + seconds
        self.error_type = error_type

    def is_set(self):
        if self.parent.is_set():
            return True
        if time.monotonic() >= self.deadline:
            raise self.error_type("creep_stage_timeout", "长时测试阶段超过硬超时，立即卸载")
        return False

    def wait(self, seconds):
        if self.is_set():
            return True
        result = self.parent.wait(min(seconds, max(0, self.deadline - time.monotonic())))
        return result or self.is_set()


# 只有动作或瞬时通信异常允许自动重试；超力、过流、零点异常等先卸载并暂停。
RECOVERABLE_CREEP_ERRORS = frozenset({
    "motion_timeout", "motion_position_error", "lookup_position_error",
    "ocr_unstable", "motor_communication_timeout", "creep_stage_timeout",
    "motor_fault", "pressure_meter_unavailable",
})
CREEP_MAX_ATTEMPTS = 3
CREEP_MAX_CONSECUTIVE_FAILURES = 3
CREEP_PRESS_TIMEOUT_SECONDS = 30.0
CREEP_RELEASE_TIMEOUT_SECONDS = 20.0
CREEP_RECOVERY_SECONDS = 360.0


def prevent_idle_sleep(enabled):
    """仅在当前测试线程存活期间阻止Windows空闲休眠，退出后恢复。"""
    if sys.platform == "win32":
        import ctypes
        return bool(ctypes.windll.kernel32.SetThreadExecutionState(
            0x80000001 if enabled else 0x80000000
        ))
    return True

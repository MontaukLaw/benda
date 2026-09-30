"""管理 candleLight USB 句柄，覆盖 python-can 4.6 的关闭及初始化失败清理。"""

import logging

from can import BusABC
from can.interfaces.gs_usb import GsUsbBus
from usb.util import dispose_resources


LOG = logging.getLogger(__name__)


class ManagedGsUsbBus(GsUsbBus):
    def __init__(self, *args, **kwargs):
        # 父类在 USB 初始化完成前就标记为已打开；失败时也必须回收句柄。
        self.gs_usb = None
        self._periodic_tasks = []
        self._is_shutdown = False
        try:
            super().__init__(*args, **kwargs)
        except Exception:
            try:
                self.shutdown()
            except Exception:
                LOG.exception("CAN 初始化失败后的 USB 清理失败")
            raise

    def shutdown(self):
        if self._is_shutdown:
            return
        # 只关闭自己持有的设备，不采用父类关闭时重新扫描并启动设备的流程。
        device = self.gs_usb
        try:
            BusABC.shutdown(self)
        finally:
            if device is not None:
                try:
                    device.stop()
                finally:
                    # stop 只停止 CAN 控制器，PyUSB 接口和句柄还需显式释放。
                    dispose_resources(device.gs_usb)


def open_gs_usb_bus(**kwargs):
    kwargs.pop("interface", None)
    return ManagedGsUsbBus(**kwargs)

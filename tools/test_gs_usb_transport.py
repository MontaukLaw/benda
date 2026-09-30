"""使用真实 python-can 初始化流程和模拟 USB 设备验证清理，不访问硬件。"""

import gc
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "PY_SRIPT/algo_monitor"))

from gs_usb_transport import ManagedGsUsbBus


class TransportTests(unittest.TestCase):
    def device(self):
        device = Mock()
        device.device_capability.fclk_can = 48000000
        return device

    def test_reconnect_releases_before_next_open_without_shutdown_scan(self):
        device = self.device()
        events = []
        device.start.side_effect = lambda: events.append("start")
        device.stop.side_effect = lambda: events.append("stop")
        with patch("can.interfaces.gs_usb.GsUsb.scan", return_value=[device]) as scan, \
                patch("gs_usb_transport.dispose_resources", side_effect=lambda _: events.append("dispose")) as dispose:
            for _ in range(10):
                bus = ManagedGsUsbBus(channel=0, bitrate=500000)
                bus.shutdown()
                bus.shutdown()
            self.assertEqual(scan.call_count, 10)
            self.assertEqual(events, ["start", "stop", "dispose"] * 10)
            self.assertEqual(dispose.call_count, 10)
            dispose.assert_called_with(device.gs_usb)

    def test_missing_device_does_not_warn_or_attempt_usb_cleanup(self):
        with patch("can.interfaces.gs_usb.GsUsb.scan", return_value=[]), \
                patch("gs_usb_transport.dispose_resources") as dispose, \
                self.assertNoLogs("can.bus", level="WARNING"):
            with self.assertRaisesRegex(Exception, "Cannot find device"):
                ManagedGsUsbBus(channel=0)
            gc.collect()
            dispose.assert_not_called()

    def test_initialization_failure_releases_usb_and_preserves_error(self):
        for failing_method in ("set_timing", "start"):
            with self.subTest(method=failing_method):
                device = self.device()
                getattr(device, failing_method).side_effect = RuntimeError("original USB failure")
                with patch("can.interfaces.gs_usb.GsUsb.scan", return_value=[device]), \
                        patch("gs_usb_transport.dispose_resources") as dispose, \
                        self.assertNoLogs("can.bus", level="WARNING"):
                    with self.assertRaisesRegex(RuntimeError, "original USB failure"):
                        ManagedGsUsbBus(channel=0)
                    gc.collect()
                    device.stop.assert_called_once()
                    dispose.assert_called_once_with(device.gs_usb)

    def test_stop_failure_still_releases_handle(self):
        device = self.device()
        with patch("can.interfaces.gs_usb.GsUsb.scan", return_value=[device]), \
                patch("gs_usb_transport.dispose_resources") as dispose:
            bus = ManagedGsUsbBus(channel=0)
            device.stop.side_effect = RuntimeError("stop failed")
            with self.assertRaisesRegex(RuntimeError, "stop failed"):
                bus.shutdown()
            dispose.assert_called_once_with(device.gs_usb)
            bus.shutdown()
            device.stop.assert_called_once()

    def test_cleanup_failure_does_not_replace_initialization_error(self):
        device = self.device()
        device.start.side_effect = RuntimeError("open failed")
        with patch("can.interfaces.gs_usb.GsUsb.scan", return_value=[device]), \
                patch("gs_usb_transport.dispose_resources", side_effect=RuntimeError("release failed")), \
                self.assertLogs("gs_usb_transport", level="ERROR"):
            with self.assertRaisesRegex(RuntimeError, "open failed"):
                ManagedGsUsbBus(channel=0)


if __name__ == "__main__":
    unittest.main()

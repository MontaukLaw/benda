"""Current application startup regression (sensor-only UI)."""
from test_sensor_only_monitor import SensorOnlyTests
from test_serial_autoconnect import AutoConnectTests

if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)

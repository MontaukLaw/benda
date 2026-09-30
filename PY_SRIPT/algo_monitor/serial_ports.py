from config import TARGET_PORT_NAME


IGNORED_PORT_KEYWORDS = ("蓝牙", "bluetooth")


class SerialPortScanner:
    def __init__(self, target_name=TARGET_PORT_NAME):
        self.target_name = target_name

    def list_ports(self):
        try:
            from serial.tools import list_ports
        except ImportError:
            return [], "pyserial is not installed"

        ports = []
        for port in list_ports.comports():
            text = self.describe_port(port)
            if self.is_ignored_port(text):
                continue
            ports.append((port.device, text))

        if ports:
            return ports, "Available: " + ", ".join(port for port, _text in ports)
        return [], "No serial ports found"

    def find_target_port(self):
        ports, message = self.list_ports()
        for device, text in ports:
            if self.target_name.lower() in text.lower():
                return device, text

        if ports:
            names = ", ".join(device for device, _text in ports)
            return None, "Target not found. Available: " + names
        return None, message

    def describe_port(self, port):
        return " ".join(str(item) for item in (
            port.device,
            port.description,
            port.manufacturer,
            port.product,
        ) if item)

    def is_ignored_port(self, text):
        normalized = text.lower()
        return any(keyword in normalized for keyword in IGNORED_PORT_KEYWORDS)

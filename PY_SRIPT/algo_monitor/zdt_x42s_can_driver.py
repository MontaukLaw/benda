"""ZDT X42S 自由协议 CAN 驱动；独立于 PD42S1 和 Modbus RTU。

依据用户手册 V1.0.5 第 4.2、5.2.1、5.3.2、5.5.12、5.8 节。
提供状态读取、使能、编码器校准、Emm普通相对位置测试及碰撞回零。
"""

from dataclasses import dataclass
import logging
import math
import sys
import threading
import time
from motor_speed_units import valid_rpm

try:
    import can
except ModuleNotFoundError:
    can = None


LOG = logging.getLogger(__name__)


class ZDTError(Exception):
    pass


class ZDTTimeoutError(ZDTError):
    pass


class ZDTProtocolError(ZDTError):
    pass


@dataclass(frozen=True)
class ZDTConfig:
    interface: str = "gs_usb"
    channel: int | str = 0
    bitrate: int = 500000
    node_id: int = 1
    timeout: float = 0.2
    # 当前实机单读温度采用00正、01负；手册第72页的相反说明保留为配置兼容项。
    temperature_positive_sign: int = 0
    steps_per_turn: int = 3200

    def __post_init__(self):
        if type(self.steps_per_turn) is not int or not 1 <= self.steps_per_turn <= 1000000:
            raise ValueError("steps_per_turn 必须为 1～1000000 的整数，且与电机细分一致")
        if type(self.node_id) is not int or not 1 <= self.node_id <= 255:
            raise ValueError("新电机地址必须为 1～255，禁止使用广播地址")
        if self.bitrate not in (10000, 20000, 50000, 83333, 100000, 125000, 250000, 500000, 800000, 1000000):
            raise ValueError("不支持的 CAN 速率")
        if not math.isfinite(self.timeout) or not 0.01 <= self.timeout <= 1.0:
            raise ValueError("CAN 响应超时必须为 0.01～1 秒")
        if self.temperature_positive_sign not in (0, 1):
            raise ValueError("temperature_positive_sign 必须为 0 或 1")
        if not self.interface:
            raise ValueError("CAN 接口不能为空")
        if self.interface == "gs_usb":
            if type(self.channel) is not int or self.channel < 0:
                raise ValueError("candleLight 的 channel 必须为从 0 开始的整数设备序号")
        elif not self.channel:
            raise ValueError("CAN 通道不能为空")


def prepare_gs_usb_backend():
    """为candleLight加载USB库，Windows使用随Python依赖提供的libusb DLL。"""
    try:
        import usb.backend.libusb1
        import gs_usb.gs_usb
        if sys.platform == "win32":
            import libusb_package
            # PyUSB会缓存此后端，gs_usb扫描时可直接复用，不改系统PATH和USB驱动。
            backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        else:
            backend = usb.backend.libusb1.get_backend()
    except ImportError as exc:
        # 使用运行App的解释器，避免py启动器把依赖安装到Conda以外的环境。
        missing = getattr(exc, "name", None) or str(exc)
        raise ZDTError(
            f"无法导入 candleLight USB 依赖：{missing}；当前 Python：{sys.executable}。"
            f'请在此环境执行："{sys.executable}" -m pip install '
            '"python-can[gs-usb]==4.6.1" libusb-package'
        ) from exc
    if backend is None:
        raise ZDTError("无法加载 libusb；请安装与当前 Python 位数匹配的 libusb-package")


def signed_magnitude(data):
    if len(data) < 2 or data[0] not in (0, 1):
        raise ZDTProtocolError("无效的符号/数值字段")
    return (-1 if data[0] else 1) * int.from_bytes(data[1:], "big")


def degrees_to_pulses(angle_deg, pulses_per_turn):
    """按实际指令细分换算角度，四舍五入到整脉冲，不混用编码器65536计数。"""
    if not math.isfinite(angle_deg) or angle_deg <= 0:
        raise ValueError("相对转角必须为正数")
    if type(pulses_per_turn) is not int or not 1 <= pulses_per_turn <= 1000000:
        raise ValueError("每圈指令脉冲数必须为1～1000000的整数")
    pulses = math.floor(angle_deg / 360 * pulses_per_turn + 0.5)
    if not 1 <= pulses <= 0xFFFFFFFF:
        raise ValueError("角度换算后必须为1～4294967295脉冲，请增大过小的角度")
    return pulses


def parse_system_status(packet):
    """从完整逻辑报文自动识别 X/Emm 格式，保留真实反馈和原始单位。"""
    if len(packet) not in (31, 37) or packet[1] != 0x43 or packet[-1] != 0x6B:
        raise ZDTProtocolError("系统状态报文长度、功能码或校验尾错误")
    is_x = len(packet) == 37
    if packet[2:4] != bytes((len(packet), 12 if is_x else 9)):
        raise ZDTProtocolError("系统状态字节数或参数数目不符")
    u16 = lambda start: int.from_bytes(packet[start:start + 2], "big")
    offset = 14 if is_x else 10
    position_scale = 0.1 if is_x else 360.0 / 65536
    raw_position = signed_magnitude(packet[offset + 8:offset + 13])
    state = packet[-2]
    home = packet[-3]
    result = {
        "firmware": "X" if is_x else "Emm",
        "voltage_mv": u16(4),
        "bus_current_ma": u16(6) if is_x else None,
        "phase_current_ma": u16(8 if is_x else 6),
        "encoder": u16(12 if is_x else 8),
        "position_raw": raw_position,
        "position_deg": raw_position * position_scale,
        "target_deg": signed_magnitude(packet[offset:offset + 5]) * position_scale,
        "speed_rpm": signed_magnitude(packet[offset + 5:offset + 8]) * (0.1 if is_x else 1),
        "error_deg": signed_magnitude(packet[offset + 13:offset + 18]) * (0.01 if is_x else position_scale),
        "state_flags": state,
        "home_flags": home,
        "enabled": bool(state & 1),
        "in_position": bool(state & 2),
        "stall": bool(state & 4),
        "stall_protection": bool(state & 8),
        "encoder_ready": bool(home & 1),
        "calibration_ready": bool(home & 2),
        "homing": bool(home & 4),
        "home_failed": bool(home & 8),
        "overtemperature": bool(home & 16),
        "overcurrent": bool(home & 32),
    }
    if is_x:
        result["temperature_c"] = signed_magnitude(packet[32:34])
        result["temperature_raw"] = packet[32:34].hex(" ").upper()
    return result


class ZDTX42SMotor:
    def __init__(self, config=None, bus_factory=None):
        self.config = config or ZDTConfig()
        self.bus = None
        self._bus_factory = bus_factory
        self._lock = threading.Lock()
        self._firmware = None
        self._emm_speed_scale = None
        self._temperature_cache = None

    def open(self):
        if self.bus is not None:
            return
        self._temperature_cache = None
        if can is None and self._bus_factory is None:
            raise ZDTError("缺少 python-can，无法连接 CAN")
        factory = self._bus_factory or can.Bus
        if self.config.interface == "gs_usb" and self._bus_factory is None:
            prepare_gs_usb_backend()
            # 延迟导入，其他 CAN 后端不依赖 gs_usb；句柄由本应用确定性释放。
            from gs_usb_transport import open_gs_usb_bus
            factory = open_gs_usb_bus
        try:
            self.bus = factory(interface=self.config.interface, channel=self.config.channel,
                               bitrate=self.config.bitrate, receive_own_messages=False)
        except Exception as exc:
            LOG.exception("CAN 打开失败：interface=%s channel=%s", self.config.interface, self.config.channel)
            if self.config.interface == "gs_usb":
                raise ZDTError(
                    f"无法打开 candleLight USB CAN（设备序号 {self.config.channel}）：{exc}。"
                    "请检查USB连接、WinUSB驱动，以及是否被其他CAN软件占用。"
                ) from exc
            raise

    def close(self):
        with self._lock:
            bus, self.bus = self.bus, None
            self._firmware = None
            self._emm_speed_scale = None
            if bus is not None:
                bus.shutdown()

    def _request(self, code, data=b"", response_length=None):
        try:
            return self._request_transaction(code, data, response_length)
        except Exception:
            # 通信异常后单位不再可信，后续运动须在静止反馈下重新建立。
            self._emm_speed_scale = None
            raise

    def _request_transaction(self, code, data=b"", response_length=None):
        # 单个服务线程串行事务；不重发校准等写命令，避免超时后重复动作。
        with self._lock:
            if self.bus is None:
                raise ZDTError("CAN 尚未连接")
            # 清理先前事务的迟到回复；限制清理时间，避免繁忙总线阻塞退出。
            drain_deadline = time.monotonic() + self.config.timeout
            while self.bus.recv(timeout=0) is not None:
                if time.monotonic() >= drain_deadline:
                    raise ZDTError("CAN 接收队列持续繁忙")
            deadline = time.monotonic() + self.config.timeout
            payload = data + b"\x6b"
            for part, start in enumerate(range(0, len(payload), 7)):
                self.bus.send(can.Message(
                    arbitration_id=(self.config.node_id << 8) | part,
                    is_extended_id=True, data=bytes((code,)) + payload[start:start + 7],
                ), timeout=max(0, deadline - time.monotonic()))
            assembled = bytearray((self.config.node_id, code))
            expected_part = 0
            expected_length = response_length
            while time.monotonic() < deadline:
                # gs_usb按毫秒取整；正超时至少1ms，避免舍入成USB无限等待的0。
                message = self.bus.recv(timeout=max(0.001, deadline - time.monotonic()))
                if message is None:
                    break
                if (not message.is_extended_id or message.is_error_frame or message.is_remote_frame
                        or getattr(message, "is_rx", True) is False
                        or message.arbitration_id >> 8 != self.config.node_id):
                    continue
                chunk = bytes(message.data)
                if not chunk or chunk[0] != code:
                    continue
                part = message.arbitration_id & 0xFF
                if part != expected_part or not 2 <= len(chunk) <= 8:
                    raise ZDTProtocolError("CAN 分包序号或长度错误")
                if part == 0 and len(chunk) == 3 and chunk[-1] == 0x6B and chunk[1] in (0xE2, 0xEE):
                    raise ZDTProtocolError(f"电机拒绝 0x{code:02X}：0x{chunk[1]:02X}")
                assembled.extend(chunk[1:])
                expected_part += 1
                if code == 0x43 and expected_length is None:
                    expected_length = assembled[2]
                    if expected_length not in (31, 37):
                        raise ZDTProtocolError("未知系统状态格式")
                if expected_length is not None and len(assembled) >= expected_length:
                    if len(assembled) != expected_length or assembled[-1] != 0x6B:
                        raise ZDTProtocolError("CAN 响应长度或固定校验字节错误")
                    return bytes(assembled)
                if len(chunk) != 8:
                    raise ZDTProtocolError("CAN 中间分包不足8字节")
            raise ZDTTimeoutError(
                f"CAN适配器已打开，但电机0x{code:02X}响应超时；请检查供电、CAN H/L、终端电阻、"
                f"地址{self.config.node_id}、{self.config.bitrate / 1000:g}K速率、CAN1_MAP和0x6B校验设置"
            )

    def read_status(self):
        result = parse_system_status(self._request(0x43, b"\x7a"))
        result["sampled_at"] = time.monotonic()
        if result["firmware"] == "Emm":
            # 温度独立命令最多每2秒读取一次；位置、电流、保护标志每轮仍读取真实状态。
            cached = self._temperature_cache
            if cached is None or result['sampled_at']-cached['temperature_sampled_at'] >= 2:
                packet = self._request(0x39, response_length=5)
                sign, magnitude = packet[2:4]
                if sign not in (0, 1):
                    raise ZDTProtocolError("温度符号无效")
                cached = dict(temperature_c=magnitude * (1 if sign == self.config.temperature_positive_sign else -1),
                              temperature_raw=packet[2:4].hex(" ").upper(), temperature_sampled_at=time.monotonic())
                self._temperature_cache = cached
            result.update(cached)
        self._firmware = result["firmware"]
        return result

    def _action(self, code, data):
        packet = self._request(code, data, response_length=4)
        if packet[2] != 0x02:
            self._emm_speed_scale = None
            raise ZDTProtocolError(f"动作未被确认：{packet.hex(' ')}")

    def _encode_motion_speed(self, speed_rpm):
        """确认Emm命令速度单位后编码；仅修改本次上电配置，不写Flash。"""
        if not valid_rpm(speed_rpm):
            raise ValueError("速度必须为0.1～3000 RPM，精确到0.1 RPM")
        # 手册FD速度字段上限3000；超过300 RPM时仅允许整数速度，静止后切回1RPM编码。
        scale = 10 if speed_rpm <= 300 else 1
        if not valid_rpm(speed_rpm, step=1/scale):
            raise ValueError("超过300 RPM时仅支持整数速度")
        if self._emm_speed_scale != scale:
            sample = self.read_status()
            if (sample['firmware'] != 'Emm' or sample['speed_rpm'] != 0 or not sample['in_position']
                    or not sample['enabled'] or not sample['encoder_ready'] or not sample['calibration_ready']
                    or any(sample[k] for k in ('homing', 'stall', 'stall_protection', 'overtemperature', 'overcurrent'))):
                raise ZDTError('速度单位尚未确认或需切换；须先停止并取得无故障位置反馈')
        # 每条运动命令前重新确认同一单位，覆盖掉电重启/外部改设置；无确认绝不发送位移。
        self._emm_speed_scale = None
        self._action(0x4F, bytes((0x71, 0, int(scale == 10))))
        self._emm_speed_scale = scale
        return round(speed_rpm*scale)

    def set_enabled(self, enabled):
        self._action(0xF3, bytes((0xAB, int(bool(enabled)), 0)))

    def calibrate_encoder(self):
        self._action(0x06, b"\x45")

    def zero_position(self):
        """手册5.2.3：将当前角度设为零，不触发机械回零，不重发。"""
        self._action(0x0A, b"\x6d")

    def set_current_limit_ma(self, current_ma):
        """手册5.6.13：只修改本次上电的电流限制，不写入非易失存储。"""
        if type(current_ma) is not int or not 1 <= current_ma <= 5000:
            raise ValueError("电流限制必须为1～5000 mA")
        self._action(0x45, b"\x66\x00" + current_ma.to_bytes(2, "big"))

    def move_relative_degrees(self, angle_deg, speed_rpm, acceleration, pulses_per_turn, ccw):
        """保留角度调用入口，换算后统一通过整数脉冲接口发送。"""
        pulses = degrees_to_pulses(angle_deg, pulses_per_turn)
        return self.move_relative_pulses(pulses, speed_rpm, acceleration, ccw)

    def move_relative_pulses(self, pulses, speed_rpm, acceleration, ccw):
        """Emm FD：相对当前实时位置运动，模式02、立即执行，不重发运动命令。"""
        if self._firmware != "Emm":
            raise ZDTError("位置测试仅支持已读取状态确认的Emm固件")
        if type(pulses) is not int or not 1 <= pulses <= 0xFFFFFFFF:
            raise ValueError("移动脉冲数必须为1～4294967295的整数")
        if not valid_rpm(speed_rpm):
            raise ValueError("速度必须为0.1～3000 RPM，精确到0.1 RPM")
        if type(acceleration) is not int or not 0 <= acceleration <= 255:
            raise ValueError("加速度档位必须为0～255的整数")
        if type(ccw) is not bool:
            raise ValueError("必须明确指定顺时针或逆时针")
        encoded_speed = self._encode_motion_speed(speed_rpm)
        data = (bytes((int(ccw),)) + encoded_speed.to_bytes(2, "big")
                + bytes((acceleration,)) + pulses.to_bytes(4, "big") + b"\x02\x00")
        self._action(0xFD, data)
        return pulses

    def stop_motion(self):
        """FE立即停止，保留使能状态，不使用失能松轴替代停止。"""
        self._action(0xFE, b"\x98\x00")

    def move_absolute_pulses(self, position, speed_rpm, acceleration):
        """Emm FD模式01：固定坐标终点，允许运动中平滑更新速度和加减速；超时不重发。"""
        if self._firmware != "Emm":
            raise ZDTError("绝对位置轨迹仅支持已确认的Emm固件")
        if type(position) is not int or abs(position) > 0xFFFFFFFF:
            raise ValueError("绝对位置超出协议范围")
        if not valid_rpm(speed_rpm):
            raise ValueError("速度必须为0.1～3000 RPM，精确到0.1 RPM")
        if type(acceleration) is not int or not 1 <= acceleration <= 255:
            raise ValueError("连续轨迹必须启用1～255档加减速")
        encoded_speed = self._encode_motion_speed(speed_rpm)
        data = (bytes((int(position < 0),)) + encoded_speed.to_bytes(2, "big")
                + bytes((acceleration,)) + abs(position).to_bytes(4, "big") + b"\x01\x00")
        self._action(0xFD, data)

    def read_homing_parameters(self):
        """读取电机现有回零参数及碰撞后退让设置，不修改或保存参数。"""
        packet = self._request(0x22, response_length=18)
        u16 = lambda start: int.from_bytes(packet[start:start + 2], "big")
        if packet[2] > 5 or packet[3] > 1 or packet[16] > 1:
            raise ZDTProtocolError("回零参数中的模式、方向或上电回零开关无效")
        result = {
            "mode": packet[2], "direction": packet[3], "speed_rpm": u16(4),
            "timeout_ms": int.from_bytes(packet[6:10], "big"),
            "detect_speed_rpm": u16(10), "detect_current_ma": u16(12),
            "detect_time_ms": u16(14), "power_on_homing": bool(packet[16]),
        }
        backoff = self._request(0x3F, response_length=5)
        result["backoff_raw"] = int.from_bytes(backoff[2:4], "big")
        return result

    def home_sensorless(self):
        """立即触发多圈无限位碰撞回零，模式02；接收确认不代表回零成功。"""
        if self._firmware != "Emm":
            raise ZDTError("碰撞回零测试仅支持已读取状态确认的Emm固件")
        self._action(0x9A, b"\x02\x00")

    def abort_homing(self):
        """按手册强制中断并退出回零，不能以普通位置停止替代。"""
        self._action(0x9C, b"\x48")

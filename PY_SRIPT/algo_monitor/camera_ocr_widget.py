import base64
import collections
import re
import shutil
import statistics
import subprocess
import tempfile
import threading
import time
from pathlib import Path

from pyqtgraph.Qt import QT_LIB

from qt_compat import QtCore, QtGui, QtWidgets, Signal


OCR_MODEL_IMPORT_ERROR = None
try:
    import cv2
    import numpy as np
    from rapidocr import RapidOCR
except ImportError as exc:
    cv2 = None
    np = None
    RapidOCR = None
    OCR_MODEL_IMPORT_ERROR = str(exc)


MULTIMEDIA_IMPORT_ERROR = None
try:
    if QT_LIB == "PySide6":
        from PySide6 import QtMultimedia
    elif QT_LIB == "PyQt6":
        from PyQt6 import QtMultimedia
    else:
        QtMultimedia = None
        MULTIMEDIA_IMPORT_ERROR = (
            f"当前Qt绑定为{QT_LIB}；后台摄像头采集需要PySide6或PyQt6。"
        )
except ImportError as exc:
    QtMultimedia = None
    MULTIMEDIA_IMPORT_ERROR = str(exc)


OCR_INTERVAL_MS = 50
OCR_LANGUAGE = "zh-Hans-CN"
OCR_SCRIPT_PATH = Path(__file__).resolve().with_name("windows_ocr.ps1")
OCR_IMAGE_PATH = Path(tempfile.gettempdir()) / "algo_monitor_fv_ocr.png"
OCR_MISSED_WARNING_COUNT = 5
CAMERA_NO_FRAME_WARNING_COUNT = 40
OCR_FORCE_MAX_G = 5000.0
OCR_NEGATIVE_ZERO_TOLERANCE_G = 5.0
OCR_FILTER_WINDOW_SIZE = 5
OCR_FILTER_ALPHA = 0.45
OCR_JUMP_MIN_G = 80.0
OCR_JUMP_RATIO = 0.35
OCR_JUMP_CONFIRMATION_COUNT = 2
OCR_MODEL_MIN_SCORE = 0.92
OCR_MODEL_CONSENSUS_TOLERANCE_G = 1.0
OCR_FRAME_BATCH_SIZE = 5
OCR_FRAME_STABILITY_MAX_MAD = 12.0
OCR_PREFERRED_WIDTH = 1280
OCR_PREFERRED_HEIGHT = 720
OCR_PREFERRED_FPS = 30.0
FV_ROI_LEFT = 0.20
FV_ROI_TOP = 0.38
FV_ROI_WIDTH = 0.54
FV_ROI_HEIGHT = 0.23


def _compact_ocr_text(text):
    normalized = str(text).translate(
        str.maketrans(
            {
                "：": ":",
                "．": ".",
                "。": ".",
                "·": ".",
                "，": ".",
                "－": "-",
                "−": "-",
                "–": "-",
                "＋": "+",
            }
        )
    )
    return re.sub(r"\s+", "", normalized).lower()


def _parse_first_number(compact, patterns):
    for pattern in patterns:
        match = re.search(pattern, compact)
        if match is not None:
            try:
                return float(match.group(1))
            except ValueError:
                pass
    return None


def _extract_fv_pressure(text):
    """从OCR文本中提取以g为单位的fv压力数值供诊断显示。"""
    compact = _compact_ocr_text(text)
    patterns = (
        r"(?:i|l)?fv[^0-9+\-]{0,6}([+\-]?\d+(?:\.\d+)?)g",
        r"([+\-]?\d+(?:\.\d+)?)g",
        r"(?:i|l)?fv[^0-9+\-]{0,6}([+\-]?\d+(?:\.\d+)?)(?=$|[^0-9a-z.])",
        r"([+\-]?\d+(?:\.\d+)?)(?=$|[^0-9a-z.])",
    )
    return _parse_first_number(compact, patterns)


def _extract_fv_control_pressure(text):
    """控制链路只接受fv标记、两位小数和g单位完整的压力数值。"""
    compact = _compact_ocr_text(text)
    return _parse_first_number(
        compact,
        (r"(?:i|l)?fv[^0-9+\-]{0,6}([+\-]?\d+\.\d{2})g",),
    )


def _extract_fv_model_pressure(text):
    """提取模型整行结果；缺少g时只能交给双路一致性规则确认。"""
    compact = _compact_ocr_text(text)
    return _parse_first_number(
        compact,
        (r"^(?:i|l)?fv[^0-9+\-]{0,6}([+\-]?\d+\.\d{2})g?$",),
    )


def _crop_fv_region(image):
    """按固定机位只保留fv读数的数值、符号和g单位区域。"""
    width = image.width()
    height = image.height()
    return image.copy(
        int(round(width * FV_ROI_LEFT)),
        int(round(height * FV_ROI_TOP)),
        max(1, int(round(width * FV_ROI_WIDTH))),
        max(1, int(round(height * FV_ROI_HEIGHT))),
    )


def _qimage_to_bgr(image):
    """将Qt图像转换为OpenCV BGR数组，并复制数据避免悬空引用。"""
    if cv2 is None or np is None:
        raise RuntimeError(OCR_MODEL_IMPORT_ERROR or "OpenCV不可用")
    format_group = getattr(QtGui.QImage, "Format", None)
    rgb888 = (
        format_group.Format_RGB888
        if format_group is not None
        else QtGui.QImage.Format_RGB888
    )
    converted = image.convertToFormat(rgb888)
    height = converted.height()
    width = converted.width()
    stride = converted.bytesPerLine()
    bits = converted.bits()
    if hasattr(bits, "setsize"):
        bits.setsize(height * stride)
    raw = np.frombuffer(bits, dtype=np.uint8, count=height * stride)
    rgb = raw.reshape(height, stride)[:, : width * 3].reshape(height, width, 3)
    return cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2BGR)


def _enhance_fv_for_ocr(image_bgr):
    """抑制LCD彩色摩尔纹并增强明暗不均下的字符对比度。"""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    return cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)


def _select_stable_fv_image(images):
    """从连续裁剪图中选择变化最小且更清晰的一帧，避开屏幕刷新撕裂。"""
    images = tuple(image for image in images if image is not None and not image.isNull())
    if not images:
        return None, None
    if cv2 is None or np is None or len(images) < 2:
        return images[-1].copy(), None

    grays = []
    sharpness = []
    for image in images:
        bgr = _qimage_to_bgr(image)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        comparison = cv2.resize(
            cv2.GaussianBlur(gray, (5, 5), 0),
            (256, 64),
            interpolation=cv2.INTER_AREA,
        )
        grays.append(comparison)
        sharpness.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))

    pair_differences = []
    for index in range(len(grays) - 1):
        difference = float(
            np.mean(cv2.absdiff(grays[index], grays[index + 1]))
        )
        pair_differences.append((difference, index, index + 1))
    best_difference, first, second = min(pair_differences)
    selected_index = max((first, second), key=lambda index: sharpness[index])
    candidate = images[selected_index].copy()
    if best_difference > OCR_FRAME_STABILITY_MAX_MAD:
        return None, best_difference
    return candidate, best_difference


def _camera_pixel_format_name(camera_format):
    pixel_format = camera_format.pixelFormat()
    return getattr(pixel_format, "name", str(pixel_format))


def _preferred_camera_format(device):
    """优先选择适合OCR且由驱动明确支持的1280×720、30 FPS格式。"""
    candidates = []
    for camera_format in device.videoFormats():
        resolution = camera_format.resolution()
        if (
            resolution.width() != OCR_PREFERRED_WIDTH
            or resolution.height() != OCR_PREFERRED_HEIGHT
            or float(camera_format.maxFrameRate()) < OCR_PREFERRED_FPS - 0.5
        ):
            continue
        pixel_name = _camera_pixel_format_name(camera_format)
        pixel_priority = {
            "Format_NV12": 0,
            "Format_Jpeg": 1,
        }.get(pixel_name, 2)
        candidates.append(
            (
                pixel_priority,
                abs(float(camera_format.maxFrameRate()) - OCR_PREFERRED_FPS),
                camera_format,
            )
        )
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[:2])[2]


def _camera_format_description(camera_format):
    if camera_format is None or camera_format.isNull():
        return "驱动自动选择"
    resolution = camera_format.resolution()
    return (
        f"{resolution.width()}×{resolution.height()} "
        f"{float(camera_format.maxFrameRate()):.0f} FPS "
        f"{_camera_pixel_format_name(camera_format).removeprefix('Format_')}"
    )


class OcrForceFilter:
    """先确认突变，再用中值和低通得到可供控制使用的g值。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self._history = collections.deque(maxlen=OCR_FILTER_WINDOW_SIZE)
        self._filtered = None
        self._pending_jump = []

    @staticmethod
    def _jump_values_agree(values):
        if len(values) < OCR_JUMP_CONFIRMATION_COUNT:
            return False
        center = statistics.median(values)
        tolerance = max(10.0, abs(center) * 0.05)
        return max(abs(value - center) for value in values) <= tolerance

    @staticmethod
    def _jump_values_form_monotonic_transition(baseline, values):
        """允许运动期间连续远离旧基准的真实压力变化通过二帧确认。"""
        if len(values) < OCR_JUMP_CONFIRMATION_COUNT:
            return False
        deltas = [float(value) - float(baseline) for value in values]
        direction = 1.0 if deltas[0] > 0.0 else -1.0
        if any(delta * direction <= 0.0 for delta in deltas):
            return False
        tolerance = max(10.0, abs(float(baseline)) * 0.05)
        return all(
            abs(current) + tolerance >= abs(previous)
            for previous, current in zip(deltas, deltas[1:])
        )

    def update(self, value):
        if value is None:
            return {
                "accepted": False,
                "filtered_force_g": self._filtered,
                "reason": "未识别到有效g数值",
            }
        value = float(value)
        if -OCR_NEGATIVE_ZERO_TOLERANCE_G <= value < 0.0:
            value = 0.0
        if not 0.0 <= value <= OCR_FORCE_MAX_G:
            return {
                "accepted": False,
                "filtered_force_g": self._filtered,
                "reason": f"超出0～{OCR_FORCE_MAX_G:.0f} g有效范围",
            }

        jump_confirmed = False
        if self._history:
            baseline = statistics.median(self._history)
            jump_limit = max(OCR_JUMP_MIN_G, abs(baseline) * OCR_JUMP_RATIO)
            if abs(value - baseline) > jump_limit:
                self._pending_jump.append(value)
                self._pending_jump = self._pending_jump[-OCR_JUMP_CONFIRMATION_COUNT:]
                values_agree = self._jump_values_agree(self._pending_jump)
                forms_transition = self._jump_values_form_monotonic_transition(
                    baseline,
                    self._pending_jump,
                )
                if not values_agree and not forms_transition:
                    return {
                        "accepted": False,
                        "filtered_force_g": self._filtered,
                        "reason": "明显跳变，等待下一帧确认",
                    }
                value = (
                    statistics.median(self._pending_jump)
                    if values_agree
                    else self._pending_jump[-1]
                )
                self._history.clear()
                self._pending_jump = []
                jump_confirmed = True
            else:
                self._pending_jump = []

        self._history.append(value)
        median_value = statistics.median(self._history)
        if self._filtered is None or jump_confirmed:
            self._filtered = median_value
        else:
            self._filtered = (
                OCR_FILTER_ALPHA * median_value
                + (1.0 - OCR_FILTER_ALPHA) * self._filtered
            )
        return {
            "accepted": True,
            "filtered_force_g": self._filtered,
            "reason": "",
        }


class CameraOcrWorker(QtCore.QThread):
    """在线程中执行PP-OCRv6双路识别，并维护Windows OCR备用进程。"""

    result_ready = Signal(object, str, str)
    failed = Signal(str)
    engine_ready = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._condition = threading.Condition()
        self._pending_frame = None
        self._stop_requested = False
        self._process = None
        self._rapid_engine = None

    def submit(self, frames):
        frames = tuple(
            QtMultimedia.QVideoFrame(frame)
            for frame in tuple(frames or ())
            if frame is not None and frame.isValid()
        )
        if not frames:
            return
        with self._condition:
            self._pending_frame = frames
            self._condition.notify()

    def stop(self):
        with self._condition:
            self._stop_requested = True
            self._pending_frame = None
            self._condition.notify_all()
            process = self._process
        if process is not None and process.poll() is None:
            process.kill()

    def _start_process(self):
        powershell = shutil.which("powershell.exe") or shutil.which("powershell")
        if powershell is None:
            raise RuntimeError("未找到Windows PowerShell")
        if not OCR_SCRIPT_PATH.is_file():
            raise RuntimeError(f"缺少OCR脚本：{OCR_SCRIPT_PATH.name}")
        creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        process = subprocess.Popen(
            [
                powershell,
                "-NoProfile",
                "-NonInteractive",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(OCR_SCRIPT_PATH),
                "-Server",
                "-Language",
                OCR_LANGUAGE,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            creationflags=creation_flags,
        )
        ready_line = process.stdout.readline().strip()
        if not ready_line.startswith("READY\t"):
            error = process.stderr.read().strip() if process.poll() is not None else ready_line
            process.kill()
            raise RuntimeError(error or "Windows OCR服务启动失败")
        return process, ready_line.split("\t", 1)[1]

    @staticmethod
    def _recognize_windows(process, image):
        if process is None:
            return ""
        if not image.save(str(OCR_IMAGE_PATH), "PNG"):
            raise RuntimeError(f"OCR临时截图保存失败：{OCR_IMAGE_PATH}")
        process.stdin.write(str(OCR_IMAGE_PATH) + "\n")
        process.stdin.flush()
        response = process.stdout.readline().rstrip("\r\n")
        if not response:
            raise RuntimeError("Windows OCR服务意外退出")
        status, _, encoded = response.partition("\t")
        try:
            message = base64.b64decode(encoded).decode("utf-8", errors="replace")
        except (ValueError, UnicodeError) as exc:
            raise RuntimeError(f"OCR响应格式错误：{response[:80]}") from exc
        if status != "OK":
            raise RuntimeError(message or "Windows OCR识别失败")
        return message

    @staticmethod
    def _recognize_rapid(engine, image_bgr):
        result = engine(
            image_bgr,
            use_det=False,
            use_cls=False,
            use_rec=True,
        )
        texts = tuple(result.txts or ())
        scores = tuple(result.scores or ())
        text = " ".join(str(item) for item in texts)
        score = min((float(item) for item in scores), default=0.0)
        return text, score

    def _recognize(self, process, image):
        engine = self._rapid_engine
        if engine is None:
            text = self._recognize_windows(process, image)
            return (
                _extract_fv_control_pressure(text),
                f"WindowsOCR：{text}",
                "Windows OCR备用识别",
            )

        try:
            image_bgr = _qimage_to_bgr(image)
            enhanced_bgr = _enhance_fv_for_ocr(image_bgr)
            original_text, original_score = self._recognize_rapid(engine, image_bgr)
            enhanced_text, enhanced_score = self._recognize_rapid(engine, enhanced_bgr)
        except Exception as exc:
            text = self._recognize_windows(process, image)
            return (
                _extract_fv_control_pressure(text),
                f"PP-OCRv6运行失败：{exc}\nWindowsOCR备用：{text}",
                "PP-OCRv6异常，已使用Windows OCR备用",
            )
        candidates = []
        for name, text, score in (
            ("原图", original_text, original_score),
            ("增强", enhanced_text, enhanced_score),
        ):
            complete_value = _extract_fv_control_pressure(text)
            value = (
                complete_value
                if complete_value is not None
                else _extract_fv_model_pressure(text)
            )
            if value is not None and score >= OCR_MODEL_MIN_SCORE:
                candidates.append(
                    (
                        name,
                        float(value),
                        float(score),
                        text,
                        complete_value is not None,
                    )
                )

        details = (
            f"PP-OCRv6原图[{original_score:.3f}]：{original_text or '（空）'}\n"
            f"PP-OCRv6增强[{enhanced_score:.3f}]：{enhanced_text or '（空）'}"
        )
        if len(candidates) == 2:
            difference = abs(candidates[0][1] - candidates[1][1])
            if difference <= OCR_MODEL_CONSENSUS_TOLERANCE_G:
                value = statistics.median(item[1] for item in candidates)
                if all(item[4] for item in candidates):
                    status = "PP-OCRv6原图/增强双路一致"
                else:
                    status = "PP-OCRv6双路一致（单位字符缺失，结构校验通过）"
                return value, details, status

        if len(candidates) == 1:
            name, value, score, _text, unit_complete = candidates[0]
            if unit_complete:
                return value, details, f"PP-OCRv6{name}高置信度单路"

        windows_text = self._recognize_windows(process, image)
        windows_value = _extract_fv_control_pressure(windows_text)
        details += f"\nWindowsOCR备用：{windows_text or '（空）'}"
        if windows_value is not None:
            for name, value, _score, _text, unit_complete in candidates:
                if (
                    unit_complete
                    and abs(float(windows_value) - value)
                    <= OCR_MODEL_CONSENSUS_TOLERANCE_G
                ):
                    return value, details, f"PP-OCRv6{name}/Windows备用一致"
            if not candidates:
                return float(windows_value), details, "Windows OCR备用识别"

        if len(candidates) == 2:
            reason = "PP-OCRv6原图与增强结果不一致"
        elif candidates and not candidates[0][4]:
            reason = "PP-OCRv6缺少g单位且未满足双路一致性校验"
        else:
            reason = "PP-OCRv6未得到高置信度完整fv两位小数g读数"
        return None, details, reason

    def run(self):
        try:
            process = None
            language = ""
            windows_error = ""
            try:
                process, language = self._start_process()
            except Exception as exc:
                windows_error = str(exc)
            with self._condition:
                self._process = process
            model_error = OCR_MODEL_IMPORT_ERROR or ""
            if RapidOCR is not None:
                try:
                    self._rapid_engine = RapidOCR(
                        params={
                            "Global.use_det": False,
                            "Global.use_cls": False,
                            "Global.use_rec": True,
                            "Global.log_level": "error",
                        }
                    )
                except Exception as exc:
                    model_error = str(exc)
            if self._rapid_engine is None and process is None:
                raise RuntimeError(
                    "PP-OCRv6与Windows OCR均不可用："
                    f"{model_error or '模型初始化失败'}；{windows_error}"
                )
            engines = ["PP-OCRv6 + OpenCV"] if self._rapid_engine is not None else []
            if process is not None:
                engines.append(f"Windows OCR备用({language})")
            self.engine_ready.emit(" / ".join(engines))
            while True:
                with self._condition:
                    while self._pending_frame is None and not self._stop_requested:
                        self._condition.wait()
                    if self._stop_requested:
                        break
                    frames = self._pending_frame
                    self._pending_frame = None
                cropped_images = []
                for frame in frames:
                    image = frame.toImage()
                    if image.isNull():
                        continue
                    cropped_images.append(_crop_fv_region(image))
                if not cropped_images:
                    raise RuntimeError("摄像头视频帧无法转换为OCR图片")
                image, stability_difference = _select_stable_fv_image(cropped_images)
                if image is None:
                    self.result_ready.emit(
                        None,
                        (
                            "连续裁剪图差异过大："
                            f"MAD={float(stability_difference):.2f}"
                        ),
                        "画面正在刷新，已跳过本批图像",
                    )
                    continue
                value, text, recognition_status = self._recognize(process, image)
                if stability_difference is not None:
                    recognition_status = (
                        f"{recognition_status}；多帧差异"
                        f"={float(stability_difference):.2f}"
                    )
                self.result_ready.emit(
                    value,
                    text,
                    recognition_status,
                )
        except Exception as exc:
            if not self._stop_requested:
                self.failed.emit(str(exc))
        finally:
            with self._condition:
                process = self._process
                self._process = None
                self._rapid_engine = None
            if process is not None and process.poll() is None:
                process.kill()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass


class CameraOcrWidget(QtWidgets.QWidget):
    """无预览地后台采集摄像头，并以100毫秒周期识别fv压力值。"""

    status_message = Signal(str)
    pressure_changed = Signal(object, str)
    force_sampled = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._devices = []
        self._camera = None
        self._capture_session = None
        self._video_sink = None
        self._latest_frame = None
        self._recent_frames = collections.deque(maxlen=OCR_FRAME_BATCH_SIZE)
        self._worker = None
        self._missed_count = 0
        self._no_frame_count = 0
        self._force_filter = OcrForceFilter()
        self._force_sequence = 0
        self._running = False
        self._active_camera_format_description = "驱动自动选择"
        self._dispatch_timer = QtCore.QTimer(self)
        self._dispatch_timer.setInterval(OCR_INTERVAL_MS)
        self._dispatch_timer.timeout.connect(self._dispatch_latest_frame)
        self._build_ui()
        self.refresh_cameras()
        QtCore.QTimer.singleShot(0, self.start_recognition)

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        dependency_label = QtWidgets.QLabel(
            f"摄像头按1280×720、30 FPS采集并缓存最近{OCR_FRAME_BATCH_SIZE}帧，"
            f"后台每{OCR_INTERVAL_MS} ms"
            "选择画面最稳定且清晰的一帧识别；固定机位下只截取fv读数行。"
            "PP-OCRv6对原图和OpenCV增强图执行双路识别，Windows OCR仅作备用；"
            "摄像头画面不会显示或弹出，识别失败时不保存裁剪图片。"
        )
        dependency_label.setWordWrap(True)
        dependency_label.setStyleSheet("color:#8bd5ff;")
        layout.addWidget(dependency_label)

        camera_row = QtWidgets.QHBoxLayout()
        camera_row.addWidget(QtWidgets.QLabel("摄像头"))
        self.camera_combo = QtWidgets.QComboBox()
        camera_row.addWidget(self.camera_combo, 1)
        self.refresh_button = QtWidgets.QPushButton("重新扫描")
        self.refresh_button.clicked.connect(self.refresh_cameras)
        camera_row.addWidget(self.refresh_button)
        layout.addLayout(camera_row)

        button_row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("启动后台识别")
        self.start_button.clicked.connect(self.start_recognition)
        button_row.addWidget(self.start_button)
        self.stop_button = QtWidgets.QPushButton("停止识别")
        self.stop_button.clicked.connect(self.stop_recognition)
        button_row.addWidget(self.stop_button)
        layout.addLayout(button_row)

        self.status_label = QtWidgets.QLabel("正在检查摄像头和OCR环境……")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addWidget(QtWidgets.QLabel("最近一次OCR原文（诊断用）"))
        self.result_edit = QtWidgets.QPlainTextEdit()
        self.result_edit.setReadOnly(True)
        self.result_edit.setPlaceholderText("这里不会显示摄像头画面，只显示OCR返回的文字。")
        self.result_edit.setMinimumHeight(130)
        layout.addWidget(self.result_edit, 1)

        dependency_detail = QtWidgets.QLabel(
            "运行依赖：PySide6/PyQt6 + OpenCV + RapidOCR + ONNX Runtime；"
            "模型随RapidOCR安装并在本机离线运行，Windows中文OCR语言包仅用于降级备用。"
        )
        dependency_detail.setWordWrap(True)
        layout.addWidget(dependency_detail)
        self._update_controls()

    def refresh_cameras(self):
        self.stop_recognition(quiet=True)
        self.camera_combo.clear()
        self._devices = []
        if QtMultimedia is None:
            message = MULTIMEDIA_IMPORT_ERROR or "Qt摄像头模块不可用"
            self._report_failure(message)
            self._update_controls()
            return
        self._devices = list(QtMultimedia.QMediaDevices.videoInputs())
        for device in self._devices:
            self.camera_combo.addItem(device.description())
        if self._devices:
            self.status_label.setText(f"发现 {len(self._devices)} 个摄像头，准备自动启动后台识别。")
        else:
            self._report_failure("未检测到摄像头，请检查USB连接和Windows相机权限。")
        self._update_controls()

    def start_recognition(self):
        if self._running:
            return
        index = self.camera_combo.currentIndex()
        if QtMultimedia is None or index < 0 or index >= len(self._devices):
            self._report_failure("未检测到摄像头，无法启动压力识别。")
            return
        self._latest_frame = None
        self._recent_frames.clear()
        self._missed_count = 0
        self._no_frame_count = 0
        self._force_filter.reset()
        self._worker = CameraOcrWorker(self)
        self._worker.result_ready.connect(self._on_ocr_result)
        self._worker.failed.connect(self._on_ocr_failure)
        self._worker.engine_ready.connect(self._on_engine_ready)
        self._worker.start()

        self._camera = QtMultimedia.QCamera(self._devices[index], self)
        preferred_format = _preferred_camera_format(self._devices[index])
        if preferred_format is not None:
            self._camera.setCameraFormat(preferred_format)
        self._active_camera_format_description = _camera_format_description(
            self._camera.cameraFormat()
        )
        self._capture_session = QtMultimedia.QMediaCaptureSession(self)
        self._video_sink = QtMultimedia.QVideoSink(self)
        self._video_sink.videoFrameChanged.connect(self._on_video_frame)
        self._capture_session.setCamera(self._camera)
        self._capture_session.setVideoSink(self._video_sink)
        self._camera.errorOccurred.connect(self._on_camera_error)
        self._camera.start()
        self._running = True
        self._dispatch_timer.start()
        self.status_label.setText(
            f"正在后台打开 {self._devices[index].description()}；不会显示摄像头画面。"
            f"采集格式={self._active_camera_format_description}。"
        )
        self.pressure_changed.emit(None, "摄像头启动中")
        self._update_controls()

    def stop_recognition(self, quiet=False):
        self._dispatch_timer.stop()
        self._running = False
        self._latest_frame = None
        self._recent_frames.clear()
        if self._camera is not None:
            self._camera.stop()
        if self._capture_session is not None:
            self._capture_session.setVideoSink(None)
            self._capture_session.setCamera(None)
        for item in (self._video_sink, self._capture_session, self._camera):
            if item is not None:
                item.deleteLater()
        self._video_sink = None
        self._capture_session = None
        self._camera = None
        if self._worker is not None:
            worker = self._worker
            self._worker = None
            worker.stop()
            worker.wait(2000)
            worker.deleteLater()
        if not quiet:
            self.status_label.setText("后台摄像头识别已停止。")
            self.pressure_changed.emit(None, "OCR已停止")
        self._update_controls()

    def _on_video_frame(self, frame):
        if not self._running or not frame.isValid():
            return
        self._latest_frame = QtMultimedia.QVideoFrame(frame)
        self._recent_frames.append(QtMultimedia.QVideoFrame(frame))
        self._no_frame_count = 0

    def _dispatch_latest_frame(self):
        if not self._running or self._worker is None:
            return
        if not self._recent_frames:
            self._no_frame_count += 1
            if self._no_frame_count == CAMERA_NO_FRAME_WARNING_COUNT:
                self._report_failure("摄像头已打开，但连续2秒没有收到画面。")
            return
        frames = tuple(self._recent_frames)
        self._recent_frames.clear()
        self._worker.submit(frames)

    @staticmethod
    def _crop_fv_region(image):
        return _crop_fv_region(image)

    def _on_engine_ready(self, engine_description):
        self.status_label.setText(
            f"后台识别运行中：每{OCR_INTERVAL_MS} ms处理最新一批图像；"
            f"采集={self._active_camera_format_description}；"
            f"识别引擎={engine_description}。"
        )

    def _on_ocr_result(self, value, text, recognition_status):
        self.result_edit.setPlainText(text or "（OCR未返回文字）")
        control_value = None if value is None else float(value)
        filtered = self._force_filter.update(control_value)
        if control_value is None:
            filtered["reason"] = recognition_status or "未识别到完整fv两位小数g读数"
        self._force_sequence += 1
        sample = {
            "sequence": self._force_sequence,
            "sampled_monotonic": time.monotonic(),
            "raw_text": str(text or ""),
            "raw_force_g": control_value,
            "control_raw_force_g": (
                None if control_value is None else float(control_value)
            ),
            "filtered_force_g": filtered["filtered_force_g"],
            "accepted": bool(filtered["accepted"]),
            "rejection_reason": filtered["reason"],
            "failure_image_path": "",
        }
        self.force_sampled.emit(sample)
        if not filtered["accepted"]:
            self._missed_count += 1
            if self._missed_count >= OCR_MISSED_WARNING_COUNT:
                reason = filtered["reason"] or "未识别到有效g数值"
                message = f"连续OCR无效：{reason}。请检查摄像头位置、反光和屏幕亮度。"
                self.status_label.setText(message)
                self.pressure_changed.emit(None, "未识别到fv压力")
            return
        self._missed_count = 0
        filtered_force = float(filtered["filtered_force_g"])
        self.status_label.setText(
            f"识别正常（{recognition_status}）：原始={float(control_value):.2f} g，"
            f"滤波={filtered_force:.2f} g；"
            f"每{OCR_INTERVAL_MS} ms取最新帧。"
        )
        self.pressure_changed.emit(filtered_force, "识别正常（已滤波）")

    def _on_ocr_failure(self, message):
        self._report_failure(f"OCR进程失败：{message}")

    def _on_camera_error(self, _error, error_string):
        self._report_failure(f"摄像头错误：{error_string}")

    def _report_failure(self, message):
        self.status_label.setText(message)
        self.pressure_changed.emit(None, message)
        self.status_message.emit(message)

    def _update_controls(self):
        available = bool(self._devices) and QtMultimedia is not None
        self.camera_combo.setEnabled(available and not self._running)
        self.refresh_button.setEnabled(not self._running)
        self.start_button.setEnabled(available and not self._running)
        self.stop_button.setEnabled(self._running)

    def shutdown(self):
        self.stop_recognition(quiet=True)

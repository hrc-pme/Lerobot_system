#!/usr/bin/env python3
"""
RoboPoint Visual Servoing GUI

This provides visual servoing GUI for D415 camera with LLM integration.
"""

import sys
import os
import subprocess
import time
import argparse
import json
import numpy as np
import requests
from PIL import Image, ImageDraw, ImageQt
import cv2

try:
    os.environ.pop('QT_PLUGIN_PATH', None)
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    for p in [
        '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms',
        '/usr/lib/x86_64-linux-gnu/qt/plugins/platforms',
        '/usr/lib/qt/plugins/platforms',
    ]:
        if os.path.isdir(p):
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = p
            root = os.path.dirname(p)
            os.environ['QT_PLUGIN_PATH'] = root
            break
except Exception:
    pass

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QComboBox, QSlider,
    QGroupBox, QSplitter, QSpinBox, QCheckBox, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QFont, QTextCursor, QImage

# Import RoboPoint modules: add repo root to sys.path
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if repo_root not in sys.path:
    sys.path.append(repo_root)
# Use local VLPoint package instead of RoboPoint
from point.conversation import default_conversation, conv_templates, SeparatorStyle
from point.utils import build_logger

from VLServo.moveit_marker_bridge import MoveItMarkerBridge

import zmq
import yaml
from yaml.loader import SafeLoader

logger = build_logger("pyqt_server", "pyqt_server.log")


class WhitePointGraspProcess:
    """Manage the white_point publisher process (local by default)."""
    def __init__(self):
        self.proc = None

    def start(self, *args, **kwargs):
        # Accept both positional and keyword for robustness across callers
        if self.proc and self.proc.poll() is None:
            return  # already running
        # Parse args
        if args and len(args) >= 2:
            x, y = int(args[0]), int(args[1])
            use_remote = bool(args[2]) if len(args) >= 3 else False
            radius_m = float(args[3]) if len(args) >= 4 else 0.0
            extra_push_m = float(args[4]) if len(args) >= 5 else None
        else:
            x = int(kwargs.get('x'))
            y = int(kwargs.get('y'))
            use_remote = bool(kwargs.get('use_remote', False))
            radius_m = float(kwargs.get('radius_m', 0.0))
            extra_push_m = kwargs.get('extra_push_m', None)
        cmd = [
            sys.executable, "-m",
            "VLServo.white_point",
            "-x", str(x), "-y", str(y),
        ]
        if radius_m and radius_m > 0:
            cmd += ["--radius-m", str(radius_m)]
        if extra_push_m is not None:
            try:
                epm = float(extra_push_m)
                if epm >= 0.0:
                    cmd += ["--extra-push-m", str(epm)]
            except Exception:
                pass
        if use_remote:
            cmd.append("-r")
        logger.info(f"Starting LLM (White-Point) Grasping: {' '.join(cmd)}")
        env = os.environ.copy()
        sep = os.pathsep
        existing = env.get('PYTHONPATH', '')
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(pkg_dir)
        env['PYTHONPATH'] = f"{parent_dir}{sep}{pkg_dir}{(sep + existing) if existing else ''}"
        env.setdefault('MPLBACKEND', 'Agg')
        env.setdefault('QT_QPA_PLATFORM', 'offscreen')
        self.proc = subprocess.Popen(cmd, cwd=pkg_dir, env=env)
        return self.proc

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
        self.proc = None


class CameraPublisherProcess:
    """Manage a standalone D415 ZMQ publisher (send_d415_images).

    This avoids the GUI owning the RealSense device. The GUI simply subscribes
    to the ZMQ stream for display, and head/controller nodes subscribe too.
    """
    def __init__(self):
        self.proc = None

    def start(self, exposure: str = 'low', use_remote_bind: bool = False):
        if self.proc and self.proc.poll() is None:
            return
        env = os.environ.copy()
        sep = os.pathsep
        existing = env.get('PYTHONPATH', '')
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(pkg_dir)
        env['PYTHONPATH'] = f"{parent_dir}{sep}{pkg_dir}{(sep + existing) if existing else ''}"
        cmd = [sys.executable, '-m', 'VLServo.send_d415_images', '-e', str(exposure)]
        if use_remote_bind:
            cmd.append('-r')
        logger.info(f"Starting D415 publisher: {' '.join(cmd)}")
        self.proc = subprocess.Popen(cmd, cwd=pkg_dir, env=env)
        return self.proc

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate(); self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
        self.proc = None


def pil_to_qimage(img: Image.Image) -> QImage:
    try:
        if hasattr(ImageQt, "toqimage"):
            qimg = ImageQt.toqimage(img)
            return qimg.copy()
    except Exception:
        pass
    pil_img = img
    if pil_img.mode not in ("RGB", "RGBA"):
        pil_img = pil_img.convert("RGB")
    if pil_img.mode == "RGBA":
        data = pil_img.tobytes("raw", "RGBA")
        qimg = QImage(data, pil_img.width, pil_img.height, pil_img.width * 4, QImage.Format_RGBA8888)
    else:
        data = pil_img.tobytes("raw", "RGB")
        qimg = QImage(data, pil_img.width, pil_img.height, pil_img.width * 3, QImage.Format_RGB888)
    return qimg.copy()


def qpixmap_from_pil(img: Image.Image) -> QPixmap:
    return QPixmap.fromImage(pil_to_qimage(img))


class ChatWidget(QTextEdit):
    """Chat area that can render text and inline images (Pillow -> QImage)."""
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setMinimumHeight(400)
        self.setFont(QFont("Arial", 14))

    def add_message(self, sender, content, is_image=False):
        cursor = self.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(f"\n{sender}:\n")
        if is_image and isinstance(content, tuple):
            text, image, _ = content
            cursor.insertText(f"{text}\n")
            if image is not None:
                qimage = pil_to_qimage(image)
                max_w = 600
                if qimage.width() > max_w:
                    qimage = qimage.scaled(max_w, int(qimage.height() * (max_w / qimage.width())), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                cursor.insertImage(qimage)
                cursor.insertText("\n")
        else:
            cursor.insertText(f"{content}\n")
        self.moveCursor(QTextCursor.End)


class ServerProcess:
    def __init__(self):
        self.controller_process = None
        self.model_worker_process = None
        self.controller_url = "http://127.0.0.1:11000"

    def start_controller(self, host="0.0.0.0", port=11000):
        cmd = [sys.executable, "-m", "point.serve.controller", "--host", host, "--port", str(port)]
        logger.info(f"Starting controller: {' '.join(cmd)}")
        self.controller_process = subprocess.Popen(cmd)
        self.controller_url = f"http://{('127.0.0.1' if host in ['0.0.0.0', '::'] else host)}:{port}"

    def start_model_worker(self, host="0.0.0.0", controller_url="http://127.0.0.1:11000",
                           port=22000, worker_url="http://127.0.0.1:22000",
                           model_path="wentao-yuan/robopoint-v1-vicuna-v1.5-13b", load_4bit=True):
        cmd = [
            sys.executable, "-m", "point.serve.model_worker",
            "--host", host,
            "--port", str(port),
            "--controller-address", controller_url,
            "--worker-address", worker_url,
            "--model-path", model_path
        ]
        if load_4bit:
            cmd.append("--load-4bit")
        logger.info(f"Starting model worker: {' '.join(cmd)}")
        self.model_worker_process = subprocess.Popen(cmd)

    def stop_all(self):
        if self.controller_process:
            self.controller_process.terminate()
            self.controller_process = None
        if self.model_worker_process:
            self.model_worker_process.terminate()
            self.model_worker_process = None


class ModelWorkerThread(QThread):
    response_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    finished_request = pyqtSignal()

    def __init__(self, controller_url="http://127.0.0.1:11000"):
        super().__init__()
        self.controller_url = controller_url
        self.request_data = None

    def set_request_data(self, data):
        self.request_data = data

    def run(self):
        try:
            ret = requests.post(self.controller_url + "/get_worker_address", json={"model": self.request_data["model"]})
            worker_addr = ret.json()["address"]
            if worker_addr == "":
                self.error_occurred.emit("No available worker")
                return
            response = requests.post(worker_addr + "/worker_generate_stream",
                                     headers={"User-Agent": "RoboPoint Client"},
                                     json=self.request_data, stream=True, timeout=30)
            for chunk in response.iter_lines(decode_unicode=False, delimiter=b"\0"):
                if not chunk:
                    continue
                data = json.loads(chunk.decode())
                if data.get("error_code", 1) == 0:
                    output = data["text"][len(self.request_data["prompt"]):].strip()
                    self.response_received.emit(output)
                else:
                    self.error_occurred.emit(f"Error: {data.get('text','')} (code: {data.get('error_code')})")
                    return
            self.finished_request.emit()
        except Exception as e:
            self.error_occurred.emit(f"Request failed: {str(e)}")


class ImageLabel(QLabel):
    # Add signal for mouse click events
    mouse_clicked = pyqtSignal(int, int)  # x, y coordinates
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 300)
        self.setStyleSheet("border: 2px dashed #aaa;")
        self.setAlignment(Qt.AlignCenter)
        self.setText("Waiting for camera frames…")
        self.setAcceptDrops(False)
        self.image_path = None
        self.original_image = None
        self.scale_factor = 1.0  # Track scaling for coordinate conversion
        # Track source frame dimensions for numpy-based frames
        self.source_w = None
        self.source_h = None

    def load_pil(self, pil_image):
        self.image_path = None
        self.original_image = pil_image
        # Set source dimensions for click mapping
        try:
            self.source_w, self.source_h = pil_image.width, pil_image.height
        except Exception:
            self.source_w, self.source_h = None, None
        pixmap = qpixmap_from_pil(self.original_image)
        scaled_pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # Calculate scale factor for coordinate conversion
        if self.original_image and scaled_pixmap.width() > 0 and scaled_pixmap.height() > 0:
            self.scale_factor = min(
                self.original_image.width / scaled_pixmap.width(),
                self.original_image.height / scaled_pixmap.height()
            )
        else:
            self.scale_factor = 1.0
            
        self.setPixmap(scaled_pixmap)

    def load_rgb_np(self, np_rgb):
        """Fast path: draw a numpy RGB array directly without PIL conversions."""
        try:
            if np_rgb is None:
                return
            h, w = np_rgb.shape[:2]
            # Update source dimensions for click mapping
            self.source_w, self.source_h = int(w), int(h)
            self.original_image = None  # Make it clear we are using numpy-based source
            # Ensure 3-channel uint8
            if np_rgb.dtype != np.uint8 or np_rgb.ndim != 3 or np_rgb.shape[2] != 3:
                return self.load_pil(Image.fromarray(np_rgb))
            bytes_per_line = 3 * w
            qimg = QImage(np_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            qimg = qimg.copy()  # detach from numpy buffer
            pixmap = QPixmap.fromImage(qimg)
            scaled_pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            # Track scale factor for click mapping similar to load_pil
            if scaled_pixmap.width() > 0 and scaled_pixmap.height() > 0:
                self.scale_factor = min(w / scaled_pixmap.width(), h / scaled_pixmap.height())
            else:
                self.scale_factor = 1.0
            self.setPixmap(scaled_pixmap)
        except Exception:
            # Fallback to PIL path on any error
            try:
                self.load_pil(Image.fromarray(np_rgb))
            except Exception:
                pass

    def mousePressEvent(self, event):
        """Handle mouse click events and convert to original image coordinates."""
        if event.button() == Qt.LeftButton:
            # Get click position relative to the widget
            click_pos = event.pos()
            
            # Get the actual pixmap position within the label
            pixmap = self.pixmap()
            if pixmap and (self.source_w is not None) and (self.source_h is not None):
                # Calculate the offset to center the pixmap in the label
                label_size = self.size()
                pixmap_size = pixmap.size()
                
                offset_x = (label_size.width() - pixmap_size.width()) // 2
                offset_y = (label_size.height() - pixmap_size.height()) // 2
                
                # Adjust click position relative to pixmap
                pixmap_x = click_pos.x() - offset_x
                pixmap_y = click_pos.y() - offset_y
                
                # Check if click is within pixmap bounds
                if (0 <= pixmap_x < pixmap_size.width() and 
                    0 <= pixmap_y < pixmap_size.height()):
                    
                    # Convert to original image coordinates
                    original_x = int(pixmap_x * self.scale_factor)
                    original_y = int(pixmap_y * self.scale_factor)
                    
                    # Clamp to image bounds
                    original_x = max(0, min(original_x, self.source_w - 1))
                    original_y = max(0, min(original_y, self.source_h - 1))
                    
                    # Emit the signal with original image coordinates
                    self.mouse_clicked.emit(original_x, original_y)
        
        super().mousePressEvent(event)


class SimplePointTracker:
    """Template-matching based point tracker to keep LLM white-point attached to the object.

    Initializes a template around the provided pixel and, on update, finds the
    best match within a local window to produce a new pixel location.
    """
    def __init__(self, template_size: int = 41, search_radius: int = 40, min_score: float = 0.4):
        self.template_size = max(11, int(template_size) | 1)
        self.search_radius = max(8, int(search_radius))
        self.min_score = float(min_score)
        self.template = None
        self.px = None
        self.py = None
        self.initialized = False

    def _extract_patch(self, gray, cx, cy, size):
        h, w = gray.shape[:2]
        half = size // 2
        x0 = max(0, int(cx) - half); x1 = min(w, int(cx) + half + 1)
        y0 = max(0, int(cy) - half); y1 = min(h, int(cy) + half + 1)
        patch = gray[y0:y1, x0:x1]
        if patch.shape[0] != size or patch.shape[1] != size:
            pad_t = size - patch.shape[0]
            pad_l = size - patch.shape[1]
            patch = cv2.copyMakeBorder(patch, 0, pad_t, 0, pad_l, cv2.BORDER_REPLICATE)
        return patch

    def initialize(self, color_image, px, py):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) if color_image.ndim == 3 else color_image
        self.template = self._extract_patch(gray, px, py, self.template_size)
        self.px, self.py = int(px), int(py)
        self.initialized = True

    def update(self, color_image):
        if not self.initialized:
            raise RuntimeError('Tracker not initialized')
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) if color_image.ndim == 3 else color_image
        h, w = gray.shape[:2]
        half_t = self.template_size // 2
        sr = self.search_radius
        x0 = max(0, self.px - sr - half_t); x1 = min(w, self.px + sr + half_t + 1)
        y0 = max(0, self.py - sr - half_t); y1 = min(h, self.py + sr + half_t + 1)
        search = gray[y0:y1, x0:x1]
        if search.shape[0] < self.template_size or search.shape[1] < self.template_size:
            return self.px, self.py
        res = cv2.matchTemplate(search, self.template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        # If quality too low, keep last location
        if max_val < self.min_score:
            return self.px, self.py
        cx = x0 + max_loc[0] + half_t
        cy = y0 + max_loc[1] + half_t
        self.px, self.py = int(cx), int(cy)
        return self.px, self.py

    def reset(self):
        self.template = None
        self.initialized = False
        self.px = None
        self.py = None


class CameraReceiverThread(QThread):
    frame_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(self, port, use_remote=False):
        super().__init__()
        self.port = port
        self.use_remote = use_remote
        self._running = True
        self._socket = None
        self._context = None

    def run(self):
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.SUB)
            self._socket.setsockopt(zmq.SUBSCRIBE, b'')
            self._socket.setsockopt(zmq.SNDHWM, 1)
            self._socket.setsockopt(zmq.RCVHWM, 1)
            self._socket.setsockopt(zmq.CONFLATE, 1)
            if self.use_remote:
                address = 'tcp://127.0.0.3:' + str(self.port)  # robot_ip
            else:
                address = 'tcp://127.0.0.1:' + str(self.port)
            self._socket.connect(address)
            self.status_changed.emit(f"Connected ({address})")
            while self._running:
                try:
                    out = self._socket.recv_pyobj(flags=0)
                except zmq.error.ZMQError:
                    break
                if not self._running:
                    break
                if isinstance(out, dict):
                    self.frame_received.emit(out)
        except Exception as e:
            self.status_changed.emit(f"Error: {e}")
        finally:
            try:
                if self._socket is not None:
                    self._socket.close(0)
                if self._context is not None:
                    self._context.term()
            except Exception:
                pass
            self.status_changed.emit("Disconnected")

    def stop(self):
        self._running = False
        try:
            if self._socket is not None:
                self._socket.close(0)
        except Exception:
            pass


class DirectCameraThread(QThread):
    frame_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(self, camera: str = 'D415', exposure: str = 'low'):
        super().__init__()
        self.camera = camera
        self.exposure = exposure
        self._running = True
        self._pipeline = None
        self._profile = None
        self._ctx = None
        self._pub = None
        self._pub_every_n = 3
        self._frame_idx = 0

    def stop(self):
        self._running = False

    def run(self):
        import numpy as np
        import time
        import zmq
        from VLServo import d415_helpers as dh

        port = 4415  # D415 camera port

        def _open_camera():
            pipeline, profile = dh.start_d415(self.exposure)
            self._pipeline, self._profile = pipeline, profile
            return pipeline, profile

        def _open_pub():
            try:
                if self._ctx is None:
                    self._ctx = zmq.Context()
                if self._pub is None:
                    self._pub = self._ctx.socket(zmq.PUB)
                    self._pub.setsockopt(zmq.SNDHWM, 1)
                    self._pub.setsockopt(zmq.RCVHWM, 1)
                    addr = 'tcp://' + '127.0.0.1' + ':' + str(port)
                    # Bind may fail if another publisher already bound; in that case, skip publishing
                    try:
                        self._pub.bind(addr)
                        self.status_changed.emit(f"Camera started ({self.camera}, exposure={self.exposure}).\nPublishing {addr}")
                    except Exception as e:
                        # Fallback: close pub to avoid misleading state
                        try:
                            self._pub.close(0)
                        except Exception:
                            pass
                        self._pub = None
                        self.status_changed.emit(f"Camera started ({self.camera}, exposure={self.exposure}).\nPublish disabled: {e}")
            except Exception as e:
                self.status_changed.emit(f"Camera started ({self.camera}, exposure={self.exposure}).\nPublish init error: {e}")

        # Initialize
        try:
            pipeline, profile = _open_camera()
            _open_pub()
        except Exception as e:
            self.status_changed.emit(f"Error opening camera: {e}")
            # Retry loop until running is cleared
            while self._running:
                time.sleep(0.5)
                try:
                    pipeline, profile = _open_camera()
                    _open_pub()
                    break
                except Exception:
                    continue

        first_frame = True
        depth_camera_info = None
        color_camera_info = None
        depth_scale = None

        while self._running:
            try:
                self._frame_idx += 1
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if (not depth_frame) or (not color_frame):
                    continue

                if first_frame:
                    depth_scale = dh.get_depth_scale(profile)
                    depth_camera_info = dh.get_camera_info(depth_frame)
                    color_camera_info = dh.get_camera_info(color_frame)
                    try:
                        if isinstance(depth_camera_info, dict):
                            depth_camera_info.pop('distortion_model', None)
                        if isinstance(color_camera_info, dict):
                            color_camera_info.pop('distortion_model', None)
                    except Exception:
                        pass
                    first_frame = False

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                out = {
                    'depth_camera_info': depth_camera_info,
                    'color_camera_info': color_camera_info,
                    'depth_scale': depth_scale,
                    'color_image': color_image,
                    'depth_image': depth_image,
                }
                self.frame_received.emit(out)
                try:
                    if self._pub is not None and (self._frame_idx % max(1, int(self._pub_every_n))) == 0:
                        self._pub.send_pyobj(out)
                except Exception:
                    pass
            except Exception as e:
                # Attempt a soft restart of the RealSense pipeline
                self.status_changed.emit(f"Error: {e}. Restarting camera…")
                try:
                    if self._pipeline is not None:
                        self._pipeline.stop()
                except Exception:
                    pass
                self._pipeline = None
                time.sleep(0.2)
                try:
                    pipeline, profile = _open_camera()
                    first_frame = True
                    continue
                except Exception as e2:
                    self.status_changed.emit(f"Restart failed: {e2}")
                    time.sleep(0.5)
                    continue

        # Shutdown
        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        except Exception:
            pass
        try:
            if self._pub is not None:
                self._pub.close(0)
            if self._ctx is not None:
                self._ctx.term()
        except Exception:
            pass
        self.status_changed.emit("Camera stopped")


class RoboPointMainWindow(QMainWindow):
    def __init__(self, controller_url=None, autostart=False, model_path="wentao-yuan/robopoint-v1-vicuna-v1.5-13b",
                 load_4bit=True, use_remote_stream=False):
        super().__init__()
        self.setWindowTitle("RoboPoint - Visual Servoing")
        self.setGeometry(100, 100, 1200, 800)

        self.server_process = ServerProcess()
        self.worker_thread = ModelWorkerThread()
        self.model_path = model_path
        self.load_4bit = load_4bit
        self.autostart = autostart
        self.conversation_state = default_conversation.copy()
        self.models = []
        self._pending_response_text = None

        # Camera state
        self.use_remote_stream = use_remote_stream
        self.camera_thread = None
        self.latest_camera_info = None
        self.latest_depth_scale = None
        self.latest_depth_image = None
        self.last_color_frame_bgr = None
        self.latest_llm_points = []  # list of (x,y) pixel coords from LLM annotations
        self.latest_llm_points_size = None  # (width, height) of image used for those coords
        self.llm_tracker = None
        self.llm_tracked_px = None
        self.llm_tracked_py = None
        self.llm_clusters = None
        self.llm_cluster_idx = -1
        self.user_clicked_position = False  # Flag to track if user manually clicked
        # Track whether an LLM grasping session is active (awaiting/using white point)
        self.llm_session_active = False
        # UI frame update limiter to reduce heavy redraw stutter
        self.ui_fps_limit = 15.0
        self._last_ui_frame_ts = 0.0
        self._last_cam_ts = 0.0
        self._frame_idx = 0

        # MoveIt marker integration
        self.moveit_bridge = None
        self.moveit_bridge_failed = False
        self.moveit_robot_name = os.getenv('MOVEIT_ROBOT_NAME', 'vx300s')
        self.moveit_planning_group = os.getenv('MOVEIT_PLANNING_GROUP', 'interbotix_arm')
        self.moveit_eef_link = os.getenv('MOVEIT_EE_LINK', f"{self.moveit_robot_name}/ee_gripper_link")
        self.moveit_target_frame = os.getenv('MOVEIT_TARGET_FRAME', 'world')
        self.moveit_camera_frame = os.getenv('MOVEIT_CAMERA_FRAME', 'camera_color_optical_frame')
        joint_env = os.getenv(
            'MOVEIT_JOINT_ORDER',
            'waist,shoulder,elbow,forearm_roll,wrist_angle,wrist_rotate'
        )
        self.moveit_joint_order = [j.strip() for j in joint_env.split(',') if j.strip()]
        if not self.moveit_joint_order:
            self.moveit_joint_order = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.moveit_has_cached_plan = False
        self.latest_target_xyz_camera = None
        self.latest_target_xyz_world = None
        self.latest_target_depth_m = None
        self.latest_target_stamp = 0.0

        # Direct camera capture thread (no external process)
        self.direct_camera_thread = None  # deprecated for GUI ownership; prefer external publisher

        # External processes
        # External process for LLM (white-point) grasping
        self.white_point_proc = WhitePointGraspProcess()
        # Standalone D415 publisher
        self.camera_pub_proc = CameraPublisherProcess()
        # Track current camera selection for display transforms
        # Default to D415 for camera stream
        self.current_camera = 'D415'

        # Import helpers
        from VLServo import d415_helpers_without_pyrealsense as dh
        self.dh = dh

        self.setup_ui()
        self.setup_connections()

        if controller_url:
            self.server_process.controller_url = controller_url
            self.worker_thread.controller_url = controller_url

        if autostart:
            self.start_services()
        else:
            QTimer.singleShot(1000, self.refresh_model_list)

        # If subscribing to a remote robot's ZMQ stream, start receiver now
        if self.use_remote_stream:
            self.start_camera_receiver()

        # Command publisher for live LLM grasp target updates
        try:
            self._llm_cmd_ctx = zmq.Context()
            self._llm_cmd_pub = self._llm_cmd_ctx.socket(zmq.PUB)
            self._llm_cmd_pub.setsockopt(zmq.SNDHWM, 1)
            addr = 'tcp://127.0.0.1:4444'  # LLM command port
            try:
                # Prefer binding; if already in use (another GUI), fall back to connect
                self._llm_cmd_pub.bind(addr)
            except Exception:
                try:
                    self._llm_cmd_pub.connect(addr)
                except Exception:
                    self._llm_cmd_pub = None
        except Exception:
            self._llm_cmd_ctx = None
            self._llm_cmd_pub = None

    def _publish_llm_target(self, px: int, py: int):
        try:
            if getattr(self, '_llm_cmd_pub', None) is None:
                return
            msg = {'target_px': int(px), 'target_py': int(py)}
            self._llm_cmd_pub.send_pyobj(msg)
        except Exception:
            pass

    def setup_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        left = QWidget(); left_layout = QVBoxLayout(left)
        # Model selection (keep widget for logic, hide in UI)
        model_group = QGroupBox("Model Selection")
        model_layout = QVBoxLayout(model_group)
        self.model_selector = QComboBox()
        model_layout.addWidget(self.model_selector)
        model_group.setVisible(False)
        left_layout.addWidget(model_group)

        # Camera stream group
        stream_group = QGroupBox("Camera Stream")
        stream_layout = QVBoxLayout(stream_group)
        self.image_label = ImageLabel()
        stream_layout.addWidget(self.image_label)
        # Do not show overlay toggle or camera status in UI
        try:
            self.overlay_white_dot_cb = QCheckBox("Overlay white dot")
            # Keep hidden but enabled so the white dot is visible after clicks
            self.overlay_white_dot_cb.setChecked(True)
            self.overlay_white_dot_cb.setVisible(False)
            # Not added to layout on purpose
        except Exception:
            self.overlay_white_dot_cb = None
        self.camera_status = QLabel("Status: Disconnected")
        self.camera_status.setVisible(False)
        # Not added to layout on purpose
        left_layout.addWidget(stream_group)

        # Parameters group (kept but hidden, so dependent code can read defaults)
        params_group = QGroupBox("Parameters")
        params_layout = QVBoxLayout(params_group)
        params_layout.addWidget(QLabel("Temperature:"))
        self.temperature_slider = QSlider(Qt.Horizontal); self.temperature_slider.setRange(0, 100); self.temperature_slider.setValue(100)
        self.temperature_label = QLabel("1.0")
        trow = QHBoxLayout(); trow.addWidget(self.temperature_slider); trow.addWidget(self.temperature_label)
        params_layout.addLayout(trow)
        params_layout.addWidget(QLabel("Top P:"))
        self.top_p_slider = QSlider(Qt.Horizontal); self.top_p_slider.setRange(0, 100); self.top_p_slider.setValue(70)
        self.top_p_label = QLabel("0.7")
        prow = QHBoxLayout(); prow.addWidget(self.top_p_slider); prow.addWidget(self.top_p_label)
        params_layout.addLayout(prow)
        params_layout.addWidget(QLabel("Max Output Tokens:"))
        self.max_tokens_spin = QSpinBox(); self.max_tokens_spin.setRange(0, 1024); self.max_tokens_spin.setValue(512)
        params_layout.addWidget(self.max_tokens_spin)
        left_layout.addWidget(params_group)
        params_group.setVisible(False)

        # Service status group (moved to bottom, single-line status)
        status_group = QGroupBox("Service Status")
        status_layout = QVBoxLayout(status_group)
        self.service_status = QLabel("Controller: Not Started | Model Worker: Not Started")
        status_layout.addWidget(self.service_status)
        # Prepare buttons but do not add to layout (no buttons requested)
        self.start_services_btn = QPushButton("Start Services"); self.stop_services_btn = QPushButton("Stop Services")

        # Camera control
        sender_group = QGroupBox("Camera", parent=left)
        sender_layout = QVBoxLayout(sender_group)
        # Always use D415 camera
        row1 = QHBoxLayout(); row1.addWidget(QLabel("Camera:"))
        self.camera_selector = QComboBox(parent=sender_group); self.camera_selector.addItems(["D415"]) ; row1.addWidget(self.camera_selector)
        # Explicitly hide the selector and do not add this row to the layout
        try:
            self.camera_selector.setVisible(False)
        except Exception:
            pass
        # Do not add row1 to layout so only exposure and start/stop remain visible
        row2 = QHBoxLayout(); row2.addWidget(QLabel("Exposure:"))
        self.exposure_selector = QComboBox(parent=sender_group); self.exposure_selector.addItems(["low", "medium", "auto"]) ; row2.addWidget(self.exposure_selector)
        sender_layout.addLayout(row2)
        btnrow = QHBoxLayout();
        self.btn_start_sender = QPushButton("Start Camera", parent=sender_group); self.btn_stop_sender = QPushButton("Stop Camera", parent=sender_group); self.btn_stop_sender.setEnabled(False)
        btnrow.addWidget(self.btn_start_sender); btnrow.addWidget(self.btn_stop_sender)
        sender_layout.addLayout(btnrow)
        # Show camera group in the GUI so users can control exposure and start/stop
        left_layout.addWidget(sender_group)
        sender_group.setVisible(True)

        # MoveIt controls with Plan and Grasping buttons
        llm_group = QGroupBox("MoveIt Control")
        llm_layout = QVBoxLayout(llm_group)
        
        # Plan button styled to match grasping button
        self.btn_start_llm = QPushButton("Plan")
        self.btn_start_llm.setStyleSheet(
            "QPushButton { font-size: 14px; font-weight: bold; padding: 10px; min-height: 44px; background-color: #1E88E5; color: white; }"
        )
        
        # New Grasping button
        self.btn_grasping = QPushButton("Grasping")
        self.btn_grasping.setEnabled(True)
        self.btn_grasping.setStyleSheet(
            "QPushButton { font-size: 14px; font-weight: bold; padding: 10px; min-height: 44px; background-color: #4CAF50; color: white; }"
        )
        self.default_grasp_forward_distance_m = 0.20  # Maintain 20 cm forward motion for grasping
        
        self.llm_status = QLabel("MoveIt: Waiting for target")
        llm_layout.addWidget(self.btn_start_llm)
        llm_layout.addWidget(self.btn_grasping)
        llm_layout.addWidget(self.llm_status)
        left_layout.addWidget(llm_group)

        left_layout.addStretch()
        # Place service status at very bottom
        left_layout.addWidget(status_group)

        # Right panel: chat
        right = QWidget(); right_layout = QVBoxLayout(right)
        self.chat_widget = ChatWidget()
        right_layout.addWidget(self.chat_widget)
        inrow = QHBoxLayout(); self.text_input = QLineEdit(); self.text_input.setPlaceholderText("Enter your question about the image...")
        self.text_input.setMinimumHeight(48)
        self.text_input.setFont(QFont("Arial", 16))
        self.text_input.setStyleSheet("QLineEdit { padding: 10px; font-size: 18px; }")
        self.send_button = QPushButton("Send"); self.send_button.setEnabled(False)
        self.send_button.setMinimumHeight(44)
        self.send_button.setStyleSheet("QPushButton { font-size: 14px; padding: 0 16px; }")
        inrow.addWidget(self.text_input); inrow.addWidget(self.send_button)
        right_layout.addLayout(inrow)
        actrow = QHBoxLayout(); self.clear_button = QPushButton("Clear Chat"); self.regenerate_button = QPushButton("Regenerate")
        actrow.addWidget(self.clear_button); actrow.addWidget(self.regenerate_button); actrow.addStretch(); right_layout.addLayout(actrow)

        splitter = QSplitter(Qt.Horizontal); splitter.addWidget(left); splitter.addWidget(right); splitter.setSizes([400, 800])
        main_layout.addWidget(splitter)

        # Enable manual service control by default; autostart remains off
        self.start_services_btn.setEnabled(True)
        self.stop_services_btn.setEnabled(False)

    def setup_connections(self):
        self.send_button.clicked.connect(self.send_message)
        self.text_input.returnPressed.connect(self.send_message)
        self.clear_button.clicked.connect(self.clear_chat)
        self.regenerate_button.clicked.connect(self.regenerate_response)
        self.start_services_btn.clicked.connect(self.start_services)
        self.stop_services_btn.clicked.connect(self.stop_services)
        self.temperature_slider.valueChanged.connect(self.update_temperature)
        self.top_p_slider.valueChanged.connect(self.update_top_p)
        self.btn_start_sender.clicked.connect(self.start_camera)
        self.btn_stop_sender.clicked.connect(self.stop_camera)
        self.camera_selector.currentTextChanged.connect(lambda _: self.on_camera_selection_changed())
        self.btn_start_llm.clicked.connect(self.plan_moveit_target)
        self.btn_grasping.clicked.connect(self.execute_grasping)

        # Image label mouse click connection
        self.image_label.mouse_clicked.connect(self.on_image_clicked)

        # Worker thread connections (streaming output)
        self.worker_thread.response_received.connect(self.handle_response)
        self.worker_thread.error_occurred.connect(self.handle_error)
        self.worker_thread.finished_request.connect(self.request_finished)

        # Background status timer
        self.status_timer = QTimer(); self.status_timer.timeout.connect(self.check_service_status); self.status_timer.start(5000)
        # Camera watchdog is only relevant in deprecated direct mode; keep disabled by default
        # self.cam_watchdog = QTimer(); self.cam_watchdog.timeout.connect(self.check_cam_health); self.cam_watchdog.start(1500)

    def start_camera_receiver(self):
        # Only subscribe when using a remote stream; otherwise, start local camera via buttons
        if not self.use_remote_stream:
            return
        if self.camera_thread is not None:
            return
        # Always subscribe to D415 port
        port = 4415  # D415 camera port
        self.camera_thread = CameraReceiverThread(port=port, use_remote=True)
        self.camera_thread.frame_received.connect(self.on_camera_frame)
        self.camera_thread.status_changed.connect(self.on_camera_status)
        self.camera_thread.start()

    def restart_camera_receiver(self):
        if self.camera_thread is not None:
            try:
                self.camera_thread.stop(); self.camera_thread.wait(500)
            except Exception:
                pass
            self.camera_thread = None
        self.start_camera_receiver()

    def on_camera_selection_changed(self):
        # If subscribing to remote stream, switch the SUB port
        try:
            self.current_camera = self.camera_selector.currentText()
        except Exception:
            pass
        if self.use_remote_stream:
            self.restart_camera_receiver()

    def _ensure_moveit_bridge(self) -> bool:
        if self.moveit_bridge is not None:
            return True
        if self.moveit_bridge_failed:
            return False
        try:
            self.moveit_bridge = MoveItMarkerBridge(
                robot_name=self.moveit_robot_name,
                planning_group=self.moveit_planning_group,
                eef_link=self.moveit_eef_link,
                joint_order=self.moveit_joint_order,
                camera_frame=self.moveit_camera_frame,
                target_frame=self.moveit_target_frame,
            )
            return True
        except Exception as exc:
            logger.error(f"Failed to initialize MoveIt bridge: {exc}")
            self.llm_status.setText(f"MoveIt: Bridge init failed ({exc})")
            self.moveit_bridge_failed = True
            return False

    def _latest_target_xyz(self):
        if self.latest_target_xyz_world is None:
            return None
        return tuple(float(v) for v in self.latest_target_xyz_world)

    @staticmethod
    def _camera_to_world(camera_vec):
        x_c, y_c, z_c = map(float, camera_vec)
        world_x = z_c
        world_y = -x_c
        world_z = -y_c
        return np.array([world_x, world_y, world_z], dtype=np.float32)

    @staticmethod
    def _format_whitepoint_axes(camera_vec):
        x_c, y_c, z_c = map(float, camera_vec)
        x_right_cm = x_c * 100.0
        y_up_cm = (-y_c) * 100.0
        z_forward_cm = z_c * 100.0
        return np.array([x_right_cm, y_up_cm, z_forward_cm], dtype=np.float32)

    def plan_moveit_target(self):
        try:
            if not self._ensure_moveit_bridge():
                return
            target_xyz = self._latest_target_xyz()
            if target_xyz is None:
                self.llm_status.setText("MoveIt: No depth target yet")
                return

            target_vec = np.array(target_xyz, dtype=np.float64)

            result = self.moveit_bridge.plan_to_xyz(
                target_vec.tolist(),
                source_frame=self.moveit_camera_frame,
                target_orientation=[0.0, 0.0, 0.0, 1.0],
            )
            if result.success:
                if self.latest_target_xyz_camera is not None:
                    disp = self._format_whitepoint_axes(self.latest_target_xyz_camera) / 100.0
                    self.llm_status.setText(
                        f"MoveIt: Plan ready @ right:{disp[0]:.3f}m up:{disp[1]:.3f}m forward:{disp[2]:.3f}m"
                    )
                else:
                    self.llm_status.setText(
                        f"MoveIt: Plan ready @ X:{target_vec[0]:.3f} Y:{target_vec[1]:.3f} Z:{target_vec[2]:.3f} m"
                    )
                self.moveit_has_cached_plan = True
            else:
                self.llm_status.setText(f"MoveIt: Plan failed ({result.message})")
                self.moveit_has_cached_plan = False
        except Exception as exc:
            logger.error(f"MoveIt plan error: {exc}")
            self.llm_status.setText(f"MoveIt: Plan error ({exc})")
            self.moveit_has_cached_plan = False

    def execute_grasping(self):
        """
        Execute grasping sequence:
        1. Open the gripper
        2. Move forward along world X-axis by the specified distance
        3. Close the gripper
        """
        try:
            if not self._ensure_moveit_bridge():
                self.llm_status.setText("Grasping: MoveIt bridge not ready")
                return
            
            self.llm_status.setText("Grasping: Step 1/3 - Opening gripper...")
            self.btn_grasping.setEnabled(False)
            
            # Step 1: Open the gripper
            result = self.moveit_bridge.open_gripper(delay=2.0)
            if not result.success:
                self.llm_status.setText(f"Grasping: Failed to open gripper - {result.message}")
                self.btn_grasping.setEnabled(True)
                logger.error(f"Failed to open gripper: {result.message}")
                return
            
            logger.info("Gripper opened successfully")
            
            # Step 2: Move forward along world X-axis
            distance_m = float(self.default_grasp_forward_distance_m)
            direction_label = "world X-axis"
            
            self.llm_status.setText(f"Grasping: Step 2/3 - Moving {distance_m*100:.1f}cm along world X-axis...")
            logger.info(f"Moving {distance_m}m along world X-axis...")
            result = self.moveit_bridge.move_relative_x(distance_m)
            
            if not result.success:
                self.llm_status.setText(f"Grasping: Failed to move forward ({direction_label}) - {result.message}")
                self.btn_grasping.setEnabled(True)
                logger.error(f"Failed to move forward along {direction_label}: {result.message}")
                return
            
            logger.info(f"Moved {distance_m}m along {direction_label} successfully")
            
            # Wait a moment for the motion to settle
            self.llm_status.setText("Grasping: Step 3/3 - Closing gripper...")
            time.sleep(0.5)
            
            # Step 3: Close the gripper
            result = self.moveit_bridge.close_gripper(delay=2.0)
            if not result.success:
                self.llm_status.setText(f"Grasping: Failed to close gripper - {result.message}")
                self.btn_grasping.setEnabled(True)
                logger.error(f"Failed to close gripper: {result.message}")
                return
            
            logger.info("Gripper closed successfully")
            self.llm_status.setText("Grasping: ✓ Complete! Object grasped successfully.")
            self.btn_grasping.setEnabled(True)
            
        except Exception as exc:
            import traceback
            traceback.print_exc()
            logger.error(f"Grasping error: {exc}")
            self.llm_status.setText(f"Grasping: Error - {exc}")
            self.btn_grasping.setEnabled(True)

    def closeEvent(self, event):
        try:
            if self.moveit_bridge is not None:
                self.moveit_bridge.destroy()
        except Exception:
            pass
        super().closeEvent(event)

    def on_camera_status(self, text):
        self.camera_status.setText(f"Status: {text}")

    def on_camera_frame(self, output):
        try:
            # Throttle UI redraws to reduce choppiness from heavy image conversions
            try:
                now_ts = time.time()
                min_dt = 1.0 / max(1.0, float(self.ui_fps_limit))
                if (now_ts - self._last_ui_frame_ts) < min_dt:
                    return
            except Exception:
                pass
            # Mark last camera frame time for watchdog
            try:
                self._last_cam_ts = time.time()
            except Exception:
                pass
            self._frame_idx += 1
            color_image = output.get('color_image', None)
            depth_image = output.get('depth_image', None)
            if color_image is None:
                return
            # cache BGR frame for auto-detection on LLM start
            try:
                self.last_color_frame_bgr = color_image
            except Exception:
                pass
            self.latest_camera_info = output.get('depth_camera_info', output.get('color_camera_info', None))
            self.latest_depth_scale = output.get('depth_scale', None)
            self.latest_depth_image = depth_image
            if isinstance(color_image, np.ndarray) and color_image.ndim == 3 and color_image.shape[2] == 3:
                import cv2
                rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            else:
                rgb = color_image if isinstance(color_image, np.ndarray) else np.array(color_image)
            vis = np.copy(rgb)
            h0, w0 = vis.shape[:2]
            
            # Overlay LLM-selected point with tracking to stick to the object
            # Draw the white dot even if depth info isn't available yet; add text only when depth exists.
            have_depth = (self.latest_depth_image is not None) and (self.latest_camera_info is not None) and (self.latest_depth_scale is not None)
            overlay_enabled = (self.overlay_white_dot_cb.isChecked() if getattr(self, 'overlay_white_dot_cb', None) is not None else True)
            # Show if we have any candidate: either parsed LLM crosses or a user/LLM tracked pixel
            show_white_point = overlay_enabled and (bool(self.latest_llm_points) or (self.llm_tracked_px is not None and self.llm_tracked_py is not None))
            
            if show_white_point:
                try:
                    import cv2
                    h, w = vis.shape[:2]
                    
                    # Initialize tracker on first use if we have LLM points and user hasn't manually clicked
                    if (self.latest_llm_points and not self.user_clicked_position and 
                        (self.llm_tracker is None or (self.llm_tracked_px is None))):
                        cx0, cy0 = w // 2, h // 2
                        best = None
                        for (px0, py0) in self.latest_llm_points:
                            d0 = float(np.hypot(px0 - cx0, py0 - cy0))
                            if (best is None) or (d0 < best[0]):
                                best = (d0, px0, py0)
                        if best is not None:
                            self.llm_tracked_px, self.llm_tracked_py = int(best[1]), int(best[2])
                            self.llm_tracker = SimplePointTracker(template_size=41, search_radius=40)
                            self.llm_tracker.initialize(color_image, self.llm_tracked_px, self.llm_tracked_py)
                    
                    # If we have a tracked point but no tracker initialized, initialize it
                    elif (self.llm_tracked_px is not None and self.llm_tracked_py is not None and 
                          (self.llm_tracker is None or not self.llm_tracker.initialized)):
                        self.llm_tracker = SimplePointTracker(template_size=41, search_radius=40)
                        self.llm_tracker.initialize(color_image, self.llm_tracked_px, self.llm_tracked_py)
                    
                    # Update tracker location if we have one
                    if self.llm_tracker is not None and self.llm_tracker.initialized:
                        self.llm_tracked_px, self.llm_tracked_py = self.llm_tracker.update(color_image)
                        
                    # Use current tracked position
                    if self.llm_tracked_px is not None and self.llm_tracked_py is not None:
                        px, py = int(self.llm_tracked_px), int(self.llm_tracked_py)
                        # Snap off pure white/black to nearest valid pixel
                        try:
                            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
                            px_adj, py_adj = self._nearest_non_white_black(hsv, px, py, radius=5)
                            px, py = px_adj, py_adj
                            self.llm_tracked_px, self.llm_tracked_py = px, py
                        except Exception:
                            pass
                        # Robust depth (optional): compute only when depth info exists, and decimate to reduce cost
                        depth_m = None
                        if have_depth and (self._frame_idx % 2 == 0):
                            k = 3
                            y0 = max(0, py - k); y1 = min(self.latest_depth_image.shape[0], py + k + 1)
                            x0 = max(0, px - k); x1 = min(self.latest_depth_image.shape[1], px + k + 1)
                            patch = self.latest_depth_image[y0:y1, x0:x1]
                            if patch is not None and patch.size > 0:
                                vals = patch.reshape(-1)
                                if np.issubdtype(vals.dtype, np.floating):
                                    vals = vals[np.isfinite(vals)]
                                    vals_m = vals
                                else:
                                    vals_m = vals.astype(np.float32) * float(self.latest_depth_scale)
                                vals_m = vals_m[vals_m > 0]
                                if vals_m.size > 0:
                                    depth_m = float(np.median(vals_m))
                        # Draw overlay
                        # Always draw a fast white dot overlay
                        cv2.circle(vis, (px, py), 6, (255, 255, 255), -1, lineType=cv2.LINE_AA)
                        line = None
                        if (depth_m is not None) and (depth_m > 0):
                            center_xyz = self.dh.pixel_to_3d(np.array([px, py], dtype=np.float32), depth_m, self.latest_camera_info)
                            if np.all(np.isfinite(center_xyz)):
                                norm_m = float(np.linalg.norm(center_xyz))
                                if norm_m < 5.0:
                                    self.latest_target_xyz_camera = np.array(center_xyz, dtype=np.float32)
                                    self.latest_target_xyz_world = self._camera_to_world(center_xyz)
                                    self.latest_target_depth_m = float(depth_m)
                                    self.latest_target_stamp = time.time()
                                    display_vals = self._format_whitepoint_axes(center_xyz)
                                    line = f"{display_vals[0]:.1f}, {display_vals[1]:.1f}, {display_vals[2]:.1f} cm"
                                else:
                                    self.latest_target_xyz_camera = None
                                    self.latest_target_xyz_world = None
                            else:
                                self.latest_target_xyz_camera = None
                                self.latest_target_xyz_world = None
                        elif self.latest_target_xyz_camera is not None:
                            display_vals = self._format_whitepoint_axes(self.latest_target_xyz_camera)
                            line = f"{display_vals[0]:.1f}, {display_vals[1]:.1f}, {display_vals[2]:.1f} cm"

                        if self.latest_target_xyz_camera is None:
                            self.latest_target_xyz_world = None

                        if line is not None:
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            font_scale = 0.6
                            t_fg, t_bg = 1, 4
                            (tw, th), _ = cv2.getTextSize(line, font, font_scale, t_fg)
                            tx = int(px - tw / 2); ty = int(py - 55)
                            cv2.putText(vis, line, (tx, ty), font, font_scale, (0, 0, 0), t_bg, cv2.LINE_AA)
                            cv2.putText(vis, line, (tx, ty), font, font_scale, (255, 255, 255), t_fg, cv2.LINE_AA)

                except Exception:
                    pass
            # Draw without PIL to keep GUI smooth
            self.image_label.load_rgb_np(vis)
            try:
                self._last_ui_frame_ts = now_ts
            except Exception:
                pass
        except Exception as e:
            logger.error(f"Failed to handle camera frame: {e}")

    def _auto_detect_white_points(self, bgr_image, max_points=1):
        """Detect bright low-saturation regions as white points.

        Returns list of (x,y) pixel coordinates in image space.
        """
        try:
            if bgr_image is None:
                return []
            img = bgr_image
            h, w = img.shape[:2]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Low saturation, high value
            lower = np.array([0, 0, 200], dtype=np.uint8)
            upper = np.array([180, 40, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return []
            cx, cy = w // 2, h // 2
            cands = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 20:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                d = float(np.hypot(x - cx, y - cy))
                cands.append((area, -d, x, y))
            cands.sort(reverse=True)
            pts = [(x, y) for (_, _, x, y) in cands[:max_points]]
            return pts
        except Exception:
            return []

    def _select_point_from_crosses(self, bgr_image, crosses):
        """Given a set of LLM crosses, choose a good interior grasp pixel.

        Strategy:
        - Estimate dominant HSV color near the crosses.
        - Build a color mask around that dominant color and restrict it by the
          convex hull of the crosses to focus on the intended object.
        - Use distance transform to find the most interior pixel (far from edges).
        - Fallback to the mean of crosses if mask is empty.
        """
        try:
            if bgr_image is None or not crosses:
                return None
            img = bgr_image
            h, w = img.shape[:2]
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            H, S, V = cv2.split(hsv)
            # Sample small patches around each cross to collect HSV samples
            hs, ss, vs = [], [], []
            k = 4
            for (x, y) in crosses:
                xi0 = max(0, int(x) - k); xi1 = min(w, int(x) + k + 1)
                yi0 = max(0, int(y) - k); yi1 = min(h, int(y) + k + 1)
                if (xi1 - xi0) <= 0 or (yi1 - yi0) <= 0:
                    continue
                hs.append(H[yi0:yi1, xi0:xi1].reshape(-1))
                ss.append(S[yi0:yi1, xi0:xi1].reshape(-1))
                vs.append(V[yi0:yi1, xi0:xi1].reshape(-1))
            if not hs:
                # fallback: average crosses
                mx = int(np.mean([p[0] for p in crosses])); my = int(np.mean([p[1] for p in crosses]))
                return (mx, my)
            hs = np.concatenate(hs); ss = np.concatenate(ss); vs = np.concatenate(vs)
            # Robust central tendency
            h_med = int(np.median(hs)); s_med = int(np.median(ss)); v_med = int(np.median(vs))
            # Build color threshold around median. Allowable spreads
            dh = 15
            ds = 60
            dv = 60
            lower = np.array([max(0, h_med - dh), max(0, s_med - ds), max(0, v_med - dv)], dtype=np.uint8)
            upper = np.array([min(180, h_med + dh), min(255, s_med + ds), min(255, v_med + dv)], dtype=np.uint8)
            color_mask = cv2.inRange(hsv, lower, upper)
            # Build convex hull mask from crosses
            hull = None
            pts = np.array([[int(x), int(y)] for (x, y) in crosses], dtype=np.int32)
            if len(pts) >= 3:
                hull = cv2.convexHull(pts)
            else:
                hull = pts
            hull_mask = np.zeros((h, w), dtype=np.uint8)
            if hull is not None and len(hull) >= 3:
                cv2.fillConvexPoly(hull_mask, hull, 255, lineType=cv2.LINE_AA)
            elif len(hull) == 2:
                cv2.line(hull_mask, tuple(hull[0]), tuple(hull[1]), 255, thickness=9, lineType=cv2.LINE_AA)
            elif len(hull) == 1:
                cv2.circle(hull_mask, tuple(hull[0][0] if hull.ndim == 3 else hull[0]), 6, 255, -1, lineType=cv2.LINE_AA)
            # Combine masks and clean up
            mask = cv2.bitwise_and(color_mask, hull_mask)
            # Remove likely white background and black pixels
            # Treat near-white/grey as background too: low S, moderately high V
            white_lower = np.array([0, 0, 200], dtype=np.uint8)
            white_upper = np.array([180, 40, 255], dtype=np.uint8)
            lightgray_lower = np.array([0, 0, 160], dtype=np.uint8)
            lightgray_upper = np.array([180, 60, 255], dtype=np.uint8)
            black_lower = np.array([0, 0, 0], dtype=np.uint8)
            black_upper = np.array([180, 255, 40], dtype=np.uint8)
            white_mask_strict = cv2.inRange(hsv, white_lower, white_upper)
            white_mask_light = cv2.inRange(hsv, lightgray_lower, lightgray_upper)
            white_mask = cv2.bitwise_or(white_mask_strict, white_mask_light)
            black_mask = cv2.inRange(hsv, black_lower, black_upper)
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(white_mask))
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(black_mask))
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            # Further erode to avoid edges when selecting interior
            mask = cv2.erode(mask, np.ones((7, 7), np.uint8), iterations=1)
            if cv2.countNonZero(mask) == 0:
                # fallback: mean of crosses
                mx = int(np.mean([p[0] for p in crosses])); my = int(np.mean([p[1] for p in crosses]))
                return (mx, my)
            # Combine cross density with interior distance for robust interior selection
            dist = cv2.distanceTransform((mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
            den = np.zeros((h, w), dtype=np.float32)
            for (x, y) in crosses:
                xi = int(np.clip(x, 0, w - 1)); yi = int(np.clip(y, 0, h - 1))
                den[yi, xi] += 1.0
            den = cv2.GaussianBlur(den, (0, 0), sigmaX=6, sigmaY=6)
            if den.max() > 0:
                den = den / den.max()
            if dist.max() > 0:
                dist = dist / dist.max()
            score = den * dist * ((mask > 0).astype(np.float32))
            minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(score)
            cx, cy = int(maxLoc[0]), int(maxLoc[1])
            return (cx, cy)
        except Exception:
            return None

    def _nearest_non_white_black(self, hsv_image, px, py, radius=5):
        """Find nearest pixel around (px,py) that is not white background nor black.

        If none found, returns the input (px,py).
        """
        h, w = hsv_image.shape[:2]
        px = int(np.clip(px, 0, w - 1)); py = int(np.clip(py, 0, h - 1))
        # Define simple white/black masks thresholds
        H, S, V = cv2.split(hsv_image)
        # Pixel-wise quick check function
        def is_bad(x, y):
            s = int(S[y, x]); v = int(V[y, x])
            # Exclude white and light-gray (low S, mid-high V), and very dark (black)
            return ((s <= 60 and v >= 160) or (v <= 40))
        if not is_bad(px, py):
            return px, py
        # Search in growing window
        for r in range(1, int(radius) + 1):
            x0 = max(0, px - r); x1 = min(w - 1, px + r)
            y0 = max(0, py - r); y1 = min(h - 1, py + r)
            # Check perimeter of the square window to keep it cheap
            for x in range(x0, x1 + 1):
                if not is_bad(x, y0):
                    return x, y0
                if not is_bad(x, y1):
                    return x, y1
            for y in range(y0, y1 + 1):
                if not is_bad(x0, y):
                    return x0, y
                if not is_bad(x1, y):
                    return x1, y
        return px, py

    def _choose_point_from_crosses_by_density(self, crosses, radius=8):
        """Pick a point where RoboPoint crosses are densest.

        - If exact duplicates exist, choose the most frequent coordinate.
        - Otherwise, for each point, count neighbors within `radius` pixels and
          select the cluster with the highest count; return its centroid.
        """
        try:
            if not crosses:
                return None
            pts = [(int(x), int(y)) for (x, y) in crosses]
            # 1) Exact duplicates (mode)
            from collections import Counter
            cnt = Counter(pts)
            ((mx, my), maxc) = max(cnt.items(), key=lambda kv: kv[1])
            if maxc >= 2:
                return (mx, my)
            # 2) Radius-based density
            pts_np = np.array(pts, dtype=np.int32)
            best_idx = -1
            best_count = -1
            best_subset = None
            r2 = radius * radius
            for i in range(len(pts_np)):
                dx = pts_np[:, 0] - pts_np[i, 0]
                dy = pts_np[:, 1] - pts_np[i, 1]
                mask = (dx * dx + dy * dy) <= r2
                count = int(np.count_nonzero(mask))
                if count > best_count:
                    best_count = count
                    best_idx = i
                    best_subset = pts_np[mask]
            if best_subset is not None and best_count > 0:
                cx = int(round(np.mean(best_subset[:, 0])))
                cy = int(round(np.mean(best_subset[:, 1])))
                return (cx, cy)
            return pts[0]
        except Exception:
            return None

    def _cluster_crosses(self, crosses):
        """Group crosses into clusters using KMeans based on spatial proximity."""
        import cv2
        if not crosses:
            return [[]]
        
        pts = np.array([[int(x), int(y)] for (x, y) in crosses], dtype=np.float32)
        n = len(pts)
        K = 1
        if n >= 24:
            K = 3
        elif n >= 10:
            K = 2
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
        flags = cv2.KMEANS_PP_CENTERS
        compactness, labels, centers = cv2.kmeans(pts, K, None, criteria, 3, flags)
        
        clusters = []
        for k in range(K):
            mask = (labels.ravel() == k)
            sub = pts[mask].astype(np.int32)
            if len(sub) > 0:
                clusters.append([(int(x), int(y)) for (x, y) in sub])
        
        # Sort by cluster size, larger clusters first
        clusters.sort(key=lambda c: -len(c))
        return clusters

    def clean_llm_points(self):
        """Clear annotated white points and reset tracking without popups."""
        try:
            # Clear all LLM-derived points and related state
            self.latest_llm_points = []
            self.latest_llm_points_size = None
            self.llm_clusters = None
            self.llm_cluster_idx = -1

            # Reset tracking state
            self.llm_tracked_px = None
            self.llm_tracked_py = None
            self.llm_tracker = None
            self.user_clicked_position = False
            self.latest_target_xyz_camera = None
            self.latest_target_xyz_world = None

            # Stop any running white point process
            running = (self.white_point_proc.proc is not None) and (self.white_point_proc.proc.poll() is None)
            if running:
                try:
                    self.white_point_proc.stop()
                    self.llm_status.setText("MoveIt: Target cleared")
                except Exception:
                    pass
            else:
                self.llm_status.setText("MoveIt: Target cleared")

            # Update button states if needed
            self.btn_start_llm.setEnabled(True)
        except Exception as e:
            logger.error(f"Failed to clean points: {e}")

    def on_image_clicked(self, x, y):
        """Handle mouse clicks on the image to set white point position."""
        try:
            # Coordinates from ImageLabel are in the displayed image space.
            px, py = x, y
            
            # Check if we have valid camera frame dimensions for coordinate validation
            if hasattr(self, 'last_color_frame_bgr') and self.last_color_frame_bgr is not None:
                h, w = self.last_color_frame_bgr.shape[:2]
                # Clamp coordinates to camera frame bounds
                px = max(0, min(px, w - 1))
                py = max(0, min(py, h - 1))
            
            # Avoid white/gray/black pixels if possible
            if hasattr(self, 'last_color_frame_bgr') and self.last_color_frame_bgr is not None:
                hsv = cv2.cvtColor(self.last_color_frame_bgr, cv2.COLOR_BGR2HSV)
                px, py = self._nearest_non_white_black(hsv, px, py, radius=6)
            
            # Update tracking target
            self.llm_tracked_px, self.llm_tracked_py = int(px), int(py)
            self.llm_tracker = None  # Reset tracker to re-initialize with new point
            self.user_clicked_position = True  # Mark that user manually set the position
            
            self.moveit_has_cached_plan = False
            self.llm_status.setText("MoveIt: Target selected")
                
        except Exception as e:
            logger.error(f"Failed to handle image click: {e}")

    def start_camera(self):
        # Stop remote subscriber if running locally
        if not self.use_remote_stream and self.camera_thread is not None:
            try:
                self.camera_thread.stop(); self.camera_thread.wait(500)
            except Exception:
                pass
            self.camera_thread = None

        # Start direct RealSense capture (force D415)
        try:
            cam = 'D415'
            exposure = self.exposure_selector.currentText()
            # Update current camera for display orientation handling
            self.current_camera = cam
            if self.direct_camera_thread is not None:
                return
            self.direct_camera_thread = DirectCameraThread(camera=cam, exposure=exposure)
            self.direct_camera_thread.frame_received.connect(self.on_camera_frame)
            self.direct_camera_thread.status_changed.connect(self.on_camera_status)
            self.direct_camera_thread.start()
            self.btn_start_sender.setEnabled(False); self.btn_stop_sender.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Camera", f"Failed to start: {e}")

    def stop_camera(self):
        try:
            if self.direct_camera_thread is not None:
                self.direct_camera_thread.stop(); self.direct_camera_thread.wait(1000)
        finally:
            self.direct_camera_thread = None
            self.btn_start_sender.setEnabled(True); self.btn_stop_sender.setEnabled(False)

    def ensure_local_camera(self):
        """Ensure the local D415 capture thread is running when using local stream."""
        try:
            if self.use_remote_stream:
                return
            # If thread missing or not running, attempt restart
            need_restart = (self.direct_camera_thread is None)
            try:
                if (self.direct_camera_thread is not None) and (not self.direct_camera_thread.isRunning()):
                    need_restart = True
            except Exception:
                pass
            if need_restart:
                self.start_camera()
        except Exception:
            pass

    def check_cam_health(self):
        """Watchdog: if local camera frames stall, restart the direct capture thread."""
        try:
            if self.use_remote_stream:
                return
            now = time.time()
            last = float(self._last_cam_ts or 0.0)
            if (now - last) > 2.5:
                # Hard restart to recover a hung RealSense pipeline
                try:
                    self.stop_camera()
                except Exception:
                    pass
                self.start_camera()
        except Exception:
            pass

    def start_services(self):
        try:
            self.service_status.setText("Controller: Starting... | Model Worker: Starting...")
            self.server_process.start_controller()
            time.sleep(3)
            self.server_process.start_model_worker(controller_url=self.server_process.controller_url,
                                                   model_path=self.model_path,
                                                   load_4bit=self.load_4bit)
            self.start_services_btn.setEnabled(False); self.stop_services_btn.setEnabled(True)
            QTimer.singleShot(10000, self.refresh_model_list)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start services: {e}")

    def stop_services(self):
        self.server_process.stop_all()
        self.service_status.setText("Controller: Stopped | Model Worker: Stopped")
        self.start_services_btn.setEnabled(True)
        self.stop_services_btn.setEnabled(False)
        self.send_button.setEnabled(False)

    def check_service_status(self):
        """Poll lightweight status with very short timeouts to avoid UI stalls."""
        ctrl_status = "Not Running"
        try:
            response = requests.post(f"{self.server_process.controller_url}/list_models", timeout=0.2)
            if response.status_code == 200:
                ctrl_status = "Running"
            else:
                ctrl_status = "Error"
        except Exception:
            if self.server_process.controller_process and self.server_process.controller_process.poll() is None:
                ctrl_status = "Starting..."
            else:
                ctrl_status = "Not Running"
        worker_status = "Running" if len(self.models) > 0 else "No Models"
        try:
            self.service_status.setText(f"Controller: {ctrl_status} | Model Worker: {worker_status}")
        except Exception:
            pass
        self.send_button.setEnabled((ctrl_status == "Running") and len(self.models) > 0)

    def refresh_model_list(self):
        try:
            base = self.server_process.controller_url
            ret = requests.post(f"{base}/refresh_all_workers", timeout=5)
            if ret.status_code == 200:
                ret = requests.post(f"{base}/list_models", timeout=5)
                if ret.status_code == 200:
                    self.models = ret.json().get("models", [])
                    # Populate selector if available
                    try:
                        self.model_selector.clear()
                        self.model_selector.addItems(self.models)
                    except Exception:
                        pass
                    # Update combined service status line
                    try:
                        worker_status = "Running" if self.models else "No Models"
                        # Preserve current controller status string if available
                        current = getattr(self, 'service_status', None)
                        ctrl_str = "Controller: Running"
                        if current is not None and current.text():
                            # try to keep previous controller status segment
                            parts = current.text().split('|')
                            if parts:
                                ctrl_str = parts[0].strip()
                        self.service_status.setText(f"{ctrl_str} | Model Worker: {worker_status}")
                        if self.models:
                            logger.info(f"Loaded models: {self.models}")
                    except Exception:
                        pass
        except Exception as e:
            logger.error(f"Failed to refresh model list: {e}")

    def update_temperature(self, value):
        self.temperature_label.setText(f"{value/100.0:.1f}")

    def update_top_p(self, value):
        self.top_p_label.setText(f"{value/100.0:.1f}")

    def send_message(self):
        """Send message to model."""
        text = self.text_input.text().strip()
        image = getattr(self.image_label, 'original_image', None)
        # If no manually loaded image, fall back to latest camera frame
        # so the LLM receives an image and can return annotated results.
        if image is None:
            try:
                if getattr(self, 'last_color_frame_bgr', None) is not None:
                    import cv2
                    bgr = self.last_color_frame_bgr
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    from PIL import Image
                    image = Image.fromarray(rgb)
            except Exception:
                image = None
        if not text and image is None:
            return
        if not self.models:
            QMessageBox.warning(self, "Warning", "No models available. Please wait for services to start.")
            return
        # Add prompt suffix
        text += " Your answer should be formatted as a list of tuples, " \
                "i.e. [(x1, y1), (x2, y2), ...], where each tuple contains the " \
                "x and y coordinates of a point satisfying the conditions above." \
                " The coordinates should be between 0 and 1, indicating the " \
                "normalized pixel locations of the points in the image."

        # Prepare conversation state
        if image is not None:
            if '<image>' not in text:
                text = '<image>\n' + text
            content = (text, image, 'Pad')  # default to Pad processing
            self.conversation_state = default_conversation.copy()
        else:
            content = text

        self.conversation_state.append_message(self.conversation_state.roles[0], content)
        self.conversation_state.append_message(self.conversation_state.roles[1], None)

        # Add user message to chat
        try:
            self.chat_widget.append(f"\nUser:\n{text}\n")
        except Exception:
            pass

        # Clear input and disable send during processing
        self.text_input.clear()
        self.send_button.setEnabled(False)
        self.send_button.setText("Processing...")

        # Send request
        self.send_request_to_worker()

    def find_vectors(self, text):
        import re
        pattern = r"\(([-+]?\d+\.?\d*(?:,\s*[-+]?\d+\.?\d*)*?)\)"
        matches = re.findall(pattern, text)
        vectors = []
        for match in matches:
            vector = [float(num) if '.' in num else int(num) for num in match.split(',')]
            vectors.append(vector)
        return vectors

    def visualize_2d(self, img, points, bboxes, scale, cross_size=9, cross_width=4):
        if img.mode != 'RGBA':
            img = img.convert('RGBA')
        draw = ImageDraw.Draw(img)
        size = int(cross_size * scale)
        width = int(cross_width * scale)
        for x, y in points:
            draw.line((x - size, y - size, x + size, y + size), fill='red', width=width)
            draw.line((x - size, y + size, x + size, y - size), fill='red', width=width)
        for x1, y1, x2, y2 in bboxes:
            draw.rectangle([x1, y1, x2, y2], outline='red', width=width)
        return img.convert('RGB')

    def process_response_visualization(self, response):
        try:
            vectors = self.find_vectors(response)
            vectors_2d = [vec for vec in vectors if len(vec) == 2]
            vectors_bbox = [vec for vec in vectors if len(vec) == 4]
            if not vectors_2d and not vectors_bbox:
                return response
            pil_images, images, transforms = self.conversation_state.get_images()
            if not pil_images:
                return response
            image = pil_images[-1].copy()
            transform = transforms[-1]
            # Use the padded model input size for de-normalization (Pad mode = 640x480)
            # LLM outputs are normalized in the padded image space
            ref_w, ref_h = 640, 480
            new_vectors = []
            for x, y in vectors_2d:
                if isinstance(x, float) and x <= 1:
                    x = x * ref_w
                    y = y * ref_h
                # For Pad mode the displayed chat image is already the 640x480 padded image,
                # so no additional transform is needed for visualization.
                new_vectors.append((x, y))
            new_bbox = []
            for x1, y1, x2, y2 in vectors_bbox:
                if isinstance(x1, float) and x1 <= 1:
                    x1, y1, x2, y2 = x1 * ref_w, y1 * ref_h, x2 * ref_w, y2 * ref_h
                # Same reasoning: draw directly in padded chat image space
                new_bbox.append((x1, y1, x2, y2))
            # Cache points for live overlay
            try:
                # Store points and the image size they are based on (padded size)
                w_vis, h_vis = ref_w, ref_h
                pts = []
                for (x, y) in new_vectors:
                    xi = int(round(x)); yi = int(round(y))
                    if 0 <= xi < w_vis and 0 <= yi < h_vis:
                        pts.append((xi, yi))
                self.latest_llm_points = pts
                self.latest_llm_points_size = (w_vis, h_vis)
                # Reset tracker so it re-initializes on next camera frame
                # Only reset position if user hasn't manually clicked
                if not self.user_clicked_position:
                    self.llm_tracker = None
                    self.llm_tracked_px = None
                    self.llm_tracked_py = None
                # Reset cluster state when new points are received
                self.llm_clusters = None
                self.llm_cluster_idx = -1

                # If an LLM session is active and the publisher isn't running yet,
                # start grasping now when points arrive.
                if self.llm_session_active:
                    has_white_running = (self.white_point_proc.proc is not None) and (self.white_point_proc.proc.poll() is None)
                    if not has_white_running and len(self.latest_llm_points) > 0:
                        chosen = self._choose_point_from_crosses_by_density(self.latest_llm_points, radius=8)
                        refined = self._select_point_from_crosses(getattr(self, 'last_color_frame_bgr', None), self.latest_llm_points)
                        if refined is not None:
                            chosen = refined if (chosen is None) else chosen
                        if chosen is not None:
                            px, py = int(chosen[0]), int(chosen[1])
                            try:
                                if self.last_color_frame_bgr is not None:
                                    hsv = cv2.cvtColor(self.last_color_frame_bgr, cv2.COLOR_BGR2HSV)
                                    px, py = self._nearest_non_white_black(hsv, px, py, radius=6)
                            except Exception:
                                pass
                            self.llm_tracked_px, self.llm_tracked_py = int(px), int(py)
                            self.llm_tracker = None
                            try:
                                self.white_point_proc.start(self.llm_tracked_px, self.llm_tracked_py, self.use_remote_stream, 0.0, None)
                                self.llm_status.setText(f"Tracking ({self.llm_tracked_px}, {self.llm_tracked_py})")
                            except Exception:
                                pass
            except Exception:
                pass
            annotated = self.visualize_2d(image, new_vectors, new_bbox, 1.0)
            return (response, annotated, 'Pad')
        except Exception as e:
            logger.error(f"Visualization error: {e}")
            return response

    def send_request_to_worker(self):
        try:
            # Determine conversation template by model
            model_name = self.model_selector.currentText() if hasattr(self, 'model_selector') else (self.models[0] if self.models else '')
            name_lower = model_name.lower() if isinstance(model_name, str) else ''
            if 'vicuna' in name_lower:
                template_name = 'vicuna_v1'
            elif 'llama' in name_lower:
                template_name = 'llava_llama_2'
            elif 'mistral' in name_lower:
                template_name = 'mistral_instruct'
            elif 'mpt' in name_lower:
                template_name = 'mpt'
            else:
                template_name = 'llava_v1'

            if len(self.conversation_state.messages) == 2:
                new_state = conv_templates[template_name].copy()
                new_state.append_message(new_state.roles[0], self.conversation_state.messages[-2][1])
                new_state.append_message(new_state.roles[1], None)
                self.conversation_state = new_state

            prompt = self.conversation_state.get_prompt()
            pil_images, images, transforms = self.conversation_state.get_images()

            request_data = {
                'model': model_name,
                'prompt': prompt,
                'temperature': self.temperature_slider.value() / 100.0,
                'top_p': self.top_p_slider.value() / 100.0,
                'max_new_tokens': min(self.max_tokens_spin.value(), 1536),
                'stop': self.conversation_state.sep if self.conversation_state.sep_style in [SeparatorStyle.SINGLE, SeparatorStyle.MPT] else self.conversation_state.sep2,
                'images': images,
            }
            self.worker_thread.set_request_data(request_data)
            self.worker_thread.start()
        except Exception as e:
            self.handle_error(f"Failed to send request: {e}")

    def handle_response(self, response):
        self._pending_response_text = response

    def handle_error(self, error_message):
        try:
            self.chat_widget.add_message("System", f"Error: {error_message}")
        except Exception:
            pass
        self._pending_response_text = None
        self.request_finished()

    def request_finished(self):
        if self._pending_response_text is not None:
            # Save assistant message into conversation state and render visualization
            try:
                self.conversation_state.messages[-1] = [self.conversation_state.roles[1], self._pending_response_text]
            except Exception:
                pass
            processed = self.process_response_visualization(self._pending_response_text)
            if isinstance(processed, tuple):
                # (text, annotated_image, mode)
                self.chat_widget.add_message("Assistant", processed, is_image=True)
            else:
                self.chat_widget.add_message("Assistant", processed)
            self._pending_response_text = None

        self.send_button.setEnabled(True)
        self.send_button.setText("Send")

    def regenerate_response(self):
        pass

    def clear_chat(self):
        try:
            self.chat_widget.clear()
            self.conversation_state = default_conversation.copy()
        except Exception:
            pass

    def closeEvent(self, event):
        try:
            if self.camera_thread is not None:
                self.camera_thread.stop(); self.camera_thread.wait(1000)
            if self.direct_camera_thread is not None:
                try:
                    self.direct_camera_thread.stop(); self.direct_camera_thread.wait(1000)
                except Exception:
                    pass
            # Stop local D415 publisher
            try:
                if hasattr(self, 'camera_pub_proc') and self.camera_pub_proc is not None:
                    self.camera_pub_proc.stop()
            except Exception:
                pass
            self.stop_services()
        except Exception:
            pass
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="RoboPoint Visual Servoing GUI")
    parser.add_argument("--controller-url", type=str, default="http://127.0.0.1:11000", help="Controller URL")
    parser.add_argument("--model-path", type=str, default="wentao-yuan/robopoint-v1-vicuna-v1.5-13b", help="Model path")
    parser.add_argument("--load-4bit", action="store_true", default=True, help="Load model in 4-bit mode")
    parser.add_argument('-r', '--remote', action='store_true', help='Receive camera images from a remote robot (subscribe to robot IP).')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("RoboPoint (Visual Servoing)")
    window = RoboPointMainWindow(controller_url=args.controller_url, autostart=False,
                                 model_path=args.model_path, load_4bit=args.load_4bit,
                                 use_remote_stream=args.remote)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

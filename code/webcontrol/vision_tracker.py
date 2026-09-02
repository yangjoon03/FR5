"""
카메라를 열어서 얼굴을 검출/추적하고, 필요하면 로봇에게 보정 이동을
보내는 모듈. 실제 카메라·얼굴 인식 로직(OpenCV)이 여기 들어있고,
좌표/보정량 계산 자체는 face_tracking.py(순수 로직, 카메라 없이도
테스트 가능)에 위임합니다.

안전상 이유로 기본값을 보수적으로 잡았습니다:
- max_step_mm 기본 3mm, 서버에서도 15mm를 하드 상한으로 강제
- tick_interval(보정 명령을 보내는 주기) 기본 0.25초 (초당 4회) -
  너무 빠르게 계속 이동 명령을 쏘지 않도록 함
- 얼굴이 인식 안 되는 동안에는 어떤 보정도 보내지 않음(가만히 있음)
- "트래킹 시작" 버튼을 눌러야만 실제로 로봇에 명령이 나감(카메라
  미리보기만 켜져 있을 때는 로봇을 전혀 움직이지 않음)
"""
import threading
import time

import cv2

from face_tracking import FaceLock, compute_correction

_HARD_MAX_STEP_MM = 15.0  # 사용자가 설정값을 아무리 높여도 이걸 넘지 않음


class CameraTracker:
    def __init__(self, robot_manager):
        self._manager = robot_manager

        self._cap = None
        self._detector = None
        self._capture_thread = None
        self._running = False

        self._lock = threading.Lock()  # 아래 공유 상태 보호
        self._latest_jpeg = None
        self._frame_w = 0
        self._frame_h = 0
        self._current_bbox = None
        self._tracking_enabled = False
        self._last_error = None

        self._face_lock = FaceLock()
        self._target_size_ratio = 0.25

        self.gains = {"x": 0.02, "y": 0.02, "z": 60.0}
        self.max_step_mm = 3.0
        self.invert = {"x": False, "y": False, "z": False}
        self.tick_interval = 0.25

    # ------------------------------------------------------------------
    def _load_detector(self):
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        detector = cv2.CascadeClassifier(cascade_path)
        if detector.empty():
            raise RuntimeError(
                "얼굴 인식 모델 파일을 찾지 못했습니다 (%s). "
                "opencv-python이 제대로 설치됐는지 확인하세요: "
                "pip install --force-reinstall opencv-python" % cascade_path
            )
        return detector

    def open(self, index: int):
        self.close()
        cap = cv2.VideoCapture(int(index))
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"카메라 인덱스 {index}를 열 수 없습니다. 다른 프로그램이 쓰고 있거나 잘못된 인덱스일 수 있습니다.")
        self._detector = self._load_detector()
        self._cap = cap
        self._face_lock.reset()
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def close(self):
        self._running = False
        self._tracking_enabled = False
        thread = self._capture_thread
        self._capture_thread = None
        if thread is not None:
            thread.join(timeout=2)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        with self._lock:
            self._latest_jpeg = None
            self._current_bbox = None
        self._face_lock.reset()

    def is_open(self) -> bool:
        return self._cap is not None

    # ------------------------------------------------------------------
    def _capture_loop(self):
        last_tick = 0.0
        while self._running and self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected = self._detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
            bbox = self._face_lock.update([tuple(int(v) for v in f) for f in detected])

            cx, cy = w // 2, h // 2
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            if bbox is not None:
                x, y, bw, bh = bbox
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = "TRACKING" if self._tracking_enabled else "PREVIEW"
            cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if self._tracking_enabled else (200, 200, 200), 2)

            encode_ok, buf = cv2.imencode(".jpg", frame)
            jpeg = buf.tobytes() if encode_ok else None

            with self._lock:
                self._latest_jpeg = jpeg
                self._frame_w, self._frame_h = w, h
                self._current_bbox = bbox

            now = time.time()
            if self._tracking_enabled and bbox is not None and (now - last_tick) >= self.tick_interval:
                last_tick = now
                self._send_correction(w, h, bbox)
            elif bbox is None:
                last_tick = now  # 사람을 놓친 동안은 보정 명령을 보내지 않음

    def _send_correction(self, w, h, bbox):
        max_step = min(self.max_step_mm, _HARD_MAX_STEP_MM)
        result = compute_correction(w, h, bbox, self._target_size_ratio, self.gains, max_step, self.invert)
        with self._lock:
            self._last_error = result
        try:
            self._manager.move_tool_offset(result["dx"], result["dy"], result["dz"], vel=10.0)
        except Exception as e:
            print("[카메라 트래킹] 보정 이동 실패:", e)

    # ------------------------------------------------------------------
    def get_jpeg(self):
        with self._lock:
            return self._latest_jpeg

    def calibrate_distance(self):
        with self._lock:
            bbox, w = self._current_bbox, self._frame_w
        if bbox is None or not w:
            raise RuntimeError("현재 인식된 얼굴이 없어 거리 기준을 설정할 수 없습니다. 얼굴이 보이는 상태에서 다시 시도하세요.")
        self._target_size_ratio = bbox[2] / float(w)
        return self._target_size_ratio

    def start_tracking(self):
        if not self.is_open():
            raise RuntimeError("먼저 카메라를 열어야 합니다.")
        self._tracking_enabled = True

    def stop_tracking(self):
        self._tracking_enabled = False

    def update_config(self, invert_x=None, invert_y=None, invert_z=None, max_step_mm=None):
        if invert_x is not None:
            self.invert["x"] = bool(invert_x)
        if invert_y is not None:
            self.invert["y"] = bool(invert_y)
        if invert_z is not None:
            self.invert["z"] = bool(invert_z)
        if max_step_mm is not None:
            self.max_step_mm = max(0.0, min(float(max_step_mm), _HARD_MAX_STEP_MM))

    def state(self):
        with self._lock:
            face_found = self._current_bbox is not None
            err = self._last_error or {}
        return {
            "opened": self.is_open(),
            "tracking_enabled": self._tracking_enabled,
            "face_found": face_found,
            "target_size_ratio": round(self._target_size_ratio, 4),
            "size_ratio": round(err.get("size_ratio", 0), 4),
            "error_x_px": err.get("error_x_px", 0),
            "error_y_px": err.get("error_y_px", 0),
            "invert": dict(self.invert),
            "max_step_mm": self.max_step_mm,
        }

"""
카메라를 열어서 얼굴을 검출/추적하고, 필요하면 로봇에게 보정 이동을
보내는 모듈. 실제 카메라·얼굴 인식 로직(OpenCV)이 여기 들어있고,
좌표/보정량 계산 자체는 face_tracking.py(순수 로직, 카메라 없이도
테스트 가능)에 위임합니다.

중앙 정렬은 손목 회전(팬/틸트)으로, 거리 유지는 공구 Z축 평행이동으로
처리합니다 (평행이동만으로 좌우/상하까지 맞추면 팔 전체가 계속 옆으로
밀려나 작업반경을 금방 벗어나므로, 회전으로 바꿔서 관절이 허용하는
각도 끝까지 넓게 따라갈 수 있게 함).

안전상 이유로 기본값을 보수적으로 잡았습니다:
- max_step_deg(팬/틸트) 기본 2°, 서버에서 10°를 하드 상한으로 강제
- max_step_mm(거리 유지) 기본 3mm, 서버에서 15mm를 하드 상한으로 강제
- tick_interval(보정 명령을 보내는 주기) 기본 0.25초 (초당 4회) -
  너무 빠르게 계속 이동 명령을 쏘지 않도록 함
- 얼굴이 인식 안 되는 동안에는 어떤 보정도 보내지 않음(가만히 있음)
- "트래킹 시작" 버튼을 눌러야만 실제로 로봇에 명령이 나감(카메라
  미리보기만 켜져 있을 때는 로봇을 전혀 움직이지 않음)
"""
import os
import threading
import time

import cv2

from face_tracking import FaceLock, compute_correction

_HARD_MAX_STEP_MM = 15.0   # z(거리) 이동 - 사용자가 설정값을 아무리 높여도 이걸 넘지 않음
_HARD_MAX_STEP_DEG = 10.0  # 팬/틸트 회전 - 사용자가 설정값을 아무리 높여도 이걸 넘지 않음

# 일부 opencv-python 배포판(특히 headless나 conda 빌드 일부)은
# cv2.data.haarcascades 안에 실제 모델 xml 파일을 담고 있지 않은 경우가
# 있습니다(설치는 됐지만 데이터 리소스가 빠진 상태). 그래서 저장소 안에
# 같은 파일을 직접 포함시켜두고, 시스템 쪽에 없으면 이걸 대신 씁니다.
_BUNDLED_CASCADE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "models", "haarcascade_frontalface_default.xml"
)


def list_available_cameras(max_index: int = 5, already_open_index: int = None):
    """
    OpenCV엔 "연결된 카메라 목록"을 바로 주는 API가 없어서, 인덱스
    0번부터 max_index-1번까지 순서대로 실제로 열어보고, 열리는 것만
    추려서 반환합니다 (해상도도 같이 확인).

    - already_open_index: 지금 이미 이 서버가 열어서 쓰고 있는 인덱스가
      있으면 다시 열어보지 않고(장치가 이미 점유돼서 어차피 실패하거나
      스트림이 끊길 수 있음) "현재 연결됨"으로만 표시.
    """
    results = []
    for i in range(max_index):
        if already_open_index is not None and i == already_open_index:
            results.append({"index": i, "width": None, "height": None, "in_use": True})
            continue
        cap = cv2.VideoCapture(i)
        try:
            opened = cap.isOpened()
            width = height = None
            if opened:
                ok, frame = cap.read()
                if ok and frame is not None:
                    height, width = frame.shape[:2]
        finally:
            cap.release()
        if opened:
            results.append({"index": i, "width": width, "height": height, "in_use": False})
    return results


class CameraTracker:
    def __init__(self, robot_manager):
        self._manager = robot_manager

        self._cap = None
        self._index = None
        self._detector = None
        self._capture_thread = None
        self._running = False

        self._lock = threading.Lock()  # 아래 공유 상태 보호
        self._latest_jpeg = None
        self._frame_w = 0
        self._frame_h = 0
        self._current_bbox = None       # 화면 표시/락 판단용 - 검출된 원본 박스
        self._smooth_bbox = None        # 보정 계산용 - 프레임간 흔들림을 줄인 값
        self._smoothing_alpha = 0.35    # 낮을수록 부드럽지만 반응이 느려짐
        self._tracking_enabled = False
        self._last_error = None
        self._last_move_result = None

        self._face_lock = FaceLock()
        self._target_size_ratio = 0.25

        self.gains = {"pan": 0.03, "tilt": 0.03, "z": 60.0}  # pan/tilt: °/px, z: mm/size_ratio오차
        self.max_step_deg = 2.0
        self.max_step_mm = 3.0
        self.invert = {"pan": False, "tilt": False, "z": False}
        self.tick_interval = 0.25

    # ------------------------------------------------------------------
    def _load_detector(self):
        system_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        for candidate in (system_path, _BUNDLED_CASCADE_PATH):
            if not os.path.exists(candidate):
                continue
            detector = cv2.CascadeClassifier(candidate)
            if not detector.empty():
                return detector
        raise RuntimeError(
            "얼굴 인식 모델 파일을 찾지 못했습니다 (시스템 경로: %s / 내장 경로: %s 둘 다 없거나 로드 실패). "
            "code/webcontrol/models/haarcascade_frontalface_default.xml 파일이 저장소에 그대로 있는지 확인하세요."
            % (system_path, _BUNDLED_CASCADE_PATH)
        )

    def open(self, index: int):
        self.close()
        cap = cv2.VideoCapture(int(index))
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"카메라 인덱스 {index}를 열 수 없습니다. 다른 프로그램이 쓰고 있거나 잘못된 인덱스일 수 있습니다.")
        # macOS(AVFoundation)에서 일부 웹캠은 기본 포맷 협상이 잘못돼서
        # 흑백처럼 보이는 경우가 있음 - MJPG로 강제 지정하면 대부분 해결됨.
        # 실패해도(장치가 MJPG를 지원 안 해도) 무시하고 기본 포맷 그대로 사용.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._detector = self._load_detector()
        self._cap = cap
        self._index = int(index)
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
        self._index = None
        with self._lock:
            self._latest_jpeg = None
            self._current_bbox = None
        self._face_lock.reset()

    def is_open(self) -> bool:
        return self._cap is not None

    def current_index(self):
        return self._index

    # ------------------------------------------------------------------
    def _capture_loop(self):
        last_tick = 0.0
        while self._running and self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            if frame.ndim == 2:  # 장치가 진짜로 단일 채널(흑백)만 주는 경우 - 그리기/인코딩 위해 3채널로 변환
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected = self._detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
            bbox = self._face_lock.update([tuple(int(v) for v in f) for f in detected])

            if bbox is None:
                self._smooth_bbox = None  # 사람이 바뀌었을 수도 있으니 평활화 값도 리셋
            else:
                a = self._smoothing_alpha
                if self._smooth_bbox is None:
                    self._smooth_bbox = tuple(float(v) for v in bbox)
                else:
                    self._smooth_bbox = tuple(
                        a * new + (1 - a) * old for new, old in zip(bbox, self._smooth_bbox)
                    )

            # 이후 표시/보정 계산은 전부 평활화된 값을 씀 (화면에 그리는 박스도
            # 덜 떨려 보이고, 보정 계산도 프레임 하나의 튀는 값에 안 흔들림)
            smooth = None if self._smooth_bbox is None else tuple(int(v) for v in self._smooth_bbox)

            cx, cy = w // 2, h // 2
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            if smooth is not None:
                x, y, bw, bh = smooth
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = "TRACKING" if self._tracking_enabled else "PREVIEW"
            cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if self._tracking_enabled else (200, 200, 200), 2)

            encode_ok, buf = cv2.imencode(".jpg", frame)
            jpeg = buf.tobytes() if encode_ok else None

            with self._lock:
                self._latest_jpeg = jpeg
                self._frame_w, self._frame_h = w, h
                self._current_bbox = smooth

            now = time.time()
            if self._tracking_enabled and smooth is not None and (now - last_tick) >= self.tick_interval:
                last_tick = now
                self._send_correction(w, h, smooth)
            elif smooth is None:
                last_tick = now  # 사람을 놓친 동안은 보정 명령을 보내지 않음

    def _send_correction(self, w, h, bbox):
        max_deg = min(self.max_step_deg, _HARD_MAX_STEP_DEG)
        max_mm = min(self.max_step_mm, _HARD_MAX_STEP_MM)
        result = compute_correction(w, h, bbox, self._target_size_ratio, self.gains, max_deg, max_mm, self.invert)
        with self._lock:
            self._last_error = result
        try:
            # 중앙 정렬 = 손목 회전(팬=dry, 틸트=drx), 거리 유지 = 공구 Z 평행이동(dz)
            move_error = self._manager.move_tool_offset(
                0.0, 0.0, result["dz"],
                drx_deg=result["d_tilt"], dry_deg=result["d_pan"], drz_deg=0.0,
                vel=10.0,
            )
            with self._lock:
                self._last_move_result = {"error": move_error, "exception": None}
            if move_error != 0:
                # 이 SDK는 실패를 예외가 아니라 반환값(에러코드)으로 알려주므로,
                # 여기서 안 찍으면 "왜 안 움직이지?" 상황이 화면에 전혀 안 보임.
                print(f"[카메라 트래킹] 보정 이동 반환값(에러): {move_error} "
                      f"(0이 아니면 실패 - 로봇 활성화 여부/안전정지 상태를 확인하세요)")
        except Exception as e:
            with self._lock:
                self._last_move_result = {"error": None, "exception": str(e)}
            print("[카메라 트래킹] 보정 이동 실패(예외):", e)

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

    def update_config(self, invert_pan=None, invert_tilt=None, invert_z=None,
                      max_step_deg=None, max_step_mm=None):
        if invert_pan is not None:
            self.invert["pan"] = bool(invert_pan)
        if invert_tilt is not None:
            self.invert["tilt"] = bool(invert_tilt)
        if invert_z is not None:
            self.invert["z"] = bool(invert_z)
        if max_step_deg is not None:
            self.max_step_deg = max(0.0, min(float(max_step_deg), _HARD_MAX_STEP_DEG))
        if max_step_mm is not None:
            self.max_step_mm = max(0.0, min(float(max_step_mm), _HARD_MAX_STEP_MM))

    def state(self):
        with self._lock:
            face_found = self._current_bbox is not None
            err = self._last_error or {}
            move = self._last_move_result or {}
        return {
            "opened": self.is_open(),
            "index": self._index,
            "tracking_enabled": self._tracking_enabled,
            "face_found": face_found,
            "target_size_ratio": round(self._target_size_ratio, 4),
            "size_ratio": round(err.get("size_ratio", 0), 4),
            "error_x_px": err.get("error_x_px", 0),
            "error_y_px": err.get("error_y_px", 0),
            "d_pan": round(err.get("d_pan", 0), 3),
            "d_tilt": round(err.get("d_tilt", 0), 3),
            "dz": round(err.get("dz", 0), 3),
            "last_move_error": move.get("error"),
            "last_move_exception": move.get("exception"),
            "invert": dict(self.invert),
            "max_step_deg": self.max_step_deg,
            "max_step_mm": self.max_step_mm,
        }

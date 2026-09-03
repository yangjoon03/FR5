"""
카메라를 열어서 (오른)손을 검출/추적하고, 손이 펴져있으면 로봇에게 보정
이동을 보내는 모듈. 실제 카메라·손 인식 로직(MediaPipe)이 여기 들어있고,
좌표/보정량 계산 자체는 face_tracking.py(순수 로직, 카메라 없이도 테스트
가능 - 얼굴이든 손이든 bbox 하나만 받으면 되는 범용 로직이라 이름만
face_tracking일 뿐 그대로 재사용)에 위임합니다.

동작 방식
- MediaPipe GestureRecognizer로 프레임마다 손을 찾고, 손모양(제스처)까지
  같이 분류합니다 (별도 로직 없이 모델이 바로 "편 손"/"주먹" 등을 알려줌).
- 여러 손이 보여도 "오른손(Right)"만 후보로 걸러낸 뒤, 그 중 하나만 계속
  추적합니다(FaceLock 재사용 - 원래 얼굴용으로 짠 "한 개체만 유지" 로직인데
  bbox만 있으면 되니 손에도 그대로 씀).
- **손을 편 상태(Open_Palm)일 때만** 보정 명령을 보냄. 주먹(Closed_Fist)을
  쥐거나 다른 모양이면 그 순간 아무 명령도 안 보내고 가만히 있음(정지).
- 중앙 정렬은 손목 회전(팬/틸트), 거리 유지는 공구 Z축 평행이동 -
  02_movement의 J6 방향 이동과 동일한 메커니즘.

⚠️ 중요: MediaPipe의 손 좌우(handedness) 판정은 "입력 영상이 좌우反전된
(셀카처럼 거울에 비친) 영상"이라는 가정 하에 이루어집니다 (공식 문서에
명시됨). 로봇에 달린 카메라는 보통 거울처럼 반전되지 않은 일반 영상을
주므로, 실제로는 결과가 반대로(내가 든 오른손이 "Left"로) 나올 가능성이
높습니다. 그래서 어느 쪽을 필터링할지 뒤집을 수 있는 옵션을 뒀습니다
(invert_handedness) - 실기에서 반대로 반응하면 이걸 켜세요.

⚠️ mediapipe 버전 주의: 최신 1.x(pip 기본 설치 버전)는 이 GestureRecognizer
기능이 macOS에서 즉시 크래시(Segfault급 강제종료, 파이썬 예외로도 못 잡음)
하는 걸 확인했습니다. requirements.txt에 안정적으로 동작을 확인한
0.10.14로 고정해뒀습니다 - 절대 버전을 임의로 올리지 마세요.

안전상 이유로 기본값을 보수적으로 잡았습니다:
- max_step_deg(팬/틸트) 기본 2°, 서버에서 10°를 하드 상한으로 강제
- max_step_mm(거리 유지) 기본 3mm, 서버에서 15mm를 하드 상한으로 강제
- tick_interval(보정 명령을 보내는 주기) 기본 0.25초
- 손이 안 보이거나, 오른손이 아니거나, 편 손이 아니면 어떤 보정도 안 보냄
- "트래킹 시작" 버튼을 눌러야만 실제로 로봇에 명령이 나감
"""
import os
import threading
import time

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from face_tracking import FaceLock, compute_correction

_HARD_MAX_STEP_MM = 15.0   # z(거리) 이동 - 사용자가 설정값을 아무리 높여도 이걸 넘지 않음
_HARD_MAX_STEP_DEG = 10.0  # 팬/틸트 회전 - 사용자가 설정값을 아무리 높여도 이걸 넘지 않음

_OPEN_GESTURE = "Open_Palm"
_STOP_GESTURES = {"Closed_Fist"}  # 이 제스처면 명시적으로 "정지" 신호로 취급

_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "gesture_recognizer.task")


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


def _hand_bbox(landmarks, frame_w, frame_h):
    """MediaPipe 정규화 랜드마크(0~1) 21개를 픽셀 bbox (x,y,w,h)로 변환."""
    xs = [lm.x * frame_w for lm in landmarks]
    ys = [lm.y * frame_h for lm in landmarks]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    return (int(x0), int(y0), int(x1 - x0), int(y1 - y0))


class CameraTracker:
    def __init__(self, robot_manager):
        self._manager = robot_manager

        self._cap = None
        self._index = None
        self._recognizer = None
        self._capture_thread = None
        self._running = False

        self._lock = threading.Lock()  # 아래 공유 상태 보호
        self._latest_jpeg = None
        self._frame_w = 0
        self._frame_h = 0
        self._current_bbox = None       # 화면 표시/락 판단용 - 검출된 원본 박스
        self._smooth_bbox = None        # 보정 계산용 - 프레임간 흔들림을 줄인 값
        self._smoothing_alpha = 0.35    # 낮을수록 부드럽지만 반응이 느려짐
        self._current_gesture = None
        self._tracking_enabled = False
        self._last_error = None
        self._last_move_result = None

        self._hand_lock = FaceLock()  # 이름만 FaceLock, bbox 하나만 있으면 뭐든 추적 가능한 범용 로직
        self._target_size_ratio = 0.25

        self.gains = {"pan": 0.03, "tilt": 0.03, "z": 60.0}  # pan/tilt: °/px, z: mm/size_ratio오차
        self.max_step_deg = 2.0
        self.max_step_mm = 3.0
        self.invert = {"pan": False, "tilt": False, "z": False}
        self.invert_handedness = False  # 실기에서 반대 손이 잡히면 켜기 (모듈 docstring 참고)
        self.tick_interval = 0.25

    # ------------------------------------------------------------------
    def _load_recognizer(self):
        if not os.path.exists(_MODEL_PATH):
            raise RuntimeError(
                "손 인식 모델 파일을 찾지 못했습니다 (%s). "
                "code/webcontrol/models/gesture_recognizer.task 파일이 저장소에 그대로 있는지 확인하세요."
                % _MODEL_PATH
            )
        base_options = mp_python.BaseOptions(model_asset_path=_MODEL_PATH)
        options = mp_vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=4,  # 여러 손/여러 사람이 보여도 그 중 오른손만 골라내기 위해 넉넉히
        )
        return mp_vision.GestureRecognizer.create_from_options(options)

    def open(self, index: int):
        self.close()
        cap = cv2.VideoCapture(int(index))
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"카메라 인덱스 {index}를 열 수 없습니다. 다른 프로그램이 쓰고 있거나 잘못된 인덱스일 수 있습니다.")
        # macOS(AVFoundation)에서 일부 웹캠은 기본 포맷 협상이 잘못돼서
        # 흑백처럼 보이는 경우가 있음 - MJPG로 강제 지정하면 대부분 해결됨.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._recognizer = self._load_recognizer()
        self._cap = cap
        self._index = int(index)
        self._hand_lock.reset()
        self._smooth_bbox = None
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
        if self._recognizer is not None:
            try:
                self._recognizer.close()
            except Exception:
                pass
            self._recognizer = None
        self._index = None
        with self._lock:
            self._latest_jpeg = None
            self._current_bbox = None
            self._current_gesture = None
        self._hand_lock.reset()
        self._smooth_bbox = None

    def is_open(self) -> bool:
        return self._cap is not None

    def current_index(self):
        return self._index

    # ------------------------------------------------------------------
    def _detect_right_hand_candidates(self, frame, w, h):
        """
        이번 프레임에서 검출된 손들 중 오른손만 골라서
        [(bbox, gesture_label), ...] 리스트로 반환.
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._recognizer.recognize(mp_image)

        wanted_label = "Left" if self.invert_handedness else "Right"
        candidates = []
        for i, handedness in enumerate(result.handedness):
            if not handedness:
                continue
            if handedness[0].category_name != wanted_label:
                continue
            bbox = _hand_bbox(result.hand_landmarks[i], w, h)
            gesture = result.gestures[i][0].category_name if result.gestures[i] else None
            candidates.append((bbox, gesture))
        return candidates

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
            candidates = self._detect_right_hand_candidates(frame, w, h)
            gesture_by_bbox = {bbox: g for bbox, g in candidates}
            bbox = self._hand_lock.update([bbox for bbox, _ in candidates])
            gesture = gesture_by_bbox.get(bbox)

            if bbox is None:
                self._smooth_bbox = None  # 사람/손이 바뀌었을 수도 있으니 평활화 값도 리셋
            else:
                a = self._smoothing_alpha
                if self._smooth_bbox is None:
                    self._smooth_bbox = tuple(float(v) for v in bbox)
                else:
                    self._smooth_bbox = tuple(
                        a * new + (1 - a) * old for new, old in zip(bbox, self._smooth_bbox)
                    )

            # 이후 표시/보정 계산은 전부 평활화된 위치를 씀 (제스처는 평활화 대상이
            # 아니라 이번 프레임에 매칭된 값을 그대로 씀 - 어차피 모델이 프레임마다 분류)
            smooth = None if self._smooth_bbox is None else tuple(int(v) for v in self._smooth_bbox)
            is_open_hand = gesture == _OPEN_GESTURE

            cx, cy = w // 2, h // 2
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            if smooth is not None:
                x, y, bw, bh = smooth
                color = (0, 255, 0) if is_open_hand else (0, 165, 255)
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
                cv2.putText(frame, str(gesture), (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            label = "TRACKING" if self._tracking_enabled else "PREVIEW"
            cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if self._tracking_enabled else (200, 200, 200), 2)

            encode_ok, buf = cv2.imencode(".jpg", frame)
            jpeg = buf.tobytes() if encode_ok else None

            with self._lock:
                self._latest_jpeg = jpeg
                self._frame_w, self._frame_h = w, h
                self._current_bbox = smooth
                self._current_gesture = gesture

            now = time.time()
            should_move = self._tracking_enabled and smooth is not None and is_open_hand
            if should_move and (now - last_tick) >= self.tick_interval:
                last_tick = now
                self._send_correction(w, h, smooth)
            elif not should_move:
                last_tick = now  # 주먹을 쥐었거나 손을 놓친 동안은 보정 명령을 보내지 않음

    def _send_correction(self, w, h, bbox):
        max_deg = min(self.max_step_deg, _HARD_MAX_STEP_DEG)
        max_mm = min(self.max_step_mm, _HARD_MAX_STEP_MM)
        result = compute_correction(w, h, bbox, self._target_size_ratio, self.gains, max_deg, max_mm, self.invert)
        with self._lock:
            self._last_error = result
        try:
            # 중앙 정렬 = 손목 회전(팬=dry, 틸트=drx), 거리 유지 = 공구 Z 평행이동(dz)
            # blend_r>0: 매 틱마다 완전히 멈췄다 재출발하지 않고 부드럽게 이어붙임
            move_error = self._manager.move_tool_offset(
                0.0, 0.0, result["dz"],
                drx_deg=result["d_tilt"], dry_deg=result["d_pan"], drz_deg=0.0,
                vel=10.0, blend_r=10.0,
            )
            with self._lock:
                self._last_move_result = {"error": move_error, "exception": None}
            if move_error != 0:
                # 이 SDK는 실패를 예외가 아니라 반환값(에러코드)으로 알려주므로,
                # 여기서 안 찍으면 "왜 안 움직이지?" 상황이 화면에 전혀 안 보임.
                print(f"[손 트래킹] 보정 이동 반환값(에러): {move_error} "
                      f"(0이 아니면 실패 - 로봇 활성화 여부/안전정지 상태를 확인하세요)")
        except Exception as e:
            with self._lock:
                self._last_move_result = {"error": None, "exception": str(e)}
            print("[손 트래킹] 보정 이동 실패(예외):", e)

    # ------------------------------------------------------------------
    def get_jpeg(self):
        with self._lock:
            return self._latest_jpeg

    def calibrate_distance(self):
        with self._lock:
            bbox, w = self._current_bbox, self._frame_w
        if bbox is None or not w:
            raise RuntimeError("현재 인식된 오른손이 없어 거리 기준을 설정할 수 없습니다. 손이 보이는 상태에서 다시 시도하세요.")
        self._target_size_ratio = bbox[2] / float(w)
        return self._target_size_ratio

    def start_tracking(self):
        if not self.is_open():
            raise RuntimeError("먼저 카메라를 열어야 합니다.")
        self._tracking_enabled = True

    def stop_tracking(self):
        self._tracking_enabled = False

    def update_config(self, invert_pan=None, invert_tilt=None, invert_z=None,
                      invert_handedness=None, max_step_deg=None, max_step_mm=None):
        if invert_pan is not None:
            self.invert["pan"] = bool(invert_pan)
        if invert_tilt is not None:
            self.invert["tilt"] = bool(invert_tilt)
        if invert_z is not None:
            self.invert["z"] = bool(invert_z)
        if invert_handedness is not None:
            self.invert_handedness = bool(invert_handedness)
        if max_step_deg is not None:
            self.max_step_deg = max(0.0, min(float(max_step_deg), _HARD_MAX_STEP_DEG))
        if max_step_mm is not None:
            self.max_step_mm = max(0.0, min(float(max_step_mm), _HARD_MAX_STEP_MM))

    def state(self):
        with self._lock:
            hand_found = self._current_bbox is not None
            gesture = self._current_gesture
            err = self._last_error or {}
            move = self._last_move_result or {}
        return {
            "opened": self.is_open(),
            "index": self._index,
            "tracking_enabled": self._tracking_enabled,
            "hand_found": hand_found,
            "gesture": gesture,
            "is_open_hand": gesture == _OPEN_GESTURE,
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
            "invert_handedness": self.invert_handedness,
            "max_step_deg": self.max_step_deg,
            "max_step_mm": self.max_step_mm,
        }

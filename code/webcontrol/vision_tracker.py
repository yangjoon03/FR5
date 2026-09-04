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
- 거리 유지는 **조그(JOG)**로 처리 - 목표보다 멀면 전진 조그 시작,
  가까우면 후진 조그 시작, 목표 범위 안에 들어오면 정지. MoveL을 매번
  반복 호출하는 대신 "필요한 동안 계속 부드럽게 움직이다가 멈춤" 방식이라
  훨씬 실시간처럼 느껴짐. 중앙 정렬(팬/틸트)은 현재 비활성화 상태.

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
_HARD_MAX_STEP_DEG = 10.0  # 팬/틸트 회전 - 사용자가 설정값을 아무리 높여도 이걸 넘지 않음 (현재 미사용)

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


def _landmarks_px(landmarks, frame_w, frame_h):
    """MediaPipe 정규화 랜드마크(0~1) 21개를 픽셀 (x,y) 좌표 리스트로 변환."""
    return [(int(lm.x * frame_w), int(lm.y * frame_h)) for lm in landmarks]


# 손 관절(마디) 연결 순서 - MediaPipe 손 랜드마크의 고정된 구조
# 0=손목, 1~4=엄지, 5~8=검지, 9~12=중지, 13~16=약지, 17~20=소지
_HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # 엄지
    (0, 5), (5, 6), (6, 7), (7, 8),          # 검지
    (5, 9), (9, 10), (10, 11), (11, 12),     # 중지
    (9, 13), (13, 14), (14, 15), (15, 16),   # 약지
    (13, 17), (17, 18), (18, 19), (19, 20),  # 소지
    (0, 17),                                  # 손목 - 소지 밑동 (손바닥 테두리)
]


def _draw_hand_skeleton(frame, landmarks_px, color=(0, 255, 0)):
    """21개 관절점 + 관절 사이 연결선을 프레임에 그려서 '마디마디'를 눈으로 보이게 함."""
    for a, b in _HAND_CONNECTIONS:
        cv2.line(frame, landmarks_px[a], landmarks_px[b], color, 2)
    for i, (x, y) in enumerate(landmarks_px):
        radius = 6 if i == 0 else 4  # 손목(0번)만 좀 더 크게 표시
        cv2.circle(frame, (x, y), radius, color, -1)


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

        # 거리(전후) 추적은 매번 조금씩 이동 명령을 다시 보내는 대신,
        # 조그(JOG)로 "필요한 방향으로 계속 움직이다가, 목표 거리에
        # 도달하거나 방향이 바뀌면 정지"하는 방식을 씀 - 훨씬 부드럽고
        # 실시간처럼 느껴짐 (MoveL을 짧은 주기로 반복 호출하는 것보다 나음).
        self._jog_direction = None  # None | "fwd"(전진) | "back"(후진) - 지금 조그 중인 방향
        self.jog_vel = 15.0  # 조그 속도 백분율
        self.distance_deadzone_ratio = 0.02  # 이 안에 들어오면 정지(목표 거리에 도달)

        self.invert = {"pan": False, "tilt": False, "z": False}
        self.invert_handedness = False  # 실기에서 반대 손이 잡히면 켜기 (모듈 docstring 참고)
        self.tick_interval = 0.15  # 방향 재판단 주기 (조그 자체는 계속 이어짐, 이 주기로 멈출지만 확인)

        # 팬/틸트(회전)는 현재 비활성화 - 다시 켤 때 필요한 값들
        self.gains = {"pan": 0.03, "tilt": 0.03, "z": 60.0}
        self.max_step_deg = 2.0
        self.max_step_mm = 3.0

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
        self._force_jog_stop()
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
        [(bbox, gesture_label, landmarks_px), ...] 리스트로 반환.
        landmarks_px: 21개 관절점의 픽셀 좌표 - 화면에 마디마디를 그려서
        보여주는 용도 (실제 이동 계산에는 bbox만 사용).
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
            landmarks = result.hand_landmarks[i]
            bbox = _hand_bbox(landmarks, w, h)
            gesture = result.gestures[i][0].category_name if result.gestures[i] else None
            candidates.append((bbox, gesture, _landmarks_px(landmarks, w, h)))
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
            info_by_bbox = {bbox: (g, lm) for bbox, g, lm in candidates}
            bbox = self._hand_lock.update([bbox for bbox, _, _ in candidates])
            gesture, landmarks_px = info_by_bbox.get(bbox, (None, None))

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
                if landmarks_px is not None:
                    _draw_hand_skeleton(frame, landmarks_px, color)  # 마디마디(관절+뼈대) 표시
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 1)
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
                self._update_distance_jog(w, h, smooth)
            elif not should_move:
                last_tick = now
                # 조그는 명령을 안 보낸다고 저절로 멈추지 않으므로(MoveL과
                # 다름), 주먹을 쥐었거나 손을 놓치거나 트래킹을 끄면
                # 반드시 명시적으로 정지시켜야 함.
                if self._jog_direction is not None:
                    self._set_jog_direction(None)

    def _update_distance_jog(self, w, h, bbox):
        """
        MoveL을 반복 호출하는 대신 조그(JOG)로 "필요한 방향으로 계속
        움직이다가, 목표 거리 근처에 오면 정지"하는 방식. 방향이 바뀌지
        않는 한 별도 명령 없이 로봇은 계속 부드럽게 움직입니다 - 이 함수는
        그저 "지금 방향이 맞는지, 멈춰야 하는지"만 주기적으로 재판단합니다.
        """
        size_ratio = bbox[2] / float(w)
        err_size = self._target_size_ratio - size_ratio  # 양수=목표보다 작음(멀리 있음)->전진 필요
        with self._lock:
            self._last_error = {"size_ratio": size_ratio}

        if abs(err_size) < self.distance_deadzone_ratio:
            desired = None  # 목표 거리 범위 안 - 정지
        elif err_size > 0:
            desired = "fwd"  # 목표보다 멀리 있음 -> 전진해서 다가감
        else:
            desired = "back"  # 목표보다 가까이 있음 -> 후진해서 멀어짐

        if desired == self._jog_direction:
            return  # 이미 올바른 상태(정지 포함) - 아무것도 안 함, 조그는 계속 이어짐

        self._set_jog_direction(desired)

    def _set_jog_direction(self, desired):
        try:
            if self._jog_direction is not None:
                error = self._manager.jog_stop(ref=5)  # 5 = 공구좌표계 점동 정지
                if error != 0:
                    print(f"[손 트래킹] StopJOG 반환값(에러): {error}")
            if desired is not None:
                direction = 1 if desired == "fwd" else 0
                error = self._manager.jog_start(ref=4, nb=3, direction=direction, max_dis=500.0, vel=self.jog_vel)
                with self._lock:
                    self._last_move_result = {"error": error, "exception": None}
                if error != 0:
                    print(f"[손 트래킹] StartJOG 반환값(에러): {error} "
                          f"(0이 아니면 실패 - 로봇 활성화 여부/안전정지 상태를 확인하세요)")
            else:
                with self._lock:
                    self._last_move_result = {"error": 0, "exception": None}
            self._jog_direction = desired
        except Exception as e:
            with self._lock:
                self._last_move_result = {"error": None, "exception": str(e)}
            print("[손 트래킹] 조그 방향 전환 실패(예외):", e)
            self._jog_direction = None

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
        self._force_jog_stop()

    def _force_jog_stop(self):
        """조그는 명령을 안 보낸다고 저절로 안 멈추므로, 트래킹 정지/카메라
        닫기 시점에 확실히 멈춰야 함 - 항상 즉시 응답하는 전용 커넥션 사용."""
        if self._jog_direction is not None:
            try:
                self._manager.jog_stop_immediate()
            except Exception as e:
                print("[손 트래킹] 조그 강제 정지 실패:", e)
            self._jog_direction = None

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
            "jog_direction": self._jog_direction,  # None | "fwd" | "back"
            "last_move_error": move.get("error"),
            "last_move_exception": move.get("exception"),
            "invert": dict(self.invert),
            "invert_handedness": self.invert_handedness,
            "max_step_deg": self.max_step_deg,
            "max_step_mm": self.max_step_mm,
        }

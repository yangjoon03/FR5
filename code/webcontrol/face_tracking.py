"""
얼굴 위치를 보고 "로봇을 얼마나/어느 방향으로 움직여야 하는가"를 계산하는
순수 로직 모듈. OpenCV나 실제 카메라에 의존하지 않아서 카메라 없이도
단위 테스트가 가능합니다 (vision_tracker.py가 이 모듈을 감싸서 실제
카메라 프레임에 적용합니다).
"""


def bbox_center(bbox):
    x, y, w, h = bbox
    return (x + w / 2.0, y + h / 2.0)


def bbox_distance(a, b):
    ax, ay = bbox_center(a)
    bx, by = bbox_center(b)
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


class FaceLock:
    """
    여러 얼굴 후보 중 "한 사람만" 계속 추적하기 위한 상태 머신.

    - 아직 아무도 안 잡았으면: 가장 큰(=카메라에 가장 가까운) 얼굴을 새로 잡는다.
    - 이미 잡은 사람이 있으면: 이전 위치와 가장 가까운 후보를 같은 사람으로
      간주해서 위치만 갱신한다 (다른 사람이 옆에 나타나도 무시).
    - 잡은 사람이 몇 프레임 연속으로 안 보이면(예: 고개를 돌림) 놓친 것으로
      보되, 유예 프레임 동안은 마지막 위치를 유지해서 잠깐의 인식 실패에는
      흔들리지 않는다. 유예 프레임을 넘기면 놓친 것으로 확정하고, 그 다음
      부터는 다시 "가장 큰 얼굴"을 새로 찾는다.
    """

    def __init__(self, max_center_jump=140, lost_grace_frames=10):
        self.locked_bbox = None
        self.lost_count = 0
        self.max_center_jump = max_center_jump
        self.lost_grace_frames = lost_grace_frames

    def update(self, faces):
        """faces: [(x,y,w,h), ...] 이번 프레임에서 검출된 얼굴들. 반환: 추적 중인 bbox 또는 None."""
        if self.locked_bbox is not None:
            candidate = self._closest(faces, self.locked_bbox)
            if candidate is not None:
                self.locked_bbox = candidate
                self.lost_count = 0
                return self.locked_bbox
            self.lost_count += 1
            if self.lost_count > self.lost_grace_frames:
                self.locked_bbox = None
                self.lost_count = 0
            else:
                return self.locked_bbox  # 잠깐 안 보여도 이전 위치를 유지 (놓친 것으로 확정 전)

        if self.locked_bbox is None and faces:
            self.locked_bbox = max(faces, key=lambda f: f[2] * f[3])
        return self.locked_bbox

    def _closest(self, faces, ref_bbox):
        best, best_d = None, None
        for f in faces:
            d = bbox_distance(f, ref_bbox)
            if d <= self.max_center_jump and (best_d is None or d < best_d):
                best, best_d = f, d
        return best

    def reset(self):
        self.locked_bbox = None
        self.lost_count = 0


def compute_correction(frame_w, frame_h, bbox, target_size_ratio, gains, max_step_mm, invert):
    """
    현재 얼굴 위치/크기와 목표(화면 중앙, 목표 크기)의 차이를 로봇의 툴
    좌표계 기준 이동량(mm)으로 변환합니다.

    - bbox: (x,y,w,h) 픽셀 좌표 (검출된 얼굴 박스)
    - target_size_ratio: "이 정도 크기로 보이면 원하는 거리"의 기준값
      (얼굴 박스 너비 / 프레임 너비, 0~1)
    - gains: {"x":.., "y":.., "z":..} 오차 1단위당 몇 mm 움직일지 (비례 게인)
      x/y는 px당 mm, z는 size_ratio 오차당 mm
    - max_step_mm: 한 틱(주기)당 최대 이동량 - 안전을 위한 클램프
    - invert: {"x":bool,"y":bool,"z":bool} 카메라 장착 방향에 따라 부호 반전

    좌표 규약(반전 전 기준):
    - err_x_px 양수(얼굴이 화면 오른쪽) -> dx 양수 -> 툴 좌표계 +X로 이동
    - err_y_px 양수(얼굴이 화면 아래쪽) -> dy 양수 -> 툴 좌표계 +Y로 이동
    - err_size 양수(얼굴이 목표보다 작음=더 멀리 있음) -> dz 양수 -> 툴 +Z(전진)
    실제 카메라가 로봇에 어떤 방향으로 달려 있는지는 알 수 없으므로,
    실기에서 반대로 움직이면 invert로 뒤집어서 맞추면 됩니다.

    반환: dict(dx, dy, dz, size_ratio, error_x_px, error_y_px)
    """
    x, y, w, h = bbox
    if frame_w <= 0 or frame_h <= 0:
        raise ValueError("frame_w/frame_h는 0보다 커야 합니다")

    cx, cy = frame_w / 2.0, frame_h / 2.0
    face_cx, face_cy = x + w / 2.0, y + h / 2.0

    err_x_px = face_cx - cx
    err_y_px = face_cy - cy
    size_ratio = w / float(frame_w)
    err_size = target_size_ratio - size_ratio

    def clamp(v):
        return max(-max_step_mm, min(max_step_mm, v))

    dx = clamp(err_x_px * gains["x"])
    dy = clamp(err_y_px * gains["y"])
    dz = clamp(err_size * gains["z"])

    if invert.get("x"):
        dx = -dx
    if invert.get("y"):
        dy = -dy
    if invert.get("z"):
        dz = -dz

    return {
        "dx": dx, "dy": dy, "dz": dz,
        "size_ratio": size_ratio,
        "error_x_px": err_x_px, "error_y_px": err_y_px,
    }

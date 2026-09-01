"""
도형/궤적 계산 모듈
------------------------------------------------------------
웹 UI는 티치펜던트처럼 3D 공간에서 점을 직접 찍을 수 없기 때문에,
"현재 위치를 기준으로 반지름 10cm 원을 그려라", "한 변 5cm인 정삼각형을
시계방향으로 그려라" 같은 파라미터만 입력받아서, 실제 로봇에 보낼
좌표점들을 이 모듈이 계산합니다.

좌표계 규칙
- desc_pos = [x, y, z, rx, ry, rz], 단위 [mm][°] (SDK 기준)
- 이 모듈의 모든 함수는 "자세(rx,ry,rz)는 그대로 유지"하고 위치(x,y,z)만
  계산합니다. 즉 그림을 그리는 동안 툴(펜/그리퍼)의 방향은 바뀌지 않습니다.
- plane 파라미터로 어느 평면 위에서 도형을 그릴지 고릅니다.
  'XY' -> x,y를 움직이고 z 고정 (바닥과 평행한 평면, 위에서 내려다본 도형)
  'XZ' -> x,z를 움직이고 y 고정 (앞뒤/위아래 평면)
  'YZ' -> y,z를 움직이고 x 고정 (좌우/위아래 평면)
- direction: 'ccw'(반시계) 또는 'cw'(시계). 두 축을 (u,v)라고 할 때
  u축에서 v축으로 향하는 방향을 양(+)의 회전(반시계)으로 봅니다.
"""
import math

_PLANE_AXES = {
    "XY": (0, 1, 2),  # (u축 인덱스, v축 인덱스, 고정축 인덱스)
    "XZ": (0, 2, 1),
    "YZ": (1, 2, 0),
}


def _plane_indices(plane: str):
    plane = plane.upper()
    if plane not in _PLANE_AXES:
        raise ValueError("plane은 'XY', 'XZ', 'YZ' 중 하나여야 합니다: %s" % plane)
    return _PLANE_AXES[plane]


def _sign(direction: str) -> int:
    direction = direction.lower()
    if direction not in ("cw", "ccw"):
        raise ValueError("direction은 'cw' 또는 'ccw'여야 합니다: %s" % direction)
    return -1 if direction == "cw" else 1


def make_point(base_pose, u, v, plane):
    """base_pose(자세 포함 6값)를 복사해서 지정한 평면의 u,v 좌표만 바꿔 반환."""
    ui, vi, _ = _plane_indices(plane)
    p = list(base_pose)
    p[ui] = u
    p[vi] = v
    return p


def linear_offset(current_pose, axis: str, distance_mm: float, sign: str = "+"):
    """
    현재 위치에서 한 축(x/y/z) 방향으로 distance_mm 만큼 평행이동한 목표
    좌표 1개를 반환합니다. (수직/수평 이동에 사용)
    - axis: 'x' | 'y' | 'z'
    - sign: '+' | '-'
    """
    axis = axis.lower()
    idx = {"x": 0, "y": 1, "z": 2}.get(axis)
    if idx is None:
        raise ValueError("axis는 'x','y','z' 중 하나여야 합니다: %s" % axis)
    d = distance_mm if sign == "+" else -distance_mm
    target = list(current_pose)
    target[idx] += d
    return target


def rotate_offset(current_pose, axis: str, angle_deg: float, sign: str = "+"):
    """
    현재 자세에서 한 회전축(rx/ry/rz) 방향으로 angle_deg 만큼 돌린 목표
    좌표 1개를 반환합니다. (제자리 회전에 사용, 위치 x,y,z는 그대로)
    """
    axis = axis.lower()
    idx = {"rx": 3, "ry": 4, "rz": 5}.get(axis)
    if idx is None:
        raise ValueError("axis는 'rx','ry','rz' 중 하나여야 합니다: %s" % axis)
    d = angle_deg if sign == "+" else -angle_deg
    target = list(current_pose)
    target[idx] += d
    return target


def circle_points(current_pose, radius_mm: float, plane: str = "XY", direction: str = "ccw"):
    """
    현재 위치를 원 위의 한 점으로 삼아, 지정한 반지름의 완전한 원을 그리기
    위한 (경유점, 목표점) 2개를 계산합니다. robot.Circle()에 그대로 넣으면
    현재위치 -> 경유점 -> 목표점 -> (자동으로) 현재위치, 한 바퀴를 돕니다.

    원의 중심은 "현재 위치에서 u축 방향으로 radius_mm만큼 이동한 지점"으로
    잡습니다. 그 결과 현재 위치가 중심 기준 180도 지점이 되고, 나머지
    두 점을 120도 간격으로 배치해 3점으로 원을 유일하게 결정합니다.
    """
    ui, vi, _ = _plane_indices(plane)
    s = _sign(direction)
    u0, v0 = current_pose[ui], current_pose[vi]
    cu, cv = u0 + radius_mm, v0  # 원의 중심

    def point_at(angle_deg):
        rad = math.radians(angle_deg)
        u = cu + radius_mm * math.cos(rad)
        v = cv + radius_mm * math.sin(rad)
        return make_point(current_pose, u, v, plane)

    via = point_at(180 + s * 120)
    target = point_at(180 + s * 240)
    return via, target


def arc_points(current_pose, radius_mm: float, sweep_deg: float, plane: str = "XY", direction: str = "ccw"):
    """
    현재 위치를 시작점으로, 지정한 반지름의 원 위를 sweep_deg(°)만큼 도는
    호를 그리기 위한 (경유점, 목표점)을 계산합니다. robot.MoveC()에 사용.
    sweep_deg는 0보다 크고 360보다 작아야 합니다(완전한 원은 circle_points 사용).
    """
    if not (0 < sweep_deg < 360):
        raise ValueError("호(arc)의 각도는 0~360 사이여야 합니다 (360이면 원을 사용하세요): %s" % sweep_deg)
    ui, vi, _ = _plane_indices(plane)
    s = _sign(direction)
    u0, v0 = current_pose[ui], current_pose[vi]
    cu, cv = u0 + radius_mm, v0

    def point_at(angle_deg):
        rad = math.radians(angle_deg)
        u = cu + radius_mm * math.cos(rad)
        v = cv + radius_mm * math.sin(rad)
        return make_point(current_pose, u, v, plane)

    via = point_at(180 + s * sweep_deg / 2)
    target = point_at(180 + s * sweep_deg)
    return via, target


def polygon_points(current_pose, sides: int, side_length_mm: float, plane: str = "XY", direction: str = "ccw"):
    """
    현재 위치를 첫 꼭짓점으로 삼아, 정n각형의 나머지 꼭짓점들을 계산합니다.
    (SDK에 다각형 명령이 없으므로 MoveL을 여러 번 이어서 그립니다)

    반환값: 첫 꼭짓점(현재 위치)은 제외한 나머지 (sides)개의 좌표 리스트.
    순서대로 MoveL 하면 정n각형을 그리고 출발점으로 되돌아옵니다.
    """
    if sides < 3:
        raise ValueError("다각형은 변이 3개 이상이어야 합니다: %s" % sides)
    ui, vi, _ = _plane_indices(plane)
    s = _sign(direction)
    exterior = 360.0 / sides

    u, v = current_pose[ui], current_pose[vi]
    heading = 0.0  # +u축 방향을 기준(0도)으로 시작
    points = []
    for _ in range(sides):
        rad = math.radians(heading)
        u += side_length_mm * math.cos(rad)
        v += side_length_mm * math.sin(rad)
        points.append(make_point(current_pose, u, v, plane))
        heading += s * exterior
    return points


def ellipse_points(current_pose, a_mm: float, b_mm: float, plane: str = "XY", direction: str = "ccw",
                    segments: int = 36):
    """
    현재 위치를 타원 위의 한 점(긴/짧은 축 중 a_mm 쪽 끝)으로 삼아, 타원
    둘레를 segments개의 점으로 근사합니다. (SDK에 타원 명령이 없어 다각형과
    같은 방식으로 잘게 쪼갠 직선 구간으로 근사 - segments를 늘릴수록 더
    매끄러운 타원에 가까워집니다. 36 정도면 육안상 충분히 매끄럽습니다)

    - a_mm: 시작점 방향 반지름(장축 또는 단축, 그리는 시작 방향의 반경)
    - b_mm: 시작점과 수직인 방향의 반지름

    반환값: 시작점(현재 위치) 제외, 나머지 segments개의 좌표 리스트.
    순서대로 MoveL(또는 스플라인) 하면 타원을 그리고 출발점으로 돌아옵니다.
    """
    if segments < 8:
        raise ValueError("타원 근사 점 개수(segments)는 8개 이상을 권장합니다: %s" % segments)
    ui, vi, _ = _plane_indices(plane)
    s = _sign(direction)
    u0, v0 = current_pose[ui], current_pose[vi]
    cu, cv = u0 + a_mm, v0  # 중심은 현재 위치에서 a_mm만큼 u축으로 이동한 곳

    points = []
    for i in range(1, segments + 1):
        angle = 180 + s * (360.0 * i / segments)
        rad = math.radians(angle)
        u = cu + a_mm * math.cos(rad)
        v = cv + b_mm * math.sin(rad)
        points.append(make_point(current_pose, u, v, plane))
    return points

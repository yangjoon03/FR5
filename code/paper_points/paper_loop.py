"""저장한 종이 모서리 4곳으로 "사각 구역 한 바퀴" 경로(웨이포인트)를 만드는 순수 계산 모듈. 로봇을 부르지 않는다.

모서리는 1 → 2 → 3 → 4 순서(시계 또는 반시계 어느 쪽이든 한 방향)로 저장돼 있어야 한다.
- "종이에서 나오는 방향"(법선의 부호)은 베이스 +Z(위)가 아니라 **저장한 펜 자세의 펜 축(도구 +Z)의 반대 방향**으로 정한다.
  종이가 바닥에 있으면 위쪽, 세워져 있으면 로봇 쪽(펜이 있는 쪽)이 된다. 펜 축과 법선이 크게 어긋나면 방향을 확신할 수 없어 거부한다.
- 공중 한 바퀴: 모든 점을 종이에서 나오는 방향으로 air_mm 띄운 채 돈다. 펜이 종이에 닿지 않는다.
- 종이에 닿게: 모서리 1 위쪽에서 내려와 저장한 위치 그대로 4곳을 돌고, 다시 위로 들어 올린다.
펜 방향(rx, ry, rz)은 모서리 1에서 저장한 자세를 전 구간에서 유지한다.
"""
import math

MIN_SIDE_MM = 30.0          # 변 길이 범위(이 밖이면 모서리를 잘못 저장했을 가능성이 크다)
MAX_SIDE_MM = 600.0
MIN_AIR_MM = 10.0
MAX_AIR_MM = 100.0
APPROACH_LIFT_MM = 10.0     # 종이에 닿게 돌 때 접근·복귀 높이
MAX_START_DIST_MM = 250.0   # 현재 펜 위치에서 첫 접근점까지 허용 거리(첫 이동은 직선이다)
MAX_PEN_NORMAL_DEG = 60.0    # 펜 축과 종이 법선이 이보다 어긋나면 "종이에서 나오는 방향"을 확신할 수 없어 거부
WARN_PEN_NORMAL_DEG = 20.0
MAX_ORI_SPREAD_DEG = 15.0   # 저장한 네 위치에서 펜 기울기(rx, ry) 차이가 이보다 크면 거부. 펜 축 둘레 회전(rz)은 끝 위치에 영향이 없어 보지 않는다


class LoopError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def _sub(a, b): return [a[i] - b[i] for i in range(3)]
def _add(a, b): return [a[i] + b[i] for i in range(3)]
def _mul(a, k): return [x * k for x in a]
def _dot(a, b): return sum(a[i] * b[i] for i in range(3))
def _norm(a): return math.sqrt(_dot(a, a))
def _cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def tool_z_axis(rx, ry, rz):
    """펜(도구 +Z)이 가리키는 방향의 단위벡터(베이스 좌표). 자세 규칙: R = Rz(rz)·Ry(ry)·Rx(rx), 각도는 도(°).
    예) [180, 0, 0] → (0, 0, -1) (아래를 향함)"""
    a, b, c = math.radians(rx), math.radians(ry), math.radians(rz)
    x1, y1, z1 = 0.0, -math.sin(a), math.cos(a)                              # Rx(a)·e_z
    x2, y2, z2 = x1 * math.cos(b) + z1 * math.sin(b), y1, -x1 * math.sin(b) + z1 * math.cos(b)   # Ry(b)
    return [x2 * math.cos(c) - y2 * math.sin(c), x2 * math.sin(c) + y2 * math.cos(c), z2]          # Rz(c)


def rot(rx, ry, rz):
    """자세(도) → 회전행렬(행 우선 3x3). R = Rz(rz)·Ry(ry)·Rx(rx). 3번째 열이 도구 +Z(펜 축)다."""
    a, b, c = math.radians(rx), math.radians(ry), math.radians(rz)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
            [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
            [-sb, cb * sa, cb * ca]]


def euler(R):
    """회전행렬 → 자세(도). rot() 의 반대."""
    ry = math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0]))))
    return [math.degrees(math.atan2(R[2][1], R[2][2])), ry, math.degrees(math.atan2(R[1][0], R[0][0]))]


def _col(R, k): return [R[0][k], R[1][k], R[2][k]]


def _tool_off(R, off):
    """도구 좌표계 오프셋 off=(x,y,z) 를 베이스 좌표로: R·off"""
    return [R[i][0] * off[0] + R[i][1] * off[1] + R[i][2] * off[2] for i in range(3)]


def tcp_pose(info, tip):
    """펜 끝 위치(tip, 베이스 좌표) → 로봇에 보낼 TCP 자세. TCP 가 펜 끝이 아니면 도구 오프셋(pen_off)만큼 물린다."""
    ori = info["orientation"]
    off = _tool_off(rot(*ori), info.get("pen_off", [0.0, 0.0, info.get("pen_mm", 0.0)]))
    return [round(x, 3) for x in _sub(tip, off)] + list(ori)


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _segments_cross(a, b, c, d):
    """평면 위 두 선분 ab, cd 가 서로 가로지르는지."""
    def ccw(p, q, r): return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return ccw(a, c, d) * ccw(b, c, d) < 0 and ccw(a, b, c) * ccw(a, b, d) < 0


def analyze(points, pen_mm=0.0, tip_tcp=False, align=False, pen_x=0.0, pen_y=0.0):
    """저장된 4곳(점 4개, 없으면 None)을 검사하고 종이 평면 정보를 돌려준다.
    pen_mm(+pen_x, pen_y): 저장한 TCP 에서 펜 끝까지 도구 좌표계 기준 오프셋(z 가 펜 축 방향 거리, x·y 는 옆으로 나온 정도). 모두 0 이면 TCP 가 펜 끝이라고 본다. 모서리를 저장할 때 펜이 조금씩 다르게 기울어도
    펜 끝 위치로 환산해서 평면을 만든다. tip_tcp=True 는 도구 TCP 가 이미 펜 끝에 설정되어 있다는 뜻(pen_mm=0 이 맞다).
    align: 그리는 동안 펜을 종이 평면에 수직으로 세운다(펜 축 둘레 회전은 모서리 1을 유지). 펜 끝 위치를 알 때만 쓸 수 있다."""
    missing = [i + 1 for i, p in enumerate(points) if not p]
    if missing:
        raise LoopError("모서리 " + ", ".join(map(str, missing)) + "을(를) 먼저 저장해 주세요.")
    poses = [p["pose"] for p in points]
    if not 0.0 <= pen_mm <= 500.0:
        raise LoopError("펜 끝까지 길이는 0~500mm 사이여야 해요.")
    off = [float(pen_x), float(pen_y), float(pen_mm)]
    if math.sqrt(sum(v * v for v in off)) > 500.0:
        raise LoopError("펜 끝 오프셋이 너무 커요(500mm 이하).")
    known = tip_tcp or any(abs(v) > 1e-9 for v in off)
    if align and not known:
        raise LoopError("펜을 수직으로 세우려면 펜 끝 위치를 알아야 해요. '도구 TCP 가 이미 펜 끝'을 체크하거나, 펜 끝까지 길이(mm)를 입력해 주세요.")
    P = [_add(q[:3], _tool_off(rot(*q[3:6]), off)) for q in poses]      # 저장한 TCP → 펜 끝 위치
    sides = [_norm(_sub(P[(i + 1) % 4], P[i])) for i in range(4)]
    for i, s in enumerate(sides):
        if not MIN_SIDE_MM <= s <= MAX_SIDE_MM:
            raise LoopError(f"모서리 {i + 1}→{(i + 1) % 4 + 1} 거리가 {s:.0f}mm예요(허용 {MIN_SIDE_MM:.0f}~{MAX_SIDE_MM:.0f}mm). 모서리 위치를 다시 저장해 주세요.")
    n = _cross(_sub(P[2], P[0]), _sub(P[3], P[1]))        # 두 대각선의 외적 = 종이 법선
    length = _norm(n)
    if length < 1e-6:
        raise LoopError("네 점이 한 직선에 가깝거나 저장 순서가 꼬여 있어요. 모서리를 1→2→3→4 순서로 한 방향(시계 또는 반시계)으로 다시 저장해 주세요.")
    n = _mul(n, 1.0 / length)
    # 법선의 부호: 종이에서 펜 쪽으로 나오는 방향 = 펜 축(도구 +Z)의 반대 방향
    out = _mul(tool_z_axis(*poses[0][3:6]), -1.0)
    if _dot(n, out) < 0:
        n = _mul(n, -1.0)
    pen_angle = math.degrees(math.acos(max(-1.0, min(1.0, _dot(n, out)))))
    if pen_angle > MAX_PEN_NORMAL_DEG:
        raise LoopError(f"펜 축이 종이와 거의 수직이 아니에요(법선과 {pen_angle:.0f}° 어긋남, 허용 {MAX_PEN_NORMAL_DEG:.0f}°). "
                        "종이에서 어느 쪽이 펜 쪽인지 확신할 수 없어요. 펜을 종이에 수직에 가깝게 세워 다시 저장해 주세요.")
    center = _mul([sum(p[k] for p in P) for k in range(3)], 0.25)
    flatness = max(abs(_dot(_sub(p, center), n)) for p in P)

    # 순서 검사: 종이 평면에 투영해서 마주 보는 두 변이 가로지르면 저장 순서가 꼬인 것이다
    u = _sub(P[1], P[0]); u = _sub(u, _mul(n, _dot(u, n))); u = _mul(u, 1.0 / _norm(u))
    v = _cross(n, u)
    xy = [(_dot(_sub(p, P[0]), u), _dot(_sub(p, P[0]), v)) for p in P]
    if _segments_cross(xy[0], xy[1], xy[2], xy[3]) or _segments_cross(xy[1], xy[2], xy[3], xy[0]):
        raise LoopError("저장 순서가 꼬여 있어요. 모서리를 1→2→3→4 순서로 한 방향(시계 또는 반시계)으로 저장해 주세요.")

    angles = []
    for i in range(4):
        a, b, c = P[i - 1], P[i], P[(i + 1) % 4]
        e1, e2 = _sub(a, b), _sub(c, b)
        angles.append(math.degrees(math.acos(max(-1.0, min(1.0, _dot(e1, e2) / (_norm(e1) * _norm(e2)))))))

    base = [float(x) for x in poses[0][3:6]]
    align_deg = 0.0
    if align:
        z = _mul(n, -1.0)                                      # 펜 축 = 종이를 향하는 방향
        x0 = _col(rot(*base), 0)
        x = _sub(x0, _mul(z, _dot(x0, z)))
        if _norm(x) < 1e-6:
            x = _col(rot(*base), 1)
            x = _sub(x, _mul(z, _dot(x, z)))
        x = _mul(x, 1.0 / _norm(x))
        y = _cross(z, x)
        old_axis = tool_z_axis(*base)
        align_deg = math.degrees(math.acos(max(-1.0, min(1.0, _dot(old_axis, z)))))
        base = euler([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]])
    ori1 = poses[0][3:6]
    spread = max(abs(_wrap(q[3 + k] - ori1[k])) for q in poses for k in range(2))   # rx, ry(펜 기울기)만. rz 는 펜 자기 축 회전
    if spread > MAX_ORI_SPREAD_DEG:
        raise LoopError(f"저장한 네 위치에서 펜 기울기가 {spread:.0f}° 달라요(허용 {MAX_ORI_SPREAD_DEG:.0f}°). 펜을 같은 기울기로 세워서 다시 저장해 주세요.")

    warnings = []
    if pen_angle > WARN_PEN_NORMAL_DEG:
        warnings.append(f"펜 축이 종이 법선과 {pen_angle:.0f}° 어긋나 있어요(수직에 가까울수록 좋아요).")
    if flatness > 5.0:
        warnings.append(f"네 점이 한 평면에서 {flatness:.1f}mm 벗어나 있어요(종이가 기울었거나 저장 높이가 달라요).")
    if any(abs(a - 90.0) > 20.0 for a in angles):
        warnings.append("모서리 각도가 직각에서 20° 넘게 벗어나요: " + ", ".join(f"{a:.0f}°" for a in angles))
    if spread > 5.0:
        warnings.append(f"저장한 펜 기울기가 최대 {spread:.0f}° 달라요." + (" 펜 끝 위치로 환산해서 평면을 만들었어요." if known else " 펜 끝 위치를 모르는 상태라 이 차이가 위치 오차가 돼요(펜 끝까지 길이를 입력하면 보정돼요)."))
    if not known:
        warnings.append("펜 끝 위치(도구 TCP)를 모르는 상태예요. TCP 가 펜 끝이 아니면 저장 때 펜 기울기 차이가 그대로 높이·위치 오차가 돼요.")
    if align:
        warnings.append(f"펜을 종이에 수직으로 세워서 그려요(저장한 자세에서 {align_deg:.1f}° 보정).")
    return {"P": P, "normal": n, "sides_mm": [round(s, 1) for s in sides], "flatness_mm": round(flatness, 2),
            "angles_deg": [round(a, 1) for a in angles], "orientation": [float(x) for x in base],
            "ori_spread_deg": round(spread, 1), "pen_normal_deg": round(pen_angle, 1), "pen_mm": float(pen_mm), "pen_off": off, "tip_tcp": bool(tip_tcp),
            "align": bool(align), "align_deg": round(align_deg, 1), "elevation_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, n[2])))), 1),
            "pen_axis": [round(x, 3) for x in tool_z_axis(*poses[0][3:6])], "warnings": warnings}


def waypoints(info, mode, air_mm=30.0):
    """한 바퀴 이동 목록: [(이름, [x,y,z,rx,ry,rz]), ...]. mode = air | draw."""
    if mode not in ("air", "draw"):
        raise LoopError("방식을 골라 주세요(공중 한 바퀴 / 종이에 닿게).")
    P, n, ori = info["P"], info["normal"], info["orientation"]

    def at(i, up):
        return tcp_pose(info, _add(P[i], _mul(n, up)))

    if mode == "air":
        if not MIN_AIR_MM <= air_mm <= MAX_AIR_MM:
            raise LoopError(f"공중 높이는 {MIN_AIR_MM:.0f}~{MAX_AIR_MM:.0f}mm 사이여야 해요.")
        return [("접근 (모서리 1 위)", at(0, air_mm)), ("모서리 2 위", at(1, air_mm)), ("모서리 3 위", at(2, air_mm)),
                ("모서리 4 위", at(3, air_mm)), ("모서리 1 위 (한 바퀴 끝)", at(0, air_mm))]
    return [("접근 (모서리 1 위)", at(0, APPROACH_LIFT_MM)), ("모서리 1 (내려오기)", at(0, 0.0)), ("→ 모서리 2", at(1, 0.0)),
            ("→ 모서리 3", at(2, 0.0)), ("→ 모서리 4", at(3, 0.0)), ("→ 모서리 1 (한 바퀴 끝)", at(0, 0.0)),
            ("들어 올리기", at(0, APPROACH_LIFT_MM))]


def check_start(current_pose, first_pose):
    """현재 펜 위치가 첫 접근점에서 너무 멀면 거부한다(첫 이동은 직선이라 사이 경로를 보장할 수 없다)."""
    d = math.dist(current_pose[:3], first_pose[:3])
    if d > MAX_START_DIST_MM:
        raise LoopError(f"펜이 첫 접근점에서 {d:.0f}mm 떨어져 있어요. 첫 이동이 곧장 직선으로 가므로, 펜을 모서리 1 근처"
                        f"({MAX_START_DIST_MM:.0f}mm 이내)로 옮긴 뒤 시작해 주세요.")
    return d

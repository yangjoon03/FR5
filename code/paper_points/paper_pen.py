"""펜 끝 위치는 그대로 두고 손목 자세(펜 기울기)만 바꾸는 이동 계산. 로봇을 부르지 않는다.

펜 끝 오프셋(도구 좌표계 기준 off)을 알면, 현재 TCP 자세 (p, R) 의 펜 끝은 tip = p + R·off 다.
회전 R → R' 로 바꿔도 tip 을 고정하려면 TCP 를 p' = tip − R'·off 로 옮기면 된다.
중간 자세도 같은 식(R 을 조금씩 돌린 R_s)으로 만들어서 한 번에 이어 보내므로 펜 끝은 수십 분의 1mm 안에서만 움직인다.
"""
import math

import paper_loop as L

STEP_DEG = 2.0           # 중간 자세 간격(이만큼씩 회전)
MAX_DEG = 60.0           # 한 번에 돌릴 수 있는 최대 각도
MAX_TCP_TRAVEL_MM = 450.0
MAX_TIP_DEV_MM = 0.5     # 계산상 펜 끝이 이보다 많이 움직이면 거부


class PenError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _T(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _rodrigues(axis, ang):
    x, y, z = axis
    c, s, C = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def _log(Q):
    """회전행렬 → (단위 축, 각도 rad)"""
    cos = max(-1.0, min(1.0, (Q[0][0] + Q[1][1] + Q[2][2] - 1) / 2))
    ang = math.acos(cos)
    if ang < 1e-9:
        return [1.0, 0.0, 0.0], 0.0
    if math.pi - ang < 1e-6:                                     # 180° 근처(이 도구에서는 쓰지 않는다)
        raise PenError("회전이 180° 에 가까워서 계산할 수 없어요.")
    ax = [Q[2][1] - Q[1][2], Q[0][2] - Q[2][0], Q[1][0] - Q[0][1]]
    n = math.sqrt(sum(v * v for v in ax))
    return [v / n for v in ax], ang


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def target_rotation(R, mode, normal, direction=None, deg=0.0, u=None, v=None):
    """현재 회전 R 에서 목표 회전 R'. mode: perpendicular(펜을 판에 수직으로) | tilt(펜 축을 판 안의 방향 쪽으로 deg 만큼)."""
    a = [R[0][2], R[1][2], R[2][2]]                              # 현재 펜 축
    if mode == "perpendicular":
        z = [-x for x in normal]
        x0 = [R[0][0], R[1][0], R[2][0]]
        x = [x0[i] - z[i] * sum(x0[k] * z[k] for k in range(3)) for i in range(3)]
        if math.sqrt(sum(c * c for c in x)) < 1e-6:
            x0 = [R[0][1], R[1][1], R[2][1]]
            x = [x0[i] - z[i] * sum(x0[k] * z[k] for k in range(3)) for i in range(3)]
        x = _unit(x)
        y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
        return [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    if mode == "tilt":
        d = {"+u": u, "-u": [-c for c in u], "+v": v, "-v": [-c for c in v]}.get(direction)
        if d is None:
            raise PenError("기울일 방향(앞·뒤·좌·우)을 골라 주세요.")
        k = [a[1] * d[2] - a[2] * d[1], a[2] * d[0] - a[0] * d[2], a[0] * d[1] - a[1] * d[0]]   # 펜 축을 d 쪽으로 돌리는 회전축
        if math.sqrt(sum(c * c for c in k)) < 1e-6:
            raise PenError("펜 축이 그 방향과 나란해서 기울일 수 없어요.")
        return _mat_mul(_rodrigues(_unit(k), math.radians(deg)), R)
    raise PenError("동작을 골라 주세요(수직으로 세우기 / 기울이기).")


def plan(tcp, off, R_target):
    """tcp=[x,y,z,rx,ry,rz], off=[x,y,z](도구 좌표계 펜 끝 오프셋) → 펜 끝을 고정한 채 R_target 으로 가는 TCP 자세 목록.
    반환: {"poses": [...], "tip": [...], "angle_deg", "tcp_travel_mm", "tip_dev_mm"}"""
    R = L.rot(*tcp[3:6])
    tip = [tcp[i] + L._tool_off(R, off)[i] for i in range(3)]
    axis, ang = _log(_mat_mul(R_target, _T(R)))
    deg = math.degrees(ang)
    if deg > MAX_DEG:
        raise PenError(f"한 번에 {deg:.0f}° 를 돌려야 해서 위험해요(최대 {MAX_DEG:.0f}°). 나눠서 해 주세요.")
    n = max(2, math.ceil(deg / STEP_DEG))
    poses, dev, travel, prev = [], 0.0, 0.0, tcp[:3]
    for s in range(1, n + 1):
        Rs = _mat_mul(_rodrigues(axis, ang * s / n), R)
        o = L._tool_off(Rs, off)
        p = [tip[i] - o[i] for i in range(3)]
        e = L.euler(Rs)
        poses.append([round(c, 3) for c in p] + [round(c, 4) for c in e])
        travel += math.dist(prev, p)
        prev = p
        tip_s = [p[i] + L._tool_off(L.rot(*e), off)[i] for i in range(3)]       # 반올림 후 실제 펜 끝
        dev = max(dev, math.dist(tip_s, tip))
    if travel > MAX_TCP_TRAVEL_MM:
        raise PenError(f"펜 끝을 고정하려면 손목(TCP)이 {travel:.0f}mm 움직여야 해서 위험해요(최대 {MAX_TCP_TRAVEL_MM:.0f}mm).")
    if dev > MAX_TIP_DEV_MM:
        raise PenError(f"계산상 펜 끝이 {dev:.2f}mm 움직여요. 펜 끝 오프셋이 맞는지 확인해 주세요.")
    return {"poses": poses, "tip": [round(c, 2) for c in tip], "angle_deg": round(deg, 1), "tcp_travel_mm": round(travel, 1),
            "tip_dev_mm": round(dev, 3), "steps": n}


def pen_angle_deg(tcp, normal):
    """펜 축이 판에 수직(= −법선)에서 얼마나 기울었는지(°)"""
    z = L.tool_z_axis(*tcp[3:6])
    return math.degrees(math.acos(max(-1.0, min(1.0, -sum(z[i] * normal[i] for i in range(3))))))

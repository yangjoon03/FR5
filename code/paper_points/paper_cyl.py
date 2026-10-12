"""저장한 사각 구역 안, 종이 위 공중에 원통(위·아래 원 + 세로선 4개)을 한 번에 이어 그리는 경로를 만드는 순수 계산 모듈.
로봇을 부르지 않는다. 종이 평면 정보는 paper_loop.analyze() 의 결과를 쓴다.

종이 평면 기준 오른손 좌표계 (u, v, n): u = 모서리 1→2 방향(평면에 투영), v = n × u, n = 종이에서 펜 쪽으로 나오는 방향(paper_loop.analyze 가 펜 축의 반대 방향으로 정한다).
원은 위에서 볼 때 반시계(u→v 방향)로 돈다. 원통 축은 n 방향이다.

경로(한 번에 이어서, 같은 선을 다시 지나는 곳이 있다):
  아래 원 한 바퀴 → 0°에서 위로 → 위 원 한 바퀴 → 위 0°→90° → 90°에서 아래로 → 아래 90°→180° → 180°에서 위로
  → 위 180°→270° → 270°에서 아래로
  = 아래 원, 위 원, 세로선 4개(0°, 90°, 180°, 270°)가 모두 그려진다.
"""
import math

import paper_loop as L

DEFAULT_R_MM = 40.0
DEFAULT_Z0_MM = 20.0        # 원통 아래 원의 종이 위 높이
DEFAULT_H_MM = 50.0         # 원통 높이
MIN_Z0_MM = 10.0            # 종이에서 이 높이보다 낮게는 가지 않는다
MAX_TOP_MM = 150.0          # 원통 윗면의 종이 위 높이 상한
MIN_R_MM = 10.0
EDGE_MARGIN_MM = 15.0       # 원이 사각 구역 가장자리에서 떨어져야 하는 거리
MIN_SEG = 12                # 원 한 바퀴를 나누는 점 수 범위
MAX_SEG = 72
BLEND_FRAC = 0.4            # blendR 은 가장 짧은 구간의 이 비율 이하(구간보다 크면 컨트롤러가 거부한다: 코드 14)
BLEND_CAP_MM = 3.0


class CylError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def _sub(a, b): return [a[i] - b[i] for i in range(3)]
def _add(a, b): return [a[i] + b[i] for i in range(3)]
def _mul(a, k): return [x * k for x in a]
def _dot(a, b): return sum(a[i] * b[i] for i in range(3))
def _norm(a): return math.sqrt(_dot(a, a))
def _cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def frame(info):
    """종이 평면 좌표계와 사각 구역 범위. 원점은 모서리 1."""
    P, n = info["P"], info["normal"]
    u = _sub(P[1], P[0]); u = _sub(u, _mul(n, _dot(u, n))); u = _mul(u, 1.0 / _norm(u))
    v = _cross(n, u)
    uv = [(_dot(_sub(p, P[0]), u), _dot(_sub(p, P[0]), v)) for p in P]
    us, vs = [q[0] for q in uv], [q[1] for q in uv]
    # 평면 위에서 네 점의 평균 위치를 중심으로 쓴다(종이 높이는 평면 위로 정의한다)
    cu, cv = sum(us) / 4, sum(vs) / 4
    plane_z = sum(_dot(_sub(p, P[0]), n) for p in P) / 4
    return {"info": info, "o": P[0], "u": u, "v": v, "n": n, "umin": min(us), "umax": max(us), "vmin": min(vs), "vmax": max(vs),
            "cu": cu, "cv": cv, "plane_z": plane_z}


def available_radius(fr):
    return min(fr["cu"] - fr["umin"], fr["umax"] - fr["cu"], fr["cv"] - fr["vmin"], fr["vmax"] - fr["cv"]) - EDGE_MARGIN_MM


def _at(fr, ori, a_u, a_v, h):
    """종이 평면 좌표 (a_u, a_v) 와 종이 위 높이 h → 로봇 자세."""
    pos = _add(_add(_add(fr["o"], _mul(fr["u"], a_u)), _mul(fr["v"], a_v)), _mul(fr["n"], fr["plane_z"] + h))
    return L.tcp_pose(fr["info"], pos)                 # 펜 끝 위치 → TCP 자세(펜 길이·수직 보정 반영)


def route(info, r=DEFAULT_R_MM, z0=DEFAULT_Z0_MM, height=DEFAULT_H_MM, segments=36):
    """원통 경로. 반환: {"start": 자세, "chain": [자세...], "blend_mm": float, "parts": [...], "info": {...}}"""
    segments = int(segments)
    if not MIN_SEG <= segments <= MAX_SEG or segments % 4:
        raise CylError(f"원을 나누는 점 수는 4의 배수로 {MIN_SEG}~{MAX_SEG} 사이여야 해요.")
    fr = frame(info)
    avail = available_radius(fr)
    if not MIN_R_MM <= r <= avail:
        raise CylError(f"반지름은 {MIN_R_MM:.0f}mm 이상, 사각 구역 안에 들어오는 {max(avail, 0):.0f}mm 이하여야 해요(가장자리 여유 {EDGE_MARGIN_MM:.0f}mm 포함).")
    if z0 < MIN_Z0_MM or height <= 0 or z0 + height > MAX_TOP_MM:
        raise CylError(f"원통은 종이 위 {MIN_Z0_MM:.0f}mm 이상에서 시작하고, 윗면이 {MAX_TOP_MM:.0f}mm를 넘으면 안 돼요.")
    z1 = z0 + height
    ori = info["orientation"]

    def circ(theta_deg, h):
        t = math.radians(theta_deg)
        return _at(fr, ori, fr["cu"] + r * math.cos(t), fr["cv"] + r * math.sin(t), h)

    step = 360.0 / segments
    q = segments // 4                                            # 90° 만큼의 점 수
    start = circ(0.0, z0)
    chain, parts = [], []

    def add(label, pts):
        parts.append((label, len(chain), len(chain) + len(pts)))
        chain.extend(pts)

    add("아래 원", [circ(k * step, z0) for k in range(1, segments + 1)])                 # 0° → 360°
    add("세로선 0° (위로)", [circ(0.0, z1)])
    add("위 원", [circ(k * step, z1) for k in range(1, segments + 1)])
    add("위 0°→90°", [circ(k * step, z1) for k in range(1, q + 1)])
    add("세로선 90° (아래로)", [circ(90.0, z0)])
    add("아래 90°→180°", [circ(90.0 + k * step, z0) for k in range(1, q + 1)])
    add("세로선 180° (위로)", [circ(180.0, z1)])
    add("위 180°→270°", [circ(180.0 + k * step, z1) for k in range(1, q + 1)])
    add("세로선 270° (아래로)", [circ(270.0, z0)])

    pts = [start] + chain
    seg_lens = [math.dist(pts[i][:3], pts[i + 1][:3]) for i in range(len(pts) - 1)]
    shortest = min(seg_lens)
    blend = round(min(BLEND_CAP_MM, BLEND_FRAC * shortest), 2)
    return {"start": start, "chain": chain, "blend_mm": blend, "parts": parts,
            "info": {"radius_mm": r, "z0_mm": z0, "z1_mm": z1, "segments": segments, "points": len(chain) + 1,
                     "path_mm": round(sum(seg_lens), 1), "shortest_segment_mm": round(shortest, 2),
                     "max_radius_mm": round(avail, 1)}}

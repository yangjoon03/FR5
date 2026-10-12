"""SVG 에서 뽑은 선(폴리라인)을 저장한 종이 구역에 맞춰 로봇이 이어서 그릴 경로(자세 목록)로 바꾸는 순수 계산 모듈. 로봇을 부르지 않는다.

좌표: paper_cyl.frame() 의 종이 평면 좌표 (u, v, n). 종이를 펜 쪽(n 쪽)에서 내려다볼 때 u 는 오른쪽, v 는 위쪽이다.
그림은 기본적으로 SVG 가 화면에 보이는 그대로(위가 v+, 오른쪽이 u+) 놓는다. 어느 모서리가 위쪽인지는 저장 순서에 달려 있으므로 90° 단위 회전을 고를 수 있다.

경로(한 번에 이어서 보낸다): 펜 든 높이에서 첫 획 위로 → 내리기 → 획 → 올리기 → 다음 획 위로 → 내리기 → … → 올리기.
- "draw": 획은 종이 높이 0(저장한 모서리와 같은 높이), 펜 든 높이는 lift_mm.
- "air": 획도 air_mm 높이에서 그린다(펜이 종이에 닿지 않는 연습). 펜 든 높이는 air_mm + lift_mm.
"""
import math

import paper_cyl as CY

MARGIN_MM = 10.0
MIN_DRAW_MM = 10.0         # 그림 전체 크기가 이보다 작으면 거부
MIN_LIFT_MM = 3.0
MAX_LIFT_MM = 30.0
MIN_AIR_MM = 10.0
MAX_AIR_MM = 100.0
TOL_MM = 0.15              # 선 단순화 오차(RDP). 작을수록 원본 모양에 가깝다
MIN_SEG_MM = 1.5           # 이어지는 점 사이 간격(곡선은 이 간격으로 고르게 찍는다. 너무 촘촘하면 blend 가 못 쓰인다)
MIN_STROKE_MM = 0.6        # 이보다 짧은 획은 점으로 보고 버린다
CORNER_DEG = 35.0          # 이 이상 꺾이면 모서리로 보고 그 점을 그대로 남긴다
JOIN_MM = 1.5              # 이전 획 끝과 이 거리 안에서 시작하는 획은 펜을 들지 않고 이어 그린다
BLEND_FRAC = 0.4           # blendR 은 이웃한 구간 중 짧은 쪽의 이 비율 이하(구간보다 크면 컨트롤러가 거부한다)
BLEND_CAP_MM = 3.0
PEN_DOWN_BLEND_MM = 1.0     # 펜을 내리는 꼭짓점의 평활 반경(0 이면 거기서 감속·정지하는 것으로 보여서 작게나마 둔다)
DOT_LEN_MM = 1.0            # 점으로 남기는 아주 짧은 선의 길이(keep_dots)
CHAIN_MIN_GAP_MM = 0.4      # 곡선 모드의 이음 이동에서 이보다 가까운 조절점은 건너뛴다
REACH_CAP_MM = 4.0
REACH_MIN_MM = 1.5
MAX_POINTS = 60000
CURVE_STEP_MM = 5.0         # 곡선 모드에서 스플라인 조절점의 기본 간격(같은 간격). 많이 휘면 오차 SPLINE_DEV_MM 에 맞춰 줄인다
SPLINE_DEV_MM = 0.25        # 스플라인 곡선이 원래 모양에서 벗어나도 되는 최대 거리
SPLINE_MIN_STEP_MM = 2.5    # 조절점 간격 하한
CURVE_SPLIT_DEG = 65.0      # 곡선 모드에서 이 이상 꺾이면 거기서 스플라인을 나눈다(뾰족한 모서리)
CURVE_MIN_PIECE_MM = 8.0    # 나뉜 조각이 이보다 짧아지면 나누지 않고 한 스플라인으로 둔다(아주 짧은 스플라인은 로봇이 안 움직이는 일이 있었다)
SPLINE_MIN_MM = 8.0         # 조각 전체 길이가 이보다 짧으면 스플라인 대신 짧은 직선 이동으로 간다
CURVE_DENSE_MM = 0.3        # 곡선 모드에서 먼저 선을 이 간격으로 촘촘히 고르게 찍어 놓고 다듬는다
CURVE_CORNER_WINDOW_MM = 1.5
CURVE_SMOOTH_PTS = 3        # 조각 안에서 앞뒤 이만큼(점 수)의 평균으로 계단 흔들림을 없앤다
ROTATIONS = (0, 90, 180, 270)


class DrawError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def _rdp(pts, tol):
    """Ramer–Douglas–Peucker 단순화(반복 방식)."""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay = pts[a]; bx, by = pts[b]
        dx, dy = bx - ax, by - ay
        den = math.hypot(dx, dy)
        best, idx = -1.0, -1
        for i in range(a + 1, b):
            px, py = pts[i]
            d = math.hypot(px - ax, py - ay) if den < 1e-12 else abs(dx * (ay - py) - dy * (ax - px)) / den
            if d > best:
                best, idx = d, i
        if best > tol:
            keep[idx] = True
            stack += [(a, idx), (idx, b)]
    return [p for p, k in zip(pts, keep) if k]


def _resample(pts, step, corner_deg=CORNER_DEG):
    """선을 모양 그대로 일정한 간격(step)으로 다시 찍는다. 모서리(꺾임이 corner_deg 이상)는 그대로 남긴다.
    점을 솎아 내는 방식은 곡선이 각진 다각형이 되므로, 곡선은 호 길이 기준으로 고르게 나눈다."""
    cum = [0.0]
    for i in range(len(pts) - 1):
        cum.append(cum[-1] + math.dist(pts[i], pts[i + 1]))
    total = cum[-1]
    if total <= step:
        return [pts[0], pts[-1]]

    def at(s):
        s = min(max(s, 0.0), total)
        lo, hi = 0, len(pts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = cum[hi] - cum[lo]
        t = 0.0 if seg < 1e-12 else (s - cum[lo]) / seg
        return (pts[lo][0] + t * (pts[hi][0] - pts[lo][0]), pts[lo][1] + t * (pts[hi][1] - pts[lo][1]))

    cand = []
    for i in range(1, len(pts) - 1):
        s = cum[i]
        if s < step or total - s < step:
            continue
        a, b, c = at(s - step), pts[i], at(s + step)
        v1, v2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))
        if ang >= corner_deg:
            cand.append((ang, s))
    marks = []
    for ang, s in sorted(cand, reverse=True):                 # 센 모서리부터, 가까운 약한 모서리는 버린다
        if all(abs(s - m) >= step for m in marks):
            marks.append(s)
    marks = [0.0] + sorted(marks) + [total]
    out = [at(0.0)]
    for a, b in zip(marks, marks[1:]):
        n = max(1, round((b - a) / step))
        out += [at(a + (b - a) * k / n) for k in range(1, n + 1)]
    return out


def _length(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _inside(poly, p, margin):
    """p 가 볼록 사각형 poly 안에 있고 모든 변에서 margin 이상 떨어져 있는지."""
    sign = 0
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        ex, ey = b[0] - a[0], b[1] - a[1]
        cross = ex * (p[1] - a[1]) - ey * (p[0] - a[0])
        s = 1 if cross > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
        if abs(cross) / math.hypot(ex, ey) < margin:
            return False
    return True


def plan(info, polylines, *, rotate=0, mirror=False, margin=MARGIN_MM, scale_pct=100.0, lift=8.0, mode="air", air_mm=30.0,
         tol=TOL_MM, min_seg=MIN_SEG_MM, min_stroke=MIN_STROKE_MM, curve=False, curve_step=CURVE_STEP_MM, first_n=0, keep_dots=False):
    if mode not in ("air", "draw"):
        raise DrawError("방식을 골라 주세요(공중 연습 / 종이에 그리기).")
    if rotate not in ROTATIONS:
        raise DrawError("회전은 0, 90, 180, 270 중에서 골라 주세요.")
    if not MIN_LIFT_MM <= lift <= MAX_LIFT_MM:
        raise DrawError(f"펜 드는 높이는 {MIN_LIFT_MM:.0f}~{MAX_LIFT_MM:.0f}mm 사이여야 해요.")
    if mode == "air" and not MIN_AIR_MM <= air_mm <= MAX_AIR_MM:
        raise DrawError(f"공중 높이는 {MIN_AIR_MM:.0f}~{MAX_AIR_MM:.0f}mm 사이여야 해요.")
    if not 5.0 <= margin <= 100.0:
        raise DrawError("여백은 5~100mm 사이여야 해요.")
    if not 10.0 <= scale_pct <= 100.0:
        raise DrawError("크기는 10~100% 사이여야 해요.")
    if not 0.02 <= tol <= 2.0:
        raise DrawError("선 단순화 오차는 0.02~2mm 사이여야 해요.")
    if not 2.0 <= curve_step <= 15.0:
        raise DrawError("곡선 조절점 최대 간격은 2~15mm 사이여야 해요.")
    if first_n < 0:
        raise DrawError("처음 몇 획만 그릴지는 0 이상이어야 해요(0 = 전부).")
    step = CURVE_DENSE_MM if curve else min_seg
    if not 0.3 <= min_seg <= 10.0:
        raise DrawError("점 사이 간격은 0.3~10mm 사이여야 해요.")
    if not 0.0 <= min_stroke <= 30.0:
        raise DrawError("버릴 짧은 선 길이는 0~30mm 사이여야 해요.")

    fr = CY.frame(info)
    o, cu, cv = fr["o"], fr["cu"], fr["cv"]
    quad = [(CY._dot(CY._sub(p, o), fr["u"]), CY._dot(CY._sub(p, o), fr["v"])) for p in info["P"]]

    # 1) 그림 좌표 → 위가 +, 중심이 원점인 좌표(회전·좌우 반전 적용)
    xs = [p[0] for pl in polylines for p in pl]; ys = [p[1] for pl in polylines for p in pl]
    ox, oy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    c, s = round(math.cos(math.radians(rotate))), round(math.sin(math.radians(rotate)))
    strokes = []
    for pl in polylines:
        out = []
        for x, y in pl:
            a, b = (x - ox) * (-1 if mirror else 1), -(y - oy)
            out.append((c * a - s * b, s * a + c * b))
        strokes.append(out)
    xs = [p[0] for st in strokes for p in st]; ys = [p[1] for st in strokes for p in st]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    if max(w, h) < 1e-9:
        raise DrawError("그림의 크기가 0이에요.")

    # 2) 구역 안에 들어가는 가장 큰 배율(여백 포함)
    uext = max(q[0] for q in quad) - min(q[0] for q in quad)
    vext = max(q[1] for q in quad) - min(q[1] for q in quad)
    k = min((uext - 2 * margin) / max(w, 1e-9), (vext - 2 * margin) / max(h, 1e-9))
    if k <= 0:
        raise DrawError("여백 때문에 그릴 공간이 없어요. 여백을 줄여 주세요.")
    k *= scale_pct / 100.0
    for _ in range(120):
        placed = [[(cu + k * x, cv + k * y) for x, y in st] for st in strokes]
        if all(_inside(quad, p, margin) for st in placed for p in st):
            break
        k *= 0.97
    else:
        raise DrawError("그림이 사각 구역 안에 들어가지 않아요. 여백을 줄이거나 모서리를 다시 저장해 주세요.")
    if k * max(w, h) < MIN_DRAW_MM:
        raise DrawError(f"그림이 너무 작아요(가장 긴 쪽 {k * max(w, h):.1f}mm). 구역이나 여백을 확인해 주세요.")

    # 3) 단순화·솎기, 너무 짧은 획 버리기
    cleaned, dropped = [], 0
    for st in placed:
        dedup = [st[0]]
        for p in st[1:]:
            if math.dist(p, dedup[-1]) > 1e-6:
                dedup.append(p)
        if len(dedup) < 2:
            dropped += 1
            continue
        st2 = _resample(_rdp(dedup, tol), step)
        if len(st2) < 2 or _length(st2) < max(min_stroke, 0.5):
            if keep_dots:                                      # 점·아주 짧은 선도 버리지 않고 짧은 선분(점 찍기)으로 남긴다
                a, b = dedup[0], dedup[-1]
                cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                dx, dy = b[0] - a[0], b[1] - a[1]
                dn = math.hypot(dx, dy)
                dx, dy = (dx / dn, dy / dn) if dn > 1e-9 else (1.0, 0.0)
                cleaned.append([(cx - dx * DOT_LEN_MM / 2, cy - dy * DOT_LEN_MM / 2), (cx + dx * DOT_LEN_MM / 2, cy + dy * DOT_LEN_MM / 2)])
            else:
                dropped += 1
            continue
        cleaned.append(st2)
    if not cleaned:
        raise DrawError("그릴 수 있는 선이 남지 않았어요(너무 짧거나 작아요).")

    # 4) 순서 정하기: 현재 위치에서 가장 가까운 획부터(필요하면 뒤집어서), 끝이 가까우면 펜을 들지 않고 이어 붙인다
    pos = (quad[0][0], quad[0][1])
    left, ordered = list(cleaned), []
    while left:
        best = min(range(len(left)), key=lambda i: min(math.dist(pos, left[i][0]), math.dist(pos, left[i][-1])))
        st = left.pop(best)
        if math.dist(pos, st[-1]) < math.dist(pos, st[0]):
            st = st[::-1]
        if ordered and math.dist(ordered[-1][-1], st[0]) <= JOIN_MM:
            ordered[-1] = _resample(ordered[-1] + st[1:], step)
        else:
            ordered.append(st)
        pos = ordered[-1][-1]
    if first_n:
        ordered = ordered[:first_n]
    pen_mm = sum(_length(st) for st in ordered)

    # 5) 높이를 붙여 경로 만들기
    hd = 0.0 if mode == "draw" else air_mm
    hu = hd + lift
    if curve:
        return _curve_result(fr, info, ordered, hd, hu, quad, k, w, h, pen_mm, dropped, mode, tol, curve_step)
    seq = []                                   # (u, v, 높이, 획 번호 또는 -1(이동), 내리는 지점이면 True)
    for si, st in enumerate(ordered):
        seq.append((st[0][0], st[0][1], hu, -1, False))
        seq.append((st[0][0], st[0][1], hd, si, True))
        seq += [(p[0], p[1], hd, si, False) for p in st[1:]]
        seq.append((st[-1][0], st[-1][1], hu, -1, False))
    kept = [seq[0]]                                            # 너무 가까운 점(0.3×점 간격 미만)은 합친다(펜 내리기·올리기 같은 높이 이동은 그대로)
    for q in seq[1:-1]:
        pk = kept[-1]
        if math.dist((q[0], q[1], q[2]), (pk[0], pk[1], pk[2])) >= 0.3 * min_seg or q[4]:
            kept.append(q)
    kept.append(seq[-1])
    seq = kept
    if len(seq) - 1 > MAX_POINTS:
        raise DrawError(f"점이 {len(seq) - 1}개로 너무 많아요(최대 {MAX_POINTS}개). 더 단순한 그림을 쓰거나 크기를 줄여 주세요.")
    poses = [CY._at(fr, info["orientation"], a, b, hh) for a, b, hh, _, _ in seq]
    start, chain, meta = poses[0], poses[1:], seq[1:]
    dist = [math.dist(poses[i][:3], poses[i + 1][:3]) for i in range(len(poses) - 1)]   # dist[i]: poses[i]→poses[i+1]
    if min(dist) < 0.3 * min_seg:
        raise DrawError("경로에 거의 겹친 점이 있어요. 선을 단순화해 다시 시도해 주세요.")
    blends, reaches = [], []
    for i in range(1, len(poses)):             # poses[i] = chain[i-1]
        prev_d = dist[i - 1]
        next_d = dist[i] if i < len(dist) else prev_d
        near = min(prev_d, next_d)
        last = i == len(poses) - 1
        blends.append(-1.0 if last else round(min(PEN_DOWN_BLEND_MM if meta[i - 1][4] else BLEND_CAP_MM, BLEND_FRAC * near), 2))
        reaches.append(round(min(REACH_CAP_MM, max(REACH_MIN_MM, 0.6 * near)), 2))
    lengths = [m[3] for m in meta]

    warnings = list(info.get("warnings", []))
    if dropped:
        warnings.append(f"너무 짧은 선 {dropped}개는 건너뛰었어요.")
    return {"start": start, "chain": chain, "blends": blends, "reaches": reaches, "stroke_of": lengths,
            "strokes": ordered, "paper": quad, "warnings": warnings,
            "info": {"strokes": len(ordered), "points": len(chain) + 1, "pen_mm": round(pen_mm, 1),
                     "path_mm": round(sum(dist), 1), "scale": round(k, 4),
                     "size_mm": [round(k * w, 1), round(k * h, 1)], "pen_height_mm": round(hd, 1), "lift_height_mm": round(hu, 1),
                     "mode": mode}}


def _pieces(dense, deg, window_mm):
    """촘촘히 찍은 선을 deg 이상 꺾이는 점에서 조각으로 나눈다(조각은 그 점을 공유한다)."""
    kk = max(2, round(window_mm / CURVE_DENSE_MM))
    n = len(dense)
    ang = [0.0] * n
    for i in range(kk, n - kk):
        v1, v2 = (dense[i][0] - dense[i - kk][0], dense[i][1] - dense[i - kk][1]), (dense[i + kk][0] - dense[i][0], dense[i + kk][1] - dense[i][1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 > 1e-9 and n2 > 1e-9:
            ang[i] = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))
    cuts = [0]
    for i in range(kk, n - kk):
        if ang[i] >= deg and ang[i] == max(ang[max(0, i - kk):i + kk + 1]) and i - cuts[-1] > kk and n - 1 - i > kk:
            cuts.append(i)
    cuts.append(n - 1)
    min_pts = int(CURVE_MIN_PIECE_MM / CURVE_DENSE_MM)
    while len(cuts) > 2:                                       # 너무 짧은 조각이 생기는 모서리 중 가장 약한 것부터 없앤다
        gaps = [(cuts[j + 1] - cuts[j], j) for j in range(len(cuts) - 1)]
        short = min(gaps)
        if short[0] >= min_pts:
            break
        j = short[1]
        cand = [x for x in (j, j + 1) if 0 < x < len(cuts) - 1]
        if not cand:
            break
        cuts.pop(min(cand, key=lambda x: ang[cuts[x]]))
    return [dense[a:b + 1] for a, b in zip(cuts, cuts[1:])]


def _smooth(pts, m):
    """양 끝은 고정하고 안쪽 점을 앞뒤 m 개 평균으로 다듬는다(스켈레톤의 계단 흔들림 제거)."""
    n = len(pts)
    if n < 2 * m + 3:
        return list(pts)
    out = [pts[0]]
    for i in range(1, n - 1):
        r = min(m, i, n - 1 - i)
        out.append((sum(p[0] for p in pts[i - r:i + r + 1]) / (2 * r + 1), sum(p[1] for p in pts[i - r:i + r + 1]) / (2 * r + 1)))
    out.append(pts[-1])
    return out


def _control_points(piece, tol, max_gap):
    """다듬은 조각 → 스플라인 조절점. 굽은 곳은 오차 tol 을 지키도록 촘촘히, 곧은 곳은 max_gap 이내로 드문드문."""
    sm = _smooth(piece, min(CURVE_SMOOTH_PTS, len(piece) // 12))      # 작은 고리는 다듬으면 오그라드니 점이 적을수록 약하게
    pts = _rdp(sm, tol)
    out = [pts[0]]
    for p in pts[1:]:
        gap = math.dist(out[-1], p)
        if gap > max_gap:
            n = math.ceil(gap / max_gap)
            a = out[-1]
            out += [(a[0] + (p[0] - a[0]) * t / n, a[1] + (p[1] - a[1]) * t / n) for t in range(1, n)]
        out.append(p)
    return out


def _catmull(pts, sub=8):
    """미리보기용: 조절점을 지나는 부드러운 곡선(Catmull–Rom). 실제 곡선은 컨트롤러의 스플라인이 만든다."""
    if len(pts) < 3:
        return list(pts)
    p = [pts[0]] + list(pts) + [pts[-1]]
    out = [pts[0]]
    for i in range(1, len(p) - 2):
        for s in range(1, sub + 1):
            t = s / sub
            out.append(tuple(0.5 * ((2 * p[i][c]) + (-p[i - 1][c] + p[i + 1][c]) * t + (2 * p[i - 1][c] - 5 * p[i][c] + 4 * p[i + 1][c] - p[i + 2][c]) * t * t
                                    + (-p[i - 1][c] + 3 * p[i][c] - 3 * p[i + 1][c] + p[i + 2][c]) * t ** 3) for c in (0, 1)))
    return out


def _chain_op(start, poses):
    """짧은 조각들의 조절점을 멈추지 않고 이어 가는 동작: 블렌드(평활 반경)를 준 MoveL 을 연달아 보낸다(마지막 점에서만 도착을 기다린다).
    start 는 지금 위치(이미 도착해 있다). 너무 가까운 점은 솎는다."""
    pts = []
    prev = start
    for p in poses:
        if math.dist(p[:3], prev[:3]) >= CHAIN_MIN_GAP_MM:
            pts.append(p)
            prev = p
    if not pts:
        return None
    if math.dist(pts[-1][:3], poses[-1][:3]) > 1e-6:                  # 솎다가 마지막 점이 빠졌으면 마지막 점은 꼭 남긴다
        pts[-1] = poses[-1]
    route = [start] + pts
    dist = [math.dist(route[i][:3], route[i + 1][:3]) for i in range(len(route) - 1)]
    blends, reaches = [], []
    for i in range(1, len(route)):
        near = min(dist[i - 1], dist[i] if i < len(dist) else dist[i - 1])
        blends.append(-1.0 if i == len(route) - 1 else round(min(BLEND_CAP_MM, BLEND_FRAC * near), 2))
        reaches.append(round(min(REACH_CAP_MM, max(REACH_MIN_MM, 0.6 * near)), 2))
    return {"poses": pts, "blends": blends, "reaches": reaches}


def _at_arc(pts, cum, s):
    lo, hi = 0, len(pts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cum[hi] - cum[lo]
    t = 0.0 if seg < 1e-12 else (min(max(s, 0.0), cum[-1]) - cum[lo]) / seg
    return (pts[lo][0] + t * (pts[hi][0] - pts[lo][0]), pts[lo][1] + t * (pts[hi][1] - pts[lo][1]))


def _spline_ctrl(piece, step, dev_mm=SPLINE_DEV_MM):
    """스플라인 조절점: 다듬은 조각을 호 길이 기준으로 **같은 간격**으로 찍는다(간격이 들쭉날쭉하면 스플라인 속도가 출렁이고 로봇이 흔들린다).
    곡선이 휘어서 원래 모양에서 dev_mm 넘게 벗어나면 간격을 줄여 다시 찍는다(최소 SPLINE_MIN_STEP_MM)."""
    sm = _smooth(piece, min(CURVE_SMOOTH_PTS, len(piece) // 12))
    cum = [0.0]
    for i in range(len(sm) - 1):
        cum.append(cum[-1] + math.dist(sm[i], sm[i + 1]))
    total = cum[-1]
    while True:
        n = max(2, round(total / step))
        pts = [_at_arc(sm, cum, total * k / n) for k in range(n + 1)]
        if step <= SPLINE_MIN_STEP_MM:
            return pts
        curve = _catmull(pts, 6)
        worst = max(min(math.dist(c, q) for q in sm[::2]) for c in curve)
        if worst <= dev_mm:
            return pts
        step = max(SPLINE_MIN_STEP_MM, step * 0.75)


def _curve_result(fr, info, ordered, hd, hu, quad, k, w, h, pen_mm, dropped, mode, tol, max_gap):
    """곡선 모드: 획마다 [펜 든 채 위로 이동 → 내리기 → 스플라인(들) → 올리기]. 동작 목록(ops)으로 돌려준다.
    ops 항목: ("move", 자세, 이름) = 끝까지 기다리는 직선 이동 / ("spline", [자세...], 이름) = 첫 자세는 지금 위치, 끝까지 기다림 /
    ("chain", {"poses","blends","reaches"}, 이름) = 멈추지 않고 이어 가는 짧은 선분들."""
    ori = info["orientation"]

    def pose(p, hh):
        return CY._at(fr, ori, p[0], p[1], hh)

    ops, preview, n_ctrl = [], [], 0
    n = len(ordered)
    for si, st in enumerate(ordered):
        name = f"획 {si + 1}/{n}"
        ops.append(("move", pose(st[0], hu), f"{name} 위로 이동"))
        ops.append(("move", pose(st[0], hd), f"{name} 펜 내리기"))
        curve_pts = []
        cur = pose(st[0], hd)                                  # 지금 펜 위치(방금 내린 곳)
        pending = []                                           # 짧은 조각의 조절점을 모아 두었다가 멈추지 않고 이어서 보낸다

        def flush():
            nonlocal cur, pending
            if pending:
                op = _chain_op(cur, pending)
                if op:
                    ops.append(("chain", op, f"{name} 그리는 중"))
                    cur = op["poses"][-1]
                pending = []

        for dense in _pieces(st, CURVE_SPLIT_DEG, CURVE_CORNER_WINDOW_MM):
            if _length(dense) < SPLINE_MIN_MM:                 # 짧은 조각: 스플라인 대신 조절점을 이어 가는 짧은 선분(곡선과 차이가 거의 없다)
                piece = _control_points(dense, tol, max_gap)
                n_ctrl += len(piece)
                pending += [pose(p, hd) for p in piece[1:]]
                sm = _catmull(piece) if len(piece) >= 3 else list(piece)
            else:
                flush()
                piece = _spline_ctrl(dense, max_gap)
                n_ctrl += len(piece)
                ops.append(("spline", [pose(p, hd) for p in piece], f"{name} 곡선 그리는 중"))
                cur = ops[-1][1][-1]
                sm = _catmull(piece)
            curve_pts += sm if not curve_pts else sm[1:]
        flush()
        preview.append(curve_pts)
        ops.append(("move", pose(st[-1], hu), f"{name} 펜 들기"))
    start = ops[0][1]
    warnings = list(info.get("warnings", []))
    if dropped:
        warnings.append(f"너무 짧은 선 {dropped}개는 건너뛰었어요.")
    return {"ops": ops, "start": start, "strokes": preview, "paper": quad, "warnings": warnings, "curve": True,
            "chain": [o[1] if o[0] == "move" else (o[1][-1] if o[0] == "spline" else o[1]["poses"][-1]) for o in ops],
            "info": {"strokes": n, "points": n_ctrl, "ops": len(ops), "pen_mm": round(pen_mm, 1), "path_mm": round(pen_mm, 1),
                     "scale": round(k, 4), "size_mm": [round(k * w, 1), round(k * h, 1)], "pen_height_mm": round(hd, 1),
                     "lift_height_mm": round(hu, 1), "mode": mode, "curve": True}}

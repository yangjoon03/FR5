"""선화(흰 바탕에 검은 선) 그림 → 펜이 따라갈 중심선 획 → SVG.  로봇을 부르지 않는다.

MindDraw 가 만드는 그림은 선이 굵어서(4~5px) 윤곽을 따라가면 한 선을 두 번 그린다. 그래서 선 굵기를 1px 로 깎은 중심선(thinning)을 따라간다.
순서: 회색조 → 이진화(Otsu) → 작은 얼룩 제거 → 중심선(thinning) → 끝점·분기점 사이를 획으로 추적 → 짧은 곁가지 제거 → 단순화(RDP) → SVG(path).
"""
import math

import cv2
import numpy as np
from scipy import ndimage

import paper_draw as D

MAX_SIDE_PX = 1200          # 이보다 크면 줄여서 처리한다
LOOP_WIDER = 0.5            # 고리 한 겹을 더 그리려면 그 덩어리가 이 겹보다 펜 굵기의 이 비율만큼은 더 넓어야 한다
DOT_MAX_AREA_PEN2 = 8       # 점(동그라미)을 찍을 수 있는 혼자 떨어진 덩어리의 최대 넓이(펜 굵기² 의 배수)
FILL_OVERLAP = 0.30         # 채우기 고리를 펜 굵기의 이만큼 겹쳐 그린다(실제 선이 설정보다 가늘어도 틈이 안 남게)
FILL_PASSES = 3             # 굵은 부분을 중심선으로 다시 채우는 최대 횟수
MIN_BLOB_PX = 12            # 이보다 작은 얼룩(먼지)은 지운다
MIN_STROKE_PX = 4           # 이보다 짧은 획(곁가지·점)은 버린다
SPUR_PX = 7                # 끝점에서 분기점까지 이보다 짧은 곁가지는 지운다
DIR_PX = 8                 # 분기점에서 방향을 재는 거리
MAX_TURN_COS = -0.5        # 분기점에서 이어 붙일 두 간선의 방향 코사인 상한(-0.5 = 120° 이상 = 꺾임 60° 이내)
THICK_PEN_PX = 7            # 그림 속 펜 굵기가 이 이상이면 펜보다 작은 디테일을 정리한다
DETAIL_BLOB_PX = 5          # 최대 디테일일 때 지우는 얼룩 크기 하한
DETAIL_SPUR_PX = 4          # 최대 디테일일 때 지우는 곁가지 길이 하한
DETAIL_MIN_STROKE_PX = 2    # 최대 디테일일 때 버리는 짧은 선 하한
MIN_PEN_PX = 2.0            # 펜이 이보다 가늘면(원본 해상도로는 한 칸도 안 됨) 그림을 키워서 변환한다
MAX_UP = 6                 # 최대 확대 배수
MAX_WORK_PX = 3000         # 확대한 뒤에도 이 크기(긴 변)를 넘기지 않는다(처리 시간·메모리)
RDP_PX = 0.5                # SVG 로 내보낼 때 단순화 오차(픽셀)
MAX_BYTES = 12_000_000

_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class VecError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def _decode(data):
    if not data or len(data) > MAX_BYTES:
        raise VecError("이미지가 없거나 너무 커요(12MB 이하).")
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise VecError("이미지를 읽지 못했어요(PNG, JPG 를 올려 주세요).")
    if img.ndim == 3 and img.shape[2] == 4:                      # 투명한 곳은 흰색으로
        a = img[..., 3:4].astype(np.float32) / 255
        img = (img[..., :3] * a + 255 * (1 - a)).astype(np.uint8)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def _skeleton(gray, min_blob=MIN_BLOB_PX, close_px=0, max_side=MAX_SIDE_PX):
    h, w = gray.shape
    if max(h, w) > max_side:
        k = max_side / max(h, w)
        gray = cv2.resize(gray, (round(w * k), round(h * k)), interpolation=cv2.INTER_AREA)
    if gray.min() == gray.max():
        raise VecError("그림에 선이 없어요(한 가지 색이에요).")
    _, ink = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    if ink.mean() > 127:                                        # 어두운 배경이면 뒤집는다(선이 소수 쪽이어야 한다)
        ink = 255 - ink
    if close_px >= 3:                                           # 펜보다 작은 틈·구멍(눈의 하이라이트 같은)은 어차피 메워지니 미리 메운다
        ink = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px)))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(ink, connectivity=8)
    keep = np.zeros(n, bool); keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_blob
    ink = (keep[lab] * 255).astype(np.uint8)
    if not ink.any():
        raise VecError("그림에서 선을 찾지 못했어요.")
    return ink, cv2.ximgproc.thinning(ink, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN) > 0, gray.shape


def _graph(sk):
    """중심선 픽셀 → 그래프. 노드 = 끝점 / 붙어 있는 분기점 묶음(한 점으로 합친다), 간선 = 노드 사이의 픽셀 경로."""
    pad = np.pad(sk, 1)
    nb = sum(np.roll(np.roll(pad, -dy, 0), -dx, 1).astype(np.uint8) for dy, dx in _N8)[1:-1, 1:-1] * sk
    ys, xs = np.nonzero(sk)
    pix = set(zip(ys.tolist(), xs.tolist()))
    deg = {p: int(nb[p]) for p in pix}

    def neighbors(p):
        return [(p[0] + dy, p[1] + dx) for dy, dx in _N8 if (p[0] + dy, p[1] + dx) in pix]

    node_of, node_pos, node_end = {}, [], []              # node_end[k]: 끝점 노드면 True
    for p in pix:                                          # 끝점
        if deg[p] == 1:
            node_of[p] = len(node_pos); node_pos.append((float(p[1]), float(p[0]))); node_end.append(True)
    seen = set()
    for p in pix:                                          # 분기점 묶음(8방향으로 붙은 deg>=3 픽셀)
        if deg[p] >= 3 and p not in seen:
            comp, stack = [], [p]; seen.add(p)
            while stack:
                c = stack.pop(); comp.append(c)
                for r in neighbors(c):
                    if deg[r] >= 3 and r not in seen:
                        seen.add(r); stack.append(r)
            k = len(node_pos)
            for c in comp:
                node_of[c] = k
            node_pos.append((sum(c[1] for c in comp) / len(comp), sum(c[0] for c in comp) / len(comp))); node_end.append(False)

    edges, visited, done_pair = [], set(), set()          # edges: [점들(x,y), a, b]

    def add(a, mid, b):
        edges.append([[node_pos[a]] + [(float(c[1]), float(c[0])) for c in mid] + [node_pos[b]], a, b])

    for p, a in list(node_of.items()):
        for q in neighbors(p):
            if q in node_of:
                if node_of[q] != a and (p, q) not in done_pair:
                    done_pair.add((p, q)); done_pair.add((q, p)); add(a, [], node_of[q])
                continue
            if q in visited:
                continue
            path, prev, cur = [q], p, q
            visited.add(q)
            end = None
            while end is None:
                nn = [r for r in neighbors(cur) if r != prev]
                hit = [r for r in nn if r in node_of]
                if hit:
                    end = node_of[hit[0]]
                    break
                nn = [r for r in nn if r not in visited]
                if not nn:
                    break
                prev, cur = cur, nn[0]
                visited.add(cur); path.append(cur)
            if end is not None:
                add(a, path, end)
    cycles = []
    for p in pix:                                          # 끝점·분기점이 없는 고리(원 같은 것)
        if p not in visited and p not in node_of:
            path, prev, cur = [p], None, p
            visited.add(p)
            while True:
                nn = [r for r in neighbors(cur) if r != prev and r not in visited]
                if not nn:
                    break
                prev, cur = cur, nn[0]
                visited.add(cur); path.append(cur)
            if len(path) >= 3:
                cycles.append([(float(c[1]), float(c[0])) for c in path] + [(float(path[0][1]), float(path[0][0]))])
    return node_pos, node_end, edges, cycles


def _plength(s):
    return sum(((s[i][0] - s[i + 1][0]) ** 2 + (s[i][1] - s[i + 1][1]) ** 2) ** 0.5 for i in range(len(s) - 1))


def _outward(pts, side):
    """간선의 한쪽 끝에서 바깥(간선 안쪽)으로 향하는 단위 방향."""
    seq = pts if side == 0 else pts[::-1]
    x0, y0 = seq[0]
    for x, y in seq[1:]:
        if math.hypot(x - x0, y - y0) >= DIR_PX:
            break
    d = math.hypot(x - x0, y - y0) or 1.0
    return ((x - x0) / d, (y - y0) / d)


def _trace(sk, spur_px=SPUR_PX):
    """중심선 → 획 목록. 곁가지를 지우고, 분기점에서 가장 곧게 이어지는 간선끼리 한 획으로 잇는다."""
    node_pos, node_end, edges, cycles = _graph(sk)
    alive = [True] * len(edges)
    inc = [[] for _ in node_pos]                           # 노드별로 (간선 번호, 끝 쪽)
    for k, (_, a, b) in enumerate(edges):
        inc[a].append((k, 0)); inc[b].append((k, 1))
    for k, (pts, a, b) in enumerate(edges):                # 곁가지: 한쪽이 끝점이고 짧으면서, 다른 쪽이 분기점인 간선
        if a != b and node_end[a] != node_end[b] and _plength(pts) < spur_px:
            alive[k] = False
    for n in range(len(node_pos)):
        inc[n] = [(k, s) for k, s in inc[n] if alive[k]]
    link = {}                                              # (간선, 끝) → 이어진 (간선, 끝)
    for n, ends in enumerate(inc):
        if node_end[n] or len(ends) < 2:
            continue
        cand = []
        for i in range(len(ends)):
            for j in range(i + 1, len(ends)):
                di, dj = _outward(edges[ends[i][0]][0], ends[i][1]), _outward(edges[ends[j][0]][0], ends[j][1])
                cos = di[0] * dj[0] + di[1] * dj[1]
                if cos <= MAX_TURN_COS:                    # 거의 곧게 이어지는 쌍만(꺾임 60° 이내)
                    cand.append((cos, i, j))
        used = set()
        for cos, i, j in sorted(cand):
            if i in used or j in used:
                continue
            used.update((i, j))
            link[ends[i]] = ends[j]; link[ends[j]] = ends[i]
    strokes, used_edge = [], set()

    def follow(k, side):
        """간선 k 를 side 쪽 끝에서 시작해서 이어진 만큼 따라간다."""
        pts = []
        while k not in used_edge:
            used_edge.add(k)
            seq = edges[k][0] if side == 0 else edges[k][0][::-1]
            pts += seq if not pts else seq[1:]
            nxt = link.get((k, 1 - side))
            if nxt is None:
                break
            k, side = nxt
        return pts

    for k in range(len(edges)):                            # 먼저 획의 끝(이어진 상대가 없는 쪽)에서 시작
        if alive[k] and k not in used_edge:
            for side in (0, 1):
                if (k, side) not in link:
                    strokes.append(follow(k, side))
                    break
    for k in range(len(edges)):                            # 남은 것 = 이어져 닫힌 고리
        if alive[k] and k not in used_edge:
            strokes.append(follow(k, 0))
    return [s for s in strokes if len(s) >= 2] + cycles


def _clean(strokes, min_stroke_px):
    out = []
    for s in strokes:
        if _plength(s) < min_stroke_px:
            continue
        s = D._rdp(s, RDP_PX)
        if len(s) >= 2:
            out.append(s)
    return out


def _circle(cx, cy, r):
    n = max(8, int(2 * math.pi * r / 2))
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


def _paint(mask, strokes, width):
    for s in strokes:
        cv2.polylines(mask, [np.round(np.array(s)).astype(np.int32)], False, 255, max(1, int(round(width))), cv2.LINE_8)


def _loops(ink, pen, step=None):
    """굵은 부분을 펜 굵기 간격의 안쪽 윤곽선 고리로 채운다(윤곽을 따라 안쪽으로 한 겹씩). 원본 모양 그대로 칠해진다."""
    up = 2                                                     # 2배로 키워서 윤곽을 매끈하게
    big = (cv2.resize(ink, None, fx=up, fy=up, interpolation=cv2.INTER_LINEAR) > 127).astype(np.uint8) * 255
    dist = cv2.distanceTransform(big, cv2.DIST_L2, 5) / up
    step = step or pen
    loops, d, maxd = [], pen / 2.0, float(dist.max())
    while d <= maxd:
        n, lab = cv2.connectedComponents((dist >= d).astype(np.uint8), connectivity=8)
        if n > 1:
            cmax = ndimage.maximum(dist, lab, index=np.arange(1, n))
            ok = np.zeros(n, bool); ok[1:] = cmax >= d + LOOP_WIDER * pen      # 이 겹보다 충분히 넓은 덩어리만(아니면 한 줄 중심선으로)
            cs, _ = cv2.findContours(ok[lab].astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            for c in cs:
                if len(c) < 8:
                    continue
                pts = [(float(p[0][0]) / up, float(p[0][1]) / up) for p in c]
                pts = D._rdp(pts + [pts[0]], RDP_PX)
                if len(pts) >= 4:
                    loops.append(pts)
        d += step
    return loops


def detail_params(level):
    """디테일 수준(0~1, 1 = 최대)을 변환 기준으로: (지우는 얼룩 크기 px, 지우는 곁가지 길이 px, 버리는 짧은 선 px, 채우기 최소 면적 비율).
    0.75 ≈ 예전 기본값, 1.0 = 예전 '최대 디테일'."""
    t = min(1.0, max(0.0, level))
    s = (1 - t) ** 2                                           # 낮은 쪽에서 크게 달라지도록 제곱으로
    return int(round(5 + s * 100)), int(round(4 + s * 20)), 2 + s * 28, 0.25 + (1 - t) * 1.0


def to_strokes(data, min_stroke_px=MIN_STROKE_PX, pen_pct=60.0, fill=True, pen_px=None, detail=False, detail_level=None, overlap=FILL_OVERLAP):
    """이미지 바이트 → {"strokes": [[(x,y)...]], "size": (w,h), "pen_px": 펜 굵기(px)}
    펜 굵기 = 원본 선 굵기 × pen_pct%. 펜보다 굵은 부분은 윤곽을 따라 안쪽으로 한 겹씩 고리로 채우고(fill=True),
    펜보다 가는 선과 남은 부분은 중심선으로 긋는다. 펜이 가늘수록(pen_pct 낮을수록) 원본과 같아진다."""
    # 디테일 수준 t(0~1): 높을수록 작은 얼룩·짧은 선·곁가지도 버리지 않고 남긴다(먼지가 섞일 수 있다). detail_level 이 없으면 detail 로(True=1.0, False=0.75)
    t = float(detail_level) / 100.0 if detail_level is not None else (1.0 if detail else 0.75)
    blob, spur, min_stroke_px, fill_min = detail_params(t)
    hi = t >= 0.9
    gray = _decode(data)
    gh, gw = gray.shape
    if max(gh, gw) > MAX_SIDE_PX:
        k = MAX_SIDE_PX / max(gh, gw)
        gray = cv2.resize(gray, (round(gw * k), round(gh * k)), interpolation=cv2.INTER_AREA)
    up = 1                                                      # 가는 펜: 그림을 키워서 굵은 부분을 펜 폭 간격으로 빈틈없이 채운다(키우지 않으면 한 칸보다 가는 간격을 못 만든다)
    if fill and pen_px and pen_px < MIN_PEN_PX:
        up = max(1, min(MAX_UP, int(math.ceil(MIN_PEN_PX / pen_px)), MAX_WORK_PX // max(gray.shape)))
        gray = cv2.resize(gray, None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC)
        pen_px = pen_px * up
        blob *= up * up; spur *= up; min_stroke_px *= up
    close_px = 0
    if pen_px and pen_px >= THICK_PEN_PX:                       # 펜이 굵으면 펜보다 작은 디테일은 뭉개져서 지저분해지니 같이 정리한다(최대 디테일이어도)
        close_px = int(round(0.5 * pen_px)) | 1
        blob = max(blob, int(0.35 * pen_px * pen_px))
        spur = max(spur, int(0.8 * pen_px))
        min_stroke_px = max(min_stroke_px, 1.2 * pen_px)
    ink, sk, (h, w) = _skeleton(gray, blob, close_px, max_side=10 ** 9)
    dist = cv2.distanceTransform(ink, cv2.DIST_L2, 3)
    line_px = 2.0 * float(np.median(dist[sk]))                   # 원본 선 굵기(px)
    pen = max(1.5, float(pen_px)) if pen_px else max(2.0, line_px * pen_pct / 100.0)
    out = [lp for lp in _loops(ink, pen, pen * (1.0 - min(0.6, max(0.0, overlap)))) if _plength(lp) >= min_stroke_px] if fill else []
    covered = np.zeros(ink.shape, np.uint8)
    _paint(covered, out, pen)
    near = cv2.dilate(covered, np.ones((5, 5), np.uint8))
    _, ink_lab, ink_stats, _ = cv2.connectedComponentsWithStats((ink > 0).astype(np.uint8), connectivity=8)   # 원본 선·점 덩어리(혼자 떨어진 점 판별용)
    for it in range(FILL_PASSES if fill else 1):
        resid_all = ((ink > 0) & (covered == 0)).astype(np.uint8) * 255
        opened = cv2.morphologyEx(resid_all, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        resid = np.where((near > 0), opened, resid_all).astype(np.uint8)              # 고리 가장자리 얇은 찌꺼기만 버린다
        n, lab, stats, cent = cv2.connectedComponentsWithStats(resid, connectivity=8)
        keep = np.zeros(n, bool); keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= (blob if it == 0 else max(3 if hi else 6, fill_min * pen * pen))
        resid = (keep[lab] * 255).astype(np.uint8)
        if not resid.any():
            break
        new = _clean(_trace(cv2.ximgproc.thinning(resid, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN) > 0, spur), min_stroke_px if it == 0 else max(1.5 if hi else 3.0, fill_min * pen))
        made = np.zeros(ink.shape, np.uint8); _paint(made, new, pen)
        for c in range(1, n):                                      # 중심선이 안 나오는 작은 덩어리는 작은 동그라미(점)로. 단, 원본에서 **혼자 떨어진 작은 점**일 때만
            if not keep[c]:                                         # (굵은 선 곁에 남은 찌꺼기는 점으로 찍으면 선 위에 엉뚱한 점이 생긴다)
                continue
            cx_, cy_ = int(cent[c][0]), int(cent[c][1])
            ink_id = ink_lab[min(max(cy_, 0), ink_lab.shape[0] - 1), min(max(cx_, 0), ink_lab.shape[1] - 1)]
            if ink_id == 0 or ink_stats[ink_id, cv2.CC_STAT_AREA] > max(40, DOT_MAX_AREA_PEN2 * pen * pen):
                continue
            if keep[c] and not (made[lab == c] > 0).any():
                r = max(1.0, math.sqrt(stats[c, cv2.CC_STAT_AREA] / math.pi) * 0.5)
                new.append(_circle(float(cent[c][0]), float(cent[c][1]), r))
        if not new:
            break
        out += new
        _paint(covered, new, pen)
        near = cv2.dilate(covered, np.ones((5, 5), np.uint8))
    if not out:
        raise VecError("그릴 수 있는 선이 남지 않았어요.")
    if up > 1:                                                  # 원래 그림 좌표로 되돌린다
        out = [[(round(x / up, 2), round(y / up, 2)) for x, y in st] for st in out]
        w, h, pen, line_px = w // up, h // up, pen / up, line_px / up
    return {"strokes": out, "size": (w, h), "pen_px": round(pen, 2), "line_px": round(line_px, 1)}


def to_svg(strokes, size, pen_px=0):
    w, h = size
    body = "\n".join('<path d="M' + " L".join(f"{x} {y}" for x, y in s) + '" fill="none" stroke="#000"/>' for s in strokes)
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" data-pen-px="{pen_px}">\n{body}\n</svg>'


def vectorize(data, min_stroke_px=MIN_STROKE_PX, pen_pct=60.0, fill=True, pen_px=None, detail=False, detail_level=None, overlap=FILL_OVERLAP):
    r = to_strokes(data, min_stroke_px, pen_pct, fill, pen_px, detail, detail_level, overlap)
    return {"svg": to_svg(r["strokes"], r["size"], r["pen_px"]), "strokes": len(r["strokes"]), "size": list(r["size"]),
            "points": sum(len(s) for s in r["strokes"]), "pen_px": r["pen_px"]}

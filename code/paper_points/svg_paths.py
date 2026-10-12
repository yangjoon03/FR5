"""SVG → 선(폴리라인) 변환. 펜으로 따라 그릴 선만 뽑는다(채우기는 무시하고 모든 도형의 윤곽을 따라간다).

지원: path(M L H V C S Q T A Z 와 소문자), line, polyline, polygon, rect, circle, ellipse, g 와 transform(translate/scale/rotate/matrix/skewX/skewY).
지원하지 않음(경고만 하고 건너뜀): text, image, use, 클리핑/마스크/그라디언트 등.
곡선은 짧은 직선으로 쪼갠다. 결과 좌표는 SVG 사용자 단위(y 는 아래로 증가)이다.
"""
import math
import re
import xml.etree.ElementTree as ET

MAX_BYTES = 2_000_000
CURVE_STEPS = 32           # 3차/2차 베지에 하나를 나누는 직선 수
ARC_STEP_DEG = 5.0         # 호를 나누는 각도 간격

_NUM = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")
_SKIP = {"defs", "clippath", "mask", "symbol", "pattern", "marker", "style", "script", "title", "desc", "metadata",
         "lineargradient", "radialgradient", "filter"}
_UNSUPPORTED = {"text", "image", "use", "foreignobject"}


class SvgError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


# ---------------------------------------------------------------- 아핀 변환 (a, b, c, d, e, f) = [[a c e], [b d f]]
IDENTITY = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def _mul(m, n):
    return (m[0] * n[0] + m[2] * n[1], m[1] * n[0] + m[3] * n[1], m[0] * n[2] + m[2] * n[3], m[1] * n[2] + m[3] * n[3],
            m[0] * n[4] + m[2] * n[5] + m[4], m[1] * n[4] + m[3] * n[5] + m[5])


def _apply(m, x, y):
    return (m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5])


def parse_transform(text):
    m = IDENTITY
    for name, args in re.findall(r"(\w+)\s*\(([^)]*)\)", text or ""):
        v = [float(x) for x in _NUM.findall(args)]
        name = name.lower()
        if name == "translate" and v:
            t = (1, 0, 0, 1, v[0], v[1] if len(v) > 1 else 0.0)
        elif name == "scale" and v:
            t = (v[0], 0, 0, v[1] if len(v) > 1 else v[0], 0, 0)
        elif name == "rotate" and v:
            a = math.radians(v[0]); c, s = math.cos(a), math.sin(a)
            t = (c, s, -s, c, 0, 0)
            if len(v) >= 3:
                t = _mul(_mul((1, 0, 0, 1, v[1], v[2]), t), (1, 0, 0, 1, -v[1], -v[2]))
        elif name == "matrix" and len(v) == 6:
            t = tuple(v)
        elif name == "skewx" and v:
            t = (1, 0, math.tan(math.radians(v[0])), 1, 0, 0)
        elif name == "skewy" and v:
            t = (1, math.tan(math.radians(v[0])), 0, 1, 0, 0)
        else:
            continue
        m = _mul(m, t)
    return m


# ---------------------------------------------------------------- path 'd' 해석
def _tokens(d):
    for m in re.finditer(r"([MmLlHhVvCcSsQqTtAaZz])|(" + _NUM.pattern + ")", d):
        yield m.group(1) or float(m.group(2))


def _arc_points(x1, y1, rx, ry, phi_deg, large, sweep, x2, y2):
    """SVG 호(endpoint 표현) → 점 목록(시작점 제외, 끝점 포함). 표준 변환 알고리즘(SVG 명세 F.6.5)."""
    if (x1, y1) == (x2, y2):
        return []
    rx, ry = abs(rx), abs(ry)
    if rx == 0 or ry == 0:
        return [(x2, y2)]
    phi = math.radians(phi_deg % 360)
    cp, sp = math.cos(phi), math.sin(phi)
    dx, dy = (x1 - x2) / 2, (y1 - y2) / 2
    x1p, y1p = cp * dx + sp * dy, -sp * dx + cp * dy
    lam = x1p ** 2 / rx ** 2 + y1p ** 2 / ry ** 2
    if lam > 1:                                  # 반지름이 모자라면 키운다
        rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
    num = rx ** 2 * ry ** 2 - rx ** 2 * y1p ** 2 - ry ** 2 * x1p ** 2
    den = rx ** 2 * y1p ** 2 + ry ** 2 * x1p ** 2
    co = math.sqrt(max(0.0, num / den)) if den else 0.0
    if large == sweep:
        co = -co
    cxp, cyp = co * rx * y1p / ry, -co * ry * x1p / rx
    cx, cy = cp * cxp - sp * cyp + (x1 + x2) / 2, sp * cxp + cp * cyp + (y1 + y2) / 2

    def ang(ux, uy, vx, vy):
        a = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
        return a

    t1 = ang(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry)
    dt = ang((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry)
    if not sweep and dt > 0:
        dt -= 2 * math.pi
    elif sweep and dt < 0:
        dt += 2 * math.pi
    n = max(2, int(math.ceil(abs(math.degrees(dt)) / ARC_STEP_DEG)))
    pts = []
    for i in range(1, n + 1):
        t = t1 + dt * i / n
        ex, ey = rx * math.cos(t), ry * math.sin(t)
        pts.append((cp * ex - sp * ey + cx, sp * ex + cp * ey + cy))
    pts[-1] = (x2, y2)
    return pts


def _bezier(p0, c1, c2, p3):
    out = []
    for i in range(1, CURVE_STEPS + 1):
        t = i / CURVE_STEPS; u = 1 - t
        out.append((u ** 3 * p0[0] + 3 * u * u * t * c1[0] + 3 * u * t * t * c2[0] + t ** 3 * p3[0],
                    u ** 3 * p0[1] + 3 * u * u * t * c1[1] + 3 * u * t * t * c2[1] + t ** 3 * p3[1]))
    return out


def _quad(p0, c, p2):
    return _bezier(p0, (p0[0] + 2 * (c[0] - p0[0]) / 3, p0[1] + 2 * (c[1] - p0[1]) / 3),
                   (p2[0] + 2 * (c[0] - p2[0]) / 3, p2[1] + 2 * (c[1] - p2[1]) / 3), p2)


def parse_path_d(d):
    """path 의 d 속성 → 하위 경로(점 목록) 목록. 닫힌 경로는 마지막 점이 시작점과 같다."""
    toks = list(_tokens(d or ""))
    subs, cur = [], []
    i, cmd = 0, None
    x = y = sx = sy = 0.0
    last_c = None            # 직전 3차 베지에 두 번째 제어점(S 용)
    last_q = None            # 직전 2차 베지에 제어점(T 용)

    def num():
        nonlocal i
        if i >= len(toks) or isinstance(toks[i], str):
            raise SvgError("path 데이터 형식이 올바르지 않아요.")
        v = toks[i]; i += 1
        return v

    def flag():
        return num() != 0

    while i < len(toks):
        if isinstance(toks[i], str):
            cmd = toks[i]; i += 1
            if cmd in "Zz":
                if cur:
                    cur.append((sx, sy)); subs.append(cur); cur = []
                x, y = sx, sy
                last_c = last_q = None
                continue
        elif cmd is None:
            raise SvgError("path 데이터가 명령으로 시작하지 않아요.")
        rel = cmd.islower()
        c = cmd.upper()
        ox, oy = (x, y) if rel else (0.0, 0.0)
        if c == "M":
            nx, ny = num() + ox, num() + oy
            if cur:
                subs.append(cur)
            cur = [(nx, ny)]
            x, y = sx, sy = nx, ny
            cmd = "l" if rel else "L"                # 이어지는 좌표쌍은 선분
            last_c = last_q = None
        elif c == "L":
            x, y = num() + ox, num() + oy; cur.append((x, y)); last_c = last_q = None
        elif c == "H":
            x = num() + ox; cur.append((x, y)); last_c = last_q = None
        elif c == "V":
            y = num() + oy; cur.append((x, y)); last_c = last_q = None
        elif c == "C":
            c1 = (num() + ox, num() + oy); c2 = (num() + ox, num() + oy); p = (num() + ox, num() + oy)
            cur.extend(_bezier((x, y), c1, c2, p)); x, y = p; last_c, last_q = c2, None
        elif c == "S":
            c1 = (2 * x - last_c[0], 2 * y - last_c[1]) if last_c else (x, y)
            c2 = (num() + ox, num() + oy); p = (num() + ox, num() + oy)
            cur.extend(_bezier((x, y), c1, c2, p)); x, y = p; last_c, last_q = c2, None
        elif c == "Q":
            q = (num() + ox, num() + oy); p = (num() + ox, num() + oy)
            cur.extend(_quad((x, y), q, p)); x, y = p; last_q, last_c = q, None
        elif c == "T":
            q = (2 * x - last_q[0], 2 * y - last_q[1]) if last_q else (x, y)
            p = (num() + ox, num() + oy)
            cur.extend(_quad((x, y), q, p)); x, y = p; last_q, last_c = q, None
        elif c == "A":
            rx, ry, rot = num(), num(), num(); large, sweep = flag(), flag(); p = (num() + ox, num() + oy)
            cur.extend(_arc_points(x, y, rx, ry, rot, large, sweep, p[0], p[1]) or [p]); x, y = p; last_c = last_q = None
        else:
            raise SvgError(f"지원하지 않는 path 명령이에요: {cmd}")
        if not cur:
            cur = [(x, y)]
    if cur:
        subs.append(cur)
    return subs


# ---------------------------------------------------------------- 도형 → 선
def _f(el, name, default=0.0):
    m = _NUM.match((el.get(name) or "").strip())
    return float(m.group(0)) if m else default


def _ellipse_pts(cx, cy, rx, ry):
    n = int(360 / ARC_STEP_DEG)
    return [(cx + rx * math.cos(2 * math.pi * k / n), cy + ry * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


def _shape_polylines(tag, el):
    if tag == "path":
        return parse_path_d(el.get("d"))
    if tag == "line":
        return [[(_f(el, "x1"), _f(el, "y1")), (_f(el, "x2"), _f(el, "y2"))]]
    if tag in ("polyline", "polygon"):
        v = [float(t) for t in _NUM.findall(el.get("points") or "")]
        pts = [(v[k], v[k + 1]) for k in range(0, len(v) - 1, 2)]
        if tag == "polygon" and pts:
            pts.append(pts[0])
        return [pts] if len(pts) >= 2 else []
    if tag == "rect":
        x, y, w, h = _f(el, "x"), _f(el, "y"), _f(el, "width"), _f(el, "height")
        return [[(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]] if w > 0 and h > 0 else []
    if tag == "circle":
        r = _f(el, "r")
        return [_ellipse_pts(_f(el, "cx"), _f(el, "cy"), r, r)] if r > 0 else []
    if tag == "ellipse":
        rx, ry = _f(el, "rx"), _f(el, "ry")
        return [_ellipse_pts(_f(el, "cx"), _f(el, "cy"), rx, ry)] if rx > 0 and ry > 0 else []
    return []


def _hidden(el):
    style = (el.get("style") or "").replace(" ", "").lower()
    return (el.get("display") == "none" or "display:none" in style or el.get("visibility") == "hidden"
            or "visibility:hidden" in style)


def parse_svg(text):
    """SVG 문자열 → {"polylines": [[(x,y),...]], "bbox": (x0,y0,x1,y1), "warnings": [...]}"""
    if not text or not text.strip():
        raise SvgError("SVG 내용이 비어 있어요.")
    if len(text.encode("utf-8")) > MAX_BYTES:
        raise SvgError("SVG 파일이 너무 커요(2MB 이하).")
    low = text.lower()
    if "<!entity" in low or "<!doctype" in low and "[" in low:      # 외부/중첩 엔티티 확장 공격을 막는다
        raise SvgError("엔티티가 들어 있는 SVG 는 읽지 않아요.")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise SvgError(f"SVG 를 읽지 못했어요: {e}")
    if root.tag.split("}")[-1].lower() != "svg":
        raise SvgError("<svg> 로 시작하는 파일이 아니에요.")
    polylines, warnings, skipped = [], [], set()

    def walk(el, m):
        tag = el.tag.split("}")[-1].lower()
        if tag in _SKIP or _hidden(el):
            return
        m = _mul(m, parse_transform(el.get("transform")))
        if tag in _UNSUPPORTED:
            skipped.add(tag)
            return
        for sub in _shape_polylines(tag, el):
            pts = [_apply(m, px, py) for px, py in sub]
            if len(pts) >= 2:
                polylines.append(pts)
        for child in el:
            walk(child, m)

    # 루트 svg 의 viewBox 오프셋은 쓰지 않는다(그림의 실제 경계 상자로 맞춘다)
    walk(root, IDENTITY)
    if skipped:
        warnings.append("지원하지 않는 요소를 건너뛰었어요: " + ", ".join(sorted(skipped)))
    if not polylines:
        raise SvgError("그릴 선이 없어요. path, line, rect, circle 같은 도형이 있는 SVG 가 필요해요.")
    xs = [p[0] for pl in polylines for p in pl]; ys = [p[1] for pl in polylines for p in pl]
    try:
        pen_px = float(root.get("data-pen-px") or 0)             # 우리 변환기가 적어 둔 "그림 속 펜 굵기"(px)
    except ValueError:
        pen_px = 0.0
    return {"polylines": polylines, "bbox": (min(xs), min(ys), max(xs), max(ys)), "warnings": warnings, "pen_px": pen_px}

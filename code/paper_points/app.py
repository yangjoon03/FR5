"""종이 모서리 4곳의 로봇 위치를 저장하는 작은 도구.  실행: python app.py  →  http://localhost:5051

사용법: 로봇을 직접(티치펜던트·드래그 티칭 등) 종이의 모서리 1곳으로 옮긴 뒤, 이 화면에서 "현재 위치 저장"을 누른다.
모서리 저장은 로봇에 **읽기 명령만** 보낸다(연결, 현재 TCP 위치 조회).

"펜 들기 테스트", "사각 구역 한 바퀴", "원통(공중)", "SVG 그리기"만 예외로, 도구(TCP) 좌표계 Z축을 따라 **조금 뒤로 갔다가 다시 앞으로** 가는 직선 이동(MoveL 도구 오프셋)을 보낸다.
webcontrol 의 "Z (J6 전진/후진)"와 같은 방식이며, 펜이 도구 +Z 방향으로 종이를 향해 있다고 가정한다. 안전장치: 거리·속도 상한,
확인 체크 필수, 로봇 에러코드가 있으면 거부, 동시에 한 동작만, 정지하면 다음 이동을 보내지 않는다. 활성화·드래그 티칭 명령은 보내지 않는다.
"""
import base64
import json
import math
import os
import re
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

import paper_cyl as CY
import paper_draw as D
import paper_loop as L
import paper_pen as PN
import paper_tcp as T
import svg_paths as S
import vectorize as V

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, "..", "webcontrol"))   # robot_manager(연결·상태 조회 래퍼)를 재사용한다
POINTS_FILE = os.path.join(HERE, "paper_points.json")
HISTORY_FILE = os.path.join(HERE, "paper_points_history.jsonl")   # 저장·삭제할 때마다 덧붙이는 이력(지우지 않는다)
COUNT = 4
MAX_LIFT_MM = 30.0     # 한 번에 뒤로 갈 수 있는 거리 상한
MAX_LIFT_VEL = 15.0    # 펜 들기 테스트·사각 구역·원통의 속도 상한(%)
MAX_DRAW_VEL = 100.0   # SVG 그리기 속도 상한(%) — 로봇 명령이 받는 최대값

app = Flask(__name__, static_folder=None)
_lock = threading.Lock()
_rm = None
_rm_error = ""


def _get_rm():
    global _rm, _rm_error
    with _lock:
        if _rm is None and not _rm_error:
            try:
                from robot_manager import RobotManager
                _rm = RobotManager()
            except Exception as e:
                _rm_error = f"로봇 모듈(webcontrol/robot_manager)을 불러오지 못했어요: {e}"
        return _rm


def _load():
    if not os.path.exists(POINTS_FILE):
        return [None] * COUNT
    with open(POINTS_FILE, encoding="utf-8") as f:
        pts = json.load(f)
    return (pts + [None] * COUNT)[:COUNT]


def _save(points, why=""):
    # 바꾸기 전에 지금 상태를 이력에 남긴다. 실수로 지우거나 덮어써도 마지막 값을 되살릴 수 있다.
    try:
        with open(HISTORY_FILE, "a", encoding="utf-8") as f:
            f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "why": why, "before": _load(), "after": points},
                               ensure_ascii=False) + "\n")
    except OSError:
        pass
    tmp = POINTS_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(points, f, ensure_ascii=False, indent=2)
    os.replace(tmp, POINTS_FILE)


FILL_PEN_MIN_MM = 0.1   # 채우기 간격·점 간격을 정할 때 쓰는 펜 굵기의 하한(가는 펜은 같은 자리를 여러 번 그어 채운다)
FINE_PEN_MM = 0.4       # 이보다 가는 펜은 겹침을 20%로(0.05mm 간격은 로봇 오차보다 촘촘하니 펜 폭 간격으로 여러 줄)
CLEAN_MM = 0.6         # 이보다 작은 무늬(넥타이 체크무늬 같은)는 로봇이 따라가지 못하고 선이 엉키니 펜이 가늘어도 이 크기 기준으로 정리한다
THIN_PEN_PX = 2.5   # 그림 속 펜 굵기가 이보다 가늘면 경고
SETTINGS_FILE = os.path.join(HERE, "paper_settings.json")
DEFAULT_SETTINGS = {"pen_mm": 0.0, "pen_x": 0.0, "pen_y": 0.0, "tip_tcp": False, "align": False}


def _settings():
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            return {**DEFAULT_SETTINGS, **json.load(f)}
    except (OSError, ValueError):
        return dict(DEFAULT_SETTINGS)


def _analyze():
    """저장한 모서리 + 펜 설정(펜 끝까지 길이·수직 보정)으로 종이 평면을 만든다."""
    s = _settings()
    return L.analyze(_load(), pen_mm=float(s["pen_mm"]), tip_tcp=bool(s["tip_tcp"]), align=bool(s["align"]), pen_x=float(s["pen_x"]), pen_y=float(s["pen_y"]))


def _err(msg, status=400):
    return jsonify({"error": msg}), status


def _distances(points):
    """저장된 이웃한 모서리 사이의 거리(mm). 저장 값이 맞는지 눈으로 확인하는 용도다."""
    out = []
    for i in range(COUNT):
        a, b = points[i], points[(i + 1) % COUNT]
        out.append(round(math.dist(a["pose"][:3], b["pose"][:3]), 1) if a and b else None)
    return out


# ---------------------------------------------------------------- 펜 들기 테스트(뒤로 갔다가 앞으로)
_motion_lock = threading.Lock()
_abort = threading.Event()
_motion = {"state": "idle", "step": "", "error": "", "poses": {}, "residual_mm": None, "moved_mm": None}


def _set_motion(**kw):
    with _lock:
        _motion.update(kw)


def _code(ret):
    return ret[0] if isinstance(ret, (tuple, list)) else ret


def _lift_worker(rm, plan, mm, vel):
    """plan: ["retract", "advance"] 중 일부. 각 단계는 도구 Z축 방향 직선 이동이고, 끝날 때까지 기다린다."""
    try:
        start = rm.status()["tcp_pose_mm_deg"]
        _set_motion(state="running", poses={"start": start}, residual_mm=None, moved_mm=None, error="")
        for step in plan:
            if _abort.is_set():                       # 정지했으면 다음 이동은 보내지 않는다
                _set_motion(state="stopped", step="")
                return
            _set_motion(step=step)
            sign = "-" if step == "retract" else "+"  # webcontrol: − = 후진(뒤로), + = 전진(앞으로)
            code = _code(rm.move_linear("z", mm / 10.0, sign, vel, frame="tool"))
            if code not in (0, None):
                if _abort.is_set():
                    _set_motion(state="stopped", step="")
                else:
                    _set_motion(state="error", step="", error=f"로봇이 이동을 거부했어요(코드 {code}). 로봇 상태와 에러를 확인하세요.")
                return
            with _lock:
                _motion["poses"][step] = rm.status()["tcp_pose_mm_deg"]
        poses = _motion["poses"]
        res = {}
        if "retract" in poses:
            res["moved_mm"] = round(math.dist(start[:3], poses["retract"][:3]), 2)
        if "advance" in poses:
            res["residual_mm"] = round(math.dist(start[:3], poses["advance"][:3]), 2)
        _set_motion(state="done", step="", **res)
    except Exception as e:
        _set_motion(state="error", step="", error=f"이동 중 오류가 났어요: {e}")
    finally:
        _motion_lock.release()


@app.post("/api/lift")
def lift():
    """mode: roundtrip(뒤로 갔다가 앞으로) | retract(뒤로만) | advance(앞으로만). 확인(confirm) 체크가 있어야 시작한다."""
    data = request.get_json(silent=True) or {}
    mode = data.get("mode")
    plan = {"roundtrip": ["retract", "advance"], "retract": ["retract"], "advance": ["advance"]}.get(mode)
    if plan is None:
        return _err("동작을 골라 주세요.")
    try:
        mm, vel = float(data.get("mm", 10)), float(data.get("vel", 10))
    except (TypeError, ValueError):
        return _err("거리와 속도는 숫자여야 해요.")
    if not (1.0 <= mm <= MAX_LIFT_MM):
        return _err(f"거리는 1~{MAX_LIFT_MM:.0f}mm 사이여야 해요.")
    if not (1.0 <= vel <= MAX_LIFT_VEL):
        return _err(f"속도는 1~{MAX_LIFT_VEL:.0f}% 사이여야 해요.")
    if not data.get("confirm"):
        return _err("펜 끝 주변이 비어 있는지 확인하고 체크해 주세요.", 409)
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        err = rm.status().get("error_code") or []
    except Exception as e:
        return _err(f"로봇 상태를 읽지 못했어요: {e}", 502)
    if any(c not in (0, None) for c in err):
        return _err(f"로봇에 에러가 있어요({err}). 에러를 초기화한 뒤 다시 시도해 주세요.", 409)
    if not _motion_lock.acquire(blocking=False):
        return _err("이미 움직이는 중이에요.", 409)
    _abort.clear()
    threading.Thread(target=_lift_worker, args=(rm, plan, mm, vel), daemon=True).start()
    return jsonify({"started": mode, "mm": mm, "vel": vel})


@app.post("/api/stop")
def stop():
    """언제든 정지: 다음 이동을 보내지 않고, 이동 중인 로봇에는 별도 연결로 정지 명령을 보낸다."""
    _abort.set()
    rm = _get_rm()
    if rm is not None and rm.is_connected():
        try:
            rm.stop()
        except Exception as e:
            return _err(f"정지 명령을 보내지 못했어요: {e}", 502)
    return jsonify({"stopped": True})


# ---------------------------------------------------------------- 사각 구역 한 바퀴 (저장한 모서리 4곳 → 1·2·3·4·1)
_loop = {"state": "idle", "step": "", "index": 0, "total": 0, "error": "", "mode": ""}


def _set_loop(**kw):
    with _lock:
        _loop.update(kw)


def _loop_worker(rm, wps, vel, mode):
    try:
        _set_loop(state="running", index=0, total=len(wps), error="", mode=mode, step="")
        for i, (name, pose) in enumerate(wps):
            if _abort.is_set():                          # 정지했으면 다음 이동은 보내지 않는다
                _set_loop(state="stopped", step="")
                return
            _set_loop(step=name, index=i)
            code = _code(rm.move_to_pose(pose, vel=vel))
            if code not in (0, None):
                if _abort.is_set():
                    _set_loop(state="stopped", step="")
                else:
                    _set_loop(state="error", step="", error=f"'{name}' 이동을 로봇이 거부했어요(코드 {code}). 로봇 상태와 에러를 확인하세요.")
                return
        _set_loop(state="done", step="", index=len(wps))
    except Exception as e:
        _set_loop(state="error", step="", error=f"이동 중 오류가 났어요: {e}")
    finally:
        _motion_lock.release()


def _loop_plan(data):
    mode = data.get("mode")
    try:
        air = float(data.get("air_mm", 30))
    except (TypeError, ValueError):
        raise L.LoopError("공중 높이는 숫자여야 해요.")
    info = _analyze()
    return info, L.waypoints(info, mode, air), mode


@app.post("/api/loop/plan")
def loop_plan():
    """이동할 점들을 미리 계산해 보여 준다. 로봇에는 아무 명령도 보내지 않는다(읽기 전용)."""
    try:
        info, wps, mode = _loop_plan(request.get_json(silent=True) or {})
    except L.LoopError as e:
        return _err(str(e))
    return jsonify({"mode": mode, "sides_mm": info["sides_mm"], "angles_deg": info["angles_deg"], "flatness_mm": info["flatness_mm"],
                    "ori_spread_deg": info["ori_spread_deg"], "warnings": info["warnings"],
                    "waypoints": [{"name": n, "pose": p} for n, p in wps]})


@app.post("/api/loop/run")
def loop_run():
    data = request.get_json(silent=True) or {}
    try:
        vel = float(data.get("vel", 10))
    except (TypeError, ValueError):
        return _err("속도는 숫자여야 해요.")
    if not (1.0 <= vel <= MAX_LIFT_VEL):
        return _err(f"속도는 1~{MAX_LIFT_VEL:.0f}% 사이여야 해요.")
    if not data.get("confirm"):
        return _err("로봇 주변이 비어 있는지 확인하고 체크해 주세요.", 409)
    if data.get("mode") == "draw" and not data.get("confirm_draw"):
        return _err("펜이 종이에 닿아도 안전한지 확인해 주세요.", 409)
    try:
        info, wps, mode = _loop_plan(data)
    except L.LoopError as e:
        return _err(str(e))
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        st = rm.status()
        if any(c not in (0, None) for c in (st.get("error_code") or [])):
            return _err(f"로봇에 에러가 있어요({st.get('error_code')}). 에러를 초기화한 뒤 다시 시도해 주세요.", 409)
        L.check_start(st["tcp_pose_mm_deg"], wps[0][1])
    except L.LoopError as e:
        return _err(str(e), 409)
    except Exception as e:
        return _err(f"로봇 상태를 읽지 못했어요: {e}", 502)
    if not _motion_lock.acquire(blocking=False):
        return _err("이미 움직이는 중이에요.", 409)
    _abort.clear()
    threading.Thread(target=_loop_worker, args=(rm, wps, vel, mode), daemon=True).start()
    return jsonify({"started": mode, "waypoints": len(wps), "vel": vel})


# ---------------------------------------------------------------- 원통(공중): 위·아래 원 + 세로선을 끊김 없이 한 번에
# MoveL 을 blendR(0 이상, 비블로킹)로 연달아 보내 정지 없이 이어 간다. 컨트롤러 명령 큐는 볼 수 없으므로(이 로봇은 CNDE 가 안 붙는다)
# 대신 **로봇의 실제 TCP 위치를 계속 읽어** 로봇이 어느 점까지 지나갔는지 따라가고, 지나간 점보다 LOOKAHEAD 개까지만 앞서서 보낸다.
_cyl = {"state": "idle", "step": "", "sent": 0, "reached": 0, "total": 0, "error": "", "ahead_max": 0, "starved": 0}
TRACE_MAX = 400000
LOOKAHEAD = 8            # 로봇이 지나간 점보다 최대 이만큼만 앞서 보낸다(컨트롤러에 쌓아 두는 점의 수)
REACH_CAP_MM = 4.0       # 로봇 위치가 점에서 이 거리 안이면 그 점을 지나간 것으로 본다(가장 짧은 구간의 60%를 넘지 않게)
NO_PROGRESS_S = 6.0      # 로봇 위치가 이 시간 동안 거의 안 변하는데 앞에 보낸 점이 남아 있으면 멈춘 것으로 보고 정지한다
POSE_FAIL_MAX = 5        # 위치 읽기가 연속으로 이만큼 실패하면 정지한다
SEND_SLEEP_S = 0.01


def _set_cyl(**kw):
    with _lock:
        _cyl.update(kw)


def _tcp(rm):
    """로봇의 현재 TCP 위치(mm). 컨트롤러에 직접 묻는 방식이라 CNDE 와 무관하다."""
    with rm._rpc_lock:
        r = rm.robot.GetActualTCPPose(1)
    if isinstance(r, (tuple, list)) and len(r) == 2 and r[0] == 0 and len(r[1]) >= 6:
        return [float(v) for v in r[1][:6]]
    raise RuntimeError(f"위치를 읽지 못했어요(응답 {r})")


def _cyl_params(data):
    try:
        r = float(data.get("r", CY.DEFAULT_R_MM)); z0 = float(data.get("z0", CY.DEFAULT_Z0_MM))
        h = float(data.get("height", CY.DEFAULT_H_MM)); seg = int(data.get("segments", 36))
    except (TypeError, ValueError):
        raise CY.CylError("반지름·높이·점 수는 숫자여야 해요.")
    return CY.route(_analyze(), r=r, z0=z0, height=h, segments=seg)


def _follow_chain(rm, start, chain, blends, reaches, vel, report, label=lambda i: "", trace=None):
    """로봇이 start 에 이미 있다고 보고, chain 을 blend 로 이어서 보낸다. 로봇의 실제 위치로 지나간 점을 따라가며 LOOKAHEAD 개까지만 앞서 보낸다.
    blends[i]·reaches[i] 는 chain[i] 의 평활 반경(마지막은 -1 = 도착까지 기다림)과 '지나갔다'고 보는 거리다.
    report(**상태) 로 진행을 알린다. 반환: ("ok", "") | ("stopped", "") | ("error", 메시지) — 에러면 이미 로봇을 정지시켰다."""
    route = [start] + chain                             # route[0] = 시작점, route[i] = chain[i-1]
    pts = [q[:3] for q in route]
    tool, user = rm.default_tool, rm.default_user
    sent = reached = 0                                  # sent: 지금까지 보낸 chain 점 수, reached: 로봇이 지나간 route 인덱스
    ahead_max = starved = 0
    starving = False
    fails = 0
    last_pose, last_move = None, time.time()
    while sent < len(chain):
        if _abort.is_set():
            return "stopped", ""
        try:
            tcp = _tcp(rm)
            fails = 0
        except Exception as e:
            fails += 1
            if fails >= POSE_FAIL_MAX:
                rm.stop()
                return "error", f"로봇 위치를 계속 읽지 못해 멈췄어요: {e}"
            time.sleep(SEND_SLEEP_S)
            continue
        if trace is not None and len(trace) < TRACE_MAX and (not trace or math.dist(tcp[:3], trace[-1]) > 0.2):
            trace.append(tcp[:3])                          # 실제로 지나간 로봇 위치(계획 경로와 비교하려고 모아 둔다)
        for i in range(reached + 1, min(reached + LOOKAHEAD + 2, len(route))):     # 지나간 점 갱신(앞쪽 창 안에서만)
            if math.dist(tcp[:3], pts[i]) <= reaches[i - 1]:
                reached = i
        ahead = sent - reached
        ahead_max = max(ahead_max, ahead)
        now = time.time()
        if last_pose is None or math.dist(tcp[:3], last_pose) > 0.3:
            last_pose, last_move = tcp[:3], now
        elif ahead >= 1 and now - last_move > NO_PROGRESS_S:
            rm.stop()
            return "error", "로봇이 더 이상 움직이지 않는데 아직 지나가지 못한 점이 있어요. 안전하게 멈췄어요."
        if ahead <= 0 and sent > 0:                      # 로봇이 보낸 점을 다 지나갔는데 더 못 보냈다 = 보내는 속도가 못 따라가 끊겼을 수 있다
            if not starving:
                starved += 1
            starving = True
        else:
            starving = False
        if ahead < LOOKAHEAD:
            i = sent                                     # 보낼 chain 인덱스
            with rm._rpc_lock:
                ret = rm.robot.MoveL(chain[i], tool=tool, user=user, vel=vel, blendR=-1.0 if i == len(chain) - 1 else blends[i])
            code = _code(ret)
            if code not in (0, None):
                if _abort.is_set():
                    return "stopped", ""
                rm.stop()
                return "error", f"{i + 1}번째 점 이동을 로봇이 거부했어요(코드 {code}). 로봇 상태와 에러를 확인하세요."
            sent += 1
            report(step=label(i), sent=sent, reached=reached, ahead_max=ahead_max, starved=starved)
        else:
            report(reached=reached, ahead_max=ahead_max, starved=starved)
            time.sleep(SEND_SLEEP_S)
    report(sent=len(chain), reached=len(chain), ahead_max=ahead_max, starved=starved)
    return "ok", ""


def _tracking_error(route, trace):
    """계획한 경로(route: 자세 목록)와 실제 지나간 위치(trace)의 거리 통계(mm). 경로를 0.25mm 간격으로 촘촘히 찍어 가장 가까운 점까지의 거리를 잰다."""
    if len(trace) < 5 or len(route) < 2:
        return None
    from scipy.spatial import cKDTree
    pts = []
    for i in range(len(route) - 1):
        p, q = route[i][:3], route[i + 1][:3]
        n = max(1, int(math.dist(p, q) / 0.25))
        pts += [[p[k] + (q[k] - p[k]) * t / n for k in range(3)] for t in range(n)]
    pts.append(list(route[-1][:3]))
    d = sorted(float(x) for x in cKDTree(pts).query(trace)[0])
    return {"n": len(d), "mean": round(sum(d) / len(d), 2), "p95": round(d[int(0.95 * (len(d) - 1))], 2), "max": round(d[-1], 2)}


def _stream(rm, start, chain, blends, reaches, vel, setter, label, on_trace=None):
    """start 로 블로킹 이동한 뒤 chain 을 이어서 보낸다(원통·직선 이음 그리기용)."""
    try:
        setter(state="running", step="시작점으로 이동", sent=0, reached=0, total=len(chain), error="", ahead_max=0, starved=0)
        code = _code(rm.move_to_pose(start, vel=vel))            # 시작점까지는 블로킹으로 정확히
        if code not in (0, None):
            setter(state="stopped" if _abort.is_set() else "error", step="",
                   error="" if _abort.is_set() else f"시작점 이동을 로봇이 거부했어요(코드 {code}).")
            return
        trace = []
        status, msg = _follow_chain(rm, start, chain, blends, reaches, vel, setter, label, trace)
        try:
            setter(track=_tracking_error([start] + chain, trace))
            if on_trace:
                on_trace(trace)
        except Exception:
            pass
        if status == "ok":
            setter(state="done", step="")
        elif status == "stopped":
            setter(state="stopped", step="")
        else:
            setter(state="error", step="", error=msg)
    except Exception as e:
        try:
            rm.stop()
        except Exception:
            pass
        setter(state="error", step="", error=f"이동 중 오류가 났어요: {e}")
    finally:
        _motion_lock.release()


def _cyl_worker(rm, rt, vel):
    def label(i):                                       # i: chain 인덱스
        for name, a, b in rt["parts"]:
            if a <= i < b:
                return name
        return ""

    n = len(rt["chain"])
    reach = min(REACH_CAP_MM, 0.6 * rt["info"]["shortest_segment_mm"])
    _stream(rm, rt["start"], rt["chain"], [rt["blend_mm"]] * n, [reach] * n, vel, _set_cyl, label)


@app.post("/api/cyl/check")
def cyl_check():
    """로봇 위치를 읽을 수 있는지 확인한다(읽기 전용). 이 값으로 이동 진행을 따라가요."""
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        t0 = time.time(); a = _tcp(rm); time.sleep(0.2); b = _tcp(rm)
        return jsonify({"ok": True, "tcp": a, "read_ms": round((time.time() - t0 - 0.2) * 500, 1)})
    except Exception as e:
        return _err(f"위치를 읽지 못했어요: {e}", 502)


@app.post("/api/cyl/plan")
def cyl_plan():
    """원통 경로를 미리 계산해 보여 준다. 로봇에는 아무 명령도 보내지 않는다."""
    try:
        rt = _cyl_params(request.get_json(silent=True) or {})
    except (L.LoopError, CY.CylError) as e:
        return _err(str(e))
    return jsonify({"info": rt["info"], "blend_mm": rt["blend_mm"], "start": rt["start"], "end": rt["chain"][-1],
                    "parts": [{"name": n, "from": a, "to": b} for n, a, b in rt["parts"]]})


@app.post("/api/cyl/run")
def cyl_run():
    data = request.get_json(silent=True) or {}
    try:
        vel = float(data.get("vel", 10))
    except (TypeError, ValueError):
        return _err("속도는 숫자여야 해요.")
    if not (1.0 <= vel <= MAX_LIFT_VEL):
        return _err(f"속도는 1~{MAX_LIFT_VEL:.0f}% 사이여야 해요.")
    if not data.get("confirm"):
        return _err("로봇 이동 경로 주변이 비어 있는지 확인하고 체크해 주세요.", 409)
    try:
        rt = _cyl_params(data)
    except (L.LoopError, CY.CylError) as e:
        return _err(str(e))
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        st = rm.status()
        if any(c not in (0, None) for c in (st.get("error_code") or [])):
            return _err(f"로봇에 에러가 있어요({st.get('error_code')}). 에러를 초기화한 뒤 다시 시도해 주세요.", 409)
        L.check_start(st["tcp_pose_mm_deg"], rt["start"])
    except L.LoopError as e:
        return _err(str(e), 409)
    except Exception as e:
        return _err(f"로봇 상태를 읽지 못했어요: {e}", 502)
    if not _motion_lock.acquire(blocking=False):
        return _err("이미 움직이는 중이에요.", 409)
    _abort.clear()
    threading.Thread(target=_cyl_worker, args=(rm, rt, vel), daemon=True).start()
    return jsonify({"started": True, "points": rt["info"]["points"], "blend_mm": rt["blend_mm"], "vel": vel})


# ---------------------------------------------------------------- SVG 그리기: SVG 의 선을 종이 구역에 맞춰 끊김 없이 이어서 그린다(원통과 같은 이동 방식)
_draw = {"state": "idle", "step": "", "sent": 0, "reached": 0, "total": 0, "error": "", "ahead_max": 0, "starved": 0, "fallback": 0, "track": None}


def _set_draw(**kw):
    with _lock:
        _draw.update(kw)


def _draw_plan(data):
    try:
        opts = dict(rotate=int(data.get("rotate", 0)), mirror=bool(data.get("mirror", False)), margin=float(data.get("margin", D.MARGIN_MM)),
                    scale_pct=float(data.get("scale_pct", 100)), lift=float(data.get("lift", 8)), mode=data.get("mode"),
                    air_mm=float(data.get("air_mm", 30)), tol=float(data.get("tol", D.TOL_MM)),
                    min_seg=float(data.get("min_seg", D.MIN_SEG_MM)), min_stroke=float(data.get("min_stroke", D.MIN_STROKE_MM)),
                    trim=float(data.get("trim", 0) or 0), order=str(data.get("order") or "topdown"), curve=bool(data.get("curve", False)), curve_step=float(data.get("curve_step", D.CURVE_STEP_MM)), first_n=int(data.get("first_n", 0) or 0))
    except (TypeError, ValueError):
        raise D.DrawError("회전·여백·크기·높이는 숫자여야 해요.")
    level = _level(data)
    t = level / 100.0 if level is not None else (1.0 if data.get("detail") else None)
    if t is not None:                                           # 디테일 수준: 낮으면 짧은 선·점을 버리고 점 간격·오차를 크게, 높으면 펜 굵기에 맞춰 촘촘히
        try:
            pen = max(float(data.get("pen_mm") or 0.8), FILL_PEN_MIN_MM)
        except (TypeError, ValueError):
            pen = 0.8
        try:
            floor = float(data.get("seg_floor") or 1.0)
        except (TypeError, ValueError):
            floor = 1.0
        seg_lo = max(0.3, floor, min(1.2, 0.9 * pen))               # 로봇이 한 점씩 따라가는 한계: 점 간격은 floor(기본 1mm) 아래로 내려가지 않는다
        tol_lo = max(0.03, min(0.1, 0.25 * pen))
        opts.update(min_stroke=round(max((1 - t) ** 2 * 6.0, 0.4 * pen), 2), keep_dots=t >= 0.7,   # 펜 굵기의 0.4배보다 짧은 조각만 뺀다(눈동자 속 같은 짧은 채우기 획은 남긴다. 먼지 같은 짧은 조각은 변환 단계에서 이미 걸러진다)
                    
                    min_seg=round(seg_lo + (max(opts["min_seg"], seg_lo) - seg_lo) * (1 - t), 3), tol=round(tol_lo + (0.4 - tol_lo) * (1 - t) ** 2, 3))
    parsed = S.parse_svg(data.get("svg"))
    info0 = _analyze()
    seg0 = opts["min_seg"]
    for _ in range(12):                                         # 점이 너무 많으면(가는 펜으로 꽉 채울 때) 점 간격을 조금씩 넓혀서 한도 안에 맞춘다
        try:
            plan = D.plan(info0, parsed["polylines"], **opts)
            break
        except D.TooManyPoints:
            if opts["min_seg"] >= 4.0:
                raise
            opts["min_seg"] = round(opts["min_seg"] * 1.25, 3)
    plan["warnings"] = parsed["warnings"] + plan["warnings"]
    if opts["min_seg"] > seg0:
        plan["warnings"].append(f"점이 너무 많아서 점 간격을 {seg0}mm → {opts['min_seg']}mm 로 넓혔어요. 아주 작은 굴곡은 조금 각져 보일 수 있어요.")
    if plan["info"]["points"] > 10000:
        plan["warnings"].append(f"점이 {plan['info']['points']}개, 선 길이 약 {plan['info']['pen_mm'] / 1000:.1f}m 라 시간이 오래 걸려요. 속도가 높으면 점을 못 따라 보내 끊길 수 있으니 처음엔 \"처음 N획만\"으로 시험하세요.")
    if parsed.get("pen_px"):
        plan["info"]["pen_width_mm"] = round(parsed["pen_px"] * plan["info"]["scale"], 2)       # 이 그림을 원본과 같게 그리려면 필요한 펜 굵기
    return plan


SPLINE_MS_RANGE = (50, 10000)   # NewSplineStart 의 averageTime(ms). 의미(구간당 시간인지)는 이 로봇에서 확인되지 않았다 → 화면에서 조절한다
ARRIVE_REACH_MM = 1.5
ARRIVE_STILL_MM = 0.2
ARRIVE_STILL_S = 0.12      # 끝점 근처에서 이만큼 가만히 있으면 도착으로 본다(획마다 쌓이는 대기 시간)
ARRIVE_POLL_S = 0.02


def _arrive(rm, target):
    """스플라인은 보내 놓고 바로 돌아오므로, 로봇이 끝점에 도착해 멈출 때까지 위치를 읽으며 기다린다. 'ok' | 'stopped' | 'stuck'"""
    last, still_since, last_move = None, None, time.time()
    fails = 0
    while True:
        if _abort.is_set():
            return "stopped"
        try:
            tcp = _tcp(rm)[:3]; fails = 0
        except Exception:
            fails += 1
            if fails >= POSE_FAIL_MAX:
                return "stuck"
            time.sleep(0.05)
            continue
        moving = last is None or math.dist(tcp, last) > ARRIVE_STILL_MM
        if moving:
            last_move, still_since = time.time(), None
        elif math.dist(tcp, target[:3]) <= ARRIVE_REACH_MM:
            still_since = still_since or time.time()
            if time.time() - still_since >= ARRIVE_STILL_S:
                return "ok"
        elif time.time() - last_move > NO_PROGRESS_S:
            return "stuck"
        last = tcp
        time.sleep(ARRIVE_POLL_S)


def _spline_worker(rm, plan, vel, avg_ms, spline_vel=None):
    """곡선 모드: 획마다 직선 이동(펜 들기·내리기)과 컨트롤러 스플라인(NewSpline*, 조절점을 부드러운 곡선으로 지남)을 차례로 보낸다."""
    ops = plan["ops"]
    tool, user = rm.default_tool, rm.default_user
    svel = spline_vel or vel                                   # 곡선(스플라인)만 따로 속도를 정할 수 있다
    try:
        _set_draw(state="running", step="", sent=0, reached=0, total=len(ops), error="", ahead_max=0, starved=0, fallback=0, track=None)
        for i, (kind, arg, name) in enumerate(ops):
            if _abort.is_set():
                _set_draw(state="stopped", step="")
                return
            _set_draw(step=name, sent=i, reached=i)
            if kind == "move":
                code = _code(rm.move_to_pose(arg, vel=vel))
                bad = f"'{name}' 이동을 로봇이 거부했어요(코드 {code})." if code not in (0, None) else ""
            elif kind == "chain":                                  # 짧은 조각들: 멈추지 않고 이어서(블렌드), 마지막 점에서만 도착을 기다린다
                here = _tcp(rm)
                base = _draw["starved"]

                def rep(**kw):                                     # 끊겼을 수 있는 횟수는 누적해서 보여 준다
                    if "starved" in kw:
                        _set_draw(starved=base + kw["starved"], ahead_max=max(_draw["ahead_max"], kw.get("ahead_max", 0)))
                status, msg = _follow_chain(rm, here, arg["poses"], arg["blends"], arg["reaches"], vel, rep)
                if status == "stopped":
                    _set_draw(state="stopped", step="")
                    return
                bad = msg
            else:
                bad = ""
                with rm._rpc_lock:
                    code = _code(rm.robot.NewSplineStart(1, avg_ms))
                    if code not in (0, None):
                        bad = f"스플라인 시작을 로봇이 거부했어요(코드 {code})."
                    for j, p in enumerate(arg):
                        if bad:
                            break
                        code = _code(rm.robot.NewSplinePoint(p, tool, user, 1 if j == len(arg) - 1 else 0, ovl=svel))
                        if code not in (0, None):
                            bad = f"스플라인 {j + 1}번째 점을 로봇이 거부했어요(코드 {code})."
                    if not bad:
                        code = _code(rm.robot.NewSplineEnd())
                        if code not in (0, None):
                            bad = f"스플라인 끝 명령을 로봇이 거부했어요(코드 {code})."
                res = "ok" if bad else _arrive(rm, arg[-1])
                if res == "stopped":
                    _set_draw(state="stopped", step="")
                    return
                if bad or res == "stuck":
                    # 스플라인이 안 움직이거나 거부되면 멈추지 않고, 같은 조절점들을 짧은 직선 이동으로 이어서 계속한다(곡선만 직선으로 대체)
                    rm.stop()
                    time.sleep(0.5)
                    _set_draw(fallback=_draw["fallback"] + 1, step=name + " (직선으로 대체)")
                    bad = ""
                    for p in arg[1:]:
                        if _abort.is_set():
                            _set_draw(state="stopped", step="")
                            return
                        code = _code(rm.move_to_pose(p, vel=vel))
                        if code not in (0, None):
                            bad = f"'{name}' 곡선이 안 되어 직선으로 대체하려 했지만 로봇이 거부했어요(코드 {code})."
                            break
            if bad:
                if _abort.is_set():
                    _set_draw(state="stopped", step="")
                else:
                    _set_draw(state="error", step="", error=bad)
                    rm.stop()
                return
        _set_draw(state="done", step="", sent=len(ops), reached=len(ops))
    except Exception as e:
        try:
            rm.stop()
        except Exception:
            pass
        _set_draw(state="error", step="", error=f"이동 중 오류가 났어요: {e}")
    finally:
        _motion_lock.release()


TRACE_FILE = os.path.join(HERE, "paper_last_trace.json")     # 서버를 다시 켜도 마지막 실제 경로가 남도록 파일에도 저장한다
try:
    with open(TRACE_FILE, encoding="utf-8") as _f:
        _last_trace = json.load(_f)
except (OSError, ValueError):
    _last_trace = {"time": "", "uv": []}


def _save_trace(trace):
    """실제로 지나간 로봇 위치(TCP)를 펜 끝 위치로 바꿔 종이 평면 좌표 (u, v, 종이 위 높이)로 저장한다. 미리보기와 겹쳐 비교하려고 쓴다."""
    info = _analyze()
    fr = CY.frame(info)
    R = L.rot(*info["orientation"])
    off = L._tool_off(R, info["pen_off"])
    step = max(1, len(trace) // 30000)
    uv = []
    for p in trace[::step]:
        d = [p[i] + off[i] - fr["o"][i] for i in range(3)]
        uv.append([round(CY._dot(d, fr["u"]), 2), round(CY._dot(d, fr["v"]), 2), round(CY._dot(d, fr["n"]) - fr["plane_z"], 2)])
    with _lock:
        _last_trace.update(time=time.strftime("%Y-%m-%d %H:%M:%S"), uv=uv)
        tmp = TRACE_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(_last_trace, f)
        os.replace(tmp, TRACE_FILE)


@app.get("/api/draw/trace")
def draw_trace():
    """마지막 그리기에서 로봇이 실제로 지나간 경로(펜 끝, 종이 평면 좌표)."""
    with _lock:
        return jsonify(dict(_last_trace))


def _draw_worker(rm, plan, vel):
    stroke_of, n = plan["stroke_of"], plan["info"]["strokes"]

    def label(i):
        s = stroke_of[i]
        return f"획 {s + 1}/{n} 그리는 중" if s >= 0 else "펜 들고 이동 중"

    _stream(rm, plan["start"], plan["chain"], plan["blends"], plan["reaches"], vel, _set_draw, label, on_trace=_save_trace)


# MindDraw(../painting)가 만든 선그림 목록과 그림 → SVG 변환(중심선 따라가기)
ART_DIR = os.path.join(os.environ.get("MINDDRAW_DATA") or os.path.join(HERE, "..", "painting", "data"), "current", "images")
ART_RE = re.compile(r"^(art_\d+|panel_\d+_[1-4])\.png$")   # art = 장면 그림 전체, panel = 네 컷 중 한 컷(그 컷만 종이 전체 크기로 그린다)


@app.get("/api/draw/images")
def draw_images():
    names = sorted(n for n in (os.listdir(ART_DIR) if os.path.isdir(ART_DIR) else []) if ART_RE.match(n))
    return jsonify({"images": names, "dir": os.path.abspath(ART_DIR)})


@app.get("/api/draw/image/<name>")
def draw_image(name):
    if not ART_RE.match(name):
        return _err("이미지 이름이 올바르지 않아요.", 404)
    return send_from_directory(ART_DIR, name)


def _overlap(data, default=None):
    """채우기 고리를 겹쳐 그리는 비율(0~60%). 화면 입력은 %, 없으면 default(없으면 기본값)."""
    v = data.get("overlap")
    if v is None or v == "":
        return V.FILL_OVERLAP if default is None else default
    v = float(v) / 100.0
    if not 0.0 <= v <= 0.6:
        raise ValueError("채우기 겹침은 0~60% 사이여야 해요.")
    return v


def _level(data):
    """디테일 수준 0~100(화면 슬라이더). 없으면 None(예전 detail 참/거짓을 따른다)."""
    v = data.get("detail_level")
    if v is None or v == "":
        return None
    v = float(v)
    if not 0.0 <= v <= 100.0:
        raise ValueError("디테일 수준은 0~100 사이여야 해요.")
    return v


@app.post("/api/draw/vectorize")
def draw_vectorize():
    """MindDraw 그림(name) 또는 올린 이미지(image_b64)를 중심선 획의 SVG 로 바꾼다. 로봇과 무관하다."""
    data = request.get_json(silent=True) or {}
    try:
        if data.get("name"):
            if not ART_RE.match(str(data["name"])):
                return _err("이미지 이름이 올바르지 않아요.")
            path = os.path.join(ART_DIR, data["name"])
            if not os.path.isfile(path):
                return _err("그 그림이 없어요.", 404)
            with open(path, "rb") as f:
                raw = f.read()
        else:
            raw = base64.b64decode(data.get("image_b64") or "", validate=False)
        fill = bool(data.get("fill", True))
        if data.get("pen_mm"):
            # 펜 끝 굵기(mm) → 그림 속 펜 굵기(px): 저장한 종이에 이 그림을 맞췄을 때의 배율(mm/px)로 환산한다
            pen_mm = float(data["pen_mm"])
            if not 0.05 <= pen_mm <= 10.0:
                return _err("펜 굵기는 0.05~10mm 사이여야 해요.")
            r0 = V.to_strokes(raw, fill=False)
            pl = S.parse_svg(V.to_svg(r0["strokes"], r0["size"]))["polylines"]
            try:
                p0 = D.plan(_analyze(), pl, mode="air", rotate=int(data.get("rotate", 0)), mirror=bool(data.get("mirror", False)),
                            margin=float(data.get("margin", D.MARGIN_MM)), scale_pct=float(data.get("scale_pct", 100)))
            except (D.DrawError, L.LoopError) as e:
                return _err(f"펜 굵기(mm)로 계산하려면 종이 모서리 4곳이 먼저 필요해요: {e}")
            res = V.vectorize(raw, float(data.get("min_stroke_px", V.MIN_STROKE_PX)), fill=fill, pen_px=max(pen_mm, FILL_PEN_MIN_MM) / p0["info"]["scale"], detail=bool(data.get("detail", False)), detail_level=_level(data), overlap=_overlap(data, 0.2 if pen_mm < FINE_PEN_MM else None),
                               clean_px=CLEAN_MM / p0["info"]["scale"])
            res.update(pen_mm=pen_mm, mm_per_px=p0["info"]["scale"])
            if res["pen_px"] < THIN_PEN_PX:
                res["warning"] = (f"펜이 그림 속에서 {res['pen_px']}px 로 원본 선(약 5px)보다 가늘어서, 굵은 선과 검은 부분은 여러 번 겹쳐 그려 원본과 같게 만들어요. "
                                  "그만큼 점이 많아지고 시간이 오래 걸려요.")
            if p0["info"]["scale"] < 0.22 and pen_mm > 0.6:       # 네 컷 전체처럼 한 장에 작은 그림이 촘촘히 든 경우: 굵은 펜으로는 눈·입 같은 작은 부분이 뭉개진다
                res["warning"] = ((res.get("warning") or "") + " 이 그림은 작은 그림이 촘촘히 들어 있어서 펜이 굵으면 눈·입 같은 부분이 뭉개져요. 네 컷 중 한 컷만 고르면(그림 목록의 '네 컷 중 N컷만') 종이 전체 크기로 그려서 또렷해요.").strip()
            return jsonify(res)
        return jsonify(V.vectorize(raw, float(data.get("min_stroke_px", V.MIN_STROKE_PX)), float(data.get("pen_pct", 60)), fill, detail=bool(data.get("detail", False)), detail_level=_level(data), overlap=_overlap(data)))
    except (V.VecError, ValueError) as e:
        return _err(str(e))


@app.post("/api/draw/plan")
def draw_plan():
    """SVG 를 읽어 종이에 맞춘 경로를 미리 계산해 보여 준다. 로봇에는 아무 명령도 보내지 않는다."""
    try:
        plan = _draw_plan(request.get_json(silent=True) or {})
    except (S.SvgError, D.DrawError, L.LoopError) as e:
        return _err(str(e))
    return jsonify({"info": plan["info"], "warnings": plan["warnings"], "paper": plan["paper"], "strokes": plan["strokes"],
                    "start": plan["start"], "end": plan["chain"][-1]})


@app.post("/api/draw/run")
def draw_run():
    data = request.get_json(silent=True) or {}
    try:
        vel = float(data.get("vel", 10))
    except (TypeError, ValueError):
        return _err("속도는 숫자여야 해요.")
    if not (1.0 <= vel <= MAX_DRAW_VEL):
        return _err(f"속도는 1~{MAX_DRAW_VEL:.0f}% 사이여야 해요.")
    try:
        avg_ms = int(data.get("avg_ms", 2000))
        spline_vel = float(data["spline_vel"]) if data.get("spline_vel") not in (None, "", 0) else None
    except (TypeError, ValueError):
        return _err("곡선 시간·곡선 속도는 숫자여야 해요.")
    if spline_vel is not None and not 1.0 <= spline_vel <= MAX_DRAW_VEL:
        return _err(f"곡선 속도는 1~{MAX_DRAW_VEL:.0f}% 사이여야 해요.")
    if not SPLINE_MS_RANGE[0] <= avg_ms <= SPLINE_MS_RANGE[1]:
        return _err(f"곡선 시간은 {SPLINE_MS_RANGE[0]}~{SPLINE_MS_RANGE[1]}ms 사이여야 해요.")
    if not data.get("confirm"):
        return _err("로봇 이동 경로 주변이 비어 있는지 확인하고 체크해 주세요.", 409)
    if data.get("mode") == "draw" and not data.get("confirm_draw"):
        return _err("펜이 종이에 닿아도 안전한지 확인해 주세요.", 409)
    try:
        plan = _draw_plan(data)
    except (S.SvgError, D.DrawError, L.LoopError) as e:
        return _err(str(e))
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        st = rm.status()
        if any(c not in (0, None) for c in (st.get("error_code") or [])):
            return _err(f"로봇에 에러가 있어요({st.get('error_code')}). 에러를 초기화한 뒤 다시 시도해 주세요.", 409)
        L.check_start(st["tcp_pose_mm_deg"], plan["start"])
    except L.LoopError as e:
        return _err(str(e), 409)
    except Exception as e:
        return _err(f"로봇 상태를 읽지 못했어요: {e}", 502)
    if not _motion_lock.acquire(blocking=False):
        return _err("이미 움직이는 중이에요.", 409)
    _abort.clear()
    if plan.get("curve"):
        threading.Thread(target=_spline_worker, args=(rm, plan, vel, avg_ms, spline_vel), daemon=True).start()
    else:
        threading.Thread(target=_draw_worker, args=(rm, plan, vel), daemon=True).start()
    return jsonify({"started": True, "points": plan["info"]["points"], "strokes": plan["info"]["strokes"], "vel": vel})


@app.get("/api/settings")
def settings_get():
    return jsonify(_settings())


def _check_settings(data):
    try:
        s = {"pen_mm": float(data.get("pen_mm", 0)), "pen_x": float(data.get("pen_x", 0)), "pen_y": float(data.get("pen_y", 0)),
             "tip_tcp": bool(data.get("tip_tcp", False)), "align": bool(data.get("align", False))}
    except (TypeError, ValueError):
        raise L.LoopError("펜 끝 오프셋은 숫자여야 해요.")
    if not 0.0 <= s["pen_mm"] <= 500.0 or abs(s["pen_x"]) > 200.0 or abs(s["pen_y"]) > 200.0:
        raise L.LoopError("펜 끝까지 길이는 0~500mm, 옆으로 나온 정도(x, y)는 ±200mm 이내여야 해요.")
    known = any(abs(s[k]) > 0 for k in ("pen_mm", "pen_x", "pen_y"))
    if s["tip_tcp"] and known:
        raise L.LoopError("도구 TCP 가 이미 펜 끝이면 펜 끝 오프셋은 모두 0 이어야 해요(이중으로 더해져요).")
    if s["align"] and not (s["tip_tcp"] or known):
        raise L.LoopError("펜을 수직으로 세우려면 '도구 TCP 가 이미 펜 끝'을 체크하거나 펜 끝 오프셋을 입력해 주세요.")
    return s


def _write_settings(s):
    tmp = SETTINGS_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(s, f, ensure_ascii=False, indent=2)
    os.replace(tmp, SETTINGS_FILE)


@app.post("/api/settings")
def settings_set():
    """펜 끝 보정 설정 저장: pen_mm(펜 끝까지 길이 z), pen_x·pen_y(옆으로 나온 정도), tip_tcp(도구 TCP 가 이미 펜 끝), align(펜을 종이에 수직으로)."""
    try:
        s = _check_settings(request.get_json(silent=True) or {})
    except L.LoopError as e:
        return _err(str(e))
    _write_settings(s)
    return jsonify(s)


# ---------------------------------------------------------------- 펜 끝 교정: 한 점에 펜 끝을 대고 자세를 바꿔 여러 번 저장 → 펜 끝 오프셋(x,y,z)
TCP_FILE = os.path.join(HERE, "paper_tcp_poses.json")


def _tcp_load():
    try:
        with open(TCP_FILE, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return []


def _tcp_store(items, why):
    try:
        with open(HISTORY_FILE, "a", encoding="utf-8") as f:       # 모서리 이력 파일에 같이 남긴다(지우지 않는다)
            f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "why": "tcp " + why, "before": _tcp_load(), "after": items}, ensure_ascii=False) + "\n")
    except OSError:
        pass
    tmp = TCP_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)
    os.replace(tmp, TCP_FILE)


def _tcp_summary(items):
    out = []
    for i, it in enumerate(items):
        axis = L.tool_z_axis(*it["pose"][3:6])
        out.append({"index": i + 1, "pose": it["pose"], "time": it["time"],
                    "angle_from_first_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, sum(axis[k] * L.tool_z_axis(*items[0]["pose"][3:6])[k] for k in range(3)))))), 1)})
    return out


@app.get("/api/tcp")
def tcp_state():
    return jsonify({"poses": _tcp_summary(_tcp_load()), "min": T.MIN_POSES, "max": T.MAX_POSES})


@app.post("/api/tcp/save")
def tcp_save():
    """지금 로봇의 TCP 자세를 교정용으로 하나 추가한다(읽기 명령만 보낸다)."""
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        pose = rm.status()["tcp_pose_mm_deg"]
    except Exception as e:
        return _err(f"현재 위치를 읽지 못했어요: {e}", 502)
    if not pose or len(pose) < 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose[:6]):
        return _err("현재 위치 값이 올바르지 않아요. 다시 시도해 주세요.", 502)
    with _lock:
        items = _tcp_load()
        if len(items) >= T.MAX_POSES:
            return _err(f"교정용 자세는 최대 {T.MAX_POSES}개예요. 불필요한 것을 지우거나 모두 지우고 다시 시작하세요.", 409)
        items.append({"pose": [float(v) for v in pose[:6]], "time": time.strftime("%H:%M:%S")})
        _tcp_store(items, "save")
    return jsonify({"poses": _tcp_summary(items)})


@app.post("/api/tcp/remove")
def tcp_remove():
    idx = (request.get_json(silent=True) or {}).get("index")
    with _lock:
        items = _tcp_load()
        if idx == "all":
            items = []
        elif isinstance(idx, int) and 1 <= idx <= len(items):
            items.pop(idx - 1)
        else:
            return _err("지울 번호 또는 all 을 알려 주세요.")
        _tcp_store(items, f"remove {idx}")
    return jsonify({"poses": _tcp_summary(items)})


@app.post("/api/tcp/solve")
def tcp_solve():
    """저장한 자세들로 펜 끝 오프셋을 계산한다(저장·적용은 하지 않는다)."""
    try:
        return jsonify(T.solve([it["pose"] for it in _tcp_load()]))
    except T.TcpError as e:
        return _err(str(e))


@app.post("/api/tcp/apply")
def tcp_apply():
    """계산한 펜 끝 오프셋을 펜 끝 설정(pen_x, pen_y, pen_mm)으로 저장한다. 수직 보정 선택은 그대로 둔다."""
    try:
        r = T.solve([it["pose"] for it in _tcp_load()])
        s = _settings()
        x, y, z = r["offset"]
        s.update(pen_x=round(x, 1), pen_y=round(y, 1), pen_mm=round(z, 1), tip_tcp=False)
        if z < 0 or math.sqrt(x * x + y * y + z * z) > 500.0 or abs(x) > 200.0 or abs(y) > 200.0:
            return _err(f"계산한 오프셋 {r['offset']} 이 허용 범위를 벗어나요. 자세를 다시 저장해 주세요.")
        s = _check_settings(s)
    except (T.TcpError, L.LoopError) as e:
        return _err(str(e))
    _write_settings(s)
    r["applied"] = s
    return jsonify(r)


@app.get("/api/plane")
def plane():
    """저장한 모서리와 펜 설정으로 계산한 종이 평면 확인(읽기 전용). 판 기울기, 펜-법선 각도, 평면성 등."""
    try:
        info = _analyze()
    except L.LoopError as e:
        return _err(str(e))
    return jsonify({k: info[k] for k in ("sides_mm", "angles_deg", "flatness_mm", "ori_spread_deg", "pen_normal_deg", "pen_mm", "pen_off", "tip_tcp", "align",
                                         "align_deg", "elevation_deg", "warnings", "normal", "orientation")})


@app.post("/api/tool/check")
def tool_check():
    """로봇의 현재 도구 TCP 오프셋을 읽는다(읽기 전용). 0 이면 TCP 가 플랜지 중심이고 펜 끝이 아니다."""
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        with rm._rpc_lock:
            r = rm.robot.GetTCPOffset(1)
    except Exception as e:
        return _err(f"도구 오프셋을 읽지 못했어요: {e}", 502)
    if not (isinstance(r, (tuple, list)) and len(r) == 2 and r[0] == 0 and r[1]):
        return _err(f"도구 오프셋 응답이 올바르지 않아요: {r}", 502)
    off = [float(v) for v in r[1][:6]]
    mag = math.sqrt(sum(v * v for v in off[:3]))
    return jsonify({"offset": off, "length_mm": round(mag, 1), "is_flange": mag < 1.0})


# ---------------------------------------------------------------- 그리기 세팅 저장·불러오기(이름 붙여서 기억해 둔다)
PRESETS_FILE = os.path.join(HERE, "paper_presets.json")


def _presets():
    try:
        with open(PRESETS_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


@app.get("/api/presets")
def presets_get():
    return jsonify(_presets())


@app.post("/api/presets")
def presets_set():
    """이름을 붙여 그리기 화면의 입력값들을 저장한다(같은 이름이면 덮어쓴다). values 는 {입력칸 id: 값(숫자·글자·참거짓)}."""
    data = request.get_json(silent=True) or {}
    name = str(data.get("name", "")).strip()[:40]
    values = data.get("values")
    if not name:
        return _err("세팅 이름을 입력해 주세요.")
    if not isinstance(values, dict) or not 0 < len(values) <= 60 or len(json.dumps(values)) > 4000:
        return _err("저장할 세팅 값이 올바르지 않아요.")
    if not all(isinstance(k, str) and len(k) <= 40 and isinstance(v, (int, float, str, bool)) and len(str(v)) <= 60 for k, v in values.items()):
        return _err("저장할 세팅 값이 올바르지 않아요.")
    with _lock:
        p = _presets()
        if name in p:                                   # 덮어쓰기 전에 이전 값을 이력에 남긴다
            try:
                with open(HISTORY_FILE, "a", encoding="utf-8") as f:
                    f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "why": "preset overwrite " + name, "before": p[name], "after": values}, ensure_ascii=False) + "\n")
            except OSError:
                pass
        p[name] = values
        tmp = PRESETS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(p, f, ensure_ascii=False, indent=2)
        os.replace(tmp, PRESETS_FILE)
    return jsonify(p)


@app.post("/api/presets/delete")
def presets_delete():
    name = str((request.get_json(silent=True) or {}).get("name", ""))
    with _lock:
        p = _presets()
        if name not in p:
            return _err("그런 세팅이 없어요.", 404)
        try:
            with open(HISTORY_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "why": "preset delete " + name, "before": p[name], "after": None}, ensure_ascii=False) + "\n")
        except OSError:
            pass
        del p[name]
        tmp = PRESETS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(p, f, ensure_ascii=False, indent=2)
        os.replace(tmp, PRESETS_FILE)
    return jsonify(p)


# ---------------------------------------------------------------- 펜 끝 고정 자세 변경: 펜 끝은 그 자리에 두고 손목만 돌려 펜을 세우거나 기울인다
_pen = {"state": "idle", "step": "", "error": "", "sent": 0, "total": 0}


def _set_pen(**kw):
    with _lock:
        _pen.update(kw)


def _pen_plan(data):
    """현재 TCP 자세에서 펜 끝을 고정한 채 목표 자세로 가는 경로. 반환: (계획, 현재 TCP, 현재 펜-법선 각도)"""
    s = _settings()
    off = [float(s["pen_x"]), float(s["pen_y"]), float(s["pen_mm"])]
    if not (s["tip_tcp"] or any(abs(v) > 0 for v in off)):
        raise PN.PenError("펜 끝 위치를 모르면 펜 끝을 고정할 수 없어요. 위 '펜 끝 · 기울기 보정'에서 펜 끝 오프셋을 먼저 정해 주세요.")
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        raise PN.PenError("먼저 로봇에 연결해 주세요.")
    try:
        tcp = [float(v) for v in rm.status()["tcp_pose_mm_deg"][:6]]
    except Exception as e:
        raise PN.PenError(f"현재 위치를 읽지 못했어요: {e}")
    info = _analyze()
    fr = CY.frame(info)
    mode = data.get("mode")
    try:
        deg = float(data.get("deg", 0))
    except (TypeError, ValueError):
        raise PN.PenError("각도는 숫자여야 해요.")
    if mode == "tilt" and not 0.5 <= deg <= PN.MAX_DEG:
        raise PN.PenError(f"기울일 각도는 0.5~{PN.MAX_DEG:.0f}° 사이여야 해요.")
    n_ = info["normal"]
    P_ = info["P"]
    d14 = [P_[3][i] - P_[0][i] for i in range(3)]                  # 모서리 1 → 4 방향(판 안으로 투영)
    d14 = [d14[i] - n_[i] * sum(d14[k] * n_[k] for k in range(3)) for i in range(3)]
    m14 = math.sqrt(sum(c * c for c in d14))
    Rt = PN.target_rotation(L.rot(*tcp[3:6]), mode, n_, data.get("dir"), deg, fr["u"], [c / m14 for c in d14])
    plan = PN.plan(tcp, off if not s["tip_tcp"] else [0.0, 0.0, 0.0], Rt)
    return plan, tcp, round(PN.pen_angle_deg(tcp, info["normal"]), 1), round(PN.pen_angle_deg(plan["poses"][-1], info["normal"]), 1)


@app.post("/api/pen/plan")
def pen_plan():
    """현재 자세에서 펜 끝을 고정한 채 돌렸을 때의 경로를 미리 계산한다(로봇은 읽기만 한다)."""
    try:
        plan, tcp, before, after = _pen_plan(request.get_json(silent=True) or {})
    except (PN.PenError, L.LoopError) as e:
        return _err(str(e), 409 if "연결" in str(e) else 400)
    return jsonify({**{k: plan[k] for k in ("tip", "angle_deg", "tcp_travel_mm", "tip_dev_mm", "steps")}, "pen_angle_before": before, "pen_angle_after": after, "tcp": tcp})


def _pen_worker(rm, tcp, plan, vel):
    chain = plan["poses"]
    route = [tcp] + chain
    d = [math.dist(route[i][:3], route[i + 1][:3]) for i in range(len(route) - 1)]
    blends = [-1.0 if i == len(chain) - 1 else round(min(3.0, 0.4 * min(d[i], d[i + 1])), 2) for i in range(len(chain))]
    reaches = [round(min(REACH_CAP_MM, max(1.5, 0.6 * min(d[i], d[i + 1] if i + 1 < len(d) else d[i]))), 2) for i in range(len(chain))]
    try:
        _set_pen(state="running", step="펜 끝 고정 회전", error="", sent=0, total=len(chain))
        status, msg = _follow_chain(rm, tcp, chain, blends, reaches, vel, lambda **kw: _set_pen(**{k: kw[k] for k in ("sent",) if k in kw}))
        _set_pen(state={"ok": "done", "stopped": "stopped", "error": "error"}[status], step="", error=msg)
    except Exception as e:
        try:
            rm.stop()
        except Exception:
            pass
        _set_pen(state="error", step="", error=f"이동 중 오류가 났어요: {e}")
    finally:
        _motion_lock.release()


@app.post("/api/pen/run")
def pen_run():
    data = request.get_json(silent=True) or {}
    try:
        vel = float(data.get("vel", 5))
    except (TypeError, ValueError):
        return _err("속도는 숫자여야 해요.")
    if not 1.0 <= vel <= MAX_LIFT_VEL:
        return _err(f"속도는 1~{MAX_LIFT_VEL:.0f}% 사이여야 해요.")
    if not data.get("confirm"):
        return _err("펜 둘레(손목이 도는 공간)가 비어 있는지 확인하고 체크해 주세요.", 409)
    try:
        plan, tcp, before, after = _pen_plan(data)
        st = _get_rm().status()
        if any(c not in (0, None) for c in (st.get("error_code") or [])):
            return _err(f"로봇에 에러가 있어요({st.get('error_code')}). 에러를 초기화한 뒤 다시 시도해 주세요.", 409)
    except (PN.PenError, L.LoopError) as e:
        return _err(str(e), 409 if "연결" in str(e) else 400)
    if not _motion_lock.acquire(blocking=False):
        return _err("이미 움직이는 중이에요.", 409)
    _abort.clear()
    threading.Thread(target=_pen_worker, args=(_get_rm(), tcp, plan, vel), daemon=True).start()
    return jsonify({"started": True, "angle_deg": plan["angle_deg"], "pen_angle_before": before, "pen_angle_after": after})


@app.get("/api/pen/angle")
def pen_angle():
    """지금 펜이 판에 수직에서 얼마나 기울었는지(읽기 전용)."""
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        tcp = [float(v) for v in rm.status()["tcp_pose_mm_deg"][:6]]
        return jsonify({"pen_angle": round(PN.pen_angle_deg(tcp, _analyze()["normal"]), 1), "state": dict(_pen)})
    except Exception as e:
        return _err(str(e), 502)


@app.get("/")
def index():
    return send_from_directory(HERE, "index.html")


_status_cache = {}


def _any_running():
    with _lock:
        return "running" in (_motion["state"], _loop["state"], _cyl["state"], _draw["state"], _pen["state"])


@app.get("/api/state")
def state():
    rm = _get_rm()
    points = _load()
    out = {"available": rm is not None, "reason": _rm_error, "connected": False, "ip": "", "tcp": None,
           "error_code": None, "points": points, "distances": _distances(points)}
    with _lock:
        out["motion"] = dict(_motion)
        out["loop"] = dict(_loop)
        out["cyl"] = dict(_cyl)
        out["draw"] = dict(_draw)
        out["pen"] = dict(_pen)
    if rm is not None and rm.is_connected():
        try:
            if _any_running() and _status_cache:    # 이동 중(마지막 블로킹 MoveL 등)에는 연결을 붙잡지 않고 직전 값을 보여 준다
                st = _status_cache
                out["stale"] = True
            else:
                st = rm.status()                   # 읽기 전용 조회
                _status_cache.clear(); _status_cache.update(st)
            out.update(connected=True, ip=st.get("ip", ""), tcp=st.get("tcp_pose_mm_deg"), error_code=st.get("error_code"))
        except Exception as e:
            out["status_error"] = str(e)
    return jsonify(out)


@app.post("/api/connect")
def connect():
    rm = _get_rm()
    if rm is None:
        return _err(_rm_error, 503)
    ip = ((request.get_json(silent=True) or {}).get("ip") or "192.168.58.2").strip()[:40]
    try:
        info = rm.connect(ip)
    except Exception as e:
        return _err(f"로봇에 연결하지 못했어요: {e}", 502)
    return jsonify(info)


@app.post("/api/save")
def save():
    """지금 로봇의 TCP 위치를 모서리 index(1~4)로 저장한다."""
    idx = (request.get_json(silent=True) or {}).get("index")
    if idx not in range(1, COUNT + 1):
        return _err("저장할 모서리 번호(1~4)를 골라 주세요.")
    rm = _get_rm()
    if rm is None or not rm.is_connected():
        return _err("먼저 로봇에 연결해 주세요.", 409)
    try:
        pose = rm.status()["tcp_pose_mm_deg"]
    except Exception as e:
        return _err(f"현재 위치를 읽지 못했어요: {e}", 502)
    if not pose or len(pose) < 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose[:6]):
        return _err("현재 위치 값이 올바르지 않아요. 다시 시도해 주세요.", 502)
    with _lock:
        points = _load()
        points[idx - 1] = {"pose": [float(v) for v in pose[:6]], "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        _save(points, f"save {idx}")
    return jsonify({"saved": idx, "pose": points[idx - 1]["pose"]})


@app.post("/api/clear")
def clear():
    idx = (request.get_json(silent=True) or {}).get("index")
    with _lock:
        points = _load()
        if idx == "all":
            points = [None] * COUNT
        elif idx in range(1, COUNT + 1):
            points[idx - 1] = None
        else:
            return _err("지울 모서리 번호(1~4) 또는 all 을 알려 주세요.")
        _save(points, f"clear {idx}")
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=int(os.environ.get("PORT", "5051")), debug=False, threaded=True)

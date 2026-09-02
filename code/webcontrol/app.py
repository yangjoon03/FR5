"""
FR5 로봇 웹 컨트롤 패널 - Flask 백엔드

실행: python3 app.py  (기본 포트 5000)
브라우저에서 http://localhost:5000 접속

이 서버는 브라우저의 fetch() 요청을 받아 fairino Python SDK
(code/fairino/Robot.py)로 실제 로봇에 명령을 보냅니다. 로봇 컨트롤러와
같은 네트워크(또는 이 PC에서 접근 가능한 네트워크)에 있어야 동작합니다.
"""
import time

from flask import Flask, request, jsonify, send_from_directory, Response

from robot_manager import manager, RobotNotConnected
from vision_tracker import CameraTracker, list_available_cameras

app = Flask(__name__, static_folder="static", static_url_path="")
tracker = CameraTracker(manager)


def ok(data=None):
    return jsonify({"ok": True, "data": data})


def fail(message, status=400):
    return jsonify({"ok": False, "error": str(message)}), status


@app.errorhandler(RobotNotConnected)
def handle_not_connected(e):
    return fail(str(e), 409)


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


# ----------------------------------------------------------------------
# 연결 / 상태 / 활성화 / 모드
# ----------------------------------------------------------------------
@app.route("/api/connect", methods=["POST"])
def api_connect():
    body = request.get_json(force=True) or {}
    ip = body.get("ip", "192.168.58.2")
    try:
        info = manager.connect(ip)
        return ok(info)
    except Exception as e:
        return fail(f"연결 실패: {e}", 500)


@app.route("/api/status", methods=["GET"])
def api_status():
    try:
        return ok(manager.status())
    except RobotNotConnected as e:
        return handle_not_connected(e)
    except Exception as e:
        return fail(str(e), 500)


@app.route("/api/enable", methods=["POST"])
def api_enable():
    body = request.get_json(force=True) or {}
    state = int(body.get("state", 1))
    error = manager.enable(state)
    return ok({"error": error})


@app.route("/api/mode", methods=["POST"])
def api_mode():
    body = request.get_json(force=True) or {}
    state = int(body.get("state", 0))
    error = manager.mode(state)
    return ok({"error": error})


@app.route("/api/drag_teach", methods=["POST"])
def api_drag_teach():
    body = request.get_json(force=True) or {}
    state = int(body.get("state", 0))
    error = manager.drag_teach(state)
    return ok({"error": error})


@app.route("/api/reset_error", methods=["POST"])
def api_reset_error():
    error = manager.reset_error()
    return ok({"error": error})


# ----------------------------------------------------------------------
# 정지 / 일시정지 / 재개
# ----------------------------------------------------------------------
@app.route("/api/stop", methods=["POST"])
def api_stop():
    tracker.stop_tracking()  # 카메라 트래킹이 계속 보정 명령을 보내지 않도록 같이 끔
    error = manager.stop()
    return ok({"error": error})


@app.route("/api/pause", methods=["POST"])
def api_pause():
    error = manager.pause()
    return ok({"error": error})


@app.route("/api/resume", methods=["POST"])
def api_resume():
    error = manager.resume()
    return ok({"error": error})


# ----------------------------------------------------------------------
# 조그 (수동 점동) - 버튼을 누르는 동안만 이동
# ----------------------------------------------------------------------
@app.route("/api/jog/start", methods=["POST"])
def api_jog_start():
    body = request.get_json(force=True) or {}
    error = manager.jog_start(
        ref=int(body.get("ref", 0)),
        nb=int(body.get("nb", 1)),
        direction=int(body.get("dir", 1)),
        max_dis=float(body.get("max_dis", 90)),
        vel=float(body.get("vel", 30)),
    )
    return ok({"error": error})


@app.route("/api/jog/stop", methods=["POST"])
def api_jog_stop():
    body = request.get_json(force=True) or {}
    error = manager.jog_stop(ref=int(body.get("ref", 1)))
    return ok({"error": error})


@app.route("/api/jog/stop_immediate", methods=["POST"])
def api_jog_stop_immediate():
    error = manager.jog_stop_immediate()
    return ok({"error": error})


# ----------------------------------------------------------------------
# 직선/회전 상대 이동 (수직/수평 이동, 제자리 회전)
# ----------------------------------------------------------------------
@app.route("/api/move/linear", methods=["POST"])
def api_move_linear():
    body = request.get_json(force=True) or {}
    try:
        error = manager.move_linear(
            axis=body["axis"], distance_cm=float(body["distance_cm"]),
            sign=body.get("sign", "+"), vel=body.get("vel"), frame=body.get("frame", "base"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


@app.route("/api/move/rotate", methods=["POST"])
def api_move_rotate():
    body = request.get_json(force=True) or {}
    try:
        error = manager.move_rotate(
            axis=body["axis"], angle_deg=float(body["angle_deg"]),
            sign=body.get("sign", "+"), vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


# ----------------------------------------------------------------------
# 도형 그리기
# ----------------------------------------------------------------------
@app.route("/api/shape/circle", methods=["POST"])
def api_shape_circle():
    body = request.get_json(force=True) or {}
    try:
        error = manager.draw_circle(
            radius_cm=float(body["radius_cm"]), plane=body.get("plane", "XY"),
            direction=body.get("direction", "ccw"), vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


@app.route("/api/shape/arc", methods=["POST"])
def api_shape_arc():
    body = request.get_json(force=True) or {}
    try:
        error = manager.draw_arc(
            radius_cm=float(body["radius_cm"]), sweep_deg=float(body["sweep_deg"]),
            plane=body.get("plane", "XY"), direction=body.get("direction", "ccw"), vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


@app.route("/api/shape/polygon", methods=["POST"])
def api_shape_polygon():
    body = request.get_json(force=True) or {}
    try:
        error = manager.draw_polygon(
            sides=int(body["sides"]), side_length_cm=float(body["side_length_cm"]),
            plane=body.get("plane", "XY"), direction=body.get("direction", "ccw"), vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


@app.route("/api/shape/ellipse", methods=["POST"])
def api_shape_ellipse():
    body = request.get_json(force=True) or {}
    try:
        error = manager.draw_ellipse(
            a_cm=float(body["a_cm"]), b_cm=float(body["b_cm"]), plane=body.get("plane", "XY"),
            direction=body.get("direction", "ccw"), segments=int(body.get("segments", 36)), vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


@app.route("/api/shape/spiral", methods=["POST"])
def api_shape_spiral():
    body = request.get_json(force=True) or {}
    try:
        error = manager.draw_spiral(
            turns=float(body["turns"]), tilt_deg=float(body.get("tilt_deg", 0)),
            radius_init_cm=float(body["radius_init_cm"]), radius_add_cm=float(body.get("radius_add_cm", 0)),
            axis_add_cm=float(body.get("axis_add_cm", 0)), direction=body.get("direction", "cw"),
            vel=body.get("vel"),
        )
        return ok({"error": error})
    except (KeyError, ValueError) as e:
        return fail(f"파라미터 오류: {e}")


# ----------------------------------------------------------------------
# 카메라 얼굴 트래킹
# ----------------------------------------------------------------------
@app.route("/api/camera/list", methods=["GET"])
def api_camera_list():
    try:
        max_index = int(request.args.get("max_index", 5))
        cams = list_available_cameras(max_index, already_open_index=tracker.current_index())
        return ok({"cameras": cams})
    except Exception as e:
        return fail(f"카메라 목록 조회 실패: {e}", 500)


@app.route("/api/camera/open", methods=["POST"])
def api_camera_open():
    body = request.get_json(force=True) or {}
    try:
        tracker.open(int(body.get("index", 0)))
        return ok({"opened": True})
    except Exception as e:
        return fail(f"카메라 열기 실패: {e}", 500)


@app.route("/api/camera/close", methods=["POST"])
def api_camera_close():
    tracker.close()
    return ok({"opened": False})


@app.route("/api/camera/stream")
def api_camera_stream():
    def gen():
        while tracker.is_open():
            jpeg = tracker.get_jpeg()
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/camera/state", methods=["GET"])
def api_camera_state():
    return ok(tracker.state())


@app.route("/api/camera/calibrate", methods=["POST"])
def api_camera_calibrate():
    try:
        ratio = tracker.calibrate_distance()
        return ok({"target_size_ratio": ratio})
    except Exception as e:
        return fail(str(e), 400)


@app.route("/api/camera/config", methods=["POST"])
def api_camera_config():
    body = request.get_json(force=True) or {}
    tracker.update_config(
        invert_pan=body.get("invert_pan"), invert_tilt=body.get("invert_tilt"), invert_z=body.get("invert_z"),
        max_step_deg=body.get("max_step_deg"), max_step_mm=body.get("max_step_mm"),
    )
    return ok(tracker.state())


@app.route("/api/camera/track/start", methods=["POST"])
def api_camera_track_start():
    try:
        tracker.start_tracking()
        return ok({"tracking": True})
    except Exception as e:
        return fail(str(e), 400)


@app.route("/api/camera/track/stop", methods=["POST"])
def api_camera_track_stop():
    tracker.stop_tracking()
    return ok({"tracking": False})


if __name__ == "__main__":
    # threaded=True: "정지" 버튼이 이동 중에도 즉시 처리되도록 동시 요청 허용
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)

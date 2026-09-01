"""
로봇 연결/명령 실행을 스레드-세이프하게 감싸는 모듈.

Flask는 요청마다(또는 스레드풀에서) 여러 스레드로 동시에 콜백을
실행할 수 있으므로, 실제로 로봇에 이동 명령을 보내는 부분은 반드시
락(lock)으로 한 번에 하나씩만 실행되게 막아야 합니다. 그렇지 않으면
"원 그리기"와 "수직 이동"을 동시에 눌렀을 때 두 명령이 뒤섞여 로봇이
예측 불가능하게 움직일 수 있습니다.

정지(StopMotion)/상태 조회 같은 명령은 락과 무관하게 언제든 바로
실행되어야 하므로(움직이는 도중에도 "정지" 버튼이 먹혀야 함) 별도로
처리합니다.
"""
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from fairino import Robot  # noqa: E402

import geometry  # noqa: E402


def cm_to_mm(v: float) -> float:
    return float(v) * 10.0


class RobotNotConnected(Exception):
    pass


class RobotManager:
    def __init__(self):
        self._robot = None
        self._ip = None
        self._motion_lock = threading.Lock()  # 이동 명령 전용 락 (한 번에 하나만)
        self._state_lock = threading.Lock()   # 연결 객체 자체를 보호하는 락
        self.default_vel = 20.0
        self.default_tool = 0
        self.default_user = 0

    # ------------------------------------------------------------------
    # 연결 / 기본 상태
    # ------------------------------------------------------------------
    def connect(self, ip: str):
        with self._state_lock:
            self._robot = Robot.RPC(ip)
            self._ip = ip
            error, sdk = self._robot.GetSDKVersion()
            return {"ip": ip, "sdk_version": sdk[0], "robot_version": sdk[1]}

    @property
    def robot(self):
        if self._robot is None:
            raise RobotNotConnected("로봇에 연결되어 있지 않습니다. 먼저 /api/connect를 호출하세요.")
        return self._robot

    def is_connected(self) -> bool:
        return self._robot is not None

    def status(self):
        if not self.is_connected():
            return {"connected": False}
        r = self.robot
        err, err_codes = r.GetRobotErrorCode()
        _, tcp_pose = r.GetActualTCPPose(1)
        _, joint_pos = r.GetActualJointPosDegree(1)
        _, is_drag = r.IsInDragTeach()
        _, prog_state = r.GetProgramState()
        return {
            "connected": True,
            "ip": self._ip,
            "error_code": err_codes,
            "tcp_pose_mm_deg": tcp_pose,
            "joint_pos_deg": joint_pos,
            "is_drag_teach": bool(is_drag),
            # 1=정지/프로그램없음, 2=실행중, 3=일시정지 (SDK 정의)
            "program_state": prog_state,
        }

    def current_tcp_pose(self):
        _, pose = self.robot.GetActualTCPPose(1)
        return pose

    # ------------------------------------------------------------------
    # 활성화 / 모드 / 에러
    # ------------------------------------------------------------------
    def enable(self, state: int):
        return self.robot.RobotEnable(int(state))

    def mode(self, state: int):
        return self.robot.Mode(int(state))

    def drag_teach(self, state: int):
        return self.robot.DragTeachSwitch(int(state))

    def reset_error(self):
        return self.robot.ResetAllError()

    # ------------------------------------------------------------------
    # 정지/일시정지/재개 - 락 없이 즉시 실행 (움직이는 도중에도 눌려야 함)
    # ------------------------------------------------------------------
    def stop(self):
        return self.robot.StopMotion()

    def pause(self):
        return self.robot.PauseMotion()

    def resume(self):
        return self.robot.ResumeMotion()

    # ------------------------------------------------------------------
    # 조그(수동 점동) - 사람이 버튼을 누르고 있는 동안만 움직임
    # ------------------------------------------------------------------
    def jog_start(self, ref: int, nb: int, direction: int, max_dis: float, vel: float = 30.0):
        return self.robot.StartJOG(ref=ref, nb=nb, dir=direction, max_dis=max_dis, vel=vel)

    def jog_stop(self, ref: int):
        return self.robot.StopJOG(ref=ref)

    def jog_stop_immediate(self):
        return self.robot.ImmStopJOG()

    # ------------------------------------------------------------------
    # 직선/회전 상대 이동 (수직/수평 이동, 제자리 회전)
    # ------------------------------------------------------------------
    def move_linear(self, axis: str, distance_cm: float, sign: str = "+", vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            target = geometry.linear_offset(current, axis, cm_to_mm(distance_cm), sign)
            return self.robot.MoveL(target, tool=self.default_tool, user=self.default_user, vel=vel)

    def move_rotate(self, axis: str, angle_deg: float, sign: str = "+", vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            target = geometry.rotate_offset(current, axis, angle_deg, sign)
            return self.robot.MoveL(target, tool=self.default_tool, user=self.default_user, vel=vel)

    def move_to_pose(self, desc_pos, vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            return self.robot.MoveL(desc_pos, tool=self.default_tool, user=self.default_user, vel=vel)

    # ------------------------------------------------------------------
    # 도형 그리기
    # ------------------------------------------------------------------
    def draw_circle(self, radius_cm: float, plane: str = "XY", direction: str = "ccw", vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            via, target = geometry.circle_points(current, cm_to_mm(radius_cm), plane, direction)
            return self.robot.Circle(
                desc_pos_p=via, tool_p=self.default_tool, user_p=self.default_user,
                desc_pos_t=target, tool_t=self.default_tool, user_t=self.default_user,
                vel_p=vel, vel_t=vel,
            )

    def draw_arc(self, radius_cm: float, sweep_deg: float, plane: str = "XY", direction: str = "ccw",
                 vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            via, target = geometry.arc_points(current, cm_to_mm(radius_cm), sweep_deg, plane, direction)
            return self.robot.MoveC(
                desc_pos_p=via, tool_p=self.default_tool, user_p=self.default_user,
                desc_pos_t=target, tool_t=self.default_tool, user_t=self.default_user,
                vel_p=vel, vel_t=vel,
            )

    def draw_polygon(self, sides: int, side_length_cm: float, plane: str = "XY", direction: str = "ccw",
                      vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            points = geometry.polygon_points(current, sides, cm_to_mm(side_length_cm), plane, direction)
            error = 0
            for p in points:
                error = self.robot.MoveL(p, tool=self.default_tool, user=self.default_user, vel=vel, blendR=5.0)
            return error

    def draw_ellipse(self, a_cm: float, b_cm: float, plane: str = "XY", direction: str = "ccw",
                      segments: int = 36, vel: float = None):
        vel = vel or self.default_vel
        with self._motion_lock:
            current = self.current_tcp_pose()
            points = geometry.ellipse_points(current, cm_to_mm(a_cm), cm_to_mm(b_cm), plane, direction, segments)
            error = 0
            for p in points:
                error = self.robot.MoveL(p, tool=self.default_tool, user=self.default_user, vel=vel, blendR=5.0)
            return error

    def draw_spiral(self, turns: float, tilt_deg: float, radius_init_cm: float, radius_add_cm: float,
                     axis_add_cm: float, direction: str = "cw", vel: float = None):
        """나선(스파이럴) 이동. NewSpiral을 그대로 감쌈 (05_move_arc_circle.py의 NewSpiral 참고)."""
        vel = vel or self.default_vel
        rot_direction = 0 if direction == "cw" else 1
        with self._motion_lock:
            current = self.current_tcp_pose()
            param = [turns, tilt_deg, cm_to_mm(radius_init_cm), cm_to_mm(radius_add_cm),
                     cm_to_mm(axis_add_cm), rot_direction, 0]
            return self.robot.NewSpiral(desc_pos=current, tool=self.default_tool, user=self.default_user,
                                         param=param, vel=vel)


manager = RobotManager()

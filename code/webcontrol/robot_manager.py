"""
로봇 연결/명령 실행을 스레드-세이프하게 감싸는 모듈.

핵심 문제: 파이썬 표준 `xmlrpc.client.ServerProxy`는 커넥션 하나를
재사용하는데, **그 커넥션에 동시에 두 개의 요청을 보내면
`CannotSendRequest`/`ResponseNotReady` 예외가 납니다** (스레드 세이프
하지 않음). Flask가 요청마다 다른 스레드에서 콜백을 실행하기 때문에,
버튼을 빠르게 연타하거나 이동 중에 다른 명령을 누르면 실제로 이 문제가
발생합니다 (조그 시작/정지를 빠르게 반복 클릭했을 때 발생한 버그).

그래서 연결마다 **두 개의 독립된 XML-RPC 커넥션**을 둡니다.
1. `self._robot` (기본 SDK 래퍼) - 상태조회/활성화/이동/조그 등 순서대로
   처리해도 되는 모든 명령. `_rpc_lock`으로 한 번에 하나씩만 실행.
2. `self._stop_proxy` (원본 XML-RPC 커넥션 하나 더) - 정지/일시정지/재개처럼
   "이동 중이어도 무조건 바로 먹혀야 하는" 명령 전용. `_rpc_lock`과는
   완전히 별개의 커넥션이라, 메인 커넥션이 긴 MoveL 응답을 기다리는
   중이어도 이 채널로는 즉시 정지 명령을 보낼 수 있습니다. 다만 이
   커넥션 자체도 동시에 두 요청이 들어오면 안 되므로 `_stop_lock`으로
   따로 보호합니다.
"""
import os
import sys
import threading
import xmlrpc.client

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
        self._stop_proxy = None
        self._ip = None
        self._rpc_lock = threading.Lock()     # 메인 커넥션(self._robot) 전용 - 한 번에 하나만
        self._stop_lock = threading.Lock()    # 정지 전용 커넥션 보호 - 메인 락과 완전히 독립
        self._state_lock = threading.Lock()   # 연결 자체(연결/해제)를 보호하는 락
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

            # 이 SDK 빌드는 XML-RPC(20003, 실제 이동 명령 채널)와 CNDE
            # (20005, 최신 실시간 상태 스트리밍 채널) 둘 다 성공해야
            # RPC.is_connect=True로 설정하고, 그렇지 않으면 모든 SDK 함수가
            # 실제로 실행되지 않고 그냥 -4를 반환하도록 되어 있다
            # (xmlrpc_timeout 데코레이터가 RPC.is_connect를 먼저 체크함).
            # 컨트롤러가 CNDE 포트를 막아두거나 지원하지 않으면 XML-RPC만
            # 살아있어도 전체가 잠겨버리므로, 이동 명령에 실제로 필요한
            # XML-RPC 채널만 직접 우회 테스트해서 is_connect를 복구한다.
            try:
                self._robot.robot.GetControllerIP()  # 데코레이터를 안 거치는 원본 XML-RPC 호출
            except Exception as e:
                self._robot = None
                self._ip = None
                raise ConnectionError(f"XML-RPC(20003) 연결 자체가 안 됩니다: {e}")
            Robot.RPC.is_connect = True

            # 정지/일시정지/재개 전용 독립 커넥션 (메인 커넥션과 별개의 소켓)
            self._stop_proxy = xmlrpc.client.ServerProxy(f"http://{ip}:20003")

            with self._rpc_lock:
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
        with self._rpc_lock:
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

    def _current_tcp_pose_locked(self):
        """이미 _rpc_lock을 잡고 있는 상태에서 호출 (재진입 아님, 락 재획득 안 함)."""
        _, pose = self.robot.GetActualTCPPose(1)
        return pose

    # ------------------------------------------------------------------
    # 활성화 / 모드 / 에러 - 메인 커넥션, 락으로 직렬화
    # ------------------------------------------------------------------
    def enable(self, state: int):
        with self._rpc_lock:
            return self.robot.RobotEnable(int(state))

    def mode(self, state: int):
        with self._rpc_lock:
            return self.robot.Mode(int(state))

    def drag_teach(self, state: int):
        with self._rpc_lock:
            return self.robot.DragTeachSwitch(int(state))

    def reset_error(self):
        with self._rpc_lock:
            return self.robot.ResetAllError()

    # ------------------------------------------------------------------
    # 정지/일시정지/재개 - 별도 커넥션, 메인 락과 무관하게 항상 즉시 실행
    # ------------------------------------------------------------------
    def stop(self):
        if self._stop_proxy is None:
            raise RobotNotConnected("로봇에 연결되어 있지 않습니다. 먼저 /api/connect를 호출하세요.")
        with self._stop_lock:
            return self._stop_proxy.StopMotion()

    def pause(self):
        if self._stop_proxy is None:
            raise RobotNotConnected("로봇에 연결되어 있지 않습니다. 먼저 /api/connect를 호출하세요.")
        with self._stop_lock:
            return self._stop_proxy.PauseMotion()

    def resume(self):
        if self._stop_proxy is None:
            raise RobotNotConnected("로봇에 연결되어 있지 않습니다. 먼저 /api/connect를 호출하세요.")
        with self._stop_lock:
            return self._stop_proxy.ResumeMotion()

    def jog_stop_immediate(self):
        """조그 즉시정지도 '정지' 계열이므로 메인 락을 기다리지 않는 별도 커넥션 사용."""
        if self._stop_proxy is None:
            raise RobotNotConnected("로봇에 연결되어 있지 않습니다. 먼저 /api/connect를 호출하세요.")
        with self._stop_lock:
            return self._stop_proxy.ImmStopJOG()

    # ------------------------------------------------------------------
    # 조그(수동 점동) - 시작/일반정지는 메인 커넥션, 락으로 직렬화
    # ------------------------------------------------------------------
    def jog_start(self, ref: int, nb: int, direction: int, max_dis: float, vel: float = 30.0):
        with self._rpc_lock:
            return self.robot.StartJOG(ref=ref, nb=nb, dir=direction, max_dis=max_dis, vel=vel)

    def jog_stop(self, ref: int):
        with self._rpc_lock:
            return self.robot.StopJOG(ref=ref)

    # ------------------------------------------------------------------
    # 직선/회전 상대 이동 (수직/수평 이동, 제자리 회전)
    # ------------------------------------------------------------------
    def move_linear(self, axis: str, distance_cm: float, sign: str = "+", vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            target = geometry.linear_offset(current, axis, cm_to_mm(distance_cm), sign)
            return self.robot.MoveL(target, tool=self.default_tool, user=self.default_user, vel=vel)

    def move_rotate(self, axis: str, angle_deg: float, sign: str = "+", vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            target = geometry.rotate_offset(current, axis, angle_deg, sign)
            return self.robot.MoveL(target, tool=self.default_tool, user=self.default_user, vel=vel)

    def move_to_pose(self, desc_pos, vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            return self.robot.MoveL(desc_pos, tool=self.default_tool, user=self.default_user, vel=vel)

    # ------------------------------------------------------------------
    # 도형 그리기
    # ------------------------------------------------------------------
    def draw_circle(self, radius_cm: float, plane: str = "XY", direction: str = "ccw", vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            via, target = geometry.circle_points(current, cm_to_mm(radius_cm), plane, direction)
            return self.robot.Circle(
                desc_pos_p=via, tool_p=self.default_tool, user_p=self.default_user,
                desc_pos_t=target, tool_t=self.default_tool, user_t=self.default_user,
                vel_p=vel, vel_t=vel,
            )

    def draw_arc(self, radius_cm: float, sweep_deg: float, plane: str = "XY", direction: str = "ccw",
                 vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            via, target = geometry.arc_points(current, cm_to_mm(radius_cm), sweep_deg, plane, direction)
            return self.robot.MoveC(
                desc_pos_p=via, tool_p=self.default_tool, user_p=self.default_user,
                desc_pos_t=target, tool_t=self.default_tool, user_t=self.default_user,
                vel_p=vel, vel_t=vel,
            )

    def draw_polygon(self, sides: int, side_length_cm: float, plane: str = "XY", direction: str = "ccw",
                      vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            points = geometry.polygon_points(current, sides, cm_to_mm(side_length_cm), plane, direction)
            error = 0
            for p in points:
                error = self.robot.MoveL(p, tool=self.default_tool, user=self.default_user, vel=vel, blendR=5.0)
            return error

    def draw_ellipse(self, a_cm: float, b_cm: float, plane: str = "XY", direction: str = "ccw",
                      segments: int = 36, vel: float = None):
        vel = vel or self.default_vel
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            points = geometry.ellipse_points(current, cm_to_mm(a_cm), cm_to_mm(b_cm), plane, direction, segments)
            error = 0
            for p in points:
                error = self.robot.MoveL(p, tool=self.default_tool, user=self.default_user, vel=vel, blendR=5.0)
            return error

    def draw_spiral(self, turns: float, tilt_deg: float, radius_init_cm: float, radius_add_cm: float,
                     axis_add_cm: float, direction: str = "cw", vel: float = None):
        """나선(스파이럴) 이동. NewSpiral을 그대로 감쌈 (02_movement/03_move_arc_circle.py의 NewSpiral 참고)."""
        vel = vel or self.default_vel
        rot_direction = 0 if direction == "cw" else 1
        with self._rpc_lock:
            current = self._current_tcp_pose_locked()
            param = [turns, tilt_deg, cm_to_mm(radius_init_cm), cm_to_mm(radius_add_cm),
                     cm_to_mm(axis_add_cm), rot_direction, 0]
            return self.robot.NewSpiral(desc_pos=current, tool=self.default_tool, user=self.default_user,
                                         param=param, vel=vel)


manager = RobotManager()

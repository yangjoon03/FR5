"""
카테고리: 상태/로그 모니터링 (Status & Logging) - 1. 실시간 상태 구독
------------------------------------------------------------
로봇 상태를 확인하는 방법은 크게 두 가지입니다.

1) `robot.robot_state_pkg.필드명` : SDK가 백그라운드에서 항상 최신값으로
   유지해주는 "현재 상태 스냅샷" 객체. 매번 함수를 호출할 필요 없이
   속성처럼 바로 읽으면 됩니다. (관절각, 로봇모드, 에러코드, 온도 등)
2) `AddRobotRealtimeState([...])` + `GetRobotRealTimeState()` : 원하는
   상태 필드만 선택해서 구독하고, 그 값들을 담은 패키지를 받아오는 방식.
   프로그램 실행상태처럼 기본 스냅샷에 없는 특수 필드를 볼 때 사용.

원본 출처: example/TestRobotState.py, example/RobotStatePeriod.py,
           example/programRunState_test.py, example/getExaxisCoordID.py,
           example/TestLogCommand.py
"""
from fairino import Robot
from fairino.Robot import RobotState, AddRobotRealtimeState
import time


def print_robot_state_snapshot(robot):
    """
    robot.robot_state_pkg 에 들어있는 대표적인 상태값들을 출력.
    - program_state: 프로그램 실행 상태
    - robot_state: 로봇 전체 상태(정지/동작/에러 등)
    - main_code / sub_code: 에러 주/부 코드
    - robot_mode: 0=자동, 1=수동 등
    - jt_cur_pos[0..5]: 현재 관절 각도
    - tl_cur_pos[0..5]: 현재 TCP(공구 끝) 위치/자세
    - jointDriverTemperature[0..5]: 각 관절 구동기 온도 [°C]
    """
    p = robot.robot_state_pkg
    print("program_state:", p.program_state, "robot_state:", p.robot_state)
    print("main_code:", p.main_code, "sub_code:", p.sub_code, "robot_mode:", p.robot_mode)
    print("관절각:", list(p.jt_cur_pos[:6]))
    print("TCP위치:", list(p.tl_cur_pos[:6]))
    print("관절온도:", list(p.jointDriverTemperature[:6]))


def set_realtime_state_sample_period(robot, period_ms=200):
    """
    robot_state_pkg가 갱신되는 주기를 설정/조회.
    - SetRobotRealtimeStateSamplePeriod(ms): 갱신 주기 [ms]
    - GetRobotRealtimeStateSamplePeriod(): 현재 주기 조회
    주기를 짧게 하면 더 실시간에 가깝지만 통신 부하가 늘어납니다.
    """
    error = robot.SetRobotRealtimeStateSamplePeriod(period_ms)
    print("SetRobotRealtimeStateSamplePeriod 반환값:", error)
    print("GetRobotRealtimeStateSamplePeriod:", robot.GetRobotRealtimeStateSamplePeriod())


def subscribe_program_run_state():
    """
    기본 스냅샷(robot_state_pkg)에 없는 특수 필드(프로그램 실행 상태)를
    구독하는 예시. `AddRobotRealtimeState`는 로봇 연결(RPC) 전에 먼저
    호출해서 "어떤 필드를 받을지" 등록해야 합니다.
    """
    rtn = AddRobotRealtimeState([RobotState.ProgramRunState])
    print("AddRobotRealtimeState 반환값:", rtn)

    robot = Robot.RPC("192.168.58.2")
    time.sleep(0.5)

    for _ in range(20):
        rtn, pkg = robot.GetRobotRealTimeState()
        print("programRunState:", pkg.programRunState)
        time.sleep(0.1)
    robot.CloseRPC()


def subscribe_exaxis_coord_id():
    """확장축 좌표계 ID/좌표값을 실시간 구독하는 예시. subscribe_program_run_state와 같은 패턴."""
    rtn = AddRobotRealtimeState([RobotState.ExaxisCoordID])
    print("AddRobotRealtimeState 반환값:", rtn)

    robot = Robot.RPC("192.168.58.2")
    time.sleep(0.5)
    for _ in range(20):
        rtn, pkg = robot.GetRobotRealTimeState()
        print("exaxisCoordID:", pkg.exaxisCoordID, "exAxisCoord:", pkg.exAxisCoord)
        time.sleep(0.1)
    robot.CloseRPC()


def robot_logging(robot):
    """
    SDK 자체 로그 파일을 설정하고, 주기적으로 상태를 조회해 로그에 남기는 예시.
    - LoggerInit(file_path): 로그 파일 경로
    - SetLoggerLevel(lvl): 로그 레벨 (숫자가 클수록 더 상세)
    """
    robot.LoggerInit(file_path="./fairino.log")
    robot.SetLoggerLevel(lvl=3)
    for _ in range(20):
        robot.GetRobotInstallAngle()
        robot.GetActualJointPosDegree()
        robot.GetActualJointPosRadian()
        robot.GetActualJointSpeedsDegree()
        robot.GetTargetTCPCompositeSpeed()
        robot.GetActualTCPCompositeSpeed()
        time.sleep(0.1)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    print_robot_state_snapshot(robot)
    set_realtime_state_sample_period(robot)
    # robot_logging(robot)

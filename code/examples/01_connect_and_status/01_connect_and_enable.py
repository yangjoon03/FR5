"""
카테고리: 연결 / 활성화(Enable) / 상태 확인
------------------------------------------------
FR5 로봇과 통신을 시작하고, 상태를 확인하고, 로봇을 상(上)사용 가능 상태로
만드는 데 필요한 기본 함수들을 모아 정리한 예제입니다.

실제 로봇 동작(Move 계열 함수)을 호출하기 전에는 반드시
1) RPC 연결 -> 2) RobotEnable(1) 로 로봇을 활성화 순서를 거쳐야 합니다.

원본 출처: example/TestBasicCommand.py (중국어 주석을 한국어로 번역/정리)
"""
from fairino import Robot
import time


# ------------------------------------------------------------------
# 1. 로봇 컨트롤러와 연결하기
# ------------------------------------------------------------------
def connect(ip: str = "192.168.58.2"):
    """
    로봇 컨트롤러에 RPC(원격 프로시저 호출)로 접속합니다.
    - ip: 로봇 컨트롤러의 IP 주소 (Teach Pendant 설정 화면에서 확인 가능)
    - 반환값: 성공 시 Robot.RPC 객체(robot)를 반환. 실패해도 예외 없이
      객체는 생성되며, 이후 명령 호출 시 통신 실패 에러코드가 반환됨.
    """
    robot = Robot.RPC(ip)
    return robot


def check_connection(robot):
    """robot.is_conect 로 현재 연결 상태(True/False)를 확인합니다."""
    if robot.is_conect:
        print("연결 성공")
    else:
        print("연결 실패")


# ------------------------------------------------------------------
# 2. 버전 / 컨트롤러 정보 조회
# ------------------------------------------------------------------
def get_sdk_version(robot):
    """
    SDK 및 로봇 컨트롤러 펌웨어 버전을 조회합니다.
    반환: (error, [SDK 버전, 로봇 버전])
    """
    error, sdk = robot.GetSDKVersion()
    print("SDK 버전: %s, 로봇 버전: %s" % (sdk[0], sdk[1]))
    return error, sdk


def get_controller_ip(robot):
    """현재 접속 중인 컨트롤러의 IP 주소를 조회합니다."""
    error, ip = robot.GetControllerIP()
    print("컨트롤러 IP: %s" % ip)
    return error, ip


# ------------------------------------------------------------------
# 3. 로봇 활성화(Enable) / 비활성화
# ------------------------------------------------------------------
def robot_enable(robot, state: int):
    """
    로봇을 상(上)사용(활성화) 또는 하(下)사용(비활성화) 상태로 전환합니다.
    - state: 0 = 비활성화(disable, 모터 토크 off), 1 = 활성화(enable, 모터 토크 on)
    - 반환: 에러코드 (0 = 성공)

    주의: 로봇이 서보 On(활성화) 되어야만 MoveJ/MoveL 등 모든 Move 계열
          함수를 실행할 수 있습니다. 활성화 실패 시 안전 정지(E-Stop),
          비상정지 버튼, 안전문 등의 상태를 먼저 점검하세요.
    """
    error = robot.RobotEnable(state)
    print("RobotEnable(%d) 반환값: %s" % (state, error))
    return error


# ------------------------------------------------------------------
# 4. 동작 모드 전환 (자동 / 수동)
# ------------------------------------------------------------------
def set_mode(robot, state: int):
    """
    로봇 동작 모드를 전환합니다.
    - state: 0 = 자동 모드, 1 = 수동 모드
    """
    error = robot.Mode(state)
    print("Mode(%d) 반환값: %s" % (state, error))
    return error


# ------------------------------------------------------------------
# 5. 드래그 티칭(Drag Teach) 모드 - 사람이 팔을 손으로 잡고 움직이는 모드
# ------------------------------------------------------------------
def drag_teach_switch(robot, state: int):
    """
    드래그 티칭 모드 진입/해제.
    - state: 0 = 드래그 티칭 해제, 1 = 드래그 티칭 진입
    드래그 티칭 중에는 사람이 로봇 팔을 손으로 잡고 자유롭게 움직일 수 있습니다.
    """
    error = robot.DragTeachSwitch(state)
    print("DragTeachSwitch(%d) 반환값: %s" % (state, error))
    return error


def is_in_drag_teach(robot):
    """현재 드래그 티칭 모드인지 조회합니다. 반환: (error, 0=아님/1=맞음)"""
    error, is_drag = robot.IsInDragTeach()
    if is_drag == 0:
        print("현재 드래그 티칭 모드 아님")
    elif is_drag == 1:
        print("현재 드래그 티칭 모드")
    return error, is_drag


# ------------------------------------------------------------------
# 6. 에러 상태 조회 / 초기화
# ------------------------------------------------------------------
def get_robot_error_code(robot):
    """
    현재 로봇의 에러 코드를 조회합니다.
    반환: (error, [maincode, subcode]) - maincode: 주 에러코드, subcode: 세부 에러코드
    """
    error, codes = robot.GetRobotErrorCode()
    print("로봇 에러코드: %s" % (codes,))
    return error, codes


def reset_all_error(robot):
    """
    복구 가능한 에러(리셋 가능한 알람)를 초기화합니다.
    비상정지, 안전 인터록 등 물리적으로 원인이 남아있는 에러는 해제되지 않습니다.
    """
    error = robot.ResetAllError()
    print("ResetAllError 반환값: %s" % error)
    return error


if __name__ == "__main__":
    # 사용 예시 - 실제 로봇 IP로 바꿔서 사용하세요.
    robot = connect("192.168.58.2")
    check_connection(robot)
    get_sdk_version(robot)
    get_controller_ip(robot)

    # 에러가 있다면 먼저 초기화
    reset_all_error(robot)
    get_robot_error_code(robot)

    # 로봇 활성화 (Move 계열 함수를 쓰기 전 필수)
    robot_enable(robot, 1)

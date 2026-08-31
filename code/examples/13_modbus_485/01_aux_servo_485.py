"""
카테고리: Modbus/RS-485 - RS-485 확장(보조) 서보축 제어
------------------------------------------------------------
로봇 6축 + 컨베이어/견인차(외부축, UDP 통신)와는 또 다른 경로로, RS-485
시리얼 통신으로 연결된 "보조 서보 드라이버 축"을 직접 제어하는 기능입니다.
(예: 회전 지그, 턴테이블처럼 단순한 1축 장비를 RS-485로 저렴하게 연결)

동작 흐름: 파라미터 설정 -> 제어모드 선택(속도/위치) -> 활성화 -> 원점복귀
          -> 목표 속도/위치 지령 -> (필요시) 비상정지 가속도 설정

원본 출처: example/Test485Command.py, example/485setacc.py, example/485stop.py
(세 파일 모두 사실상 같은 기능을 반복 테스트하는 내용이라 하나로 통합함)
"""
from fairino import Robot
import time


def aux_servo_configure(robot, servo_id=1):
    """
    보조 서보 드라이버 사양을 등록합니다.
    - AuxServoSetParam(servoId, servoCompany, servoModel, servoSoftVersion,
      servoResolution, axisMechTransRatio): 제조사/모델/펌웨어버전/
      엔코더 분해능/기구 감속비
    """
    error = robot.AuxServoSetParam(servoId=servo_id, servoCompany=1, servoModel=1,
                                    servoSoftVersion=1, servoResolution=131072,
                                    axisMechTransRatio=13.45)
    print("AuxServoSetParam 반환값:", error)
    error, company, model, sw, resolution, ratio = robot.AuxServoGetParam(servoId=servo_id)
    print("제조사:", company, "모델:", model, "분해능:", resolution, "감속비:", ratio)


def aux_servo_enable_and_home(robot, servo_id=1, mode=1):
    """
    보조축 활성화 및 원점 복귀.
    - AuxServoSetControlMode(servoId, mode): 0=위치제어 모드, 1=속도제어 모드
    - AuxServoEnable(servoId, status): 0=비활성화, 1=활성화
    - AuxServoHoming(servoId, mode, searchVel, latchVel, acc): 원점 탐색
      속도/래치속도/가속도를 지정해 원점 센서까지 이동
    """
    robot.AuxServoSetControlMode(servoId=servo_id, mode=mode)
    time.sleep(1)
    robot.AuxServoEnable(servoId=servo_id, status=0)
    time.sleep(1)
    robot.AuxServoEnable(servoId=servo_id, status=1)
    time.sleep(1)
    error = robot.AuxServoHoming(servoId=servo_id, mode=1, searchVel=20, latchVel=20, acc=100)
    print("AuxServoHoming 반환값:", error)


def aux_servo_speed_mode_demo(robot, servo_id=1):
    """속도 제어 모드: 목표 속도를 지령 (양수/음수로 방향, 0으로 정지)."""
    aux_servo_enable_and_home(robot, servo_id, mode=1)
    robot.AuxServoSetTargetSpeed(servoId=servo_id, speed=30, acc=100)
    time.sleep(2)
    robot.AuxServoSetTargetSpeed(servoId=servo_id, speed=-50, acc=100)
    time.sleep(2)
    error = robot.AuxServoSetTargetSpeed(servoId=servo_id, speed=0, acc=100)
    print("AuxServoSetTargetSpeed(정지) 반환값:", error)


def aux_servo_position_mode_demo(robot, servo_id=1):
    """위치 제어 모드: 목표 위치로 이동."""
    aux_servo_enable_and_home(robot, servo_id, mode=0)
    error = robot.AuxServoSetTargetPos(servoId=servo_id, pos=200, speed=30, acc=100)
    print("AuxServoSetTargetPos 반환값:", error)
    time.sleep(2)
    robot.AuxServoSetTargetPos(servoId=servo_id, pos=-300, speed=30, acc=100)


def aux_servo_status_monitor(robot, servo_id=1, cycles=10):
    """
    보조축 상태 실시간 조회.
    - robot_state_pkg.auxState.servoPos / servoVel: 현재 위치/속도
    - robot_state_pkg.auxState.servoState: 상태 비트마스크
      (bit7 = 1이면 비상정지 상태)
    - AuxServosetStatusID(servoId): 상태 조회 대상 축 번호 지정
    """
    robot.AuxServosetStatusID(servoId=servo_id)
    for _ in range(cycles):
        p = robot.robot_state_pkg.auxState
        estop = (p.servoState >> 7) & 0x01
        print(f"위치: {p.servoPos}, 속도: {p.servoVel}, 비상정지: {estop}")
        time.sleep(0.1)


def aux_servo_emergency_stop_accel(robot, servo_id=1, acc=4000, dec=5000):
    """
    비상정지 시 사용할 가속도(감속도)를 설정/조회.
    값이 클수록 더 급격하게(빠르게) 정지합니다.
    """
    error = robot.AuxServoSetEmergencyStopAcc(acc, dec)
    print("AuxServoSetEmergencyStopAcc 반환값:", error)
    print("현재 비상정지 가속도:", robot.AuxServoGetEmergencyStopAcc())


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    aux_servo_configure(robot)
    aux_servo_speed_mode_demo(robot)
    # aux_servo_position_mode_demo(robot)
    # aux_servo_status_monitor(robot)

"""
카테고리: 안전(Safety) - 안전 입출력(Safety I/O) 및 안전 파라미터 점검
------------------------------------------------------------
비상정지 입력, 감속 모드 트리거 등 "안전 관련 신호"를 특정 DI/DO
채널에 연결하는 설정과, 현재 안전 설정값들이 임의로 변경되지 않았는지
체크섬으로 검증하고, 안전 설정 변경 시 비밀번호로 잠그는 기능입니다.

원본 출처: example/TestSafetyIOConfig.py, example/SafetyParams_Test.py
"""
from fairino import Robot
import time


def set_safety_di_config(robot):
    """
    안전 이중 채널(CI) DI에 특수 안전 기능을 매핑합니다.
    - ID: 채널 번호 [0~3]
    - config: 0=미설정, 201=외부 비상정지 입력1, 202=외부 비상정지 입력2,
      203=1단계 감속모드, 204=2단계 감속모드, 205=3단계 감속모드
    """
    for ch, cfg in [(0, 201), (1, 202), (2, 203), (3, 204)]:
        rtn = robot.SetSafetyDIConfig(ch, cfg)
        print(f"SetSafetyDIConfig({ch}, {cfg}) 반환값: {rtn}")

    rtn, cfg = robot.GetDIConfig()
    print("GetDIConfig:", rtn, cfg)


def clear_safety_di_config(robot):
    """안전 DI 매핑을 모두 해제(0)합니다."""
    for ch in range(4):
        robot.SetSafetyDIConfig(ch, 0)


def set_safety_do_config(robot):
    """안전 DO 채널 매핑. config 값의 의미는 SetSafetyDIConfig와 동일 체계."""
    for ch, cfg in [(0, 204), (1, 205), (2, 206), (3, 207)]:
        rtn = robot.SetSafetyDOConfig(ch, cfg)
        print(f"SetSafetyDOConfig({ch}, {cfg}) 반환값: {rtn}")

    rtn, cfg = robot.GetDOConfig()
    print("GetDOConfig:", rtn, cfg)


def check_safety_params_integrity(robot, password="12345678"):
    """
    안전 파라미터가 마지막 저장 이후 변경되지 않았는지 체크섬으로 검증하고,
    변경하려면 관리자 비밀번호 인증을 거치도록 하는 예시.

    - GetSafetyParamsCheckSum(): (error, status, hex_code) 반환.
      status로 값 위/변조 여부를 판단
    - SafetyOPPasswordCheck(level, password): 안전설정 변경 권한 비밀번호 확인
    """
    error, status, hex_code = robot.GetSafetyParamsCheckSum()
    print(f"체크섬(변경 전): status={status}, hex_code={hex_code}")

    error = robot.SafetyOPPasswordCheck(0, password)
    print("비밀번호 확인 반환값:", error)

    if error == 0:
        robot.SetAnticollision(mode=0, level=[2.0] * 6, config=1)
        robot.SetCollisionStrategy(strategy=0, safeTime=1000, safeDistance=150, safetyMargin=[10] * 6)

    time.sleep(1)
    error, status, hex_code = robot.GetSafetyParamsCheckSum()
    print(f"체크섬(변경 후): status={status}, hex_code={hex_code}")


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")

    set_safety_di_config(robot)
    set_safety_do_config(robot)
    # check_safety_params_integrity(robot)

    time.sleep(1)
    robot.CloseRPC()

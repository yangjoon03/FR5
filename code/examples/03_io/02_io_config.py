"""
카테고리: I/O 제어 - IO 채널 용도/전기적 특성 설정 (IO Config)
------------------------------------------------------------
DI/DO 각 채널을 "어떤 용도(안전정지, 일시정지 등 특수기능 매핑)"로
쓸지, 그리고 "전기적으로 어떤 레벨을 켜짐(ON)으로 볼지(로우/하이 액티브)"를
설정하는 함수 모음. 일반적으로 로봇 셋업 단계에서 한 번만 설정합니다.

원본 출처: example/TestIOConfig.py
"""
from fairino import Robot
import time


def configure_io_function_mapping(robot):
    """
    DI/DO 채널에 특수 기능을 매핑합니다.
    - SetDIConfig/SetDOConfig: DI/DO 8채널 각각에 기능 번호를 할당
      (예: 특정 채널을 "일시정지 입력"으로 지정하는 식)
    - SetToolDIConfig: 툴 DI 2채널의 기능 매핑
    - Get*Config: 현재 매핑값 조회
    """
    rtn = robot.SetDIConfig([1, 2, 3, 4, 5, 6, 7, 8])
    print("SetDIConfig:", rtn)
    rtn, cfg = robot.GetDIConfig()
    print("GetDIConfig:", rtn, cfg)

    rtn = robot.SetDOConfig([9, 10, 11, 12, 13, 14, 15, 16])
    print("SetDOConfig:", rtn)
    rtn, cfg = robot.GetDOConfig()
    print("GetDOConfig:", rtn, cfg)

    rtn = robot.SetToolDIConfig([17, 18])
    print("SetToolDIConfig:", rtn)
    rtn, cfg = robot.GetToolDIConfig()
    print("GetToolDIConfig:", rtn, cfg)


def configure_io_active_level(robot):
    """
    DI/DO 각 채널의 전기적 On 기준(액티브 레벨)을 설정합니다.
    값 배열의 각 원소: 0=로우 액티브(신호가 0V일 때 ON으로 인식),
                       1=하이 액티브(신호가 24V 등일 때 ON으로 인식)

    - SetDIConfigLevel / SetDOConfigLevel: 일반 DI/DO 8채널
    - SetToolDIConfigLevel: 툴 DI 2채널
    - SetStandardDILevel / SetStandardDOLevel: 표준 DI/DO 8채널
      (컨트롤러 종류에 따라 '일반'과 '표준' IO 뱅크가 별도로 존재할 수 있음)
    """
    robot.SetDIConfigLevel([1, 1, 1, 1, 0, 0, 0, 0])
    print("GetDIConfigLevel:", robot.GetDIConfigLevel())

    robot.SetDOConfigLevel([0, 0, 0, 0, 1, 1, 1, 1])
    print("GetDOConfigLevel:", robot.GetDOConfigLevel())

    robot.SetToolDIConfigLevel([1, 0])
    print("GetToolDIConfigLevel:", robot.GetToolDIConfigLevel())

    robot.SetStandardDILevel([1, 1, 1, 1, 0, 0, 0, 0])
    print("GetStandardDILevel:", robot.GetStandardDILevel())

    robot.SetStandardDOLevel([0, 0, 0, 0, 1, 1, 1, 1])
    print("GetStandardDOLevel:", robot.GetStandardDOLevel())


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")

    configure_io_function_mapping(robot)
    configure_io_active_level(robot)

    time.sleep(2)
    robot.CloseRPC()

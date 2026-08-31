"""
카테고리: I/O 제어 - 디지털/아날로그 입출력
------------------------------------------------------------
로봇 컨트롤러 박스(제어함) 및 로봇 손목(툴)에 달린 디지털/아날로그
입출력 신호를 켜고 끄고, 읽고, 특정 조건이 될 때까지 기다리는 함수 모음.

원본 출처: example/TestIOCommand.py (중국어 주석을 한국어로 번역/정리)
"""
from fairino import Robot
import time


# ------------------------------------------------------------------
# 출력(Output) 신호 켜기/끄기
# ------------------------------------------------------------------
def set_do(robot, io_id=0):
    """
    제어함(컨트롤 박스) 디지털 출력(DO) 설정.
    - id: DO 채널 번호, 범위 [0~15]
    - status: 0=끄기, 1=켜기
    - smooth: 0=즉시 전환, 1=부드럽게 전환, 기본 0
    - block: 0=블로킹(응답 대기), 1=논블로킹, 기본 0
    """
    error = robot.SetDO(id=io_id, status=1)
    time.sleep(2)
    error = robot.SetDO(id=io_id, status=0)
    print("SetDO 반환값:", error)
    return error


def set_tool_do(robot, io_id=0):
    """
    툴(손목) 디지털 출력 설정. 파라미터 의미는 SetDO와 동일.
    - id 범위만 다름: [0~1] (툴 쪽은 채널이 2개뿐)
    """
    error = robot.SetToolDO(id=io_id, status=1)
    time.sleep(3)
    error = robot.SetToolDO(id=io_id, status=0)
    print("SetToolDO 반환값:", error)
    return error


def set_ao(robot, io_id=0):
    """제어함 아날로그 출력(AO) 설정. value: 출력 전압/전류 비율 등 실수값."""
    error = robot.SetAO(id=io_id, value=0.6)
    time.sleep(3)
    error = robot.SetAO(id=io_id, value=0.2)
    print("SetAO 반환값:", error)
    return error


def set_tool_ao(robot, io_id=0):
    """툴 아날로그 출력 설정."""
    error = robot.SetToolAO(id=io_id, value=0.6)
    time.sleep(3)
    error = robot.SetToolAO(id=io_id, value=0.2)
    print("SetToolAO 반환값:", error)
    return error


# ------------------------------------------------------------------
# 입력(Input) 신호가 원하는 상태가 될 때까지 대기
# ------------------------------------------------------------------
def wait_di(robot, io_id=1):
    """
    제어함 디지털 입력(DI)이 특정 상태가 될 때까지 대기.
    - status: 기다릴 상태 (0/1)
    - maxtime: 최대 대기시간 [ms]
    - opt: 타임아웃 시 정책. 0=프로그램 정지+타임아웃 알림,
      1=타임아웃 무시하고 계속 진행, 2=계속 대기
    """
    error = robot.WaitDI(id=io_id, status=1, maxtime=5000, opt=0)
    print("WaitDI 반환값:", error)
    return error


def wait_multi_di(robot):
    """
    여러 개의 DI를 동시에 검사해서 대기.
    - mode: 0=다중 AND(모두 만족해야 통과), 1=다중 OR(하나만 만족해도 통과)
    - id: 비트마스크. bit0~7=DI0~DI7, bit8~15=CI0~CI7
    - status: 각 비트의 기대 상태(비트마스크)
    """
    error = robot.WaitMultiDI(mode=0, id=6, status=6, maxtime=5000, opt=0)
    print("WaitMultiDI 반환값:", error)
    return error


def wait_tool_di(robot, io_id=0):
    """툴 디지털 입력이 특정 상태가 될 때까지 대기."""
    error = robot.WaitToolDI(id=io_id, status=1, maxtime=10000, opt=0)
    print("WaitToolDI 반환값:", error)
    return error


def wait_ai(robot, io_id=0):
    """
    제어함 아날로그 입력(AI)이 조건을 만족할 때까지 대기.
    - sign: 비교 방향 (0=이상, 1=이하 등 컨트롤러 정의에 따름)
    - value: 비교 기준값
    """
    error = robot.WaitAI(id=io_id, sign=0, value=8, maxtime=5000, opt=2)
    print("WaitAI 반환값:", error)
    return error


def wait_tool_ai(robot, io_id=0):
    """툴 아날로그 입력 대기. 파라미터 의미는 wait_ai와 동일."""
    error = robot.WaitToolAI(id=io_id, sign=0, value=8, maxtime=5000, opt=2)
    print("WaitToolAI 반환값:", error)
    return error


# ------------------------------------------------------------------
# 이동 중 아날로그 출력을 연동시키는 기능 (예: 접착제 도포량을 속도에 비례)
# ------------------------------------------------------------------
def move_ao_start_stop(robot):
    """
    로봇이 움직이는 동안 제어함 AO 값을 자동으로 계속 갱신하는 기능 시작/정지.
    도포/디스펜싱처럼 "이동 속도에 맞춰 유량을 조절"하는 용도로 사용.
    - MoveAOStart(id, mode, s, e): AO 자동 출력 시작
    - MoveAOStop(): AO 자동 출력 정지
    """
    error = robot.MoveAOStart(0, 100, 98, 1)
    print("MoveAOStart 반환값:", error)
    time.sleep(3)
    error, joint_pos = robot.GetActualJointPosDegree()
    joint_pos[0] -= 10
    robot.MoveJ(joint_pos, tool=0, user=0)
    time.sleep(3)
    error = robot.MoveAOStop()
    print("MoveAOStop 반환값:", error)
    return error


def move_tool_ao_start_stop(robot):
    """MoveAOStart/Stop의 툴 AO 버전."""
    error = robot.MoveToolAOStart(0, 100, 98, 1)
    print("MoveToolAOStart 반환값:", error)
    time.sleep(3)
    error, joint_pos = robot.GetActualJointPosDegree()
    joint_pos[0] += 10
    robot.MoveJ(joint_pos, tool=0, user=0)
    time.sleep(3)
    error = robot.MoveToolAOStop()
    print("MoveToolAOStop 반환값:", error)
    return error


# ------------------------------------------------------------------
# 확장(외부) IO 모듈 제어
# ------------------------------------------------------------------
def set_aux_io(robot):
    """
    확장 IO 모듈(외부 DO/AO 카드 등)의 채널을 켜고 아날로그 값을 설정.
    - SetAuxDO(DONum, bOpen, smooth, block): 확장 DO 켜기/끄기
    - SetAuxAO(AONum, value, block): 확장 AO 값 설정
    - SetOutputResetExtDO/AO(resetFlag=1): 확장 DO/AO 전체 초기화(리셋)
    """
    robot.SetAuxDO(DONum=0, bOpen=True, smooth=False, block=False)
    robot.SetAuxDO(DONum=1, bOpen=True, smooth=False, block=False)
    robot.SetAuxAO(AONum=0, value=1234, block=False)
    time.sleep(2)
    robot.SetOutputResetExtDO(resetFlag=1)
    robot.SetOutputResetExtAO(resetFlag=1)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    set_do(robot)
    set_tool_do(robot)
    set_ao(robot)
    set_tool_ao(robot)
    # wait_di(robot)          # 실제 입력 신호가 준비된 경우에만 실행 권장 (블로킹됨)
    # wait_multi_di(robot)
    # move_ao_start_stop(robot)

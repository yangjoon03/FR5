"""
카테고리: 그리퍼/말단장치 - 다지(多指) 로봇 손 (Dexterous Hand)
------------------------------------------------------------
집게형 그리퍼가 아니라, 사람 손처럼 여러 개의 손가락(최대 16채널,
그 중 12개를 실제 손가락으로 사용)을 각각 독립적으로 제어하는
고급 말단장치입니다. 각 손가락의 목표 각도/속도/힘을 배열로 한 번에
지정해서 다양한 파지 패턴(집기, 쥐기, 손짓 등)을 만들 수 있습니다.

원본 출처: example/TestFiveDexterousHands.py, example/Test_DexterousHands.py
"""
from fairino import Robot
import time


def clear_dexterous_hand_error(robot):
    """다지 손의 에러 상태 초기화."""
    error = robot.ClearDexterousHandsError()
    print("ClearDexterousHandsError 반환값:", error)
    return error


def configure_dexterous_hand_functions(robot, hand_id=1):
    """
    다지 손의 동작 옵션(기능 스위치)을 설정합니다.
    - SetDexterousHandsFunc(id, func_flags[32]): 32개 기능 비트 배열
      (예: 인덱스 20 = "다축 동기 운동" 사용 여부 등, 각 인덱스의 의미는
      장비 매뉴얼 참고)
    - GetDexterousHandsFunc(id): 현재 설정된 기능 비트 조회
    """
    func_flags = [0] * 32
    for idx in (2, 3, 4, 9, 10, 11, 20, 22):
        func_flags[idx] = 1
    error = robot.SetDexterousHandsFunc(hand_id, func_flags)
    print("SetDexterousHandsFunc 반환값:", error)
    return error


def activate_dexterous_hand(robot, hand_id=1):
    """다지 손 활성화(서보 On)."""
    error = robot.SetDexterousHandsAct(hand_id, 1)
    print("SetDexterousHandsAct 반환값:", error)
    return error


def move_dexterous_hand(robot, hand_id=1):
    """
    12개 손가락 관절을 동시에 목표 각도로 이동.
    - SetDexterousHandsMove(idstart, slaveNum, pos[16], speed[16], force[16], max_time):
      idstart=시작 슬레이브 번호, slaveNum=제어할 손가락(채널) 개수,
      pos/speed/force는 16칸 배열(실제 사용은 앞 12칸, 나머지 4칸은 0으로 채움)
      pos: 각 손가락 목표 각도[°], speed: 속도%, force: 파지력%,
      max_time: 최대 동작 대기시간[ms]
    """
    finger_count = 12
    pos = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0]
    speed = [50] * 12 + [0] * 4
    force = [50] * 12 + [0] * 4
    error = robot.SetDexterousHandsMove(hand_id, finger_count, pos, speed, force, 12000)
    print("SetDexterousHandsMove 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    clear_dexterous_hand_error(robot)
    configure_dexterous_hand_functions(robot)
    activate_dexterous_hand(robot)
    move_dexterous_hand(robot)

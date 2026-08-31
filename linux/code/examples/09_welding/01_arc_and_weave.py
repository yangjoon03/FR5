"""
카테고리: 용접(Welding) - 아크 시작/종료 및 위빙(Weave, 좌우 흔들기)
------------------------------------------------------------
아크 용접의 가장 기본적인 두 동작:
1) 아크(용접 불꽃) 켜기/끄기 - ARCStart / ARCEnd
2) 위빙(Weave) - 용접선을 따라가면서 토치를 좌우(또는 지그재그)로
   흔들어 용접 비드를 넓게/균일하게 만드는 동작 - WeaveStart / WeaveEnd
   (실제 흔드는 폭/주기/패턴은 티치펜던트에서 "위빙 설정 번호"로
   미리 등록해두고, 여기서는 그 번호만 지정합니다)

이 두 함수는 항상 MoveL 같은 이동 명령과 "겹쳐서" 사용합니다: 이동을
시작하기 직전에 ARCStart/WeaveStart를 호출해두면, 그 이동이 진행되는
동안 계속 아크가 유지되고 토치가 흔들립니다.

원본 출처: example/ArcWeldTrace.py (아크+위빙 기본 흐름만 발췌.
           나머지 이 파일의 심화 기능인 ArcWeldTraceControl/
           MultilayerOffsetTrsfToBase/ArcWeldTraceReplay*은
           02_arc_seam_tracking_and_multilayer.py 참고)
"""
from fairino import Robot
import time


def arc_start(robot, io_type=1, arc_num=0, timeout=3000):
    """
    아크(용접 불꽃) 켜기.
    - io_type: 0=컨트롤러 자체 IO, 1=확장 IO 로 용접기와 연결
    - arc_num: 미리 저장해둔 "용접기 설정 프로파일" 번호
    - timeout: 아크 발생 확인 대기시간 [ms] (이 시간 안에 아크가 안 붙으면 에러)
    """
    error = robot.ARCStart(io_type, arc_num, timeout)
    print("ARCStart 반환값:", error)
    return error


def arc_end(robot, io_type=1, arc_num=0, timeout=10000):
    """아크 끄기. timeout: 소호(불꽃 꺼짐) 확인 대기시간 [ms]."""
    error = robot.ARCEnd(io_type, arc_num, timeout)
    print("ARCEnd 반환값:", error)
    return error


def weave_start(robot, weave_num=0):
    """
    위빙 시작. weave_num: 티치펜던트에 미리 등록해둔 위빙 패턴 번호
    (진폭, 주기, 좌우/삼각/원형 패턴 등이 그 번호에 저장되어 있음).
    """
    error = robot.WeaveStart(weave_num)
    print("WeaveStart 반환값:", error)
    return error


def weave_end(robot, weave_num=0):
    """위빙 종료."""
    error = robot.WeaveEnd(weave_num)
    print("WeaveEnd 반환값:", error)
    return error


def basic_weld_pass_demo(robot):
    """
    가장 단순한 "한 줄 용접" 흐름 예시:
    안전위치 -> 용접 시작위치로 이동 -> 아크ON+위빙ON -> 용접선 직선이동
    -> 위빙OFF+아크OFF -> 안전위치로 복귀
    """
    safe_j = [-25.734, -63.778, 81.502, -108.975, -93.392, 56.021]
    start_j = [-24.090, -63.501, 84.288, -111.940, -93.426, 57.669]
    end_d = [-563.961, 215.359, -0.681, 2.845, -40.476, -87.443]
    tool = 13

    robot.MoveJ(safe_j, tool, 0, vel=10)
    robot.MoveJ(start_j, tool, 0, vel=10)

    arc_start(robot, io_type=1, arc_num=0, timeout=3000)
    weave_start(robot, weave_num=0)

    robot.MoveL(end_d, tool, 0, vel=1)

    weave_end(robot, weave_num=0)
    arc_end(robot, io_type=1, arc_num=0, timeout=10000)

    robot.MoveJ(safe_j, tool, 0, vel=10)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    basic_weld_pass_demo(robot)

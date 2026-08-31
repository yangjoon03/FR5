"""
카테고리: 이동(Move) - 조그(Jog) 수동 점동 이동
------------------------------------------------------------
티치펜던트의 방향 버튼을 누르고 있는 것과 같은 "점동(Jog)" 이동입니다.
목표 좌표를 지정하는 대신, "어느 축을, 어느 방향으로, 얼마나" 움직일지를
지정합니다. 버튼을 누르고 있는 동안 계속 움직이다가 손을 떼면(Stop) 멈추는
방식의 수동 조작에 사용됩니다.

StartJOG   : 조그 이동 시작
StopJOG    : 감속 정지
ImmStopJOG : 즉시 정지 (비상 정지에 가까움)

원본 출처: example/TestMotionCommand.py 의 startjog(), stopjog(), immstopjog()
"""
from fairino import Robot
import time


def start_jog(robot, ref=0, nb=1, direction=0, max_dis=10, vel=30, acc=100):
    """
    조그(점동) 이동 시작.

    - ref: 0=관절 점동, 2=베이스 좌표계, 4=툴 좌표계, 8=사용자(작업물) 좌표계
    - nb : 축 번호. 관절이면 1~6(관절1~6), 좌표계 점동이면 1~6(x,y,z,rx,ry,rz)
    - direction: 0=음(-) 방향, 1=양(+) 방향
    - max_dis: 한 번에 움직일 수 있는 최대 각도/거리 [°] 또는 [mm]
    - vel: 속도 백분율 [0~100], 기본 30
    - acc: 가속도 백분율 [0~100], 기본 100
    """
    error = robot.StartJOG(ref=ref, nb=nb, dir=direction, max_dis=max_dis, vel=vel, acc=acc)
    print("StartJOG 반환값:", error)
    return error


def stop_jog(robot, ref=1):
    """
    조그 감속 정지.
    - ref: 1=관절 점동 정지, 3=베이스좌표계 정지, 5=툴좌표계 정지, 9=사용자좌표계 정지
      (StartJOG에서 사용한 ref 값에 맞춰 +1 한 값을 넣는다고 이해하면 쉽습니다)
    """
    error = robot.StopJOG(ref=ref)
    print("StopJOG 반환값:", error)
    return error


def imm_stop_jog(robot):
    """조그 즉시 정지 (감속 없이 바로 멈춤)."""
    error = robot.ImmStopJOG()
    print("ImmStopJOG 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    # 관절1을 양(+) 방향으로 최대 90도까지 점동 이동 후 3초 뒤 감속 정지
    start_jog(robot, ref=0, nb=1, direction=1, max_dis=90, vel=50)
    time.sleep(3)
    stop_jog(robot, ref=1)

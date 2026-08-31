"""
카테고리: 이동(Move) - 서보(Servo) 실시간 스트리밍 이동
------------------------------------------------------------
일반 Move 계열(MoveJ/MoveL 등)이 "한 지점으로 이동"을 명령하는 방식이라면,
ServoJ/ServoCart는 아주 짧은 주기(수 ms)마다 다음 위치를 계속 보내주는
방식입니다. 실시간 추종 제어, 힘 제어 연동, 외부 센서 기반 보정 등에 사용됩니다.

사용 순서: ServoMoveStart() -> ServoJ()/ServoCart() 반복 호출 -> ServoMoveEnd()

원본 출처: example/TestMotionCommand.py 의 servoj(), servocart()
"""
from fairino import Robot
import time


def servo_j_stream(robot, duration_steps=100, step_sleep=0.1):
    """
    관절 공간 서보 이동 예시.
    현재 관절 각도를 기준으로 5번 관절(j5)을 조금씩 움직이는 예시입니다.

    - joint_pos: 목표 관절 각도 [j1..j6] (매 호출마다 갱신해서 보냄)
    - axisPos: 외부 축 위치 [0,0,0,0] (외부 축이 없다면 0으로 채움)
    - cmdT: 명령 전송 주기(초), 권장 범위 0.001~0.0016 (기본 0.008)
    """
    error, pos = robot.GetActualJointPosDegree()
    robot.ServoMoveStart()
    for _ in range(duration_steps):
        time.sleep(step_sleep)
        pos[4] -= 0.2  # 5번 관절을 조금씩 이동
        error = robot.ServoJ(joint_pos=pos, axisPos=[0, 0, 0, 0])
    robot.ServoMoveEnd()
    print("ServoJ 반환값:", error)
    return error


def servo_cart_stream(robot, duration_steps=200, step_sleep=0.008):
    """
    데카르트 공간 서보 이동 예시 (증분 이동 모드).

    - mode: 0=절대 이동(기준좌표계), 1=증분 이동(기준좌표계), 2=증분 이동(툴좌표계)
    - desc_pos: 목표 위치 또는 위치 증분값 [x,y,z,rx,ry,rz]
    - exaxis: 확장 축 위치 (일반적으로 []로 비워둠)
    """
    robot.ServoMoveStart()
    pos = [0, 0, 0.2, 0, 0, 0]  # z방향으로 0.2mm씩 증분 이동
    error = 0
    for _ in range(duration_steps):
        time.sleep(step_sleep)
        error = robot.ServoCart(mode=1, desc_pos=pos, exaxis=[])
    robot.ServoMoveEnd()
    print("ServoCart 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    servo_j_stream(robot)
    # servo_cart_stream(robot)

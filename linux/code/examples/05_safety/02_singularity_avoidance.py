"""
카테고리: 안전(Safety) - 특이점(Singularity) 회피
------------------------------------------------------------
로봇 팔이 "특이점"(관절이 이론적으로 무한히 빠르게 돌아야 하는 자세,
예: 팔이 완전히 펴지거나 손목 축이 일직선으로 겹치는 자세)에 가까워지면
속도가 급격히 튀거나 경로를 벗어날 수 있습니다. SingularAvoidStart는
그런 위험 구간에 다가갈 때 자동으로 속도를 낮추거나 경로를 보정해서
보호해주는 기능입니다.

특이점의 종류 (관절 이름 기준):
- 어깨(Shoulder) 특이점: 손목 중심이 관절1 회전축 바로 위에 위치할 때
- 팔꿈치(Elbow) 특이점: 팔이 완전히 펴져서 팔꿈치가 일직선이 될 때
- 손목(Wrist) 특이점: 손목의 4축과 6축이 나란히 겹칠 때

원본 출처: example/TestSingularAvoid_new.py
"""
from fairino import Robot


def singular_avoid_start(robot, protect_mode=0, min_shoulder=100, min_elbow=50, min_wrist=10):
    """
    특이점 보호 시작.
    - protect_mode: 0=관절공간 모드, 1=데카르트(직교) 공간 모드
    - min_shoulder_pos: 어깨 특이점 보호 반경 [mm], 기본 100
    - min_elbow_pos: 팔꿈치 특이점 보호 반경 [mm], 기본 50
    - min_wrist_pos: 손목 특이점 보호 각도/반경, 기본 10

    이 값들이 클수록 더 일찍(더 멀리서) 감속/보정을 시작해 안전하지만
    움직일 수 있는 자세 범위가 줄어듭니다.
    """
    error = robot.SingularAvoidStart(protect_mode, min_shoulder, min_elbow, min_wrist)
    print("SingularAvoidStart 반환값:", error)
    return error


def singular_avoid_end(robot):
    """특이점 보호 종료."""
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd 반환값:", error)
    return error


def demo_arc_elbow_singularity(robot):
    """
    원호(MoveC) 이동 중 팔꿈치 특이점을 지나가는 예시.
    SingularAvoidStart를 원호 이동 전에 호출해두면, 팔꿈치가 펴지는
    구간에서 로봇이 자동으로 속도를 낮추거나 자세를 보정합니다.
    """
    start = [-57.170, -690.147, 370.969, 176.438, -8.320, 169.881]
    mid = [-71.044, -743.395, 375.996, -179.499, -5.398, 168.739]
    end = [-439.979, -512.743, 396.472, 178.112, 3.625, 146.576]

    robot.MoveL(desc_pos=start, tool=0, user=0, vel=50)
    singular_avoid_start(robot, protect_mode=0, min_shoulder=10, min_elbow=5, min_wrist=5)
    robot.MoveC(desc_pos_p=mid, tool_p=0, user_p=0, desc_pos_t=end, tool_t=0, user_t=0, vel_p=50, vel_t=50)
    singular_avoid_end(robot)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    demo_arc_elbow_singularity(robot)

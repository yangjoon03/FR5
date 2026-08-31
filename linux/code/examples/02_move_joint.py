"""
카테고리: 이동(Move) - 관절 공간 이동 (Joint Space Move)
------------------------------------------------------------
MoveJ: 6개 관절 각도를 목표값으로 지정해서 이동시키는 함수.
      로봇 내부적으로 관절 공간에서 가장 부드러운 경로로 움직이며,
      경로가 직선이 되는 것을 보장하지 않습니다. (경로 예측이 어려움 대신
      가장 빠르고 부드럽게 움직임)

원본 출처: example/TestMotionCommand.py 의 movej(), movej_test()
"""
from fairino import Robot


def move_j(robot, joint_pos, tool=0, user=0, vel=20.0):
    """
    관절 공간 이동 (자동 정/역기구학 계산).

    파라미터:
    - joint_pos: 목표 관절 각도 리스트 [j1,j2,j3,j4,j5,j6], 단위 [°] (필수)
    - tool: 툴 좌표계 번호, 범위 [0~14] (필수)
    - user: 사용자(작업물) 좌표계 번호, 범위 [0~14] (필수)
    - vel: 속도 백분율, 범위 [0~100], 기본 20.0
    - (그 외 desc_pos/acc/ovl/exaxis_pos/blendT/offset_flag/offset_pos 등은
      기본값을 그대로 사용해도 무방한 고급 옵션입니다)

    반환: 에러코드 (0 = 성공)
    """
    error = robot.MoveJ(joint_pos, tool=tool, user=user, vel=vel)
    print("MoveJ 반환값:", error)
    return error


def move_j_blend(robot, joint_pos, tool=0, user=0, vel=20.0, blendT=200.0):
    """
    관절 이동을 블렌딩(비정지, 부드러운 연결)으로 실행하는 예시.
    - blendT: -1.0 이면 목표점에 완전히 정지(블로킹) 후 다음 명령 실행,
      0~500(ms) 사이 값을 주면 목표점 부근에서 감속하지 않고
      다음 동작과 부드럽게 이어붙임(논블로킹에 가까움).
    """
    error = robot.MoveJ(joint_pos, tool=tool, user=user, vel=vel, blendT=blendT)
    print("MoveJ(blendT=%.1f) 반환값: %s" % (blendT, error))
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    # 예시 관절 각도(°) - 실제 사용 시 안전한 값으로 교체하세요.
    JP1 = [28.166, -108.269, -59.859, -87, 94.532, -0.7]
    JP2 = [72.515, -86.774, 81.525, -87.724, -91.964, 92.958]

    move_j(robot, JP1, tool=0, user=0, vel=30)
    move_j(robot, JP2, tool=0, user=0, vel=30)

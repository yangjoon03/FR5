"""
카테고리: 이동(Move) - 직선 이동 (Linear / Cartesian Straight-Line Move)
------------------------------------------------------------------------
MoveL   : 목표 좌표까지 "직선 경로"로 이동 (자동 정/역기구학 계산 포함)
MoveCart: 목표 좌표까지 점대점(P2P) 이동. MoveL과 달리 관절 보간 방식이며
          중간 경로는 직선을 보장하지 않지만 특이점 회피에 유리함.

원본 출처: example/TestMotionCommand.py 의 movel(), movecart(),
           jointoverspeedprotect()
"""
from fairino import Robot


def move_l(robot, desc_pos, tool=0, user=0, vel=20.0, blendR=-1.0):
    """
    데카르트(직교) 공간 직선 이동.

    파라미터:
    - desc_pos: 목표 위치/자세 [x,y,z,rx,ry,rz], 단위 [mm][°] (필수)
    - tool: 툴 좌표계 번호 [0~14] (필수)
    - user: 사용자(작업물) 좌표계 번호 [0~14] (필수)
    - vel: 속도 백분율 [0~100], 기본 20.0
    - blendR: -1.0 이면 목표점에서 완전히 정지(블로킹),
      0~1000(mm) 값을 주면 그 반경 안에서 다음 동작과 부드럽게 연결(논블로킹)

    반환: 에러코드 (0 = 성공)
    """
    error = robot.MoveL(desc_pos, tool=tool, user=user, vel=vel, blendR=blendR)
    print("MoveL 반환값:", error)
    return error


def move_cart(robot, desc_pos, tool=0, user=0, vel=20.0):
    """
    데카르트 공간 점대점(P2P) 이동.
    MoveL과 파라미터는 비슷하지만, 경로가 직선이 아닐 수 있는 대신
    특이점(singularity) 근처에서도 비교적 안전하게 동작합니다.
    """
    error = robot.MoveCart(desc_pos, tool=tool, user=user, vel=vel)
    print("MoveCart 반환값:", error)
    return error


def move_l_with_overspeed_protect(robot, desc_pos, tool=0, user=0, vel=100.0,
                                   overSpeedStrategy=3, speedPercent=100):
    """
    관절 초과속도 보호 옵션을 켠 상태에서 MoveL을 실행하는 예시.

    - overSpeedStrategy:
        0 = 보호 기능 끔
        1 = 표준
        2 = 초과속도 시 에러 정지
        3 = 적응형 감속 (초과속도가 예상되면 자동으로 속도를 낮춤)
    - speedPercent: 허용 감속 임계값 백분율 [0~100], 기본 10
    """
    error = robot.MoveL(desc_pos, tool=tool, user=user, vel=vel,
                         overSpeedStrategy=overSpeedStrategy,
                         speedPercent=speedPercent)
    print("MoveL(초과속도 보호) 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]
    DP2 = [-65.169, -529.17, 518.018, -177.189, 3.119, 69.556]

    move_l(robot, DP1, tool=0, user=0, vel=30)
    move_cart(robot, DP2, tool=0, user=0, vel=30)

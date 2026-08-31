"""
카테고리: 이동(Move) - 원호 / 원형 / 나선형 이동 (Arc / Circle / Spiral)
------------------------------------------------------------------------
MoveC     : 시작점 -> 경유점(P) -> 목표점(T) 을 지나는 "원호(arc)" 이동
Circle    : 시작점 -> 경유점(P) -> 목표점(T) 을 지나는 "완전한 원(전체 원)" 이동
            (Circle은 경유점/목표점 두 개의 점과 현재 위치, 총 3점으로 원을 그림)
NewSpiral : 나선형(스파이럴) 이동. 반지름을 점점 늘리거나 줄이며 원을 그림
            (용접/디스펜싱 등에서 나선형 패턴을 그릴 때 사용)

원본 출처: example/TestMotionCommand.py 의 movec(), circle(), newspiral()
"""
from fairino import Robot


def move_c(robot, desc_pos_p, desc_pos_t, tool_p=0, user_p=0, tool_t=0, user_t=0):
    """
    데카르트 공간 원호 이동.
    현재 위치 -> desc_pos_p(경유점) -> desc_pos_t(목표점) 을 지나는 호를 그립니다.

    - desc_pos_p / tool_p / user_p: 경유점 좌표 및 좌표계 (필수)
    - desc_pos_t / tool_t / user_t: 목표점 좌표 및 좌표계 (필수)
    """
    error = robot.MoveC(desc_pos_p=desc_pos_p, tool_p=tool_p, user_p=user_p,
                         desc_pos_t=desc_pos_t, tool_t=tool_t, user_t=user_t)
    print("MoveC 반환값:", error)
    return error


def move_circle(robot, desc_pos_p, desc_pos_t, tool_p=0, user_p=0, tool_t=0, user_t=0):
    """
    데카르트 공간 완전한 원(전체 원) 이동.
    현재 위치, desc_pos_p(경유점), desc_pos_t(목표점) 세 점을 지나는
    원을 한 바퀴 그리고, 시작 위치로 돌아옵니다.

    주의: 원호(MoveC)와 달리 한 바퀴를 전부 도는 동작이므로
    주변 장애물과의 충돌 반경을 충분히 확인해야 합니다.
    """
    error = robot.Circle(desc_pos_p=desc_pos_p, tool_p=tool_p, user_p=user_p,
                          desc_pos_t=desc_pos_t, tool_t=tool_t, user_t=user_t)
    print("Circle 반환값:", error)
    return error


def new_spiral(robot, desc_pos, param, tool=0, user=0, vel=20.0):
    """
    나선형(스파이럴) 이동.

    - desc_pos: 목표 위치/자세 [x,y,z,rx,ry,rz] (필수)
    - param: [circle_num, circle_angle, rad_init, rad_add, rotaxis_add,
              rot_direction, velAccMode]
        circle_num    : 회전 횟수(원의 개수)
        circle_angle  : 나선의 기울기 각도
        rad_init      : 초기 반지름
        rad_add       : 회전마다 반지름 증가량
        rotaxis_add   : 회전축 방향 증가량
        rot_direction : 회전 방향, 0=시계방향, 1=반시계방향
        velAccMode    : 속도/가속도 모드, 0=각속도 일정, 1=선속도 일정

    예: param=[5.0, 10, 30, 10, 5, 0, 0] -> 5바퀴, 기울기10°, 초기반지름30,
        반지름증가10, 축증가5, 시계방향, 각속도일정

    주의: 현재 설치된 SDK(code/fairino/Robot.py)는 param에 velAccMode까지
    총 7개 값을 요구합니다. example/ 폴더의 기존 예제 일부는 6개짜리
    param(예: [5.0,10,30,10,5,0])을 쓰는데, 이는 구버전 SDK 기준이라
    최신 SDK에서는 IndexError가 날 수 있으니 반드시 7개로 맞춰 호출하세요.
    """
    error = robot.NewSpiral(desc_pos=desc_pos, tool=tool, user=user,
                             param=param, vel=vel)
    print("NewSpiral 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]
    DP2 = [-65.169, -529.17, 518.018, -177.189, 3.119, 69.556]
    DP2_h = [-65.169, -529.17, 528.018, -177.189, 3.119, 69.556]
    DP3 = [102.939, -378.069, 613.165, 176.687, 1.217, 86.329]

    # 원호: 현재위치 -> DP3(경유) -> DP1(목표)
    move_c(robot, desc_pos_p=DP3, desc_pos_t=DP1)

    # 완전한 원: 현재위치 -> DP3(경유) -> DP2(목표)
    move_circle(robot, desc_pos_p=DP3, desc_pos_t=DP2)

    # 나선형: DP2_h 를 향해 5바퀴 도는 스파이럴
    new_spiral(robot, desc_pos=DP2_h, param=[5.0, 10, 30, 10, 5, 0, 0])

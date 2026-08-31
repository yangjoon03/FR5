"""
카테고리: 이동(Move) - 스플라인(여러 점을 부드럽게 잇는) 이동
------------------------------------------------------------
여러 개의 점을 미리 등록해두고, 그 점들을 부드러운 곡선으로 한 번에
이어서 움직이고 싶을 때 사용합니다. (일반 MoveL을 여러 번 호출하면
점마다 감속-가속이 생기지만, 스플라인은 점들을 부드럽게 통과함)

SplineStart / SplinePTP / SplineEnd       : 기본 스플라인(관절 기반)
NewSplineStart / NewSplinePoint / NewSplineEnd : 신규 스플라인(좌표점 기반,
    경로점을 원하는 만큼 추가 가능, 마지막 점에서 lastFlag=1)

원본 출처: example/TestMotionCommand.py 의 splineptp(), newsplineptp()
"""
from fairino import Robot


def spline_ptp(robot, joint_pos, tool=0, user=0):
    """
    기본 스플라인 이동(관절 목표점 1개짜리 간단 예시).
    실제로는 SplinePTP를 여러 번 호출해 점을 이어 붙이는 방식으로 사용합니다.
    """
    robot.SplineStart()
    error = robot.SplinePTP(joint_pos=joint_pos, tool=tool, user=user)
    robot.SplineEnd()
    print("SplinePTP 반환값:", error)
    return error


def new_spline_ptp(robot, points, tool=0, user=0, spline_type=0):
    """
    신규 스플라인 이동: 여러 개의 데카르트 좌표점을 등록해서
    부드러운 하나의 경로로 이어 붙입니다.

    - points: [[x,y,z,rx,ry,rz], ...] 형태의 좌표점 리스트 (2개 이상 권장)
    - spline_type: 0=원호 과도(부드러운 코너), 1=지정된 점 경로를 그대로 지나감

    마지막 점을 등록할 때는 lastFlag=1 을 반드시 넘겨야 스플라인이 종료 신호를
    받고 실제 동작을 시작합니다.
    """
    robot.NewSplineStart(type=spline_type)
    error = None
    for i, p in enumerate(points):
        last = 1 if i == len(points) - 1 else 0
        error = robot.NewSplinePoint(desc_pos=p, tool=tool, user=user, lastFlag=last)
    robot.NewSplineEnd()
    print("NewSplinePoint(마지막) 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    JP2 = [72.515, -86.774, 81.525, -87.724, -91.964, 92.958]
    spline_ptp(robot, JP2)

    pos1 = [-104.846, 309.573, 336.647, 179.681, -0.419, -92.692]
    pos2 = [-194.846, 309.573, 336.647, 179.681, -0.419, -92.692]
    pos3 = [-254.846, 259.573, 336.647, 179.681, -0.419, -92.692]
    pos4 = [-304.846, 259.573, 336.647, 179.681, -0.419, -92.692]

    # 먼저 시작점 근처로 일반 이동한 뒤, 스플라인으로 나머지 점을 부드럽게 통과
    robot.MoveCart(pos1, tool=0, user=0, vel=30)
    new_spline_ptp(robot, [pos1, pos2, pos3, pos4])

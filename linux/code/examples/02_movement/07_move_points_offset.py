"""
카테고리: 이동(Move) - 포인트 오프셋(Points Offset) 옵션
------------------------------------------------------------
미리 저장해둔 좌표(포인트)들을 그대로 재사용하되, 실행 시점에 특정
방향으로 "전체 위치를 일괄 이동(오프셋)"시키고 싶을 때 사용합니다.
예: 같은 궤적을 팔레트 위 여러 칸에 반복 적용할 때, 매번 좌표를
새로 계산하지 않고 오프셋만 바꿔가며 재사용할 수 있습니다.

PointsOffsetEnable  -> 오프셋 적용 시작 (이후의 Move 명령에 자동 반영)
... Move 계열 명령 실행 ...
PointsOffsetDisable -> 오프셋 적용 종료 (원래 좌표로 복귀)

원본 출처: example/TestMotionCommand.py 의 pointsoffset()
"""
from fairino import Robot


def move_with_offset(robot, desc_pos, offset_pos, tool=0, user=0, flag=0):
    """
    포인트 오프셋을 적용한 상태로 이동합니다.

    - flag: 0=베이스/사용자 좌표계 기준 오프셋, 2=툴 좌표계 기준 오프셋
    - offset_pos: 오프셋량 [dx,dy,dz,drx,dry,drz], 단위 [mm][°]
    """
    robot.PointsOffsetEnable(flag=flag, offset_pos=offset_pos)
    error = robot.MoveL(desc_pos, tool=tool, user=user)
    robot.PointsOffsetDisable()
    print("PointsOffset 적용 MoveL 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]

    # DP1 위치에서 z축으로 -100mm 만큼 오프셋 이동
    move_with_offset(robot, DP1, offset_pos=[0, 0, -100, 0, 0, 0])

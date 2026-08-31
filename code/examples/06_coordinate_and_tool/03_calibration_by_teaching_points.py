"""
카테고리: 좌표계/툴 설정 - 여러 점을 찍어서 좌표계 자동 계산(캘리브레이션)
------------------------------------------------------------
새 그리퍼나 작업물을 달았을 때, 좌표값을 손으로 계산하는 대신 로봇을
몇 개의 서로 다른 자세로 움직여 "점을 찍고" 나면 로봇이 자동으로 툴/
공작물 좌표계를 계산해주는 표준 캘리브레이션 절차입니다.

- 툴 좌표계(TCP) 계산: 같은 점을 서로 다른 자세로 찌르게 해서 그
  공통 교차점을 찾는 방식. 점이 많을수록(4점/6점) 자세(방향)까지
  정확하게 계산됩니다.
  - 6점법: SetToolPoint(1~6) -> ComputeTool()
  - 4점법(위치만, 자세 제외): SetTcp4RefPoint(1~4) -> ComputeTcp4()
- 공작물 좌표계 계산: 원점, X축 방향점, Y축(또는 XY평면) 방향점 3개를
  찍어서 평면과 방향을 계산.
  - 3점법: SetWObjCoordPoint(1~3) -> ComputeWObjCoord()
- 외부(고정된) 툴 좌표계 계산: SetExTCPPoint(1~3) -> ComputeExTCF()

원본 출처: example/TestSetCommand.py
"""
from fairino import Robot


def calibrate_tool_6point(robot, joints):
    """
    6점법 TCP 캘리브레이션. joints: 서로 다른 6개 자세의 관절각 리스트.
    같은 뾰족한 기준점(고정핀 등)을 6가지 다른 자세로 찍어야 정확합니다.
    """
    for i, j in enumerate(joints, start=1):
        robot.MoveJ(j, tool=0, user=0, vel=30)
        robot.SetToolPoint(point_num=i)

    error, tool_coord = robot.ComputeTool()
    print("6점법 계산 결과(툴좌표):", tool_coord)
    return tool_coord


def calibrate_tool_4point(robot, joints):
    """4점법 - 위치(x,y,z)만 계산, 자세는 그대로 둠. 더 간단하지만 방향 보정은 없음."""
    for i, j in enumerate(joints, start=1):
        robot.MoveJ(j, tool=0, user=0, vel=20)
        robot.SetTcp4RefPoint(point_num=i)

    error, tool_coord = robot.ComputeTcp4()
    print("4점법 계산 결과(툴좌표):", tool_coord)
    return tool_coord


def calibrate_workpiece_3point(robot, joints, save_id=4):
    """
    3점법 공작물 좌표계 캘리브레이션.
    joints[0]=원점, joints[1]=X축 방향점, joints[2]=평면상의 또 다른 점
    - ComputeWObjCoord(method): method=0 등 계산 방식 옵션
    """
    for i, j in enumerate(joints, start=1):
        robot.MoveJ(j, tool=0, user=0, vel=20)
        robot.SetWObjCoordPoint(point_num=i)

    error, wobj_coord = robot.ComputeWObjCoord(method=0)
    print("3점법 계산 결과(공작물좌표):", wobj_coord)

    robot.SetWObjCoord(id=save_id, w_coord=wobj_coord)
    robot.SetWObjList(id=save_id, w_coord=wobj_coord)
    return wobj_coord


def calibrate_external_tool_3point(robot, joints):
    """외부(고정 장착) 툴 좌표계 3점법 캘리브레이션."""
    for i, j in enumerate(joints, start=1):
        robot.MoveJ(j, tool=0, user=0, vel=20)
        robot.SetExTCPPoint(point_num=i)

    error, extool_coord = robot.ComputeExTCF()
    print("외부 툴좌표계 계산 결과:", extool_coord)
    return extool_coord


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    tool_joints = [
        [-89.407, -148.279, -83.169, -45.689, 133.689, 41.705],
        [-67.595, -143.7, -88.006, -48.514, 57.073, 56.189],
        [-88.229, -152.355, -67.815, -78.07, 129.029, 58.739],
        [-77.528, -141.519, -89.826, -37.184, 90.274, 41.769],
        [-76.744, -138.219, -97.714, -32.595, 90.255, 42.558],
        [-77.595, -138.454, -90.065, -40.014, 90.275, 41.709],
    ]
    calibrate_tool_6point(robot, tool_joints)

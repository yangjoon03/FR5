"""
카테고리: 좌표계/툴 설정 - 이동 중 좌표 변환 (Tool/WorkPiece Transform)
------------------------------------------------------------
ToolTrsfStart/End 와 WorkPieceTrsfStart/End 는 "지금부터 하는 이동 명령들의
좌표를 다른 툴/공작물 기준으로 재해석해서 실행하라"는 스위치입니다.
좌표값 자체를 다시 계산하지 않고, 같은 궤적을 다른 도구/좌표계 기준으로
재사용하고 싶을 때 유용합니다.

원본 출처: example/TestTRSF.py, example/TestWorkPieceTrsf.py
"""
from fairino import Robot
import time


def tool_transform_demo(robot, enable=True):
    """
    ToolTrsfStart(toolNum) ~ ToolTrsfEnd() 구간 안에서 실행되는 이동 명령은
    지정한 toolNum 툴 좌표계 기준으로 좌표가 재해석됩니다.
    (같은 joint_pos/desc_pos 좌표라도 어떤 툴 번호가 활성인지에 따라
    실제로 로봇이 도달하는 위치가 달라질 수 있음)
    """
    start_j = [52.850, -84.327, 102.163, -112.843, -84.131, 0.063]
    start_d = [-226.699, -501.969, 264.638, -174.973, 5.852, 143.301]
    end_j = [52.850, -77.596, 111.785, -129.196, -84.131, 0.062]
    end_d = [-226.702, -501.973, 155.833, -174.973, 5.852, 143.301]

    if enable:
        robot.ToolTrsfStart(1)
        robot.MoveJ(start_j, 0, 0, start_d)
        robot.MoveJ(end_j, 0, 0, end_d)
        robot.ToolTrsfEnd()
    else:
        robot.MoveJ(start_j, 0, 0, start_d)
        robot.MoveJ(end_j, 0, 0, end_d)


def workpiece_transform_demo(robot):
    """
    WorkPieceTrsfStart(wobjNum) ~ WorkPieceTrsfEnd() 구간 안에서 실행되는
    이동 명령은 지정한 공작물 좌표계 기준으로 좌표가 재해석됩니다.
    예: 같은 궤적 프로그램을 파레트 위 여러 칸에 반복 적용할 때, 매번
    좌표를 다시 계산하지 않고 공작물 좌표계만 바꿔서 재사용할 수 있습니다.
    (08_move_points_offset.py의 PointsOffset과 비슷한 목적이지만, 이쪽은
    "좌표계 자체"를 바꾸는 방식이라는 차이가 있습니다)
    """
    j1 = [-11.188, -64.165, -107.299, -76.706, 89.590, 92.983]
    d1 = [225.986, 190.694, 394.238, -6.230, -23.797, -98.972]
    tool, workpiece = 1, 1

    print("--- 공작물 변환 적용 전 ---")
    robot.MoveJ(joint_pos=j1, desc_pos=d1, tool=tool, user=workpiece, vel=100)

    print("--- 공작물 변환 시작 ---")
    rtn = robot.WorkPieceTrsfStart(2)
    print("WorkPieceTrsfStart 반환값:", rtn)

    robot.MoveJ(joint_pos=j1, desc_pos=d1, tool=tool, user=workpiece, vel=100)

    rtn = robot.WorkPieceTrsfEnd()
    print("WorkPieceTrsfEnd 반환값:", rtn)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    tool_transform_demo(robot, enable=True)
    # workpiece_transform_demo(robot)

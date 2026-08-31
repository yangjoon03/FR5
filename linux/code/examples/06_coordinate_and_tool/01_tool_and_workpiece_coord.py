"""
카테고리: 좌표계/툴 설정 (Coordinate & Tool)
------------------------------------------------------------
로봇 좌표는 항상 "어떤 좌표계 기준인가"에 따라 값이 달라집니다.
- 툴(Tool) 좌표계: 그리퍼/공구 끝(TCP)이 로봇 손목 플랜지 대비 어디에
  달려있는지를 정의. 이걸 정확히 설정해야 MoveL 등에서 원하는 지점을
  정확히 찌를 수 있습니다.
- 사용자/공작물(User/Workpiece) 좌표계: 작업 대상물(테이블, 지그 등)
  기준 좌표계. 부품이 놓인 방향/위치를 기준으로 좌표를 다루고 싶을 때 사용.
- 부하(Load/Payload): 툴 끝에 달린 물건의 무게/무게중심. 정확히 입력해야
  로봇이 힘/가속도를 올바르게 보상합니다.

원본 출처: example/TestCoord.py
"""
from fairino import Robot
import time


def get_current_coords(robot):
    """현재 활성화된 툴/공작물/확장툴/확장축 좌표값을 조회."""
    error, tool_coord = robot.GetCurToolCoord()
    print("현재 툴 좌표계:", tool_coord)
    error, wobj_coord = robot.GetCurWObjCoord()
    print("현재 공작물 좌표계:", wobj_coord)
    error, extool_coord = robot.GetCurExToolCoord()
    print("현재 확장 툴 좌표계:", extool_coord)
    error, exaxis_coord = robot.GetCurExAxisCoord()
    print("현재 확장축 좌표계:", exaxis_coord)


def get_coord_by_id(robot, coord_id=1):
    """
    번호(id)로 저장된 툴/공작물 좌표계 정보를 조회.
    - GetToolCoordWithID(id): (error, [x,y,z,rx,ry,rz], type, install, toolID, loadNo)
      type: 0=툴, 1=센서 / install: 0=말단 장착, 1=외부 장착
    - GetWObjCoordWithID(id): (error, [x,y,z,rx,ry,rz], refFrame)
    - GetTargetPayloadWithID(id): (error, weight[kg], cog[x,y,z])
    """
    error, tool_coord, type_val, install, tool_id, load_no = robot.GetToolCoordWithID(coord_id)
    print(f"툴좌표계[{coord_id}]:", tool_coord, "type=", type_val, "install=", install)

    error, wobj_coord, ref_frame = robot.GetWObjCoordWithID(coord_id)
    print(f"공작물좌표계[{coord_id}]:", wobj_coord, "기준프레임=", ref_frame)

    error, weight, cog = robot.GetTargetPayloadWithID(coord_id)
    print(f"부하정보[{coord_id}]: 무게={weight}kg 무게중심={cog}")


def set_tool_coord(robot, tool_id=1):
    """
    툴 좌표계 설정.
    - id: 저장할 번호 [0~14]
    - coord: [x,y,z,rx,ry,rz] 플랜지 기준 TCP 위치/자세 [mm][°]
    - type: 0=일반 툴, 1=힘센서
    - install: 0=로봇 말단 장착, 1=로봇 외부(고정) 장착
    - toolID / loadNum: 이 좌표계와 연결할 툴 번호 / 부하(payload) 번호
      (SetToolCoord는 이 6개 인자가 모두 필수이며 기본값이 없습니다)
    """
    coord = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    error = robot.SetToolCoord(tool_id, coord, 0, 0, 1, 0)
    print("SetToolCoord 반환값:", error)
    return error


def set_workpiece_coord(robot, wobj_id=1):
    """
    공작물(사용자) 좌표계 설정.
    - id: 저장할 번호 [0~14]
    - coord: [x,y,z,rx,ry,rz] 베이스 기준 공작물 원점 위치/자세
    - refFrame: 참조 프레임(0=베이스 등)
    """
    coord = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    error = robot.SetWObjCoord(wobj_id, coord, 0)
    print("SetWObjCoord 반환값:", error)
    return error


def set_payload(robot, tool_id=1):
    """
    툴 끝에 달린 부하(그리퍼+작업물)의 무게와 무게중심을 설정.
    - SetLoadWeight(id, weight[kg])
    - SetLoadCoord(x, y, z, id): 무게중심 위치 [mm] (플랜지 기준)
    """
    error = robot.SetLoadWeight(tool_id, 1.3)
    print("SetLoadWeight 반환값:", error)
    error = robot.SetLoadCoord(10.0, 20.0, 30.0, tool_id)
    print("SetLoadCoord 반환값:", error)
    return error


def set_extended_tool_coord(robot, extool_id=21):
    """
    확장 툴 좌표계 설정 (예: 로봇이 아니라 외부 고정 장치에 달린 공구를
    기준으로 좌표를 다루고 싶을 때).
    - etcp: 확장 툴 TCP 위치/자세
    - etool: 확장 공작물(워크피스) 기준 위치/자세
    """
    etcp = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
    etool = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0]
    error = robot.SetExToolCoord(extool_id, etcp, etool)
    print("SetExToolCoord 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    get_current_coords(robot)
    get_coord_by_id(robot, 1)
    set_tool_coord(robot, 1)
    set_workpiece_coord(robot, 1)
    set_payload(robot, 1)

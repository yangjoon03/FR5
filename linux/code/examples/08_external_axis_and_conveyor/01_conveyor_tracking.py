"""
카테고리: 외부축/컨베이어 - 컨베이어 벨트 트래킹 (Conveyor Tracking)
------------------------------------------------------------
컨베이어(전송) 벨트 위를 흘러가는 부품을, 벨트가 움직이는 동안 로봇이
같이 따라가며 집는(Pick) 기능입니다. 벨트 인코더 신호를 로봇이 실시간으로
읽어서, 부품의 "지금 위치"를 계속 갱신하며 이동 목표를 보정합니다.

동작 흐름:
1) ConveyorSetParam 으로 벨트 파라미터(엔코더 분해능, 속도 등) 설정
2) 기준점들(참조점/캐치점 등) 기록 - ConveyorPoint*Record 계열
3) ConveyorTrackStart 로 추적 시작
4) ConveyorIODetect 로 부품이 검출 위치를 지났는지 확인
5) ConveyorGetTrackData 로 부품의 현재 위치를 계산
6) ConveyorTrackMoveL 로 "벨트 위 이동 중인 목표점"을 향해 추적 이동
7) ConveyorTrackEnd 로 추적 종료

원본 출처: example/TestDrivingBeltCommand.py, example/TestStationaryTrack.py,
           example/ConveyorTrackSet.py
"""
from fairino import Robot
import time


def axle_sensor_setup(robot):
    """
    벨트 인코더 등 축(axle) 센서 통신 설정.
    - AxleSensorConfig(idCompany, idDevice, idSoftware, idBus): 센서 제조사/장치/버스 설정
    - AxleSensorActivate(1): 센서 활성화
    - AxleSensorRegWrite(...): 센서 레지스터 직접 쓰기 (제조사별 저수준 설정)
    """
    error = robot.AxleSensorConfig(idCompany=18, idDevice=0, idSoftware=0, idBus=1)
    print("AxleSensorConfig 반환값:", error)
    error, company, device = robot.AxleSensorConfigGet()
    print("현재 센서 설정 - 제조사:", company, "장치:", device)
    robot.AxleSensorActivate(1)


def conveyor_set_param(robot):
    """
    컨베이어 파라미터 설정.
    - param: [encoder_ch, encoder_resolution, belt_speed_mm_s, ...]
      (정확한 배열 의미는 컨트롤러 매뉴얼 참고, 엔코더 채널/분해능/속도 등)
    """
    error = robot.ConveyorSetParam(param=[1, 10000, 2.0, 0, 1, 20])
    print("ConveyorSetParam 반환값:", error)
    return error


def conveyor_record_reference_points(robot):
    """
    벨트 추적에 필요한 기준점들을 현재 로봇 위치로 기록합니다.
    - ConveyorPointIORecord(): IO 검출 위치 기록 (부품이 이 지점을 지나면 센서 반응)
    - ConveyorPointARecord() / ConveyorPointBRecord(): 벨트 이동 방향을 알기 위한
      A점/B점 기록 (두 점 사이 거리와 엔코더 변화량으로 벨트 속도/방향 계산)
    - ConveyorRefPointRecord(): 추적 좌표계의 기준(원점) 기록
    """
    robot.ConveyorPointIORecord()
    robot.ConveyorPointARecord()
    robot.ConveyorRefPointRecord()
    robot.ConveyorPointBRecord()


def conveyor_track_and_pick(robot):
    """
    벨트 추적 + 그리퍼로 집기까지의 전체 흐름 예시.
    - ConveyorCatchPointComp(cmp): 실제 파지 위치 보정값 [dx,dy,dz]
    - ConveyorIODetect(max_t): 부품이 검출센서를 지나가길 최대 max_t[ms] 대기
    - ConveyorGetTrackData(mode): 현재 부품 위치 계산 (mode=1: 동적 추적)
    - ConveyorTrackStart(status): 1=추적 시작
    - ConveyorTrackMoveL(point_name, tool, user): 등록된 이름(point_name)의
      "추적 목표점"으로 직선 이동 (목표가 계속 움직이는 벨트 위 좌표라는 점이
      일반 MoveL과 다름)
    - ConveyorTrackEnd(): 추적 종료
    """
    pos1 = [-351.549, 87.914, 354.176, -179.679, -0.134, 2.468]
    pos2 = [-351.558, -247.286, 354.131, -179.679, -0.142, 2.474]

    error = robot.ConveyorCatchPointComp(cmp=[0.0, 0.0, 0.0])
    if error != 0:
        print("ConveyorCatchPointComp 실패:", error)
        return

    robot.MoveCart(pos1, 1, 0)
    robot.ConveyorIODetect(max_t=10000)
    robot.ConveyorGetTrackData(mode=1)
    robot.ConveyorTrackStart(status=1)
    robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0)
    robot.MoveGripper(1, 60, 60, 30, 30000, 0)
    robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0)
    robot.ConveyorTrackEnd()
    robot.MoveCart(pos2, 1, 0)
    robot.MoveGripper(1, 100, 60, 30, 30000, 0)


def stationary_track_demo(robot):
    """
    "정지 추적(Stationary Track)": 벨트가 멈춘 상태에서, 벨트 위 부품 좌표를
    한 번 계산해서 그 지점으로 이동하는 단순화된 버전 (실시간 추적이 아님).
    - SetStationaryTrackPara / MoveStationary / WaitStationaryMotionDone
    """
    j1 = [-35.146, -102.684, 120.805, -100.401, -90.295, 150.105]
    d1 = [-121.814, -348.341, 209.978, -173.152, -3.585, -5.446]

    robot.ConveyorSetParam(param=[0, 10000, 200, 0, 0, 10])
    robot.MoveJ(joint_pos=j1, desc_pos=d1, tool=1, user=1, vel=100)

    robot.SetDO(6, 1, 0, 0)
    robot.ConveyorTrackStart(2)
    robot.ConveyorIODetect(10000)
    robot.ConveyorGetTrackData(2)
    robot.SetStationaryTrackPara(0, 5, 5)
    robot.MoveStationary()
    robot.WaitStationaryMotionDone()
    robot.ConveyorTrackEnd()
    robot.SetDO(6, 0, 0, 0)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    conveyor_set_param(robot)
    # conveyor_record_reference_points(robot)
    # conveyor_track_and_pick(robot)
    # stationary_track_demo(robot)

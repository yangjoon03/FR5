"""
카테고리: 외부축/컨베이어 - 확장(외부)축 및 견인차(Tractor/AGV)
------------------------------------------------------------
로봇 6축 이외에 추가로 붙는 "외부축"(회전 테이블, 리니어 슬라이드,
AGV/견인차 등)을 UDP 통신으로 직접 구동하는 저수준(로우레벨) 기능들입니다.
설정이 많고 컨트롤러/장비 사양에 매우 의존적이므로, 실제 사용 시 반드시
장비 제조사 매뉴얼과 함께 파라미터를 맞춰야 합니다. 여기서는 "어떤 순서로,
어떤 함수를 호출하는지" 흐름 위주로 정리합니다.

원본 출처: example/ExtendedAxes_Test.py, example/TestTractorCommand.py
"""
from fairino import Robot
import time


def setup_udp_external_axis_link(robot, plc_ip="192.168.58.2", port=2021):
    """
    외부축을 제어하는 PLC/드라이버와 UDP로 직접 연결하는 초기 설정 흐름.
    - ExtDevSetUDPComParam(ip, port, ...): 통신 파라미터 (주기, 끊김감지 시간 등)
    - ExtDevLoadUDPDriver(): UDP 드라이버 로드(연결 시작)
    - ExtAxisParamConfig(axisId, ...): 각 외부축의 위치범위/속도/감속비/
      인코더 분해능 등 물리 파라미터 설정
    - SetAxisDHParaConfig(axisId, ...): 외부축의 DH 파라미터(로봇 기준
      기구학적 위치/방향) 설정 - 로봇 좌표계와 외부축 좌표계를 연결하는 값
    """
    error = robot.ExtDevSetUDPComParam(plc_ip, port, 2, 50, 5, 50, 1, 50, 10)
    print("ExtDevSetUDPComParam 반환값:", error)
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver 반환값:", error)

    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0)


def activate_external_axis_coord(robot, axis_coord_num=0, tool_num=0, coord=None, calib_flag=0):
    """
    외부축 좌표계를 활성화합니다. 이후 이동 명령들이 이 좌표계를 인식하게 됨.
    - axisCoordNum: 외부축 번호(비트마스크로 여러 축 동시 지정 가능)
    - toolNum: 연결할 좌표계 번호
    - coord: 좌표계 값 [x,y,z,rx,ry,rz]
    - calibFlag: 0=캘리브레이션 안 함, 1=캘리브레이션 수행
    """
    coord = coord or [0, 0, 0, 0, 0, 0]
    error = robot.ExtAxisActiveECoordSys(axis_coord_num, tool_num, coord, calib_flag)
    print("ExtAxisActiveECoordSys 반환값:", error)
    return error


def jog_external_axis(robot, axis_id=1, direction=1, vel=50, acc=50, max_distance=10):
    """
    외부축을 조그(수동 점동)로 움직입니다. (07_move_jog.py의 StartJOG와
    비슷한 개념이지만 로봇 6축이 아니라 외부축 전용 함수를 사용)
    """
    error = robot.ExtAxisStartJog(axis_id, direction, vel, acc, max_distance)
    print("ExtAxisStartJog 반환값:", error)
    return error


def tractor_move_demo(robot):
    """
    견인차/AGV형 외부 이동체 제어 예시.
    - TractorEnable(state): 견인차 서보 On/Off
    - TractorHoming(): 원점 복귀
    - TractorMoveL(distance_mm, vel): 직선 이동
    - TractorMoveC(radius_mm, angle_deg, vel): 원호 이동
    - TractorStop(): 정지
    """
    setup_udp_external_axis_link(robot)

    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)

    robot.TractorMoveL(100, 20)
    time.sleep(5)
    robot.TractorMoveL(-100, 20)
    time.sleep(5)
    robot.TractorMoveC(50, 60, 20)
    time.sleep(5)
    error = robot.TractorMoveC(50, -60, 20)
    print("TractorMoveC 반환값:", error)


def tractor_stop_demo(robot):
    """이동 중인 견인차를 강제로 정지시키는 예시."""
    setup_udp_external_axis_link(robot)
    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)
    robot.TractorMoveC(300, 90, 20)
    time.sleep(4)
    error = robot.TractorStop()
    print("TractorStop 반환값:", error)
    return error


def welder_control_mode_switch(robot):
    """
    용접기(Weld machine)를 특정 DO 채널로 제어 모드 전환하는 예시.
    외부 장치(용접기)를 UDP/DO로 함께 제어할 때 사용하는 패턴입니다.
    - SetWeldMachineCtrlModeExtDoNum(DONum): 어떤 DO 채널로 용접기를 제어할지 지정
    - SetWeldMachineCtrlMode(mode): 0/1 두 모드를 토글
    """
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()

    robot.SetWeldMachineCtrlModeExtDoNum(DONum=17)
    for _ in range(5):
        robot.SetWeldMachineCtrlMode(mode=0)
        time.sleep(0.5)
        robot.SetWeldMachineCtrlMode(mode=1)
        time.sleep(0.5)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    # tractor_move_demo(robot)   # 실제 견인차 장비가 연결된 경우에만 실행
    # welder_control_mode_switch(robot)

"""
카테고리: 힘 제어 (Force/Torque Control) - 6축 힘토크 센서
------------------------------------------------------------
손목에 장착하는 6축 힘/토크(F/T) 센서를 이용해 "힘을 느끼면서" 움직이는
기능들. 정밀 조립, 표면 추종(디버링/폴리싱), 협동작업(드래그), 충돌 감지
등에 사용됩니다.

원본 출처: example/TestForceControlCommand.py

⚠️ 매우 중요 (발견한 SDK/예제 불일치): 원본 예제의 ft_contol()/ft_test() 등은
`robot.FT_Control(..., sensor_num=8, force_torque=[...], gain=[...], ...)`
처럼 예전 파라미터 이름을 사용합니다. 그러나 현재 SDK
(`code/fairino/Robot.py`)의 실제 시그니처는
`FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M=None, B=None, ...)`
로 파라미터 이름이 `sensor_num→sensor_id`, `force_torque→ft`, `gain→ft_pid`로
바뀌었습니다. 구예제 코드를 그대로 실행하면
`TypeError: FT_Control() got an unexpected keyword argument 'sensor_num'`
가 발생합니다. 아래 함수들은 새 파라미터 이름으로 수정해서 작성했습니다.
"""
from fairino import Robot
import time


# ------------------------------------------------------------------
# 센서 설정 / 활성화 / 영점 교정
# ------------------------------------------------------------------
def ft_config(robot, company=24, device=0):
    """
    힘센서 통신 설정.
    - company: 센서 제조사 코드 (예: 17=쿤웨이/坤维, 24=... 컨트롤러 매뉴얼 참고)
    - device: 장치 번호
    """
    error = robot.FT_SetConfig(company=company, device=device)
    print("FT_SetConfig 반환값:", error)
    time.sleep(1)
    error, config = robot.FT_GetConfig()
    print("센서번호:%s 제조사:%s 장치번호:%s SW버전:%s" % tuple(config))
    return error


def ft_activate(robot):
    """
    힘센서 활성화 및 영점(zero) 교정.
    - FT_Activate(state): 0=리셋, 1=활성화
    - FT_SetZero(state): 0=영점 해제, 1=영점 교정(현재 힘값을 0으로 맞춤)
    센서를 쓰기 전 반드시 활성화 -> 영점교정 순서를 거쳐야 정확한 값이 나옵니다.
    """
    robot.FT_Activate(state=0)
    time.sleep(1)
    error = robot.FT_Activate(state=1)
    print("힘센서 활성화:", error)
    time.sleep(1)
    robot.FT_SetZero(state=0)
    time.sleep(1)
    error = robot.FT_SetZero(state=1)
    print("영점 교정:", error)
    return error


# ------------------------------------------------------------------
# 부하(그리퍼/공구) 무게 자동 식별
# ------------------------------------------------------------------
def ft_payload_identify(robot, tool_id=10):
    """
    센서 끝에 달린 그리퍼/공구의 무게와 무게중심을 자동으로 계산합니다.
    로봇이 3개의 서로 다른 자세를 취하며 데이터를 기록한 뒤 계산합니다.

    - FT_PdIdenRecord / FT_PdCogIdenCompute: 무게 식별
    - FT_PdCogIdenRecord(tool_id, index): index번째 자세에서 데이터 기록
    """
    robot.FT_SetRCS(ref=0, coord=[0, 0, 0, 1, 0, 0])  # 기준좌표계: 0=툴, 1=베이스
    robot.SetToolCoord(tool_id, [0.0] * 6, 1, 0)  # tool_type=1(센서), install=0(말단 장착)
    robot.FT_PdIdenRecord(tool_id)
    error = robot.FT_PdCogIdenCompute(tool_id)
    print("부하 무게 계산 반환값(kg):", error)

    d1 = [-14.404, -455.283, 319.847, -172.935, 25.141, -68.097]
    d2 = [-107.999, -599.174, 285.939, 153.472, 12.686, -71.284]
    d3 = [6.586, -704.897, 309.638, 178.909, -27.759, -70.479]
    robot.MoveCart(d1, tool=2, user=0)
    robot.FT_PdCogIdenRecord(tool_id, 1)
    robot.MoveCart(d2, tool=2, user=0)
    robot.FT_PdCogIdenRecord(tool_id, 2)
    robot.MoveCart(d3, tool=2, user=0)
    robot.FT_PdCogIdenRecord(tool_id, 3)
    error, pos = robot.FT_PdCogIdenCompute()
    print("무게중심 계산 결과:", pos)
    return error


# ------------------------------------------------------------------
# 정력(定力) 제어 - "이 방향으로는 항상 N만큼의 힘만 유지하며 이동"
# ------------------------------------------------------------------
def ft_constant_force_control(robot):
    """
    지정한 방향(축)으로 목표 힘을 유지하면서 나머지 방향은 일반 위치제어로
    움직이는 "정력 제어(Constant Force Control)".
    예: 표면을 일정한 압력으로 누르면서 디버링/폴리싱 하는 용도.

    - flag: 1=시작, 0=종료
    - sensor_id: 힘센서 번호
    - select: [fx,fy,fz,mx,my,mz] 각 축의 힘제어 적용 여부(0/1).
      예 [0,0,1,0,0,0]은 "z축 방향 힘만 제어"
    - ft: 각 축의 목표 힘/토크값 [N] 또는 [N·m]
    - ft_pid: [f_p,f_i,f_d,m_p,m_i,m_d] 힘 PID 게인
    - adj_sign: 0=자동보정 끔, 1=켬
    - max_dis / max_ang: 목표 자세 대비 최대 보정 거리[mm]/각도[°]
    """
    jp1 = [-21.724, -136.814, -59.518, -68.853, 89.245, -66.359]
    dp2 = [738.755, -102.812, 226.704, 177.488, 2.566, -27.209]

    robot.FT_Activate(state=1)
    robot.MoveJ(jp1, tool=0, user=0, vel=10)

    error = robot.FT_Control(
        flag=1, sensor_id=8, select=[0, 0, 1, 0, 0, 0], ft=[0, 0, -10, 0, 0, 0],
        ft_pid=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)
    print("FT_Control(정력제어 시작) 반환값:", error)

    robot.MoveL(desc_pos=dp2, tool=0, user=0, vel=10)

    error = robot.FT_Control(
        flag=0, sensor_id=8, select=[0, 0, 1, 0, 0, 0], ft=[0, 0, -10, 0, 0, 0],
        ft_pid=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)
    print("FT_Control(정력제어 종료) 반환값:", error)
    robot.FT_Activate(state=0)
    return error


# ------------------------------------------------------------------
# 충돌 감지(가드) - 힘이 임계값을 넘으면 자동으로 멈춤
# ------------------------------------------------------------------
def ft_collision_guard(robot):
    """
    이동 중 힘/토크가 지정한 범위를 벗어나면 로봇을 즉시 정지시키는 안전 기능.
    감지 범위 = (ft - min_threshold, ft + max_threshold)

    - select: 감시할 축 [fx,fy,fz,mx,my,mz] (0/1)
    - force_torque: 기준 힘/토크값
    - max_threshold / min_threshold: 기준값 대비 허용 상한/하한
    """
    d1 = [-14.404, -455.283, 319.847, -172.935, 25.141, -68.097]
    d2 = [-107.999, -599.174, 285.939, 153.472, 12.686, -71.284]

    robot.FT_Guard(flag=1, sensor_num=8, select=[1, 0, 0, 0, 0, 0], force_torque=[0, 0, 0, 0, 0, 0],
                   max_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01],
                   min_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    robot.MoveCart(d1, tool=2, user=0, vel=30)
    robot.MoveCart(d2, tool=2, user=0, vel=30)
    error = robot.FT_Guard(flag=0, sensor_num=8, select=[1, 0, 0, 0, 0, 0], force_torque=[0, 0, 0, 0, 0, 0],
                            max_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01],
                            min_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    print("FT_Guard 반환값:", error)
    return error


# ------------------------------------------------------------------
# 유연(컴플라이언스) 제어 - 사람이 밀면 밀리는 듯한 부드러운 저항
# ------------------------------------------------------------------
def ft_compliance(robot):
    """
    지정한 방향으로 외력이 가해지면 로봇이 스프링처럼 부드럽게 반응하도록
    만드는 기능. FT_ComplianceStart(p, force) - p: 순응 계수(클수록 부드러움),
    force: 반응을 시작하는 힘 임계값.
    """
    dp1 = [703.996, -391.695, 240.708, -178.756, -4.709, -45.447]
    robot.FT_Control(flag=1, sensor_id=8, select=[1, 1, 1, 0, 0, 0], ft=[-10, -10, -10, 0, 0, 0],
                      ft_pid=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)
    error = robot.FT_ComplianceStart(p=0.00005, force=10.0)
    print("FT_ComplianceStart 반환값:", error)
    robot.MoveL(dp1, tool=0, user=0, vel=10)
    error = robot.FT_ComplianceStop()
    print("FT_ComplianceStop 반환값:", error)
    robot.FT_Control(flag=0, sensor_id=8, select=[1, 1, 1, 0, 0, 0], ft=[-10, -10, -10, 0, 0, 0],
                      ft_pid=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)
    return error


# ------------------------------------------------------------------
# 힘센서 보조 드래그 티칭 / 부하 설정
# ------------------------------------------------------------------
def ft_assisted_drag(robot):
    """
    사람이 손으로 로봇을 밀면 힘센서 값으로 그 방향을 감지해서 부드럽게
    따라가도록 만드는 "힘 보조 드래그" 기능.
    - M/B/K/F: 임피던스 제어 파라미터 (질량/댐핑/강성/힘 임계값)
    - Fmax/Vmax: 최대 허용 힘 / 최대 속도 (안전 상한)
    """
    error = robot.EndForceDragControl(
        status=1, asaptiveFlag=0, interfereDragFlag=0, ingularityConstraintsFlag=0,
        M=[15.0, 15.0, 15.0, 0.5, 0.5, 0.1], B=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0],
        K=[0.0] * 6, F=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0], Fmax=50, Vmax=100)
    print("EndForceDragControl(시작) 반환값:", error)
    time.sleep(5)
    error = robot.EndForceDragControl(
        status=0, asaptiveFlag=0, interfereDragFlag=0, ingularityConstraintsFlag=0,
        M=[15.0, 15.0, 15.0, 0.5, 0.5, 0.1], B=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0],
        K=[0.0] * 6, F=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0], Fmax=50, Vmax=100)
    print("EndForceDragControl(종료) 반환값:", error)
    return error


def ft_set_payload(robot):
    """힘센서 기준 부하(그리퍼) 무게/무게중심을 수동으로 설정/조회."""
    robot.SetForceSensorPayload(1.34)
    robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765)
    error, weight = robot.GetForceSensorPayload()
    print("부하 무게:", weight)
    error, x, y, z = robot.GetForceSensorPayloadCog()
    print("부하 무게중심:", x, y, z)
    return error


# ------------------------------------------------------------------
# 힘 기반 탐색 전략 - 조립/끼움 작업에서 정확한 위치를 "느껴서" 찾기
# ------------------------------------------------------------------
def ft_spiral_search(robot, rcs=0, ft=5, dr=0.7, max_t_ms=3000.0, max_vel=3.0, strategy=0):
    """
    나선형으로 움직이며 힘이 감지되는 지점(구멍 등)을 탐색.
    - rcs: 기준좌표계, 0=툴좌표계, 1=베이스좌표계
    - ft: 감지 힘/토크 임계값 [N/Nm], 0~100
    - dr: 한 바퀴당 반지름 증가량 [mm]
    - max_t_ms: 최대 탐색 시간 [ms]
    - max_vel: 최대 선속도 [mm/s]
    - strategy: 힘이 감지되지 않을 때 처리, 0=에러, 1=경고 후 계속 진행
    """
    error = robot.FT_SpiralSearch(rcs, ft, dr, max_t_ms, max_vel, strategy)
    print("FT_SpiralSearch 반환값:", error)
    return error


def ft_linear_insertion(robot, rcs=0, ft=20.0, disMax=15.0, linorn=1, lin_v=10.0, lin_a=1.0, strategy=0):
    """
    한 방향으로 직선 이동하며 힘이 임계값에 도달할 때까지 삽입하는 동작
    (핀을 구멍에 끼우는 조립 작업 등에 사용).
    - disMax: 최대 삽입 거리 [mm]
    - linorn: 삽입 방향, 0=음(-), 1=양(+)
    """
    error = robot.FT_LinInsertion(rcs, ft, disMax, linorn, lin_v, lin_a, strategy)
    print("FT_LinInsertion 반환값:", error)
    return error


def ft_find_surface(robot, rcs=0, direction=1, axis=1, disMax=15.0, ft=5, lin_v=50.0, lin_a=20.0, strategy=0):
    """
    지정한 축 방향으로 이동하다가 표면에 닿아 힘이 임계값을 넘으면 정지
    (표면 위치를 사전에 정확히 몰라도 "찾아서" 접촉시킴).
    - direction: 이동 방향, 1=양(+), 2=음(-)
    - axis: 이동 축, 1=x, 2=y, 3=z
    - ft: 접촉 판정 힘 임계값 [N]
    """
    error = robot.FT_FindSurface(rcs, direction, axis, disMax, ft, lin_v, lin_a, strategy)
    print("FT_FindSurface 반환값:", error)
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    ft_config(robot)
    ft_activate(robot)
    # ft_constant_force_control(robot)
    # ft_collision_guard(robot)
    # ft_compliance(robot)
    # ft_spiral_search(robot)
    # ft_linear_insertion(robot)
    # ft_find_surface(robot)


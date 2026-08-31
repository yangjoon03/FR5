"""
카테고리: 상태/로그 모니터링 (Status & Logging) - 2. 진단/버전/운동학 조회
------------------------------------------------------------
로봇 설정값 조회, 하드웨어/펌웨어 버전 확인, 정/역기구학 계산,
통신 끊김 시 정지 옵션 등 "설치/진단" 성격의 조회 함수 모음.

원본 출처: example/TestStatusCheck.py, example/TestRobotStopOnComDisc.py,
           example/GetJointDriverTorque_Temperature.py
"""
from fairino import Robot
import time


def install_angle(robot):
    """
    로봇 설치 각도 설정/조회. (바닥/벽/천장 등 어떻게 설치됐는지 알려줘야
    중력 보상이 정확해집니다)
    - SetRobotInstallAngle(yangle, zangle): 설치 기울기 각도 [°]
    """
    robot.SetRobotInstallAngle(yangle=90, zangle=90)
    error, angle = robot.GetRobotInstallAngle()
    print("현재 설치각도:", angle)
    return error


def kinematics_demo(robot):
    """
    정기구학/역기구학 계산 (로봇을 실제로 움직이지 않고 좌표만 계산).
    - GetInverseKin(type, desc_pos): 데카르트 좌표 -> 관절각 계산
    - GetInverseKinHasSolution(type, desc_pos, joint_pos_ref): 그 좌표에
      도달 가능한 해가 있는지만 빠르게 확인 (참조 관절각 근처에서 탐색)
    - GetForwardKin(joint_pos): 관절각 -> 데카르트 좌표 계산
    """
    DP1 = [327.359, -420.973, 518.377, -177.199, 3.209, 114.449]
    JP1 = [117.408, -86.777, 81.499, -87.788, -92.964, 92.959]
    JP2 = [72.515, -86.774, 81.525, -87.724, -91.964, 92.958]

    error, jp = robot.GetInverseKin(type=0, desc_pos=DP1)
    print("역기구학 결과(관절각):", jp, "/ 원본:", JP1)

    error, has_sol = robot.GetInverseKinHasSolution(type=0, desc_pos=DP1, joint_pos_ref=JP1)
    print("역기구학 해 존재 여부:", has_sol)

    error, dp = robot.GetForwardKin(joint_pos=JP2)
    print("정기구학 결과(좌표):", dp)


def version_and_diagnostics(robot):
    """하드웨어/펌웨어 버전, SSH 공개키 조회."""
    error, *hard = robot.GetSlaveHardVersion()
    print("하드웨어 버전:", hard)
    error, *firm = robot.GetSlaveFirmVersion()
    print("펌웨어 버전:", firm)
    error, key = robot.GetSSHKeygen()
    print("SSH 공개키:", key)


def config_and_limits(robot):
    """현재 설정값 일괄 조회: 부하, 관절 구성, 속도, 좌표계 오프셋, 소프트 리밋."""
    error, payload = robot.GetTargetPayload()
    error, cog = robot.GetTargetPayloadCog()
    print("현재 부하:", payload, "무게중심:", cog)

    error, joints_cfg = robot.GetRobotCurJointsConfig()
    print("현재 관절 구성:", joints_cfg)

    error, clock = robot.GetSystemClock()
    print("시스템 시간:", clock)

    error, vel = robot.GetDefaultTransVel()
    print("기본 속도:", vel)

    error, tcp = robot.GetTCPOffset()
    print("현재 툴좌표계 오프셋:", tcp)

    error, wobj = robot.GetWObjOffset()
    print("현재 공작물좌표계 오프셋:", wobj)

    error, soft_limit = robot.GetJointSoftLimitDeg()
    print("관절 소프트 리밋[°]:", soft_limit)


def motion_done_and_teaching_point(robot):
    """
    이동 완료 여부 폴링, 저장된 티칭 포인트 조회.
    - GetRobotMotionDone(): 직전 Move 명령이 끝났는지 확인 (논블로킹 이동 후 대기용)
    - GetRobotTeachingPoint(name): 티치펜던트에서 저장한 포인트 이름으로 조회
    """
    error, done = robot.GetRobotMotionDone()
    print("이동 완료 여부:", done)
    error, point = robot.GetRobotTeachingPoint(name="P1")
    print("티칭 포인트 'P1':", point)


def joint_torque_and_temperature(robot):
    """
    관절을 움직이면서 구동기 토크/온도를 관찰하는 예시.
    - GetJointDriverTorque(): 각 관절 현재 토크
    - GetJointDriverTemperature(): 각 관절 구동기 온도 [°C]
    """
    error, joint = robot.GetActualJointPosDegree()
    joint[0] += 10
    robot.MoveJ(joint, 0, 0)
    print("관절 토크:", robot.GetJointDriverTorque())
    print("관절 온도:", robot.GetJointDriverTemperature())
    joint[0] -= 10
    robot.MoveJ(joint, 0, 0)


def stop_on_communication_disconnect(robot):
    """
    PC-로봇 간 통신(RPC/UDP 등)이 끊겼을 때 로봇을 자동으로 정지시킬지
    설정합니다. 원격 제어 중 네트워크 장애로 로봇이 계속 움직이는
    상황을 막는 안전 기능입니다.
    - SetRobotStopOnComDisc(index, enable, confirm_time_ms): index=채널 번호,
      enable=사용 여부, confirm_time_ms=끊김 확정까지 대기시간[ms]
    """
    for idx, ms in [(0, 330), (1, 550), (2, 110), (3, 220)]:
        rtn = robot.SetRobotStopOnComDisc(idx, True, ms)
        print(f"SetRobotStopOnComDisc({idx}) 반환값:", rtn)

    for idx in range(4):
        rtn, enable, confirm_time = robot.GetRobotStopOnComDisc(idx)
        print(f"GetRobotStopOnComDisc({idx}): enable={enable}, confirm_time={confirm_time}ms")


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    kinematics_demo(robot)
    config_and_limits(robot)
    motion_done_and_teaching_point(robot)
    # stop_on_communication_disconnect(robot)

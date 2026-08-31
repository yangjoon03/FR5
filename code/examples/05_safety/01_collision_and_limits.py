"""
카테고리: 안전(Safety) - 충돌 감지 / 관절 제한 / 마찰 보상 / 토크 제한
------------------------------------------------------------
로봇이 사람이나 장애물과 부딪혔을 때 어떻게 반응할지, 관절이 얼마나
회전할 수 있는지, 힘센서 없이도 마찰력을 보상해 부드럽게 움직이게
하는 기능, 관절 파워(토크)를 제한하는 기능들을 모아 정리했습니다.

원본 출처: example/TestSafetySetCommand.py, example/SafetyParams_Test.py
"""
from fairino import Robot
import time


def set_collision_level(robot):
    """
    충돌 감지 민감도를 설정합니다.
    - mode: 0=등급(1~10단계), 1=백분율(0~100%)
    - level: [j1..j6] 각 관절의 충돌 감지 임계값 (mode에 따라 단위가 등급/%)
    - config: 설정을 즉시 적용할지 등 옵션 플래그
    값이 작을수록 더 민감하게(작은 충격에도) 충돌로 감지합니다.
    """
    error = robot.SetAnticollision(mode=0, level=[2.0] * 6, config=1)
    print("SetAnticollision 반환값:", error)
    return error


def set_collision_strategy(robot):
    """
    충돌이 감지됐을 때 로봇이 어떻게 반응할지 정책을 설정합니다.
    - strategy: 0=에러 후 일시정지, 1=계속 진행, 2=에러 후 정지,
      3=중력 보상 모드, 4=진동 반응 모드, 5=충돌 후 튕겨나가기(리바운드) 모드
    - safeTime: 안전 정지까지 걸리는 시간 [ms]
    - safeDistance: 안전 정지 거리 [mm]
    - safetyMargin: [j1..j6] 각 관절의 안전 여유값
    """
    error = robot.SetCollisionStrategy(strategy=0, safeTime=1000, safeDistance=150,
                                        safetyMargin=[10] * 6)
    print("SetCollisionStrategy 반환값:", error)
    return error


def set_static_collision(robot):
    """정지 상태에서도 충돌을 감지해서 계속 동작할지 여부를 설정."""
    error = robot.SetStaticCollisionOnOff(status=1)
    print("SetStaticCollisionOnOff 반환값:", error)
    return error


def set_joint_limits(robot):
    """
    관절 소프트웨어 리밋(회전 가능 범위)을 설정합니다.
    - SetLimitPositive(p_limit): 각 관절의 양(+)방향 최대 각도 [°]
    - SetLimitNegative(n_limit): 각 관절의 음(-)방향 최대 각도 [°]
    실제 하드웨어 한계보다 안쪽으로 좁혀서 설정하면 특정 자세로
    넘어가지 못하게 소프트웨어적으로 제한할 수 있습니다.
    """
    error = robot.SetLimitPositive(p_limit=[170.0, 80.0, 150.0, 80.0, 170.0, 160.0])
    print("SetLimitPositive 반환값:", error)
    error = robot.SetLimitNegative(n_limit=[-170.0, -260.0, -150.0, -260.0, -170.0, -160.0])
    print("SetLimitNegative 반환값:", error)
    return error


def set_friction_compensation(robot, mount="level"):
    """
    관절 마찰력 보상 계수를 설정합니다. (힘센서 없이도 드래그 티칭이 부드럽도록)
    로봇 설치 자세에 따라 함수가 다릅니다 (마찰 방향이 중력에 따라 달라지므로):
    - mount="level"  : 바닥 정착(정방향) 설치 -> SetFrictionValue_level
    - mount="wall"   : 벽면 측면 설치         -> SetFrictionValue_wall
    - mount="ceiling": 천장 역방향 설치        -> SetFrictionValue_ceiling
    - mount="freedom": 임의 각도 자유 설치     -> SetFrictionValue_freedom
    - coeff: [j1..j6] 각 관절 마찰 보상 계수
    """
    robot.FrictionCompensationOnOff(state=1)
    coeff = [0.5] * 6
    if mount == "level":
        error = robot.SetFrictionValue_level(coeff=coeff)
    elif mount == "wall":
        error = robot.SetFrictionValue_wall(coeff=coeff)
    elif mount == "ceiling":
        error = robot.SetFrictionValue_ceiling(coeff=coeff)
    else:
        error = robot.SetFrictionValue_freedom(coeff=coeff)
    print("SetFrictionValue_%s 반환값: %s" % (mount, error))
    return error


def joint_power_limit_and_torque_servo(robot):
    """
    관절 파워(토크x각속도) 상한을 걸어두고, 현재 관절 토크를 계속 유지하도록
    실시간 토크 서보(ServoJT)를 흘려보내는 예시. (사람과의 협업 시 힘을
    제한하는 안전 응용에 사용)

    - SetPowerLimit(status, power): status=1 켜짐, power=최대 허용 파워[W]
    - GetJointTorques(): 현재 관절 토크 조회
    - ServoJTStart()/ServoJT(torque, interval)/ServoJTEnd(): 실시간 토크
      스트리밍 (ServoJ가 위치를 스트리밍하듯, 토크값을 계속 흘려보냄)
    """
    robot.DragTeachSwitch(state=1)
    error = robot.SetPowerLimit(status=1, power=2)
    print("SetPowerLimit 반환값:", error)
    error, joint_torque = robot.GetJointTorques()
    print("현재 관절 토크:", joint_torque)

    robot.ServoJTStart()
    for _ in range(10):
        robot.ServoJT(torque=joint_torque, interval=0.001)
        time.sleep(0.01)
    robot.ServoJTEnd()
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    set_collision_level(robot)
    set_collision_strategy(robot)
    set_joint_limits(robot)
    # set_friction_compensation(robot, mount="level")
    # joint_power_limit_and_torque_servo(robot)

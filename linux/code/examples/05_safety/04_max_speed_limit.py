"""
카테고리: 안전(Safety) - 최대 속도 제한(안전 속도 파라미터)
------------------------------------------------------------
TCP(공구 끝) 속도와 각 관절 속도에 절대적인 상한선을 걸어두는 기능.
사람과 협업하는 구역에서 로봇이 아무리 빠른 vel(%) 값을 받아도
일정 속도 이상은 절대 넘지 않도록 강제할 때 사용합니다.

원본 출처: example/TestSetVelReducePara.py, example/TestSetJointVelReducePara.py

⚠️ 원본 예제의 주석은 각 파라미터 의미를 추측(물음표)으로 적어뒀는데,
Robot.py의 실제 @brief 문서를 확인해 정확한 의미로 정정했습니다.
"""
from fairino import Robot
import time


def set_max_speed_limit(robot, enable=1, max_tcp_vel=30, strategy=1,
                         max_joint_vel=None):
    """
    안전 속도 파라미터 설정.
    - enable: 0=끔, 1=수동모드에서만 적용, 2=모든 모드에서 적용
      (2는 자동 감속 미지원 조합에 주의 - 컨트롤러 매뉴얼 확인)
    - max_tcp_vel: 허용 최대 TCP 속도 [mm/s], 범위 0~1000
    - strategy: 속도 초과 시 정책. 0=정지+경고, 1=자동으로 속도 제한,
      2=정지+경고 후 로봇 비활성화(하사용)
    - max_joint_vel: [j1..j6] 각 관절 최대 각속도 [°/s], 기본 전부 45.0
    """
    max_joint_vel = max_joint_vel or [45.0] * 6
    error = robot.SetVelReducePara(enable, max_tcp_vel, strategy, max_joint_vel)
    print("SetVelReducePara 반환값:", error)
    return error


def demo_speed_limit_effect(robot):
    """
    속도 제한을 껐을 때와 켰을 때 같은 MoveJ(vel=100)의 실제 이동 속도가
    달라지는 것을 비교하는 예시.
    """
    j1 = [0, -90, 90, 0, 0, 0]
    j2 = [90, -90, 90, 0, 0, 0]

    robot.SetSpeed(80)

    print("--- 속도 제한 끔 ---")
    set_max_speed_limit(robot, enable=0, max_tcp_vel=30, strategy=1)
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100)

    print("--- 속도 제한 켬 (수동모드, 자동 감속) ---")
    set_max_speed_limit(robot, enable=1, max_tcp_vel=30, strategy=1)
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100)

    time.sleep(2)
    robot.ResetAllError()
    robot.RobotEnable(1)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    demo_speed_limit_effect(robot)

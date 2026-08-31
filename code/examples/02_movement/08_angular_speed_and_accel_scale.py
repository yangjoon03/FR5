"""
카테고리: 이동(Move) - 자세(회전) 속도 지정 & 가속도 스케일
------------------------------------------------------------
MoveL의 vel 파라미터는 기본적으로 "위치(직선 이동)" 속도를 의미합니다.
그런데 자세(rx,ry,rz 회전)가 위치보다 훨씬 크게 바뀌는 이동(예: 제자리에서
손목만 90도 돌리는 동작)에서는, 위치 속도 기준만으로는 자세가 너무 빠르게
홱 돌아가 버릴 수 있습니다. AngularSpeedStart는 그 구간 동안 "자세 회전
속도"를 별도 비율로 지정해서 더 부드럽게 만드는 기능입니다.

- AngularSpeedStart(ratio): 자세 속도 비율 [0~300%] 지정 시작
- AngularSpeedEnd(): 종료 (원래 방식으로 복귀)
- SetOaccScale(acc): 회전(자세) 가속도 비율을 설정 (선형 가속도와 별개로
  자세 가속만 조절하고 싶을 때)

원본 출처: example/TestAngular.py
"""
from fairino import Robot


def set_orientation_accel_scale(robot, acc_percent=100):
    """자세(회전) 가속도 비율을 설정. acc_percent: 0~100%."""
    error = robot.SetOaccScale(acc_percent)
    print("SetOaccScale 반환값:", error)
    return error


def move_with_angular_speed(robot, desc_pos, tool=7, user=0, vel=10.0, angular_ratio=50):
    """
    자세 속도를 지정한 상태로 MoveL 실행.
    - angular_ratio: 자세 회전 속도 비율 [0~300%] (100 초과 시 기본보다 더 빠르게 회전)
    """
    error = robot.AngularSpeedStart(angular_ratio)
    print("AngularSpeedStart 반환값:", error)

    error = robot.MoveL(desc_pos, tool=tool, user=user, vel=vel)
    print("MoveL 반환값:", error)

    robot.AngularSpeedEnd()
    return error


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    DP = [-29.155, -580.477, 25.884, -44.795, -2.374, -172.261]
    set_orientation_accel_scale(robot, 100)
    move_with_angular_speed(robot, DP, tool=7, user=0, vel=10.0, angular_ratio=50)

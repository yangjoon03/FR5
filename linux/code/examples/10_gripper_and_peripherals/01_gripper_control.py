"""
카테고리: 그리퍼/말단장치 (Gripper & Peripherals)
------------------------------------------------------------
로봇 손목에 다는 전동 그리퍼(집게)를 설정, 활성화, 움직이고, 물건을
놓쳤는지(낙하 감지) 확인하는 기능들.

원본 출처: example/TestPeripheralsCommand.py,
           example/TestGripperWaitMotionDone.py,
           example/GripperDropAlarmTest.py
"""
from fairino import Robot
import time


def gripper_config(robot, company=3, device=0):
    """
    그리퍼 통신 설정.
    - company: 그리퍼 제조사 코드
    - device: 장치 번호
    """
    error = robot.SetGripperConfig(company=company, device=device)
    print("SetGripperConfig 반환값:", error)
    time.sleep(1)
    error, config = robot.GetGripperConfig()
    print("그리퍼 설정값:", config)
    return error


def gripper_activate_and_move(robot, index=1):
    """
    그리퍼 활성화 후 특정 위치로 이동.
    - ActGripper(index, action): action 0=리셋, 1=활성화
    - MoveGripper(index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque):
      pos=목표 위치(0~100, 열림~닫힘 비율), vel=속도%, force=파지력%,
      maxtime=최대 대기시간[ms], block=0 블로킹/1 논블로킹,
      type=0 위치제어/그 외 회전형 그리퍼 모드,
      rotNum/rotVel/rotTorque=회전형 그리퍼일 때 회전수/속도/토크
    - GetGripperMotionDone(): 그리퍼 동작 완료 여부 조회
    """
    robot.ActGripper(index=index, action=0)
    time.sleep(1)
    robot.ActGripper(index=index, action=1)
    time.sleep(2)

    error = robot.MoveGripper(index=index, pos=20, vel=50, force=50, maxtime=30000,
                               block=0, type=0, rotNum=0, rotVel=0, rotTorque=0)
    print("MoveGripper 반환값:", error)
    time.sleep(3)

    error, state = robot.GetGripperMotionDone()
    print("그리퍼 동작 완료 상태:", state)
    return error


def gripper_wait_motion_done(robot, index=1):
    """
    그리퍼 동작이 끝날 때까지 대기 (블로킹 방식 대안).
    - GripperWaitMotionDone(status, timeout, strategy, type):
      status=대기할 완료상태, timeout=최대 대기시간[ms],
      strategy=타임아웃 처리방식, type=그리퍼 종류

    robot_state_pkg.gripper_motiondone 값 의미:
      0=아직 동작 중, 1=완료(물체 미검출), 2=완료(물체 검출됨/파지 성공)
    """
    robot.MoveGripper(index, 0, 100, 100, 30000, 0, 0, 0, 0, 0)  # 열기
    time.sleep(4)
    robot.MoveGripper(index, 90, 100, 100, 30000, 0, 0, 0, 0, 0)  # 닫기

    rtn = robot.GripperWaitMotionDone(index, 30000, 0, 0)
    print("GripperWaitMotionDone 반환값:", rtn)
    return rtn


def compute_pick_poses(robot, desc_pos, z_length=10, z_angle=0):
    """
    파지(pick) 지점 기준으로 "접근 전 위치(prepick)"와 "빠져나올 위치
    (postpick)"를 자동 계산합니다. 보통 물체 바로 위/아래로 z_length[mm]
    만큼 떨어진 지점을 계산해줍니다 (충돌 없이 접근/후퇴하기 위함).
    - ComputePrePick(desc_pos, zlength, zangle): 파지 전 접근 위치 계산
    - ComputePostPick(desc_pos, zlength, zangle): 파지 후 후퇴 위치 계산
    """
    error, prepick = robot.ComputePrePick(desc_pos=desc_pos, zlength=z_length, zangle=z_angle)
    print("접근 위치(prepick):", prepick)
    error, postpick = robot.ComputePostPick(desc_pos=desc_pos, zlength=z_length, zangle=z_angle)
    print("후퇴 위치(postpick):", postpick)
    return prepick, postpick


def check_workpiece_drop(robot):
    """
    파지 중 물건을 놓쳤는지(낙하) 감지하는 예시.
    robot_state_pkg.gripper_fault 값: 0=정상, 1=485 통신 타임아웃,
    2=명령 오류, 3=공작물 낙하, 그 외=그리퍼 자체 고장코드
    """
    error, pkg = robot.GetRobotRealTimeState()
    if pkg.gripper_fault == 3:
        print("⚠ 경고: 파지 중이던 물건을 놓쳤습니다 (낙하 감지)")
        robot.ResetAllError()
    else:
        print("그리퍼 상태 정상, gripper_fault =", pkg.gripper_fault)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    gripper_config(robot)
    gripper_activate_and_move(robot)
    check_workpiece_drop(robot)

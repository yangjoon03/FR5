"""
카테고리: 고급 궤적 - 궤적 파일 업로드/재생 (TrajectoryJ, TPD)
------------------------------------------------------------
사람이 직접 좌표를 하나하나 입력하는 대신, 미리 만들어둔 궤적 파일
(관절각을 촘촘히 나열한 텍스트 파일 등)을 컨트롤러에 업로드해서
그대로 재생하는 기능입니다. 3D 스캔 경로, CAM 소프트웨어가 뽑아낸
경로, 손으로 그린 로고 궤적 등을 재현할 때 사용합니다.

TrajectoryJ 흐름: 업로드 -> 로드 -> 시작점으로 이동 -> 재생 -> (재생 중
                 속도 실시간 조절 가능) -> 완료 대기
TPD(Teach Playback Data)는 힘/토크 값까지 함께 기록해둔 확장 버전으로,
용접/폴리싱처럼 위치뿐 아니라 힘도 재현해야 하는 작업에 사용합니다.

원본 출처: example/TestUploadTrajectoryJ.py, example/TestSetTrajectoryJSpeed.py,
           example/TestTPDCommand.py, example/Version_compatibility.py
"""
from fairino import Robot
import time


def upload_and_play_trajectory(robot, local_file="./horse.txt", traj_name="horse.txt"):
    """
    궤적 파일을 업로드하고 처음부터 끝까지 재생.
    - TrajectoryJUpLoad(local_path): 로컬 파일을 컨트롤러로 업로드
    - LoadTrajectoryJ(name, ovl, opt): 업로드된 파일을 로드.
      ovl=속도 백분율, opt=1이면 반복 재생
    - GetTrajectoryStartPose(name): 궤적의 첫 좌표를 가져와, 먼저 그
      지점으로 안전하게 이동한 뒤 재생을 시작해야 함
    - GetTrajectoryPointNum(): 궤적에 포함된 총 점 개수
    - MoveTrajectoryJ(): 실제 재생 시작 (논블로킹 - 완료는 motion_done으로 확인)
    """
    robot.TrajectoryJUpLoad(local_file)
    error = robot.LoadTrajectoryJ(name=traj_name, ovl=100, opt=1)
    print("LoadTrajectoryJ 반환값:", error)

    error, start_pose = robot.GetTrajectoryStartPose(name=traj_name)
    print("궤적 시작좌표:", start_pose)

    robot.SetSpeed(50)
    robot.MoveCart(desc_pos=start_pose, tool=0, user=0, vel=100, acc=100, ovl=100, blendT=-1, config=-1)

    error, point_num = robot.GetTrajectoryPointNum()
    print("궤적 점 개수:", point_num)

    error = robot.MoveTrajectoryJ()
    print("MoveTrajectoryJ 반환값:", error)


def adjust_speed_while_playing(robot):
    """
    재생 도중 실시간으로 속도를 바꾸는 예시. 재생이 끝났는지는
    `GetRobotRealTimeState().motion_done`으로 폴링해서 확인합니다.
    - SetTrajectoryJSpeed(ovl, mode) / SetTrajectoryJSpeed1(ovl) /
      SetTrajectoryJSpeed3(ovl, mode, status): 버전별 속도 조절 함수
      (신규 프로젝트에서는 SetTrajectoryJSpeed 사용을 권장)
    """
    while True:
        error, pkg = robot.GetRobotRealTimeState()
        if pkg.motion_done != 0:
            print("재생 완료")
            break
        robot.SetTrajectoryJSpeed(ovl=10.0, mode=0)
        time.sleep(3)
        robot.SetTrajectoryJSpeed(ovl=80.0, mode=0)
        time.sleep(3)


def delete_trajectory(robot, traj_name="horse.txt"):
    """업로드된 궤적 파일을 컨트롤러에서 삭제."""
    error = robot.TrajectoryJDelete(traj_name)
    print("TrajectoryJDelete 반환값:", error)
    return error


def tpd_playback_demo(robot, tpd_name="polish1"):
    """
    TPD(힘/토크 포함 궤적) 재생 개요.
    - SetTPDStart(name) / MoveTPD() / SetTPDParam(...) / SetTPDDelete(name)
    - SetTrajectoryJForceFx/Fy/Fz/TorqueTx/Ty/Tz: 재생 중 각 축 방향의
      목표 힘/토크를 실시간으로 조절 (표면을 누르는 힘 등을 재생 중 변경)
    - LoadTPD(name) / GetTPDStartPose(name): TrajectoryJ와 대응되는 로드/시작좌표 조회
    """
    robot.DragTeachSwitch(1)  # 드래그 티칭으로 직접 궤적+힘을 기록하는 것이 일반적인 사용법
    # ... 사람이 로봇을 잡고 움직이며 SetTPDStart로 기록 ...
    robot.DragTeachSwitch(0)

    error = robot.LoadTPD(tpd_name)
    print("LoadTPD 반환값:", error)
    error, start_pose = robot.GetTPDStartPose(tpd_name)
    print("TPD 시작좌표:", start_pose)

    robot.MoveCart(start_pose, tool=0, user=0)
    error = robot.MoveTPD()
    print("MoveTPD 반환값:", error)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    print("실제 궤적 파일이 컨트롤러에 준비된 경우에만 아래 함수들을 실행하세요.")
    # upload_and_play_trajectory(robot)
    # adjust_speed_while_playing(robot)

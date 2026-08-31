"""
카테고리: 고급 궤적/서보 (Advanced Trajectory & Servo)
------------------------------------------------------------
02_movement/04_move_servo_realtime.py의 ServoJ/ServoCart보다 더 저수준인
실시간 스트리밍 제어 방식들입니다.

- ServoJMultiPos : ServoJ를 "한 점씩"이 아니라 "여러 점을 한 번에 묶어서"
  전송. 통신 횟수를 줄여 더 매끄러운 고주파 궤적 재생에 유리합니다.
- ServoJV       : 위치가 아니라 "관절 속도"를 직접 지령하는 속도제어 서보.
  (ServoJ는 "이 각도로 가라", ServoJV는 "이 속도로 계속 움직여라")
- ServoMIT      : 위치+속도+토크를 함께 지령하는 임피던스 제어형 서보
  (MIT 방식 - 위치오차*posGain + 속도오차*velGain + 순수토크를 합산해서
  최종 토크 지령을 만듦). 사람이 밀면 부드럽게 반응하는 협동 제어에 사용.

원본 출처: example/ServoJMultiPos_test.py, example/ServoJVtest.py,
           example/ServoMITtest.py
"""
from fairino import Robot
import time


def servo_j_multi_pos_demo(robot, joint_pos_list):
    """
    여러 관절 목표점을 한 번의 호출로 전송.
    - joint_pos_list: [[j1..j6], [j1..j6], ...] 형태의 목표점 리스트
    - axisPos: 외부축 위치 [0,0,0,0]
    - 반환값: (error, servo_j_cmd_count) - 실제로 큐에 들어간 명령 개수

    로봇 내부 명령 큐(mc_queue_len)가 가득 차면 실패하므로, 아래처럼
    `GetRobotRealTimeState().mc_queue_len`을 보고 여유가 있을 때만
    보내는 것이 안전합니다.
    """
    rtn, pkg = robot.GetRobotRealTimeState()
    room = max(0, 50 - pkg.mc_queue_len)
    batch = joint_pos_list[:min(room, 10)]
    if not batch:
        print("명령 큐가 가득 참, 잠시 대기 필요")
        return None

    error, count = robot.ServoJMultiPos(batch, [0.0, 0.0, 0.0, 0.0],
                                         acc=100.0, vel=100.0, cmdT=0.008,
                                         filterT=0.008, gain=1.0, id=0, cmdType=0)
    print(f"ServoJMultiPos: {len(batch)}개 전송, 반환값={error}, 큐카운트={count}")
    return error


def servo_jv_demo(robot, cycles=200):
    """
    관절 속도 직접 제어. joint_vel[0]=10.0은 "1번 관절을 계속 10°/s로
    회전시켜라"는 뜻입니다. 목표 각도가 없으므로 정지시키려면 다시
    0으로 채운 속도를 보내야 합니다.
    """
    joint_vel = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    exis_vel = [0.0, 0.0, 0.0, 0.0]
    for _ in range(cycles):
        error = robot.ServoJV(joint_vel=joint_vel, exis_vel=exis_vel, acc=0.0, vel=0.0,
                               cmdT=0.008, filterT=0.0, gain=0.0, id=0, comType=1)
        time.sleep(0.01)
    print("ServoJV 스트리밍 종료, 마지막 반환값:", error)


def servo_mit_demo(robot):
    """
    MIT 방식 임피던스 서보. 6번 관절에 아주 작은 토크(0.03)만 계속
    걸어서, 마치 사람이 살짝 미는 것처럼 관절이 천천히 돌아가게 하는 예시.
    - posGain/desPos: 목표 위치 및 위치 게인 (0으로 두면 위치 추종 안 함)
    - velGain/desVel: 목표 속도 및 속도 게인
    - torques: 각 관절에 추가로 더할 순수 토크값
    - ServoMITStart(1)/ServoMITEnd(1): 시작/종료
    """
    robot.ServoMITStart(1)
    robot.DragTeachSwitch(1)

    torques = [0.0] * 6
    pos_gain = [0.0] * 6
    des_pos = [0.0] * 6
    vel_gain = [0.0] * 6
    des_vel = [0.0] * 6

    torques[5] = 0.03
    while True:
        robot.ServoMIT(pos_gain, des_pos, vel_gain, des_vel, torques, 0.008, comType=1)
        time.sleep(0.001)
        error, pkg = robot.GetRobotRealTimeState()
        if pkg.jt_cur_pos[5] > 30:
            break

    robot.DragTeachSwitch(0)
    robot.ServoMITEnd(1)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    # servo_jv_demo(robot, cycles=50)
    # servo_mit_demo(robot)
    print("실제 궤적 파일/장비 없이는 데모 실행을 생략합니다.")

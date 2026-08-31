"""
카테고리: 시스템/버전 (System & Version)
------------------------------------------------------------
컨트롤러 시간 동기화, SDK/펌웨어 버전 확인, 소프트웨어 업그레이드,
전역 시스템 변수, 포인트 테이블(좌표 세트) 전환 등 "장비 관리" 성격의
기능들.

원본 출처: example/gettime.py, example/testSetAndGetRobotTime.py,
           example/softwareupgrade.py, example/TestSetCommand.py(일부),
           example/TestPointTableCommand.py
"""
from fairino import Robot
import time
from datetime import datetime, timedelta


def read_realtime_clock(robot, cycles=5):
    """
    robot_state_pkg에 들어있는 컨트롤러의 현재 날짜/시간 필드를 읽습니다.
    (year, mouth[원문 오타 그대로 유지된 필드명], day, hour, minute,
    second, millisecond)
    """
    for _ in range(cycles):
        p = robot.robot_state_pkg
        print(f"{p.year}-{p.mouth:02d}-{p.day:02d} {p.hour:02d}:{p.minute:02d}:{p.second:02d}.{p.millisecond}")
        time.sleep(0.5)


def sync_robot_time_to_pc(robot):
    """
    컨트롤러 시간을 PC(SDK를 실행 중인 컴퓨터) 시간에 맞춰 동기화.
    - GetSystemClock(): 컨트롤러의 현재 시각(ms, UTC+8 기준 저장)
    - SetRobottime(): 현재 PC 시간으로 컨트롤러 시간을 덮어씀
    """
    error, t_ms = robot.GetSystemClock()
    before = datetime.fromtimestamp(t_ms / 1000.0) - timedelta(hours=8)
    print("동기화 전 로봇 시간:", before)

    error = robot.SetRobottime()
    print("SetRobottime 반환값:", error)

    error, t_ms_after = robot.GetSystemClock()
    after = datetime.fromtimestamp(t_ms_after / 1000.0) - timedelta(hours=8)
    print("동기화 후 로봇 시간:", after, "/ PC 시간:", datetime.now())


def software_upgrade(robot, package_path="./software.tar.gz", block=False):
    """
    컨트롤러 소프트웨어(펌웨어) 업그레이드.
    - SoftwareUpgrade(filePath, block): block=True면 업그레이드가 끝날 때까지
      이 함수 호출이 대기(블로킹), False면 즉시 반환(논블로킹)
    - GetSoftwareUpgradeState(): 진행률(0~100)을 폴링
    ⚠️ 실제 로봇의 펌웨어를 변경하는 작업이므로, 반드시 제조사가 배포한
    정식 패키지로만 실행하고 업그레이드 중 전원을 차단하지 마세요.
    """
    error = robot.SoftwareUpgrade(package_path, block)
    print("SoftwareUpgrade 반환값:", error)
    while robot.GetSoftwareUpgradeState() != 100:
        print("업그레이드 진행률:", robot.GetSoftwareUpgradeState())
        time.sleep(0.5)


def system_variable_demo(robot, var_id=1):
    """
    컨트롤러 내부의 전역 시스템 변수를 읽고 씁니다. Lua 프로그램과 값을
    주고받는 통로로 자주 사용됩니다 (예: PC에서 변수를 세팅하면 로봇
    프로그램이 그 값을 읽어 분기 처리).
    """
    error = robot.SetSysVarValue(id=var_id, value=10)
    print("SetSysVarValue 반환값:", error)
    error, value = robot.GetSysVarValue(id=var_id)
    print(f"시스템 변수[{var_id}] 값:", value)


def point_table_demo(robot, table_name="point_table_test1.db"):
    """
    포인트 테이블: 미리 저장해둔 좌표 세트를 통째로 업/다운로드하거나
    전환하는 기능. 같은 로봇 프로그램(Lua)을 여러 제품 모델에 재사용할 때,
    좌표만 담긴 테이블을 바꿔 끼우는 방식으로 활용합니다.
    - PointTableUpLoad(path): PC의 포인트 테이블 파일을 컨트롤러로 업로드
    - PointTableDownLoad(name, save_path): 컨트롤러의 테이블을 PC로 다운로드
    - PointTableSwitch(name): 현재 사용할 테이블 전환
    - PointTableUpdateLua(name, lua_file_name): 테이블 변경사항을 지정한
      Lua 프로그램에 반영(재컴파일)
    """
    robot.PointTableUpLoad(point_table_file_path=f"./{table_name}")
    error = robot.PointTableSwitch(point_table_name=table_name)
    print("PointTableSwitch 반환값:", error)


def robot_install_position(robot, method=0):
    """
    로봇 설치 방식(고정/이동체 위 설치 등) 설정. 07_status_and_log의
    install angle(기울기)과 달리, 이건 "고정 설치인지 아닌지"의 방식 자체.
    - method: 0=고정 설치, 그 외=제조사 정의값(이동체 탑재 등)
    """
    error = robot.SetRobotInstallPos(method=method)
    print("SetRobotInstallPos 반환값:", error)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    read_realtime_clock(robot, cycles=2)
    system_variable_demo(robot)
    # sync_robot_time_to_pc(robot)
    # software_upgrade(robot)  # 실제 업그레이드 패키지가 있을 때만 실행

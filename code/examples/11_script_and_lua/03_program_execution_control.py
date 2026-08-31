"""
카테고리: 스크립트/Lua - 사용자 프로그램(Lua) 실행 제어
------------------------------------------------------------
티치펜던트에서 "프로그램 실행/일시정지/재개/정지" 버튼을 누르는 것과
동일한 동작을 SDK에서 직접 제어하는 기능들. 01/02번 파일의 Lua는
"장치를 확장"하는 용도였다면, 이건 "사용자가 작성한 작업 순서(.lua
프로그램) 자체"를 실행/제어하는 것입니다.

원본 출처: example/TestWebAppCommand.py
"""
from fairino import Robot
import time


def load_and_run_program(robot, program_path="/fruser/testPTP.lua"):
    """
    프로그램 로드 후 실행.
    - ProgramLoad(program_name): 컨트롤러에 저장된 .lua 프로그램 경로 지정
    - GetLoadedProgram(): 현재 로드된 프로그램 이름 조회
    - ProgramRun(): 실행 시작 (자동 모드에서만 동작, Mode(0) 먼저 필요)
    """
    robot.Mode(state=0)
    robot.ProgramLoad(program_name=program_path)
    error, name = robot.GetLoadedProgram()
    print("로드된 프로그램:", name)
    robot.ProgramRun()


def set_boot_autorun_program(robot, program_path="/fruser/1010Test.lua"):
    """컨트롤러 부팅 시 자동으로 로드할 기본 프로그램을 지정."""
    error = robot.LoadDefaultProgConfig(flag=0, program_name=program_path)
    print("LoadDefaultProgConfig 반환값:", error)


def monitor_program_state(robot, duration_s=2.0):
    """
    실행 중인 프로그램의 현재 줄 번호와 상태를 폴링.
    - GetCurrentLine(): 현재 실행 중인 줄 번호
    - GetProgramState(): 프로그램 상태(정지/실행/일시정지 등)
    """
    end = time.time() + duration_s
    while time.time() < end:
        error, line = robot.GetCurrentLine()
        error, state = robot.GetProgramState()
        print(f"현재 줄: {line}, 상태: {state}")
        time.sleep(0.1)


def pause_resume_stop_demo(robot):
    """
    실행 중인 프로그램을 일시정지/재개를 5회 반복한 뒤 정지.
    - PauseMotion() / ResumeMotion() / StopMotion()
    """
    for _ in range(5):
        robot.PauseMotion()
        time.sleep(2)
        robot.ResumeMotion()
        time.sleep(2)
    error = robot.StopMotion()
    print("StopMotion 반환값:", error)


def lua_file_management(robot):
    """
    프로그램(.lua) 파일 자체를 업로드/다운로드/삭제/목록 조회.
    (11_script_and_lua/02와 다른 점: 여기서는 "제어 로직 확장용" Lua가
    아니라 "사용자가 짠 작업 프로그램" 파일을 다룸)
    """
    robot.LuaUpload(filePath="./program.lua")
    robot.LuaDownLoad(fileName="program.lua", savePath="./downloaded/")
    error, count, names = robot.GetLuaList()
    print(f"업로드된 Lua 프로그램 {count}개:", names)
    # robot.LuaDelete(fileName="program.lua")


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    lua_file_management(robot)
    # load_and_run_program(robot)
    # monitor_program_state(robot)
    # pause_resume_stop_demo(robot)

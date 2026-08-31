"""
카테고리: 스크립트/Lua - 말단(End/Axle) 장치용 Lua 개방 프로토콜
------------------------------------------------------------
서드파티 그리퍼/힘센서 등 "말단 장치"를 로봇 컨트롤러가 표준으로
지원하지 않을 때, 제조사가 제공하는 Lua 스크립트를 컨트롤러에 업로드해서
통신 프로토콜을 직접 해석하게 만드는 기능입니다. (일종의 플러그인)

원본 출처: example/TestLuaEndCommand.py, example/EndLua.py
"""
from fairino import Robot
import time


def upload_end_axle_lua(robot, file_path="./AXLE_LUA_End_DaHuan_WeiHang.lua"):
    """
    말단 장치용 Lua 프로토콜 파일을 컨트롤러에 업로드합니다.
    보통 그리퍼/센서 제조사가 자기 장비에 맞는 .lua 파일을 제공합니다.
    """
    robot.LoggerInit(output_model=0)
    robot.SetLoggerLevel(lvl=4)
    error = robot.AxleLuaUpload(filePath=file_path)
    print("AxleLuaUpload 반환값:", error)
    return error


def configure_axle_communication(robot):
    """
    말단 장치와의 RS485 등 시리얼 통신 파라미터 설정/조회.
    - SetAxleCommunicationParam(baudRate, dataBit, stopBit, verify(패리티),
      timeout, timeoutTimes, period): 통신 규격 설정
    """
    error = robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1000)
    print("SetAxleCommunicationParam 반환값:", error)
    error, baud, data_bit, stop_bit, verify, timeout, timeout_times, period = robot.GetAxleCommunicationParam()
    print("현재 통신 파라미터:", baud, data_bit, stop_bit, verify, timeout, timeout_times, period)


def enable_axle_lua(robot, gripper=1, force_sensor=0, io=0):
    """
    업로드한 Lua 스크립트 실행을 활성화하고, 어떤 장치 종류로 동작할지 지정.
    - SetAxleLuaEnable(1): Lua 실행 활성화
    - SetAxleLuaEnableDeviceType(forceSensorEnable, gripperEnable, IOEnable):
      이 Lua가 힘센서/그리퍼/IO 중 무엇으로 동작할지 지정 (0/1)
    """
    robot.SetAxleLuaEnable(1)
    error, enabled = robot.GetAxleLuaEnableStatus(enable=0)
    print("Lua 실행 활성화 상태:", enabled)

    robot.SetAxleLuaEnableDeviceType(force_sensor, gripper, io)
    error, fs, gr, io_en = robot.GetAxleLuaEnableDeviceType()
    print("장치 타입 - 힘센서:", fs, "그리퍼:", gr, "IO:", io_en)


def use_lua_gripper(robot):
    """
    Lua로 등록된 그리퍼를 표준 그리퍼 함수(ActGripper/MoveGripper)로 그대로
    제어할 수 있음을 보여주는 예시. (Lua는 "통신 방식을 알려주는 역할"만
    하고, 실제 제어 인터페이스는 표준 그리퍼 API와 동일)
    """
    robot.ActGripper(1, 0)
    time.sleep(1)
    error = robot.MoveGripper(1, 50, 100, 100, 5000, 0)
    print("MoveGripper 반환값:", error)


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)

    upload_end_axle_lua(robot)
    configure_axle_communication(robot)
    enable_axle_lua(robot, gripper=1)
    # use_lua_gripper(robot)

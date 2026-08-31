"""
카테고리: 스크립트/Lua - 컨트롤러 개방(Ctrl Open) Lua 스크립트
------------------------------------------------------------
말단 장치용 Lua(01_end_axle_lua.py)와 달리, 이건 로봇 "컨트롤러 자체"의
동작 로직을 사용자가 Lua 스크립트로 확장하는 기능입니다 (예: 특정
용접기 제어 로직, 특수 폴리싱 시퀀스 등을 컨트롤러 안에서 직접 실행).

흐름: 업로드 -> 이름 등록 -> 로드(실행 활성화) -> ... 사용 ... -> 언로드
     -> (필요시) 삭제

원본 출처: example/TestCtrlOpenLuaOperate.py
"""
from fairino import Robot
import time


def upload_and_download_lua(robot):
    """
    Lua 파일을 컨트롤러에 업로드하거나, 컨트롤러에 있는 파일을 PC로 내려받습니다.
    - OpenLuaUpload(local_path): PC의 .lua 파일을 컨트롤러로 업로드
    - OpenLuaDownload(remote_filename, local_dir): 컨트롤러의 파일을 PC로 다운로드
    """
    robot.OpenLuaUpload("./CtrlDev_WELDING_A.lua")
    robot.OpenLuaUpload("./CtrlDev_SWDPOLISH.lua")

    robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "./downloaded/")
    robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "./downloaded/")


def register_and_load_lua(robot):
    """
    업로드된 Lua 파일을 "슬롯 번호"에 등록하고 실행(로드)합니다.
    - SetCtrlOpenLUAName(slot, filename): slot(0~3 등)에 파일 등록
    - GetCtrlOpenLUAName(): 각 슬롯에 등록된 파일명 조회
    - LoadCtrlOpenLUA(slot): 등록된 슬롯의 Lua를 실제로 로드/실행
    - UnloadCtrlOpenLUA(slot): 실행 중지
    """
    robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua")
    robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua")

    error, names = robot.GetCtrlOpenLUAName()
    print("등록된 Lua 파일들:", names)

    robot.LoadCtrlOpenLUA(1)
    time.sleep(2)
    robot.UnloadCtrlOpenLUA(1)


def delete_lua_files(robot):
    """
    - OpenLuaDelete(filename): 특정 파일 삭제
    - AllOpenLuaDelete(): 업로드된 모든 Lua 파일 삭제
    """
    robot.OpenLuaDelete("CtrlDev_WELDING_A.lua")
    robot.AllOpenLuaDelete()


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")

    upload_and_download_lua(robot)
    register_and_load_lua(robot)
    # delete_lua_files(robot)

    time.sleep(1)
    robot.CloseRPC()

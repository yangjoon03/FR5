"""
카테고리: 용접(Welding) - 심화 기능 요약 (아크 추적 / 재용접 / 레이저 추적 / 스플라인 위빙)
------------------------------------------------------------
아래 기능들은 용접 장비(레이저 센서, 특정 용접기 모델 등) 사양에
크게 의존하는 고급 기능이라, 파라미터 하나하나까지 완전히 일반화하기
어렵습니다. 여기서는 "이 기능이 왜 있고, 어떤 함수들로 구성되는지"
흐름 위주로 정리했습니다. 실제 적용 시 장비 매뉴얼과 함께 보세요.

원본 출처: example/ArcWeldTrace.py, example/TestReWeld.py,
           example/TestSplineWeave.py, example/TestOriginPointWeave.py,
           example/TestWeaveSpeedAndOffset.py,
           example/TestLaserRecordReplayExaxisWithWave.py,
           example/testLaserWeld.py
"""
from fairino import Robot


# ------------------------------------------------------------------
# 1. 아크 추적(Arc Weld Trace) - 용접 중 실시간으로 이음매를 따라가기
# ------------------------------------------------------------------
def arc_seam_tracking(robot):
    """
    용접 중 아크 센서 신호로 이음매(seam) 위치 오차를 실시간 보정하며
    따라가는 기능. ArcWeldTraceControl(flag, ...)로 켜고 끄며, 그 사이의
    이동 명령이 자동으로 좌우/상하 보정을 받습니다.
    - ArcWeldTraceControl(flag, ...): flag=1 시작/0 종료, 나머지 파라미터는
      감도/보정속도/범위 등 (제조사 튜닝값, 매뉴얼 참고)
    - MultilayerOffsetTrsfToBase(origin, x_ref, z_ref, dx, dy, dz):
      다층 용접(여러 겹 쌓는 용접)에서, 원점/X축기준점/Z축기준점 3점으로
      정의한 국소좌표계 기준 오프셋(dx,dy,dz)을 베이스좌표계 오프셋값으로
      변환. 매 층(layer)마다 같은 궤적을 조금씩 밀어서 재사용할 때 사용.
    - ArcWeldTraceReplayStart/End(): 한 번 배운(기록한) 보정 궤적을 다시
      재생(replay)하는 구간을 표시.
    """
    error = robot.ARCStart(1, 0, 3000)
    robot.WeaveStart(0)
    error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10)
    print("ArcWeldTraceControl(시작) 반환값:", error)

    # ... 이 사이에서 MoveL 등으로 용접선을 따라 이동 ...

    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10)
    robot.WeaveEnd(0)
    robot.ARCEnd(1, 0, 10000)


# ------------------------------------------------------------------
# 2. 아크 끊김 후 재용접 (Re-Weld After Break-off)
# ------------------------------------------------------------------
def reweld_after_arc_breakoff_setup(robot):
    """
    용접 중 아크가 갑자기 끊기는 것(단선, 순간정전 등)을 감지하고, 끊긴
    지점부터 자동으로 이어서 다시 용접을 시작하는 기능.
    - WeldingSetCheckArcInterruptionParam / Get*: 아크 끊김 판정 기준 설정/조회
    - WeldingSetReWeldAfterBreakOffParam / Get*: 재용접 시작 방식(끊긴 지점에서
      얼마나 뒤로 물러나 다시 시작할지 등) 설정/조회
    - WeldingStartReWeldAfterBreakOff(): 재용접 기능 활성화
    이 기능은 보통 좌표계 계산(ComputeWObjCoord 등)과 함께 쓰여, 끊긴
    지점의 실제 좌표를 역산해서 그 자리부터 다시 이어붙입니다.
    """
    robot.WeldingSetCheckArcInterruptionParam(1, 100, 3)  # 예시 파라미터 - 매뉴얼 참고
    robot.WeldingSetReWeldAfterBreakOffParam(1, 5.0)
    error = robot.WeldingStartReWeldAfterBreakOff()
    print("WeldingStartReWeldAfterBreakOff 반환값:", error)


# ------------------------------------------------------------------
# 3. 레이저 추적 용접 (Laser Tracking) / 원점 위빙
# ------------------------------------------------------------------
def laser_tracking_weld_setup(robot):
    """
    레이저 변위 센서로 이음매 위치를 실시간으로 읽으며 위빙 중심을 계속
    보정하는 기능 (아크 센서 대신 레이저 센서를 쓰는 버전).
    - LaserTrackingSensorConfig / LaserTrackingSensorSamplePeriod: 센서 통신/주기 설정
    - LaserTrackingLaserOnOff(1): 레이저 켜기
    - LaserTrackingTrackOnOff(1): 추적 보정 켜기
    - OriginPointWeaveStart/End: "원점 기준 위빙" - 위빙 중심 자체가
      매 사이클 조금씩 보정되는 고급 위빙 모드 시작/종료
    """
    robot.LaserTrackingSensorConfig(1, 0, 0, 1)  # 예시 - 실제 센서 사양에 맞게 조정
    robot.LaserTrackingSensorSamplePeriod(10)
    robot.LaserTrackingLaserOnOff(1)
    robot.LaserTrackingTrackOnOff(1)
    error = robot.OriginPointWeaveStart(0)
    print("OriginPointWeaveStart 반환값:", error)
    # ... MoveL 등으로 용접선 이동 ...
    robot.OriginPointWeaveEnd()
    robot.LaserTrackingTrackOnOff(0)
    robot.LaserTrackingLaserOnOff(0)


# ------------------------------------------------------------------
# 4. 레이저 용접(Laser Welding, 아크 대신 레이저로 접합)
# ------------------------------------------------------------------
def laser_welding_demo(robot):
    """
    레이저 용접기를 확장 DO로 켜고 파라미터(출력 등)를 설정하는 예시.
    - SetLaserWeldingParam: 출력/초점 등 용접 파라미터
    - SetLaserWeldingStartExtDoNum(DONum): 레이저 On/Off를 트리거할 DO 채널 지정
    - SetLaserWeldingStartEnd(1/0): 레이저 발진 시작/종료
    """
    robot.ExtDevLoadUDPDriver()
    robot.SetLaserWeldingStartExtDoNum(DONum=17)
    robot.SetLaserWeldingParam(power=80, focus=0)  # 예시 - 실제 파라미터명은 매뉴얼 참고
    error = robot.SetLaserWeldingStartEnd(1)
    print("레이저 용접 시작 반환값:", error)
    # ... MoveL 이동 ...
    robot.SetLaserWeldingStartEnd(0)


# ------------------------------------------------------------------
# 5. 스플라인 기반 위빙 (곡선 궤적 + 위빙 동시 적용)
# ------------------------------------------------------------------
def spline_weave_demo(robot):
    """
    06_move_spline.py의 NewSpline 경로 이동에 위빙을 겹쳐 적용하는 예시.
    직선이 아니라 곡면을 따라가는 용접 이음매에 위빙을 적용할 때 사용.
    - SetWeaveBackCenterConfig / GetWeaveBackCenterConfig: 위빙 중심 보정 설정/조회
    """
    p1 = [-104.846, 309.573, 336.647, 179.681, -0.419, -92.692]
    p2 = [-194.846, 309.573, 336.647, 179.681, -0.419, -92.692]

    robot.NewSplineStart(type=0)
    robot.WeaveStart(0)
    robot.NewSplinePoint(desc_pos=p1, tool=0, user=0, lastFlag=0)
    robot.NewSplinePoint(desc_pos=p2, tool=0, user=0, lastFlag=1)
    robot.WeaveEnd(0)
    robot.NewSplineEnd()


if __name__ == "__main__":
    robot = Robot.RPC("192.168.58.2")
    robot.RobotEnable(1)
    print("이 파일의 함수들은 실제 용접 장비/센서가 연결된 환경에서만 의미가 있습니다.")

# FR5 로봇 이동/활성화 함수 정리 (Move & Enable Function Reference)

`example/` 폴더에 있던 중국어 주석 예제 코드(`TestBasicCommand.py`,
`TestMotionCommand.py` 등)를 기능별(카테고리별)로 다시 정리하고,
모든 함수 설명을 한국어로 번역해서 `code/examples/` 아래에 새로 작성했습니다.

> ⚠️ 이 문서와 예제 코드는 **로봇이 연결되지 않은 상태에서** 소스코드
> (`code/fairino/Robot.py`)를 읽고 정리한 것입니다. 실제 로봇에 연결해서
> 동작 테스트는 하지 않았으니, 처음 실행할 때는 반드시 저속(`vel` 값을
> 낮게)으로, 충돌 반경을 확보한 상태에서 테스트하세요.

## 왜 정리했나

`example/` 폴더에는 날짜별로 이름 붙은 임시 테스트 파일(`test0602.py`,
`Test0104.py` 등)이 150개 넘게 쌓여 있고, 정작 각 기능(이동, 활성화 등)이
어떤 파일에 있는지 파악하기 어려웠습니다. 또한 원본 SDK(`Robot.py`)의
함수 설명이 전부 중국어(`@brief`, `@param` 주석)로만 되어 있어서 그대로는
읽기 어려웠습니다.

그래서 실제로 로봇을 "움직이는" 함수들과 "활성화/상태" 관련 함수들만
추려서, 이동 방식(직선/관절/원형/나선형/서보/스플라인/조그)별로 파일을
나누고 한국어 설명을 달았습니다.

## 폴더 구조

```
code/
├── fairino/              # SDK 본체 (Robot.py 등, 건드리지 않음)
├── examples/             # 이번에 새로 정리한 카테고리별 예제
│   ├── 01_connect_and_enable.py
│   ├── 02_move_joint.py
│   ├── 03_move_linear.py
│   ├── 04_move_arc_circle.py
│   ├── 05_move_servo_realtime.py
│   ├── 06_move_spline.py
│   ├── 07_move_jog.py
│   └── 08_move_points_offset.py
└── README.md             # 이 문서
```

각 파일은 단독으로 실행 가능한(`if __name__ == "__main__":`) 형태이며,
맨 위 IP(`192.168.58.2`)는 실제 컨트롤러 IP로 바꿔서 사용하면 됩니다.

## 공통 사용 순서

로봇을 실제로 움직이려면 항상 아래 순서를 지켜야 합니다.

1. **연결**: `robot = Robot.RPC("컨트롤러_IP")`
2. **에러 확인/초기화**: `robot.GetRobotErrorCode()` → 필요 시 `robot.ResetAllError()`
3. **활성화(Enable)**: `robot.RobotEnable(1)` — 이걸 안 하면 모든 Move 계열
   함수가 실패하거나 무시됩니다.
4. **이동(Move 계열 함수 호출)**

---

## 카테고리별 정리

### 1. 연결 / 활성화 / 상태 확인 — `01_connect_and_enable.py`

| 함수 | 설명 | 주요 파라미터 |
|---|---|---|
| `Robot.RPC(ip)` | 컨트롤러 접속, `robot` 객체 생성 | `ip`: 컨트롤러 IP |
| `is_conect` | 연결 상태 조회(속성) | - |
| `GetSDKVersion()` | SDK/로봇 펌웨어 버전 조회 | - |
| `GetControllerIP()` | 접속 중인 컨트롤러 IP 조회 | - |
| `RobotEnable(state)` | 로봇 상사용/하사용 전환 | `state`: 0=하사용(비활성화), 1=상사용(활성화) |
| `Mode(state)` | 자동/수동 모드 전환 | `state`: 0=자동, 1=수동 |
| `DragTeachSwitch(state)` | 드래그 티칭(핸드 가이딩) 진입/해제 | `state`: 0=해제, 1=진입 |
| `IsInDragTeach()` | 드래그 티칭 모드 여부 조회 | - |
| `GetRobotErrorCode()` | 현재 에러코드 조회 | 반환: `[maincode, subcode]` |
| `ResetAllError()` | 복구 가능한 에러 초기화 | - |

**활성화 관련 주의사항**
- `RobotEnable(1)`이 실패하면 비상정지 버튼, 안전문(Safety Door),
  펜던트의 안전 정지 상태(`GetSafetyCode()`)부터 확인해야 합니다.
- 자동 모드(`Mode(0)`)에서만 원격(PC)에서 보낸 Move 명령이 정상 동작하는
  경우가 많습니다. 수동 모드에서는 컨트롤러 설정에 따라 거부될 수 있습니다.

---

### 2. 관절 이동 — `02_move_joint.py`

| 함수 | 이동 방식 | 필수 파라미터 | 비고 |
|---|---|---|---|
| `MoveJ(joint_pos, tool, user, vel=20.0, ...)` | 관절 공간 이동 | `joint_pos`(목표 관절각 6개, °), `tool`, `user` | 경로가 직선이 아닐 수 있음. 가장 빠르고 부드러운 방식 |

- `joint_pos`: `[j1,j2,j3,j4,j5,j6]`, 단위 [°]
- `tool` / `user`: 툴/사용자(작업물) 좌표계 번호, 0~14
- `vel`: 속도 백분율 [0~100]
- `blendT`: -1.0=목표점 정지(블로킹) / 0~500(ms)=평활 연결(논블로킹)

---

### 3. 직선/점대점 이동 — `03_move_linear.py`

| 함수 | 이동 방식 | 필수 파라미터 | 비고 |
|---|---|---|---|
| `MoveL(desc_pos, tool, user, vel=20.0, blendR=-1.0, ...)` | 데카르트 직선 이동 | `desc_pos`(목표 좌표, mm/°), `tool`, `user` | 경로가 항상 직선 |
| `MoveCart(desc_pos, tool, user, vel=20.0, ...)` | 데카르트 점대점 이동 | 위와 동일 | 관절 보간, 특이점 회피에 유리, 경로는 직선 아닐 수 있음 |

- `desc_pos`: `[x,y,z,rx,ry,rz]`, 단위 [mm][°]
- `blendR`: -1.0=정지(블로킹) / 0~1000(mm)=평활 반경(논블로킹)
- `overSpeedStrategy`(MoveL 전용): 0=끔, 1=표준, 2=초과속도 시 에러정지,
  3=적응형 감속. `speedPercent`와 함께 사용.

---

### 4. 원호/원형/나선 이동 — `04_move_arc_circle.py`

| 함수 | 이동 방식 | 필수 파라미터 | 비고 |
|---|---|---|---|
| `MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, ...)` | 원호(호) 이동 | 경유점(`_p`) + 목표점(`_t`) 좌표/좌표계 | 현재위치→경유점→목표점을 지나는 호 |
| `Circle(desc_pos_p, ..., desc_pos_t, ...)` | 완전한 원(전체 원) 이동 | 위와 동일 | 한 바퀴를 다 돌아 시작 위치로 복귀. 충돌 반경 확인 필수 |
| `NewSpiral(desc_pos, tool, user, param, ...)` | 나선형(스파이럴) 이동 | `param`(7개 값, 아래 참고) | 반지름이 점점 커지거나 작아지는 원 궤적 |

`NewSpiral`의 `param`:
`[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]`

| 인덱스 | 이름 | 의미 |
|---|---|---|
| 0 | circle_num | 회전 횟수(원 개수) |
| 1 | circle_angle | 나선 기울기 각도 |
| 2 | rad_init | 초기 반지름 |
| 3 | rad_add | 회전마다 반지름 증가량 |
| 4 | rotaxis_add | 회전축 방향 증가량 |
| 5 | rot_direction | 0=시계방향, 1=반시계방향 |
| 6 | velAccMode | 0=각속도 일정, 1=선속도 일정 |

> **발견한 이슈**: `example/TestMotionCommand.py`의 `newspiral()`은
> `param`을 6개 값(`[5.0,10,30,10,5,0]`)으로만 호출하지만, 현재
> `code/fairino/Robot.py`의 `NewSpiral`은 `param[6]`(velAccMode)까지
> 읽으므로 **6개짜리 리스트를 넣으면 `IndexError`가 납니다.** 반드시
> 7번째 값(velAccMode)까지 채워서 호출하세요. (`04_move_arc_circle.py`에는
> 이미 7개로 수정 반영함)

---

### 5. 서보(실시간 스트리밍) 이동 — `05_move_servo_realtime.py`

| 함수 | 설명 |
|---|---|
| `ServoMoveStart(cmdType=0)` | 서보 이동 시작 (0=XML-RPC, 1=UDP 통신) |
| `ServoJ(joint_pos, axisPos, cmdT=0.008, ...)` | 관절 공간 서보 이동, 짧은 주기로 계속 호출 |
| `ServoCart(mode, desc_pos, exaxis, cmdT=0.008, ...)` | 데카르트 공간 서보 이동 |
| `ServoMoveEnd(cmdType=0)` | 서보 이동 종료 |

- `ServoJ`/`ServoCart`는 **수 ms 주기로 반복 호출**하는 실시간 스트리밍
  방식입니다. 일반 Move 함수처럼 한 번 호출하고 끝나는 게 아닙니다.
- `ServoCart`의 `mode`: 0=절대(베이스), 1=증분(베이스), 2=증분(툴)

> **발견한 이슈**: `example/TestMotionCommand.py`의 `servocart()`는
> `robot.ServoCart(mode=1, desc_pos=pos)`처럼 `exaxis` 인자를 생략하고
> 호출하는데, 현재 SDK의 `ServoCart` 시그니처는 `exaxis`에 기본값이 없어
> **필수 인자**입니다. 그대로 실행하면 `TypeError`가 납니다. 반드시
> `exaxis=[]`(확장 축 없을 시) 를 명시해서 호출하세요.
> (`05_move_servo_realtime.py`에는 이미 수정 반영함)

---

### 6. 스플라인(다중 점 부드럽게 잇기) 이동 — `06_move_spline.py`

| 함수 | 설명 |
|---|---|
| `SplineStart()` / `SplinePTP(joint_pos, tool, user)` / `SplineEnd()` | 기본 스플라인 (관절 기반) |
| `NewSplineStart(type, averageTime=2000)` | 신규 스플라인 시작. `type`: 0=원호 과도, 1=지정 경로점 통과 |
| `NewSplinePoint(desc_pos, tool, user, lastFlag, ...)` | 경로점 추가. 마지막 점은 `lastFlag=1` |
| `NewSplineEnd()` | 신규 스플라인 종료 (실제 동작 트리거) |

여러 좌표점을 순서대로 `NewSplinePoint`로 등록한 뒤, 마지막 점에서
`lastFlag=1`로 넘겨야 로봇이 실제로 그 경로를 부드럽게 이어서 움직입니다.

---

### 7. 조그(수동 점동) 이동 — `07_move_jog.py`

| 함수 | 설명 | 주요 파라미터 |
|---|---|---|
| `StartJOG(ref, nb, dir, max_dis, vel=20.0, acc=100.0)` | 점동 시작 | `ref`: 0=관절,2=베이스,4=툴,8=사용자좌표계 / `dir`: 0=음(-),1=양(+) |
| `StopJOG(ref)` | 감속 정지 | `ref`: 1=관절,3=베이스,5=툴,9=사용자좌표계 정지 |
| `ImmStopJOG()` | 즉시 정지 | - |

티치펜던트 방향키를 누르고 있는 것과 같은 방식으로, `StartJOG` 호출 후
`StopJOG`/`ImmStopJOG`를 호출하기 전까지 계속 그 방향으로 이동합니다.

---

### 8. 포인트 오프셋 옵션 — `08_move_points_offset.py`

| 함수 | 설명 | 주요 파라미터 |
|---|---|---|
| `PointsOffsetEnable(flag, offset_pos)` | 이후 실행되는 Move 좌표에 일괄 오프셋 적용 시작 | `flag`: 0=베이스/사용자좌표계, 2=툴좌표계 |
| `PointsOffsetDisable()` | 오프셋 적용 종료 | - |

팔레타이징처럼 동일 궤적을 여러 위치에 반복 적용할 때, 좌표를 매번
새로 계산하지 않고 오프셋만 바꿔서 재사용할 수 있습니다.

---

## 원본 예제와의 매핑

| 새 파일 | 원본 파일 / 함수 |
|---|---|
| `01_connect_and_enable.py` | `example/TestBasicCommand.py` 전체 |
| `02_move_joint.py` | `example/TestMotionCommand.py` - `movej`, `movej_test` |
| `03_move_linear.py` | `example/TestMotionCommand.py` - `movel`, `movecart`, `jointoverspeedprotect` |
| `04_move_arc_circle.py` | `example/TestMotionCommand.py` - `movec`, `circle`, `newspiral` |
| `05_move_servo_realtime.py` | `example/TestMotionCommand.py` - `servoj`, `servocart` |
| `06_move_spline.py` | `example/TestMotionCommand.py` - `splineptp`, `newsplineptp` |
| `07_move_jog.py` | `example/TestMotionCommand.py` - `startjog`, `stopjog`, `immstopjog` |
| `08_move_points_offset.py` | `example/TestMotionCommand.py` - `pointsoffset` |

## 다음에 정리하면 좋을 것들

`example/` 폴더에는 이 외에도 다음과 같은 카테고리가 더 있습니다.
필요하면 같은 방식으로 추가 정리할 수 있습니다.

- I/O 제어: `TestIOCommand.py`, `TestIOConfig.py`
- 힘 제어(Force Control): `TestForceControlCommand.py`, `FTstrategy.py`
- 안전 설정: `TestSafetySetCommand.py`, `TestSafetyIOConfig.py`, `SafetyParams_Test.py`
- 좌표계/툴 설정: `TestCoord.py`, `TestTRSF.py`, `TestWorkPieceTrsf.py`
- 로그/상태 모니터링: `TestLogCommand.py`, `TestRobotState.py`, `RobotStatePeriod.py`
- 외부 축/컨베이어: `ExtendedAxes_Test.py`, `ConveyorTrackSet.py`, `TestStationaryTrack.py`
- 용접(Weld) 관련: `ArcWeldTrace.py`, `TestReWeld.py`, `TestSplineWeave.py`, `TestOriginPointWeave.py`

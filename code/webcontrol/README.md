# FR5 웹 컨트롤 패널

브라우저에서 버튼/입력값으로 로봇을 움직이는 로컬 웹 도구. 티치펜던트처럼
3D 공간에 점을 직접 찍을 수 없으므로, "현재 위치 기준으로 반지름 10cm
원을 그려라" 같은 파라미터 입력 방식으로 동작합니다.

## 실행

```bash
cd code/webcontrol
pip install -r requirements.txt
python3 app.py
```

브라우저에서 `http://localhost:5000` 접속 → IP 입력 후 "연결" → "활성화 ON".

## 구성

- `geometry.py` — 현재 위치 기준으로 원/호/다각형/타원/직선/회전 목표 좌표를 계산
- `robot_manager.py` — `fairino.Robot`을 감싸는 스레드-세이프 래퍼. 메인 커넥션(상태조회/활성화/이동/조그)은 `_rpc_lock`으로 직렬화하고, 정지/일시정지/재개/조그즉시정지는 완전히 별도의 XML-RPC 커넥션(`_stop_lock`)으로 분리해서 메인 커넥션이 긴 이동 명령으로 바빠도 항상 즉시 처리됨
- `face_tracking.py` — 위치 → 로봇 이동량 변환 순수 로직 (얼굴이든 손이든 bbox 하나만 받으면 되는 범용 로직이라 이름만 남음, 카메라 없이 단위 테스트 가능)
- `vision_tracker.py` — OpenCV로 카메라를 열고 MediaPipe로 오른손+손모양(제스처)을 검출/추적, "편 손"일 때만 조그(`robot_manager.jog_start`/`jog_stop`)로 거리 추적
- `app.py` — Flask REST API
- `static/` — 프론트엔드 (index.html / style.css / app.js)

## 도형 계산 방식

- 모든 도형은 **현재 TCP 위치를 도형 위의 한 점으로 삼아** 계산합니다 (자세는 고정 유지).
- **원/호**: `robot.Circle()` / `robot.MoveC()`를 그대로 사용 — 3점(현재+경유+목표)으로 정확한 원/호를 만듭니다.
- **정다각형/타원**: SDK에 다각형·타원 명령이 없어서, 꼭짓점(또는 타원을 잘게 쪼갠 점들)을 계산해 `MoveL`을 연달아 호출합니다. 타원은 `segments` 값을 늘릴수록 매끄러워집니다.
- **나선**: `robot.NewSpiral()`을 그대로 사용.

## API 요약

| 엔드포인트 | 설명 |
|---|---|
| `POST /api/connect {ip}` | 연결 |
| `GET /api/status` | 상태(에러코드/TCP위치/관절각도/드래그티칭/프로그램상태) |
| `POST /api/enable {state}` | 활성화 0/1 |
| `POST /api/mode {state}` | 자동0/수동1 |
| `POST /api/drag_teach {state}` | 드래그 티칭, state 1=진입/0=해제 |
| `POST /api/reset_error` | 에러 초기화 |
| `POST /api/stop` / `pause` / `resume` | 즉시정지/일시정지/재개 (`StopMotion`/`PauseMotion`/`ResumeMotion`) |
| `POST /api/jog/start {ref,nb,dir,max_dis,vel}` / `jog/stop {ref}` / `jog/stop_immediate` | 조그 |
| `POST /api/move/linear {axis,distance_cm,sign,vel}` | 수직/수평 상대 이동 |
| `POST /api/move/rotate {axis,angle_deg,sign,vel}` | 제자리 회전 |
| `POST /api/shape/circle {radius_cm,plane,direction,vel}` | 원 |
| `POST /api/shape/arc {radius_cm,sweep_deg,plane,direction,vel}` | 호 |
| `POST /api/shape/polygon {sides,side_length_cm,plane,direction,vel}` | 정다각형 |
| `POST /api/shape/ellipse {a_cm,b_cm,plane,direction,segments,vel}` | 타원 |
| `POST /api/shape/spiral {turns,tilt_deg,radius_init_cm,radius_add_cm,axis_add_cm,direction,vel}` | 나선 |
| `POST /api/camera/open {index}` / `close` | 카메라 열기/닫기 |
| `GET /api/camera/stream` | MJPEG 실시간 미리보기 (손 박스 + 인식된 손모양 텍스트 표시됨) |
| `GET /api/camera/state` | 트래킹 상태(인식여부/손모양/크기비율/조그방향 등) |
| `POST /api/camera/calibrate` | 지금 보이는 오른손 크기를 "유지할 거리" 기준으로 저장 |
| `POST /api/camera/config {invert_pan,invert_tilt,invert_z,invert_handedness,max_step_deg,max_step_mm}` | 좌우손 반전 등 설정 (팬/틸트 관련 값은 현재 비활성 기능용으로 남겨둔 것) |
| `POST /api/camera/track/start` / `track/stop` | 손 추적(로봇 이동) 시작/정지 (수동 스위치 - 아래 제스처 게이트와 별개) |

`plane`: `XY`/`XZ`/`YZ`, `direction`: `cw`/`ccw`.

## 조그(JOG)란?

티치펜던트의 방향 버튼을 누르고 있는 것과 같은 개념입니다. 목표 좌표를
주는 일반 Move 계열(MoveJ/MoveL 등)과 달리, **"어느 축을, 어느 방향으로"만**
지정하면 **멈추라는 명령을 받을 때까지 그 방향으로 계속** 움직입니다.

- `StartJOG(ref, nb, dir, max_dis, vel)` — 조그 시작. `ref`: 기준좌표계
  (0=관절, 2=베이스, 4=툴, 8=사용자), `nb`: 축 번호(1~6), `dir`: 0=음(−)/1=양(+),
  `max_dis`: 안전 상한(이 거리/각도를 넘으면 자동 정지), `vel`: 속도(%)
- `StopJOG(ref)` — 감속 정지 (`ref`는 시작값+1: 1=관절,3=베이스,5=툴,9=사용자)
- `ImmStopJOG()` — 감속 없이 즉시 정지

일반 Move는 "여기서 저기까지"처럼 시작과 끝이 정해진 이동이고, 조그는
"이 방향으로, 멈추라고 할 때까지"만 다루는 이동이라 **매번 새 목표를
계산해서 명령을 다시 보낼 필요가 없습니다.** 그래서 카메라로 계속
따라가야 하는 트래킹 같은 용도에는 조그가 훨씬 부드럽고 반응이 빠릅니다
(처음엔 `MoveL`을 반복 호출하는 방식으로 만들었다가, 매번 완전히
멈췄다 재출발하느라 굼뜨고 반응이 느려서 조그 방식으로 바꿨습니다).

## 카메라 오른손 트래킹 동작 방식

1. 매 프레임 MediaPipe `GestureRecognizer`로 손을 검출 — 위치(21개 랜드마크)와 손모양(제스처: `Open_Palm`, `Closed_Fist` 등), 좌우손(`Left`/`Right`)까지 한 모델이 같이 알려줌.
2. 검출된 손 중 **`Right`(오른손)로 분류된 것만** 후보로 남김. MediaPipe의 좌우손 판정은 "거울에 비친(셀카) 영상" 기준이라 로봇 카메라(반전 안 된 일반 영상)에서는 실제 오른손이 `Left`로 잡힐 수 있음 — `invert_handedness`로 반전 가능.
3. `face_tracking.FaceLock`(이름만 얼굴용, bbox면 뭐든 추적하는 범용 로직)이 그 후보들 중 **한 손만** 계속 추적 — 처음엔 가장 큰 손을 잡고, 그 다음부터는 이전 위치와 가장 가까운 것만 같은 손으로 인정. 잠깐(기본 10프레임) 안 보여도 마지막 위치를 유지하고, 그보다 오래 안 보이면 놓친 것으로 확정해 다음 프레임부터 다시 찾음.
4. **손을 편 상태(`Open_Palm`)일 때만** 0.15초마다 지금 손 크기(화면 너비 대비 손 박스 너비 비율)와 캘리브레이션해둔 목표 크기를 비교합니다 (`vision_tracker._update_distance_jog`):
   - 손이 목표보다 작음(멀리 있음) → 아직 전진 조그 중이 아니면 `jog_start(ref=4, nb=3, dir=1)`(공구 Z축, 전진) 시작
   - 손이 목표보다 큼(가까이 있음) → `jog_start(ref=4, nb=3, dir=0)`(후진) 시작
   - 차이가 데드존(`distance_deadzone_ratio=0.02`) 안이면 → `jog_stop(ref=5)`으로 정지
   - **이미 원하는 방향으로 조그 중이면 아무 명령도 다시 안 보냄** — 로봇은 그 사이 계속 부드럽게 움직이고, 이 판단 루프는 그저 "방향을 바꾸거나 멈춰야 하는 순간"만 감지함.
   - **주먹(`Closed_Fist`)을 쥐거나 다른 모양이면, 혹은 손을 놓치면** 그 즉시 조그를 강제로 멈춤(`ImmStopJOG`) — 이게 사실상의 "실시간 정지 제스처".
   - 중앙 정렬(좌우/상하, 손목 회전)은 현재 비활성화 상태 — 거리(전후)만 동작함.
5. 로봇 팔은 베이스가 고정돼 있어서, 손이 카메라 화각/관절 가동범위를 완전히 벗어나면 물리적으로 따라갈 수 없음 — 이 경우는 그냥 손을 놓친 것으로 처리되어 조그가 멈춥니다.
6. 흔들림(jitter) 대책: 검출된 bbox는 지수평활(`_smoothing_alpha=0.35`)로 부드럽게 만들어서, 인식 자체의 프레임간 흔들림 때문에 방향이 계속 바뀌는 걸 줄입니다.

**안전장치**
- "트래킹 시작"을 누르기 전에는 미리보기만 하고 로봇에 어떤 명령도 보내지 않음 (수동 스위치).
- 트래킹이 켜져 있어도 **손을 펴고 있을 때만** 실제로 움직임 (제스처 게이트).
- 조그는 명령을 안 보낸다고 저절로 멈추지 않으므로(일반 Move와 다른 점), 손을 놓치거나 주먹을 쥐거나 "트래킹 정지"/카메라 닫기를 누르면 **항상 즉시 응답하는 별도 커넥션으로 `ImmStopJOG()`를 강제 호출**해서 확실히 멈춥니다.
- 상단 "정지" 버튼을 누르면 트래킹도 같이 꺼짐.
- 조그 방향(전진이 실제로 다가가는 방향이 맞는지 등)은 검증 못 했으므로, 반대로 움직이면 UI의 "거리(전후) 반전" 체크박스로 뒤집어서 맞춰야 합니다.

⚠️ **mediapipe 버전을 절대 올리지 마세요.** 최신 1.x(pip 기본 설치)는 `GestureRecognizer.recognize()` 호출 시 이 macOS 환경에서 매번 강제 종료(세그폴트급 crash, 파이썬 `try/except`로도 못 잡음 — 서버 프로세스 자체가 죽음)하는 것을 실제로 재현해서 확인했습니다. `requirements.txt`에 정상 동작을 확인한 `mediapipe==0.10.14`로 고정해뒀습니다.

⚠️ 코드 자체는 로봇 미연결 환경(카메라도 없음)에서 작성돼서, 개발 중엔 (1) 좌표/제스처 게이트 계산 로직 단위 테스트, (2) 빈 프레임으로 실제 MediaPipe 모델을 끝까지 돌려 크래시 없음 확인, (3) 가짜 인식 결과로 오른손 필터링·좌우반전·조그 상태 전환(전진→정지→후진→강제정지) 로직 검증까지만 할 수 있었습니다. 이후 실제 로봇으로 거리(조그) 추적 동작을 확인했고, 그 과정에서 겪은 문제와 원인은 아래와 같습니다.
- `blendR`(MoveL 평활 반경)이 실제 이동 거리보다 크면 컨트롤러가 이동을 거부함(반환값 14) — 지금은 조그 방식이라 해당 없음.
- `ServoCart`는 공식 권장 호출 주기(1~1.6ms)보다 훨씬 느리게(수십 ms) 호출하면 로봇이 아예 반응하지 않음 — 그래서 이 프로젝트에서는 거리 추적에 ServoCart 대신 조그를 씀.
- 중앙 정렬(손목 회전, 팬/틸트)은 아직 실기 검증 전이라 비활성화 상태로 남겨뒀습니다.

## 알려진 제약

- 활성화(Enable) 상태를 직접 조회하는 SDK 함수가 없어, 현재 활성화 여부는 화면에 표시하지 않습니다 (버튼을 누른 결과의 반환값으로만 확인).
- 도형 그리기는 이동 명령을 순차 실행하므로, 한 도형이 끝나기 전에 다른 이동을 누르면 락에 걸려 대기합니다. 즉시 멈추려면 상단 "정지" 버튼을 쓰세요.
- 로봇 미연결 상태에서 코드만 보고 작성했습니다 — 실기 연결 후 저속으로 먼저 검증하세요.

### SDK 연결 게이트 우회 (`is_connect`)

이 SDK 빌드는 XML-RPC(20003, 실제 이동 명령 채널)와 CNDE(20005, 실시간
상태 스트리밍 채널) **둘 다** 성공해야 `RPC.is_connect=True`로 설정하고,
하나라도 실패하면 모든 SDK 함수가 실제 동작 없이 `-4`만 반환합니다.
로봇 컨트롤러가 CNDE 포트를 막아두거나 지원하지 않는 경우(`Connection
refused`) XML-RPC는 멀쩡한데도 전체가 잠겨버리는 문제가 있어서,
`robot_manager.connect()`가 원본 XML-RPC 프록시(`robot.robot`)로
`GetControllerIP()`를 직접 호출해 XML-RPC 채널만 따로 확인한 뒤
`Robot.RPC.is_connect`를 강제로 복구합니다. 단, CNDE 자체가 필요한
기능(`07_status_and_log`의 `AddRobotRealtimeState`류 실시간 구독)은
CNDE가 실제로 안 붙어 있으면 여전히 동작하지 않습니다.

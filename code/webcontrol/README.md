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
- `vision_tracker.py` — OpenCV로 카메라를 열고 MediaPipe로 오른손+손모양(제스처)을 검출/추적, "편 손"일 때만 `robot_manager.servo_cart_offset()`(ServoCart 실시간 스트리밍)으로 보정 명령 전송
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
| `GET /api/camera/state` | 트래킹 상태(인식여부/손모양/중심오차/크기비율 등) |
| `POST /api/camera/calibrate` | 지금 보이는 오른손 크기를 "유지할 거리" 기준으로 저장 |
| `POST /api/camera/config {invert_pan,invert_tilt,invert_z,invert_handedness,max_step_deg,max_step_mm}` | 회전/이동/좌우손 반전 및 최대 회전폭·이동폭 설정 |
| `POST /api/camera/track/start` / `track/stop` | 손 추적(로봇 이동) 시작/정지 (수동 스위치 - 아래 제스처 게이트와 별개) |

`plane`: `XY`/`XZ`/`YZ`, `direction`: `cw`/`ccw`.

## 카메라 오른손 트래킹 동작 방식

1. 매 프레임 MediaPipe `GestureRecognizer`로 손을 검출 — 위치(21개 랜드마크)와 손모양(제스처: `Open_Palm`, `Closed_Fist` 등), 좌우손(`Left`/`Right`)까지 한 모델이 같이 알려줌.
2. 검출된 손 중 **`Right`(오른손)로 분류된 것만** 후보로 남김. MediaPipe의 좌우손 판정은 "거울에 비친(셀카) 영상" 기준이라 로봇 카메라(반전 안 된 일반 영상)에서는 실제 오른손이 `Left`로 잡힐 수 있음 — `invert_handedness`로 반전 가능.
3. `face_tracking.FaceLock`(이름만 얼굴용, bbox면 뭐든 추적하는 범용 로직)이 그 후보들 중 **한 손만** 계속 추적 — 처음엔 가장 큰 손을 잡고, 그 다음부터는 이전 위치와 가장 가까운 것만 같은 손으로 인정. 잠깐(기본 10프레임) 안 보여도 마지막 위치를 유지하고, 그보다 오래 안 보이면 놓친 것으로 확정해 다음 프레임부터 다시 찾음.
4. **손을 편 상태(`Open_Palm`)일 때만** 0.05초(20Hz)마다 `face_tracking.compute_correction()`이 화면 중심과의 오차(x,y)·목표 크기와의 차이(거리)를 계산해서, **중앙 정렬은 손목 회전(팬=dry, 틸트=drx)으로, 거리 유지는 공구 Z축 평행이동(dz)으로** 조합한 뒤 `robot.ServoCart(mode=2, ...)`(실시간 스트리밍, `servo_cart_offset()`)로 공구 좌표계 기준 증분을 보냄. `MoveL` 방식은 매번 목표 자세를 새로 계산/재생하고 현재 위치도 다시 조회해야 해서 사람이 움직이는 속도를 따라가기엔 느렸음 — `ServoMoveStart()`~`ServoMoveEnd()` 세션 안에서 가벼운 증분만 계속 흘려보내는 방식으로 바꿔서 지연을 줄임. **주먹(`Closed_Fist`)을 쥐거나 그 외 모양이면, 혹은 손을 놓치면 그 순간 아무 명령도 안 보내고 멈춤** — 이게 사실상의 "실시간 정지 제스처".
   - 평행이동만으로 좌우/상하까지 맞추면 팔 전체가 계속 옆으로 밀려나서 작업반경을 금방 벗어나지만, 회전은 제자리에서 방향만 바뀌므로 관절이 허용하는 각도 끝까지 훨씬 넓은 범위를 따라갈 수 있음.
   - 다만 로봇 팔은 베이스가 고정돼 있어서, 손이 카메라 화각/관절 가동범위를 완전히 벗어나면 물리적으로 따라갈 수 없음 — 이 경우는 그냥 손을 놓친 것으로 처리됨.
5. 흔들림(jitter) 대책: 검출된 bbox는 지수평활(`_smoothing_alpha=0.35`)로 부드럽게 만들고, `compute_correction`에 데드존(`deadzone_px=20`, `deadzone_ratio=0.02`)을 둬서 손이 실제로 안 움직였는데도 인식 흔들림만으로 로봇이 반응하지 않게 함.

**안전장치**
- 최대 회전폭(`max_step_deg`, 기본 0.4°)은 서버에서 2°, 최대 이동폭(`max_step_mm`, 기본 0.6mm)은 3mm를 상한으로 강제 — API로 더 큰 값을 넣어도 이 값에서 잘림 (0.05초마다 반복되므로 MoveL 방식 때보다 한 틱당 상한을 낮춰서 전체 속도는 비슷하게 유지).
- "트래킹 시작"을 누르기 전에는 미리보기만 하고 로봇에 어떤 명령도 보내지 않음 (수동 스위치).
- 트래킹이 켜져 있어도 **손을 펴고 있을 때만** 실제로 움직임 (제스처 게이트).
- 상단 "정지" 버튼을 누르면 트래킹도 같이 꺼짐.
- 카메라 장착 방향(팬이 실제로 Ry인지 Rz인지, 부호가 맞는지 등)은 검증 못 했으므로, 반대로 돌거나 엉뚱하게 움직이면 UI의 반전 체크박스로 축별로 뒤집어서 맞춰야 합니다.

⚠️ **mediapipe 버전을 절대 올리지 마세요.** 최신 1.x(pip 기본 설치)는 `GestureRecognizer.recognize()` 호출 시 이 macOS 환경에서 매번 강제 종료(세그폴트급 crash, 파이썬 `try/except`로도 못 잡음 — 서버 프로세스 자체가 죽음)하는 것을 실제로 재현해서 확인했습니다. `requirements.txt`에 정상 동작을 확인한 `mediapipe==0.10.14`로 고정해뒀습니다.

⚠️ **이 기능은 실제 카메라·로봇에 전혀 테스트하지 못했습니다** (이 저장소는 로봇 미연결 환경에서 작성됨, 개발 환경엔 실제 카메라도 없었음). 검증한 것: (1) 좌표/제스처 게이트 계산 로직 단위 테스트, (2) 빈 프레임으로 실제 MediaPipe 모델을 끝까지 돌려 크래시 없음 확인, (3) 가짜 인식 결과로 오른손 필터링·좌우반전·"편 손일 때만 이동" 로직이 의도대로 동작함을 확인. 실제 카메라의 손 인식 정확도, 실제 로봇의 회전 방향/속도感은 검증되지 않았습니다. 반드시 최대 이동폭을 낮게, 정지 버튼에 손을 댄 채로 테스트하세요.

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

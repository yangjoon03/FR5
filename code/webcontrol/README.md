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
- `face_tracking.py` — 얼굴 위치 → 로봇 이동량 변환 순수 로직 (카메라 없이 단위 테스트 가능)
- `vision_tracker.py` — OpenCV로 카메라를 열고 얼굴을 검출/추적, 트래킹 중이면 `robot_manager.move_tool_offset()`으로 보정 명령 전송
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
| `GET /api/camera/stream` | MJPEG 실시간 미리보기 (얼굴 박스 표시됨) |
| `GET /api/camera/state` | 트래킹 상태(인식여부/중심오차/크기비율 등) |
| `POST /api/camera/calibrate` | 지금 보이는 얼굴 크기를 "유지할 거리" 기준으로 저장 |
| `POST /api/camera/config {invert_x,invert_y,invert_z,max_step_mm}` | 축 반전/최대 이동폭 설정 |
| `POST /api/camera/track/start` / `track/stop` | 얼굴 추적(로봇 이동) 시작/정지 |

`plane`: `XY`/`XZ`/`YZ`, `direction`: `cw`/`ccw`.

## 카메라 얼굴 트래킹 동작 방식

1. 매 프레임 Haar Cascade(`cv2.CascadeClassifier`)로 얼굴을 검출.
2. `face_tracking.FaceLock`이 **한 사람만** 계속 추적 — 처음엔 가장 큰(가장 가까운) 얼굴을 잡고, 그 다음부터는 이전 위치와 가장 가까운 얼굴만 같은 사람으로 인정. 잠깐(기본 10프레임) 안 보여도 마지막 위치를 유지하고, 그보다 오래 안 보이면 놓친 것으로 확정해 다음 프레임부터 다시 새 사람을 찾음.
3. "트래킹 시작"을 누른 상태에서 얼굴이 인식되면, 0.25초마다 `face_tracking.compute_correction()`이 화면 중심과의 오차(x,y)·목표 크기와의 차이(거리)를 계산해서 `robot.move_tool_offset(dx,dy,dz)`로 공구 좌표계 기준 보정 이동을 보냄 (dz가 곧 J6/공구 전진-후진 방향의 거리 유지).
4. 얼굴이 안 보이는 동안은 아무 명령도 보내지 않음(가만히 있음, 함부로 움직이지 않음).

**안전장치**
- 최대 이동폭(`max_step_mm`)은 서버에서 15mm를 상한으로 강제 — API로 더 큰 값을 넣어도 15mm에서 잘림.
- "트래킹 시작"을 누르기 전에는 미리보기만 하고 로봇에 어떤 명령도 보내지 않음.
- 상단 "정지" 버튼을 누르면 트래킹도 같이 꺼짐.
- 카메라 장착 방향(로봇 기준 좌/우/상/하/전/후가 실제로 어느 쪽인지)은 검증 못 했으므로, 반대로 움직이면 UI의 반전 체크박스로 축별로 뒤집어서 맞춰야 합니다.

⚠️ **이 기능은 실제 카메라·로봇에 전혀 테스트하지 못했습니다** (이 저장소는 로봇 미연결 환경에서 작성됨, 개발 환경엔 카메라 접근 권한도 없었음). `face_tracking.py`의 좌표 계산 로직만 단위 테스트로 검증했고, OpenCV 카메라 캡처·얼굴 검출·실제 로봇 반응은 검증되지 않았습니다. 반드시 처음에는 물체(인형 등)로, 최대 이동폭을 낮게, 정지 버튼에 손을 댄 채로 테스트하세요.

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

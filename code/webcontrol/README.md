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
- `robot_manager.py` — `fairino.Robot`을 감싸는 스레드-세이프 래퍼 (이동 명령은 락으로 직렬화, 정지 계열은 락 없이 즉시 실행)
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

`plane`: `XY`/`XZ`/`YZ`, `direction`: `cw`/`ccw`.

## 알려진 제약

- 활성화(Enable) 상태를 직접 조회하는 SDK 함수가 없어, 현재 활성화 여부는 화면에 표시하지 않습니다 (버튼을 누른 결과의 반환값으로만 확인).
- 도형 그리기는 이동 명령을 순차 실행하므로, 한 도형이 끝나기 전에 다른 이동을 누르면 락에 걸려 대기합니다. 즉시 멈추려면 상단 "정지" 버튼을 쓰세요.
- 로봇 미연결 상태에서 코드만 보고 작성했습니다 — 실기 연결 후 저속으로 먼저 검증하세요.

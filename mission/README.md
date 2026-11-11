# 로봇팔 배송 미션 코드 (mission/)

팀별 배송 미션의 **로봇팔 담당** 코드입니다. 규정과 설계 이유는 기획안(최종본)을 따릅니다.
(실물에서는 아직 한 번도 돌려 보지 않았습니다 — 아래 "검증 상태" 참고)

## 폴더 한눈에

| 파일 | 하는 일 |
|---|---|
| `menu.py` (`미션_한방.cmd` / `미션_한방.sh`) | **한방 메뉴**: 번호만 누르면 점검 · 티칭 · 연습 · 실행. 처음엔 이것 (`--studio` / `--fake` / `--port`) |
| `doctor.py` | **점검**: 팔 · 카메라 · 자세 · 구역 · 색 · 기록 · 통신 중 무엇이 비었는지와 다음에 할 일 (읽기만) |
| `m0_capture_frames.py` | Pi 에서 카메라 번호 찾기 · 사진 찍기 (구역/색 티칭용 사진) |
| `m0_teach_poses.py` | **자세 티칭**: 팔을 손으로 움직여 자세 저장 (wait, above_red, …) + 공 쥔 그리퍼 % 저장 |
| `m0_teach_rois.py` | **구역 티칭**: 카메라 화면에서 "여기만 본다" 네모 그리기 |
| `m0_teach_colors.py` | **색 티칭**: 빨강·파랑 포스트잇/공 기준색 등록 → 잠금(lock) |
| `mission_main.py` | **본체(상태 기계)**: 도착 감지 → 색 → 집기/넣기(최대 3회) → done → 대기 자세 → clear |
| `store.py` | 작업 기록 (전원이 꺼져도 시도 횟수 · 완료 상태가 남음, A/B 이중 저장 + 순번 + 체크섬) |
| `vision.py` | 카메라로 보는 일: 포스트잇 찾기, 색 강제 선택, 공 찾기, 도착·떠남 확인 |
| `m3_pick_place.py` | 집기 · 넣기 동작 (티칭한 자세만 사용) |
| `arm_io.py` | 팔을 미션 용어로 감싼 것 (`soarm.Robot` 위) |
| `link.py` | 자동차와 통신 (ROS2 상태 메시지 `/arm/state`, 응답 `/car/ack`, `/car/hello`) |
| `car_stub.py` | 자동차 쪽 흉내 (약속을 보여 주는 예시. 실제 자동차 코드는 자동차 담당이 만든다) |
| `sim.py` | **가상 세계** (실물 없이 전체 흐름 연습 · 시험) |
| `config.py` | 모든 숫자 (시간, 개수, 구역 …). 티칭하면 `mission/config.json` 이 생긴다 |

실행 중 생기는 파일(`mission/runtime/`: 작업 기록, 기준색, 로그)은 git 에 올리지 않습니다.

## 1. 먼저 연습 (실물 · 카메라 · ROS2 없이)

```bash
python -m mission.mission_main --sim --list          # 시나리오 목록
python -m mission.mission_main --sim ok              # 한 번에 성공
python -m mission.mission_main --sim fail3           # 3번 실패 → 실패 통보
python -m mission.mission_main --sim hidden_forever  # 안전 중단 예시
```

(Windows 는 `set PYTHONUTF8=1`, 리눅스는 `export PYTHONUTF8=1` 을 먼저 하면 한글 파일이 깨지지 않습니다.)

시험:
```bash
python -m unittest tests.test_mission_store tests.test_mission_vision tests.test_mission_flow
```

## 1-b. PC 에서 ArmStudio 가상 팔 · 가상 카메라로 연습 (Pi · 실물 없이)

ArmStudio 앱을 켜 두면(앱 창이 열려 있어야 카메라가 나온다) 가상 팔 + 손목 가상 카메라로 미션을 연습할 수 있다.

```powershell
.venv\Scripts\python.exe -m mission.m0_teach_poses --studio          # 가상 팔로 자세 티칭 (앱 안에 저장됨)
.venv\Scripts\python.exe -m mission.m0_teach_rois --studio --only note
.venv\Scripts\python.exe -m mission.m0_teach_colors teach note_red --studio
.venv\Scripts\python.exe -m mission.mission_main --studio            # 가상 팔 + 가상 카메라로 미션 실행
```

- 자세는 앱의 티칭 탭에서 저장해도 된다 (이름만 `wait, above_red, red, ...` 와 같게).
- 앱에는 빨간 큐브 · 파란 큐브 · 노란 공이 기본으로 있다. 공은 `Robot.add_object(...)` 로 더 만들 수 있다.
- 가상 팔은 실물과 다르게 단순하다. **실물에서 다시 티칭하고 확인해야 한다.**

## 2. 티칭 순서 (실물, 1일 차)

0. 팔 · 카메라 연결 확인. 그리퍼 카메라 번호는 `camera/c1_find_camera.py` 로 찾아 `mission/config.json` 의 `camera` 에.
1. **자세**: Pi에서 `bash "배달 한방.sh"` → **3번 자세 티칭 → 2번 실물 팔**을 고릅니다.
   VS Code 아래쪽 **Ports → Forward a Port**에서 `8081`을 전달한 뒤 브라우저 아이콘을 누릅니다.
   기존 카메라 미리보기가 열려 있으면 먼저 끄세요. 손목 영상이 보이면 팔을 손으로 받치고
   **팔 힘 풀기 → 자세 선택 → 저장** 순으로 진행합니다. **↑·↓**로 자세 선택, **S**로 저장할 수 있습니다.
   마지막에는 **공을 쥔 집게**를 골라 저장합니다. **자세 사이는 직선으로 오가도 부딪히지 않아야 합니다.**
   가상 팔 화면은 메뉴 3번의 **1번**으로 열 수 있고, 연습한 자세는 파일에 저장되지 않습니다.
2. **구역**: 메뉴 **4번**을 열고 같은 브라우저 탭을 새로고침합니다. 항목의 설명에 맞춰 팔의 자세를 잡고,
   **Space**로 화면을 멈춘 뒤 영상 위에서 네모를 그려 **S**로 저장합니다.
3. **색**: 메뉴 **5번**에서 빨강·파랑 포스트잇과 공을 각각 같은 방식으로 등록합니다. 색이 고른 부분만 선택합니다.
   **6번**에서 두 포스트잇을 번갈아 보여 판정을 확인합니다. **12번**에서는 차가 지나갈 때와
   상차 구역에 멈췄을 때를 비교합니다. 검증이 끝나면 주문 배정 전에 **7번**으로 기준색을 잠급니다.
4. 설치: `python -m mission.mission_main --init-store` (작업 기록 만들기, 한 번만)
5. 먼저 `--no-ros` 로 팔 + 카메라만, 천천히. 그다음 자동차와 연결.

카메라 화면은 **2번 → Enter**로도 열 수 있습니다. 다른 카메라 메뉴로 넘어갈 때는 화면에서
**티칭 끝내기**를 누르고, 새 메뉴를 고른 뒤 기존 브라우저 탭을 새로고침합니다.
경기용 **10번**은 Pi 내부에서 카메라를 사용합니다. 경기 전에 브라우저와 SSH 연결을 닫습니다.

```bash
python -m mission.mission_main --no-ros      # ROS2 없이 (통신은 메모리에만)
python -m mission.mission_main               # 실제 (Pi 의 Docker, ROS2, 팀 전용 ROS_DOMAIN_ID)
```

## 3. 자동차 담당과 맞출 약속 (ROS2)

- 팔이 계속 발행: `/arm/state` (JSON 문자열, 1 Hz). 필드: `arm_boot_id, job_id, step, result, done, clear, status`
- 자동차가 발행: `/car/ack` `{job_id, step, car_boot_id}`, `/car/hello` `{car_boot_id}`
- 자동차는 **`done`(확정 결과) + 같은 job 의 `clear` + 자기 상차 대기 상태** 가 모두 맞을 때만 출발. ack 는 출발 조건이 아님.
- 자동차는 상차 구역에 서면 **팔 메시지와 무관하게 스스로 출발을 잠근다.**
- 자동차가 재시작하면 hello 를 보내고, 저장된 "출발 허가" 를 그대로 실행하지 않는다 (새 clear 필요).
- 예시 구현: `car_stub.py`

## 4. 검증 상태 (솔직하게)

| 확인함 | 확인 못 함 |
|---|---|
| 가상 세계에서 시나리오 9개 동작 (`--sim`) | **실물 팔로 움직여 본 적 없음** |
| 시험 42개 통과 (기록 저장·복구, 색 판정, 도착 확인, 상태 기계) | 실제 카메라 · 조명에서의 색 판정 정확도 |
| 티칭 도구 3개를 임시 경로 + 가상 팔로 실행 | 마우스로 구역을 그리는 창(실제 GUI), 실시간 verify 창 |
| 기록 파일 손상 · 정전 시나리오 | ROS2 실제 통신 (`RosLink`, `car_stub`) — 이 PC 에 ROS2 없음 |
| | 그리퍼 % 기준, 모터 온도 기준 (`config.py` 숫자는 임시값) |

실물에서 처음 돌릴 때: 주변을 비우고, `speed_slow` 를 낮추고, 전원 스위치에 손을 두세요.

## 5. 새 방식: 팔이 차를 세우고 출발시킨다 (2026-11-10 추가)

팀 합의(`Team_koo/docs/interfaces.md`)에 맞춰 **새 실행 파일**을 따로 만들었다. 위 1~4절의 `mission_main.py` 는 예전 방식(차가 스스로 섬)이며 지우지 않고 남겨 두었다.

| 파일 | 하는 일 |
|---|---|
| `motions.py` | **동작 저장소**: 동작 = 자세 여러 개를 순서대로. 같은 동작을 여러 회차로 저장 · 이름 변경 · 삭제 · ★ 선택 · 성적표. `data/motions.json` |
| `motion_page.py` | 허브 **3번 동작 티칭** 화면 (힘 빼기 → 지점 추가 → 저장 → ▶ 재생 → 잘 됨/안 됨 → ★ 이걸로 사용) |
| `watch_page.py` | 허브 **1번 대기 자세 저장** 화면 (차가 오는 쪽을 바라보는 고정 자세 `watch` 를 절대 위치로 저장 · 확인. 카메라 필요 없음) |
| `place_learn.py` | 허브 **4번 바구니 넣기 학습**의 저장소: 세트(★ 실행용) · 회차(✔ 사용 / ✘ 제외) · YOLO 사진 · 모터 값. `data/place_learn.json`, `data/place_learn/` |
| `place_flow.py` | 상차 한 사이클의 단계 2~8 (보이면 바로 정지 · 색 · 집기 · 바구니 위 실시간 맞추기 · 안으로 · 집게 해제 · 성공 확인). 4번과 `loading.py` 가 같이 쓴다 |
| `place_page.py` | 허브 **4번** 화면: 실제 순서대로 돌리며 5~8 을 손으로 가르치기 + 고른 단계부터 테스트 |
| `track_test.py` | (허브 메뉴에서는 삭제됨, 2026-11-11) 예전 차 추적 테스트 |
| `yolo.py` | **YOLO 인식** (`models/arm_v1_yolo11n_320.onnx`, OpenCV 만 사용): 차 · 상자 · 포스트잇 · 공이 든 상자. 허브 **2번**에서 확인 |
| `loading.py` | **상차 미션 본체** (2026-11-11 방식): 차 전원 기다림 → 대기 자세 → 차 + 상자가 보이면 바로 정지 신호 → 완전 정차 확인 → 색 → 집기 → 4번 학습으로 넣기 → 성공 확인 → 대기 자세 → 출발 신호. 실패는 3번까지 (설정 '예전 추적 방식' 을 켜면 예전 흐름). **차는 `/arm/pause` (True = 정지, False = 출발) 로 서고 간다** (차 drive_node 는 loading_status 2 를 안 봄). 오류로 팔이 대기 자세에 못 돌아가면 차는 세워 둔다. **넣기 = 바구니 찾아가기 (학습 없이, place_flow.seek_place)**: 대기 자세 → 카메라 방향 스스로 재기 → 바구니를 화면 가운데로 → 화면의 50% 까지 다가가며 집게를 5° 까지 세움 → 집게 열기 → 5 cm 위 → 성공 확인 |
| `load_link.py` | 차와 통신: `/arm/loading_status` (LoadingStatus) 발행, `/car/status` (CarStatus) 수신. `--no-ros` 면 화면 출력만 |

준비 순서 (실물, 2026-11-11 방식):
1. 허브 1번: 대기 자세 `watch` 저장 (차가 오는 쪽을 바라보는 고정 자세)
2. 허브 2번: 차와 상자가 YOLO 에 잘 잡히는지 확인 (모든 화면 오른쪽 아래 작은 카메라 창에서도 보인다)
3. 허브 3번: `빨간공_집기` · `파란공_집기` 를 여러 회차 가르치고 ▶ 재생 → ★ 선택
4. 허브 4번 (Docker 안에서 `./허브_도커.sh`): 세트를 만들고 [학습 시작] → 5 · 6 · 7 · 8 저장 → [다음 단계 진행]. 여러 번 반복한 뒤 테스트
5. `python -m mission.loading --check` → `--no-ros` → `./상차_도커.sh`

```bash
python -m mission.hub --start motion             # 3번 화면 (Windows: 동작티칭_한방.cmd). 바로 열 화면: watchpose, yolo, motion, place, settings
python -m mission.hub --start motion --fake-arm  # 팔 없이 화면만 눌러 보기
python -m mission.loading --check                # 준비 확인 (팔 안 움직임)
python -m mission.loading --no-ros --jobs 1      # 차 통신 없이 한 대만
python -m unittest tests.test_mission_motions tests.test_mission_loading tests.test_mission_tracktest
```

차 쪽과 맞출 내용 · 합의 문서와 다른 곳: `docs/mission/interface-draft.md`.
확인 못 한 것: 실물 팔 · 실제 카메라에서의 YOLO · ROS2 통신 · 차가 앞뒤로 어긋났을 때의 넣기.

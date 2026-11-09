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
1. **자세**: `python -m mission.m0_teach_poses` → `off`(손으로 받치고) → 팔을 옮김 → `save wait` …
   필요한 자세: `wait, above_red, red, above_blue, blue, above_basket, basket, retreat_mid, recovery_wait`
   마지막에 그리퍼에 공을 물린 채 `ballgrip`. **자세 사이는 직선으로 오가도 부딪히지 않아야 합니다.**
2. **구역**: 각 자세로 팔을 보낸 뒤 `python -m mission.m0_teach_rois --only 이름`
   (`--list` 로 남은 것 확인)
3. **색**: `python -m mission.m0_teach_colors teach note_red` … `note_blue`, `ball_red`, `ball_blue`
   → `verify` 로 실시간 확인 → **둘 다 검증한 뒤** `lock` (주문이 정해지기 전에!)
4. 설치: `python -m mission.mission_main --init-store` (작업 기록 만들기, 한 번만)
5. 먼저 `--no-ros` 로 팔 + 카메라만, 천천히. 그다음 자동차와 연결.

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

# 인수인계: 로봇팔 배송 미션 (Codex 가 이어서 진행하기 위한 문서)

작성 시점: 2026-11-09. 작성: Claude Code 세션. 이어받는 쪽(Codex 등)은 **이 문서 → `arm-plan-final.md` → `mission/README.md`** 순서로 읽으세요.

## 0. 사용자와 일하는 규칙 (반드시 지킬 것)
- 사용자는 **코딩 초보 (로봇 · 임베디드 부트캠프 수강생)**. 답변은 **한국어**, 어려운 말은 풀어서, 코드 설명 끝에 **단어장 표**.
- 순서: **설명 → 계획 → 확인 → 제작 → 검수**. 파일을 만들거나 크게 바꾸기 전에 먼저 쉬운 말로 설명하고 확인을 받는다.
- 비밀번호 · API 키 · 토큰을 파일이나 코드에 쓰지 않는다. 비밀번호는 **사용자가 터미널에 직접** 입력한다.
- 파일 삭제 · 덮어쓰기, 인터넷 다운로드 · 설치, 외부에 올리기 전에 **먼저 묻는다**.
- 실물 팔을 움직이는 코드는 `--sim` / `--studio` / `SOARM_PORT=fake` 로 먼저 확인한다.
- 한글 윈도우는 `PYTHONUTF8=1` 필수. `.venv`(Python 3.14)에 lerobot 을 설치하지 말 것. `scripts/calibrate.py` 는 함부로 실행하지 말 것.
- **대회 규정은 절대 깨면 안 된다.** (`docs/mission/README.md` 가 규정 원문 요약)
- 자세한 사용자 규칙: `CLAUDE.md`, `C:\Users\lenovo\.claude\CLAUDE.md`.

## 1. 목표
팀별 배송 미션의 **로봇팔 담당** 코드를 완성한다. 자동차가 상차 구역에 서면 팔이 **스스로 감지**하고, 그리퍼 카메라로 바구니 포스트잇 색(빨강/파랑)을 읽어 같은 색 공을 집어 바구니에 넣고, ROS2 로 자동차에 알린다. 3차 시도까지, 실패하면 실패 통보. (규정 전체: `docs/mission/README.md`)

## 2. 사용자가 확정한 정책 (재논의하지 말 것)
1. **"승인 보류" 상태는 없다.** 규정에 근거 없는 사항은 보수적 기본값으로 정해 구현하고 시험으로 입증한다. 강사 확인 대기 없음.
2. **색은 티칭으로 무조건 판정**: 경기 전에 빨강·파랑 기준색을 등록(티칭)하고 **잠금**. 판정은 두 기준색 중 가까운 쪽을 항상 선택 (판정 불가 상태 없음). 주문 배정 후 설정 수정 금지, "현재 주문 지정" 기능 금지.
3. **결과 불명은 실패 시도 1회로 센다.** (단, 실패 처리 전에 바구니 안 실제 관측으로 성공 여부를 먼저 판정: `check_success` 하나만 사용)
4. 놓기(그리퍼 열기) 후 다시 집거나 넣는 동작은 새 시도. 시도는 최대 3회, 시도 수 증가를 **저장 성공한 뒤에만** 팔이 움직인다.
5. 경기 중 사람의 제어 명령 · 색 입력 · 시작/재시작 신호 · 파일 수정 금지. 허용: 규정이 허용한 "떨어진 공을 받침대 원위치에 놓기"만.
6. 안전 중단(SafetyAbort)은 "무조건 주행 완료"가 아니다: 기록 손상 · 하드웨어 이상 시 멈추고 `clear` 를 내지 않는다.
- 이 정책들의 근거와 Codex 와의 12회 검토 결과는 `docs/mission/arm-plan-final.md` (최종본, Codex 12차 검토 APPROVED).

## 3. 환경
| 항목 | 내용 |
|---|---|
| PC | Windows 11, 레포 `C:\Users\lenovo\so101-lab`, `.venv`(Python 3.14, numpy · opencv · pyserial 설치됨), 브랜치 **`feature/arm-mission`** (커밋 안 됨!) |
| ArmStudio | PC 앱. `http://127.0.0.1:8000` (**이 PC 안에서만** 접근). 가상 팔 · 가상 카메라(wrist/top/front) · 가상 물체. SDK: `C:\Users\lenovo\ArmStudio\armstudio` |
| Pi (팔 Pi) | `intel@192.168.0.23` (Wi‑Fi `intel_5G`), 레포 `~/so101-lab`, Python 3.13, **`~/so101-lab/.venv` 만들고 numpy · opencv-python-headless · pyserial 설치함**. ROS2 Humble 은 Docker (`docs/raspberry_pi.md`) |
| SSH | WSL `~/.ssh/config` 에 `Host arm` (intel@192.168.0.23, `id_ed25519`) 추가함 (원본 `config.bak`). 파일 복사는 PC 터미널에서 `scp -r C:\Users\lenovo\so101-lab\mission intel@192.168.0.23:~/so101-lab/` (**비밀번호는 사용자가 직접 입력**) |
| 팔 | SO-ARM101 팔로워(12V). **현재 PC · Pi 어디에서도 포트가 인식되지 않음** (PC: 시리얼 포트 0개, Pi: `/dev/ttyACM*` 없음). 팀 공용 장비라 사용자가 항상 직접 꽂아 볼 수는 없음 |
| 레포 원격 | 현재 `origin` = 개인 `th2102da/so101-lab`. 팀 레포 `https://github.com/hwihwi-edu/Team_koo` 는 **아직 연결 안 함**. 팀 규칙: 팀장이 브랜치를 확인하기 전까지는 **본인 브랜치에 커밋·푸시만** (main 에 직접 푸시 · 합치기 · PR 금지) |
| 반드시 알아 둘 점 | 규정상 로봇팔 Pi ↔ 자동차 Pi 의 ROS2 통신만 허용. **대회 당일 실행은 Pi 에서**. PC 연습 · ArmStudio 는 준비용. 자세(`data/poses.json`)와 보정(`data/calibration.json`)은 **실행하는 기기에서 티칭**해야 어긋나지 않음 |

## 4. 코드 지도 (`mission/`)
| 파일 | 역할 |
|---|---|
| `menu.py` + `미션_한방.cmd`(Windows) / `미션_한방.sh`(Pi) | **한방 메뉴**: 점검 · 카메라 · 자세/구역/색 티칭 · 잠금 · 가상 연습 · 실행. `--studio` / `--fake` / `--port` |
| `doctor.py` | **점검 모듈** (읽기만): 환경 · 팔 · 카메라 · 자세 9개 · 구역 8개 · 색 4개 + 잠금 · 작업 기록 · 통신. "다음에 할 일" 출력 |
| `mission_main.py` | 본체 상태 기계 (`Mission`): 도착 → 색 → 시도(≤3) → 분기 → done → 대기 자세 → (실제 관절 확인) → clear → 자동차 떠남. 옵션 `--sim/--studio/--no-ros/--init-store/--jobs` |
| `store.py` | 작업 기록 (A/B 번갈아 저장 + seq + 체크섬 + 추가 전용 로그, 불일치 시 `StoreError` → 안전 중단) |
| `vision.py` | 기준색(티칭·잠금) · 포스트잇 찾기 · 강제 색 선택 · 투표 · 공 찾기 · 도착/떠남 · `check_success` |
| `m3_pick_place.py` · `arm_io.py` | 집기/넣기 동작(티칭 자세 사용) · 팔 래퍼 (`soarm.Robot` 위) |
| `link.py` · `car_stub.py` | ROS2 상태 메시지(`/arm/state` 1Hz, `/car/ack`, `/car/hello`) · 자동차 쪽 예시 |
| `studio.py` | ArmStudio 어댑터 (`StudioRobot`, `StudioCap`) |
| `sim.py` | 가상 세계 (시나리오 9개) — 시험에도 사용 |
| `m0_teach_poses.py` / `m0_teach_rois.py` / `m0_teach_colors.py` / `m0_capture_frames.py` | 티칭 도구 (자세 / 구역 / 색 / 사진 찍기). `--studio` 지원(카메라 사진 찍기 제외) |
| `config.py` | 모든 숫자 (`mission/config.json` 이 생기면 덮어씀). 임시값 다수 |
| `errors.py` | `SafetyAbort` |
| 시험 | `tests/test_mission_store.py`, `test_mission_vision.py`, `test_mission_flow.py` (총 42개, `python -m unittest ...`) |

실행 예: `미션_한방.cmd --fake` · `.venv\Scripts\python.exe -m mission.mission_main --sim fail3` · `.venv\Scripts\python.exe -m unittest tests.test_mission_store tests.test_mission_vision tests.test_mission_flow`

## 5. 검증 상태
| 확인함 | 확인 못 함 |
|---|---|
| 시험 42개 통과 (PC 3.14, Pi 3.13 둘 다) | **실물 팔 동작 (한 번도 안 돌려 봄)** |
| 가상 시나리오 9개 동작 | 실제 카메라 · 조명에서 색 판정 정확도 |
| 티칭 도구를 가상 팔 + 임시 경로로 실행 | GUI 창(구역 드래그, verify 실시간 창) |
| ArmStudio 연결: 각도 · 그리퍼 · 자세 · 온도 읽기 | **ArmStudio 가상 카메라: 503** ("앱 화면이 열려 있어야" — 3D 시뮬레이터 화면이 보이게 두어야 하는 것으로 추정, 미해결) |
| 한방 메뉴 · 점검 `--fake` 동작 (스크립트 입력으로) | 메뉴 `--studio`, 실물 모드, `doctor --studio`, 윈도우 더블클릭 실행 |
| | ROS2 실제 통신 (`RosLink`, `car_stub`): PC 에 ROS2 없음, Pi 는 Docker 안에서만 |

## 6. 앞으로 할 일 (우선순위)
**P0 — 바로**
1. **커밋** (사용자 허락 후): 브랜치 `feature/arm-mission`. 새 파일: `mission/`, `tests/test_mission_*.py`, `docs/mission/arm-plan-final.md`, `docs/mission/HANDOFF.md`, `미션_한방.cmd/.sh`, `.gitignore`(2줄). 어느 레포(개인 / Team_koo)에 올릴지 사용자에게 확인.
2. **Pi 에 최신 코드 재복사** (menu · doctor · capture 등 추가됨): scp 명령은 3절. 비밀번호는 사용자가 입력.
3. **팔 연결**: 팔로워 12V 전원 + 데이터 USB 케이블을 **Pi 에** 연결 → `ls /dev/ttyACM*`. 안 보이면 `lsusb`, `dmesg | tail`.
4. 1일 차 실물 확인: 센서 가용성(관절 위치 읽기 필수, 온도, 부하), 그리퍼 카메라 번호(`m0_capture_frames --scan`), 바구니 · 포스트잇 실제 크기, 자세 사이 직선 이동 시 충돌 없는지.
5. 티칭: `미션_한방.sh`(Pi) 의 3번(자세 9개 + ballgrip) → 구역 8개 → 색 4개 → 검증 → **잠금**. Pi 에는 화면이 없으므로 구역 · 색 티칭은 **Pi 에서 사진 촬영(`m0_capture_frames`) → PC 로 복사 → PC 에서 `--image` 로 티칭 → `mission/config.json` · `mission/runtime/colors.json` 을 Pi 로 복사**.

**P1**
6. ArmStudio 카메라 503 해결(3D 시뮬레이터 탭을 띄우고 `StudioCap` 확인) → `미션_한방.cmd --studio` 로 전체 흐름(가상 공 집기) 검증. 필요하면 `Robot.add_object` 로 공 · 포스트잇 대용 물체를 세팅하는 스크립트 추가.
7. **Pi 에서 ROS2 통신 검증** (Docker Humble): `RosLink` · `car_stub` 실제 시험, 팀 전용 `ROS_DOMAIN_ID`, 자동차 담당(D)과 토픽/메시지 약속 확정 (`mission/README.md` 3절).
8. **전원 켜면 자동 실행** (규정: 전원 켜면 스스로 대기 자세): systemd 또는 Docker entrypoint. 아직 없음. 부팅 시 `status=idle` 이면 대기 자세, 아니면 복구 경로.
9. 실물 `--no-ros` 로 `mission_main` 저속 시험 → 14개 중단 지점 시험(기획안 E절)을 실물/가상으로 확인.

**P2 — 기획안 대비 간소화된 부분 (필요 시 보강)**
- 노출 조정 · 다른 자세 재관측(색 판정 전), 관측 자세별 ROI 분리 · 카메라 이동 중 프레임 폐기는 기본만 구현.
- 온도/부하 기반 과열 스케줄은 임계값(`temp_warn/stop`)만 있고 임시값. 부하(`get_load`) 기반 잡힘 판정은 미사용(개도 + 영상).
- `area_quiet` 임계값, `car_left_step`(포스트잇 사라짐 기준), 도착 확인의 "바구니 형태" 검증(현재 포스트잇 크기 · 모양 · 위치 · 정지)은 현장 튜닝 필요.
- 재시작 복구는 티칭 자세 근처(`SAFE_POSES`)에서만 이동. 낮은 자세 → 위 자세 수직 이동만 허용.
- `studio.py` 어댑터 단위 시험 없음.

## 6-b. 차 추적 (2026-11-09 추가, 실물 미검증)
- **자세 2종**: `watch`(추적 시작 자세: 멀리서 도로 전체) / `wait`(포스트잇 대기 자세: 정차 위치의 포스트잇이 크게 보임). 추적을 끄면 예전처럼 `wait` 한 곳만 쓴다.
- 설정 8번 "차 추적": `tracking_enabled`(기본 꺼짐), `track_pan_sign`(방향), 좌우 한계, 세기, 정차 시간 등 (`mission/config.py` 의 track_* 주석 참고).
- 동작: `watch`에서 **빈 도로 기준 사진**(`mission/runtime/watch_baseline.png`, 허브 4번 버튼)과 비교해 차(바뀐 가장 큰 덩어리)를 찾고, **받침(1번 관절)만** 좌우 한계 안에서 조금씩 돌려 따라간다 → 정차하면 `wait`로 이동 → 기존 도착 확인(`arrival` 구역 안에서 포스트잇 정지) → 색 판정. 20초 안에 확인이 안 되면(횡단보도 등) 다시 `watch`로, 차가 한 번 움직이기 전에는 같은 차를 다시 쫓지 않는다.
- **빈 도로 기준 사진은 받침 각도별로 여러 장**(`mission/runtime/watch_baselines/pan_+000.png` …, 허브 3번 "빈 도로 기준 사진 전체 촬영"이 받침을 좌우로 돌려 가며 자동 촬영). 받침이 돌면 배경이 밀려서 한 장으로는 오인하기 때문이다. 현재 각도와 `track_baseline_tol_deg` 안에 맞는 사진이 없으면 "못 찾음"으로 둔다.
- **자동차 모양 등록**(허브 13번): 앞/뒤/옆/비스듬히/위 방향으로 차를 찍어 `mission/runtime/car_views/`에 저장. 선택 기능 `track_use_car_look`(기본 꺼짐)을 켜면 등록한 차와 색 분포가 닮은 덩어리만 차로 인정(HSV 히스토그램 상관). 임계값은 현장 튜닝 필요.
- **저장된 사진 · 설정 값 보기**(허브 14번, 티칭 화면 위쪽 링크): 빈 도로 사진, 구역 네모를 그려 넣은 사진, 등록한 자동차 사진, 설정 값 요약.
- 티칭 화면의 저장 결과 메시지는 화면 위쪽에 색(성공/실패)으로 표시한다 (예전엔 맨 아래라 안 보였음).
- **방향 자동 뒤집기는 일부러 없앴다**(움직이는 차를 보고 뒤집으면 오해함). 허브 3번 "추적 방향 확인"이 정지한 차를 보며 받침을 +2° 돌려 측정해 `track_pan_sign` 을 채운다.
- 시험: `tests/test_mission_track.py`(차 찾기, 추적 흐름, 안전 범위, 준비 안 됐으면 안 움직임), `tests/test_mission_hub.py`(설정, 기준 사진, 방향 측정).
- **미확인**: 실제 카메라의 차 인식 정확도(어두운 차 + 어두운 도로), 조명 변화, 팔 속도와 추적 안정성, 1번 관절이 카메라 회전 중심인지.

## 6-c. 집기 연습 · 학습 (허브 15번, 2026-11-09 추가, 실물 미검증)
- `mission/practice.py` (연습 전용, 대회 코드와 분리). 5단계(위로 → 잡기 → 들어올리기 → 바구니로 → 놓고 빠지기)를 단계마다 확인 / N단계까지 / 무한 반복.
- 색: 빨강 · 파랑 · 번갈아 · **바구니 포스트잇을 기준색으로 읽어서**(대기 자세, 안 보이면 사람이 계속 누를 때까지 대기).
- 5 미만에서 끝내면 공을 받침대로 되돌려 놓고(받침대 구역이 있으면 공이 보일 때만 자동 계속), 5단계면 공 올려놓기를 기다림.
- 기록 `mission/runtime/practice/log.jsonl`(추가만). `suggest_grip` 이 공을 확실히 쥔 값으로 범위를 제안 → 사람이 '적용'(백업 후 `grip_ball_min/max_pct`).
- 끝나면(정지·오류 포함) wait 자세 → 토크 끄기. GPU/AI 학습 아님. 시험: `tests/test_mission_practice.py` + hub 시험.

## 7. 알려진 함정
- 포트는 한 프로그램만: ArmStudio(PC)가 켜져 있으면 실물 팔 포트를 못 연다.
- `python -m mission.mission_main` 으로 실행하면 모듈 이름이 `__main__` 이 되어 같은 클래스가 둘로 갈라질 수 있음 → 공통 클래스는 별도 파일(`errors.py`)로 뺐음. 새 공통 클래스도 같은 방식으로.
- PowerShell 에서 `R` 은 `Invoke-History` 별칭. 한글 파일은 UTF-8(BOM 없음)로 읽고 쓸 것.
- 대회 모드(`mission_main` 인자 없음)는 **잠긴 기준색**과 `--init-store` 로 만든 작업 기록이 필요. `--studio`/`--sim` 은 자동으로 만든다.
- Pi 에 `.pyc`(3.14) 가 섞여 복사된 상태(무해).

## 8. Codex 에게 줄 첫 프롬프트 (복사해서 사용)
```
레포 so101-lab 의 브랜치 feature/arm-mission 에서 이어서 작업해 줘. 먼저 docs/mission/HANDOFF.md, docs/mission/arm-plan-final.md,
mission/README.md 를 읽고, HANDOFF 6절 P0 → P1 순서로 진행해. 사용자는 코딩 초보(한국어, 단어장, 설명→계획→확인→제작→검수).
대회 규정은 절대 깨지 말 것. 실물 팔은 --sim/--studio/--fake 로 먼저. 비밀번호·키는 절대 파일에 쓰지 말 것.
커밋/푸시/설치/덮어쓰기는 사용자에게 먼저 물어봐. 시험: python -m unittest tests.test_mission_store tests.test_mission_vision tests.test_mission_flow
```

## 9. 2026-11-10 변경: 방침이 바뀌었다 (먼저 읽을 것)
- **팔이 차를 세우고 출발시킨다** (팀 합의 `Team_koo/docs/interfaces.md`). 2절의 "차가 스스로 서고, 통신은 도착 감지에 쓰지 않는다" 는 더 이상 유효하지 않다.
- 새 실행 파일은 `mission/loading.py` (+ `load_link.py`, `yolo.py`, `motions.py`, `motion_page.py`). `mission_main.py` 는 예전 방식으로 남겨 둠 (수정 안 함).
- 색은 YOLO 의 포스트잇 인식 다수결로 정한다 (예전의 "기준색 티칭 · 잠금" 은 새 실행 파일에서 쓰지 않는다). 못 읽으면 추측하지 않고 FAILED.
- 공은 YOLO 로 찾지 않는다 (학습 사진 17장). 빨강 · 파랑 각각 티칭한 동작(`빨간공_집기`, `파란공_집기`)으로 집는다.
- 작업 기록(store) · 재시작 복구는 새 실행 파일에 없다. 전원이 꺼졌다 켜지면 대기 자세에서 새로 시작한다.
- 자세한 사용법: `mission/README.md` 5절. 차 쪽과 확인할 것: `docs/mission/interface-draft.md`.
- 시험: `python -m unittest discover -s tests -p "test_mission_*.py"` (147개 통과, 2026-11-10 PC).

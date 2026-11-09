# CLAUDE.md — so101-lab (SO-ARM101 로봇팔 수업 + 팀별 배송 미션)

Claude Code 가 이 폴더에서 일할 때 먼저 읽는 파일. 자세한 미션 규정은 `docs/mission/README.md`.

## 0. 사용자와 일하는 방식 (가장 중요)
- 사용자는 **코딩 초보 (로봇 · 임베디드 부트캠프 수강생)**. 한국어로 대답한다.
- 설명할 때 **단어(용어) 하나하나 풀어서**. 코드 설명 끝에는 `단어장 (반복용)` 표 (def, self, class, topic …).
- 명령어는 **복사해서 바로 붙여 넣을 수 있게** 코드 블록으로. PowerShell / bash 어느 쪽인지 표시.
- 실물 팔을 움직이는 코드는 **`--dry` (계산만) 또는 `SOARM_PORT=fake` 로 먼저** 확인하게 안내.
- 사용자가 "한방에" 라고 하면 = 한 번 실행으로 끝나는 스크립트 / `.cmd` 런처를 원함.
- 길게 늘어놓기보다: 무엇을 했는지 → 실행 명령 → 확인할 것 → 단어장.

## 1. 환경
| 항목 | 값 |
|---|---|
| PC | Windows 11, `C:\Users\lenovo\so101-lab` (이 레포, https://github.com/th2102da/so101-lab) |
| 레포 가상환경 | `.venv` (Python **3.14**) — soarm SDK · 카메라(my_*) 용. `.venv\Scripts\activate` |
| LeRobot 가상환경 | `C:\Users\lenovo\lerobot-env` (Python **3.12**, lerobot 0.6.1, torch 2.11 CPU) — 3.14 로는 numpy 빌드 실패. **`.venv` 에 lerobot 을 깔지 말 것** (numpy · opencv-headless 로 바뀌어 카메라 창이 깨짐) |
| 한글 윈도우 | 반드시 `PYTHONUTF8=1` (없으면 `data/calibration.json` 을 cp949 로 읽다 실패 → 보정값 무시됨). `카메라_한방.cmd` 에 들어 있음 |
| 팔 | SO-ARM101: 팔로워(12 V, 작업 팔) · 리더(5 V). 모터 STS3215 × 6, ID 1~6 (1 받침 · 2 어깨 · 3 팔꿈치 · 4 손목 꺾기 · 5 손목 돌리기 · 6 그리퍼), 값 0~4095, 가운데 2048 |
| 포트 | PC 팔로워 보통 `COM4` (전압으로 구분: 12 V = 팔로워). Pi 에서는 `/dev/ttyACM0` |
| 포트는 한 프로그램만 | ArmStudio 앱 · 다른 터미널이 쓰면 `Device or resource busy` / 연결 실패 |
| 카메라 | `camera/camera.json` (index 1, 640×480). 위에서 비스듬히 보는 고정 카메라 |
| Pi | Raspberry Pi 5, ROS2 Humble 은 Docker (`docs/raspberry_pi.md`). 팀마다 `ROS_DOMAIN_ID` 다르게 |

## 2. 레포 구조 (원래 수업 코드)
- `soarm/` — SDK. `Robot(port)`: `angles() · torque() · move() · move_all() · gripper(0~100) · wait_until_settled() · save_pose/go_pose · stop() · close()`. 안전장치 내장 (토크 켤 때 튐 방지, 범위 자르기). `soarm/driver_sdk.py` 의 `STS3215Driver` 는 저수준 (ping · get_voltage · get_temperature · get_load).
- `lessons/` 01~08 기초 실습, `project/` p1~p4 색 분류, `theory/` FK · IK.
- `camera/c1~c10` + `camlib.py` — 카메라 수업 (픽셀 → 책상 좌표 H, FK, `ik_down`, `move_urdf`, `tip_xyz`).
- `ros2/so101_ros` — 노드: `arm_node` (`/joint_states`, `/arm/go_pose` · `/arm/gripper` · `/arm/torque` · `/arm/stop`), `camera_node`, `color_node` (`/color/detected` red · blue · none), `task_node`. `ros2/so101_interfaces` srv: GoPose · SavePose · SetGripper.
- `car/` — HiWonder 차(MentorPi) 예제: `carlib.Car`, `c4_car_and_arm.py` (차 + 팔 한 프로그램), `station_node.py`, `link_check.py`.
- `scripts/` — `find_port.py`, `check_motors.py` (읽기만), `from_lerobot.py` (LeRobot 캘리브 → data/calibration.json). **`calibrate.py` 는 EEPROM 을 새로 써서 LeRobot 캘리브와 어긋나므로 함부로 실행하지 말 것.**
- `data/calibration.json` (팔 보정, ArmStudio 에서 복사), `data/poses.json` (`home` 등).

## 3. 이미 만든 것 (camera/my_*.py, `카메라_한방.cmd` 메뉴 1~14)
| 파일 | 역할 |
|---|---|
| `my_menu.py` | 한방 메뉴 (콘솔 모드 저장 · 복구, PYTHONUTF8) |
| `my_pipeline.py` | 처음 셋팅: 카메라 → 색 → 매핑 → home |
| `my_calib.py` | **매핑 (노란 종이)**. 손끝 검사 · RANSAC · 3×3 칸 안내 · 6점/5칸/한 줄 검사. `table_map.json` |
| `my_map.py` | `load_map()` = H + 정밀 보정(`map_fix.json`). `Cam3D` = 높이까지 투영 (solvePnP) |
| `my_finetune.py` | 정밀 보정 (팔이 공 위로 → IJKL 로 맞추기) |
| `my_setup.py` | 경기장 · 골대를 손끝으로 짚기 (+ 바닥 높이, C = 3D 모델 맞추기) |
| `my_floor.py` | 바닥 평면 `floor.json`, `MIN_CLEAR` 8 mm |
| `my_colors.py` | 빨강 · 파랑 · 노랑 HSV (노랑 추가) |
| `my_reach.py` | IK: 수직 → 안 되면 60° 까지 비스듬히 |
| `my_keys.py` | 키 입력 (한글 IME · CapsLock · 터미널 · 화살표). msvcrt 는 kbhit 폴링 (막히면 콘솔 깨짐) |
| `my_live.py` | 화면 스레드 / 팔 일꾼 스레드 분리 (Latest, Status) |
| `my_dribble.py` | 드리블 축구 (WASD 탭/홀드, K 슛, 자동, ESC 긴급정지, 가벽 · 컵 · 부딪힘 감시, 로그, 온도) |
| `my_teach_kick.py` / `my_kicklib.py` | 슛 · 드리블 세기 티칭과 학습 |
| `my_pick_color.py`, `my_follow_color.py`, `my_goto_xy.py`, `my_soccer.py`, `my_config.py`, `my_goal.py` | 색 집기 · 따라가기 · xy 이동 · 골대 차기 · 상세 설정 · 골 라인 |

## 4. 배운 것 (같은 실수 반복 금지)
- 빨강 마커는 실패: **로봇 팔에 빨간 전선이 많아서** 종이로 잘못 잡음 → 매핑은 **노란 종이**.
- 점 4개 · 한 줄로 몰린 매핑은 다른 곳 오차가 수십 cm → 6점 이상, 화면 전체에 고루.
- **카메라를 움직이면 매핑 무효** → 7(매핑) → 12(경기장·골대) → 13(정밀 보정) 다시. 카메라는 테이프로 고정.
- 공중의 손끝을 바닥 평면 H 로 그리면 크게 어긋남 → 높이 있는 점은 `Cam3D.project`.
- FK 오차는 IK 와 같은 모델이라 서로 상쇄 → 매핑 짝은 반드시 **손끝 FK 로** 모은다.
- 한 줄 블로킹 `input()` / `msvcrt.getwch()` 를 스레드에 두면 프로그램 끝날 때 콘솔 모드가 깨짐 → kbhit 폴링.
- 원격 PC 반영 시 같은 이름 파일이 캐시될 수 있어 버전 이름으로 올렸음 (Claude Code 로컬에서는 무관).

## 5. 지금 할 일: 팀별 배송 미션 (로봇팔 담당)
규정 전체 · 점수 · 슬라이드 이미지: **`docs/mission/README.md`**, `docs/mission/img/*.png`.
요약:
1. 전원 켜면 스스로 **대기 자세** (그리퍼 카메라가 위에서 도로를 내려다봄) (-5)
2. 자동차가 오면 **스스로 감지** (사람 신호 금지)
3. **그리퍼(손목) 카메라**로 바구니 포스트잇 색(빨강/파랑) 읽기 → 받침대에서 같은 색 공 (빨강 = 왼쪽, 파랑 = 오른쪽, 지름 3 cm 스펀지) 집어 바구니에 (다른 색 -20)
4. **ROS2 로 자동차에 상차 완료** 알리고 대기 자세 복귀. 팀마다 ROS_DOMAIN_ID 다르게
5. 실패하면 3차 시도까지 (2차 -5, 3차 -10), 3차 실패면 실패를 알리고 차는 공 없이 출발 (-30)
6. 공은 주차 끝날 때까지 바구니 안 (-10). 로봇팔 Pi ↔ 자동차 Pi ROS2 통신만 허용

※ 미션 카메라는 **그리퍼에 달린 카메라** (지금까지 쓴 위쪽 고정 카메라와 다름). 받침대 위치는 고정, 자동차 상차 정지 위치는 팀이 정함.

## 6. 코드 규칙
- 새 미션 코드는 `mission/` 폴더 (예: `m1_wait_pose.py` …). 기존 수업 파일은 고치지 말고 가져다 쓴다.
- 파일 맨 위 docstring 에 **실행 명령 · 순서 · 고칠 숫자**를 한국어로. 주석도 초보자용 한국어.
- 팔 코드는 항상 `try / finally` 로 끝내기: **대기 자세 → 토크 끄기 → disconnect / close** (공중에서 토크 끄면 떨어짐, 포트 잠김 방지).
- 숫자(자세 · 색 범위 · 시간)는 맨 위 상수나 json 설정으로. 티칭 자세는 `data/poses.json` (`save_pose/go_pose`).
- 실행 전 `python -m py_compile`, 가능한 건 `SOARM_PORT=fake` 로 시험. 실물은 사용자가 확인.
- ROS2 코드는 Pi 의 Docker (Humble) 에서 돈다: rclpy, 표준 메시지(std_msgs) 우선.

## 7. 참고 팁 (수업 슬라이드)
- 모터 응답 없음: ① `ls /dev/ttyACM*` ② `sudo chmod 666 /dev/ttyACM0` (영구: `sudo usermod -aG dialout $USER` 후 재로그인) ③ 포트는 한 프로그램만 ④ 모터마다 LED 확인 ⑤ 선 다시 꽂기 ⑥ ping 으로 ID 1~6.
  - Permission denied → ② / No such file → ① / Device or resource busy → ③ / 연결되는데 안 움직임 → ④⑤⑥
- RGB LED (차 보드): 토픽 `/ros_robot_controller/set_rgb`, 메시지 `ros_robot_controller_msgs/RGBStates` (`RGBState(index=1|2, red, green, blue)`). 예제 `/home/ubuntu/RRCLite_demo/rgb_controller_node.py`.
- LeRobot 0.6.1: `lerobot-find-port / setup-motors / calibrate / teleoperate / record / train / rollout`, 타입 `so101_follower` · `so101_leader`, id `follower` · `leader`.

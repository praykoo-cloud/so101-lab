# so101-lab

SO-ARM101 을 **컨트롤러 앱 없이** Python 으로 바로 움직이는 실습 레포. 티칭(손으로 자세를 잡고 저장 → 재생) 중심.

- 팔을 꽂은 PC(Windows · Ubuntu)나 라즈베리파이 5 에서 같은 코드가 돈다. 필요한 것은 Python 3.10 이상과 `pyserial` 뿐
- `soarm.Robot` 은 so-arm101-controller-app 의 `sdk/soarm.py` 와 함수 이름이 같다. 그쪽은 컨트롤러 서버에 HTTP 로, 이쪽은 시리얼 포트로 바로 보낸다
- 자세 · 녹화 파일(`data/poses.json`, `data/recordings/*.json`)은 컨트롤러 앱과 같은 형식이다

## 설치

**Windows (PowerShell)**

```powershell
git clone https://github.com/th2102da/so101-lab.git
cd so101-lab
python -m venv .venv
.venv\Scripts\activate
pip install -e .
```

**Ubuntu · 라즈베리파이 OS**

```bash
git clone https://github.com/th2102da/so101-lab.git && cd so101-lab
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
sudo usermod -aG dialout $USER     # 시리얼 포트 권한. 로그아웃 후 다시 로그인
```

카메라 프로젝트까지 하려면 `pip install -e ".[camera]"` (opencv-python, numpy).

## 처음 한 번: 포트 · 팔 확인 · 관절 범위

```powershell
python scripts/find_port.py              # 보이는 포트. --unplug 로 어느 팔인지 찾기
python scripts/check_motors.py COM4      # 읽기만 함. 전압으로 팔 구분: 팔로워 약 12V, 리더 약 5V
python scripts/from_lerobot.py follower  # LeRobot 캘리브레이션 파일 → data/calibration.json
```

`data/calibration.json` 은 팔마다 다르다 (관절마다 움직일 수 있는 서보 값 범위). 만드는 방법은 둘 중 하나:

| 팔 상태 | 방법 |
|---|---|
| LeRobot 으로 캘리브레이션한 팔 | `scripts/from_lerobot.py <id>` — 모터에 아무것도 쓰지 않는다 |
| 처음 쓰는 팔 | `scripts/calibrate.py --port COM4` — 모터 EEPROM 의 가운데 기준을 새로 쓴다. 그 뒤 LeRobot 을 쓰려면 lerobot-calibrate 를 다시 해야 한다 |

## 실습 (lessons/)

파일 맨 위의 `PORT` 를 내 팔 포트로 바꾸고 실행한다 (`python lessons/01_hello.py`).

| 파일 | 내용 | 팔 |
|---|---|---|
| 01_hello | 연결, 관절 각도 · 전압 · 온도 읽기 | 안 움직임 |
| 02_joint_feel | 관절 하나 +10° 왕복, + 방향 알아보기 | 조금 |
| 03_teach_read | 티칭 1: 손으로 자세 → 각도 한 줄 출력 | 손으로 |
| 04_first_pose | 티칭 2: 적어 둔 자세로 가기 | 움직임 |
| 05_gripper | 그리퍼 열기 · 닫기, 내 물체에 맞는 닫기 값 | 그리퍼 |
| 06_save_poses | 티칭 3: 자세를 이름으로 저장 (home · above_pick · pick · above_place · place) | 손으로 |
| 07_pick_place | 티칭 4: 저장한 자세로 픽 앤 플레이스 | 움직임 |
| 08_record_play | 티칭 5: 손으로 움직인 동작 녹화 → 재생 | 손으로 → 움직임 |

팔이 없으면 `PORT` 대신 `fake` 를 쓴다 (`soarm/fake.py`, 가상 서보 버스). 같은 코드가 돈다.

## 이론 (theory/)

기구학 · 제어 이론 실습. 팔 없이 계산만 한다 (`pip install -e ".[theory]"`, numpy).
기구학은 `soarm/kinematics.py` (URDF 에서 관절 사슬을 읽어 정기구학 · 수치 역기구학).

| 파일 | 내용 |
|---|---|
| t1_fk | 2링크 정기구학 손 계산, SO-101 DH 표 vs URDF (차이 0.04 mm 이내). `COM4` 를 주면 실물 팔의 지금 손끝 |
| t2_ik_geometric | 2링크 역기구학: 코사인 법칙, 답 2개 · 답 없음 |
| t3_ik_iterative | 반복 역기구학: CCD vs 감쇠 최소제곱(DLS), 닿지 않는 목표, 시작 자세에 따라 다른 답 |
| t4_ik_neural | 신경망 역기구학 (3 → 128 → 128 → 4, numpy): 손끝 오차, 학습 범위 밖, 신경망 + DLS |
| t5_bangbang_vs_p | 관절 하나 시뮬레이션: 뱅뱅 · P · PD |

## 프로젝트: 색 분류 (project/)

카메라가 검사 자리의 색(빨강 · 파랑)만 보고, 팔은 티칭한 자세로 집어 색에 맞는 상자에 놓는다. 위치를 찾지 않으므로 카메라 보정이 필요 없다. `pip install -e ".[camera]"` 가 필요하다.

| 파일 | 내용 |
|---|---|
| p1_camera | 카메라 확인, 검사 자리(ROI) 정하기 → `project/color.json` |
| p2_color_check | 검사 자리의 색 판별 (같은 색 5장 연속이면 확정) |
| p3_teach | 자세 7개 티칭: home · above_pick · pick · above_red · red · above_blue · blue |
| p4_sort | 분류: WAIT → PICK → PLACE → CLEAR 상태 기계 |

카메라 · 색 종이가 없으면 `python scripts/make_test_video.py` 로 시험 영상(`data/color_test.mp4`, 빨강 · 파랑이 한 번씩)을 만들어 카메라 자리에 준다.

## ROS2 (ros2/) · 라즈베리파이 (docker/)

`so101_interfaces` (서비스 GoPose · SavePose · SetGripper) 와 `so101_ros` (노드 4개). 노드는 `soarm` 을 그대로 쓴다.

| 노드 | 하는 일 |
|---|---|
| arm_node | 로봇과 이야기하는 유일한 노드. `/joint_states` 발행, `/arm/torque` · `/arm/go_pose` · `/arm/save_pose` · `/arm/gripper` · `/arm/stop` 서비스 |
| camera_node | `/camera/image_raw` (영상 파일도 받는다 — 카메라 없이 시험할 때) |
| color_node | `/color/detected` (red · blue · none), `/color/debug_image` |
| task_node | p4 의 상태 기계. 토픽 하나와 서비스만 쓴다 |

```bash
ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0 rviz:=true
ros2 launch so101_ros sort.launch.py port:=/dev/ttyACM0 camera:=/dev/video0
```

라즈베리파이 5 는 Ubuntu 22.04 를 공식 지원하지 않아 ROS2 Humble 을 Docker 컨테이너로 쓴다: [docs/raspberry_pi.md](docs/raspberry_pi.md).

## Robot 한눈에

```python
from soarm import Robot

bot = Robot("COM4", speed=150)     # speed: 서보 속도 (틱/초). 200 ≈ 17.6°/s
bot.angles()                       # {1: -3.8, 2: -100.4, ...}  0° = 서보 가운데
bot.torque(True)                   # 잠그기 · torque(False) 풀기
bot.move("elbow", 30)              # 관절 하나 (base · shoulder · elbow · wrist_pitch · wrist_roll · gripper 또는 1~6)
bot.move_all({"base": 10, "elbow": 60})
bot.gripper(60)                    # 0 닫힘 ~ 100 열림
bot.wait_until_settled()           # 마지막 목표에 도착할 때까지
bot.save_pose("home"); bot.go_pose("home")   # 그리퍼는 그대로 둔다 (gripper=True 면 저장한 값까지)
frames = bot.record(5, name="take1"); bot.play("take1")
bot.stop()                         # 그 자리에서 멈춤
```

## 안전 규칙 (SDK 안에 들어 있는 것)

| 문제 | SDK 가 하는 일 |
|---|---|
| 모터는 예전 목표를 기억한다. 손으로 옮긴 뒤 토크를 켜면 그 자리로 튄다 (41° 까지 봄) | `torque(True)` 는 목표 = 지금 위치를 먼저 쓰고 켠다 |
| 토크가 꺼진 서보에 목표 위치를 쓰면 서보가 토크를 스스로 켠다 | 토크가 꺼진 관절에는 목표를 쓰지 않는다 (연결 · `stop()` 포함) |
| 토크가 꺼진 관절에 move 를 보내면 아무 일도 없어 헷갈린다 | `RobotError: 토크가 꺼진 관절이 있습니다` |
| 'moving' 판단이 느린 속도에서 너무 일찍 끝난다 (5° 명령이 0.7° 만 가고 다음으로) | `wait_until_settled()` 는 보낸 목표와 비교한다 (0.5° 안, 물체에 막히면 0.5초 멈춤으로 끝) |
| 관절 범위 밖 명령 | `data/calibration.json` 범위로 자른다 |

## 폴더

```
so101-lab/
├─ soarm/            driver_sdk.py (시리얼 드라이버) · robot.py (Robot) · color.py (색 판별) · fake.py (가상 팔) · kinematics.py
├─ lessons/          실습 01~08
├─ theory/           이론 실습 t1~t5 (기구학 · 제어)
├─ project/          색 분류 프로젝트 1~4
├─ scripts/          find_port · check_motors · from_lerobot · calibrate · make_test_video
├─ ros2/             so101_interfaces · so101_ros
├─ docker/           라즈베리파이용 ROS2 Humble 컨테이너
├─ docs/             raspberry_pi.md
├─ model/            so101.urdf · kinematics.json · assets/*.stl
├─ data/             calibration.json · poses.json · recordings/ (팔마다 다름, git 에 안 올림)
└─ tests/            pytest (가상 팔 · 합성 이미지), real_smoke.py (실물 확인)
```

## 출처

- `soarm/driver_sdk.py`, `scripts/calibrate.py`, `model/` 는 so-arm101-controller-app (커밋 091af73) 에서 가져왔다
- `model/so101.urdf` 원본: TheRobotStudio SO-ARM100 (Apache-2.0)

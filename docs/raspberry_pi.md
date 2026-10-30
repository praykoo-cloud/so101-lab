# 라즈베리파이 5 에서 하기

팔과 카메라를 Pi 에 꽂고, PC 에서 SSH 로 들어가 작업한다. ROS2 는 Docker 컨테이너(Ubuntu 22.04 + Humble)에서 돌린다.

| 왜 이렇게 | |
|---|---|
| OS 는 Raspberry Pi OS 64비트 | Pi 5 는 Ubuntu 22.04 를 공식 지원하지 않는다 (Ubuntu 는 24.04 부터) |
| ROS2 는 Docker | Humble 은 Ubuntu 22.04 용이다. 공식 `ros:humble` 이미지에 arm64 판이 있다 |
| 실습 01~08 · 프로젝트 1~4 는 컨테이너 밖에서 | Pi OS 의 Python 3 + pyserial 로 바로 돈다. Docker 없이 |

## 1. SD 카드 (PC 에서, 처음 한 번)

1. Raspberry Pi Imager → 기기 `Raspberry Pi 5`, OS `Raspberry Pi OS (64-bit)`
2. 설정(톱니바퀴): 호스트 이름 `so101-01` (팀마다 다르게) · 사용자 이름 · 비밀번호 · Wi-Fi · **SSH 사용**
3. 카드를 Pi 에 꽂고 전원. 1~2분 뒤 PC 와 같은 공유기에 붙는다

## 2. SSH 로 들어가기 (PC 의 PowerShell)

```powershell
ssh <사용자>@so101-01.local
```

`so101-01.local` 이 안 되면 공유기 관리 화면에서 Pi 의 IP 를 찾아 `ssh <사용자>@192.168.0.xx`.
VS Code 의 Remote - SSH 확장으로 같은 주소에 붙으면 Pi 의 파일을 PC 에서 바로 고칠 수 있다.

## 3. 레포 · 실습 (Pi 의 터미널)

```bash
sudo apt update && sudo apt install -y git python3-venv python3-opencv
git clone https://github.com/th2102da/so101-lab.git ~/so101-lab && cd ~/so101-lab
python3 -m venv --system-site-packages .venv && source .venv/bin/activate   # opencv 는 apt 것을 쓴다
pip install -e .
sudo usermod -aG dialout $USER          # 시리얼 포트 권한 → 로그아웃 후 다시 SSH
ls /dev/ttyACM*                         # 팔: 보통 /dev/ttyACM0
python scripts/check_motors.py /dev/ttyACM0
export SOARM_PORT=/dev/ttyACM0          # 실습 파일의 PORT 대신
python lessons/01_hello.py
```

캘리브레이션 파일은 팔마다 다르다. PC 에서 쓰던 팔이면 `data/calibration.json` 을 Pi 로 복사한다 (`scp`).
화면이 없으므로 카메라 확인은 `python project/p1_camera.py --roi 270 190 100 100 --snapshot` → `project/snapshot.jpg` 를 PC 로 가져와 본다.
카메라는 일반 USB 웹캠(UVC)을 쓴다. 깊이 카메라처럼 전용 드라이버가 있는 카메라는 OpenCV 로 열면 깨진 화면이 나오거나 열리지 않는다.

## 4. Docker 로 ROS2 Humble

```bash
curl -fsSL https://get.docker.com | sh          # Docker 공식 설치 스크립트
sudo usermod -aG docker $USER                   # → 로그아웃 후 다시 SSH
cd ~/so101-lab
docker compose -f docker/compose.yaml build     # 처음 한 번 (ros:humble arm64 이미지를 받는다)
docker compose -f docker/compose.yaml run --rm so101
```

**이미지 파일로 넣기.** 다른 Pi 5 에서 `docker save so101-lab:humble | gzip > so101-lab_humble_arm64.tar.gz` 로 만든 파일이 있으면, 빌드 없이 넣을 수 있다:

```bash
docker load < so101-lab_humble_arm64.tar.gz     # → Loaded image: so101-lab:humble
docker compose -f docker/compose.yaml run --rm so101
```

이미지에는 Ubuntu 22.04 + Humble + 의존 패키지만 들어 있다. 레포 코드는 컨테이너가 뜰 때 `~/so101-lab` 을 붙여서 쓰고, `ros2/` 를 그때 빌드한다 → 코드는 `git pull` 로만 바꾸면 되고 이미지는 다시 넣지 않는다.

컨테이너 안 (`root@so101-01`):

```bash
ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0     # 팔만
ros2 launch so101_ros sort.launch.py port:=/dev/ttyACM0 camera:=/dev/video0   # 색 분류 전체
```

- 컨테이너는 Pi 의 `~/so101-lab` 을 그대로 쓴다. 자세(`data/poses.json`) · 카메라 설정(`project/color.json`)이 Pi 에 남는다
- 컨테이너는 Pi 의 `/dev` 를 통째로 본다. 팔 · 카메라 이름이 `/dev/ttyACM0` · `/dev/video0` 가 아니면 launch 의 `port:=` · `camera:=` 만 바꾼다
- 끝낼 때는 Ctrl+C. 뒤에서 띄운 컨테이너(`run -d`)는 `docker stop <이름>` 으로 끄면 Ctrl+C 와 같이 2 ~ 3초 안에 내려간다

## 5. PC 에서 보기 (RViz)

PC 의 WSL Ubuntu 22.04 + Humble 에서. PC 에도 레포를 받아 `colcon build` 해 둔다 (RViz 가 팔 메시를 `package://so101_ros/meshes/` 에서 찾는다):

```bash
export ROS_DOMAIN_ID=<팀 번호>          # Pi 와 같은 번호. Pi 쪽은 compose 실행 전에 같은 값으로 export
ros2 topic list                          # /joint_states · /color/detected 가 보여야 한다
rviz2 -d ~/so101-lab/ros2/so101_ros/rviz/so101.rviz     # Fixed Frame base_link, RobotModel 이 미리 들어 있다
```

같은 공유기에 있어야 하고, WSL 은 미러 네트워크(`networkingMode=mirrored`)여야 한다. 공유기가 멀티캐스트를 막으면 노드가 안 보인다 → 멀티캐스트가 되는 공유기를 따로 쓴다.
다른 공유기(다른 Wi-Fi)에 있으면 Pi 로 ping 은 가도 노드는 보이지 않고 `ros2 service call` 이 멈춘다.

## 6. HiWonder 차와 같은 도메인에서

차(MentorPi)도 `/joint_states` · `/robot_description` · `robot_state_publisher` · `base_link` 를 쓴다. 팔을 그냥 띄우면 이 이름들이 섞인다. 팔에 네임스페이스를 준다:

```bash
ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0 namespace:=so101
```

→ `/so101/joint_states` · `/so101/robot_description`, TF 프레임 `so101/base_link` ... 서비스 `/arm/*` 는 그대로. RViz 는 Fixed Frame `so101/base_link`, RobotModel Topic `/so101/robot_description`.

`GoPose` 같은 so101 전용 서비스는 `so101_interfaces` 가 깔린 쪽에서만 부를 수 있다. 차 컨테이너에서는 `std_srvs` 서비스(`/arm/stop` · `/arm/torque`)만 된다.

## 문제가 생기면

| 증상 | 할 일 |
|---|---|
| 처음 `ros2 node list` 가 오래 멈추거나 `TimeoutError: [Errno 110]` | `ros2 daemon start` 를 먼저 한 번 |
| `ros2 node list` 가 `!rclpy.ok()` 오류 | 망을 바꾸기 전에 뜬 데몬이 남았다 → `ros2 daemon stop` 후 `ros2 daemon start` |
| PC 에서 Pi 의 노드가 안 보인다 | 같은 공유기인지, `ROS_DOMAIN_ID` 가 같은지, WSL 이 미러 네트워크인지 |
| RViz 에 팔 모양이 안 나온다 | PC 에서도 `colcon build` 를 했는지 (`package://so101_ros`) |
| Pi 에서 `git clone` 이 `could not read Username` | 레포가 비공개다 → 공개 레포를 쓰거나 GitHub 토큰으로 받는다. 토큰이 없으면 PC 에서 `git bundle create so101.bundle main` → `scp` → Pi 에서 `git clone -b main so101.bundle ~/so101-lab` (`-b main` 이 없으면 빈 폴더) |

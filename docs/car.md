# HiWonder MentorPi 차와 같이 쓰기 (보충)

차(MentorPi, 메카넘 바퀴)의 Pi 5 에 팔로워를 꽂고, 차 컨테이너와 so101-lab 컨테이너를 같은 ROS_DOMAIN_ID 로 띄운다.
한 Pi 에서 맞춘 뒤, 팔을 자기 Pi 로 옮겨도 같은 코드가 돈다 ([두 Pi 로](#두-pi-로-차-pi--팔-pi)).
예제(`car/`)는 ROS2 표준 메시지(Twist · Odometry · Imu · LaserScan · UInt16)만 쓰므로 HiWonder 패키지 없이 so101-lab 컨테이너 · WSL 에서 돈다.

## 차의 장치

| 장치 | USB ID | 이름 |
|---|---|---|
| STM32 확장 보드 (모터 4 · IMU · 배터리 · 부저) | `1a86:55d4` | `/dev/rrc` (udev) |
| LD19 라이다 | `1a86:7523` (CH340) | `/dev/ldlidar` → ttyUSB0 |
| HP60C 깊이 카메라 | `3482:6723` | ROS 노드만 연다 (OpenCV 로 안 열림) |
| SO-101 팔로워 | `1a86:55d3` | ttyACM0 또는 ttyACM1 → `/dev/serial/by-id/...` 로 |

`lsusb` 에 `55d4` 가 없으면 바퀴가 돌지 않는다. STM 보드의 USB 포트와 Pi 를 **데이터용** USB 선으로 잇는다 (충전 전용 선은 전원만 간다).

## 차 쪽 (차 Pi 에 SSH)

```bash
~/docker_shell.sh                                # = docker exec -it -u ubuntu -w /home/ubuntu IntelPi /bin/zsh
export ROS_DOMAIN_ID=30                          # 기본 30 (~/.zshrc). 팀 번호로 바꿔도 된다. 아래 모두 같은 값
ros2 launch controller controller.launch.py      # STM 보드 · 오도메트리 · IMU 필터
ros2 launch peripherals lidar.launch.py          # 다른 창에서. /scan_raw
ros2 launch peripherals depth_camera.launch.py   # 다른 창에서. /ascamera/...
```

`docker exec IntelPi bash` (root · bash) 로 들어가면 ros2 가 없고 도메인도 0 이다. 그때는 `export HOME=/home/ubuntu; source ~/ros2_ws/.robotrc; export ROS_DOMAIN_ID=30`.

## 팔 쪽 (같은 Pi)

```bash
cd ~/so101-lab
ls -l /dev/serial/by-id/                         # 팔의 이름 (번호는 꽂는 순서로 바뀐다)
export ROS_DOMAIN_ID=30
docker compose -f docker/compose.yaml run --rm --name so101car so101 \
  ros2 launch so101_ros arm.launch.py port:=/dev/serial/by-id/<팔> namespace:=so101
```

예제는 다른 창에서 `docker exec -it so101car bash` 로 들어가 실행한다.

compose 는 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 를 넘긴다. 같은 Pi 에서 차 노드는 ubuntu 사용자, 이 컨테이너는 root 라 FastDDS 의 공유 메모리로는 서로 데이터를 받지 못한다 (노드 목록에도 안 나온다).

## 예제 (car/)

| 파일 | 하는 일 | 차 |
|---|---|---|
| c1_sensors | 배터리 · IMU yaw · 오도메트리 · 라이다 앞/왼/오/뒤 | 안 움직임 |
| c2_drive | 앞뒤 · 옆 3 cm, 15° 회전. 명령 vs 오도메트리 vs IMU | 움직임 |
| c3_lidar_stop | 앞 30 cm 까지 천천히, 최대 10 cm (`--dry` 는 판단만) | 움직임 |
| c4_car_and_arm | 다가가기 → 그리퍼 열고 닫기 → 돌아오기 (`--dry` 는 팔만) | 움직임 |

- 차는 마지막 명령을 계속 따른다. `carlib.Car` 는 Ctrl+C · `docker stop` · `timeout` 에도 0 을 보내고 끝난다
- `/controller/cmd_vel` 은 속도를 자르지 않는다 → `carlib` 이 0.2 m/s · 1.0 rad/s 에서 자른다
- 오도메트리(`/odom`)는 명령한 속도를 더한 값이다. 미끄러져도 모른다. 회전은 IMU 가 실제로 잰 값과 비교한다
- 라이다는 차 앞이 0°, 왼쪽이 +90°. 10 cm 안 점은 차 자신이라 버린다. 뒤쪽은 차체에 가린다
- 라이다는 책상 끝 같은 낭떠러지를 못 본다. 앞에 잡힌 점이 없으면 c3 · c4 는 멈춘다 (모르면 멈춤)

## 두 Pi 로 (차 Pi + 팔 Pi)

실제로는 팔을 자기 Pi 에 꽂는다. 같은 공유기 · 같은 `ROS_DOMAIN_ID` 면 위의 것이 그대로 돈다. 팔 쪽 컨테이너만 다른 Pi 로 옮긴다.

| 어디 | 띄우는 것 | 이유 |
|---|---|---|
| 팔 Pi | so101-lab: `arm.launch.py port:=<by-id> namespace:=so101` | 팔 USB 가 꽂힌 곳. 자세 · 캘리브레이션(`data/`)도 여기 |
| 차 Pi | IntelPi: controller · lidar (· depth_camera) | 차 보드 · 센서가 꽂힌 곳 |
| 차 Pi | so101-lab (두뇌, 팔 없이): `car/c4_car_and_arm.py` | 차는 마지막 명령을 따른다 → 차를 모는 프로그램은 차 Pi 에 (Wi-Fi 가 끊겨도 0 을 보낸다). `GoPose` 를 부르려면 so101_interfaces 가 필요 |

```bash
# 팔 Pi
cd ~/so101-lab && export ROS_DOMAIN_ID=30
docker compose -f docker/compose.yaml run --rm --name so101arm so101   ros2 launch so101_ros arm.launch.py port:=/dev/serial/by-id/<팔> namespace:=so101

# 차 Pi (차 launch 는 위 '차 쪽' 그대로)
cd ~/so101-lab && export ROS_DOMAIN_ID=30
docker compose -f docker/compose.yaml run --rm --name so101brain so101 bash
ros2 node list                                   # /so101/arm_node 가 보여야 한다
python3 car/link_check.py                        # 서비스 왕복 · joint_states 속도 · 시계 차
python3 car/c4_car_and_arm.py --dry
```

먼저 확인할 것 (두 Pi 모두):

- `hostname -I` 와 `ping` 으로 같은 공유기 · 같은 대역인지
- `ros2 multicast receive` (한쪽) / `ros2 multicast send` (다른 쪽) → `Received from ...` 이 나와야 노드가 서로 보인다
- `timedatectl` → `System clock synchronized: yes`. 시계가 어긋나면 메시지 시각이 그만큼 틀린다 (link_check 의 '찍힌 시각 → 받은 시각')
- 자세 파일은 팔 Pi 의 `data/poses.json`. 다른 곳에서 저장한 자세는 `scp` 로 옮긴다

두 로봇의 TF 는 팔이 붙은 자리 하나로 잇는다 (m, 차 `base_link` 기준, 자로 잰 값):

```bash
ros2 run tf2_ros static_transform_publisher --x 0.05 --z 0.10 --frame-id base_link --child-frame-id so101/base_link
ros2 run tf2_ros tf2_echo base_footprint so101/gripper_frame_link     # 차 기준 그리퍼 끝
```

c4 는 팔 서비스를 차가 선 뒤에만, 시간 제한(찾기 5초 · 응답 20초)을 두고 부른다. 팔 Pi 가 없거나 중간에 꺼지면 '가 없다' · '응답 없음' 으로 멈추고 차는 움직이지 않는다.

팔을 차 위에 올릴 때: 팔 서보는 12 V (차 배터리는 2셀 8.4 V → 그대로 못 쓴다), 팔 Pi 는 5 V 전원을 따로, 장착 위치를 static TF 에 넣는다.

## 차 카메라로 색 판별

```bash
ros2 run so101_ros color_node --ros-args -r /camera/image_raw:=/ascamera/camera_publisher/rgb0/image
```

## 안 될 때

| 증상 | 확인 |
|---|---|
| 바퀴가 안 돈다, `/dev/rrc` 가 없다 | `lsusb \| grep 55d4`. STM 보드 USB 선 (데이터용) |
| `차 토픽이 안 들어온다` | 차에서 controller.launch.py, 양쪽 `ROS_DOMAIN_ID` |
| 노드 목록에 차가 없다 (차는 `docker_shell.sh` 로 띄움) | 차는 ubuntu, so101-lab 컨테이너는 root → FastDDS 공유 메모리가 서로 안 열린다. compose 가 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 를 넘긴다. 다른 셸에서는 직접 export |
| 팔: `모터 [1..6] 이 응답하지 않습니다` | 팔 12 V 어댑터. USB 만으로는 포트만 보인다 |
| 팔 포트가 차 보드로 열린다 | `/dev/serial/by-id/` 이름으로 |
| `/arm/go_pose ... 토크가 꺼진 관절` | `/arm/torque true` 먼저 (c4 는 한다) |
| WSL 에서 `ros2 node list` 가 멈춘다 | `ros2 daemon stop` 뒤 다시 |
| 다른 Pi 의 노드가 안 보인다 | 같은 공유기 · 대역인지, `ros2 multicast receive / send`, 양쪽 `ROS_DOMAIN_ID` |
| `자세 'home' 이 없습니다` | 자세 파일이 팔 Pi 에 없다 → 팔 Pi 에서 티칭 또는 `scp data/poses.json` |
| `/arm/... 응답 없음` | 팔 Pi 가 꺼졌거나 Wi-Fi. c4 는 20초 뒤 멈춘다 |
| link_check 의 시각 차가 수백 ms | 두 기계 시계 → `timedatectl` |

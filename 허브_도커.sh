#!/usr/bin/env bash
# 배달 한방 허브를 Docker(ROS2) 안에서 켭니다 → 4번 '바구니 넣기 학습' 에서 차에게 진짜 정지 · 출발 신호를 보낼 수 있어요.
#
# 사용 (Pi 터미널, 레포 폴더에서):
#     ./허브_도커.sh                     # 팔 포트 /dev/ttyACM0
#     ./허브_도커.sh /dev/ttyACM1        # 팔 포트가 다를 때
# 그다음 노트북 브라우저로  http://<Pi 의 IP>:8081   (Pi 의 IP 는  hostname -I)
#
# 먼저 확인
#   - 이미 켜진 허브가 있으면 끄세요 (포트는 한 프로그램만):  sudo ss -ltnp | grep 8081
#   - 팀 레포의 common_interfaces 폴더가 ros2/common_interfaces 에 있어야 신호가 나가요
#   - 팀 전용 ROS_DOMAIN_ID:  export ROS_DOMAIN_ID=<팀 번호>  (자동차 Pi 와 같은 값)
# 끝내기: 허브 화면의 '허브 끝내기' 또는 이 터미널에서 Ctrl+C
cd "$(dirname "$0")" || exit 1
PORT="${1:-/dev/ttyACM0}"
if [ ! -d ros2/common_interfaces ]; then
    echo "주의: ros2/common_interfaces 폴더가 없어요. 차 통신(ROS2)이 안 되고 '차 통신 없이' 로만 쓸 수 있어요."
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  팔 포트=$PORT"
exec docker compose -f docker/compose.yaml run --rm so101 python3 -m mission.hub --lan --port "$PORT"

#!/usr/bin/env bash
# 상차 미션(진짜)을 Docker(ROS2) 안에서 실행합니다. 차 전원이 켜지면(첫 상태 메시지) 팔이 대기 자세로 가서 기다려요.
#
# 사용 (Pi 터미널, 레포 폴더에서. 허브는 먼저 끄세요):
#     ./상차_도커.sh                     # 팔 포트는 설정 값(허브 6번)
#     ./상차_도커.sh --check             # 준비가 됐는지만 확인 (팔 안 움직임)
# 팀 전용 ROS_DOMAIN_ID:  export ROS_DOMAIN_ID=<팀 번호>  (자동차 Pi 와 같은 값)
# 끝내기: Ctrl+C  (대기 자세 → 토크 끄기 → 포트 닫기)
cd "$(dirname "$0")" || exit 1
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
exec docker compose -f docker/compose.yaml run --rm so101 python3 -m mission.loading "$@"

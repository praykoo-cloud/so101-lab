#!/bin/bash
# 컨테이너가 뜰 때마다: soarm 설치(처음 한 번) → ros2/ 빌드 (.ws, 바뀐 것만) → 명령 실행
set -e
source /opt/ros/humble/setup.bash
cd /root/so101-lab
# 레포 밖(/)에서 검사한다. 레포 안에서는 현재 폴더 때문에 import 가 되어 설치를 건너뛰게 된다
(cd / && python3 -c "import soarm" 2>/dev/null) || pip3 install -q --no-deps -e .
mkdir -p .ws/src
ln -sfn /root/so101-lab/ros2/so101_interfaces .ws/src/so101_interfaces
ln -sfn /root/so101-lab/ros2/so101_ros .ws/src/so101_ros
(cd .ws && colcon build --symlink-install > /tmp/colcon.log 2>&1) || { cat /tmp/colcon.log; exit 1; }
source .ws/install/setup.bash
exec "$@"

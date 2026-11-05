#!/bin/bash
# 컨테이너가 뜰 때마다: soarm 설치(처음 한 번) → ros2/ 빌드 (.ws, 바뀐 것만) → 명령 실행
# 준비는 root 로 하고, 명령은 레포 폴더의 주인(Pi 사용자, 보통 1000번)으로 실행한다.
# → 컨테이너 안에서 만든 파일도 Pi 사용자 것 (root 소유 파일 때문에 Pi 에서 못 고치는 일이 없다)
# compose.yaml 이 이 파일을 레포에서 직접 부른다 → 고쳐도 이미지를 다시 빌드할 필요 없음
set -e
source /opt/ros/humble/setup.bash
cd /root/so101-lab
# 레포 밖(/)에서 검사한다. 레포 안에서는 현재 폴더 때문에 import 가 되어 설치를 건너뛰게 된다
(cd / && python3 -c "import soarm" 2>/dev/null) || pip3 install -q --no-deps -e . 2>&1 | grep -v "Running pip as the 'root' user" || true
mkdir -p .ws/src
ln -sfn /root/so101-lab/ros2/so101_interfaces .ws/src/so101_interfaces
ln -sfn /root/so101-lab/ros2/so101_ros .ws/src/so101_ros
(cd .ws && colcon build --symlink-install > /tmp/colcon.log 2>&1) || { cat /tmp/colcon.log; exit 1; }
source .ws/install/setup.bash

uid=$(stat -c %u /root/so101-lab)
gid=$(stat -c %g /root/so101-lab)
if [ "$(id -u)" != 0 ] || [ "$uid" = 0 ] || [ "${SO101_ROOT:-0}" = 1 ]; then
    exec "$@"                                         # 레포가 root 것이거나 SO101_ROOT=1 이면 예전처럼 root 로
fi
name=${HOST_USER:-user}
chown -R "$uid:$gid" .ws ./*.egg-info 2>/dev/null || true
getent group "$gid" >/dev/null || groupadd -g "$gid" "$name"
if ! getent passwd "$uid" >/dev/null; then
    useradd -u "$uid" -g "$gid" -G dialout,video,plugdev -m -s /bin/bash "$name"   # dialout = 팔 USB, video = 카메라
    echo 'source /opt/ros/humble/setup.bash; [ -f /root/so101-lab/.ws/install/setup.bash ] && source /root/so101-lab/.ws/install/setup.bash; cd /root/so101-lab' \
        >> "/home/$name/.bashrc"
fi
chmod 755 /root                                        # 일반 사용자가 /root/so101-lab 까지 들어갈 수 있게
user=$(getent passwd "$uid" | cut -d: -f1)
export HOME=$(getent passwd "$uid" | cut -d: -f6) USER="$user"
exec setpriv --reuid="$uid" --regid="$gid" --init-groups "$@"

# 라즈베리파이 5 (Raspberry Pi OS 64비트) 에서 ROS2 Humble 을 쓰기 위한 이미지.
# Pi 5 는 Ubuntu 22.04 를 공식 지원하지 않아 Humble 을 바로 깔 수 없다 → Ubuntu 22.04 + Humble 컨테이너.
# ros:humble 이미지는 arm64 · amd64 둘 다 있어 같은 파일로 PC 에서도 빌드된다.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-opencv python3-numpy python3-serial python3-pytest \
        ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# 레포는 실행할 때 /root/so101-lab 에 붙인다 (compose.yaml). 자세 · 카메라 설정이 Pi 쪽 폴더에 남는다
WORKDIR /root/so101-lab
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /opt/ros/humble/setup.bash; [ -f /root/so101-lab/.ws/install/setup.bash ] && source /root/so101-lab/.ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

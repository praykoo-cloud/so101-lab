"""차 1 — 센서 읽기 (차는 움직이지 않는다).

    python3 car/c1_sensors.py          # 0.5 초마다 한 줄, Ctrl+C 로 끝

배터리 · IMU 방향(yaw) · 오도메트리 위치 · 라이다 앞/왼/오/뒤 가장 가까운 거리를 보여 준다.
해 볼 것: 차를 손으로 돌려 yaw 가 바뀌는지, 손을 차 앞 30 cm 에 대 front 가 바뀌는지 본다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carlib import Car  # noqa: E402


def cm(r):
    return "  -  " if r is None else f"{r * 100:4.0f}cm"


car = Car("c1_sensors")
try:
    car.wait_ready()
    print("배터리 2셀 리튬: 8.4 V 가득, 7.0 V 아래면 충전")
    while True:
        car.spin(0.5)
        print(f"배터리 {car.battery_mv / 1000:.2f} V   yaw {car.yaw_deg:+6.1f}°   odom x {car.x:+.3f} y {car.y:+.3f}   "
              f"앞 {cm(car.front())}  왼 {cm(car.angle_range(75, 105))}  "
              f"오 {cm(car.angle_range(-105, -75))}  뒤 {cm(car.angle_range(165, 180) or car.angle_range(-180, -165))}",
              flush=True)
except KeyboardInterrupt:
    pass
finally:
    car.close()

"""차 4 — 차와 팔을 한 프로그램에서: 다가가기 → 팔 동작 → 돌아오기.

    python3 car/c4_car_and_arm.py --dry      # 차는 움직이지 않고 팔만 (그리퍼 열고 닫기)
    python3 car/c4_car_and_arm.py

필요한 것 (모두 같은 ROS_DOMAIN_ID)
  차 컨테이너   ros2 launch controller controller.launch.py  +  ros2 launch peripherals lidar.launch.py
  팔            ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0 namespace:=so101
  이 파일       so101-lab 컨테이너 안 (so101_interfaces 가 빌드되어 있어야 한다)

순서: 팔 토크 켜기 → 팔 home → 차가 앞으로 (앞 stop 거리 또는 최대 거리) → 그리퍼 열기 · 닫기 → 차가 같은 만큼 뒤로 → 팔 home
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carlib import Car  # noqa: E402

import rclpy  # noqa: E402
from so101_interfaces.srv import GoPose, SetGripper  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--stop", type=float, default=0.30)
ap.add_argument("--speed", type=float, default=0.05)
ap.add_argument("--max", type=float, default=0.30, help="최대 전진 거리 (m)")
ap.add_argument("--dry", action="store_true", help="차는 움직이지 않는다")
args = ap.parse_args()

car = Car("c4_car_and_arm")
go = car.node.create_client(GoPose, "/arm/go_pose")
grip = car.node.create_client(SetGripper, "/arm/gripper")
torque = car.node.create_client(SetBool, "/arm/torque")


def call(client, req, timeout=20.0):
    """서비스를 부르고 응답을 기다린다 (그동안 차 토픽도 계속 받는다)"""
    fut = client.call_async(req)
    end = time.time() + timeout
    while not fut.done() and time.time() < end and rclpy.ok():
        rclpy.spin_once(car.node, timeout_sec=0.05)
    if not fut.done():
        raise RuntimeError(f"{client.srv_name} 응답 없음")
    res = fut.result()
    if not res.success:
        raise RuntimeError(f"{client.srv_name}: {res.message}")
    return res


def cm(r):
    return "-" if r is None else f"{r * 100:.0f} cm"


def say(text):
    print(time.strftime("%H:%M:%S"), text, flush=True)


try:
    car.wait_ready(lidar=True)
    for c in (go, grip, torque):
        if not c.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{c.srv_name} 가 없다. 팔에서 arm.launch.py 가 돌고 있는지, ROS_DOMAIN_ID 가 같은지")
    say(f"준비: 배터리 {car.battery_mv / 1000:.2f} V, 앞 {cm(car.front())}")

    call(torque, SetBool.Request(data=True))
    r = call(go, GoPose.Request(name="home", speed=0))
    say(f"팔 home: {r.message}")

    x0, y0 = car.x, car.y
    if args.dry:
        say("dry: 차는 그대로")
    else:
        def arrived():
            d = car.front()
            moved = ((car.x - x0) ** 2 + (car.y - y0) ** 2) ** 0.5
            return d is None or d < args.stop or moved >= args.max      # 앞이 안 보이면(None) 멈춤
        car.drive(vx=args.speed, sec=args.max / args.speed + 1, until=arrived)
        car.spin(0.5)
    moved = ((car.x - x0) ** 2 + (car.y - y0) ** 2) ** 0.5
    say(f"차 도착: {moved * 100:.1f} cm 이동, 앞 {cm(car.front())}")

    for pct in (40.0, 5.0):
        r = call(grip, SetGripper.Request(percent=pct))
        say(f"그리퍼 {pct:.0f}%: {r.message} (부하 {r.load})")

    if not args.dry and moved > 0.005:
        car.drive(vx=-args.speed, sec=moved / args.speed)
        car.spin(0.5)
        say(f"차 돌아옴: 시작점에서 {((car.x - x0) ** 2 + (car.y - y0) ** 2) ** 0.5 * 100:.1f} cm")

    r = call(go, GoPose.Request(name="home", speed=0))
    say(f"팔 home: {r.message}. 끝")
except KeyboardInterrupt:
    say("Ctrl+C → 차 멈춤")
except RuntimeError as e:
    say(f"중단: {e}")
finally:
    car.close()

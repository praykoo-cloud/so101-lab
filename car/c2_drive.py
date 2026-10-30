"""차 2 — 정해진 만큼 움직이기 (cmd_vel).

    python3 car/c2_drive.py            # 앞 3 cm → 뒤 → 왼쪽 옆 3 cm → 오른쪽 옆 → 왼쪽 15° → 오른쪽 15°
    python3 car/c2_drive.py --dist 0.05 --turn 30

차는 책상 위에 둔다. 가장자리에서 30 cm 넘게 떨어뜨리고, 거리는 몇 cm 씩만. 메카넘 바퀴라 옆으로도 간다 (vy).
명령한 거리 · 각도와, 오도메트리 · IMU 가 잰 값을 나란히 찍는다.
오도메트리는 명령한 속도를 더한 값이라 미끄러져도 모른다. IMU 회전은 실제로 잰 값이다.
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carlib import Car  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--speed", type=float, default=0.03, help="m/s (0.2 까지)")
ap.add_argument("--dist", type=float, default=0.03, help="m")
ap.add_argument("--turn", type=float, default=15, help="도")
args = ap.parse_args()

car = Car("c2_drive")
sec = args.dist / args.speed
w = 0.5                                     # rad/s
tsec = math.radians(args.turn) / w


def step(label, **v):
    x0, y0, yaw0 = car.x, car.y, car.yaw_deg
    car.drive(sec=tsec if "wz" in v else sec, **v)
    car.spin(0.5)                           # 멈춘 뒤 값이 자리 잡을 때까지
    dyaw = (car.yaw_deg - yaw0 + 180) % 360 - 180
    print(f"{label:<14} odom {math.hypot(car.x - x0, car.y - y0) * 100:5.1f} cm   IMU 회전 {dyaw:+6.1f}°", flush=True)


try:
    car.wait_ready()
    print(f"속도 {args.speed} m/s, {args.dist * 100:.0f} cm 씩 ({sec:.1f} 초), 회전 {args.turn}° ({tsec:.1f} 초)")
    step("앞으로", vx=args.speed)
    step("뒤로", vx=-args.speed)
    step("왼쪽 옆으로", vy=args.speed)
    step("오른쪽 옆으로", vy=-args.speed)
    step(f"왼쪽 {args.turn:.0f}° 회전", wz=w)
    step(f"오른쪽 {args.turn:.0f}° 회전", wz=-w)
except KeyboardInterrupt:
    print("Ctrl+C → 멈춤")
finally:
    car.close()

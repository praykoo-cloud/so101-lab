"""차 3 — 라이다로 앞을 보고 멈추기.

    python3 car/c3_lidar_stop.py --dry       # 움직이지 않고 판단만 찍는다
    python3 car/c3_lidar_stop.py             # 앞 장애물이 30 cm 가 될 때까지 천천히 앞으로 (최대 10 cm)

규칙 세 개: 앞(±15°) 가장 가까운 거리가 stop 보다 가까우면 멈춘다. 앞에 잡힌 점이 하나도 없으면 멈춘다
(라이다는 책상 끝 같은 낭떠러지를 못 본다). 라이다 값이 0.5 초 넘게 안 오면 멈춘다.
해 볼 것: 상자를 차 앞 35 cm 에 놓고 실행 → 5 cm 가서 30 cm 앞에서 서는지. --stop 0.2 로 바꿔 본다.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carlib import Car  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--stop", type=float, default=0.30, help="이 거리(m)보다 가까우면 멈춘다")
ap.add_argument("--speed", type=float, default=0.03)
ap.add_argument("--max", type=float, default=0.10, help="최대 이동 거리 (m). 책상 위라 작게")
ap.add_argument("--dry", action="store_true", help="움직이지 않고 판단만")
args = ap.parse_args()

car = Car("c3_lidar_stop")
reason = "30 초 지남 (dry)" if args.dry else "최대 거리"
try:
    car.wait_ready(lidar=True)
    x0, y0 = car.x, car.y
    last_scan, last_stamp = None, time.time()
    end = time.time() + (30 if args.dry else args.max / args.speed + 1)
    while time.time() < end:
        car.spin(0.1)
        if car.scan is not last_scan:
            last_scan, last_stamp = car.scan, time.time()
        d = car.front()
        moved = ((car.x - x0) ** 2 + (car.y - y0) ** 2) ** 0.5
        if time.time() - last_stamp > 0.5:
            reason = "라이다 값이 끊김"
            break
        if d is None:                       # 앞에 잡힌 점이 없다: 비었는지, 낭떠러지(책상 끝)인지 모른다
            blocked = "앞에 잡힌 점이 없다 (모르면 멈춤)"
        elif d < args.stop:
            blocked = f"앞 {d * 100:.0f} cm < {args.stop * 100:.0f} cm"
        else:
            blocked = None
        if blocked:
            reason = blocked
            if not args.dry:
                break
        if moved >= args.max:
            break
        print(f"앞 {'-' if d is None else f'{d * 100:.0f}'} cm  → {'멈춤' if blocked else '전진'}"
              f"{' (dry)' if args.dry else ''}   간 거리 {moved * 100:.0f} cm", flush=True)
        if not args.dry:
            car.send(vx=args.speed)
except KeyboardInterrupt:
    reason = "Ctrl+C"
finally:
    car.close()
    print("멈춤:", reason)

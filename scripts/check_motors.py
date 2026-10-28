"""모터 6개 점검 (읽기만 함 — 토크 · 위치를 바꾸지 않는다).

    python scripts/check_motors.py COM4

전압으로 팔을 구분할 수 있다: 팔로워 약 12V, 리더 약 5V.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from soarm.driver_sdk import JOINT_IDS, JOINT_LIMITS, JOINT_NAMES, STS3215Driver  # noqa: E402

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
arm = STS3215Driver(port=port)
if not arm.connect():
    sys.exit(f"{port} 을 열 수 없습니다 (포트 이름 · 다른 프로그램이 쓰는지 확인)")
try:
    print(f"{port}")
    print(f"  {'ID':<3}{'관절':<13}{'응답':<5}{'위치':>6}{'각도':>8}{'범위':>13}{'토크':>5}{'전압':>7}{'온도':>5}")
    volts = []
    for sid in JOINT_IDS:
        if not arm.ping(sid):
            print(f"  {sid:<3}{JOINT_NAMES[sid-1]:<13}없음")
            continue
        pos = arm.get_position(sid)
        v = arm.get_voltage(sid)
        volts.append(v or 0)
        lim = JOINT_LIMITS[sid]
        print(f"  {sid:<3}{JOINT_NAMES[sid-1]:<13}{'OK':<5}{pos:>6}{arm.position_to_degrees(pos):>8.1f}"
              f"{lim['min']:>6}~{lim['max']:<6}{'켜짐' if arm.get_torque(sid) else '꺼짐':>5}{v:>6.1f}V{arm.get_temperature(sid):>4}℃")
    if not volts:
        print("\n  모터가 하나도 응답하지 않습니다. 팔 전원(어댑터)과 3핀 케이블을 확인하세요"
              " (USB 만 꽂으면 포트는 보이지만 모터는 응답하지 않습니다)")
    else:
        avg = sum(volts) / len(volts)
        print(f"\n  평균 전압 {avg:.1f}V → {'팔로워 (12V)' if avg > 9 else '리더 (5V)'} 로 보입니다")
finally:
    arm.disconnect()

"""팔이 꽂힌 포트 찾기.

    python scripts/find_port.py           보이는 포트 목록
    python scripts/find_port.py --unplug  USB 를 뽑았다 꽂아서 어느 팔인지 찾기 (lerobot-find-port 와 같은 방법)
"""
import sys
import time

from serial.tools import list_ports


def ports():
    return {p.device: p for p in list_ports.comports()}


before = ports()
print("보이는 포트:")
for dev, p in sorted(before.items()):
    print(f"  {dev:14s} {p.description}")

if "--unplug" in sys.argv:
    print("\n찾을 팔의 USB 를 뽑으세요 (30초 기다림) ...")
    t0 = time.time()
    while time.time() - t0 < 30:
        gone = set(before) - set(ports())
        if gone:
            print(f"→ 이 팔의 포트: {gone.pop()}   (USB 를 다시 꽂으세요)")
            break
        time.sleep(0.2)
    else:
        print("사라진 포트가 없습니다. 다시 실행하세요")

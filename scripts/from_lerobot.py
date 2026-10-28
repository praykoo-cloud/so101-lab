"""LeRobot 캘리브레이션 파일 → data/calibration.json (모터에는 아무것도 쓰지 않는다).

LeRobot 으로 이미 캘리브레이션한 팔은 모터 EEPROM 에 가운데(2048) 기준과 관절 범위가 들어 있다.
그 파일의 range_min / range_max 를 컨트롤러 앱 · 이 SDK 의 min / max 로 옮긴다.
scripts/calibrate.py 를 다시 돌리면 EEPROM 의 가운데 기준을 새로 쓰므로 LeRobot 캘리브레이션과 어긋난다.

    python scripts/from_lerobot.py follower                      # robots/so_follower/follower.json
    python scripts/from_lerobot.py leader --leader               # teleoperators/so_leader/leader.json
"""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

ap = argparse.ArgumentParser()
ap.add_argument("id", help="LeRobot 캘리브레이션 때 쓴 id (예: follower)")
ap.add_argument("--leader", action="store_true", help="리더 팔 파일에서")
ap.add_argument("--file", help="캘리브레이션 json 경로를 직접 줄 때")
ap.add_argument("--out", default=str(ROOT / "data" / "calibration.json"))
a = ap.parse_args()

base = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration"
src = Path(a.file) if a.file else base / ("teleoperators/so_leader" if a.leader else "robots/so_follower") / f"{a.id}.json"
lr = json.loads(src.read_text(encoding="utf-8"))

cal = {"robot": a.id, "source": str(src), "joints": {}}
for sid, name in enumerate(NAMES, start=1):
    m = lr[name]
    cal["joints"][str(sid)] = {"min": int(m["range_min"]), "max": int(m["range_max"]),
                               "homing_offset": int(m["homing_offset"])}

out = Path(a.out)
out.parent.mkdir(parents=True, exist_ok=True)
if out.exists():
    out.with_suffix(".bak.json").write_text(out.read_text(encoding="utf-8"), encoding="utf-8")
out.write_text(json.dumps(cal, indent=2), encoding="utf-8")
print(f"{src}\n→ {out}")
for sid, name in enumerate(NAMES, start=1):
    j = cal["joints"][str(sid)]
    print(f"  {sid} {name:<14} {j['min']:>5} ~ {j['max']:<5}")

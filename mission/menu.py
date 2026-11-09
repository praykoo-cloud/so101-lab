"""로봇팔 미션 한방 메뉴 — 번호만 누르면 해당 도구를 실행하고, 끝나면 메뉴로 돌아온다.

    미션_한방.cmd 더블클릭 (Windows)        ← 실물 팔 (이 PC 에 꽂았을 때)
    미션_한방.cmd --studio                  ← ArmStudio 가상 팔 · 가상 카메라
    미션_한방.cmd --fake                    ← 팔 없이 (가상 팔)
    ./미션_한방.sh                           ← Pi (리눅스)

직접:  python -m mission.menu [--studio | --fake | --port COM4]

메뉴 맨 위에 "지금 상태" 와 "다음에 할 일" 이 나온다. 처음이면 위에서부터 차례로 하면 된다.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PY = sys.executable
os.environ["PYTHONUTF8"] = "1"      # 하위 프로그램이 json 을 UTF-8 로 읽게 (한글 윈도우 cp949 오류 막기)

MENU = """
==================== 로봇팔 배송 미션 한방 메뉴 ====================
  1  🩺 점검: 지금 무엇이 되고 안 되는지 (팔을 안 움직임)   ← 처음엔 이것
  2  📷 카메라 번호 찾기 · 사진 찍기
  3  🦾 자세 티칭: 팔을 손으로 잡아 저장 (wait, above_red … + 공 쥔 그리퍼 %)
  4  ▭  구역 티칭: 카메라 화면에 네모 그리기 (도착 · 포스트잇 · 받침대 · 바구니 …)
  5  🎨 색 티칭: 빨강 · 파랑 포스트잇과 공을 카메라로 보여 주기
  6  👀 색 확인: 지금 화면의 포스트잇을 판정해 보기
  7  🔒 기준색 잠금 / 해제 (주문 배정 전에만!)
  8  🧪 가상 연습: 팔 · 카메라 · ROS2 없이 시나리오로 흐름 보기
  9  ▶  미션 연습 실행 (ROS2 없이)
 10  🚀 미션 진짜 실행 (팔 + 카메라 + ROS2)   ← 규정대로 대회 방식
 11  🗂  작업 기록 만들기 (최초 한 번)
  0  끝
===================================================================="""


class Mode:
    def __init__(self, studio=False, fake=False, port=None):
        self.studio, self.fake, self.port = studio, fake, port

    def label(self):
        if self.studio:
            return "ArmStudio 가상 팔 · 가상 카메라 (앱을 켜 두세요)"
        if self.fake:
            return "가상 팔 (실물 · 포트 필요 없음)"
        return f"실물 팔 ({self.port or '설정 / 자동'})"

    def arm_args(self):                       # 자세 티칭 · 팔을 쓰는 도구용
        if self.studio:
            return ["--studio"]
        if self.fake:
            return ["--port", "fake"]
        return ["--port", self.port] if self.port else []

    def cam_args(self):                       # 카메라를 쓰는 도구용
        return ["--studio"] if self.studio else []

    def run_args(self):                       # mission_main 용
        return ["--studio"] if self.studio else []


def ask(msg):
    try:
        return input(msg).strip()
    except (KeyboardInterrupt, EOFError):
        print()
        return "0"


def yes(msg):
    return ask(msg + " [y/N] ").lower()[:1] in ("y", "ㅛ")


def console_mode(mode=None):
    """윈도우 터미널 입력 설정 읽기(mode=None) / 되돌리기. 하위 프로그램이 바꿔 놓고 끝나도 원래대로"""
    if os.name != "nt":
        return None
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        h = k32.GetStdHandle(-10)
        if mode is None:
            m = ctypes.c_uint32()
            return m.value if k32.GetConsoleMode(h, ctypes.byref(m)) else None
        k32.SetConsoleMode(h, mode)
        k32.FlushConsoleInputBuffer(h)
    except Exception:
        return None


def run(module, *args, env=None):
    cmd = [PY, "-m", f"mission.{module}", *map(str, args)]
    print("\n  ▶ " + " ".join(["python", "-m", f"mission.{module}", *map(str, args)]))
    saved = console_mode()
    try:
        subprocess.run(cmd, cwd=ROOT, env={**os.environ, **(env or {})})
    except KeyboardInterrupt:
        print("\n  (Ctrl+C 로 멈췄습니다)")
    if saved is not None:
        console_mode(saved)
    print("  (입력이 안 되면 이 까만 창을 마우스로 한 번 클릭하세요)")


def careful(msg="팔이 움직입니다. 주변을 비웠나요? 전원 스위치에 손이 닿나요?"):
    return yes("  " + msg)


# ── 상태 표시 ───────────────────────────────────────────────────────────
def status_block(mode):
    try:
        from .doctor import collect, next_step, summary_line
        rows = collect(mode.studio, mode.fake, check_camera=False)      # 메뉴에서는 카메라를 안 열어 본다 (느림)
        return f"  지금 상태: {summary_line(rows)}\n  다음에 할 일: {next_step(rows)}"
    except Exception as e:
        return f"  (상태를 읽지 못했습니다: {e})"


def pick(items, label, default_first=True):
    """items: [(이름, 완료 여부, 설명)] → 고른 이름 (Enter = 아직 안 한 첫 번째)"""
    first_todo = next((n for n, done, _ in items if not done), items[0][0])
    for i, (n, done, desc) in enumerate(items, 1):
        print(f"   {i:2d} [{'✔' if done else ' '}] {n:14s} {desc}")
    ans = ask(f"  {label} 번호 (Enter = {first_todo}, 0 = 취소): ")
    if ans == "0":
        return None
    if ans == "":
        return first_todo
    if ans.isdigit() and 1 <= int(ans) <= len(items):
        return items[int(ans) - 1][0]
    print("  번호를 다시 확인하세요")
    return None


def do_cameras(mode):
    if mode.studio:
        print("  ArmStudio 가상 카메라는 번호가 필요 없습니다 (손목 카메라 wrist).")
        return
    k = ask("  1 카메라 번호 찾기 (0~9 열어 보기) / 2 사진 찍기 : ")
    if k == "1":
        run("m0_capture_frames", "--scan")
    elif k == "2":
        label = ask("  파일 이름 앞부분 (예: wait, note_red, stand_red): ") or "shot"
        run("m0_capture_frames", "--label", label, "--count", "5")


def do_rois(mode):
    from .config import ROI_HELP, MissionConfig
    cfg = MissionConfig.load()
    items = [(n, cfg.roi_taught(n), h) for n, h in ROI_HELP.items()]
    name = pick(items, "구역")
    if not name:
        return
    print("  이 구역을 볼 수 있는 자세로 팔을 보내 두세요 (설명 참고). 창에서 드래그 → Enter, 건너뛰기 c")
    img = ask("  사진 파일로 하려면 경로 입력 (Enter = 카메라): ")
    run("m0_teach_rois", "--only", name, *(["--image", img] if img else mode.cam_args()))


def do_colors(mode):
    from .vision import COLORS_FILE, REF_NAMES, ColorRefs, is_locked
    if is_locked(COLORS_FILE):
        print("  기준색이 잠겨 있습니다. 고치려면 먼저 7번에서 해제 (주문 배정 전에만)")
        return
    refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
    desc = {"note_red": "빨간 포스트잇", "note_blue": "파란 포스트잇", "ball_red": "빨간 공 (받침대에 놓인 상태)",
            "ball_blue": "파란 공 (받침대에 놓인 상태)"}
    name = pick([(n, n in refs.refs, desc[n]) for n in REF_NAMES], "등록할 것")
    if not name:
        return
    print("  화면이 뜨면 스페이스 → 그 사진에서 색 영역을 드래그 → Enter. 조명이 다를 때마다 한 번 더 하면 더 튼튼해집니다.")
    img = ask("  사진 파일로 하려면 경로 입력 (Enter = 카메라): ")
    run("m0_teach_colors", "teach", name, *(["--image", img] if img else mode.cam_args()))


def do_lock(mode):
    k = ask("  1 잠그기 / 2 풀기 (주문 배정 전에만) / 3 상태 : ")
    if k == "1":
        print("  ★ 빨강 · 파랑을 둘 다 등록하고 6번으로 검증한 뒤에 잠그세요. 잠그면 경기 중 수정 금지입니다.")
        if yes("  잠글까요?"):
            run("m0_teach_colors", "lock")
    elif k == "2":
        run("m0_teach_colors", "unlock")
    elif k == "3":
        run("m0_teach_colors", "status")


def do_sim():
    from .sim import SCENARIOS
    names = list(SCENARIOS)
    for i, n in enumerate(names, 1):
        print(f"   {i:2d} {n:18s} {SCENARIOS[n]}")
    k = ask("  시나리오 번호 (Enter = 1): ") or "1"
    if k.isdigit() and 1 <= int(k) <= len(names):
        run("mission_main", "--sim", names[int(k) - 1])


def main(argv=None):
    ap = argparse.ArgumentParser(description="로봇팔 미션 한방 메뉴")
    ap.add_argument("--studio", action="store_true", help="ArmStudio 가상 팔 · 카메라")
    ap.add_argument("--fake", action="store_true", help="가상 팔 (포트 없이)")
    ap.add_argument("--port", default=None, help="팔 포트 (예: COM4, /dev/ttyACM0)")
    args = ap.parse_args(argv)
    mode = Mode(args.studio, args.fake, args.port)
    if mode.fake:
        os.environ["SOARM_PORT"] = "fake"
    print(f"\n  모드: {mode.label()}")
    if not mode.studio and not mode.fake:
        print("  * ArmStudio 앱은 꺼 주세요 (포트는 한 프로그램만 씀)")
    while True:
        print(MENU)
        print(status_block(mode))
        k = ask("\n  번호: ")
        if k == "0":
            break
        elif k == "1":
            run("doctor", *(["--studio"] if mode.studio else ["--fake"] if mode.fake else []))
        elif k == "2":
            do_cameras(mode)
        elif k == "3":
            print("  off → 손으로 받치고 팔을 옮김 → save 이름 → … (list 로 남은 자세 확인, 마지막에 공을 물리고 ballgrip, q 끝)")
            if mode.studio or yes("  팔 토크를 끄고 손으로 움직이게 됩니다. 시작할까요?"):
                run("m0_teach_poses", *mode.arm_args())
        elif k == "4":
            do_rois(mode)
        elif k == "5":
            do_colors(mode)
        elif k == "6":
            run("m0_teach_colors", "verify", *mode.cam_args())
        elif k == "7":
            do_lock(mode)
        elif k == "8":
            do_sim()
        elif k == "9":
            if mode.studio or mode.fake or careful():
                print("  ROS2 없이 팔 + 카메라로 한 사이클씩 (Ctrl+C 로 멈춤). 첫 실행은 느리게, 전원 스위치에 손을 두세요.")
                run("mission_main", "--no-ros", *mode.run_args())
        elif k == "10":
            if mode.studio:
                print("  진짜 실행은 실물 팔 모드에서만 합니다 (--studio 없이 실행).")
            elif mode.fake:
                print("  가상 팔 모드에서는 진짜 실행을 하지 않습니다.")
            else:
                print("  ★ 대회 방식: 기준색이 잠겨 있어야 하고, 자동차 Pi 와 같은 ROS_DOMAIN_ID 여야 합니다. 사람 신호는 없습니다.")
                if careful():
                    run("mission_main")
        elif k == "11":
            run("mission_main", "--init-store")
    print("끝!")


if __name__ == "__main__":
    main()

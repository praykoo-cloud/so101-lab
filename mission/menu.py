"""로봇팔 미션 한방 — 브라우저 허브(기본) 또는 터미널 메뉴(--menu). 실물 카메라 · 실물 팔로만 동작한다.

    ./"배달 한방.sh"            (Pi)  → 브라우저 허브가 열린다: 1~12번을 화면에서 골라 설정
    ./"배달 한방.sh" --menu           → 예전 터미널 메뉴
    미션_한방.cmd                (PC)  → 같은 허브 (PC 에 팔 · 카메라를 꽂았을 때)

직접:  python -m mission.menu [--menu] [--port /dev/ttyACM0]

허브: VS Code 의 Ports 에서 8081 을 전달하고 노트북 브라우저로 http://127.0.0.1:8081 을 연다.
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
  2  📷 손목 카메라 보기 · 번호 찾기 · 사진 찍기
  3  🦾 자세 티칭: 손목 카메라를 보면서 자세 저장
  4  ▭  구역 설정: 카메라 화면에 네모 그리기 (도착 · 포스트잇 · 받침대 · 바구니 …)
  5  🎨 색상 등록: 빨강 · 파랑 포스트잇과 공을 카메라로 보여 주기
  6  👀 색상 확인: 지금 화면의 포스트잇을 판정해 보기
  7  🔒 기준색 잠금 / 해제 (주문 배정 전에만!)
  8  ⚙  설정 값 · 저장 파일: 오차 범위 · 시간 · 구역 · 그리퍼 % 를 보고 직접 수정
  9  ▶  미션 연습 실행 (ROS2 없이, 팔 + 카메라)
 10  🚀 미션 진짜 실행 (팔 + 카메라 + ROS2)   ← 규정대로 대회 방식
 11  🗂  작업 기록: 만들기 / 보기 (최초 한 번)
 12  🚘 차 도착 확인: 상차 구역 안에 멈춘 차를 카메라로 확인 (팔은 움직이지 않음)
 13  🌐 브라우저 허브: 1~12번을 화면에서 골라 들어가기
  0  끝
===================================================================="""


class Mode:
    def __init__(self, port=None):
        self.port = port

    def label(self):
        return f"실물 팔 ({self.port or '설정 / 자동'})"

    def hub_args(self):
        return ["--port", self.port] if self.port else []


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


def status_block():
    try:
        from .doctor import collect, next_step, summary_line
        rows = collect(False, False, check_camera=False)      # 메뉴에서는 카메라를 안 열어 본다 (느림)
        return f"  지금 상태: {summary_line(rows)}\n  다음에 할 일: {next_step(rows)}"
    except Exception as e:
        return f"  (상태를 읽지 못했습니다: {e})"


def open_hub(mode, start=None):
    """브라우저 허브를 연다 (start = 바로 열 화면). 끝내려면 허브의 '허브 끝내기' 또는 Ctrl+C"""
    run("hub", *mode.hub_args(), *(["--start", start] if start else []))


def do_cameras(mode):
    k = ask("  Enter = 브라우저로 보기 / 1 번호 찾기 / 2 사진 찍기 / 0 돌아가기: ")
    if not k:
        open_hub(mode, "view")
    elif k == "1":
        run("m0_capture_frames", "--scan")
    elif k == "2":
        label = ask("  파일 이름 앞부분 (예: wait, note_red, stand_red): ") or "shot"
        run("m0_capture_frames", "--label", label, "--count", "5")


def terminal_menu(mode):
    print(f"\n  모드: {mode.label()}")
    print("  * ArmStudio 같은 다른 프로그램이 팔 포트를 쓰고 있으면 끄세요 (포트는 한 프로그램만 씀)")
    starts = {"3": "pose", "4": "roi", "5": "color", "6": "verify", "7": "lock", "8": "settings",
              "11": "store", "12": "arrival", "13": None}
    while True:
        print(MENU)
        print(status_block())
        k = ask("\n  번호: ")
        if k == "0":
            break
        elif k == "1":
            run("doctor")
        elif k == "2":
            do_cameras(mode)
        elif k in starts:
            open_hub(mode, starts[k])
        elif k == "9":
            if careful():
                print("  ROS2 없이 팔 + 카메라로 한 사이클씩 (Ctrl+C 로 멈춤). 첫 실행은 느리게, 전원 스위치에 손을 두세요.")
                run("mission_main", "--no-ros")
        elif k == "10":
            print("  ★ 대회 방식: 기준색이 잠겨 있어야 하고, 자동차 Pi 와 같은 ROS_DOMAIN_ID 여야 합니다. 경기 중 사람 신호 · SSH · 브라우저는 쓰지 않습니다.")
            if careful():
                run("mission_main")
    print("끝!")


def main(argv=None):
    ap = argparse.ArgumentParser(description="로봇팔 미션 한방")
    ap.add_argument("--menu", action="store_true", help="브라우저 허브 대신 터미널 메뉴")
    ap.add_argument("--port", default=None, help="팔 포트 (예: COM4, /dev/ttyACM1)")
    args = ap.parse_args(argv)
    mode = Mode(args.port)
    if args.menu:
        terminal_menu(mode)
        return 0
    from .hub import main as hub_main
    return hub_main(mode.hub_args())


if __name__ == "__main__":
    raise SystemExit(main())

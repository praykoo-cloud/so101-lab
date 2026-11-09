"""자세 티칭: 팔을 손으로 움직여서 자세를 외우게 한다 (data/poses.json 에 저장).

실행 (실물 팔을 꽂은 PC / Pi):
    python -m mission.m0_teach_poses                 # 설정의 포트 사용
    python -m mission.m0_teach_poses --port fake     # 연습 (가상 팔)
    python -m mission.m0_teach_poses --port COM4

순서
  1) off    → 토크를 끈다 (팔이 힘을 뺀다: 손으로 받치고 있어야 떨어지지 않는다!)
  2) 손으로 팔을 원하는 자세로 옮긴다
  3) save wait  → 지금 자세를 'wait' 라는 이름으로 저장
  4) 필요한 자세를 모두 반복 (list 로 남은 자세를 확인)
  5) on / go NAME 으로 저장한 자세를 다시 가 본다 (천천히, 주변을 비우고)
  6) 그리퍼에 3 cm 공을 물린 채로 ballgrip  → "공을 쥐었을 때의 그리퍼 %" 를 외운다 (집기 성공 판정에 쓴다)

명령: off | on | save NAME | go NAME | list | grip N (0닫힘~100열림) | ballgrip | temp | q

주의
- 자세 사이는 팔이 부딪히지 않고 직선으로 오갈 수 있게 잡는다 (wait ↔ above_* ↔ * , above_basket ↔ basket ↔ retreat_mid).
- 포트는 한 프로그램만 쓴다. ArmStudio · 다른 터미널이 쓰면 'Device or resource busy' 가 난다.
- 끝낼 때(q 또는 Ctrl+C): 'wait' 가 저장돼 있으면 거기로 간 뒤 토크를 끄고 포트를 닫는다.
"""
from __future__ import annotations

import argparse

from soarm import Robot, RobotError

from .config import POSE_HELP, MissionConfig

BALL_TOLERANCE_PCT = 8.0            # 공을 쥔 % 에서 ± 이만큼을 "공을 쥠" 으로 본다


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="자세 티칭")
    ap.add_argument("--port", default=None, help="팔 포트 (기본: mission/config.json 의 port)")
    ap.add_argument("--studio", action="store_true", help="실물 대신 ArmStudio 앱의 가상 팔 (앱을 켜 둘 것)")
    args = ap.parse_args(argv)
    cfg = MissionConfig.load()
    port = args.port or cfg.port
    try:
        if args.studio:
            from .studio import StudioRobot
            bot = StudioRobot(speed=cfg.speed_slow)
        else:
            bot = Robot(port, speed=cfg.speed_slow)
    except Exception as e:
        print(f"팔에 연결하지 못했습니다: {e}")
        return 1
    print(f"연결: {bot}")

    def show_list():
        saved = bot.poses()
        for name, help_ in POSE_HELP.items():
            mark = "저장됨" if name in saved else "  --  "
            print(f"  [{mark}] {name:14s} {help_}")
        extra = [n for n in saved if n not in POSE_HELP]
        if extra:
            print("  (다른 자세)", ", ".join(extra))
        print(f"  공 쥔 그리퍼 %: {cfg.grip_ball_min_pct:.0f} ~ {cfg.grip_ball_max_pct:.0f}"
              + ("  (아직 티칭 안 함: ballgrip)" if cfg.grip_ball_max_pct <= 0 else ""))

    show_list()
    print("\n명령: off | on | save NAME | go NAME | list | grip N | ballgrip | temp | q")
    try:
        while True:
            try:
                line = input("티칭> ").strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *rest = line.split()
            cmd = cmd.lower()
            try:
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd == "off":
                    input("⚠ 팔이 힘을 빼면 떨어질 수 있습니다. 손으로 받치고 Enter ... ")
                    bot.torque(False)
                    print("토크 OFF: 손으로 움직일 수 있습니다")
                elif cmd == "on":
                    bot.torque(True)
                    print("토크 ON: 지금 자리에서 고정")
                elif cmd == "save":
                    if not rest:
                        print("사용법: save 이름   (예: save wait)")
                        continue
                    name = rest[0]
                    if name not in POSE_HELP:
                        print(f"  (참고: '{name}' 은 미션이 쓰는 이름이 아닙니다)")
                    bot.save_pose(name)
                    print(f"저장: {name}   {bot.angles()}")
                elif cmd == "go":
                    name = rest[0] if rest else ""
                    if not bot.is_locked():
                        print("토크가 꺼져 있습니다. on 을 먼저 하세요")
                        continue
                    input(f"'{name}' 으로 천천히 이동합니다. 주변이 비어 있나요? Enter ... ")
                    bot.go_pose(name, speed=cfg.speed_slow)
                    print("도착")
                elif cmd == "list":
                    show_list()
                elif cmd == "grip":
                    bot.gripper(float(rest[0]))
                    print(f"그리퍼 → {rest[0]}%")
                elif cmd == "ballgrip":
                    pct = bot.gripper_percent()
                    if pct is None:
                        print("그리퍼 값을 읽지 못했습니다")
                        continue
                    cfg.grip_ball_min_pct = max(0.0, pct - BALL_TOLERANCE_PCT)
                    cfg.grip_ball_max_pct = min(100.0, pct + BALL_TOLERANCE_PCT)
                    cfg.save()
                    print(f"공을 쥔 그리퍼 {pct:.1f}% → 범위 {cfg.grip_ball_min_pct:.0f}~{cfg.grip_ball_max_pct:.0f}% 저장")
                elif cmd == "temp":
                    print("모터 온도:", {sid: bot.driver.get_temperature(sid) for sid in range(1, 7)})
                else:
                    print("모르는 명령. off | on | save NAME | go NAME | list | grip N | ballgrip | temp | q")
            except (RobotError, ValueError, IndexError) as e:
                print(f"안 됐습니다: {e}")
            except Exception as e:                       # ArmStudio 연결 문제 등
                print(f"안 됐습니다 ({type(e).__name__}): {e}")
    except KeyboardInterrupt:
        print()
    finally:
        try:
            if bot.is_locked() and "wait" in bot.poses():
                print("정리: 대기 자세로 이동 → 토크 끄기 → 포트 닫기")
                bot.go_pose("wait", speed=cfg.speed_slow)
            bot.torque(False)
        except Exception as e:
            print(f"(정리 중 문제: {e})")
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

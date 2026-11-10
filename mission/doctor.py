"""점검: 지금 무엇이 되어 있고 무엇이 비었는지 한 번에 보여 준다 (팔을 움직이지 않는다, 읽기만).

    python -m mission.doctor                  # 실물 (설정의 포트)
    python -m mission.doctor --studio         # ArmStudio 가상 팔
    python -m mission.doctor --fake           # 가상 팔 (포트 점검 건너뜀)

각 줄: ✔ 됨 / ✘ 안 됨(다음에 할 일 적혀 있음) / ⚠ 참고
메뉴(미션_한방)의 1번이 이 점검을 부르고, 맨 위에 "다음에 할 일" 을 알려 준다.
"""
from __future__ import annotations

import argparse
import os
import sys

from .config import POSE_HELP, ROI_HELP, RUNTIME_DIR, MissionConfig

OK, BAD, WARN = "✔", "✘", "⚠"


def collect(studio: bool = False, fake: bool = False, check_camera: bool = True) -> list[tuple[str, str, str, str]]:
    """→ [(기호, 구분, 내용, 다음에 할 일)]"""
    rows: list[tuple[str, str, str, str]] = []

    def add(ok, group, text, hint=""):
        rows.append((OK if ok is True else WARN if ok is None else BAD, group, text, hint))

    cfg = MissionConfig.load()

    # 환경
    for mod in ("cv2", "numpy", "serial"):
        try:
            __import__(mod)
            add(True, "환경", f"{mod} 설치됨")
        except ImportError:
            add(False, "환경", f"{mod} 없음", f"설치: pip install {'opencv-python' if mod == 'cv2' else 'pyserial' if mod == 'serial' else mod}")
    try:
        import soarm  # noqa: F401
        add(True, "환경", "soarm(팔 SDK) 불러옴")
    except ImportError as e:
        add(False, "환경", f"soarm 을 못 불러옴: {e}", "so101-lab 폴더 안에서 실행하세요")

    # 팔
    poses_taught: set[str] = set()
    if studio:
        try:
            from .studio import StudioRobot
            bot = StudioRobot()
            add(True, "팔", "ArmStudio 앱 연결됨 (가상 팔)")
            poses_taught = set(bot.poses())
        except Exception as e:
            add(False, "팔", f"ArmStudio 에 연결 못 함: {str(e).splitlines()[0]}", "ArmStudio 앱을 켜세요")
    elif fake or str(cfg.port).lower() == "fake" or os.environ.get("SOARM_PORT", "").lower() == "fake":
        add(None, "팔", "가상 팔 모드 (실물 점검 건너뜀)")
        try:
            from soarm import Robot
            poses_taught = set(Robot("fake", connect=False).poses())
        except Exception:
            pass
    else:
        try:
            from soarm.robot import POSES_FILE, _load_json, default_port, find_ports
            ports = find_ports()
            port = cfg.port or default_port()
            if port:
                add(True, "팔", f"팔 포트: {port}")
            elif ports:
                add(None, "팔", f"팔 보드로 보이는 포트가 여러 개: {ports}", "mission/config.json 의 port 에 하나를 적으세요")
            else:
                add(False, "팔", "팔 포트가 보이지 않습니다", "USB 케이블 · 팔로워 12V 전원 확인 (리눅스: ls /dev/ttyACM*)")
            poses_taught = set(_load_json(POSES_FILE, {}))
        except Exception as e:
            add(False, "팔", f"포트 확인 실패: {e}")

    # 카메라
    if check_camera:
        if studio:
            try:
                from .studio import StudioCap
                ok, _ = StudioCap(cfg.studio_camera, cfg.width, cfg.height).read()
                add(bool(ok), "카메라", "ArmStudio 가상 카메라" + (" 영상 나옴" if ok else " 영상 안 나옴"),
                    "" if ok else "ArmStudio 창을 최소화하지 말고 3D 시뮬레이터 화면을 띄우세요")
            except Exception as e:
                add(False, "카메라", f"가상 카메라 오류: {e}")
        else:
            try:
                import cv2
                cap = cv2.VideoCapture(cfg.camera) if not (os.name == "nt" and isinstance(cfg.camera, int)) \
                    else cv2.VideoCapture(cfg.camera, cv2.CAP_DSHOW)
                ok = cap.isOpened() and cap.read()[0]
                cap.release()
                add(bool(ok), "카메라", f"카메라 {cfg.camera} " + ("열림" if ok else "안 열림"),
                    "" if ok else "번호 찾기: 메뉴 2번 (또는 python -m mission.m0_capture_frames --scan)")
            except Exception as e:
                add(False, "카메라", f"카메라 확인 실패: {e}")

    # 자세
    required = cfg.required_poses()                    # 추적을 켰을 때만 watch 도 필요
    missing = [n for n in required if n not in poses_taught]
    add(not missing, "자세", f"자세 티칭 {len(required) - len(missing)}/{len(required)}",
        f"남은 자세: {', '.join(missing)}  → 메뉴 3번" if missing else "")
    add(cfg.grip_ball_max_pct > 0, "자세", "공 쥔 그리퍼 % " + (f"{cfg.grip_ball_min_pct:.0f}~{cfg.grip_ball_max_pct:.0f}" if cfg.grip_ball_max_pct > 0 else "티칭 안 함"),
        "" if cfg.grip_ball_max_pct > 0 else "공을 그리퍼에 물린 채 메뉴 3번의 ballgrip")

    # 구역
    required_rois = cfg.required_rois()
    left = [n for n in required_rois if not cfg.roi_taught(n)]
    add(not left, "구역", f"구역 티칭 {len(required_rois) - len(left)}/{len(required_rois)}" + (" (추적 켜짐)" if cfg.tracking_enabled else ""),
        f"남은 구역: {', '.join(left)}  → 메뉴 4번" if left else "")

    # 색
    try:
        from .vision import COLORS_FILE, LOCK_FILE, REF_NAMES, ColorRefs, is_locked
        refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
        miss = [n for n in REF_NAMES if n not in refs.refs]
        add(not miss, "색", f"기준색 등록 {len(REF_NAMES) - len(miss)}/{len(REF_NAMES)}",
            f"남은 색: {', '.join(miss)}  → 메뉴 5번" if miss else "")
        locked = is_locked(COLORS_FILE, LOCK_FILE)
        add(locked if not miss else False, "색", "기준색 잠금 " + ("됨" if locked else "안 됨"),
            "" if locked else "둘 다 검증한 뒤 메뉴 7번 (주문 배정 전에!)")
    except Exception as e:
        add(False, "색", f"기준색 확인 실패: {e}")

    # 작업 기록
    from .store import JobStore, StoreError
    sd = RUNTIME_DIR / ("studio" if studio else "")
    try:
        rec = JobStore(sd).load()
        add(True, "기록", f"작업 기록 있음 (상태 {rec['status']}, 시도 {rec['physical_attempts']}회)")
    except StoreError as e:
        add(False if e.kind != "missing" else None, "기록", f"작업 기록: {e.kind}",
            "최초 한 번: 메뉴 11번 (작업 기록 만들기)" if e.kind == "missing" else "기록이 깨졌습니다. 팀과 상의 (임의로 지우지 말 것)")

    # 통신
    dom = os.environ.get("ROS_DOMAIN_ID")
    add(None, "통신", f"ROS_DOMAIN_ID = {dom}" if dom else "ROS_DOMAIN_ID 안 정해짐 (팀 전용 번호를 두 Pi 에 같게)")
    try:
        import rclpy  # noqa: F401
        add(True, "통신", "rclpy(ROS2) 사용 가능")
    except ImportError:
        add(None, "통신", "rclpy 없음 → 이 환경에서는 ROS2 통신을 못 함 (--no-ros 로만, ROS2 는 Pi 의 Docker 에서)")
    return rows


def render(rows) -> str:
    out, last = [], None
    for sym, group, text, hint in rows:
        if group != last:
            out.append(f"  [{group}]")
            last = group
        out.append(f"    {sym} {text}" + (f"\n        → {hint}" if hint and sym != OK else ""))
    return "\n".join(out)


def next_step(rows) -> str:
    for sym, group, text, hint in rows:
        if sym == BAD and hint:
            return f"[{group}] {text} → {hint}"
    return "모든 점검 통과. 가상 연습(메뉴 8, 9) 후 진짜 실행(메뉴 10)으로"


def summary_line(rows) -> str:
    bad = sum(1 for r in rows if r[0] == BAD)
    return "모두 준비됨" if bad == 0 else f"{bad}개 남음"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="미션 점검")
    ap.add_argument("--studio", action="store_true")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--no-camera", action="store_true", help="카메라 열어 보기 건너뜀")
    args = ap.parse_args(argv)
    rows = collect(args.studio, args.fake, check_camera=not args.no_camera)
    print(render(rows))
    print("\n  다음에 할 일:", next_step(rows))
    return 0 if all(r[0] != BAD for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())

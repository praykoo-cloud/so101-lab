"""로봇팔 배송 미션 본체 (상태 기계). 실행:

    # 1) 연습 (실물 · 카메라 · ROS2 없이 흐름만. 가상 팔 + 가상 세계)
    python -m mission.mission_main --sim ok                  # 한 번에 성공
    python -m mission.mission_main --sim fail_then_ok        # 첫 시도 실패 → 복구 → 성공
    python -m mission.mission_main --sim fail3               # 3번 실패 → 실패 통보
    python -m mission.mission_main --sim --list              # 시나리오 목록

    # 2) 최초 설치 때 한 번 (작업 기록 만들기)
    python -m mission.mission_main --init-store

    # 3) 실제 (티칭이 끝난 뒤): 팔 포트와 카메라는 mission/config.json
    python -m mission.mission_main                           # 실물 팔 + 카메라 + ROS2 (Pi)
    python -m mission.mission_main --no-ros                  # ROS2 없이 (통신은 화면 출력만)

반드시 먼저 할 것 (티칭): m0_teach_poses.py → m0_teach_rois.py → m0_teach_colors.py (빨강 · 파랑 둘 다 등록 후 lock)

판단 규칙은 docs/mission 의 기획안(최종본)을 따른다. 요점:
- 사람의 시작 · 색 입력 · 재시작 신호는 코드에 없다. 도착은 카메라가 스스로 판단한다.
- 시도는 최대 3번. 시도는 "시도 수 증가 → 저장 성공 → 그 뒤에만 팔을 움직임" 순서로 시작한다.
- 한 시도가 끝나면 분기: 성공 검사 → ① 성공이면 완료 ② 3번째 실패면 실패 통보(바구니 확인 포함)
  ③ 남았으면 원위치에 공이 있을 때만 다음 시도, 없으면 복구 대기(시간 제한, 넘으면 안전 중단).
- 완료 = 중간 후퇴 → done 발행 → 대기 자세 → (실제 관절 확인) → clear 발행. clear 는 되돌릴 수 없다.
- 안전 중단(SafetyAbort): 팔을 그 자리에 멈추고 clear 를 내지 않는다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .config import RUNTIME_DIR, MissionConfig
from .errors import SafetyAbort
from .store import JobStore, StoreError

LOW_TO_ABOVE = {"red": "above_red", "blue": "above_blue", "basket": "above_basket"}
SAFE_POSES = ["wait", "retreat_mid", "above_basket", "above_red", "above_blue", "recovery_wait"]


class Mission:
    def __init__(self, cfg: MissionConfig, store: JobStore, arm, vision, link, motion,
                 clock=time.monotonic, sleep=time.sleep, log=None):
        self.cfg, self.store, self.arm, self.vision, self.link, self.motion = cfg, store, arm, vision, link, motion
        self.clock, self.sleep = clock, sleep
        self._log = log or (lambda event, **kw: print(f"[{time.strftime('%H:%M:%S')}] {event} {kw if kw else ''}"))
        self.rec: dict = {}
        self.max_wait: float | None = None      # 시험용: 도착·떠남 대기 상한 (None = 무한)
        self.jobs_done = 0                      # 이번 실행에서 끝낸 작업 수 (재시작 복구로 끝낸 것 포함)

    # ── 기록 도우미 ─────────────────────────────────────────────────────
    def log(self, event, **kw):
        self._log(event, **kw)

    def save(self, **changes) -> dict:
        rec = {**self.rec, **changes}
        try:
            self.rec = self.store.save(rec)
        except Exception as e:                  # 저장 실패 → 물리 동작 금지
            raise SafetyAbort(f"작업 기록 저장 실패: {e}") from e
        return self.rec

    def publish(self, **fields):
        self.link.set_state(**fields)

    def color(self) -> str:
        c = self.rec.get("color")
        if c not in ("red", "blue"):
            raise SafetyAbort("주문 색 기록이 없습니다")
        return c

    # ── 시작 (부팅) ─────────────────────────────────────────────────────
    def config_problems(self) -> list[str]:
        probs = []
        for name in ("arrival", "note", "stand_red", "stand_blue", "basket_inside", "outside", "gripper", "stand_area"):
            if not self.cfg.roi_taught(name):
                probs.append(f"구역 '{name}' 이 티칭되지 않았습니다 (m0_teach_rois.py)")
        if self.cfg.grip_ball_max_pct <= 0:
            probs.append("공을 쥔 그리퍼 % 가 티칭되지 않았습니다 (m0_teach_poses.py 의 ballgrip)")
        return probs

    def boot(self):
        """작업 기록을 읽고 상태별로 시작한다. 이동 없이 정지해야 하는 상태에서는 움직이지 않는다"""
        try:
            self.rec = self.store.load()
        except StoreError as e:
            raise SafetyAbort(f"작업 기록을 믿을 수 없습니다: {e}") from e
        probs = self.config_problems() + self.arm.preflight(self.motion_poses())
        if probs:
            raise SafetyAbort("시작 전 점검 실패: " + " / ".join(probs))
        self.arm.torque_on()                    # 지금 자리에서 잠금 (튀지 않음)
        status = self.rec["status"]
        self.log("boot", status=status, job_id=self.rec["job_id"], attempts=self.rec["physical_attempts"])
        self.publish(job_id=self.rec["job_id"], step="boot", status="")
        if status == "idle":                    # 콜드 부팅 경로
            if not self.arm.go("wait"):
                raise SafetyAbort("대기 자세로 가지 못했습니다")
            self.publish(job_id=0, step="idle", result=None, done=False, clear=False)
        elif status in ("in_progress", "motion_unknown"):
            self.recover_in_progress()
        else:                                   # loaded_confirmed / failed_confirmed
            self.recover_completed()

    def motion_poses(self) -> list[str]:
        from .m3_pick_place import NEEDED_POSES
        return NEEDED_POSES

    def _safe_pose_or_abort(self) -> str:
        """재시작 직후: 지금 자세가 티칭 자세 근처여야만 복구 이동을 한다 (아니면 안 움직임)"""
        pose = self.arm.nearest_pose(SAFE_POSES + list(LOW_TO_ABOVE))
        if pose is None:
            raise SafetyAbort("현재 자세가 티칭 자세와 달라 복구 이동을 하지 않습니다 (걸림 · 알 수 없는 상태)")
        if pose in LOW_TO_ABOVE:                # 낮은 자세에서는 수직으로 위 자세만 허용
            self.arm.go(LOW_TO_ABOVE[pose])
            pose = LOW_TO_ABOVE[pose]
        return pose

    def recover_in_progress(self):
        """재시작 시 in_progress / motion_unknown: 새 집기 없이 현장을 보고 분기 순서로 이어간다"""
        self._safe_pose_or_abort()
        self.color()                                            # 색 기록이 없으면 중단
        self.save(status="motion_unknown", phase="restart_recovery")
        self.publish(job_id=self.rec["job_id"], step="working", status="")
        self.drive_job(first_outcome="unknown")
        self.jobs_done += 1

    def recover_completed(self):
        """완료 기록: 새 물리 시도 금지. done_published / clear_committed 로 남은 절차만 이어간다"""
        r = self.rec
        pose = self._safe_pose_or_abort()
        self.publish(job_id=r["job_id"], step="done" if r["done_published"] else "working", result=r["result"])
        if not r["done_published"] and pose != "wait":
            if pose != "retreat_mid":
                self.arm.go("retreat_mid")                      # 정상 순서: 후퇴 자세에서 done 먼저
            self.publish_done()
        elif not r["done_published"]:
            self.log("warn", msg="대기 자세에서 재시작: done 을 이제야 발행")
            self.publish_done()
        else:
            self.publish(done=True, result=r["result"])
        self.finish_completion()
        self.jobs_done += 1

    # ── 한 사이클 ───────────────────────────────────────────────────────
    def run_forever(self, max_jobs: int | None = None):
        while max_jobs is None or self.jobs_done < max_jobs:
            if self.rec.get("status") == "idle":
                self.wait_for_car_and_start()
                self.drive_job(first_outcome=None)
                self.jobs_done += 1
            else:
                raise SafetyAbort(f"예상하지 못한 작업 상태: {self.rec.get('status')}")
            self.rec = self.store.load()

    def wait_for_car_and_start(self):
        """카메라로 자동차(포스트잇)가 멈춰 선 것을 확인 → 색 판정 → 새 작업 기록"""
        self.vision.reset_arrival()
        t0 = self.clock()
        while True:
            if self.arm.too_hot():
                raise SafetyAbort("모터 과열")
            if self.vision.arrival_step():
                vote = self.vision.read_color()
                if vote is not None:
                    break                                       # 색까지 읽었을 때만 시작
                self.vision.reset_arrival()
            if self.max_wait is not None and self.clock() - t0 > self.max_wait:
                raise TimeoutError("자동차가 오지 않았습니다 (시험용 시간 제한)")
            self.sleep(0.05)
        if vote.used_default:
            self.log("warn", msg="색 투표 · 거리가 동률이라 고정 기본값 사용", vote=vote.counts)
        job_id = self.rec["job_id"] + 1
        self.save(job_id=job_id, color=vote.color, physical_attempts=0, status="in_progress", result=None,
                  done_published=False, clear_committed=False, last_action="", phase="color_read")
        self.log("job_start", job_id=job_id, color=vote.color, counts=vote.counts, dist=vote.mean_dist)
        self.publish(job_id=job_id, step="working", result=None, done=False, clear=False, status="")

    # ── 작업 진행 (시도 · 분기) ─────────────────────────────────────────
    def drive_job(self, first_outcome: str | None):
        outcome = first_outcome
        kind = "pick"
        while True:
            if outcome is None:                                 # 새 시도
                outcome = self.run_attempt(kind)
            nxt = self.branch(outcome)
            if nxt == "complete_ok":
                return self.complete("ok")
            if nxt == "complete_fail":
                return self.complete("fail_attempts")
            if nxt == "recover_wait":
                nxt = self.recovery_wait()
                if nxt == "complete_ok":
                    return self.complete("ok")
            kind = "replace" if nxt == "attempt_replace" else "pick"
            outcome = None

    def start_attempt(self, kind: str):
        """모든 시도의 공통 시작 절차: 시도 수 증가 → 저장 성공 → 그 뒤에만 물리 동작"""
        n = self.rec["physical_attempts"]
        if n >= self.cfg.attempts_max:
            raise SafetyAbort("남은 시도가 없는데 새 시도를 시작하려 했습니다")
        if self.arm.too_hot():
            raise SafetyAbort("모터 과열: 새 시도를 시작하지 않습니다")
        self.save(physical_attempts=n + 1, last_action=kind, phase="attempt_started", status="in_progress")
        self.log("attempt_start", n=n + 1, kind=kind)

    def run_attempt(self, kind: str) -> str:
        """한 시도: 물리 동작 → 검증. → "success" | "fail" | "unknown" """
        color = self.color()
        self.start_attempt(kind)
        if kind == "pick":
            rep = self.motion.pick(self.arm, color, self.cfg)
            self.log("pick", closed=rep.closed_pct, lifted=rep.lifted_pct, suspect=rep.suspect)
            if rep.suspect:
                # 집기 실패 "의심": 영상 증거로 확정 (다 닫힘만으로 확정하지 않는다)
                if self.vision.ball_in_gripper(color):
                    self.log("note", msg="그리퍼 개도는 이상하지만 영상에 공이 물려 있어 계속")
                elif self.vision.ball_at_origin(color):
                    return self._end_attempt("fail", why="공이 원위치에 그대로 (집기 실패 확정)")
                else:
                    return self._end_attempt("unknown", why="집기 실패 의심 + 증거가 엇갈림")
        self.motion.place(self.arm, self.cfg)
        outcome = self.vision.check_success(color)               # 성공 판정은 이 함수 하나
        return self._end_attempt(outcome)

    def _end_attempt(self, outcome: str, why: str = "") -> str:
        self.save(phase=f"attempt_end:{outcome}")
        self.log("attempt_end", n=self.rec["physical_attempts"], outcome=outcome, why=why)
        return outcome

    def resolve_unknown(self) -> str:
        """결과 불명: t_unknown 안에서 재관측으로 결론을 시도. 그래도 모르면 "fail" (사용자 정책: 실패 시도로 센다)"""
        t0 = self.clock()
        self.arm.go("retreat_mid")
        while self.clock() - t0 < self.cfg.t_unknown:
            r = self.vision.check_success(self.color())
            if r in ("success", "fail"):
                return r
            self.sleep(0.2)
        self.log("unknown_as_fail", msg="결과 불명 → 실패 시도로 센다 (사용자 확정 정책)")
        return "fail"

    def branch(self, outcome: str) -> str:
        """분기 순서(고정): 성공 검사 → ① 성공 ② 3번째 실패면 실패 통보 ③ 남았으면 다음 시도 / 복구 대기"""
        color = self.color()
        if outcome == "unknown":
            outcome = self.resolve_unknown()
        if outcome == "success":
            return "complete_ok"
        n = self.rec["physical_attempts"]
        if n >= self.cfg.attempts_max:                           # 3번째 실패
            self.arm.go("retreat_mid")
            final = self.vision.check_success(color)             # 실패 통보 직전에도 같은 성공 판정 함수
            return "complete_ok" if final == "success" else "complete_fail"
        # 시도가 남았다
        if self.vision.ball_in_gripper(color):                   # 공을 쥔 채: 다시 넣기만 한다 (새 시도로 센다)
            return "attempt_replace"
        self.arm.go(f"above_{color}")
        if self.vision.ball_at_origin(color):
            return "attempt"
        self.arm.go("retreat_mid")
        if self.vision.check_success(color) == "success":        # 원위치엔 없고 바구니 안에서 보임
            return "complete_ok"
        return "recover_wait"

    def recovery_wait(self) -> str:
        """떨어진 공 복구 대기 (규정: 팀원이 공을 받침대 원위치에 놓는다). 팔은 정지.
        재개 조건: 원위치에 공 + 받침대 주변이 조용함. 시간 제한을 넘으면 안전 중단 (남은 시도를 거짓으로 소모하지 않는다)"""
        color = self.color()
        self.publish(step="recover_wait")
        self.arm.go("recovery_wait")
        t0 = self.clock()
        while self.clock() - t0 < self.cfg.t_restore_max:
            if self.vision.ball_at_origin(color) and self.vision.area_quiet(self.cfg.t_quiet) \
                    and self.vision.ball_at_origin(color):
                self.publish(step="working")
                return "attempt"
            self.sleep(0.5)
        raise SafetyAbort("공 복구 시간 초과 (공이 돌아오지 않았고 바구니에서도 성공이 확인되지 않음)")

    # ── 완료 ────────────────────────────────────────────────────────────
    def complete(self, result: str):
        """결과 확정 → 저장 → done 발행 → 대기 자세 → clear. (중간 후퇴 자세에서 done 을 먼저 낸다)"""
        if self.arm.nearest_pose(["retreat_mid"]) != "retreat_mid":
            self.arm.go("retreat_mid")
        status = "loaded_confirmed" if result == "ok" else "failed_confirmed"
        self.save(status=status, result=result, phase="confirmed")       # 확정 저장 (이후 새 물리 시도 없음)
        self.log("confirmed", result=result)
        self.publish_done()
        self.finish_completion()

    def publish_done(self):
        self.publish(job_id=self.rec["job_id"], step="done", result=self.rec["result"], done=True)
        self.save(done_published=True)

    def finish_completion(self):
        """done 이후: 대기 자세 → 실제 관절 확인 → clear_committed 저장 → clear 발행 → 자동차 떠남 확인 → idle"""
        r = self.rec
        if not r["clear_committed"]:
            if not self.arm.go("wait"):
                self.publish(status="hw_fault")
                raise SafetyAbort("대기 자세에 도달하지 못했습니다 (clear 를 내지 않음)")
            if not self.arm.at_pose("wait"):                      # 명령이 아니라 실제 관절 위치로 확인
                self.publish(status="hw_fault")
                raise SafetyAbort("관절 위치가 대기 자세와 다릅니다 (clear 를 내지 않음)")
            self.save(clear_committed=True, phase="clear_committed")     # 전송 전에 영속 저장 (이후 재작업 금지)
        self.publish(step="clear", clear=True)
        self.log("clear", job_id=self.rec["job_id"], result=self.rec["result"])
        self.wait_car_left()
        self.save(status="idle", phase="car_left", color=None, physical_attempts=0, result=None,
                  done_published=False, clear_committed=False, last_action="")
        self.publish(job_id=0, step="idle", result=None, done=False, clear=False)

    def wait_car_left(self):
        t0 = self.clock()
        while not self.vision.car_left_step():
            if self.max_wait is not None and self.clock() - t0 > self.max_wait:
                raise TimeoutError("자동차가 떠나지 않았습니다 (시험용 시간 제한)")
            self.sleep(0.2)

    # ── 실행 / 정리 ─────────────────────────────────────────────────────
    def run(self, max_jobs: int | None = None):
        try:
            self.boot()
            self.run_forever(max_jobs)
        except SafetyAbort as e:
            self.log("SAFETY_ABORT", reason=str(e))
            try:
                self.arm.hold()                                   # 그 자리에서 멈춤, clear 없음
            except Exception:
                pass
            self.publish(step="abort", status="abort", clear=False)
            raise


# ── 실행 진입점 ─────────────────────────────────────────────────────────
def _file_logger(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)

    def log(event, **kw):
        line = json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "event": event, **kw}, ensure_ascii=False, default=str)
        print(line)
        with open(path, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    return log


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="로봇팔 배송 미션")
    ap.add_argument("--sim", nargs="?", const="ok", help="가상 팔 · 가상 세계로 연습 (시나리오 이름, 기본 ok)")
    ap.add_argument("--list", action="store_true", help="--sim 시나리오 목록")
    ap.add_argument("--init-store", action="store_true", help="최초 설치: 작업 기록(idle) 만들기")
    ap.add_argument("--no-ros", action="store_true", help="ROS2 없이 (통신은 FakeLink)")
    ap.add_argument("--studio", action="store_true",
                    help="ArmStudio 앱의 가상 팔 + 가상(손목) 카메라로 (Pi · 실물 · ROS2 없이 PC 에서 연습)")
    ap.add_argument("--jobs", type=int, default=None, help="이만큼 처리하고 끝 (기본 계속)")
    args = ap.parse_args(argv)

    if args.list:
        from .sim import SCENARIOS
        for name, desc in SCENARIOS.items():
            print(f"  {name:16s} {desc}")
        return 0

    store = JobStore(RUNTIME_DIR / ("sim" if args.sim else "studio" if args.studio else ""))
    if args.studio and not (store.file_a.exists() or store.file_b.exists() or store.log.exists()):
        store.init_fresh()                                       # 가상 연습은 작업 기록을 자동으로 만든다
    if args.init_store:
        store.init_fresh()
        print(f"작업 기록을 만들었습니다: {store.dir}")
        return 0

    if args.sim:
        from .sim import build_sim
        import shutil
        shutil.rmtree(store.dir, ignore_errors=True)           # 연습은 항상 새로 시작
        store = JobStore(RUNTIME_DIR / "sim")
        store.init_fresh()
        mission = build_sim(args.sim, store)
        mission.max_wait = 60
        try:
            mission.run(max_jobs=1)
        except SafetyAbort as e:
            print(f"안전 중단: {e}")
            return 2
        print("가상 실행 끝. 마지막 상태:", mission.link.state())
        return 0

    cfg = MissionConfig.load()
    from .arm_io import ArmIO
    from .link import FakeLink, RosLink
    from .m3_pick_place import go_wait
    from . import m3_pick_place as motion
    from .vision import CameraSource, ColorRefs, Vision
    try:
        refs = ColorRefs.load(require_lock=not args.studio)     # 대회용은 잠긴 기준색만 (가상 연습은 잠금 없이도)
    except (FileNotFoundError, PermissionError, ValueError) as e:
        print(f"기준색 문제: {e}")
        return 1
    try:
        if args.studio:
            from .studio import StudioRobot
            arm = ArmIO(StudioRobot(speed=cfg.speed_slow), cfg)
        else:
            arm = ArmIO.open(cfg)
    except Exception as e:                                       # 포트 없음 · 다른 프로그램이 사용 중 · 앱이 꺼짐 등
        print(f"팔에 연결하지 못했습니다: {e}")
        return 1
    try:
        src = CameraSource(cfg, studio=args.studio)
    except Exception as e:
        print(f"카메라를 열지 못했습니다: {e}")
        arm.close()
        return 1
    link = FakeLink() if (args.no_ros or args.studio) else RosLink(cfg.state_hz)
    mission = Mission(cfg, store, arm, Vision(src, cfg, refs), link, motion, log=_file_logger(RUNTIME_DIR / "mission.log"))
    link.start()
    aborted = False
    try:
        mission.run(max_jobs=args.jobs)
    except SafetyAbort:
        aborted = True
        return 2
    except KeyboardInterrupt:
        print("중단 (사람이 Ctrl+C)")
    finally:
        try:
            if not aborted:                                      # 정리: 대기 자세 → 토크 끄기 → 닫기
                go_wait(arm)
                arm.torque_off()
        finally:
            link.stop()
            src.close()
            arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

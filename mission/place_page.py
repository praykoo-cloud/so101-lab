"""허브 4번 '바구니 넣기 학습' 화면: 실제 상차 순서대로 돌리면서 5~8 단계를 손으로 가르치고, 테스트로 확인한다.

실행 (Pi 의 Docker 안, 진짜 정차 · 출발 신호):
    ./허브_도커.sh                     → 브라우저로 http://<Pi 의 IP>:8081/p/place
    (Docker 밖 .venv 로 켜면 ROS2 가 없어서 '차 통신 없이' 를 체크해야만 시작된다)

학습 한 회차 (화면에도 적혀 있다)
  1 [학습 시작] → 2 대기 자세 · 차 기다림 → 3 차 + 상자가 보이면 정지 신호 → 4 색 읽고 공 집기 (프로그램)
  5 바구니 위 · 6 바구니 안 · 7 집게 해제 · 8 성공 확인 자세 (사람이 손으로 옮기고 저장)
  8 에서만 [다음 단계 진행] → 9 성공이면 대기 자세 + 출발 신호 / 10 실패면 공을 받침대에 놓고 [공 놓았어요] → 4 부터

시작 번호 (학습 · 테스트 모두, 고른 번호부터 '지금 상태' 에서 이어서 한다)
  2~4 : 처음부터 (대기 자세 · 차 기다림 · 정지 · 색 · 공 집기)
  4   : (테스트만) 포스트잇을 읽고 그 색에 맞는 공(빨강 = 빨간공_집기, 파랑 = 파란공_집기)을 집기까지만. 차 신호는 안 보낸다
  5~8 : 팔이 공을 쥐고 있는 상태에서 그 단계부터 (색을 골라야 한다). 학습을 6~8 부터 하면 앞 단계 값은 가장 최근 성공 회차에서 빌린다
  9   : 대기 자세로 가서 출발 신호(SUCCESS) 만
색: 자동(포스트잇 읽기) · 빨강 · 파랑. 2~4 에서 빨강 / 파랑을 고르면 포스트잇 읽기를 건너뛰고 그 색 공을 집는다

안전
- 5 · 6 · 8 에서는 팔(1~5번)만 힘을 뺀다. 집게(6번)는 힘을 유지해서 쥔 공이 떨어지지 않는다. 팔을 손으로 받칠 것!
- '정지' 는 지금 동작이 끝나면 멈추고 팔은 그 자리에서 힘을 준다. 차를 세워 둔 뒤 멈추거나 오류가 나면 차는 **그대로 서 있는다**
  (집게가 바구니 안에 있을 수 있으므로). 팔을 치운 뒤 [차 출발시키기] 를 누르면 FAILED + 일시정지 해제를 보낸다.
"""
from __future__ import annotations

import threading
import time

from .place_flow import ArmOps, FlowAbort, FlowStopped, PlaceRunner
from .place_learn import STEP_LABELS, STEP_NUM, STEPS, PlaceBook, PlaceError, make_snap

TEACH_STEPS = {5: "above", 6: "inside", 7: "release", 8: "check"}
LEARN_STARTS = (2, 5, 6, 7, 8, 9)
TEST_STARTS = (2, 4, 5, 6, 7, 8, 9)               # 테스트 4 = 포스트잇 읽고 맞는 공 집기만
COLOR_NAMES = {"red": "빨간", "blue": "파란"}
STEP_TITLES = {2: "대기 자세 · 차 기다림", 3: "정차 신호 · 완전 정차", 4: "색 읽기 · 공 집기", 5: "바구니 위", 6: "바구니 안",
               7: "집게 해제", 8: "성공 확인", 9: "성공 → 출발 신호", 10: "실패 → 4번부터 다시"}


class PlaceSession:
    """화면 뒤에서 일하는 책상. 한 번에 학습 또는 테스트 하나만 돈다 (스레드 하나)"""

    def __init__(self, robot, cfg, eye, motions, book: PlaceBook | None = None, ops=None, log=None,
                 clock=time.monotonic, sleep=time.sleep):
        self.robot, self.cfg, self.eye, self.motions = robot, cfg, eye, motions
        self.book = book or PlaceBook()
        if ops is None:
            from .arm_io import ArmIO
            ops = ArmOps(ArmIO(robot, cfg), cfg)
        self.ops = ops
        self.clock, self.sleep = clock, sleep
        self._log = log or (lambda event, **kw: print(f"[{time.strftime('%H:%M:%S')}] place {event} {kw if kw else ''}", flush=True))
        self.link = None
        self.lock = threading.Lock()                 # 팔 통신은 한 번에 하나씩
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._event = threading.Event()
        self._reply: dict = {}
        self.reset_status()

    def reset_status(self):
        self.status = {"mode": None, "set": None, "step": 0, "message": "시작 전이에요.", "error": False,
                       "waiting": None, "attempt": 0, "color": None, "results": [], "judged": None,
                       "draft": {}, "arm_torque": None, "link": None}
        self.draft: dict = {}
        self.images: dict = {}

    # ── 상태 ────────────────────────────────────────────────────────────
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def moving(self) -> bool:
        """팔을 프로그램이 움직이는 중인가 (사람 차례를 기다리는 중이면 False)"""
        return self.busy() and self.status["waiting"] is None

    def state(self) -> dict:
        st = dict(self.status)
        st["busy"] = self.busy()
        st["link"] = None if self.link is None else {"arm": self.link.snapshot()["state"], "car": self.link.car_state,
                                                      "fake": type(self.link).__name__ == "FakeLoadLink"}
        try:
            grip = None if self.moving() else self.ops.grip_pct()
        except Exception:
            grip = None
        st["grip"] = None if grip is None else round(grip, 1)
        st["book"] = self.book.summary()
        st["fake"] = not getattr(self.robot, "real", True)
        return st

    def log(self, event, **kw):
        self._log(event, **kw)

    def _say(self, step: int | None, message: str, error: bool = False):
        if step is not None:
            self.status["step"] = step
        self.status.update(message=message, error=error)
        self.log("say", step=self.status["step"], message=message)

    def _result(self, step: int, ok: bool, why: str):
        self.status["results"] = self.status["results"] + [{"step": step, "ok": bool(ok), "why": why}]
        self._say(step, why, not ok)

    def _need_idle(self):
        if self.busy():
            raise PlaceError("학습 · 테스트가 진행 중이에요. 끝나거나 '정지' 한 뒤에 다시 하세요.")

    # ── 사람 차례 기다리기 ──────────────────────────────────────────────
    def _wait_user(self, what: str, message: str) -> dict:
        self._reply = {}
        self._event.clear()
        self.status["waiting"] = what
        self._say(None, message)
        try:
            while not self._event.wait(0.2):
                if self._stop.is_set():
                    raise FlowStopped("정지 버튼")
        finally:
            self.status["waiting"] = None
        return self._reply

    def _answer(self, what: str, reply: dict | None = None):
        if self.status["waiting"] != what:
            raise PlaceError("지금 누를 차례가 아니에요. 화면 위쪽 안내를 보세요.")
        self._reply = reply or {}
        self.status["waiting"] = None                # 바로 '움직이는 중' 으로 (화면이 그 사이 팔을 읽지 않게)
        self._event.set()

    # ── 시작 ────────────────────────────────────────────────────────────
    def start(self, mode: str, sid: str, link, start_step: int = 2, color: str | None = None, pause: bool = False) -> str:
        self._need_idle()
        self.book.get_set(sid)
        learned = None
        color = color if color in ("red", "blue") else None          # 그 밖(auto 등) = 포스트잇을 읽는다
        if mode == "test":
            if start_step not in TEST_STARTS:
                raise PlaceError("시작 단계를 다시 고르세요.")
            if start_step != 4:
                learned = self.book.learned(sid)
        elif mode == "learn":
            if start_step not in LEARN_STARTS:
                raise PlaceError("시작 단계를 다시 고르세요.")
            if 6 <= start_step <= 8 and self.book.latest_success(sid) is None:
                raise PlaceError("6~8 부터 학습하려면 앞 단계 값을 빌려 올 성공 회차가 있어야 해요. 먼저 2~4 나 5 부터 한 번 학습하세요.")
        else:
            raise PlaceError("모드를 다시 고르세요.")
        if start_step >= 5 and color is None:
            raise PlaceError("5단계 이후부터 시작하면 팔이 쥔 공 색(빨강 / 파랑)을 골라 주세요.")
        if self.ops.too_hot():
            raise PlaceError("모터가 뜨거워요. 식힌 뒤에 다시 하세요.")
        self.reset_status()
        self.status.update(mode=mode, set=sid, color=color)
        self.link = link
        self._stop.clear()
        target = self._run_learn if mode == "learn" else self._run_test
        self._thread = threading.Thread(target=self._guard, args=(target, sid, learned, start_step, pause), daemon=True)
        self._thread.start()
        return "학습을 시작했어요. 팔이 움직이니 주변을 비우세요." if mode == "learn" else "테스트를 시작했어요."

    def _runner(self) -> PlaceRunner:
        return PlaceRunner(self.cfg, self.ops, self.eye, self.link, self.motions, clock=self.clock, sleep=self.sleep,
                           log=self._log, stop=self._stop)

    def _guard(self, target, *args):
        try:
            target(*args)
        except FlowStopped:
            self._safe_stop("정지했어요. 팔은 그 자리에서 힘을 주고 있어요.", False)
        except (FlowAbort, PlaceError, Exception) as error:          # 통신 끊김 · 과열 · 각도 못 읽음 → 그 자리에서 멈춘다
            self._safe_stop(f"문제가 생겨 멈췄어요: {error}", True)

    def _safe_stop(self, message: str, error: bool):
        try:
            self.ops.torque_arm(True)
            self.ops.hold()
        except Exception:
            pass
        sent = ""
        if self.car_held():
            # 집게가 바구니 안에 있을 수 있으니 차를 자동으로 출발시키지 않는다. 사람이 팔을 치운 뒤 [차 출발시키기]
            sent = " 차는 그대로 세워 뒀어요. 팔(집게)을 바구니에서 치운 뒤 [차 출발시키기] 를 누르세요."
        self.status["arm_torque"] = True
        self._say(None, message + sent, error)

    def car_held(self) -> bool:
        """팔이 차를 세워 두고 있는가 (STOP_REQUEST · IN_PROGRESS)"""
        return self.link is not None and self.link.snapshot()["state"] in ("STOP_REQUEST", "IN_PROGRESS")

    def car_go(self) -> str:
        """정지 · 오류 뒤 사람이 확인하고 차를 출발시킨다 (FAILED = 공 없이 출발, /arm/pause False)"""
        self._need_idle()
        if not self.car_held():
            raise PlaceError("지금 팔이 차를 세워 두고 있지 않아요.")
        self.link.set("FAILED")
        self.log("car_go", by="사람")
        return "차에게 FAILED(공 없이 출발)와 일시정지 해제를 보냈어요."

    def _go_wait(self, runner: PlaceRunner):
        runner.check()
        if not self.ops.go_wait():
            raise FlowAbort("대기 자세로 가지 못했어요")
        self.status["arm_torque"] = True

    def _finish(self, runner: PlaceRunner, success: bool, why: str = ""):
        self._go_wait(runner)
        self.link.set("SUCCESS" if success else "FAILED")
        if success:
            self._result(9, True, "성공! 대기 자세로 돌아와 차에게 출발 신호(SUCCESS)를 보냈어요.")
        else:
            self._say(10, f"{self.cfg.attempts_max}번 모두 실패해서 차에게 FAILED(공 없이 출발)를 보냈어요. {why}".strip(), True)

    def _car_and_color(self, runner: PlaceRunner, sid: str | None, color: str | None = None) -> str | None:
        """2 · 3 · 4(색). color 를 골랐으면 포스트잇 읽기는 건너뛴다. 실패하면 None"""
        self._say(2, "대기 자세로 가는 중…")
        self._go_wait(runner)
        self.link.set("WAITING", "", 0)
        if sid is not None and self.book.get_set(sid).get("watch") is None:
            self._capture_watch(sid)
        self._say(2, "대기 자세에서 차를 기다리는 중… (차와 상자가 함께 보이면 바로 정지 신호)")
        if not runner.wait_car_and_stop():
            self._result(3, False, "정지 신호를 냈지만 차가 서지 않았어요")
            self.link.set("FAILED")
            return None
        self._result(3, True, "차와 상자를 보고 정지 신호를 보냈고, 완전히 섰어요")
        if color is not None:
            self._say(4, f"고른 색: 포스트잇 읽기를 건너뛰고 {COLOR_NAMES[color]}공을 집어요")
            return color
        return self._read_color(runner)

    def _read_color(self, runner: PlaceRunner) -> str | None:
        self._say(4, "포스트잇 색을 읽는 중…")
        color = runner.read_color()
        if color is None:
            self._result(4, False, "포스트잇 색을 읽지 못했어요 → 차에게 FAILED(출발)")
            self.link.set("FAILED")
            return None
        self.status["color"] = color
        self._say(4, f"포스트잇 {'빨강' if color == 'red' else '파랑'} → {COLOR_NAMES[color]}공을 집어요")
        return color

    def _pick(self, runner: PlaceRunner, color: str, attempt: int) -> bool:
        self.status["attempt"] = attempt
        self.link.set("IN_PROGRESS", color, attempt)
        self._say(4, f"{attempt}번째 시도: {'빨간' if color == 'red' else '파란'}공 집는 중…")
        ok, why = runner.pick(color)
        self._result(4, ok, why)
        return ok

    def _after_fail(self, runner: PlaceRunner, attempt: int) -> bool:
        """실패한 뒤: 대기 자세로 가서 사람이 공을 받침대에 놓기를 기다린다. 다시 할 수 있으면 True"""
        self._go_wait(runner)
        if attempt >= self.cfg.attempts_max:
            return False
        self._wait_user("ball", f"{attempt}번째 실패. 떨어진 공을 받침대 제자리에 놓고 [공 놓았어요] 를 누르세요 → 4번(공 집기)부터 다시 해요.")
        return True

    # ── 학습 ────────────────────────────────────────────────────────────
    def _hold_car(self, color: str, attempt: int = 1):
        """5 이후부터 시작: 차가 이미 서 있다고 보고 계속 세워 둔다"""
        self.status["attempt"] = attempt
        self.link.set("IN_PROGRESS", color, attempt)

    def _run_learn(self, sid, _learned, start, _pause):
        runner = self._runner()
        color = self.status["color"]
        if start == 9:
            self._hold_car(color)
            return self._finish(runner, True)
        if start == 2:
            color = self._car_and_color(runner, sid, color)
            if color is None:
                return self._go_wait(runner)
        for attempt in range(1, self.cfg.attempts_max + 1):
            success = False
            if attempt == 1 and start >= 5:
                self._hold_car(color)                    # 지금 상태(공을 쥔 채)에서 바로 가르치기
                ok = True
            else:
                ok = self._pick(runner, color, attempt)
            if ok:
                success = self._teach(runner, sid, color, start if attempt == 1 else 5)
            if success:
                return self._finish(runner, True)
            if not self._after_fail(runner, attempt):
                break
        self._finish(runner, False)

    def _draft_status(self):
        self.status["draft"] = {k: {"angles": v["angles"], "grip": v["grip"], "basket": v["basket"]} for k, v in self.draft.items()}

    def _teach(self, runner: PlaceRunner, sid: str, color: str, first: int = 5) -> bool:
        """first 단계부터 손으로 가르친다. first 가 6~8 이면 앞 단계는 가장 최근 성공 회차에서 빌린다"""
        self.draft, self.images = {}, {}
        if first > 5:
            found = self.book.latest_success(sid)
            if found is None:
                raise PlaceError("앞 단계 값을 빌려 올 성공 회차가 없어요.")
            _tid, take = found
            for step in STEPS:
                if STEP_NUM[step] < first:
                    snap = take["steps"][step]
                    self.draft[step] = make_snap(snap["angles"], snap.get("grip"), snap.get("basket"), None)
                    self.images[step] = self.book.step_image(sid, snap)
            self._say(first, f"5~{first - 1} 단계는 '{take['label']}' 값을 빌려 왔어요. 지금 자세에서 {first}단계부터 가르쳐요.")
        self._draft_status()
        self.ops.torque_arm(False)
        self.status["arm_torque"] = False
        prompts = (
            (5, "5 바구니 위: 팔을 손으로 받치고 집게가 바구니 바로 위에 오게 옮긴 뒤 [5 저장] (집게는 힘이 들어가 있어 공을 쥐고 있어요)"),
            (6, "6 바구니 안: 집게를 바구니 안으로 내린 뒤 [6 저장]"),
            (7, "7 집게 해제: [팔 고정] 으로 팔을 멈춘 뒤 [조금 열기] 로 공을 놓고 [7 저장]"),
            (8, "8 성공 확인: [팔 힘 빼기] 후 바구니가 보이는 자세로 옮기고 [8 저장]"),
        )
        for num, text in prompts:
            if num >= first:
                self._wait_user(f"teach{num}", text)
        self.ops.torque_arm(True)
        self.status["arm_torque"] = True
        self._say(8, "공이 든 바구니가 보이는지 확인하는 중…")
        judged = runner.basket_has_ball()
        self.status["judged"] = judged
        reply = self._wait_user("next", f"YOLO 판정: {'성공' if judged else '실패'}. 맞으면 [다음 단계 진행], 틀리면 판정을 바꾼 뒤 진행하세요.")
        verdict = reply.get("verdict")
        success = judged if verdict not in ("ok", "fail") else verdict == "ok"
        tid = self.book.add_take(sid, self.draft, self.images, success, color)
        label = self.book.take(sid, tid)["label"]
        self._result(8, success, f"{label} 저장 ({'성공 → ✔ 사용' if success else '실패 → ✘ 제외 (기록만)'})")
        return success

    def _capture_watch(self, sid: str):
        seen = self.eye.look()
        snap = make_snap(self.ops.angles(), self.ops.grip_pct(), self.eye.basket() if seen else None, None)
        self.book.set_watch(sid, snap, self.eye.jpeg())
        self._say(2, "이 세트의 대기 자세(사진 + 모터 값)를 저장했어요 (세트마다 한 번).")

    # ── 테스트 ──────────────────────────────────────────────────────────
    def _run_test(self, sid, learned, start, pause):
        runner = self._runner()

        def maybe_pause(step):
            if pause:
                self._wait_user("continue", f"{step}단계 '{STEP_TITLES[step]}' 를 할 차례예요. [계속] 을 누르면 진행해요.")

        color = self.status["color"]
        if start == 4:
            return self._pick_only(runner, color)
        if start == 9:
            self._hold_car(color)
            return self._finish(runner, True)
        if start == 2:
            maybe_pause(2)
            color = self._car_and_color(runner, None, color)
            if color is None:
                return self._go_wait(runner)
        attempt = 1
        while True:
            if start == 2 or attempt > 1:
                maybe_pause(4)
                ok = self._pick(runner, color, attempt)
            else:
                self._hold_car(color, attempt)
                ok = True
            if ok:
                ok = self._auto(runner, learned, start if attempt == 1 else 5, maybe_pause)
            if ok:
                return self._finish(runner, True)
            if not self._after_fail(runner, attempt):
                return self._finish(runner, False)
            attempt += 1

    def _pick_only(self, runner: PlaceRunner, color: str | None):
        """테스트 4: 포스트잇을 읽고(또는 고른 색) 그 색에 맞는 공을 집기까지만. 차 신호는 건드리지 않는다"""
        if color is None:
            self._say(4, "포스트잇 색을 읽는 중…")
            color = runner.read_color()
            if color is None:
                return self._result(4, False, "포스트잇 색을 읽지 못했어요. 포스트잇이 카메라에 보이는지 확인하세요.")
            self.status["color"] = color
            self._say(4, f"포스트잇 {'빨강' if color == 'red' else '파랑'} → {COLOR_NAMES[color]}공을 집어요")
        else:
            self._say(4, f"고른 색: {COLOR_NAMES[color]}공을 집어요")
        ok, why = runner.pick(color)
        self.status["arm_torque"] = True
        tail = f" ({COLOR_NAMES[color]}공을 쥔 채 멈췄어요. 이어서 5부터 테스트할 수 있어요)" if ok else ""
        self._result(4, ok, why + tail)

    def _auto(self, runner: PlaceRunner, learned: dict, start: int, maybe_pause) -> bool:
        corr = {j: 0.0 for j in ("1", "2", "3", "4", "5")}
        if start <= 5:
            maybe_pause(5)
            self._say(5, "학습한 바구니 위 자세로 가서 바구니에 맞추는 중…")
            corr, why = runner.step_above(learned)
            self._result(5, corr is not None, why)
            if corr is None:
                return False
        for num, fn in ((6, runner.step_inside), (7, runner.step_release), (8, runner.step_check)):
            if num < start:
                continue
            maybe_pause(num)
            self._say(num, f"{num} {STEP_TITLES[num]} 하는 중…")
            ok, why = fn(learned, corr)
            self._result(num, ok, why)
            if not ok:
                return False
        return True

    # ── 버튼 ────────────────────────────────────────────────────────────
    def handle(self, action: str, data: dict) -> str:
        book = self.book
        if action == "set-create":
            sid = book.create_set(data.get("label", ""))
            return f"세트 '{book.get_set(sid)['label']}' 를 만들었어요."
        if action == "set-rename":
            book.rename_set(str(data.get("sid")), data.get("label"))
            return "세트 이름을 바꿨어요."
        if action == "set-choose":
            book.choose_set(str(data.get("sid")))
            return "이 세트를 상차 미션 실행용(★)으로 골랐어요."
        if action == "set-delete":
            self._need_idle()
            saved = book.delete_set(str(data.get("sid")))
            return f"세트를 지웠어요. 백업: {saved}"
        if action == "take-rename":
            book.rename_take(str(data.get("sid")), str(data.get("tid")), data.get("label"))
            return "회차 이름을 바꿨어요."
        if action == "take-use":
            book.set_use(str(data.get("sid")), str(data.get("tid")), data.get("use") is True)
            return "✔ 사용으로 바꿨어요." if data.get("use") is True else "✘ 제외로 바꿨어요."
        if action == "take-delete":
            self._need_idle()
            saved = book.delete_take(str(data.get("sid")), str(data.get("tid")))
            return f"회차를 지웠어요. 백업: {saved}"
        if action == "goto":
            return self._goto(data)
        if action == "watch-redo":
            self._need_idle()
            sid = str(data.get("sid"))
            book.get_set(sid)
            with self.lock:
                self._capture_watch(sid)
            return "지금 자세를 이 세트의 대기 자세(사진 + 모터 값)로 다시 저장했어요."
        if action == "save":
            return self._save_step(data)
        if action == "arm-torque":
            if self.moving():
                raise PlaceError("프로그램이 팔을 움직이는 중이에요.")
            on = data.get("on") is True
            with self.lock:
                self.ops.torque_arm(on)
            self.status["arm_torque"] = on
            return "팔(1~5번)에 힘을 줬어요. 손을 떼도 돼요." if on else "팔(1~5번) 힘을 뺐어요. 손으로 받치세요! (집게는 그대로)"
        if action == "grip":
            return self._grip(data)
        if action == "next":
            self._answer("next", {"verdict": data.get("verdict")})
            return "다음 단계로 진행해요."
        if action == "ball":
            self._answer("ball")
            return "4번(공 집기)부터 다시 해요."
        if action == "continue":
            self._answer("continue")
            return "계속해요."
        if action == "car-go":
            return self.car_go()
        if action == "stop":
            if not self.busy():
                raise PlaceError("지금 진행 중이 아니에요.")
            self._stop.set()
            return "정지 요청을 보냈어요. 지금 동작이 끝나면 멈춰요."
        raise PlaceError("모르는 버튼이에요.")

    def _save_step(self, data: dict) -> str:
        try:
            num = int(data.get("step"))
        except (TypeError, ValueError):
            raise PlaceError("저장할 단계를 다시 고르세요.") from None
        if num not in TEACH_STEPS:
            raise PlaceError("저장할 단계를 다시 고르세요.")
        if self.status["waiting"] != f"teach{num}":
            raise PlaceError(f"지금은 {num}단계를 저장할 차례가 아니에요. 5 → 6 → 7 → 8 순서로 저장하세요.")
        step = TEACH_STEPS[num]
        with self.lock:
            seen = self.eye.look()
            if seen is None:
                raise PlaceError("카메라 영상이 끊겨서 저장하지 않았어요. USB 연결을 확인하세요.")
            basket = self.eye.basket()
            if step == "above" and basket is None:
                raise PlaceError("YOLO 가 바구니(상자)를 못 찾아서 저장하지 않았어요. 바구니가 화면에 보이게 옮긴 뒤 다시 누르세요.")
            snap = make_snap(self.ops.angles(), self.ops.grip_pct(), basket, None)
        self.draft[step] = snap
        self.images[step] = self.eye.jpeg()
        self._draft_status()
        self._answer(f"teach{num}")
        return f"{STEP_LABELS[step]} 저장 (사진 + 모터 값)."

    def _grip(self, data: dict) -> str:
        if self.moving():
            raise PlaceError("프로그램이 팔을 움직이는 중이에요.")
        with self.lock:
            now = self.ops.grip_pct()
            if "pct" in data:
                pct = float(data["pct"])
            else:
                pct = (now or 0.0) + float(data.get("delta", 0))
            pct = max(0.0, min(100.0, pct))
            self.ops.grip(pct)
        return f"집게 {pct:.0f}%"

    def _goto(self, data: dict) -> str:
        """저장된 자세로 천천히 이동 (확인용)"""
        self._need_idle()
        sid, step = str(data.get("sid")), str(data.get("step"))
        if step == "watch":
            snap = self.book.get_set(sid).get("watch")
            if snap is None:
                raise PlaceError("이 세트에는 아직 대기 자세 기록이 없어요.")
        elif step in STEPS:
            snap = self.book.take(sid, str(data.get("tid")))["steps"][step]
        else:
            raise PlaceError("이동할 자세를 다시 고르세요.")
        with self.lock:
            if self.ops.too_hot():
                raise PlaceError("모터가 뜨거워요.")
            self.ops.torque_arm(True)
            self.ops.move_to(snap["angles"])
            if step == "release" and snap.get("grip") is not None:
                self.ops.grip(snap["grip"])
        self.status["arm_torque"] = True
        return f"{'대기 자세' if step == 'watch' else STEP_LABELS[step]} 자세로 이동했어요."

    def stop_and_join(self, timeout: float = 20.0):
        if self.busy():
            self._stop.set()
            self._thread.join(timeout)


# ── 화면 ────────────────────────────────────────────────────────────────
def place_body() -> str:
    step_rows = "".join(f"<li data-step={n}><b>{n}</b> {name}</li>" for n, name in STEP_TITLES.items())
    first = "<option value=2>2~4 처음부터 (대기 · 차 · 정지 · 색 · 공 집기)</option>"
    later = "".join(f"<option value={n}>{n} {STEP_TITLES[n]} 부터 (지금 상태에서)</option>" for n in range(5, 10))
    learn_opts = first + later
    test_opts = first + "<option value=4>4 포스트잇 읽고 맞는 공 집기만</option>" + later
    color_opts = "<option value=auto>포스트잇 읽기 (자동)</option><option value=red>빨강 공</option><option value=blue>파랑 공</option>"
    return f"""
<h1><span class="num g-prep">4</span>바구니 넣기 학습</h1>
<p class=sub>실제 상차 순서대로 돌리면서 <b>5 바구니 위 · 6 바구니 안 · 7 집게 해제 · 8 성공 확인</b> 을 손으로 가르쳐요.
회차마다 YOLO 사진과 모터 값이 저장되고, 여러 회차의 평균으로 실행해요. 2~4 · 9 · 10 은 프로그램이 알아서 해요.</p>
<p class=msg id=out role=status aria-live=polite>불러오는 중…</p>
<div class=pl-grid>
 <div>
  <img id=cam alt='YOLO 인식 카메라 화면' class=pl-cam>
  <p class=small id=camtext>YOLO 인식 화면 (차 · 상자 · 포스트잇 · 공에 네모)</p>
  <ol class=pl-steps id=steps>{step_rows}</ol>
  <p class=small id=linktext></p>
 </div>
 <div>
  <h2 style=margin-top:0>세트</h2>
  <p><select id=set aria-label='세트 고르기'></select>
   <button class=sec id=setnew>새 세트</button> <button class=sec id=setname>이름 바꾸기</button>
   <button class=sec id=setstar>★ 실행용으로</button> <button class=sec id=setdel>세트 지우기</button></p>
  <p class=small id=settext></p>
  <p><label><input type=checkbox id=noros> 차 통신 없이 (신호를 화면에만 표시. ROS2 가 없을 때 연습용)</label></p>
  <h2>학습</h2>
  <p><select id=lfrom aria-label='학습 시작 단계'>{learn_opts}</select> <select id=lcolor aria-label='학습 공 색'>{color_opts}</select></p>
  <p class=small>5 이후부터는 지금 상태(공을 쥔 채)에서 시작해요. 6~8 부터면 앞 단계 값은 가장 최근 성공 회차에서 빌려 와요.</p>
  <p><button id=learn>▶ 학습 시작</button> <button id=stop style="background:#b3261e;border-color:#b3261e">■ 정지</button>
     <button id=cargo class=sec hidden>차 출발시키기 (공 없이)</button></p>
  <div id=teach class=msg hidden>
   <p><button data-save=5>5 저장</button> <button data-save=6>6 저장</button> <button data-save=7>7 저장</button> <button data-save=8>8 저장</button></p>
   <p><button class=sec id=armoff>팔 힘 빼기</button> <button class=sec id=armon>팔 고정 (힘 주기)</button>
      <span class=small id=torquetext></span></p>
   <p>집게 <button class=sec data-grip=-5>조금 닫기 -5%</button> <button class=sec data-grip=5>조금 열기 +5%</button>
      <input type=range id=grip min=0 max=100 step=1 style="width:160px;vertical-align:middle" aria-label='집게 열림 %'> <b id=griptext>-</b></p>
  </div>
  <div id=nextbox class=msg hidden>
   <p id=judge></p>
   <p>판정: <select id=verdict><option value=auto>YOLO 판정대로</option><option value=ok>성공으로</option><option value=fail>실패로</option></select>
      <button id=next>다음 단계 진행 ▶</button></p>
  </div>
  <p><button id=ball hidden>공 놓았어요 → 4번부터</button> <button id=cont hidden>계속 ▶</button></p>
  <h2>테스트</h2>
  <p><select id=from aria-label='테스트 시작 단계'>{test_opts}</select> <select id=color aria-label='테스트 공 색'>{color_opts}</select>
     <label><input type=checkbox id=pause> 한 단계씩 멈추기</label> <button id=test>▶ 테스트</button></p>
  <p class=small>학습한 ✔ 회차들의 평균으로 자동 실행해요. '포스트잇 읽기 (자동)' 이면 포스트잇 색을 읽고 그 색 공(빨강 = 빨간공_집기, 파랑 = 파란공_집기)을 집어요.
     5 이후부터 시작하면 팔이 공을 쥐고 있어야 하고 공 색을 골라야 해요.</p>
 </div>
</div>
<h2>저장된 회차</h2>
<div id=watch></div>
<div id=takes></div>
"""


PLACE_CSS = """
.pl-grid{display:grid;grid-template-columns:minmax(320px,1fr) minmax(320px,1fr);gap:18px;align-items:start}
@media (max-width:760px){.pl-grid{grid-template-columns:1fr}}
.pl-cam{width:100%;min-height:200px;border:1px solid var(--line);border-radius:8px;background:#111}
.pl-steps{list-style:none;padding:0;margin:8px 0;display:grid;grid-template-columns:1fr 1fr;gap:4px}
.pl-steps li{padding:4px 8px;border:1px solid var(--line);border-radius:6px;background:#fff;font-size:14px}
.pl-steps li.now{border-color:var(--brand);background:#e3f2f7;font-weight:700}.pl-steps li.ok{color:var(--ok)}.pl-steps li.bad{color:var(--bad)}
.pl-take{background:#fff;border:1px solid var(--line);border-radius:10px;padding:10px 12px;margin:10px 0}
.pl-take.off{opacity:.6}.pl-shots{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.pl-shots figure{margin:0}.pl-shots img{width:100%;border-radius:6px;border:1px solid var(--line)}
.pl-shots figcaption{font-size:12px;color:var(--mute)}
"""


def place_script(token: str) -> str:
    return """<script>
const T='__TOKEN__';const $=id=>document.getElementById(id);let S=null,camBusy=false;
const NAMES={above:'5 바구니 위',inside:'6 바구니 안',release:'7 집게 해제',check:'8 성공 확인'};
function say(t,ok){const o=$('out');o.textContent=t;o.className='msg '+(ok===false?'bad':ok?'ok':'')}
async function post(a,d){const r=await fetch('/api/place/'+a,{method:'POST',headers:{'Content-Type':'application/json','X-Hub-Token':T},body:JSON.stringify(d||{})});
 const j=await r.json().catch(()=>({message:'응답을 읽지 못했어요.'}));say(j.message,r.ok);refresh();return r.ok}
const sid=()=>$('set').value;
function ang(a){return a?Object.keys(a).map(k=>k+':'+a[k]).join(' '):'-'}
function shot(s,set,file,cap){return '<figure>'+(file?'<a target=_blank href="/img/place/'+set+'/'+file+'"><img loading=lazy src="/img/place/'+set+'/'+file+'"></a>':'<div class=small>(사진 없음)</div>')+'<figcaption>'+cap+'</figcaption></figure>'}
function esc(t){return String(t==null?'':t).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function render(d){S=d;const b=d.book;const sel=$('set');const keep=sel.value;
 sel.innerHTML=b.sets.map(s=>'<option value="'+s.id+'">'+(s.chosen?'★ ':'')+esc(s.label)+' ('+s.used+'/'+s.takes.length+' ✔)</option>').join('')||'<option value="">세트 없음: 새 세트를 만드세요</option>';
 if(keep&&b.sets.some(s=>s.id===keep))sel.value=keep;else if(d.set)sel.value=d.set;else if(b.chosen)sel.value=b.chosen;
 const set=b.sets.find(s=>s.id===sel.value);
 $('settext').textContent=set?('만든 때 '+set.created+' · ★ = 상차 미션 실행에 쓰는 세트'):'';
 const run=d.busy,w=d.waiting||'';
 ['learn','test','setdel','lfrom','lcolor','from','color'].forEach(i=>$(i).disabled=run||!set);$('stop').disabled=!run;
 $('teach').hidden=!w.startsWith('teach');
 document.querySelectorAll('[data-save]').forEach(x=>x.disabled=w!=='teach'+x.dataset.save);
 $('nextbox').hidden=w!=='next';$('judge').textContent=d.judged===null?'':'YOLO 판정: '+(d.judged?'성공 (공이 든 바구니 보임)':'실패 (안 보임)');
 $('cargo').hidden=run||!(d.link&&(d.link.arm==='STOP_REQUEST'||d.link.arm==='IN_PROGRESS'));
 $('ball').hidden=w!=='ball';$('cont').hidden=w!=='continue';
 $('torquetext').textContent=d.arm_torque===false?'지금: 팔 힘 빠짐 (손으로 받치세요)':d.arm_torque?'지금: 팔 고정됨':'';
 $('griptext').textContent=d.grip==null?'-':d.grip+'%';if(d.grip!=null&&document.activeElement!==$('grip'))$('grip').value=d.grip;
 if(!d.mode&&$('out').textContent==='불러오는 중…')say(set?'준비됐어요. 세트를 고르고 [▶ 학습 시작] 또는 [▶ 테스트] 를 누르세요.':'먼저 [새 세트] 로 세트를 만드세요.');
 if(d.mode)say((d.busy?'['+(d.mode==='learn'?'학습':'테스트')+' '+d.step+'단계] ':'')+d.message,d.error?false:null);
 const res={};(d.results||[]).forEach(r=>res[r.step]=r.ok);
 document.querySelectorAll('#steps li').forEach(li=>{const n=+li.dataset.step;li.className=(d.busy&&n===d.step?'now ':'')+(n in res?(res[n]?'ok':'bad'):'')});
 const L=d.link;$('linktext').textContent=L?('차 통신'+(L.fake?'(화면에만)':'')+': 팔 → '+L.arm+' · 차 상태 '+(L.car||'아직 못 받음')+(d.attempt?' · 시도 '+d.attempt+'번째':'')+(d.color?' · 색 '+(d.color==='red'?'빨강':'파랑'):'')):'';
 if(!set){$('watch').innerHTML='';$('takes').innerHTML='<p class=small>세트를 만들고 학습을 시작하세요.</p>';return}
 const wt=set.watch;$('watch').innerHTML='<div class=pl-take><b>대기 자세 (세트마다 한 번)</b> '+(wt?'<span class=small>'+wt.t+' · 모터 '+ang(wt.angles)+' · 집게 '+wt.grip+'%</span>':'<span class=warn>아직 없음 (학습을 시작하면 자동 저장)</span>')+
  '<div class=pl-shots style=max-width:200px>'+(wt?shot(0,set.id,wt.image,'대기 자세 YOLO 사진'):'')+'</div><p><button class=sec data-goto=watch '+(wt?'':'disabled')+'>이 자세로 이동</button> <button class=sec id=watchredo>지금 자세로 다시 저장</button></p></div>';
 $('takes').innerHTML=set.takes.slice().reverse().map(t=>'<div class="pl-take'+(t.use?'':' off')+'"><b>'+esc(t.label)+'</b> <span class=small>'+t.created+' · '+(t.success?'<span class=ok>성공</span>':'<span class=bad>실패</span>')+' · 색 '+(t.color==='red'?'빨강':t.color==='blue'?'파랑':'-')+' · '+(t.use?'✔ 사용':'✘ 제외')+'</span>'+
  '<div class=pl-shots>'+Object.keys(NAMES).map(k=>{const s=t.steps[k];return shot(0,set.id,s.image,NAMES[k]+'<br>모터 '+ang(s.angles)+' · 집게 '+s.grip+'%'+(s.basket?'<br>바구니 x'+s.basket.cx+' y'+s.basket.cy:'')+'<br><button class=sec data-goto='+k+' data-tid='+t.id+'>이 자세로</button>')}).join('')+'</div>'+
  '<p><button class=sec data-use='+(t.use?'0':'1')+' data-tid='+t.id+'>'+(t.use?'✘ 제외하기':'✔ 사용하기')+'</button> <button class=sec data-rename='+t.id+'>이름 바꾸기</button> <button class=sec data-del='+t.id+'>지우기</button></p></div>').join('')||'<p class=small>아직 회차가 없어요.</p>';
 document.querySelectorAll('[data-goto]').forEach(x=>{x.disabled=x.disabled||run;x.onclick=()=>{if(confirm('팔이 저장된 자세로 움직여요. 주변을 비웠나요?'))post('goto',{sid:sid(),tid:x.dataset.tid,step:x.dataset.goto})}});
 document.querySelectorAll('[data-use]').forEach(x=>x.onclick=()=>post('take-use',{sid:sid(),tid:x.dataset.tid,use:x.dataset.use==='1'}));
 document.querySelectorAll('[data-rename]').forEach(x=>x.onclick=()=>{const n=prompt('새 회차 이름 (1~30자)');if(n)post('take-rename',{sid:sid(),tid:x.dataset.rename,label:n})});
 document.querySelectorAll('[data-del]').forEach(x=>x.onclick=()=>{if(confirm('이 회차를 지울까요? (백업이 남아요)'))post('take-delete',{sid:sid(),tid:x.dataset.del})});
 const wr=$('watchredo');if(wr){wr.disabled=run;wr.onclick=()=>{if(confirm('지금 팔 자세와 카메라 사진을 이 세트의 대기 자세로 다시 저장할까요?'))post('watch-redo',{sid:sid()})}}}
async function refresh(){try{render(await (await fetch('/api/place/state',{cache:'no-store'})).json())}catch(e){say('허브와 연결이 끊겼어요. 허브가 켜져 있는지 확인하세요.',false)}}
function cam(){if(camBusy)return;camBusy=true;const i=new Image();i.onload=()=>{$('cam').src=i.src;camBusy=false;$('camtext').textContent='YOLO 인식 화면'};
 i.onerror=()=>{camBusy=false;$('camtext').textContent='카메라 화면을 받지 못했어요 (USB · 카메라 번호 확인)'};i.src='/img/live.jpg?t='+Date.now()}
$('set').onchange=refresh;
$('setnew').onclick=()=>{const n=prompt('새 세트 이름 (비우면 자동)','');if(n!==null)post('set-create',{label:n})};
$('setname').onclick=()=>{const n=prompt('세트 새 이름');if(n)post('set-rename',{sid:sid(),label:n})};
$('setstar').onclick=()=>post('set-choose',{sid:sid()});
$('setdel').onclick=()=>{if(confirm('세트와 그 안의 회차를 모두 지울까요? (백업이 남아요)'))post('set-delete',{sid:sid()})};
function holding(f,c){if(f<5||f>8)return true;if(c==='auto'){say('5 이후부터는 팔이 쥔 공 색(빨강 / 파랑)을 골라 주세요.',false);return false}
 return confirm('지금 팔이 '+(c==='red'?'빨간':'파란')+' 공을 쥐고 있나요?')}
$('learn').onclick=()=>{const f=+$('lfrom').value,c=$('lcolor').value;if(f===9&&c==='auto'){say('9 는 공 색(빨강 / 파랑)을 골라 주세요.',false);return}if(!holding(f,c))return;
 if(confirm(f===2?'팔이 대기 자세로 움직이고, 차가 오면 정지 신호를 보낸 뒤 공을 집어요. 주변을 비웠나요?':f+'단계부터 학습해요. 주변을 비웠나요?'))post('learn',{sid:sid(),start:f,color:c,no_ros:$('noros').checked})};
$('test').onclick=()=>{const f=+$('from').value,c=$('color').value;if(f===9&&c==='auto'){say('9 는 공 색(빨강 / 파랑)을 골라 주세요.',false);return}if(!holding(f,c))return;
 if(confirm(f+'단계부터 자동으로 움직여요. 주변을 비웠나요?'))post('test',{sid:sid(),start:f,color:c,pause:$('pause').checked,no_ros:$('noros').checked})};
$('stop').onclick=()=>post('stop');
$('cargo').onclick=()=>{if(confirm('팔(집게)이 바구니에서 완전히 빠져 있나요? 차가 공 없이 출발해요.'))post('car-go')};
document.querySelectorAll('[data-save]').forEach(x=>x.onclick=()=>post('save',{step:+x.dataset.save}));
$('armoff').onclick=()=>{if(confirm('팔(1~5번) 힘을 뺄까요? 손으로 받치세요. 집게는 그대로 공을 쥐고 있어요.'))post('arm-torque',{on:false})};
$('armon').onclick=()=>post('arm-torque',{on:true});
document.querySelectorAll('[data-grip]').forEach(x=>x.onclick=()=>post('grip',{delta:+x.dataset.grip}));
$('grip').onchange=()=>post('grip',{pct:+$('grip').value});
$('next').onclick=()=>post('next',{verdict:$('verdict').value});
$('ball').onclick=()=>post('ball');$('cont').onclick=()=>post('continue');
refresh();setInterval(refresh,1000);cam();setInterval(cam,500);
</script>""".replace("__TOKEN__", token)

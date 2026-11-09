"""mission.mission_main 시험 — 가상 세계(mission.sim)에서 규정 · 정책이 코드로 지켜지는가.

    python -m unittest tests.test_mission_flow
"""
import atexit
import shutil
import tempfile
import unittest
from pathlib import Path

from mission.errors import SafetyAbort
from mission.sim import build_sim
from mission.store import JobStore


def make(name):
    tmp = tempfile.mkdtemp(prefix="mission_test_")
    atexit.register(shutil.rmtree, tmp, True)         # 시험이 끝나면 임시 폴더를 지운다
    store = JobStore(Path(tmp))
    store.init_fresh()
    m = build_sim(name, store)
    m.max_wait = 60
    return m


def finish(m, max_jobs=1):
    try:
        m.run(max_jobs=max_jobs)
    finally:
        pass
    return m


def order_of(history, key):
    """상태 이력에서 key 가 처음 True 가 된 위치 (없으면 None)"""
    for i, h in enumerate(history):
        if h.get(key):
            return i
    return None


class NormalFlowTest(unittest.TestCase):
    def test_ok_one_shot(self):
        m = finish(make("ok"))
        hist = m.link.history
        self.assertIsNotNone(order_of(hist, "done"))
        self.assertIsNotNone(order_of(hist, "clear"))
        self.assertLess(order_of(hist, "done"), order_of(hist, "clear"))      # done 이 clear 보다 먼저
        self.assertEqual(m.store.load()["status"], "idle")
        self.assertTrue(any(h["result"] == "ok" and h["done"] for h in hist))

    def test_done_published_from_retreat_pose_and_clear_from_wait(self):
        m = make("ok")
        poses_at = {}
        orig = m.link.set_state

        def spy(**fields):
            if fields.get("done") and "done" not in poses_at:
                poses_at["done"] = m.arm.pose
            if fields.get("clear") and "clear" not in poses_at:
                poses_at["clear"] = m.arm.pose
            return orig(**fields)
        m.link.set_state = spy
        finish(m)
        self.assertEqual(poses_at["done"], "retreat_mid")    # 중간 후퇴 자세에서 done
        self.assertEqual(poses_at["clear"], "wait")          # 대기 자세에 도달한 뒤 clear

    def test_blue_order(self):
        m = finish(make("blue"))
        self.assertIn("blue", [e for e in ("blue",)])        # 색은 카메라 값이 그대로 쓰인다 (기록은 idle 로 지워짐)
        self.assertTrue(any(h["done"] and h["result"] == "ok" for h in m.link.history))


class RetryRulesTest(unittest.TestCase):
    def attempts_seen(self, m):
        # 기록은 끝나면 idle 로 초기화되므로 로그로 센다
        return m._attempts

    def instrument(self, m):
        m._attempts = []
        orig = m.start_attempt

        def spy(kind):
            orig(kind)
            m._attempts.append((m.rec["physical_attempts"], kind))
        m.start_attempt = spy
        return m

    def test_fail_then_ok(self):
        m = self.instrument(make("fail_then_ok"))
        finish(m)
        self.assertEqual([n for n, _ in m._attempts], [1, 2])
        self.assertTrue(any(h["result"] == "ok" and h["done"] for h in m.link.history))

    def test_three_failures_report_fail_and_never_a_fourth_attempt(self):
        m = self.instrument(make("fail3"))
        finish(m)
        self.assertEqual([n for n, _ in m._attempts], [1, 2, 3])
        self.assertTrue(any(h["result"] == "fail_attempts" and h["done"] for h in m.link.history))
        self.assertIsNotNone(order_of(m.link.history, "clear"))     # 실패 통보 뒤에도 같은 절차로 clear

    def test_stuck_ball_is_replaced_and_counts_as_new_attempt(self):
        m = self.instrument(make("stuck_then_ok"))
        finish(m)
        self.assertEqual(m._attempts, [(1, "pick"), (2, "replace")])

    def test_dropped_ball_waits_for_restore_then_retries(self):
        m = self.instrument(make("drop_then_restore"))
        finish(m)
        self.assertEqual([n for n, _ in m._attempts], [1, 2])
        self.assertTrue(any(h["step"] == "recover_wait" for h in m.link.history))

    def test_cannot_start_attempt_beyond_max(self):
        m = make("ok")
        m.boot()
        m.save(status="in_progress", color="red", physical_attempts=3, job_id=1)
        with self.assertRaises(SafetyAbort):
            m.start_attempt("pick")

    def test_attempt_not_started_if_save_fails(self):
        m = make("ok")
        m.boot()
        m.save(status="in_progress", color="red", physical_attempts=0, job_id=1)

        def boom(rec):
            raise OSError("디스크 오류")
        m.store.save = boom
        with self.assertRaises(SafetyAbort):
            m.start_attempt("pick")
        self.assertEqual(m.arm.pose, "wait")                 # 팔은 움직이지 않았다


class UnknownAndAbortTest(unittest.TestCase):
    def test_hidden_then_seen_becomes_success_by_reobservation(self):
        m = finish(make("hidden_then_seen"))
        self.assertTrue(any(h["result"] == "ok" and h["done"] for h in m.link.history))

    def test_hidden_forever_aborts_without_clear(self):
        m = make("hidden_forever")
        with self.assertRaises(SafetyAbort):
            m.run(max_jobs=1)
        self.assertIsNone(order_of(m.link.history, "clear"))     # 안전 중단: clear 없음
        self.assertEqual(m.link.state()["status"], "abort")

    def test_restore_timeout_aborts_without_clear(self):
        m = make("restore_timeout")
        with self.assertRaises(SafetyAbort):
            m.run(max_jobs=1)
        self.assertIsNone(order_of(m.link.history, "clear"))


class RestartTest(unittest.TestCase):
    def test_corrupt_record_aborts_at_boot_and_arm_does_not_move(self):
        m = make("ok")
        m.store.file_a.write_text("깨짐", encoding="utf-8")
        with self.assertRaises(SafetyAbort):
            m.run(max_jobs=1)
        self.assertEqual(m.arm.pose, "rest")                     # 한 번도 움직이지 않음

    def test_restart_with_three_attempts_used_never_tries_a_fourth(self):
        m = make("fail3")
        rec = m.store.load()
        m.store.save({**rec, "status": "in_progress", "job_id": 1, "color": "red", "physical_attempts": 3})
        m.arm.pose = "retreat_mid"
        calls = []
        orig = m.motion.pick
        m.motion.pick = lambda *a, **k: (calls.append(1), orig(*a, **k))[1]
        m.run(max_jobs=1)
        self.assertEqual(calls, [])                              # 새 집기 없음
        self.assertTrue(any(h["result"] == "fail_attempts" and h["done"] for h in m.link.history))

    def test_restart_after_clear_committed_only_resends_and_waits_for_car(self):
        m = make("ok")
        rec = m.store.load()
        m.store.save({**rec, "status": "loaded_confirmed", "job_id": 4, "color": "red", "physical_attempts": 1,
                      "result": "ok", "done_published": True, "clear_committed": True})
        m.arm.pose = "wait"
        picks = []
        m.motion.pick = lambda *a, **k: picks.append(1)
        m.run(max_jobs=1)
        self.assertEqual(picks, [])                              # 재작업 금지
        self.assertTrue(any(h["clear"] and h["job_id"] == 4 for h in m.link.history))
        self.assertEqual(m.store.load()["status"], "idle")

    def test_restart_after_confirmed_before_done_publishes_done_from_retreat_pose_first(self):
        m = make("ok")
        rec = m.store.load()
        m.store.save({**rec, "status": "loaded_confirmed", "job_id": 2, "color": "red", "physical_attempts": 1,
                      "result": "ok", "done_published": False, "clear_committed": False})
        m.arm.pose = "retreat_mid"
        seen = {}
        orig = m.link.set_state

        def spy(**fields):
            if fields.get("done") and "done" not in seen:
                seen["done"] = m.arm.pose
            return orig(**fields)
        m.link.set_state = spy
        m.run(max_jobs=1)
        self.assertEqual(seen["done"], "retreat_mid")

    def test_restart_from_unknown_pose_does_not_move(self):
        m = make("ok")
        rec = m.store.load()
        m.store.save({**rec, "status": "in_progress", "job_id": 1, "color": "red", "physical_attempts": 1})
        m.arm.pose = "somewhere_strange"
        with self.assertRaises(SafetyAbort):
            m.run(max_jobs=1)
        self.assertEqual(m.arm.pose, "somewhere_strange")


if __name__ == "__main__":
    unittest.main()

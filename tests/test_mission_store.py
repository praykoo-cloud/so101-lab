"""mission.store 시험 — 전원이 꺼져도 시도 횟수 · 완료 상태를 잃지 않는가.

    python -m unittest tests.test_mission_store      (pytest 가 있으면 python -m pytest tests 도 된다)
"""
import json
import tempfile
import unittest
from pathlib import Path

from mission.store import JobStore, StoreError


class StoreTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.store = JobStore(self.dir)

    def tearDown(self):
        self._tmp.cleanup()

    def test_missing_is_not_idle(self):
        with self.assertRaises(StoreError) as cm:
            self.store.load()
        self.assertEqual(cm.exception.kind, "missing")

    def test_init_then_roundtrip(self):
        self.store.init_fresh()
        rec = JobStore(self.dir).load()
        self.assertEqual(rec["status"], "idle")
        self.assertEqual(rec["physical_attempts"], 0)
        rec = self.store.save({**rec, "job_id": 1, "color": "red", "status": "in_progress", "physical_attempts": 2})
        again = JobStore(self.dir).load()
        self.assertEqual(again["physical_attempts"], 2)
        self.assertEqual(again["color"], "red")
        self.assertEqual(again["seq"], rec["seq"])

    def test_init_twice_refused(self):
        self.store.init_fresh()
        with self.assertRaises(StoreError):
            JobStore(self.dir).init_fresh()

    def test_ab_files_alternate(self):
        self.store.init_fresh()
        rec = self.store.load()
        for i in range(3):
            rec = self.store.save(rec)
        self.assertTrue(self.store.file_a.exists() and self.store.file_b.exists())

    def test_corrupt_latest_file_is_a_safety_abort_not_a_rollback(self):
        """최신 파일이 깨졌을 때 이전 정상 파일로 조용히 되돌아가면 시도 횟수를 잃는다 → 오류로 멈춘다"""
        self.store.init_fresh()
        rec = self.store.load()
        rec = self.store.save({**rec, "status": "in_progress", "physical_attempts": 1})
        rec = self.store.save({**rec, "physical_attempts": 2})
        latest = self.store.file_a if rec["seq"] % 2 == 1 else self.store.file_b
        latest.write_text("{ 깨진 파일", encoding="utf-8")
        with self.assertRaises(StoreError) as cm:
            JobStore(self.dir).load()
        self.assertEqual(cm.exception.kind, "seq_mismatch")

    def test_log_ahead_of_file_detected(self):
        self.store.init_fresh()
        with open(self.store.log, "a") as f:
            f.write("99\n")
        with self.assertRaises(StoreError) as cm:
            JobStore(self.dir).load()
        self.assertEqual(cm.exception.kind, "seq_mismatch")

    def test_file_ahead_of_log_detected(self):
        """파일 교체 후 로그 기록 전에 꺼진 경우"""
        self.store.init_fresh()
        rec = self.store.load()
        self.store.save(rec)
        lines = self.store.log.read_text().splitlines()
        self.store.log.write_text("\n".join(lines[:-1]) + "\n")
        with self.assertRaises(StoreError) as cm:
            JobStore(self.dir).load()
        self.assertEqual(cm.exception.kind, "seq_mismatch")

    def test_torn_log_line_ignored(self):
        self.store.init_fresh()
        with open(self.store.log, "a") as f:
            f.write("9")                      # 줄바꿈 없이 쓰다 만 줄
        self.assertEqual(JobStore(self.dir).load()["status"], "idle")

    def test_tampered_checksum_rejected(self):
        self.store.init_fresh()
        data = json.loads(self.store.file_a.read_text(encoding="utf-8"))
        data["physical_attempts"] = 0
        data["job_id"] = 5                    # 체크섬은 그대로 → 무효
        self.store.file_a.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaises(StoreError) as cm:
            JobStore(self.dir).load()
        self.assertEqual(cm.exception.kind, "lost_state")

    def test_save_requires_load(self):
        with self.assertRaises(StoreError):
            JobStore(self.dir).save({"status": "idle"})

    def test_bad_status_rejected(self):
        self.store.init_fresh()
        with self.assertRaises(ValueError):
            self.store.save({**self.store.load(), "status": "???"})


if __name__ == "__main__":
    unittest.main()

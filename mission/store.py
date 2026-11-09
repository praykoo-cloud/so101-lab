"""작업 기록 저장소 (전원이 꺼져도 시도 횟수 · 주문 색 · 완료 상태가 남게 한다).

규칙 (기획안 D9):
- 기록은 한 레코드 한 덩어리: job_id, color, physical_attempts, status, result, done_published, clear_committed ...
- 저장할 때마다 순번(seq)이 1 씩 올라간다.
- 저장 순서: 임시 파일에 쓰기 → 디스크에 반영(fsync) → 원자적 교체(os.replace) → 같은 seq 를 로그에 한 줄 추가(fsync).
  이 모두가 끝난 뒤에만 save() 가 돌아온다. 그 전에는 팔을 움직이거나 clear 를 내면 안 된다.
- 파일은 A/B 두 개를 번갈아 쓴다. 한쪽이 깨져도 다른 쪽이 남는다.
- 부팅 때: 유효한 파일 중 가장 큰 seq 를 고르고, 로그의 마지막 seq 와 비교한다.
  둘이 다르면(어느 쪽이 크든) 최신 상태를 잃었을 수 있으므로 **재시도하지 않고 안전 중단** (StoreError).
- 파일이 하나도 없으면 최초 설치 때 만든 것만 정상 (init_fresh). 없는데 idle 로 취급하지 않는다.
"""
from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path

STATUSES = ("idle", "in_progress", "motion_unknown", "loaded_confirmed", "failed_confirmed")


class StoreError(RuntimeError):
    """기록을 믿을 수 없다. kind: missing | lost_state | seq_mismatch | exists | not_loaded"""

    def __init__(self, kind: str, message: str):
        super().__init__(f"{kind}: {message}")
        self.kind = kind


def new_record() -> dict:
    return {
        "seq": 0,
        "job_id": 0,                 # 작업(자동차 1대) 번호. 재부팅해도 이어진다
        "color": None,               # "red" | "blue" (카메라로 확인해 저장한 값만)
        "physical_attempts": 0,      # 지금까지 소모한 시도 수 (최대 3)
        "last_action": "",           # 마지막 물리 동작
        "phase": "",
        "status": "idle",
        "result": None,              # "ok" | "fail_attempts" (확정된 뒤에만)
        "done_published": False,
        "clear_committed": False,
        "updated": "",
    }


def _checksum(rec: dict) -> str:
    body = {k: v for k, v in rec.items() if k != "checksum"}
    return hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False).encode("utf-8")).hexdigest()


def _write_atomic(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    if os.name != "nt":                      # 폴더 항목도 디스크에 반영 (리눅스/Pi)
        fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


class JobStore:
    def __init__(self, directory: Path | str):
        self.dir = Path(directory)
        self.file_a = self.dir / "job_a.json"
        self.file_b = self.dir / "job_b.json"
        self.log = self.dir / "job.log"
        self._seq: int | None = None         # load() / init_fresh() 뒤에 값이 생긴다

    # ── 읽기 ────────────────────────────────────────────────────────────
    def _read_valid(self, path: Path):
        try:
            rec = json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, ValueError, OSError):
            return None
        if not isinstance(rec, dict) or rec.get("checksum") != _checksum(rec):
            return None
        if rec.get("status") not in STATUSES or not isinstance(rec.get("seq"), int):
            return None
        return rec

    def _log_last(self):
        """로그의 마지막 seq. 줄바꿈으로 끝난 줄만 인정한다 (쓰다 만 줄은 무시)"""
        try:
            raw = self.log.read_bytes().decode("ascii", errors="ignore")
        except FileNotFoundError:
            return None
        last = None
        for line in raw.split("\n")[:-1]:         # 마지막 조각은 줄바꿈 없이 끝난(쓰다 만) 줄이라 버린다
            if line.strip().isdigit():
                last = int(line.strip())
        return last

    def load(self) -> dict:
        valid = [r for r in (self._read_valid(self.file_a), self._read_valid(self.file_b)) if r]
        log_last = self._log_last()
        if not valid:
            if log_last is None and not self.file_a.exists() and not self.file_b.exists():
                raise StoreError("missing", "작업 기록이 없습니다 (최초 설치라면 init_fresh 를 한 번 실행)")
            raise StoreError("lost_state", "작업 기록 파일이 모두 깨졌습니다 → 안전 중단")
        rec = max(valid, key=lambda r: r["seq"])
        if log_last != rec["seq"]:
            raise StoreError("seq_mismatch",
                             f"파일 seq={rec['seq']} 로그 seq={log_last}: 최신 상태를 잃었을 수 있습니다 → 안전 중단")
        self._seq = rec["seq"]
        return {k: v for k, v in rec.items() if k != "checksum"}

    # ── 쓰기 ────────────────────────────────────────────────────────────
    def init_fresh(self) -> dict:
        """최초 설치 때 한 번만: idle 기록을 만든다 (이미 있으면 거부)"""
        if self.file_a.exists() or self.file_b.exists() or self.log.exists():
            raise StoreError("exists", "이미 작업 기록이 있습니다 (지우려면 설치 절차에서 직접 정리)")
        self.dir.mkdir(parents=True, exist_ok=True)
        self._seq = 0
        return self.save(new_record())

    def save(self, rec: dict) -> dict:
        """seq 를 올려 저장. 파일 교체와 로그 기록이 모두 끝난 뒤에 돌아온다. 실패하면 예외 (→ 물리 동작 금지)"""
        if self._seq is None:
            raise StoreError("not_loaded", "load() 또는 init_fresh() 를 먼저 하세요")
        if rec.get("status") not in STATUSES:
            raise ValueError(f"status 가 이상합니다: {rec.get('status')}")
        out = dict(rec)
        out["seq"] = self._seq + 1
        out["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        out["checksum"] = _checksum(out)
        path = self.file_a if out["seq"] % 2 == 1 else self.file_b      # A/B 번갈아
        self.dir.mkdir(parents=True, exist_ok=True)
        _write_atomic(path, json.dumps(out, indent=2, ensure_ascii=False))
        with open(self.log, "a", encoding="ascii") as f:
            f.write(f"{out['seq']}\n")
            f.flush()
            os.fsync(f.fileno())
        self._seq = out["seq"]
        return {k: v for k, v in out.items() if k != "checksum"}

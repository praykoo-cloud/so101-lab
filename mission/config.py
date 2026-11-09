"""미션 설정 (숫자는 전부 여기). 티칭 도구(m0_*)가 이 파일의 값을 채운다.

설정 파일: mission/config.json   (없으면 아래 기본값 사용, 티칭하면 만들어진다)
실행 중 생기는 파일(작업 기록, 기준색, 로그): mission/runtime/   (git 에 올리지 않는다)

단어장
- ROI   : 카메라 화면에서 "여기만 보겠다" 고 정한 네모 [x, y, w, h] (픽셀)
- 티칭  : 사람이 직접 보여 줘서 로봇에게 값(자세, 색, 구역)을 외우게 하는 것
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

MISSION_DIR = Path(__file__).resolve().parent
ROOT = MISSION_DIR.parent
RUNTIME_DIR = MISSION_DIR / "runtime"
CONFIG_FILE = MISSION_DIR / "config.json"

# 티칭해야 하는 자세 이름 (data/poses.json 에 저장됨). 설명은 티칭 도구 화면에도 나온다
POSE_HELP = {
    "wait": "대기 자세: 그리퍼 카메라가 위에서 도로(자동차 정지 위치)를 내려다본다",
    "above_red": "빨간 공 바로 위 (안전 높이)",
    "red": "빨간 공을 집는 높이",
    "above_blue": "파란 공 바로 위 (안전 높이)",
    "blue": "파란 공을 집는 높이",
    "above_basket": "바구니 바로 위 (안전 높이)",
    "basket": "바구니 안쪽에 공을 놓는 높이",
    "retreat_mid": "바구니에서 빠져나온 중간 후퇴 자세 (바구니와 닿지 않는 곳)",
    "recovery_wait": "공 복구 대기 자세 (사람이 공을 놓는 길 밖, 받침대가 보이는 곳)",
    "rest": "휴식(접힌) 자세: 전원을 켜기 전에 팔을 두는 자세 (토크 끌 때만 사용)",
}

# 티칭해야 하는 구역 이름 → 설명. 관측 자세마다 카메라가 보는 곳이 달라서 구역 이름에 자세가 들어 있다
ROI_HELP = {
    "arrival": "wait 자세에서 자동차(바구니)가 서는 구역",
    "note": "wait 자세에서 바구니 포스트잇이 보이는 구역",
    "stand_red": "recovery_wait/above_red 자세에서 받침대의 빨간 공 자리",
    "stand_blue": "recovery_wait/above_blue 자세에서 받침대의 파란 공 자리",
    "stand_area": "받침대 주변 (사람 손 · 다른 물체가 들어오는지 볼 구역)",
    "basket_inside": "retreat_mid 자세에서 바구니 안쪽",
    "outside": "retreat_mid 자세에서 바구니 밖 (도로 · 바닥 등 공이 떨어질 곳)",
    "gripper": "카메라 화면에서 그리퍼가 보이는 구역 (공이 물려 있는지 볼 곳)",
}

DEFAULT_ROIS = {name: [0, 0, 0, 0] for name in ROI_HELP}     # 0 = 아직 티칭 안 함


@dataclass
class MissionConfig:
    # 하드웨어
    port: str | None = None                  # None = SOARM_PORT 환경변수 / 자동 찾기. "fake" = 가상 팔
    camera: int | str = 0                    # 그리퍼 카메라 번호 (확인 필요: 장착 후 c1_find_camera.py 로 찾기)
    studio_camera: str = "wrist"             # ArmStudio 가상 카메라: wrist(손목) | top | front
    width: int = 640
    height: int = 480
    # 속도 (서보 틱/초, 200 ≈ 17.6도/초)
    speed_slow: int = 120
    speed_normal: int = 200
    # 시도
    attempts_max: int = 3
    # 그리퍼 % (0 닫힘 ~ 100 열림). 공을 쥐었을 때 값은 티칭 도구(m0_teach_poses 의 ballgrip)로 채운다
    grip_open_pct: float = 100.0
    grip_ball_min_pct: float = 0.0           # 티칭 전에는 0~0 (= 아무것도 "공을 쥠"으로 보지 않음 → 안전하게 실패 쪽)
    grip_ball_max_pct: float = 0.0
    # 구역
    rois: dict = field(default_factory=lambda: json.loads(json.dumps(DEFAULT_ROIS)))
    # 포스트잇 찾기
    note_area_min: int = 600                 # 픽셀 수
    note_area_max: int = 40000
    note_aspect_min: float = 0.5             # 가로/세로
    note_aspect_max: float = 2.0
    ball_area_min: int = 150                 # 공으로 인정하는 색 영역 최소 픽셀 수 (티칭 때 실제 값 확인)
    # 시간 (초)
    stop_seconds: float = 1.0                # 자동차가 이만큼 연속으로 멈춰 있어야 "도착"
    stop_move_px: float = 6.0                # 이 안에서 움직이면 멈춘 것으로 본다
    vote_frames: int = 15                    # 색 판정 투표에 쓰는 프레임 수
    success_stable_seconds: float = 1.0      # 바구니 안 공이 이만큼 안정되면 성공
    t_unknown: float = 6.0                   # 결과 불명 재관측 최대 시간
    t_restore_max: float = 120.0             # 공 복구 대기 최대 시간 (넘으면 안전 중단)
    t_quiet: float = 3.0                     # 받침대 구역이 이만큼 조용해야 재개
    t_left: float = 3.0                      # 자동차가 구역에서 이만큼 안 보이면 "떠남"
    # 자세 확인
    pose_tolerance_deg: float = 4.0          # clear 를 내기 전, 관절이 대기 자세와 이만큼 안에 있어야 함
    # 모터 온도 (도). 실측 후 고칠 것 (1일 차 확인 항목)
    temp_warn: float = 60.0
    temp_stop: float = 70.0
    # 통신
    state_hz: float = 1.0                    # 상태 메시지를 이 주기로 계속 발행
    ros_domain_note: str = "팀 전용 ROS_DOMAIN_ID 를 환경변수로 설정 (두 Pi 같은 값)"

    @classmethod
    def load(cls, path: Path | str = CONFIG_FILE) -> "MissionConfig":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except FileNotFoundError:
            return cls()
        cfg = cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
        for name in ROI_HELP:                # 새로 생긴 구역 이름은 기본값으로 채운다
            cfg.rois.setdefault(name, [0, 0, 0, 0])
        return cfg

    def save(self, path: Path | str = CONFIG_FILE) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=False), encoding="utf-8")
        return path

    def roi(self, name: str) -> list[int]:
        return [int(v) for v in self.rois.get(name, [0, 0, 0, 0])]

    def roi_taught(self, name: str) -> bool:
        x, y, w, h = self.roi(name)
        return w > 0 and h > 0

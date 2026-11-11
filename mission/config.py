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
    "watch": "추적 시작 자세: 멀리서 도로 전체와 다가오는 차가 보임 (추적 모드에서만 필요, 설정 8번에서 켜기)",
    "wait": "포스트잇 대기 자세: 정차 위치의 차와 포스트잇이 크고 선명하게 보임 (색을 읽는 고정 자세)",
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
    "track_road": "watch 자세에서 차가 다니는 도로 구역 (추적 모드에서만 필요. 이 안에서만 차를 찾아요)",
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

# 우리 자동차를 등록할 때 찍는 방향 (카메라에서 보이는 차의 모습). 방향마다 사진을 여러 장 찍어도 된다
CAR_DIRECTIONS = {
    "front": "앞",
    "back": "뒤",
    "left": "왼쪽 옆",
    "right": "오른쪽 옆",
    "front_left": "앞 왼쪽 비스듬히",
    "front_right": "앞 오른쪽 비스듬히",
    "back_left": "뒤 왼쪽 비스듬히",
    "back_right": "뒤 오른쪽 비스듬히",
    "top": "위에서",
}


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
    # 차 추적 (설정 8번에서 켜고 끈다. 꺼 두면 예전처럼 wait 자세 한 곳에서만 기다린다)
    tracking_enabled: bool = False
    track_pan_limit_deg: float = 25.0        # watch 자세의 받침(1번 관절) 각도에서 좌우로 이만큼까지만 돌아감
    track_pan_sign: int = 1                  # 받침을 + 방향으로 돌렸을 때 차가 화면 가운데로 오면 1, 멀어지면 -1 (허브 3번 '추적 방향 확인'이 정지한 물체로 측정해 채움)
    track_gain_deg_per_px: float = 0.04      # 차가 화면 가운데에서 1픽셀 벗어날 때 돌릴 각도 (작을수록 부드럽고 느림)
    track_deadband_px: float = 30.0          # 가운데에서 이 안이면 돌리지 않음 (떨림 방지)
    track_max_step_deg: float = 3.0          # 한 번에 최대 이만큼만 돌림 (안전)
    track_use_car_look: bool = False         # 켜면 '등록한 우리 자동차 모양'과 닮은 덩어리만 차로 인정 (꺼 두면 크기·움직임만 봄)
    track_look_min_score: float = 0.5        # 닮은 정도(0~1)가 이 값 이상이어야 우리 차로 인정
    track_period: float = 0.2                # 추적을 몇 초마다 갱신할지
    track_baseline_step_deg: float = 2.0     # 빈 도로 기준 사진을 받침 몇 도마다 한 장씩 찍을지 (작을수록 정확, 오래 걸림)
    track_baseline_tol_deg: float = 1.5      # 지금 받침 각도와 이만큼 안에 있는 기준 사진이 있어야 차를 찾음 (없으면 안전하게 '못 찾음')
    track_diff_threshold: int = 35           # 빈 도로 사진과 이만큼(0~255) 밝기가 다르면 '차 후보'
    track_min_area: int = 1200               # 차 후보 덩어리 최소 크기(픽셀 수)
    track_max_area: int = 150000             # 최대 크기 (화면 전체가 변하는 조명 변화 제외)
    track_stop_seconds: float = 2.0          # 차가 이만큼 멈춰 있으면 '정차'로 보고 wait 자세로 이동
    track_stop_px: float = 12.0              # 정차로 보는 움직임 오차(픽셀)
    track_lost_seconds: float = 3.0          # 차가 이만큼 안 보이면 추적을 멈추고 watch 자세로 돌아감
    arrival_timeout: float = 20.0            # wait 자세로 옮긴 뒤 이 시간 안에 정차 구역 확인이 안 되면 다시 추적
    # YOLO 인식 (mission/yolo.py) · 팔이 차를 세우는 상차 방식 (mission/loading.py)
    yolo_model: str = "arm_v1_yolo11n_320.onnx"   # mission/models 안의 파일 이름 (640 이 더 정확하지만 느림)
    yolo_conf: float = 0.45                  # 확신 정도가 이보다 낮은 것은 버린다 (0~1)
    load_wait_pose: str = "watch"            # 대기 자세 이름: 차가 오는 쪽을 바라보는 고정 자세
    load_stop_lead_deg: float = 2.0          # 기준 받침 각도보다 이만큼 먼저 정지 신호를 낸다 (차가 미끄러지는 만큼)
    load_stop_confirm_seconds: float = 1.5   # 정지 신호 뒤 차가 이만큼 연속으로 안 움직여야 '완전 정차' (1~2초)
    load_stop_timeout: float = 15.0          # 정지 신호 뒤 이 시간 안에 완전 정차가 안 되면 이번 차는 포기 (FAILED)
    load_pan_margin_deg: float = 15.0        # 추적할 때 받침이 '대기 자세 ~ 기준 각도' 밖으로 더 돌 수 있는 여유
    load_offset_max_deg: float = 25.0        # 차 위치 보정으로 받침을 더 돌리는 최대 각도 (넘으면 그 시도는 실패)
    load_color_frames: int = 15              # 포스트잇 색을 정할 때 보는 사진 수
    load_verify_seconds: float = 3.0         # 넣은 뒤 '공이 든 상자' 를 이 시간 동안 찾아본다
    load_verify_hits: int = 3                # 그동안 이만큼 넘게 보여야 성공
    load_retry_wait_seconds: float = 5.0     # 실패 후 다음 시도 전에 기다리는 시간 (떨어진 공을 사람이 제자리에 놓는 시간)
    load_leave_timeout: float = 30.0         # 결과를 알린 뒤 차가 떠나기를 기다리는 최대 시간
    load_see_frames: int = 2                 # 차 + 상자가 연속 이만큼 보이면 바로 정지 신호 (한 장 잘못 본 것에 속지 않게. 3 → 2: 반응 빠르게)
    load_see_period: float = 0.05            # 차를 기다리는 동안 사진 사이에 쉬는 시간(초). 작을수록 정지 신호가 빠르다 (YOLO 한 번 ≈ 0.05초)
    load_use_tracking: bool = False          # 켜면 예전 방식 (받침을 돌려 차를 따라가다 기준 각도에서 정지 신호)
    load_wait_car_power: bool = True         # 상차 미션: 차의 첫 상태 메시지(차 전원 켜짐)를 받은 뒤 대기 자세로 간다
    # 바구니 넣기 (허브 4번 학습 · 실시간 맞추기)
    place_servo_gain_deg_per_px: float = 0.03   # 바구니가 학습 자리에서 1픽셀 벗어날 때 돌릴 각도
    place_servo_deadband_px: float = 15.0       # 학습 자리에서 이 안이면 '맞췄다'
    place_servo_max_deg: float = 15.0           # 학습 자세에서 관절마다 이만큼까지만 더 움직인다 (안전)
    place_servo_seconds: float = 8.0            # 이 시간 안에 못 맞추면 그 시도는 실패
    place_servo_tilt_sign: int = 0              # 세로 맞추기 (어깨 2번): 0 = 안 함, 1 / -1 = 방향 ('저장 자세' 방식에서만)
    # YOLO 넣기 (5 바구니 위를 YOLO 로 맞추고, 6 은 수직으로 내려가고, 8 은 수직으로 올라온다)
    place_use_yolo: bool = True                 # 켜면 YOLO 넣기, 끄면 예전 '저장 자세' 방식 (학습한 모터 값 평균)
    place_step_mm: float = 10.0                 # 맞출 때 손끝을 한 번에 최대 이만큼(mm) 옮긴다
    place_max_shift_mm: float = 80.0            # 학습한 5 자리에서 이만큼(mm) 넘게 옮겨야 하면 실패 (안전)
    place_yolo_gain: float = 0.8                # 한 번에 남은 거리의 몇 배를 옮길지 (1 = 한 번에 다, 작을수록 천천히)
    place_calib_mm: float = 15.0                # 카메라 방향 재기: 앞뒤 · 좌우로 이만큼(mm) 움직여 본다
    place_extra_down_mm: float = 20.0           # 6 내려가기: 학습한 6 높이보다 이만큼 더 낮게는 절대 안 간다 (안전)
    place_down_steps: int = 3                   # 6 내려가기 / 8 올라오기를 몇 번에 나눠서 할지
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

    def required_poses(self) -> list[str]:
        """지금 설정에서 꼭 티칭해야 하는 자세 (rest 는 선택, watch 는 추적을 켰을 때만)"""
        return [n for n in POSE_HELP if n != "rest" and (n != "watch" or self.tracking_enabled)]

    def required_rois(self) -> list[str]:
        return [n for n in ROI_HELP if n != "track_road" or self.tracking_enabled]

    def roi(self, name: str) -> list[int]:
        return [int(v) for v in self.rois.get(name, [0, 0, 0, 0])]

    def roi_taught(self, name: str) -> bool:
        x, y, w, h = self.roi(name)
        return w > 0 and h > 0

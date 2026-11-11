"""배달 한방 허브: 브라우저 한 화면에서 1~12 카테고리를 골라 설정하고, 저장된 값(오차 · 시간 · 구역 · 그리퍼 %)을 직접 고친다.

실행 (Pi):
    ./"배달 한방.sh"            (메뉴에서 2~8, 11, 12 번을 고르면 이 허브가 열린다)
    .venv/bin/python -m mission.hub                       # 허브 첫 화면
    .venv/bin/python -m mission.hub --start settings      # 바로 설정 값 화면
VS Code 의 Ports 에서 8081 을 전달하고 노트북 브라우저로 http://127.0.0.1:8081 을 연다.
    .venv/bin/python -m mission.hub --lan                 # 포트 전달 없이 노트북에서 http://<Pi 의 IP>:8081 로 바로 열기
    (--lan 은 같은 와이파이의 누구나 이 화면을 열어 팔을 움직일 수 있다. 준비할 때만 쓰고, 다 쓰면 허브를 끈다)

모두 **실물**만 쓴다 (가짜 영상 · 가상 팔 없음).
- 카메라는 처음 필요한 화면에서 열고, 영상이 안 들어오면 이유와 함께 알려 준다.
- 자세 티칭은 팔이 연결돼 있을 때만 열린다 (구역 · 색 · 확인 · 설정은 카메라만 있으면 된다).
- 기존 티칭 화면(teach_preview)의 안전 규칙은 그대로다: 영상이 끊기면 저장 금지, 잠근 기준색은 수정 금지, 로컬 주소만 허용.
- 경기 중에는 SSH · 브라우저를 쓰지 않는다 (규정). 이 허브는 경기 전 준비용이다.
"""
from __future__ import annotations

import argparse
import html
import ipaddress
import json
import os
import re
import secrets
import shutil
import time
from contextlib import ExitStack
from datetime import datetime
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from .config import CAR_DIRECTIONS, CONFIG_FILE, POSE_HELP, ROI_HELP, RUNTIME_DIR, MissionConfig
from soarm import RobotError

from .store import JobStore, StoreError
from .teach_preview import POSE_LABELS, ROI_LABELS, TITLES, LatestCamera, Teaching, make_handler
from .vision import (BASELINES_DIR, COLORS_FILE, LOCK_FILE, REF_NAMES, ColorRefs, is_locked, list_car_views, lock_colors,
                     read_baselines)

# ── 1~12 카테고리 ───────────────────────────────────────────────────────
# (번호, 제목, 설명, 종류, 이동할 곳)   종류: mode = 카메라/팔 화면, page = 설명·편집 화면
OLD_CATEGORIES = [                       # 예전 1~18번 (첫 화면에서는 숨김. 주소(/go/..., /p/...)로는 그대로 열린다)
    (1, "점검", "지금 무엇이 되고 안 되는지 한눈에 (팔을 움직이지 않음)", "page", "doctor"),
    (2, "손목 카메라 보기", "실제 카메라 영상이 들어오는지 확인", "mode", "view"),
    (3, "자세 티칭", "팔을 손으로 잡아 자세 9개 + 공 쥔 집게 저장 (팔 연결 필요)", "mode", "pose"),
    (4, "구역 설정", "도착 · 포스트잇 · 받침대 · 바구니 구역을 영상에 네모로 그리기", "mode", "roi"),
    (5, "색상 등록", "빨강 · 파랑 포스트잇과 공의 기준색 등록", "mode", "color"),
    (6, "색상 확인", "지금 보이는 포스트잇을 어떻게 판정하는지 확인", "mode", "verify"),
    (7, "기준색 잠금", "등록한 기준색을 고정 / 해제 (주문 배정 전에만)", "page", "lock"),
    (8, "설정 값 · 저장 파일", "오차 범위 · 시간 · 구역 · 그리퍼 % 를 보고 직접 수정", "page", "settings"),
    (9, "미션 연습 실행", "팔 + 카메라로 ROS2 없이 한 사이클 (터미널에서 실행)", "page", "run9"),
    (10, "미션 진짜 실행", "대회 방식 실행. 경기 중에는 SSH · 브라우저 · 사람 신호를 쓰지 않음", "page", "run10"),
    (11, "작업 기록", "시도 횟수 · 완료 상태 기록 만들기 / 보기 (최초 한 번)", "page", "store"),
    (12, "차 도착 확인", "상차 구역에 멈춘 차를 실제 미션과 같은 기준으로 확인 (팔 안 움직임)", "mode", "arrival"),
    (13, "자동차 모양 등록", "우리 자동차를 앞 · 뒤 · 옆 · 비스듬히 여러 방향으로 찍어 등록 (추적에서 우리 차를 알아보는 데 씀)", "mode", "car"),
    (14, "저장된 사진 · 설정 값 보기", "빈 도로 사진, 구역을 그려 넣은 사진, 등록한 자동차 사진과 설정 값을 눈으로 확인", "page", "photos"),
    (15, "집기 연습 · 학습", "빨강·파랑 공을 단계별로 무한 반복하며 연습하고, 기록으로 그리퍼 범위를 다듬기 (팔 연결 필요, 연습 전용)", "page", "practice"),
    (16, "동작 티칭", "팔을 손으로 움직여 동작(자세 여러 개)을 가르치고, 여러 회차 중 가장 안정적인 것을 골라 저장 (빨간공 · 파란공 집기, 바구니 넣기)", "page", "motion"),
    (17, "YOLO 인식 확인", "카메라에 차 · 상자 · 포스트잇 · 공이 든 상자가 어떻게 잡히는지 네모로 확인 (팔 안 움직임)", "page", "yolo"),
    (18, "상차 미션 실행 (팔이 차를 세움)", "차를 추적 → 정지 신호 → 완전 정차 확인 → 집기 · 넣기 → 출발 신호 (터미널에서 실행)", "page", "run18"),
]
MODE_NAMES = {"view", "pose", "roi", "color", "verify", "arrival", "car"}
PAGE_NAMES = {"doctor", "lock", "settings", "run9", "run10", "store", "photos", "practice", "motion", "yolo", "run18",
              "watchpose", "track"}

# 첫 화면에 보이는 새 메뉴. 번호 색은 묶음마다 다르다 (준비 = 주황, 테스트 = 초록, 실행 = 보라, 설정 = 회색)
GROUPS = {"prep": ("준비", "처음에 한 번씩 해 두는 것"), "test": ("테스트", "팔을 움직여 보며 확인"),
          "run": ("실행", "경기 방식으로 돌리기"), "set": ("설정", "숫자 고치기")}
CATEGORIES = [
    (1, "대기 자세 저장", "차가 오는 쪽을 바라보는 고정 자세를 절대 위치로 저장하고 확인 (팔만 연결하면 됨, 카메라 필요 없음)", "page", "watchpose", "prep"),
    (2, "카메라 · YOLO 확인", "손목 카메라에 차 · 상자 · 포스트잇 · 공이 든 상자가 어떻게 잡히는지 네모로 확인 (팔 안 움직임)", "page", "yolo", "prep"),
    (3, "동작 티칭", "팔을 손으로 움직여 동작(자세 여러 개)을 가르치고 회차별로 저장 · 재생 · 선택 (빨간공 집기, 파란공 집기, 바구니 넣기)", "page", "motion", "prep"),
    (4, "차 추적 테스트", "차를 쫓아가는 것만 시험: 받침만 돌려 차(상자)를 화면 가운데에 두기. 정지 신호 · 집기는 하지 않음", "page", "track", "test"),
    (5, "상차 미션 실행", "차 추적 → 정지 신호 → 완전 정차 확인 → 집기 · 넣기 → 출발 신호 (터미널에서 실행)", "page", "run18", "run"),
    (6, "설정 값", "추적 · YOLO · 상차 숫자와 카메라 번호 · 팔 포트를 보고 직접 수정", "page", "settings", "set"),
]

# ── 설정 값 목록 (그룹, 이름, 한글 이름, 설명, 형식, 최소, 최대) ──────────────
SETTINGS = [
    ("카메라", [
        ("camera", "카메라 번호", "손목 카메라 번호. Pi 에서 /dev/video0 이면 0 (메뉴 2번 '번호 찾기'로 확인)", "camera", None, None),
        ("width", "영상 가로(픽셀)", "구역 좌표의 기준이 되는 영상 크기. 바꾸면 구역을 다시 그려야 합니다", "int", 160, 4096),
        ("height", "영상 세로(픽셀)", "위와 같음", "int", 120, 4096),
    ]),
    ("팔", [
        ("port", "팔 포트", "비우면 자동. 예: /dev/ttyACM1 (재연결하면 이름이 바뀔 수 있음)", "text", None, None),
        ("speed_slow", "느린 속도", "서보 틱/초. 200 ≈ 초당 17.6도. 처음에는 낮게", "int", 10, 1000),
        ("speed_normal", "보통 속도", "서보 틱/초", "int", 10, 2000),
    ]),
    ("그리퍼(집게) %", [
        ("grip_open_pct", "열림 값(%)", "집게를 완전히 열었을 때 (100)", "float", 0, 100),
        ("grip_ball_min_pct", "공 쥠 최소(%)", "공을 쥐었을 때 열림 정도의 아래쪽 한계 (오차 범위)", "float", 0, 100),
        ("grip_ball_max_pct", "공 쥠 최대(%)", "위쪽 한계. 0 이면 아직 티칭 안 함 (자세 티칭 3번의 '공을 쥔 집게'로 채움)", "float", 0, 100),
    ]),
    ("포스트잇 · 공 찾기", [
        ("note_area_min", "포스트잇 최소 크기(픽셀 수)", "이보다 작은 색 덩어리는 포스트잇이 아님", "int", 1, 1000000),
        ("note_area_max", "포스트잇 최대 크기(픽셀 수)", "이보다 큰 덩어리는 포스트잇이 아님", "int", 1, 1000000),
        ("note_aspect_min", "가로/세로 최소 비율", "포스트잇 모양 허용 범위 (정사각형 = 1)", "float", 0.05, 20),
        ("note_aspect_max", "가로/세로 최대 비율", "위와 같음", "float", 0.05, 20),
        ("ball_area_min", "공 최소 크기(픽셀 수)", "공으로 인정하는 색 영역 최소 크기", "int", 1, 1000000),
    ]),
    ("시간 · 오차 범위", [
        ("stop_seconds", "차가 멈췄다고 보는 시간(초)", "포스트잇이 이 시간 동안 안 움직이면 '도착'", "float", 0.1, 60),
        ("stop_move_px", "멈춘 것으로 보는 움직임 오차(픽셀)", "이 폭 안에서 움직이면 멈춘 것으로 인정", "float", 0, 200),
        ("vote_frames", "색 투표 프레임 수", "색을 이 장수만큼 보고 다수결", "int", 1, 100),
        ("success_stable_seconds", "성공 확인 시간(초)", "바구니 안에서 공이 이만큼 안정적으로 보여야 성공", "float", 0.1, 30),
        ("t_unknown", "결과 불명 재관측 시간(초)", "이 시간 안에 결론이 안 나면 실패 시도로 셈", "float", 0.5, 120),
        ("t_restore_max", "공 복구 대기 최대(초)", "넘으면 안전 중단", "float", 5, 3600),
        ("t_quiet", "받침대 조용함 시간(초)", "사람 손이 빠지고 이만큼 조용해야 재개", "float", 0.5, 60),
        ("t_left", "자동차 떠남 확인 시간(초)", "포스트잇이 이만큼 안 보이면 떠난 것", "float", 0.5, 60),
        ("pose_tolerance_deg", "자세 확인 오차(도)", "clear 를 내기 전, 관절이 대기 자세와 이만큼 안에 있어야 함", "float", 0.5, 30),
    ]),
    ("모터 온도(도)", [
        ("temp_warn", "경고 온도", "넘으면 새 시도를 시작하지 않음 (실측 후 조정)", "float", 20, 100),
        ("temp_stop", "중단 온도", "넘으면 안전 중단 (실측 후 조정)", "float", 20, 120),
    ]),
    ("통신", [
        ("state_hz", "상태 메시지 주기(Hz)", "/arm/state 를 초당 몇 번 보낼지", "float", 0.1, 20),
    ]),
    ("차 모양 확인 (선택)", [
        ("track_use_car_look", "등록한 자동차 모양으로 확인", "켜면 13번에서 등록한 우리 자동차 사진과 색 분포가 닮은 덩어리만 차로 인정해요 (엉뚱한 물체 오인 방지). 사진을 등록한 뒤에만 켜세요", "bool", None, None),
        ("track_look_min_score", "최소 닮음(0~1)", "이 값 이상으로 닮아야 우리 차로 인정. 너무 높으면 우리 차도 놓쳐요 (처음엔 0.5)", "float", 0, 1),
    ]),
    ("YOLO · 상차 (팔이 차를 세우는 방식, 5번)", [
        ("yolo_model", "YOLO 모델 파일", "mission/models 안의 파일 이름. arm_v1_yolo11n_320.onnx (빠름) 또는 arm_v1_yolo11n_640.onnx (정확, 느림)", "text", None, None),
        ("yolo_conf", "YOLO 확신 기준(0~1)", "이보다 확신이 낮은 것은 버려요. 엉뚱한 것을 잡으면 올리고, 놓치면 내리세요", "float", 0.05, 0.95),
        ("load_stop_lead_deg", "정지 신호를 미리 내는 각도(도)", "기준 받침 각도보다 이만큼 먼저 정지 신호를 내요 (차가 신호를 받고 미끄러지는 만큼)", "float", 0, 20),
        ("load_stop_confirm_seconds", "완전 정차 확인 시간(초)", "정지 신호 뒤 차가 이 시간 동안 안 움직여야 집기를 시작해요 (1~2초 권장)", "float", 0.5, 10),
        ("load_stop_timeout", "정차 기다리는 최대 시간(초)", "정지 신호 뒤 이 시간 안에 안 멈추면 이번 차는 포기하고 출발시켜요", "float", 3, 120),
        ("load_offset_max_deg", "차 위치 보정 최대 각도(도)", "차가 기준 위치에서 이만큼 넘게 벗어나 서면 넣기를 하지 않아요 (안전)", "float", 1, 60),
        ("load_verify_seconds", "성공 확인 시간(초)", "넣은 뒤 '공이 든 상자' 를 이 시간 동안 찾아봐요", "float", 0.5, 20),
        ("load_retry_wait_seconds", "다음 시도 전 대기(초)", "실패한 뒤 다음 시도까지 기다려요 (떨어진 공을 제자리에 놓는 시간)", "float", 0, 120),
    ]),
    ("차 추적", [
        ("tracking_enabled", "차 추적 사용", "켜면 추적 시작 자세(watch)에서 차를 쫓아가 정차하면 포스트잇 대기 자세(wait)로 옮겨요. 끄면 예전처럼 wait 자세 한 곳에서만 기다려요. "
                             "켜기 전에 3번에서 watch 자세, 4번에서 추적 도로 구역 + 빈 도로 기준 사진이 필요해요", "bool", None, None),
        ("track_pan_sign", "추적 방향 (+1 / -1)", "받침을 + 로 돌렸을 때 차가 가운데로 오면 1, 멀어지면 -1. 3번의 ‘추적 방향 확인’이 정지한 차로 재서 채워 줘요", "sign", -1, 1),
        ("track_pan_limit_deg", "좌우 한계(도)", "추적 시작 자세에서 받침을 좌우로 이만큼까지만 돌려요 (안전)", "float", 1, 60),
        ("track_gain_deg_per_px", "돌리는 세기(도/픽셀)", "차가 가운데에서 1픽셀 벗어날 때 돌릴 각도. 작을수록 부드럽고 느려요", "float", 0.005, 0.5),
        ("track_deadband_px", "가운데 허용 폭(픽셀)", "차가 가운데에서 이 안이면 안 돌려요 (떨림 방지)", "float", 0, 200),
        ("track_max_step_deg", "한 번에 최대 각도(도)", "한 번에 이만큼 넘게는 안 돌려요 (안전)", "float", 0.2, 10),
        ("track_period", "추적 갱신 주기(초)", "차를 몇 초마다 다시 보고 돌릴지", "float", 0.05, 2),
        ("track_diff_threshold", "차 후보 밝기 차이(0~255)", "빈 도로 사진과 이만큼 밝기가 다르면 차 후보로 봐요. 조명 변화에 오인하면 올리세요", "int", 5, 200),
        ("track_min_area", "차 최소 크기(픽셀 수)", "이보다 작은 변화는 차가 아님", "int", 50, 1000000),
        ("track_max_area", "차 최대 크기(픽셀 수)", "이보다 크면 조명이 바뀐 것으로 보고 무시", "int", 100, 1000000),
        ("track_stop_seconds", "정차로 보는 시간(초)", "차가 이 시간 동안 안 움직이면 정차로 보고 wait 자세로 가요", "float", 0.5, 30),
        ("track_stop_px", "정차로 보는 움직임 오차(픽셀)", "화면에서 이 폭 안이면 멈춘 것으로 봐요", "float", 1, 100),
        ("track_lost_seconds", "차를 놓쳤다고 보는 시간(초)", "이 시간 동안 안 보이면 추적을 멈추고 시작 자세로 돌아가요", "float", 0.5, 60),
        ("arrival_timeout", "정차 구역 확인 제한(초)", "wait 자세로 옮긴 뒤 이 시간 안에 포스트잇이 정차 구역에서 확인되지 않으면 다시 추적해요 (횡단보도 등에서 멈춘 차)", "float", 3, 300),
    ]),
]
READONLY = {"attempts_max": ("시도 최대 횟수", "규정: 3회 (수정할 수 없음)")}


def esc(value) -> str:
    return html.escape(str(value), quote=True)


CSS = """
:root{--bg:#eef2f6;--card:#fff;--ink:#16213a;--mute:#5a6785;--brand:#0f6f8f;--ok:#17803d;--bad:#b3261e;--warn:#9a6700;--line:#d6dde8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 system-ui,"Noto Sans KR",sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px 16px 48px}h1{font-size:28px;margin:8px 0}h2{font-size:20px;margin:24px 0 8px}
p.sub{color:var(--mute);margin:0 0 16px}a{color:var(--brand)}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:12px}
.card{display:block;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px 16px;text-decoration:none;color:inherit}
.card:hover,.card:focus-visible{border-color:var(--brand);outline:2px solid var(--brand)}
.card b{display:block;font-size:17px}.card span{color:var(--mute);font-size:14px}
.num{display:inline-block;min-width:28px;height:28px;line-height:28px;text-align:center;background:var(--brand);color:#fff;border-radius:14px;margin-right:8px;font-size:14px}
table{border-collapse:collapse;width:100%;background:var(--card);border:1px solid var(--line);border-radius:8px}
th,td{padding:8px 10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top;font-size:15px}th{background:#f4f7fb}
input[type=text],input[type=number]{width:100%;padding:6px 8px;border:1px solid var(--line);border-radius:6px;font:inherit}
input.roi{width:78px}button{padding:10px 16px;border-radius:8px;border:1px solid var(--brand);background:var(--brand);color:#fff;font:inherit;cursor:pointer}
button.sec{background:#fff;color:var(--brand)}button:disabled{opacity:.5;cursor:not-allowed}
.msg{padding:10px 14px;border-radius:8px;margin:12px 0;background:#fff;border:1px solid var(--line)}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
code,pre{background:#e8edf4;padding:2px 6px;border-radius:5px}pre{padding:10px 12px;overflow:auto}.small{font-size:13px;color:var(--mute)}
nav.top{margin-bottom:8px}
.num.g-prep{background:#c2410c}.num.g-test{background:#15803d}.num.g-run{background:#6d28d9}.num.g-set{background:#475569}
.card .num{color:#fff;font-size:15px;font-weight:700}
.card.g-prep{border-left:6px solid #c2410c}.card.g-test{border-left:6px solid #15803d}.card.g-run{border-left:6px solid #6d28d9}.card.g-set{border-left:6px solid #475569}
h2.grp{display:flex;align-items:baseline;gap:10px}h2.grp small{font-weight:400;font-size:14px;color:var(--mute)}
h2.grp.g-prep{color:#c2410c}h2.grp.g-test{color:#15803d}h2.grp.g-run{color:#6d28d9}h2.grp.g-set{color:#475569}
"""


def page(title: str, body: str, script: str = "") -> bytes:
    return (f"<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
            f"<title>{esc(title)} · 배달 한방</title><style>{CSS}</style></head><body><main>"
            f"<nav class=top><a href='/hub'>← 배달 한방 허브</a></nav>{body}</main>{script}</body></html>").encode()


def fmt_time(path: Path) -> str:
    try:
        return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")
    except OSError:
        return "-"


class Hub:
    """허브 전체 상태: 카메라 · 팔 · 지금 열려 있는 티칭 화면"""

    def __init__(self, arm_port=None, config_path: Path = CONFIG_FILE):
        self.arm_port = arm_port
        self.config_path = Path(config_path)
        self.backup_dir = RUNTIME_DIR / "backup"      # 지우기 전에 원본을 여기에 복사해 둔다
        self.token = secrets.token_urlsafe()
        self.camera = None
        self.arm = None
        self.teaching = None
        self.teach_cls = None
        self.quit = False
        self.practice = None                   # 15번 집기 연습 (실행 중이면 팔 · 카메라를 혼자 쓴다)
        self.allow_lan = False                 # --lan: 같은 네트워크의 다른 기기에서 Pi 의 IP 로 열 수 있게
        self.allow_fake = False                # --fake-arm: 화면 연습용 가짜 팔 허용 (16번 동작 티칭)
        self.motion_desk = None                # 16번 동작 티칭 (재생 중이면 팔을 혼자 쓴다)
        self.detector = None                   # 17번 YOLO (처음 열 때 모델을 읽는다)
        self.yolo_jpeg = None                  # 2번: 마지막으로 그린 사진
        self.watch_desk = None                 # 1번 대기 자세 저장
        self.track = None                      # 4번 차 추적 테스트 (실행 중이면 팔 · 카메라를 혼자 쓴다)
        self.resources = ExitStack()
        self.lock = threading.RLock()          # 요청은 동시에 받되, 화면 상태를 바꾸는 일은 한 번에 하나씩

    def cfg(self) -> MissionConfig:
        return MissionConfig.load(self.config_path)

    # 카메라 / 팔은 필요한 화면에서 처음 연다 → 안 되면 이유를 말한다
    def ensure_camera(self) -> str | None:
        if self.camera is not None and self.camera.snapshot() is not None:
            return None
        try:
            from soarm.color import open_camera
            cfg = self.cfg()
            if self.camera is not None:
                self.camera.close()
                self.camera = None
            self.camera = LatestCamera(open_camera(cfg.camera, cfg.width, cfg.height), sample_colors=True)
        except Exception as error:                      # 카메라 번호 · USB · 다른 프로그램이 사용 중
            return f"카메라를 열지 못했어요: {error}. 설정 값(6번)의 카메라 번호 · USB 연결 · 다른 프로그램이 카메라를 쓰는지 확인하세요."
        if not self.camera.ready.wait(timeout=3) or self.camera.snapshot() is None:
            self.camera.close()
            self.camera = None
            return "손목 카메라 영상이 들어오지 않아요. USB 연결과 카메라 번호(설정 값 6번)를 확인하세요."
        return None

    def ensure_arm(self) -> str | None:
        if self.arm is not None:
            return None
        try:
            from soarm import Robot
            from soarm.robot import default_port
            port = self.arm_port or self.cfg().port or default_port()
            if not port:
                return "팔 포트가 보이지 않아요. 팔 12V 전원과 USB 케이블을 확인하고 포트 이름(예: /dev/ttyACM1)을 설정 값(6번)에 적으세요."
            arm = Robot(port)
            if not arm.real and not self.allow_fake:
                arm.close()
                return "실물 팔 포트가 아니에요. 실물 팔 포트를 지정하세요."
            self.arm = arm
        except Exception as error:
            return f"팔에 연결하지 못했어요: {error}"
        return None

    def enter(self, mode: str) -> str | None:
        """티칭 화면 열기. 문제가 있으면 안내 문장을 돌려준다"""
        reason = self.arm_in_use()
        if reason:
            return reason
        self.leave()
        problem = self.ensure_camera()
        if problem:
            return problem
        if mode == "pose":
            problem = self.ensure_arm()
            if problem:
                return problem
        self.teaching = Teaching(self.arm if mode == "pose" else None, self.camera, self.cfg(), False, mode)
        self.teaching.deleter = lambda data: delete_saved(self, data)      # 3·4·5번 화면에서도 저장값 지우기 (백업 포함)
        self.teaching.on_finish = self.leave                                # '티칭 끝내기' → 응답 전에 허브 상태로
        self.teach_cls = make_handler(self.teaching)
        return None

    def leave(self):
        self.teaching = None
        self.teach_cls = None

    def motion_busy(self) -> bool:
        return self.motion_desk is not None and self.motion_desk.busy()

    def track_running(self) -> bool:
        return self.track is not None and self.track.running()

    def arm_in_use(self, ignore: str = "") -> str | None:
        """다른 화면이 팔을 움직이는 중이면 이유 문장 (없으면 None). ignore = 지금 여는 화면 자신"""
        if ignore != "practice" and self.practice is not None and self.practice.running():
            return "집기 연습이 팔을 쓰는 중이에요. 집기 연습 화면에서 정지한 뒤 다시 하세요."
        if ignore != "motion" and self.motion_busy():
            return "동작 티칭(3번)이 팔을 움직이는 중이에요. 3번 화면에서 정지한 뒤 다시 하세요."
        if ignore != "track" and self.track_running():
            return "차 추적 테스트(4번)가 팔을 움직이는 중이에요. 4번 화면에서 정지한 뒤 다시 하세요."
        if ignore != "watch" and self.watch_desk is not None and self.watch_desk.busy():
            return "팔이 대기 자세로 가는 중이에요. 끝난 뒤에 다시 하세요."
        return None

    def open_watch(self) -> str | None:
        """1번: 대기 자세 저장 책상 준비 (팔 연결). 문제가 있으면 안내 문장"""
        reason = self.arm_in_use(ignore="watch")
        if reason:
            return reason
        problem = self.ensure_arm()
        if problem:
            return problem
        if self.watch_desk is None or self.watch_desk.robot is not self.arm:
            from .watch_page import WatchDesk
            self.watch_desk = WatchDesk(self.arm, self.cfg())
        elif not self.watch_desk.busy():
            self.watch_desk.cfg = self.cfg()
            self.watch_desk.arm.cfg = self.watch_desk.cfg
        return None

    def get_detector(self):
        """YOLO 모델 (설정이 바뀌면 다시 읽는다). 못 읽으면 ValueError"""
        import cv2
        from .yolo import Detector
        cfg = self.cfg()
        try:
            if self.detector is None or self.detector.path.name != Path(cfg.yolo_model).name or self.detector.conf != cfg.yolo_conf:
                self.detector = Detector(cfg.yolo_model, cfg.yolo_conf)
        except (FileNotFoundError, cv2.error) as error:
            raise ValueError(f"YOLO 모델을 읽지 못했어요: {error}") from error
        return self.detector

    def start_track(self, data: dict) -> str:
        """4번: 차 추적만 시작 (받침만 돌림. 신호 · 집기 없음). 문제가 있으면 ValueError 로 이유를 말한다"""
        from .arm_io import ArmIO
        from .track_test import SnapEye, TrackTest
        if self.track_running():
            raise ValueError("이미 추적 중이에요.")
        reason = self.arm_in_use(ignore="track")
        if reason:
            raise ValueError(reason)
        self.leave()
        problem = self.ensure_camera() or self.ensure_arm()
        if problem:
            raise ValueError(problem)
        detector = self.get_detector()
        cfg = self.cfg()
        sign = data.get("sign")
        if sign not in (1, -1):
            sign = cfg.track_pan_sign
        self.track = TrackTest(cfg, ArmIO(self.arm, cfg), SnapEye(self.camera, detector), sign=sign,
                               go_start=data.get("go_start") is True)
        self.track.start()
        return "추적을 시작했어요. 차(상자)를 카메라에 보여 주세요. 멈추려면 ‘정지’."

    def open_motion(self) -> str | None:
        """16번: 동작 티칭 책상 준비 (팔 연결). 문제가 있으면 안내 문장"""
        reason = self.arm_in_use(ignore="motion")
        if reason:
            return reason
        problem = self.ensure_arm()
        if problem:
            return problem
        if self.motion_desk is None or self.motion_desk.robot is not self.arm:
            from .motion_page import MotionDesk
            from .motions import MotionError
            try:
                self.motion_desk = MotionDesk(self.arm, self.cfg())
            except MotionError as error:
                return str(error)
        elif not self.motion_desk.busy():
            self.motion_desk.cfg = self.cfg()           # 설정(속도 등)을 고쳤으면 다음 재생부터 반영
            self.motion_desk.arm.cfg = self.motion_desk.cfg
        return None

    def yolo_look(self) -> dict:
        """17번: 지금 카메라 사진에서 YOLO 가 찾은 것 (사진은 yolo_jpeg 에 그려 둔다)"""
        import cv2
        from .yolo import Detector, draw
        problem = self.ensure_camera()
        if problem:
            return {"ok": False, "message": problem, "found": []}
        try:
            self.get_detector()
        except ValueError as error:
            return {"ok": False, "message": str(error), "found": []}
        sample = self.camera.snapshot()
        if sample is None:
            return {"ok": False, "message": "카메라 영상이 끊겼어요.", "found": []}
        frame = sample[1]
        started = time.monotonic()
        found = self.detector.detect(frame)
        took = time.monotonic() - started
        ok, buf = cv2.imencode(".jpg", draw(frame, found))
        self.yolo_jpeg = buf.tobytes() if ok else None
        return {"ok": True, "message": "", "found": [d.to_dict() for d in found], "ms": round(took * 1000),
                "width": frame.shape[1], "model": self.detector.path.name}

    def start_practice(self, data: dict) -> str:
        """15번: 연습 시작. 팔과 카메라를 열고(실물만) 문제가 있으면 ValueError 로 이유를 말한다"""
        from .arm_io import ArmIO
        from .practice import Options, Practice
        from .vision import Vision
        opts = Options.from_dict(data)
        if self.practice is not None and self.practice.running():
            raise ValueError("이미 연습 중이에요.")
        reason = self.arm_in_use(ignore="practice")
        if reason:
            raise ValueError(reason)
        self.leave()
        problem = self.ensure_camera() or self.ensure_arm()
        if problem:
            raise ValueError(problem)
        cfg = self.cfg()
        try:
            refs = ColorRefs.read(COLORS_FILE)
        except (FileNotFoundError, ValueError, OSError):
            refs = ColorRefs()

        class Snap:                                    # 허브 카메라의 최신 영상 한 장씩 (같은 영상을 너무 빨리 반복해 읽지 않게 잠깐 쉼)
            def __init__(self, cam):
                self.cam = cam

            def read(self):
                time.sleep(0.03)
                sample = self.cam.snapshot()
                return None if sample is None else sample[1]

        self.practice = Practice(ArmIO(self.arm, cfg), Vision(Snap(self.camera), cfg, refs), cfg, refs)
        self.practice.start(opts)
        return "연습을 시작했어요. 정지는 언제든 누를 수 있어요."

    def close(self):
        if self.practice is not None:
            self.practice.stop()
            self.practice.join(20)
        if self.motion_busy():                          # 재생 중이면 멈춘 뒤 닫는다 (팔은 그 자리에서 힘을 준 채 선다)
            self.motion_desk.stop()
            self.motion_desk.join(20)
        if self.track_running():                        # 추적 중이면 멈춘 뒤 닫는다
            self.track.stop()
            self.track.join(10)
        if self.watch_desk is not None and self.watch_desk.busy():
            self.watch_desk.join(20)
        self.leave()
        if self.camera is not None:
            self.camera.close()
        if self.arm is not None:
            self.arm.close()


# ── 화면 만들기 ─────────────────────────────────────────────────────────
def home_page(hub: Hub, message: str = "") -> bytes:
    sections = []
    for key, (name, hint) in GROUPS.items():
        cards = []
        for num, title, desc, kind, target, group in CATEGORIES:
            if group != key:
                continue
            href = f"/go/{target}" if kind == "mode" else f"/p/{target}"
            cards.append(f"<a class='card g-{group}' href='{href}'><b><span class='num g-{group}'>{num}</span>{esc(title)}</b>"
                         f"<span>{esc(desc)}</span></a>")
        if cards:
            sections.append(f"<h2 class='grp g-{key}'>{esc(name)} <small>{esc(hint)}</small></h2><div class=grid>{''.join(cards)}</div>")
    msg = f"<p class='msg bad' role=alert>{esc(message)}</p>" if message else ""
    body = (f"<h1>배달 한방</h1><p class=sub>경기 전 준비 화면입니다. 번호 순서대로 하세요: 준비(1~3) → 테스트(4) → 실행(5). 모두 실제 카메라 · 실제 팔로만 동작합니다.</p>"
            f"{msg}{''.join(sections)}"
            f"<p class=small>저장된 값은 6번에서 언제든 고칠 수 있어요. 경기 중에는 이 화면 · SSH를 쓰지 않습니다.</p>"
            f"<p><button class=sec id=quit>허브 끝내기</button></p>")
    script = (f"<script>document.getElementById('quit').onclick=async()=>{{if(!confirm('허브를 끝낼까요?'))return;"
              f"await fetch('/api/quit',{{method:'POST',headers:{{'X-Hub-Token':'{hub.token}'}}}});document.body.innerHTML='<main><h1>허브를 닫았어요</h1><p>이 창을 닫아도 돼요.</p></main>'}}</script>")
    return page("허브", body, script)


def doctor_page() -> bytes:
    from .doctor import collect, next_step, render
    rows = collect(False, False, check_camera=False)
    cls = {"✔": "ok", "✘": "bad", "⚠": "warn"}
    trs = "".join(f"<tr><td class={cls.get(sym, '')}>{esc(sym)}</td><td>{esc(group)}</td><td>{esc(text)}"
                  + (f"<div class=small>→ {esc(hint)}</div>" if hint and sym != '✔' else "") + "</td></tr>"
                  for sym, group, text, hint in rows)
    body = (f"<h1><span class=num>1</span>점검</h1><p class=sub>읽기만 합니다. 팔을 움직이지 않아요.</p>"
            f"<table><tr><th></th><th>구분</th><th>내용</th></tr>{trs}</table>"
            f"<p class=msg><b>다음에 할 일</b><br>{esc(next_step(rows))}</p>")
    return page("점검", body)


def lock_page(hub: Hub, message: str = "") -> bytes:
    locked = is_locked(COLORS_FILE, LOCK_FILE)
    refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
    miss = [n for n in REF_NAMES if n not in refs.refs]
    state = "<b class=ok>잠김 (고정됨)</b>" if locked else "<b class=warn>안 잠김</b>"
    rows = "".join(f"<tr><td>{esc(n)}</td><td>{'등록됨' if n in refs.refs else '<span class=bad>미등록</span>'}</td></tr>" for n in REF_NAMES)
    body = (f"<h1><span class=num>7</span>기준색 잠금</h1><p class=sub>빨강 · 파랑을 모두 등록하고 6번으로 확인한 뒤 잠그세요. "
            f"잠그면 경기 중 수정할 수 없습니다 (주문이 정해진 뒤에는 고치지 않는 규칙).</p>"
            f"<p>상태: {state}</p><table><tr><th>기준색</th><th>등록</th></tr>{rows}</table>"
            f"<p><label><input type=checkbox id=sure> 주문(포스트잇)이 아직 배정되기 전입니다</label></p>"
            f"<p><button id=lock {'disabled' if locked or miss else ''}>잠그기</button> "
            f"<button class=sec id=unlock {'disabled' if not locked else ''}>잠금 풀기</button></p><p id=out class=msg hidden></p>")
    script = (f"<script>const T='{hub.token}';async function go(a){{const r=await fetch('/api/lock',{{method:'POST',headers:{{'X-Hub-Token':T,'Content-Type':'application/json'}},"
              f"body:JSON.stringify({{action:a,before_orders:document.getElementById('sure').checked}})}});const d=await r.json();"
              f"const o=document.getElementById('out');o.hidden=false;o.textContent=d.message;if(r.ok)setTimeout(()=>location.reload(),900)}}"
              f"document.getElementById('lock').onclick=()=>go('lock');document.getElementById('unlock').onclick=()=>go('unlock')</script>")
    return page("기준색 잠금", body, script)


IMG_NAME = re.compile(r"^[A-Za-z0-9_+\-]+\.png$")          # 사진 이름은 이 모양만 (폴더 밖을 읽는 요청 방지)


def safe_image(folder, name: str) -> bytes | None:
    if not IMG_NAME.match(name):
        return None
    base = Path(folder).resolve()
    target = (base / name).resolve()
    if target.parent != base or not target.is_file():
        return None
    return target.read_bytes()


def zones_overlay(hub: Hub) -> bytes | None:
    """받침 가운데(0도) 빈 도로 사진 위에, 저장한 구역(네모)을 이름과 함께 그려 넣은 사진"""
    import cv2
    shots = read_baselines()
    if not shots:
        return None
    image = (shots[0] if 0 in shots else shots[min(shots)]).copy()
    cfg = hub.cfg()
    palette = [(0, 200, 255), (60, 220, 60), (255, 120, 40), (200, 60, 220), (40, 200, 200), (30, 30, 240), (240, 200, 30), (140, 140, 255), (255, 255, 255)]
    for i, name in enumerate(ROI_LABELS):
        if cfg.roi_taught(name):
            x, y, w, h = cfg.roi(name)
            color = palette[i % len(palette)]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, name, (x + 4, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    ok, buf = cv2.imencode(".png", image)
    return buf.tobytes() if ok else None


def photos_page(hub: Hub) -> bytes:
    """저장된 사진(빈 도로, 구역 표시, 등록한 자동차)과 설정 값을 눈으로 확인하는 화면 (읽기만)"""
    from . import vision as vm
    cfg = hub.cfg()
    shot_files = sorted(Path(vm.BASELINES_DIR).glob("pan_*.png")) if Path(vm.BASELINES_DIR).exists() else []
    legacy = Path(vm.BASELINE_FILE)

    def fig(src, caption):
        return (f"<figure style='margin:0'><a href='{src}' target='_blank'><img loading=lazy src='{src}' alt='{esc(caption)}' "
                f"style='width:100%;border-radius:6px;border:1px solid var(--line)'></a><figcaption class=small>{esc(caption)}</figcaption></figure>")
    parts = ["<h1><span class=num>14</span>저장된 사진 · 설정 값 보기</h1><p class=sub>눈으로 확인만 하는 화면이에요 (바꾸는 건 3 · 4 · 5 · 13번 또는 8번). 사진을 누르면 크게 열려요.</p>"]

    zone_rows = [(n, cfg.roi(n)) for n in ROI_LABELS if cfg.roi_taught(n)]
    if shot_files or legacy.exists():
        parts.append("<h2>구역을 그려 넣은 빈 도로 사진</h2>")
        parts.append(f"<div style='max-width:640px'>{fig('/img/zones.png', '저장한 구역(네모)을 받침 가운데 빈 도로 사진 위에 표시')}</div>")
        parts.append("<p class=small>" + (" · ".join(f"{esc(ROI_LABELS[n])} {esc(r)}" for n, r in zone_rows) if zone_rows else "아직 저장한 구역이 없어요 (4번).") + "</p>")
    else:
        parts.append("<h2>빈 도로 기준 사진</h2><p class=warn>저장된 빈 도로 사진이 없어요. 3번(전체 촬영) 또는 4번(1장)에서 찍으세요.</p>")
    if shot_files or legacy.exists():
        parts.append("<h2>빈 도로 기준 사진 (받침 각도별)</h2><div class=grid style='grid-template-columns:repeat(auto-fill,minmax(180px,1fr))'>")
        for path in shot_files:
            try:
                angle = int(path.stem.split("_", 1)[1])
            except ValueError:
                continue
            parts.append(fig(f"/img/baseline/{path.name}", f"받침 {angle:+d}° · {path.name}"))
        if not shot_files and legacy.exists():
            parts.append(fig("/img/baseline/legacy.png", "받침 가운데 1장 (옛 방식으로 저장된 사진)"))
        parts.append("</div>")

    views = list_car_views()
    total = sum(len(v) for v in views.values())
    parts.append(f"<h2>등록한 우리 자동차 사진 ({total}장)</h2>")
    if total == 0:
        parts.append("<p class=warn>아직 등록한 사진이 없어요. 13번에서 여러 방향으로 찍어 등록하세요.</p>")
    for direction, paths in views.items():
        if paths:
            parts.append(f"<h3>{esc(CAR_DIRECTIONS[direction])} ({len(paths)}장)</h3><div class=grid style='grid-template-columns:repeat(auto-fill,minmax(150px,1fr))'>"
                         + "".join(fig(f"/img/car/{p.name}", p.name) for p in paths) + "</div>")

    rows = []
    for group, fields in SETTINGS:
        for name, label, _desc, kind, _lo, _hi in fields:
            value = getattr(cfg, name)
            shown = ("켜짐" if value else "꺼짐") if kind == "bool" else ("(비어 있음)" if value in (None, "") else value)
            rows.append(f"<tr><td>{esc(group)}</td><td>{esc(label)}</td><td><b>{esc(shown)}</b></td></tr>")
    parts.append("<h2>설정 값 한눈에 (읽기 전용)</h2><p class=small>고치려면 <a href='/p/settings'>8번 설정 값</a>으로 가세요.</p>"
                 f"<table><tr><th>묶음</th><th>항목</th><th>값</th></tr>{''.join(rows)}</table>")
    return page("저장된 사진", "".join(parts))


def field_input(name, label, kind, value) -> str:
    if kind == "bool":
        on = bool(value)
        return (f"<select data-name='{esc(name)}' aria-label='{esc(label)}'>"
                f"<option value='true'{' selected' if on else ''}>켜짐</option><option value='false'{'' if on else ' selected'}>꺼짐</option></select>")
    return f"<input type=text data-name='{esc(name)}' data-kind='{kind}' value='{esc(value)}' aria-label='{esc(label)}'>"


def saved_items_html(hub: Hub, cfg: MissionConfig, refs: ColorRefs, locked: bool) -> str:
    """저장된 자세 · 구역 · 기준색 · 공 쥔 집게를 보여 주고 하나씩 지울 수 있게 한다"""
    import soarm.robot as rb
    poses = rb._load_json(rb.POSES_FILE, {})

    def row(kind, name, label, info, exists, blocked=""):
        if not exists:
            button = "<span class=small>저장 안 됨</span>"
        elif blocked:
            button = f"<span class=small>{esc(blocked)}</span>"
        else:
            button = f"<button class=sec data-del-kind='{kind}' data-del-name='{esc(name)}' data-del-label='{esc(label)}'>지우기</button>"
        return f"<tr><td><b>{esc(label)}</b><div class=small>{esc(name)}</div></td><td>{info}</td><td>{button}</td></tr>"

    def table(title, rows):
        return f"<h3>{esc(title)}</h3><table><tr><th style='width:30%'>항목</th><th>저장된 값</th><th style='width:16%'></th></tr>{''.join(rows)}</table>"

    pose_rows = [row("pose", n, POSE_LABELS.get(n, n), esc(poses[n].get("timestamp", "저장됨")) if n in poses else "-", n in poses)
                 for n in POSE_HELP]
    grip_set = cfg.grip_ball_max_pct > 0
    pose_rows.append(row("ballgrip", "ballgrip", "공을 쥔 집게", f"{cfg.grip_ball_min_pct:.0f}% ~ {cfg.grip_ball_max_pct:.0f}%" if grip_set else "-", grip_set))
    roi_rows = [row("roi", n, ROI_LABELS[n], esc(cfg.roi(n)) if cfg.roi_taught(n) else "-", cfg.roi_taught(n)) for n in ROI_LABELS]
    color_rows = [row("color", n, n, (f"h={refs.refs[n]['h']} s={refs.refs[n]['s']} v={refs.refs[n]['v']}" if n in refs.refs else "-"),
                      n in refs.refs, "잠김: 7번에서 먼저 풀기" if locked else "") for n in REF_NAMES]
    car_views = list_car_views()
    car_rows = [row("car", d, f"자동차 {CAR_DIRECTIONS[d]}", f"{len(car_views[d])}장" if car_views[d] else "-", bool(car_views[d])) for d in CAR_DIRECTIONS]
    return ("<h2>저장된 항목 확인 · 지우기</h2>"
            f"<p class=small>마음에 안 드는 값은 <b>지우기</b>를 누른 뒤 3 · 4 · 5번에서 다시 저장하세요. 지우기 전에 자동으로 백업이 남아요 (<code>{esc(hub.backup_dir)}</code>). "
            "이미 저장한 값은 3 · 4 · 5번 화면에서 바로 덮어써도 됩니다.</p>"
            + table("자세", pose_rows) + table("구역", roi_rows) + table("기준색", color_rows) + table("자동차 사진", car_rows))


def delete_script(hub: Hub) -> str:
    return ("<script>document.querySelectorAll('[data-del-kind]').forEach(b=>b.onclick=async()=>{"
            "if(!confirm('‘'+b.dataset.delLabel+'’ 저장값을 지울까요?\\n지우기 전에 백업이 자동으로 남아요.'))return;"
            f"const r=await fetch('/api/delete',{{method:'POST',headers:{{'X-Hub-Token':'{hub.token}','Content-Type':'application/json'}},"
            "body:JSON.stringify({kind:b.dataset.delKind,name:b.dataset.delName})});"
            "const d=await r.json();alert(d.message);if(r.ok)location.reload()})</script>")


def backup_file(hub: Hub, path: Path) -> Path | None:
    """지우기 전에 원본을 백업 폴더에 복사 (실수로 지워도 되돌릴 수 있게)"""
    path = Path(path)
    if not path.exists():
        return None
    hub.backup_dir.mkdir(parents=True, exist_ok=True)
    dest = hub.backup_dir / f"{path.name}.{time.strftime('%Y%m%d-%H%M%S')}.bak"
    shutil.copy2(path, dest)
    return dest


def delete_saved(hub: Hub, data: dict) -> str:
    kind, name = data.get("kind"), data.get("name")
    if not isinstance(kind, str) or not isinstance(name, str):
        raise ValueError("지울 항목을 다시 선택하세요.")
    if kind == "pose":
        import soarm.robot as rb
        poses = rb._load_json(rb.POSES_FILE, {})
        if name not in POSE_HELP or name not in poses:
            raise ValueError("저장된 자세가 아니에요.")
        backup = backup_file(hub, rb.POSES_FILE)
        del poses[name]
        rb._save_json(rb.POSES_FILE, poses)
        label = POSE_LABELS.get(name, name)
    elif kind == "ballgrip":
        cfg = hub.cfg()
        if cfg.grip_ball_max_pct <= 0:
            raise ValueError("저장된 값이 없어요.")
        backup = backup_file(hub, hub.config_path)
        cfg.grip_ball_min_pct = cfg.grip_ball_max_pct = 0.0
        cfg.save(hub.config_path)
        label = "공을 쥔 집게"
    elif kind == "roi":
        cfg = hub.cfg()
        if name not in ROI_LABELS or not cfg.roi_taught(name):
            raise ValueError("저장된 구역이 아니에요.")
        backup = backup_file(hub, hub.config_path)
        cfg.rois[name] = [0, 0, 0, 0]
        cfg.save(hub.config_path)
        label = ROI_LABELS[name]
    elif kind == "car":
        from . import vision as vm
        paths = list_car_views().get(name) if name in CAR_DIRECTIONS else None
        if not paths:
            raise ValueError("등록된 사진이 없어요.")
        backup = hub.backup_dir / f"car_views.{name}.{time.strftime('%Y%m%d-%H%M%S')}"
        backup.mkdir(parents=True, exist_ok=True)
        for p in paths:                                              # 지우기 전에 사진을 백업 폴더로 옮겨 둔다
            shutil.move(str(p), str(backup / p.name))
        label = f"자동차 {CAR_DIRECTIONS[name]} 사진 {len(paths)}장"
    elif kind == "color":
        if is_locked(COLORS_FILE, LOCK_FILE):
            raise ValueError("기준색이 잠겨 있어요. 주문 배정 전이라면 7번에서 먼저 푸세요.")
        refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
        if name not in REF_NAMES or name not in refs.refs:
            raise ValueError("저장된 기준색이 아니에요.")
        backup = backup_file(hub, COLORS_FILE)
        del refs.refs[name]
        refs.save(COLORS_FILE)
        label = name
    else:
        raise ValueError("지울 항목을 다시 선택하세요.")
    where = f" 백업: {backup}" if backup else ""
    return f"‘{label}’ 저장값을 지웠어요. 다시 저장하려면 해당 번호(3 · 4 · 5번)로 가세요.{where}"


def settings_page(hub: Hub) -> bytes:
    from soarm.robot import POSES_FILE
    cfg = hub.cfg()
    groups = []
    for group, fields in SETTINGS:
        rows = []
        for name, label, desc, kind, lo, hi in fields:
            value = getattr(cfg, name)
            value = "" if value is None else value
            rng = f"<div class=small>범위 {lo} ~ {hi}</div>" if lo is not None else ""
            rows.append(f"<tr><td><b>{esc(label)}</b><div class=small>{esc(name)}</div></td>"
                        f"<td>{field_input(name, label, kind, value)}</td>"
                        f"<td>{esc(desc)}{rng}</td></tr>")
        groups.append(f"<h2>{esc(group)}</h2><table><tr><th style='width:26%'>항목</th><th style='width:20%'>값</th><th>설명</th></tr>{''.join(rows)}</table>")
    for name, (label, desc) in READONLY.items():
        groups.append(f"<p class=small>{esc(label)}: <b>{esc(getattr(cfg, name))}</b> · {esc(desc)}</p>")
    roi_rows = "".join(
        f"<tr><td><b>{esc(ROI_LABELS.get(n, n))}</b><div class=small>{esc(n)}</div></td><td>" +
        "".join(f"<input type=number class=roi data-roi='{esc(n)}' data-i='{i}' value='{v}' aria-label='{esc(n)} {'xywh'[i]}'> " for i, v in enumerate(cfg.roi(n))) +
        f"</td><td>{esc(ROI_HELP[n])}<div class=small>x, y, 가로, 세로 (픽셀). 0 이면 아직 설정 안 함. 마우스로 그리려면 4번</div></td></tr>"
        for n in ROI_LABELS)
    groups.append(f"<h2>구역 (네모 좌표)</h2><table><tr><th style='width:26%'>구역</th><th style='width:40%'>x · y · 가로 · 세로</th><th>설명</th></tr>{roi_rows}</table>")

    refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
    crow = "".join(
        f"<tr><td>{esc(n)}</td><td>h={r['h']} (±{r['h_tol']}) · s={r['s']} (±{r['s_tol']}) · v={r['v']} (±{r['v_tol']})</td><td>샘플 {r['n']}장</td></tr>"
        for n, r in sorted(refs.refs.items())) or "<tr><td colspan=3 class=warn>등록된 기준색이 없어요 (5번)</td></tr>"
    locked = is_locked(COLORS_FILE, LOCK_FILE)
    groups.append(f"<h2>기준색 (읽기 전용)</h2><p class=small>상태: {'잠김' if locked else '안 잠김'} · 허용 폭(±)이 오차 범위입니다. 고치려면 5번에서 다시 등록하세요.</p>"
                  f"<table><tr><th>이름</th><th>값 (색상 · 채도 · 밝기)</th><th></th></tr>{crow}</table>")

    files = [
        ("자세", POSES_FILE, "팔 자세 9개 (3번에서 저장)"),
        ("팔 보정값", Path(POSES_FILE).with_name("calibration.json"), "팔마다 다른 모터 위치 보정 (함부로 지우지 마세요)"),
        ("설정·구역", hub.config_path, "위의 설정 값과 구역이 저장되는 파일"),
        ("기준색", COLORS_FILE, "빨강 · 파랑 기준색 (5번에서 저장)"),
        ("추적 기준 사진", BASELINES_DIR, "받침 각도별 빈 도로 사진 폴더 (3번 '빈 도로 기준 사진 전체 촬영'). 추적을 쓰려면 필요"),
        ("작업 기록", RUNTIME_DIR / "job_a.json", "시도 횟수 · 완료 상태 (11번). 직접 고치면 안전 중단될 수 있어요"),
    ]
    frows = "".join(f"<tr><td><b>{esc(n)}</b></td><td><code>{esc(p)}</code></td><td>{'있음' if Path(p).exists() else '<span class=warn>없음</span>'} · 수정 {esc(fmt_time(Path(p)))}<div class=small>{esc(d)}</div></td></tr>"
                    for n, p, d in files)
    groups.append(f"<h2>저장된 파일</h2><table><tr><th>이름</th><th>위치</th><th>상태</th></tr>{frows}</table>")

    body = (f"<h1><span class=num>8</span>설정 값 · 저장 파일</h1><p class=sub>값을 고치고 맨 아래 <b>저장</b>을 누르세요. 잘못된 값은 저장되지 않고 이유가 나옵니다. "
            f"바꾼 값은 다음에 화면을 열 때부터 적용됩니다. <a href='/p/photos'>📷 저장된 사진 · 설정 값 한눈에 보기</a></p>{saved_items_html(hub, cfg, refs, locked)}{''.join(groups)}"
            f"<p><button id=save>저장</button></p><p id=out class=msg hidden role=status></p>")
    script = (f"<script>const T='{hub.token}';document.getElementById('save').onclick=async()=>{{"
              f"const s={{}},rois={{}};document.querySelectorAll('[data-name]').forEach(e=>s[e.dataset.name]=e.value);"
              f"document.querySelectorAll('[data-roi]').forEach(e=>{{(rois[e.dataset.roi]=rois[e.dataset.roi]||[0,0,0,0])[+e.dataset.i]=e.value}});"
              f"const r=await fetch('/api/settings',{{method:'POST',headers:{{'X-Hub-Token':T,'Content-Type':'application/json'}},body:JSON.stringify({{settings:s,rois}})}});"
              f"const d=await r.json();const o=document.getElementById('out');o.hidden=false;o.className='msg '+(r.ok?'ok':'bad');o.textContent=d.message}}</script>")
    return page("설정 값", body, script + delete_script(hub))


def store_page(hub: Hub) -> bytes:
    store = JobStore(RUNTIME_DIR)
    try:
        rec = store.load()
        info = (f"<p class=ok><b>작업 기록이 있어요.</b> 상태 <code>{esc(rec['status'])}</code> · 시도 {esc(rec['physical_attempts'])}회 · "
                f"작업 번호 {esc(rec['job_id'])} · 마지막 저장 {esc(rec['updated'])}</p><p class=small>직접 고치지 마세요. 손상되면 안전 중단됩니다.</p>")
        button = ""
    except StoreError as error:
        if error.kind == "missing":
            info, button = "<p class=warn><b>작업 기록이 아직 없어요.</b> 최초 설치 때 한 번만 만듭니다.</p>", "<button id=init>작업 기록 만들기</button>"
        else:
            info, button = f"<p class=bad><b>작업 기록을 믿을 수 없어요</b> ({esc(error.kind)}). 임의로 지우지 말고 팀과 상의하세요.</p>", ""
    body = f"<h1><span class=num>11</span>작업 기록</h1>{info}<p>{button}</p><p id=out class=msg hidden></p>"
    script = (f"<script>const b=document.getElementById('init');if(b)b.onclick=async()=>{{const r=await fetch('/api/store/init',{{method:'POST',headers:{{'X-Hub-Token':'{hub.token}'}}}});"
              f"const d=await r.json();const o=document.getElementById('out');o.hidden=false;o.textContent=d.message;if(r.ok)setTimeout(()=>location.reload(),900)}}</script>")
    return page("작업 기록", body, script)


def practice_page(hub: Hub) -> bytes:
    from .practice import COLOR_MODES, LOG_FILE, STEPS, read_log, suggest_grip, summarize
    records = read_log(LOG_FILE)
    stats, sug = summarize(records), suggest_grip(records)
    cfg = hub.cfg()
    names = {"red": "빨강", "blue": "파랑"}
    rows = "".join(
        f"<tr><td>{names[c]}</td><td>{p['cycles']}</td><td>{p['full']}</td><td class=ok>{p['success']}</td>"
        f"<td class=bad>{p['fail']}</td><td>{p['unknown']}</td></tr>" for c, p in stats["per_color"].items())
    reading = stats["reading"]
    reading_text = (f"바구니 색 읽기 정확도: {reading['correct']}/{reading['total']} 맞음 ({reading['wrong']}번 틀림)"
                    if reading["total"] else "바구니 색 읽기 정확도: 아직 정답을 알려 준 기록이 없어요")
    last = records[-8:][::-1]
    last_rows = "".join(
        f"<tr><td>{esc(r.get('ts', ''))[5:]}</td><td>{names.get(r.get('color'), '-')}</td><td>{esc(r.get('decided_by', ''))}</td>"
        f"<td>{r.get('steps_done', '-')}단계</td><td>{esc(r.get('closed_pct'))}</td><td>{esc(r.get('lifted_pct'))}</td>"
        f"<td>{esc(r.get('outcome', ''))}</td></tr>" for r in last) or "<tr><td colspan=7>아직 기록이 없어요.</td></tr>"
    step_opts = "".join(f"<option value={n}{' selected' if n == 5 else ''}>{n}단계 「{esc(name)}」 까지</option>" for n, name, _ in STEPS)
    color_opts = "".join(f"<option value={k}{' selected' if k == 'alternate' else ''}>{esc(v)}</option>" for k, v in COLOR_MODES.items())
    step_list = "".join(f"<li><b>{n}. {esc(name)}</b> — {esc(desc)}</li>" for n, name, desc in STEPS)
    apply_btn = (f"<button class=sec id=apply data-min='{sug.get('min')}' data-max='{sug.get('max')}'>이 범위를 설정에 적용</button>"
                 if sug["ok"] else "")
    body = f"""<h1><span class=num>15</span>집기 연습 · 학습</h1>
<p class=sub>연습 전용이에요(대회 실행과 분리). 같은 집기를 반복하면서 기록을 쌓고, 기록으로 ‘공을 쥔 그리퍼 %’ 범위를 다듬어요. AI(GPU) 학습이 아니라 <b>연습 결과를 모아 기준값을 고치는 방식</b>이에요.</p>
<div class=msg><b>안전</b> 팔이 실제로 움직여요. 주변을 비우고 전원 스위치에 손을 두세요. 정지 버튼은 지금 동작이 끝난 뒤(몇 초) 멈춰요. 끝나면 대기 자세로 간 뒤 토크를 꺼요.
자세(3번)·구역(4번)·기준색(5번)이 필요해요. 연습 중에는 3~6·12·13번 화면을 열 수 없어요.</div>
<h2>1. 한 번의 시도 = 5단계</h2><ol class=small>{step_list}</ol>
<h2>2. 설정하고 시작</h2>
<table><tr><th>색 고르기</th><td><select id=color>{color_opts}</select>
<div class=small>‘바구니 포스트잇 색을 읽어서’ 는 대기 자세에서 카메라로 포스트잇을 읽고(기준색 중 가까운 쪽) 같은 색 공을 넣어요.</div></td></tr>
<tr><th>어느 단계까지</th><td><select id=stop>{step_opts}</select>
<div class=small>5 미만이면 공을 받침대에 되돌려 놓고(공이 보이면 자동으로 다음 시도), 5면 바구니에 넣은 뒤 공을 다시 올려놓기를 기다려요.</div></td></tr>
<tr><th>단계마다 확인</th><td><label><input type=checkbox id=confirm> 각 단계 전에 ‘다음 단계’ 를 눌러야 진행</label></td></tr>
<tr><th>반복 횟수</th><td><input type=number id=max min=1 max=10000 placeholder="비우면 무한 반복" style="width:180px"></td></tr>
<tr><th>실제 포스트잇 색 (선택)</th><td><select id=expected><option value="">모름</option><option value=red>빨강</option><option value=blue>파랑</option></select>
<div class=small>바구니 색 읽기 모드에서 정답을 알려 주면 정확도를 세요.</div></td></tr></table>
<p><button id=start>연습 시작</button> <button class=sec id=resume>계속 / 다음 단계 / 공 올려놨어요</button> <button id=stop-btn style="background:#b3261e;border-color:#b3261e">정지</button></p>
<p class=msg id=status role=status>상태를 불러오는 중…</p>
<h2>3. 기록 ({stats['total']}번)</h2>
<table><tr><th>색</th><th>시도</th><th>5단계 끝까지</th><th>성공</th><th>실패</th><th>불명</th></tr>{rows}</table>
<p class=small>{esc(reading_text)}</p>
<table><tr><th>시각</th><th>색</th><th>색 결정</th><th>한 곳</th><th>닫힘 %</th><th>들어올린 뒤 %</th><th>결과</th></tr>{last_rows}</table>
<h2>4. 기록으로 다듬기 (그리퍼 범위)</h2>
<p>지금 설정: 공 쥠 {cfg.grip_ball_min_pct:g} ~ {cfg.grip_ball_max_pct:g}% (0 ~ 0 이면 아직 티칭 안 함)</p>
<p class={'ok' if sug['ok'] else 'warn'}>{esc(sug['message'])}</p><p>{apply_btn}</p>
<p class=small>적용하면 바뀌기 전 설정을 mission/runtime/backup 에 복사해 둬요. 기록 파일: mission/runtime/practice/log.jsonl (지우지 않고 계속 쌓여요).</p>
<p id=out class=msg hidden></p>"""
    script = f"""<script>
const H={{'X-Hub-Token':'{hub.token}','Content-Type':'application/json'}};
const $=id=>document.getElementById(id);
async function post(path,data){{const r=await fetch(path,{{method:'POST',headers:H,body:JSON.stringify(data||{{}})}});const d=await r.json();const o=$('out');o.hidden=false;o.textContent=d.message;o.className='msg '+(r.ok?'ok':'bad');return r.ok}}
$('start').onclick=()=>post('/api/practice/start',{{color_mode:$('color').value,stop_after:parseInt($('stop').value),confirm_each_step:$('confirm').checked,max_cycles:$('max').value,expected_color:$('expected').value}});
$('resume').onclick=()=>post('/api/practice/resume');
$('stop-btn').onclick=()=>post('/api/practice/stop');
const ap=$('apply');if(ap)ap.onclick=async()=>{{if(!confirm('공 쥠 범위를 '+ap.dataset.min+' ~ '+ap.dataset.max+'% 로 설정에 적용할까요?'))return;if(await post('/api/practice/apply-grip',{{min:parseFloat(ap.dataset.min),max:parseFloat(ap.dataset.max)}}))setTimeout(()=>location.reload(),900)}};
async function poll(){{try{{const d=await (await fetch('/api/practice/status',{{cache:'no-store'}})).json();
const s=$('status');const label={{idle:'대기',running:'실행 중',waiting:'기다리는 중',stopping:'멈추는 중'}}[d.state]||d.state;
s.textContent='['+label+'] '+d.message+(d.cycle?'  (시도 '+d.cycle+'번째'+(d.color?', 색 '+(d.color==='red'?'빨강':'파랑'):'')+')':'');
s.className='msg '+(d.error?'bad':d.state==='waiting'?'warn':'');
if(d.state==='idle'&&d.cycle&&!window._reloaded&&window._wasRunning){{window._reloaded=1;location.reload()}}
window._wasRunning=d.running||d.state!=='idle'}}catch(e){{$('status').textContent='허브와 연결이 끊겼어요. 허브가 켜져 있는지 확인하세요.'}}}}
poll();setInterval(poll,1000);
</script>"""
    return page("집기 연습", body, script)


def apply_grip_range(hub: Hub, data: dict) -> str:
    try:
        lo, hi = float(data.get("min")), float(data.get("max"))
    except (TypeError, ValueError):
        raise ValueError("범위 숫자가 올바르지 않아요.") from None
    if not (0 <= lo <= hi <= 100):
        raise ValueError("공 쥠 범위는 0 ≤ 최소 ≤ 최대 ≤ 100 이어야 해요.")
    if hub.practice is not None and hub.practice.running():
        raise ValueError("연습 중에는 적용할 수 없어요. 정지한 뒤 적용하세요.")
    backup_file(hub, hub.config_path)
    return apply_settings(hub, {"settings": {"grip_ball_min_pct": lo, "grip_ball_max_pct": hi}}) + f" (공 쥠 {lo:g} ~ {hi:g}%)"


def run_page(n: int) -> bytes:
    if n == 9:
        body = ("<h1><span class=num>9</span>미션 연습 실행</h1><p class=sub>팔을 실제로 움직이니 브라우저가 아니라 <b>터미널</b>에서 실행합니다. 주변을 비우고 전원 스위치에 손을 두세요.</p>"
                "<p class=small>이 허브(브라우저)가 켜져 있으면 터미널이 막혀 있어요. 허브를 Ctrl+C로 끄거나, VS Code 터미널의 <b>+</b>로 새 터미널을 여세요.</p>"
                "<pre>cd ~/so101-lab\n./\"배달 한방.sh\" --menu      # 터미널 메뉴가 뜨면 9번</pre>")
    else:
        body = ("<h1><span class=num>10</span>미션 진짜 실행</h1><p class=sub>대회 방식입니다. Pi 가 카메라로 스스로 차를 감지하고 자동 상차합니다. "
                "<b>경기 중에는 SSH · 브라우저 · 사람의 시작 신호를 쓰지 않습니다.</b> 시작 전에 준비를 모두 끝내 두세요.</p>"
                "<p class=small>이 허브(브라우저)가 켜져 있으면 터미널이 막혀 있어요. 허브를 Ctrl+C로 끄거나, VS Code 터미널의 <b>+</b>로 새 터미널을 여세요.</p>"
                "<pre>cd ~/so101-lab\n./\"배달 한방.sh\" --menu      # 터미널 메뉴가 뜨면 10번 (기준색 잠금 · 작업 기록 · 같은 ROS_DOMAIN_ID 필요)</pre>"
                "<p class=small>1번 점검이 모두 ✔ 인지 먼저 확인하세요.</p>")
    return page("실행", body)


def motion_page(hub: Hub) -> bytes:
    from .motion_page import motion_body, motion_script
    return page("동작 티칭", motion_body(), motion_script(hub.token))


def watch_page(hub: Hub) -> bytes:
    from .watch_page import watch_body, watch_script
    return page("대기 자세 저장", watch_body(), watch_script(hub.token))


TRACK_BODY = """
<h1><span class="num g-test">4</span>차 추적 테스트</h1>
<p class=sub>차를 쫓아가는 것만 시험합니다. 팔은 <b>받침(1번 모터)만</b> 좌우로 돌려 차(상자)를 화면 가운데에 두려고 해요.
<b>차에게 정지 · 출발 신호를 보내지 않고, 집기도 하지 않습니다.</b></p>
<p class=msg id=out role=status aria-live=polite>시작 전이에요.</p>
<div class=msg>
  <p><button id=start>▶ 추적 시작</button> <button id=stop class=sec disabled>■ 정지</button></p>
  <p><label><input type=checkbox id=gostart> 시작할 때 대기 자세로 먼저 이동</label>
     <span class=small>(체크하지 않으면 지금 자리에서 시작. 체크하면 팔이 움직이니 주변을 비우세요)</span></p>
  <p><label><input type=checkbox id=flip> 받침 방향 반대로 (‘방향 뒤집기’)</label>
     <button id=savesign class=sec>이 방향을 설정에 저장</button>
     <span class=small id=signtext></span></p>
  <p class=small>정지하면 팔은 그 자리에서 힘을 준 채 서 있어요. 대기 자세로 되돌리려면 <a href='/p/watchpose'>1번 대기 자세 저장</a>의 ‘대기 자세로 가기’를 쓰세요.
  급하면 팔 전원 스위치를 끄세요.</p>
</div>
<p class='msg warn' id=hint hidden></p>
<div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;align-items:start">
  <div><img id=shot alt='차(상자)를 찾은 곳을 네모로 그린 카메라 사진. 초록 세로선이 화면 가운데' style='max-width:100%;border:1px solid var(--line);border-radius:8px;min-height:120px'>
  <p class=small>초록 세로선 = 화면 가운데, 하늘색 두 선 = 가운데 허용 폭 (이 안이면 안 돌려요)</p></div>
  <table><tbody id=rows></tbody></table>
</div>
"""

TRACK_SCRIPT = """
<script>
const TOKEN='__TOKEN__';const $=id=>document.getElementById(id);let last=null,busyShot=false;
async function post(action,data){
  const r=await fetch('/api/track/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Hub-Token':TOKEN},body:JSON.stringify(data||{})});
  const d=await r.json().catch(()=>({message:'응답을 읽지 못했어요.'}));
  const o=$('out');o.textContent=d.message;o.className='msg '+(r.ok?'ok':'bad');return r.ok}
const STATE={idle:'대기',starting:'준비 중',running:'추적 중',done:'멈춤',error:'문제 발생'};
function row(a,b){return '<tr><th>'+a+'</th><td>'+b+'</td></tr>'}
function render(d){
  const active=d.state==='starting'||d.state==='running';
  $('start').disabled=active;$('stop').disabled=!active;$('gostart').disabled=active;$('flip').disabled=active;
  if(!active&&last===null)$('flip').checked=d.sign<0;
  $('signtext').textContent='지금 설정: '+(d.sign<0?'-1 (반대)':'+1 (기본)');
  if(d.message&&(active||d.state==='done'||d.state==='error')){$('out').textContent=STATE[d.state]+' · '+d.message;$('out').className='msg '+(d.state==='error'?'bad':'')}
  const h=$('hint');h.hidden=!d.hint;h.textContent=d.hint||'';
  const off=d.error_px==null?'-':(d.error_px>0?'오른쪽 ':'왼쪽 ')+Math.abs(d.error_px)+'픽셀'+(d.centered?' (가운데!)':'');
  $('rows').innerHTML=row('상태',STATE[d.state]||d.state)
    +row('찾은 것',d.found?d.label+' <span class=small>확신 '+d.conf+'</span>':'<span class=warn>안 보여요</span>')
    +row('가운데에서 벗어남',off)
    +row('받침이 처음에서',d.pan_from_center==null?'-':(d.pan_from_center>0?'+':'')+d.pan_from_center+'도 (한계 ±'+d.limit+'도'+(d.at_limit?', 닿음':'')+')')
    +row('받침 방향',d.sign<0?'-1 (반대)':'+1 (기본)')
    +row('놓친 횟수',d.lost_events)
    +row('평균 벗어남',d.mean_abs_error_px==null?'-':d.mean_abs_error_px+'픽셀')
    +row('찾은 사진 / 전체',d.found_frames+' / '+d.frames);
  if((active||d.frames>0)&&!busyShot){busyShot=true;const img=new Image();img.onload=()=>{$('shot').src=img.src;busyShot=false};img.onerror=()=>{busyShot=false};img.src='/img/track.jpg?t='+Date.now()}
  last=d}
async function tick(){try{render(await (await fetch('/api/track/state',{cache:'no-store'})).json())}catch(e){}setTimeout(tick,400)}
$('start').onclick=()=>{const go=$('gostart').checked;
  if(!confirm((go?'팔이 대기 자세로 먼저 움직인 뒤 ':'지금 자리에서 ')+'받침이 좌우로 움직이며 차를 따라갑니다. 주변을 비웠나요?'))return;
  $('start').disabled=true;$('out').className='msg';$('out').textContent='카메라와 팔을 준비하는 중… (처음에는 몇 초 걸려요)';
  post('start',{sign:$('flip').checked?-1:1,go_start:go})};
$('stop').onclick=()=>post('stop');
$('savesign').onclick=()=>{const s=$('flip').checked?-1:1;if(confirm('받침 방향을 '+(s<0?'-1 (반대)':'+1 (기본)')+' 로 설정에 저장할까요?'))post('save-sign',{sign:s})};
tick();
</script>
"""


def track_page(hub: Hub) -> bytes:
    return page("차 추적 테스트", TRACK_BODY, TRACK_SCRIPT.replace("__TOKEN__", hub.token))


def yolo_page() -> bytes:
    body = ("<h1><span class='num g-prep'>2</span>카메라 · YOLO 확인</h1><p class=sub>지금 손목 카메라에 보이는 것을 YOLO 모델이 어떻게 찾는지 보여 줍니다. 팔은 움직이지 않습니다.</p>"
            "<p class=msg id=out role=status aria-live=polite>사진을 기다리는 중…</p>"
            "<img id=shot alt='YOLO 가 찾은 것을 네모로 그린 카메라 사진' style='max-width:100%;border:1px solid var(--line);border-radius:8px'>"
            "<table style='margin-top:12px'><thead><tr><th>찾은 것</th><th>확신 (0~1)</th><th>가운데 x</th><th>화면 가운데에서</th><th>크기</th></tr></thead><tbody id=rows></tbody></table>"
            "<p class=small>상자가 화면 가운데(가운데에서 0 픽셀 근처)에 오게 받침을 맞추면 3번 ⑤ '기준 차 위치' 를 저장하기 좋습니다. "
            "공(ball)은 학습 사진이 적어 믿지 않습니다. 확신 기준은 6번 설정의 'YOLO 확신 기준'.</p>")
    script = ("<script>const $=id=>document.getElementById(id);"
              "async function tick(){try{const d=await (await fetch('/api/yolo/state',{cache:'no-store'})).json();"
              "if(!d.ok){$('out').textContent=d.message;$('out').className='msg bad'}else{"
              "$('out').className='msg';$('out').textContent='찾은 것 '+d.found.length+'개 · 한 장에 '+d.ms+'ms · 모델 '+d.model;"
              "$('shot').src='/img/yolo.jpg?t='+Date.now();"
              "$('rows').innerHTML=d.found.length?d.found.map(f=>{const cx=f.x+f.w/2;const off=Math.round(cx-d.width/2);"
              "return '<tr><td>'+f.label+' <span class=small>'+f.name+'</span></td><td>'+f.conf+'</td><td>'+Math.round(cx)+'</td><td>'+(off>0?'오른쪽 ':'왼쪽 ')+Math.abs(off)+'픽셀</td><td>'+f.w+'×'+f.h+'</td></tr>'}).join('')"
              ":'<tr><td colspan=5 class=small>아무것도 찾지 못했어요.</td></tr>'}}catch(e){}setTimeout(tick,500)}tick();</script>")
    return page("YOLO 확인", body, script)


def run18_page() -> bytes:
    body = ("<h1><span class='num g-run'>5</span>상차 미션 실행 (팔이 차를 세움)</h1>"
            "<p class=sub>팔을 실제로 움직이니 브라우저가 아니라 <b>터미널</b>에서 실행합니다. 주변을 비우고 전원 스위치에 손을 두세요.</p>"
            "<p>순서: 대기 자세 → 차 추적 → 정지 신호 → 계속 추적하며 완전 정차 확인 → 포스트잇 색 → 그 색 공 집기 → 바구니에 넣기 → 성공 확인 → 출발 신호.</p>"
            "<p class=small>먼저 필요한 것: 1번 대기 자세 저장 · 3번의 빨간공_집기 / 파란공_집기 / 바구니_넣기 (★ 선택) · 3번 ⑤ 기준 차 위치. 추적이 잘 되는지는 4번에서 먼저 확인하세요.</p>"
            "<p class=small>이 허브(브라우저)가 켜져 있으면 팔 포트를 쓰고 있어요. 허브를 끝낸 뒤 실행하세요.</p>"
            "<pre>cd ~/so101-lab\n.venv/bin/python -m mission.loading --check      # 준비가 됐는지만 확인 (팔 안 움직임)\n"
            ".venv/bin/python -m mission.loading --no-ros     # 차 통신 없이 연습 (신호는 화면에만 출력)\n"
            ".venv/bin/python -m mission.loading              # 진짜: ROS2 로 차와 통신 (Docker 안에서)</pre>")
    return page("상차 미션 실행", body)


# ── 설정 저장 ───────────────────────────────────────────────────────────
def apply_settings(hub: Hub, data: dict) -> str:
    cfg = hub.cfg()
    values = data.get("settings")
    rois = data.get("rois", {})
    if not isinstance(values, dict) or not isinstance(rois, dict):
        raise ValueError("요청 형식이 올바르지 않아요.")
    known = {name: (label, kind, lo, hi) for _, fields in SETTINGS for name, label, _, kind, lo, hi in fields}
    for name, raw in values.items():
        if name not in known:
            raise ValueError(f"고칠 수 없는 항목이에요: {name}")
        label, kind, lo, hi = known[name]
        text = str(raw).strip()
        if kind == "text":
            setattr(cfg, name, text or None)
            continue
        if kind == "bool":
            if text.lower() not in ("true", "false", "1", "0", "켜짐", "꺼짐", "on", "off"):
                raise ValueError(f"'{label}' 은 켜짐 또는 꺼짐이어야 해요.")
            setattr(cfg, name, text.lower() in ("true", "1", "켜짐", "on"))
            continue
        if kind == "sign":
            if text not in ("1", "+1", "-1"):
                raise ValueError(f"'{label}' 은 1 또는 -1 이어야 해요.")
            setattr(cfg, name, -1 if text == "-1" else 1)
            continue
        if kind == "camera":
            try:
                num = int(text)
            except ValueError:
                if not text.startswith("/dev/") and not text.startswith("http"):
                    raise ValueError(f"'{label}' 은 번호(예: 0) 또는 /dev/video0 같은 경로여야 해요.") from None
                setattr(cfg, name, text)
                continue
            if not 0 <= num <= 63:
                raise ValueError(f"'{label}' 은 0~63 사이 번호여야 해요.")
            setattr(cfg, name, num)
            continue
        try:
            num = int(text) if kind == "int" else float(text)
        except ValueError:
            raise ValueError(f"'{label}' 에 숫자를 넣어 주세요 (지금 값: '{text}').") from None
        if (num != num) or not lo <= num <= hi:
            raise ValueError(f"'{label}' 은 {lo} ~ {hi} 사이여야 해요 (지금 값: {num}).")
        setattr(cfg, name, num)
    for name, rect in rois.items():
        if name not in ROI_LABELS or not isinstance(rect, list) or len(rect) != 4:
            raise ValueError(f"구역 값이 올바르지 않아요: {name}")
        try:
            x, y, w, h = [int(float(v)) for v in rect]
        except (ValueError, TypeError):
            raise ValueError(f"구역 '{ROI_LABELS[name]}' 에 숫자를 넣어 주세요.") from None
        if min(x, y, w, h) < 0 or ((w == 0) != (h == 0)):
            raise ValueError(f"구역 '{ROI_LABELS[name]}': 0 이상 숫자여야 하고, 미설정은 가로·세로를 모두 0으로 두세요.")
        if w and (x + w > cfg.width or y + h > cfg.height):
            raise ValueError(f"구역 '{ROI_LABELS[name]}' 이 영상({cfg.width}×{cfg.height}) 밖으로 나가요.")
        cfg.rois[name] = [x, y, w, h]
    if cfg.grip_ball_min_pct > cfg.grip_ball_max_pct:
        raise ValueError("공 쥠 최소(%)가 최대(%)보다 클 수 없어요.")
    if cfg.note_area_min > cfg.note_area_max:
        raise ValueError("포스트잇 최소 크기가 최대 크기보다 클 수 없어요.")
    if cfg.note_aspect_min > cfg.note_aspect_max:
        raise ValueError("가로/세로 최소 비율이 최대보다 클 수 없어요.")
    if cfg.temp_warn >= cfg.temp_stop:
        raise ValueError("경고 온도는 중단 온도보다 낮아야 해요.")
    if cfg.track_min_area >= cfg.track_max_area:
        raise ValueError("차 최소 크기가 최대 크기보다 작아야 해요.")
    cfg.save(hub.config_path)
    return "저장했어요. 바꾼 값은 다음에 화면을 열 때부터 적용돼요."


# ── 서버 ────────────────────────────────────────────────────────────────
def make_router(hub: Hub):
    class Router(BaseHTTPRequestHandler):
        timeout = 3

        def log_message(self, *args):
            pass

        def send(self, status, content_type, body=b"", result=""):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            if result:
                self.send_header("X-Camera-Result", result)
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, status, data):
            self.send(status, "application/json; charset=utf-8", json.dumps(data, ensure_ascii=False).encode())

        def redirect(self, where):
            self.send_response(303)
            self.send_header("Location", where)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def local_host(self):
            host = self.headers.get("Host", "").partition(":")[0]
            if host in ("127.0.0.1", "localhost"):
                return True
            if not hub.allow_lan:
                return False
            try:                                         # --lan: 숫자 IP 주소로 연 것만 받는다 (도메인 이름은 거절)
                ipaddress.IPv4Address(host)
                return True
            except ValueError:
                return False

        def html(self, body):
            self.send(200, "text/html; charset=utf-8", body)

        def delegate(self, method):
            """지금 열린 티칭 화면(teach_preview)에 맡긴다. 첫 화면 HTML 에는 허브로 돌아가는 링크를 붙인다"""
            if method == "GET" and self.path == "/":
                original = self.send

                def with_nav(status, ctype, body=b"", result=""):
                    if ctype.startswith("text/html"):
                        link = "style='color:#fff;font-weight:700;margin-right:18px'"
                        nav = (f"<div style='padding:8px 16px;background:#0f6f8f'><a href='/hub' {link}>← 배달 한방 허브로</a>"
                               f"<a href='/p/photos' target='_blank' {link}>📷 저장된 사진 보기</a>"
                               f"<a href='/p/settings' target='_blank' {link}>⚙ 설정 값 보기</a></div>").encode()
                        body = body.replace(b"<main>", nav + b"<main>", 1)      # 티칭 화면 HTML 에는 <body> 가 없다
                    original(status, ctype, body, result)
                self.send = with_nav
            try:
                getattr(hub.teach_cls, f"do_{method}")(self)
            finally:
                self.__dict__.pop("send", None)
            if hub.teaching is not None and hub.teaching.finished:
                hub.leave()                                  # '티칭 끝내기' → 허브로

        def do_GET(self):
            with hub.lock:
                self.serve_get()

        def do_POST(self):
            with hub.lock:
                self.serve_post()

        def serve_get(self):
            if not self.local_host():
                return self.send(403, "text/plain")
            path = urlparse(self.path).path
            if hub.teach_cls is not None and (path in ("/", "/frame", "/state") or path.startswith("/frozen/")):
                return self.delegate("GET")
            if path in ("/", "/hub"):
                hub.leave()
                return self.html(home_page(hub))
            if path.startswith("/go/"):
                mode = path[4:]
                if mode not in MODE_NAMES:
                    return self.send(404, "text/plain")
                problem = hub.enter(mode)
                if problem:
                    return self.html(home_page(hub, problem))
                return self.redirect("/")
            if path == "/api/practice/status":                       # 연습 상태 (읽기만)
                state = hub.practice.status() if hub.practice is not None else {
                    "state": "idle", "message": "연습을 시작하지 않았어요.", "cycle": 0, "step": 0, "color": None,
                    "error": None, "last": None, "running": False}
                return self.send_json(200, state)
            if path == "/api/motion/state":                          # 16번 상태 (읽기만, 팔을 건드리지 않음)
                if hub.motion_desk is None:
                    return self.send_json(409, {"message": "동작 티칭 화면을 다시 열어 주세요."})
                return self.send_json(200, hub.motion_desk.state())
            if path == "/api/watch/state":                           # 1번 상태 (읽기만)
                if hub.watch_desk is None:
                    return self.send_json(409, {"message": "대기 자세 저장 화면을 다시 열어 주세요."})
                return self.send_json(200, hub.watch_desk.state())
            if path == "/api/track/state":                           # 4번 상태 (읽기만)
                from .track_test import idle_status
                return self.send_json(200, hub.track.status() if hub.track is not None else idle_status(hub.cfg()))
            if path == "/img/track.jpg":
                data = hub.track.eye.render(hub.cfg().track_deadband_px) if hub.track is not None else None
                return self.send(200, "image/jpeg", data) if data else self.send(404, "text/plain")
            if path == "/api/yolo/state":                            # 17번: 지금 사진에서 찾은 것
                return self.send_json(200, hub.yolo_look())
            if path == "/img/yolo.jpg":
                return self.send(200, "image/jpeg", hub.yolo_jpeg) if hub.yolo_jpeg else self.send(404, "text/plain")
            if path.startswith("/img/"):                              # 저장된 사진 (읽기만, 허용된 폴더 안의 .png 만)
                from . import vision as vm
                parts = path.split("/")
                data = None
                if path == "/img/zones.png":
                    data = zones_overlay(hub)
                elif len(parts) == 4 and parts[2] == "baseline":
                    data = (Path(vm.BASELINE_FILE).read_bytes() if parts[3] == "legacy.png" and Path(vm.BASELINE_FILE).exists()
                            else safe_image(vm.BASELINES_DIR, parts[3]))
                elif len(parts) == 4 and parts[2] == "car":
                    data = safe_image(vm.CAR_VIEWS_DIR, parts[3])
                return self.send(200, "image/png", data) if data else self.send(404, "text/plain")
            if path.startswith("/p/"):
                name = path[3:]
                if name not in ("photos", "settings"):               # 보기 화면은 열려 있는 티칭 화면을 닫지 않는다 (새 탭에서 열어도 안전)
                    hub.leave()
                if name == "photos":
                    return self.html(photos_page(hub))
                if name == "watchpose":
                    problem = hub.open_watch()
                    return self.html(home_page(hub, problem) if problem else watch_page(hub))
                if name == "track":
                    return self.html(track_page(hub))
                if name == "motion":
                    problem = hub.open_motion()
                    return self.html(home_page(hub, problem) if problem else motion_page(hub))
                if name == "yolo":
                    return self.html(yolo_page())
                if name == "run18":
                    return self.html(run18_page())
                if name == "practice":
                    return self.html(practice_page(hub))
                if name == "doctor":
                    return self.html(doctor_page())
                if name == "lock":
                    return self.html(lock_page(hub))
                if name == "settings":
                    return self.html(settings_page(hub))
                if name == "store":
                    return self.html(store_page(hub))
                if name in ("run9", "run10"):
                    return self.html(run_page(int(name[3:])))
            self.send(404, "text/plain")

        def drain(self):
            """거절하기 전에 상대가 보낸 본문을 읽어서 버린다 (안 읽고 닫으면 보낸 쪽이 연결 끊김 오류를 받을 수 있다)"""
            try:
                size = int(self.headers.get("Content-Length", 0))
                if 0 < size <= 16384:
                    self.rfile.read(size)
            except (ValueError, OSError):
                pass

        def serve_post(self):
            path = urlparse(self.path).path
            if not self.local_host():
                self.drain()
                return self.send(403, "text/plain")
            if path == "/action" and hub.teach_cls is not None:
                return self.delegate("POST")
            if self.headers.get("X-Hub-Token") != hub.token:
                self.drain()
                return self.send(403, "text/plain")
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 <= size <= 16384:
                    raise ValueError("요청이 너무 커요.")
                data = json.loads(self.rfile.read(size) or b"{}")
                if path == "/api/quit":
                    hub.quit = True
                    return self.send_json(200, {"message": "허브를 닫아요."})
                if path == "/api/settings":
                    return self.send_json(200, {"message": apply_settings(hub, data)})
                if path == "/api/delete":
                    return self.send_json(200, {"message": delete_saved(hub, data)})
                if path == "/api/lock":
                    action = data.get("action")
                    if action == "lock":
                        lock_colors(COLORS_FILE, LOCK_FILE)
                        return self.send_json(200, {"message": "잠갔어요. 이제 기준색을 고칠 수 없어요."})
                    if action == "unlock":
                        if data.get("before_orders") is not True:
                            raise ValueError("주문이 아직 배정되기 전인지 먼저 확인(체크)하세요.")
                        LOCK_FILE.unlink(missing_ok=True)
                        return self.send_json(200, {"message": "잠금을 풀었어요. 수정한 뒤 다시 잠그세요."})
                    raise ValueError("요청이 올바르지 않아요.")
                if path.startswith("/api/watch/"):
                    if hub.watch_desk is None:
                        raise ValueError("대기 자세 저장 화면을 다시 열어 주세요.")
                    reason = hub.arm_in_use(ignore="watch")
                    if reason:
                        raise ValueError(reason)
                    return self.send_json(200, {"message": hub.watch_desk.handle(path[len("/api/watch/"):], data)})
                if path == "/api/track/start":
                    return self.send_json(200, {"message": hub.start_track(data)})
                if path == "/api/track/stop":
                    if not hub.track_running():
                        raise ValueError("지금 추적 중이 아니에요.")
                    hub.track.stop()
                    return self.send_json(200, {"message": "정지 요청을 보냈어요. 팔은 그 자리에서 힘을 준 채 멈춰요."})
                if path == "/api/track/save-sign":
                    sign = data.get("sign")
                    if sign not in (1, -1):
                        raise ValueError("방향은 1 또는 -1 이어야 해요.")
                    cfg = hub.cfg()
                    cfg.track_pan_sign = sign
                    cfg.save(hub.config_path)
                    return self.send_json(200, {"message": f"받침 방향을 {sign:+d} 로 저장했어요. 다음 추적부터 적용돼요."})
                if path.startswith("/api/motion/"):
                    if hub.motion_desk is None:
                        raise ValueError("동작 티칭 화면을 다시 열어 주세요.")
                    reason = hub.arm_in_use(ignore="motion")
                    if reason:
                        raise ValueError(reason)
                    return self.send_json(200, {"message": hub.motion_desk.handle(path[len("/api/motion/"):], data)})
                if path == "/api/practice/start":
                    return self.send_json(200, {"message": hub.start_practice(data)})
                if path in ("/api/practice/resume", "/api/practice/stop"):
                    if hub.practice is None or not hub.practice.running():
                        raise ValueError("지금 연습 중이 아니에요.")
                    (hub.practice.resume if path.endswith("resume") else hub.practice.stop)()
                    return self.send_json(200, {"message": "보냈어요." if path.endswith("resume") else "정지 요청을 보냈어요. 지금 동작이 끝나면 멈춰요."})
                if path == "/api/practice/apply-grip":
                    return self.send_json(200, {"message": apply_grip_range(hub, data)})
                if path == "/api/store/init":
                    JobStore(RUNTIME_DIR).init_fresh()
                    return self.send_json(200, {"message": "작업 기록을 만들었어요."})
                self.send(404, "text/plain")
            except (ValueError, OSError, StoreError, json.JSONDecodeError, RobotError) as error:
                self.send_json(400, {"message": str(error)})

    return Router


def lan_ip() -> str:
    """이 기기가 와이파이에서 쓰는 IP 주소 (실제로 아무것도 보내지 않고 알아낸다)"""
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))
            return s.getsockname()[0]
    except OSError:
        return "<이 기기의 IP>"


def make_server(hub: Hub, web_port: int = 8081) -> ThreadingHTTPServer:
    """요청을 여러 개 동시에 받는 서버. 브라우저가 열어 둔 빈 연결 · 예전 탭이 서버를 막지 못한다"""
    server = ThreadingHTTPServer(("0.0.0.0" if hub.allow_lan else "127.0.0.1", web_port), make_router(hub))
    server.daemon_threads = True
    return server


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="배달 한방 허브 (브라우저)")
    parser.add_argument("--port", help="실물 팔 포트 (예: /dev/ttyACM1). 안 적으면 설정 값 · 자동 찾기")
    parser.add_argument("--web-port", type=int, default=8081)
    parser.add_argument("--lan", action="store_true",
                        help="포트 전달 없이 같은 와이파이의 노트북에서 http://<Pi 의 IP>:8081 로 열기 (누구나 열 수 있으니 준비할 때만)")
    parser.add_argument("--fake-arm", action="store_true", help="화면 연습용 가짜 팔 (16번 동작 티칭 화면을 팔 없이 눌러 볼 때)")
    parser.add_argument("--start", choices=sorted(MODE_NAMES | PAGE_NAMES), help="바로 열 화면 (안내 문구에 주소를 붙인다)")
    args = parser.parse_args(argv)
    hub = Hub("fake" if args.fake_arm else args.port)
    hub.allow_fake = args.fake_arm
    hub.allow_lan = args.lan
    try:
        server = make_server(hub, args.web_port)
    except OSError as error:
        print(f"허브를 열지 못했어요: {error} (이미 열려 있으면 그 화면을 새로고침하세요)")
        return 1
    server.timeout = 0.5
    start = ""
    if args.start:
        start = f"go/{args.start}" if args.start in MODE_NAMES else f"p/{args.start}"
    url = f"http://127.0.0.1:{server.server_port}/{start}"
    print(f"배달 한방 허브: {url}", flush=True)
    if hub.allow_lan:
        print(f"같은 와이파이의 노트북에서: http://{lan_ip()}:{server.server_port}/{start}", flush=True)
        print("주의: 같은 와이파이의 누구나 이 화면을 열어 팔을 움직일 수 있어요. 다 쓰면 허브를 끄세요.", flush=True)
    if os.name == "nt" and not os.environ.get("SSH_CONNECTION"):      # 이 PC 에서 실행하면 브라우저를 바로 연다
        import webbrowser
        webbrowser.open(url)
    print(f"Pi에 접속 중이면 VS Code의 Ports에서 {server.server_port} 포트를 전달한 뒤 브라우저로 여세요.", flush=True)
    print("이미 열려 있는 탭은 새로고침하세요. 끝내려면 허브의 ‘허브 끝내기’ 또는 이 터미널에서 Ctrl+C.", flush=True)
    try:
        while not hub.quit:
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        hub.close()
        server.server_close()
    print("허브를 닫았어요.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

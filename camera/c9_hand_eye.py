"""카메라 9 — 카메라 ↔ 로봇 캘리브레이션: 로봇이 빨간 공을 물고 여러 자리로 가며 '픽셀 ↔ 로봇 좌표' 짝을 모은다.

    python camera/c9_hand_eye.py                     세로축만: 받침 0°, 앞뒤 5자리 → 픽셀 → x (1차원)
    python camera/c9_hand_eye.py --pans -20 0 20     받침을 돌려 책상 평면 전체 → 픽셀 → (x, y) (호모그래피)
    python camera/c9_hand_eye.py --teach             직접 티칭: 토크를 끄고 공을 문 팔을 손으로 옮기며 Enter (자리 수는 마음대로)
    python camera/c9_hand_eye.py --ruler             팔 없이: 자로 잰 자리에 공을 놓고 거리(cm)를 친다
    python camera/c9_hand_eye.py --check             맞춘 결과로: 책상 위 빨간 공 → 로봇 좌표 (창 없이 글자로)
    포트: SOARM_PORT (팔 없이 시험: SOARM_PORT=fake)

세 길(자동 · 티칭 · 자)은 같은 파일(camera/hand_eye.json)을 만든다. 업계의 '9점 캘리브레이션' 과 같은 생각이다:
고정 카메라 앞에서 로봇이 표식(여기서는 공)을 여러 자리로 옮기고, 자리마다 픽셀 ↔ 로봇 좌표를 짝지어 변환을 구한다.

깊이(거리)를 모르는 카메라 하나로는 3D 를 알 수 없다. 그래서 '공은 늘 책상 위' 로 정한다.
그러면 공 중심은 늘 같은 높이(책상 + 공 반지름)의 평면 위에 있고, 그 평면 위에서는 픽셀 ↔ 로봇 좌표가 식 하나로 맞는다.
로봇 쪽 좌표는 정기구학(이론 덱)으로 계산한 집게 끝 = 문 공의 중심.

순서
  1. 공을 집게에 물린다 (Enter → 집게가 닫힌다). 팔이 공을 물고 책상 바로 위를 앞뒤로 간다
  2. 자리마다: 멈춤 → 사진 5장 → 빨간 덩어리 중심 (u, v) 의 가운데 값 + 정기구학 (x, y, z)
  3. 맞추기 → 한 점씩 빼고 맞춰 그 점을 맞혀 보는 오차 (mm) → camera/hand_eye.json

할 일
  1. 세로축만 해 보고, 한 점씩 뺀 오차가 몇 mm 인지 본다
  2. --check 로 공을 손으로 책상 위에 굴려 x 가 맞게 나오는지 본다 (세로축만이면 공을 받침 앞 직선 위에)
  3. --pans 로 평면 전체를 맞추고, c5 (손끝으로 짚기) 와 오차를 비교한다
"""
import json
import sys
import time

import numpy as np

from camlib import HERE, TableMap, find_blob, hand_eye_xy, ik_down, load_settings, move_urdf, open_cam, tip_xyz, urdf_deg
from soarm import Robot

TABLE_Z = 0.0           # 책상 높이 (로봇 좌표 m). 팔 받침이 책상 위에 바로 있으면 0
RADIUS = 0.03           # 공 반지름 (m). 지름 6 cm 공
HOVER = 0.0             # 공을 책상에서 띄우는 높이 (m). 0 = 책상에 닿게. 띄우면 '맞춘 평면' 이 높아져 놓인 공이 빗나간다 (--hover 0.005)
XS = [0.14, 0.17, 0.20, 0.23, 0.26]    # 앞뒤 자리 (m). 집게를 세운 채 닿는 곳 (약 0.30 까지)
SPEED = 120
OUT = HERE / "hand_eye.json"


def arg_list(name, default):
    if name not in sys.argv:
        return default
    i = sys.argv.index(name)
    return [float(a) for a in sys.argv[i + 1:] if not a.startswith("--")]


def ball_pixel(cap, n=5):
    """사진 n 장의 덩어리 중심 가운데 값 (흔들림 줄이기). 못 찾으면 None"""
    for _ in range(3):
        cap.read()
    pts = []
    for _ in range(n):
        ok, frame = cap.read()
        b = find_blob(frame, "red") if ok else None
        if b:
            pts.append((b["u"], b["v"]))
    if len(pts) < n // 2 + 1:
        return None
    return [float(np.median([p[0] for p in pts])), float(np.median([p[1] for p in pts]))]


# ── 맞추기 ────────────────────────────────────────────────────────────
def fit_line(pixels, xs):
    """세로축만: 픽셀을 점들이 늘어선 방향으로 투영한 s 와 x 사이의 1차원 원근식 x = (a s + b) / (c s + 1)"""
    P = np.asarray(pixels, float)
    mean = P.mean(axis=0)
    d = np.linalg.svd(P - mean)[2][0]                  # 점들이 늘어선 방향 (단위 벡터)
    s = (P - mean) @ d
    A = np.column_stack([s, np.ones_like(s), -s * np.asarray(xs)])
    a, b, c = np.linalg.lstsq(A, np.asarray(xs), rcond=None)[0]
    return {"mode": "line", "mean": mean.tolist(), "dir": d.tolist(), "abc": [float(a), float(b), float(c)]}


to_robot = hand_eye_xy                                     # 맞춘 식으로 픽셀 → 로봇 (x, y). camlib 에 있다 (c10 도 쓴다)


def fit(pixels, pts):
    xy = [p[:2] for p in pts]
    if len({round(p[1], 3) for p in pts}) == 1:        # y 가 모두 같다 = 받침 하나 = 세로축만
        m = fit_line(pixels, [p[0] for p in xy])
        m["y"] = float(pts[0][1])
    else:
        tm = TableMap.fit(pixels, pts)
        m = {"mode": "plane", "H": tm.H.tolist()}
    m["z"] = float(np.mean([p[2] for p in pts]))
    return m


def leave_one_out(pixels, pts):
    """한 점을 빼고 맞춘 식으로 그 점을 맞혀 본 오차 (mm). 맞출 때 쓴 점의 오차보다 정직하다"""
    errs = []
    need = 3 if len({round(p[1], 3) for p in pts}) == 1 else 4
    for i in range(len(pts)):
        rest = [k for k in range(len(pts)) if k != i]
        if len(rest) < need:
            return []
        m = fit([pixels[k] for k in rest], [pts[k] for k in rest])
        x, y = to_robot(m, *pixels[i])
        errs.append(round(float(np.hypot(x - pts[i][0], y - pts[i][1])) * 1000, 1))
    return errs


# ── 확인 (--check) ────────────────────────────────────────────────────
if "--check" in sys.argv:
    model = json.loads(OUT.read_text(encoding="utf-8"))
    cap = open_cam(load_settings())
    t_end = time.time() + float(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else None
    print(f"맞춘 식: {model['mode']} · 공 중심 높이 z = {model['z']:.3f} m (Ctrl+C 끝)")
    try:
        while t_end is None or time.time() < t_end:
            p = ball_pixel(cap, 3)
            if p:
                x, y = to_robot(model, *p)
                print(f"픽셀 ({p[0]:6.1f}, {p[1]:6.1f}) → 로봇 x = {x:.3f} m" + (f", y = {y:.3f} m" if model["mode"] == "plane" else ""))
            else:
                print("빨간 공이 안 보임")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    cap.release()
    sys.exit()


def finish(pixels, pts):
    """모은 짝으로 맞추고 · 한 점씩 빼고 오차 · 저장 (세 길 공통)"""
    if len(pts) < 3:
        sys.exit("점이 3개보다 적어 맞출 수 없습니다 (공이 화면 안에 있는지, 닿는 자리인지)")
    model = fit(pixels, pts)
    model["pixels"], model["points"] = pixels, pts
    errs = leave_one_out(pixels, pts)
    OUT.write_text(json.dumps(model, indent=2), encoding="utf-8")
    print(f"\n맞춘 식: {'세로축 (픽셀 → x)' if model['mode'] == 'line' else '평면 (픽셀 → x, y)'}, 점 {len(pts)}개")
    print(f"한 점씩 빼고 맞혀 본 오차 (mm): {errs}" + (f"  평균 {np.mean(errs):.1f}, 최대 {max(errs):.1f}" if errs else ""))
    print("저장:", OUT)
    sys.exit()


# ── 길 2: 직접 티칭 (--teach) ─────────────────────────────────────────
if "--teach" in sys.argv:
    cap = open_cam(load_settings())
    bot = Robot()
    bot.torque(False)
    bot.torque(True, "gripper")                             # 집게만 힘을 줘서 공을 문다
    input("빨간 공을 집게 사이에 넣고 Enter → 집게가 닫힌다 (팔의 다른 관절은 힘이 풀려 있다. 한 손으로 받치기) ")
    bot.close_gripper()
    pixels, pts = [], []
    while True:
        s = input(f"[{len(pts) + 1}] 공이 바닥에 닿게 팔을 옮기고 Enter (끝은 q Enter) ").strip().lower()
        if s == "q":
            break
        p, xyz = ball_pixel(cap), tip_xyz(bot)
        if p is None:
            print("   공이 안 보임 → 다시")
            continue
        pixels.append(p)
        pts.append(xyz)
        print(f"   픽셀 ({p[0]:6.1f}, {p[1]:6.1f})  ↔  공 중심 ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m")
    cap.release()
    bot.torque(True)                                        # 그 자리에 굳힌다 (내려놓을 때는 torque(False) 전에 받치기)
    bot.close()
    finish(pixels, pts)

# ── 길 3: 팔 없이 자로 (--ruler) ──────────────────────────────────────
if "--ruler" in sys.argv:
    cap = open_cam(load_settings())
    print("팔 받침 가운데에서 앞으로 곧게 테이프를 붙이고 cm 눈금을 적는다. 공을 눈금 위에 놓고 그 거리를 친다.")
    print("세로축만이면 x 만 (예: 18). 옆으로도 재면 'x y' (예: 18 -5, y 는 왼쪽이 +)")
    pixels, pts = [], []
    while True:
        s = input(f"[{len(pts) + 1}] 공까지 거리 cm (끝은 q): ").strip().lower()
        if s == "q":
            break
        try:
            nums = [float(v) / 100 for v in s.replace(",", " ").split()]
        except ValueError:
            print("   숫자로 (예: 18 또는 18 -5)")
            continue
        p = ball_pixel(cap)
        if p is None or not nums:
            print("   공이 안 보임 → 다시")
            continue
        pixels.append(p)
        pts.append([nums[0], nums[1] if len(nums) > 1 else 0.0, TABLE_Z + RADIUS])
        print(f"   픽셀 ({p[0]:6.1f}, {p[1]:6.1f})  ↔  ({pts[-1][0]:.3f}, {pts[-1][1]:.3f}) m")
    cap.release()
    finish(pixels, pts)

# ── 길 1: 자동 (로봇이 공을 물고 스스로) ───────────────────────────────
pans = arg_list("--pans", [0.0])
cfg = load_settings()
cap = open_cam(cfg)
bot = Robot()
print(bot, f"카메라 {cfg['camera']}, 받침 {pans}°, 앞뒤 {XS} m")
bot.torque(True)
if "--yes" not in sys.argv:
    input("빨간 공을 집게 사이에 넣고 Enter → 집게가 닫힌다 ")
bot.close_gripper()
time.sleep(1.0)

HOVER = arg_list("--hover", [HOVER])[0]
z = TABLE_Z + RADIUS + HOVER                               # 공 중심 높이 = 맞추는 평면
pixels, pts = [], []
seed = urdf_deg(bot)
for pan in pans:
    for x0 in XS:
        r = np.radians(pan)
        target = (x0 * np.cos(r), x0 * np.sin(r), z)        # 받침에서 x0 만큼, 받침 각도 pan 쪽
        q = None
        for s0 in (seed, [pan, 0, 30, 60, seed[4]], [pan, -30, 60, 75, seed[4]]):   # 시작 자세를 바꿔 가며 (접힌 자세에서는 못 찾기도 한다)
            q = ik_down(target, s0)
            if q is not None:
                break
        if q is None:
            print(f"  받침 {pan:+.0f}°, {x0:.2f} m: 닿지 않음 → 건너뜀")
            continue
        move_urdf(bot, q, SPEED)
        seed = q
        time.sleep(0.5)                                     # 흔들림이 멎게
        p = ball_pixel(cap)
        xyz = tip_xyz(bot)
        if p is None:
            print(f"  받침 {pan:+.0f}°, {x0:.2f} m: 공이 안 보임 → 건너뜀")
            continue
        pixels.append(p)
        pts.append(xyz)
        print(f"  받침 {pan:+.0f}°  픽셀 ({p[0]:6.1f}, {p[1]:6.1f})  ↔  공 중심 ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m")
cap.release()
bot.close()
finish(pixels, pts)

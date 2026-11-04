"""이론 8 — 카메라 모델: 3D 책상이 2D 화면으로 (핀홀) · 화면을 다시 책상으로 (호모그래피).

    python theory/t8_camera_model.py      # 계산만 (카메라 없이)

핀홀 카메라: 점 P(로봇 좌표)를 카메라 좌표로 옮긴 뒤 (X, Y, Z) → 픽셀 u = f·X/Z + cx, v = f·Y/Z + cy.
멀리 있을수록(Z 가 클수록) 작게 찍힌다. 책상(평면) 위 점만 생각하면 이 변환은 3×3 행렬 H 하나로 거꾸로 풀린다.
노트북 위 카메라를 흉내낸다: 팔 앞 60 cm, 높이 35 cm 에서 팔 받침 쪽을 내려다본다. f = 560 px (화각 약 60°, 640×480).

할 일
  1. 같은 1 cm 가 가까운 곳 · 먼 곳에서 몇 픽셀인지 본다 (픽셀 1 개 = 몇 mm?)
  2. 점 4개로 맞춘 H 가 책상 위 다른 점에서도 맞는지, 렌즈 휨이 있으면 얼마나 빗나가는지 본다
  3. 책상보다 높은 점(블록 윗면)을 책상 H 로 바꾸면 얼마나 빗나가는지 본다 → 물체 높이를 왜 알아야 하나
"""
import numpy as np

F, CX, CY = 560.0, 320.0, 240.0
CAM = np.array([0.80, 0.0, 0.35])          # 카메라 자리 (로봇 좌표 m): 팔 앞 0.6 m 쯤 노트북 화면 위
LOOK = np.array([0.20, 0.0, 0.0])          # 바라보는 곳: 팔 앞 책상


def camera_axes():
    z = LOOK - CAM
    z /= np.linalg.norm(z)                  # 카메라 앞
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)                  # 화면 오른쪽
    y = np.cross(z, x)                      # 화면 아래
    return np.array([x, y, z])


R = camera_axes()


def project(p, k1=0.0):
    """로봇 좌표 점 → 픽셀. k1 은 렌즈 휨 (음수 = 가장자리가 안으로 휨, 웹캠에 흔함)"""
    X, Y, Z = R @ (np.asarray(p, float) - CAM)
    x, y = X / Z, Y / Z
    r2 = x * x + y * y
    d = 1 + k1 * r2
    return np.array([F * x * d + CX, F * y * d + CY])


def fit_h(px, xy):
    """점 4개 이상 → H (직접 선형 변환, numpy 만)"""
    rows = []
    for (u, v), (x, y) in zip(px, xy):
        rows.append([u, v, 1, 0, 0, 0, -x * u, -x * v, -x])
        rows.append([0, 0, 0, u, v, 1, -y * u, -y * v, -y])
    _, _, vt = np.linalg.svd(np.array(rows))
    return vt[-1].reshape(3, 3) / vt[-1][-1]


def to_xy(H, uv):
    p = H @ np.array([uv[0], uv[1], 1.0])
    return p[:2] / p[2]


print("1. 같은 1 cm 가 화면에서 몇 픽셀인가 (책상 위, 앞뒤로 1 cm)")
for x in [0.12, 0.20, 0.30]:
    du = np.linalg.norm(project([x + 0.01, 0, 0]) - project([x, 0, 0]))
    dv = np.linalg.norm(project([x, 0.01, 0]) - project([x, 0, 0]))
    print(f"   x = {x * 100:4.0f} cm:  앞뒤 1 cm = {du:4.1f} px,  좌우 1 cm = {dv:4.1f} px  → 픽셀 하나 ≈ {10 / min(du, dv):.1f} mm")

corners = [[0.12, 0.15, 0], [0.12, -0.15, 0], [0.32, -0.15, 0], [0.32, 0.15, 0]]
tests = [[x, y, 0] for x in np.linspace(0.12, 0.32, 5) for y in np.linspace(-0.15, 0.15, 5)]
print("\n2. 네 모서리로 H 를 맞추고 책상 위 25 점에서 시험")
for k1 in [0.0, -0.05, -0.15]:
    H = fit_h([project(c, k1) for c in corners], [c[:2] for c in corners])
    err = [np.linalg.norm(to_xy(H, project(p, k1)) - p[:2]) * 1000 for p in tests]
    print(f"   렌즈 휨 k1 = {k1:5.2f}:  평균 {np.mean(err):5.1f} mm, 최대 {np.max(err):5.1f} mm")

print("\n3. 책상보다 높은 점을 책상 H 로 바꾸면 (휨 없음)")
H = fit_h([project(c) for c in corners], [c[:2] for c in corners])
for h in [0.01, 0.03, 0.05]:
    p = [0.22, 0.05, h]
    xy = to_xy(H, project(p))
    print(f"   높이 {h * 100:3.0f} cm 점 (0.220, 0.050) → ({xy[0]:.3f}, {xy[1]:.3f})  빗나감 {np.linalg.norm(xy - p[:2]) * 1000:5.1f} mm")

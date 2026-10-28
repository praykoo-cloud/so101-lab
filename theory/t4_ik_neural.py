"""이론 4 — 신경망으로 역기구학: "목표 위치 → 관절 각도" 를 외우게 한다.

    python theory/t4_ik_neural.py          # numpy 만 쓴다. 노트북 CPU 에서 1분 안쪽

순서
  1. 정답 만들기: 목표점 3000개를 DLS(t3) 로 풀어 (목표, 각도) 쌍을 만든다. 모두 같은 시작 자세에서 푼다
  2. 학습: 3 → 128 → 128 → 4 신경망 (ReLU). 손실은 각도 오차(제곱 평균)
  3. 평가: 신경망 답을 정기구학에 넣어 손끝이 목표에서 몇 mm 빗나가는지 잰다
  4. 학습 범위 밖 (pan ±45° 바깥) 에서 오차가 어떻게 커지는지 본다
  5. 신경망 답에서 DLS 를 시작하면 반복이 몇 번으로 줄어드는지 본다

확인할 것: 신경망은 빠르지만 mm 단위로는 빗나간다 → 실제로는 "신경망으로 첫 답 + 솔버로 마무리"
"""
import time

import numpy as np

from soarm.kinematics import fk, ik_numeric

rng = np.random.default_rng(0)
START = np.zeros(5)
LO, HI = np.array([-45, -90, -60, -60]), np.array([45, 60, 90, 95])   # 정답을 만들 관절 범위 (pan ±45°)


def make_pairs(n, lo=LO, hi=HI):
    X, Y = [], []
    while len(X) < n:
        q = np.r_[rng.uniform(lo, hi), 0]
        t = fk(q)
        if t[2] < 0.02:                         # 바닥(2 cm) 아래는 버린다
            continue
        r = ik_numeric(t, q0=START, tol=1e-4)   # 같은 시작 자세에서 풀어 한쪽 답으로 모은다
        if r is not None:
            X.append(t)
            Y.append(r[0][:4])
    return np.array(X), np.array(Y)


t0 = time.time()
X, Y = make_pairs(3000)
Xt, Yt = make_pairs(500)
print(f"1. 정답 쌍 3000 + 시험 500 만들기: {time.time() - t0:.1f} s")

# 표준화 (평균 0, 표준편차 1)
xm, xs, ym, ys = X.mean(0), X.std(0), Y.mean(0), Y.std(0)
sizes = [3, 128, 128, 4]
W = [rng.normal(0, np.sqrt(2 / a), (a, b)) for a, b in zip(sizes, sizes[1:])]   # He 초기화
B = [np.zeros(b) for b in sizes[1:]]
print(f"   가중치 {sum(w.size for w in W) + sum(b.size for b in B):,}개")


def forward(x):
    h = [(x - xm) / xs]
    for i, (w, b) in enumerate(zip(W, B)):
        z = h[-1] @ w + b
        h.append(np.maximum(z, 0) if i < len(W) - 1 else z)
    return h


def predict(x):
    return forward(np.atleast_2d(x))[-1] * ys + ym


# Adam, 배치 128
m = [np.zeros_like(p) for p in W + B]
v = [np.zeros_like(p) for p in W + B]
lr, step = 1e-3, 0
yn = (Y - ym) / ys
t0 = time.time()
for epoch in range(300):
    idx = rng.permutation(len(X))
    for s in range(0, len(X), 128):
        bi = idx[s:s + 128]
        h = forward(X[bi])
        g = 2 * (h[-1] - yn[bi]) / len(bi)              # 손실 = 각도(표준화) 제곱 평균
        gW, gB = [], []
        for i in reversed(range(len(W))):
            gW.insert(0, h[i].T @ g)
            gB.insert(0, g.sum(0))
            if i:
                g = (g @ W[i].T) * (h[i] > 0)
        step += 1
        for k, (p, gp) in enumerate(zip(W + B, gW + gB)):
            m[k] = 0.9 * m[k] + 0.1 * gp
            v[k] = 0.999 * v[k] + 0.001 * gp * gp
            p -= lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(v[k] / (1 - 0.999 ** step)) + 1e-8)
    if epoch in (0, 9, 99, 299):
        loss = np.mean((forward(X)[-1] - yn) ** 2)
        print(f"2. 학습 {epoch + 1:3d}회: 손실 {loss:.4f}  ({time.time() - t0:.0f} s)")


def tip_err_mm(targets, angles):
    return np.array([np.linalg.norm(fk(np.r_[a, 0]) - t) * 1000 for t, a in zip(targets, angles)])


e = tip_err_mm(Xt, predict(Xt))
print(f"3. 시험 500점 손끝 오차: 중간값 {np.median(e):.1f} mm, 평균 {e.mean():.1f} mm, 90% {np.percentile(e, 90):.1f} mm")
ang = np.abs(predict(Xt) - Yt).mean()
print(f"   각도 오차 평균 {ang:.2f}°  ← 각도는 작게 틀려도 손끝에서는 mm 로 커진다")

print("4. 학습 범위 밖 (pan 을 45° 넘겨 돌린 목표)")
for pan in (40, 50, 60, 75, 90):
    Xo, _ = make_pairs(100, lo=[pan, -90, -60, -60], hi=[pan, 60, 90, 95])
    r = np.hypot(Xo[:, 0] - 0.03884, Xo[:, 1])                            # 회전축에서 거리
    outside = np.radians(max(0, pan - 45)) * r.mean() * 1000              # 범위 밖으로 나간 호 길이
    print(f"   pan {pan:2d}° (범위 밖 {outside:5.0f} mm): 중간값 {np.median(tip_err_mm(Xo, predict(Xo))):6.1f} mm")

print("5. DLS 시작 자세: 0° 자세 vs 신경망 답 (오차 0.1 mm 까지)")
k0, k1, t_nn, t_dls = [], [], 0.0, 0.0
for t in Xt[:200]:
    s = time.perf_counter()
    guess = np.r_[predict(t)[0], 0]
    t_nn += time.perf_counter() - s
    s = time.perf_counter()
    k0.append(ik_numeric(t, q0=START, tol=1e-4, strict=False)[1])
    t_dls += time.perf_counter() - s
    k1.append(ik_numeric(t, q0=guess, tol=1e-4, strict=False)[1])
print(f"   반복 평균: 0° 에서 {np.mean(k0):.1f}번 → 신경망 답에서 {np.mean(k1):.1f}번")
print(f"   한 점 계산 시간: 신경망 {t_nn / 200 * 1e3:.2f} ms, DLS {t_dls / 200 * 1e3:.2f} ms")

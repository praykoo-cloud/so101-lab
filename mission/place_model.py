"""허브 4번 '바구니 넣기': 정차 사진의 상자 위치 → 5 바구니 위 모터 값 을 배우는 작은 모델 (2차식).

왜 이것을 쓰나
    차가 매번 조금씩 다른 자리에 선다. 학습한 5 자세의 '평균' 하나로 가면 차가 옆으로 서 있을 때 바구니를 벗어난다.
    정차 사진에서 YOLO 가 찾은 상자 중심 (x, y) 을 보면 차가 어디 섰는지 알 수 있으므로, 그 자리에 맞는 5 자세를 계산한다.

어떻게 배우나 (학습 = 숫자 몇 개 구하기, 1초도 안 걸린다)
    관절(1 받침 · 2 어깨 · 3 팔꿈치 · 4 손목 꺾기)마다
        각도 = a + b·x + c·y + d·x² + e·y² + f·x·y        (x, y = 상자 중심을 화면 크기로 나눠 -1 ~ 1 로 맞춘 값)
    의 a ~ f 를 ✔ 사용 회차들에 가장 잘 맞게 구한다 (최소제곱 + 아주 작은 릿지).
    5 손목 돌리기는 회차마다 거의 같아서 평균을 쓴다.
    .pt 같은 모델 파일은 만들지 않는다. 실행(테스트 · 미션)을 시작할 때 저장된 회차로 매번 새로 구한다
    → 회차를 더 모으면 다음 시작부터 바로 반영된다.

2026-11-16 회차 32개 시험 (한 회차씩 빼고 나머지로 맞혀 보기): 평균 자세 12.2° → 2차식 6.8° (1 받침은 1.9°)

안전
    - 학습한 상자 위치 범위(+ 여유 margin_px) 밖이면 예측하지 않는다 (2차식은 범위 밖에서 엉뚱해질 수 있다) → 평균 자세로
    - 예측한 각도는 학습 회차들의 각도 범위(+ 여유 margin_deg) 안으로 자른다
    - 회차가 min_takes 보다 적거나, 상자 위치가 가로 · 세로로 MIN_SPREAD_PX 만큼 퍼져 있지 않으면 모델을 만들지 않는다 → 평균 자세로

단어장
- 2차식(quadratic) : x, y 와 그 제곱 · 곱까지 쓰는 식. 직선식보다 휘어진 관계를 맞출 수 있다
- 릿지(ridge)      : 회차가 적을 때 숫자가 튀지 않게 아주 조금 0 쪽으로 당기는 것
"""
from __future__ import annotations

FRAME_W = 640.0
FRAME_H = 480.0
MODEL_JOINTS = ("1", "2", "3", "4")      # 예측하는 관절. 5 손목 돌리기는 평균
RIDGE = 1e-3
MIN_TAKES = 10
MIN_SPREAD_PX = 60.0                     # 상자 중심이 가로 · 세로로 이만큼은 퍼져 있어야 모델을 만든다


def _terms(cx: float, cy: float) -> list[float]:
    x = (cx - FRAME_W / 2) / (FRAME_W / 2)
    y = (cy - FRAME_H / 2) / (FRAME_H / 2)
    return [1.0, x, y, x * x, y * y, x * y]


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """가우스 소거로 a·w = b 를 푼다 (6×6 정도라 numpy 없이 충분)"""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("회차들의 상자 위치가 너무 한곳에 몰려 있어 식을 풀 수 없어요")
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r != col:
                f = m[r][col] / m[col][col]
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    return [m[i][n] / m[i][i] for i in range(n)]


def samples_from_takes(takes) -> list[tuple[dict, dict]]:
    """✔ 사용 회차 중 3 정차 사진(상자)과 5 바구니 위가 둘 다 있는 것 → [(상자, 5 각도)]"""
    out = []
    for t in takes:
        steps = t.get("steps") or {}
        box = (steps.get("stop") or {}).get("basket")
        above = steps.get("above") or {}
        if box and above.get("angles"):
            out.append((box, above["angles"]))
    return out


def fit(samples, min_takes: int = MIN_TAKES) -> dict | None:
    """samples = [(상자{cx,cy}, 각도{"1".."5"})]. 회차가 모자라거나 풀 수 없으면 None"""
    if len(samples) < max(min_takes, 7):
        return None
    for key in ("cx", "cy"):                       # 차가 한곳에만 섰던 회차들로는 위치 → 자세 관계를 알 수 없다
        vals = [float(b[key]) for b, _ in samples]
        if max(vals) - min(vals) < MIN_SPREAD_PX:
            return None
    rows = [_terms(b["cx"], b["cy"]) for b, _ in samples]
    k = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) + (RIDGE if i == j else 0.0) for j in range(k)] for i in range(k)]
    coef = {}
    try:
        for j in MODEL_JOINTS:
            ys = [float(a[j]) for _, a in samples]
            coef[j] = [round(v, 5) for v in _solve(ata, [sum(r[i] * y for r, y in zip(rows, ys)) for i in range(k)])]
    except ValueError:
        return None
    all_j = sorted({j for _, a in samples for j in a})
    return {
        "kind": "quad2",
        "n": len(samples),
        "coef": coef,
        "fixed": {j: round(sum(float(a[j]) for _, a in samples) / len(samples), 2)
                  for j in all_j if j not in MODEL_JOINTS},
        "box": {"cx": [min(b["cx"] for b, _ in samples), max(b["cx"] for b, _ in samples)],
                "cy": [min(b["cy"] for b, _ in samples), max(b["cy"] for b, _ in samples)]},
        "angle": {j: [min(float(a[j]) for _, a in samples), max(float(a[j]) for _, a in samples)] for j in MODEL_JOINTS},
    }


def predict(model: dict | None, box: dict | None, margin_px: float = 40.0,
            margin_deg: float = 5.0) -> tuple[dict | None, str]:
    """(5 바구니 위 각도, 설명). 쓸 수 없으면 (None, 이유)"""
    if not model:
        return None, "예측 모델이 없어요 (정차 사진이 있는 ✔ 회차가 모자람)"
    if not box:
        return None, "이번 정차 사진(상자 위치)이 없어요"
    cx, cy = float(box["cx"]), float(box["cy"])
    (x0, x1), (y0, y1) = model["box"]["cx"], model["box"]["cy"]
    if not (x0 - margin_px <= cx <= x1 + margin_px and y0 - margin_px <= cy <= y1 + margin_px):
        return None, (f"상자 위치 x {cx:.0f} · y {cy:.0f} 가 학습한 범위(x {x0:.0f}~{x1:.0f}, y {y0:.0f}~{y1:.0f}) "
                      "밖이에요")
    t = _terms(cx, cy)
    out = dict(model.get("fixed") or {})
    for j, w in model["coef"].items():
        lo, hi = model["angle"][j]
        v = sum(a * b for a, b in zip(w, t))
        out[j] = round(min(hi + margin_deg, max(lo - margin_deg, v)), 2)
    return out, f"상자 x {cx:.0f} · y {cy:.0f} 로 계산한 5 자세"

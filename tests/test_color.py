"""soarm.color 시험 — 합성 이미지 (카메라 없이)"""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from soarm.color import ColorConfig, classify  # noqa: E402


def frame_with(bgr, roi=(270, 190, 100, 100), fill=1.0):
    img = np.full((480, 640, 3), (120, 120, 120), np.uint8)          # 회색 바닥
    x, y, w, h = roi
    img[y:y + int(h * fill), x:x + w] = bgr
    return img


@pytest.mark.parametrize("bgr,expect", [
    ((30, 30, 200), "red"),         # 진한 빨강 (H 0 근처)
    ((60, 20, 230), "red"),         # 자홍 쪽 빨강 (H 170 근처)
    ((200, 80, 20), "blue"),
    ((120, 120, 120), "none"),      # 빈 자리
    ((40, 200, 40), "none"),        # 초록은 분류하지 않는다
])
def test_classify_colors(bgr, expect):
    color, ratios = classify(frame_with(bgr), ColorConfig())
    assert color == expect, ratios


def test_small_patch_is_none():
    # ROI 의 20% 만 빨강 → min_ratio 0.3 미만
    color, ratios = classify(frame_with((30, 30, 200), fill=0.2), ColorConfig())
    assert color == "none" and 0.15 < ratios["red"] < 0.25


def test_config_roundtrip(tmp_path):
    cfg = ColorConfig(roi=[1, 2, 3, 4], min_ratio=0.2)
    p = cfg.save(tmp_path / "c.json")
    assert ColorConfig.load(p).roi == [1, 2, 3, 4]

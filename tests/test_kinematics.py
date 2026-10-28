"""soarm.kinematics: URDF 정기구학이 ArmStudio 값과 같고, 수치 역기구학이 같은 점으로 돌아오는지."""
import numpy as np
import pytest

np_testing = pytest.importorskip("numpy.testing")

from soarm.kinematics import fk, ik_numeric  # noqa: E402


def test_fk_zero_pose_matches_armstudio():
    # ArmStudio 의 robot.fk([0, 0, 0, 0, 0]) → x=391.4 y=-0.0 z=226.5 mm
    np_testing.assert_allclose(fk([0, 0, 0, 0, 0]) * 1000, [391.4, 0.0, 226.5], atol=0.1)


def test_ik_round_trip():
    q, iters, err = ik_numeric([0.25, 0.0, 0.10])
    assert err < 1e-4
    np_testing.assert_allclose(fk(q), [0.25, 0.0, 0.10], atol=1e-4)


def test_ik_unreachable():
    assert ik_numeric([0.60, 0.0, 0.06]) is None
    q, iters, err = ik_numeric([0.60, 0.0, 0.06], strict=False)
    assert err > 0.1

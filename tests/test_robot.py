"""soarm.Robot 시험 (가짜 버스).   python -m pytest tests"""
import time

import pytest

import soarm.driver_sdk as drv
import soarm.robot as rb
from soarm import Robot, RobotError
from soarm.fake import GOAL, TORQUE, FakeBus, FakeSerial

# 실물 팔로워 한 대의 범위 (LeRobot 캘리브레이션 파일에서)
LIMITS = {1: (839, 3243), 2: (888, 3270), 3: (847, 3064), 4: (810, 3207), 5: (0, 4095), 6: (2016, 3607)}
FOLDED = {1: 1942, 2: 906, 3: 3049, 4: 958, 5: 1983, 6: 2065}


@pytest.fixture
def bus(monkeypatch, tmp_path):
    for sid, (lo, hi) in LIMITS.items():
        monkeypatch.setitem(drv.JOINT_LIMITS, sid, {**drv.JOINT_LIMITS[sid], "min": lo, "max": hi})
    monkeypatch.setattr(drv.serial, "Serial", FakeSerial)
    monkeypatch.setattr(rb, "POSES_FILE", tmp_path / "poses.json")
    monkeypatch.setattr(rb, "RECORDINGS_DIR", tmp_path / "recordings")
    # 실물에서 본 것처럼 shoulder_pan 에 예전 목표(+26틱), wrist_roll 에 467틱 차이를 남겨 둔다
    FakeSerial.bus = FakeBus(present=FOLDED, goal={1: FOLDED[1] + 26, 5: FOLDED[5] + 467})
    return FakeSerial.bus


def present(bus, sid):
    bus.servos[sid].step()
    return bus.servos[sid].get16(56)


def test_connect_leaves_torque_off(bus):
    # 실물에서 찾은 버그: connect() 가 목표를 써서 토크가 켜졌다 → 실습 01 에서 손으로 못 움직임
    Robot("FAKE")
    assert all(bus.servos[s].mem[TORQUE] == 0 for s in FOLDED)


def test_torque_on_with_stale_goal_does_not_jump(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    assert all(bus.servos[s].get16(GOAL) == FOLDED[s] for s in FOLDED)
    time.sleep(0.2)
    assert {s: present(bus, s) for s in FOLDED} == FOLDED


def test_torque_on_after_moving_by_hand_does_not_jump(bus):
    bot = Robot("FAKE")
    bus.servos[1].push(FOLDED[1] + 300)          # 토크가 꺼진 채 손으로 30° 돌림
    bot.torque(True)
    time.sleep(0.2)
    assert present(bus, 1) == FOLDED[1] + 300


def test_move_needs_torque(bus):
    bot = Robot("FAKE")
    with pytest.raises(RobotError, match="토크"):
        bot.move("base", 10)


def test_move_all_only_touches_given_joints(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    before = {s: bus.servos[s].get16(GOAL) for s in FOLDED}
    bot.move_all({"base": 5})
    after = {s: bus.servos[s].get16(GOAL) for s in FOLDED}
    assert after[1] == rb.deg_to_pos(5)
    assert {s: after[s] for s in after if s != 1} == {s: before[s] for s in before if s != 1}


def test_move_is_clamped_to_calibrated_range(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    bot.move("shoulder", -170)
    assert bus.servos[2].get16(GOAL) == LIMITS[2][0]


def test_speed_limits_motion(bus):
    bot = Robot("FAKE", speed=200)
    bot.torque(True)
    bot.move("base", rb.pos_to_deg(FOLDED[1] + 100))
    time.sleep(0.25)
    moved = present(bus, 1) - FOLDED[1]
    assert 30 < moved < 80                        # 200 틱/초 × 0.25초 ≈ 50틱


def test_wait_until_settled_waits_for_a_slow_move(bus):
    # 실물에서 찾은 버그: 느린 속도에서 '두 번 읽은 값이 비슷' 으로 판단해 바로 끝났다
    bot = Robot("FAKE", speed=100)
    bot.torque(True)
    bot.move("base", rb.pos_to_deg(FOLDED[1] + 57))     # 5°
    assert bot.wait_until_settled(timeout=3)
    assert abs(present(bus, 1) - (FOLDED[1] + 57)) <= 17


def test_gripper_percent(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    bot.gripper(100)
    assert bus.servos[6].get16(GOAL) == LIMITS[6][1]
    bot.gripper(0)
    assert bus.servos[6].get16(GOAL) == LIMITS[6][0]
    assert bot.gripper_percent() == pytest.approx((FOLDED[6] - 2016) / (3607 - 2016) * 100, abs=0.2)


def test_save_and_go_pose_roundtrip_in_controller_format(bus):
    bot = Robot("FAKE", speed=0)
    bot.save_pose("folded")
    entry = rb._load_json(rb.POSES_FILE, {})["folded"]
    assert entry["format"] == "normalized" and set(entry["positions"]) == {"1", "2", "3", "4", "5", "6"}
    bot.torque(True)
    bot.move("base", 20)
    bot.go_pose("folded", timeout=2)
    assert abs(present(bus, 1) - FOLDED[1]) <= 3


def test_go_pose_unknown_name(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    with pytest.raises(RobotError, match="없습니다"):
        bot.go_pose("nope")


def test_record_and_play(bus):
    bot = Robot("FAKE", speed=0)
    frames = bot.record(0.35, interval_ms=50, name="take1")
    assert len(frames) >= 5
    loaded = bot.load_recording("take1")
    assert len(loaded) == len(frames)
    bot.torque(True)
    bot.play("take1")
    assert abs(present(bus, 2) - FOLDED[2]) <= 3


def test_torque_off_stays_off(bus):
    bot = Robot("FAKE")
    bot.torque(True)
    bot.stop()
    bot.torque(False)
    bot.stop()                                    # 토크가 꺼진 뒤 stop() 을 또 불러도
    bot.save_pose("p")
    bot.angles()
    assert all(bus.servos[s].mem[TORQUE] == 0 for s in FOLDED)

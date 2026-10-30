"""다른 Pi 의 팔과 잘 통하는지 — 서비스 왕복 시간 · /so101/joint_states 속도 · 두 기계의 시계 차.

    python3 car/link_check.py          # 차 Pi (두뇌 컨테이너) 에서, 팔 Pi 에서 arm.launch.py namespace:=so101 이 도는 동안

팔은 움직이지 않는다 (/arm/stop 만 부른다: 그 자리에 멈춤).
'찍힌 시각 → 받은 시각' 에는 전송 시간과 두 기계의 시계 차가 함께 들어 있다. 몇 ms 면 정상, 수백 ms 면 시계가 어긋난 것.
"""
import statistics
import time

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

rclpy.init()
node = rclpy.create_node("link_check")

stop = node.create_client(Trigger, "/arm/stop")
if not stop.wait_for_service(timeout_sec=5.0):
    raise SystemExit("/arm/stop 이 없다. 팔 Pi 에서 arm.launch.py 가 도는지, ROS_DOMAIN_ID 가 같은지")
rtt = []
for _ in range(20):
    t = time.time()
    fut = stop.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    rtt.append((time.time() - t) * 1000)
print(f"/arm/stop 첫 호출 {rtt[0]:.0f} ms, 나머지 19회 중앙값 {statistics.median(rtt[1:]):.1f} ms "
      f"(최소 {min(rtt[1:]):.1f}, 최대 {max(rtt[1:]):.1f})")

lag, got = [], []


def on_state(m):
    now = node.get_clock().now().nanoseconds
    lag.append((now - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e6)
    got.append(time.time())


node.create_subscription(JointState, "/so101/joint_states", on_state, 10)
end = time.time() + 5.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
if len(got) < 2:
    raise SystemExit("/so101/joint_states 가 안 온다 (namespace:=so101 로 띄웠는지)")
print(f"/so101/joint_states {(len(got) - 1) / (got[-1] - got[0]):.1f} Hz, "
      f"찍힌 시각 → 받은 시각 중앙값 {statistics.median(lag):.1f} ms (전송 + 두 기계 시계 차)")
node.destroy_node()
rclpy.shutdown()

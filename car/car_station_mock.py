"""car_station_mock — 차 흉내. 차 없이 station_node 를 시험한다. 실제 차는 자율주행 노드가 같은 일을 한다.

    python3 car/car_station_mock.py            # 도착 → 서비스 부름 → 결과 → 출발 (한 번)
    python3 car/car_station_mock.py --loop 3   # 세 번

실제 차에 넣을 때 바꿀 곳은 '도착' 과 '출발' 두 줄이다:
    도착 = 정지선 · 표지판(YOLO) 에서 /controller/cmd_vel 0 을 보내고 선 다음
    출발 = 결과를 받은 뒤 다시 주행. 팔이 '응답할 때까지' 차는 움직이지 않는다 (팔이 바구니 안에 있을 수 있다)
"""
import sys
import time

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

TIMEOUT = 90.0                   # 팔 한 번에 넉넉히 (자세 이동 6 ~ 7번)

rclpy.init()
node = rclpy.create_node("car_station_mock")
node.create_subscription(String, "/arm_station/state", lambda m: print(f"   팔 상태: {m.data}"), 10)
serve = node.create_client(Trigger, "/arm_station/serve")

rounds = int(sys.argv[sys.argv.index("--loop") + 1]) if "--loop" in sys.argv else 1
try:
    for k in range(1, rounds + 1):
        print(f"[{k}] 차: 달리는 중 ... 스테이션 도착, 정지")
        if not serve.wait_for_service(timeout_sec=5.0):
            print("   /arm_station/serve 가 없다 (팔 쪽 station_node · ROS_DOMAIN_ID 확인) → 기다리지 않고 출발")
            continue
        t = time.time()
        fut = serve.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=TIMEOUT)
        if not fut.done():
            print(f"   {TIMEOUT:.0f} 초 안에 응답 없음 → 사람을 부른다 (차는 그대로 정지)")
            break
        r = fut.result()
        print(f"   결과: {'성공' if r.success else '실패'} · {r.message} ({time.time() - t:.1f} 초)")
        print("   차: 출발" if r.success else "   차: 출발하지 않음 (사람 확인)")
        if not r.success:
            break
        time.sleep(2.0)
finally:
    node.destroy_node()
    rclpy.try_shutdown()

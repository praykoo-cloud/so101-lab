"""가상 팔: 실물 없이 SDK · ROS2 노드를 돌리는 가짜 STS3215 버스 (serial.Serial 자리에 끼운다).

Robot("fake") 로 쓴다. 시험(tests/)과, 팔이 없을 때 쓴다.

PING · READ · WRITE 패킷에 실제 서보처럼 응답한다. 토크가 켜진 서보만 목표 쪽으로 움직이고,
속도 레지스터(틱/초)를 지킨다 (0 = 바로 도착). 시작할 때 목표를 현재와 다르게 둘 수 있어서
'예전 목표가 남아 있으면 토크를 켜는 순간 튄다' 를 재현할 수 있다.
"""
import time

TORQUE, ACCEL, GOAL, SPEED, PRESENT, LOAD, VOLT, TEMP = 40, 41, 42, 46, 56, 60, 62, 63

# 실물 팔로워의 접힌 자세 (서보 값). 가상 팔은 이 자세로 시작한다
FOLDED = {1: 1942, 2: 906, 3: 3049, 4: 958, 5: 1983, 6: 2065}


class FakeServo:
    def __init__(self, sid, present=2048, goal=None):
        self.id = sid
        self.mem = bytearray(70)
        self.mem[VOLT] = 122
        self.mem[TEMP] = 30
        self._pos = float(present)
        self.set16(PRESENT, present)
        self.set16(GOAL, present if goal is None else goal)
        self._t = time.monotonic()

    def get16(self, a):
        return self.mem[a] | (self.mem[a + 1] << 8)

    def set16(self, a, v):
        self.mem[a] = v & 0xFF
        self.mem[a + 1] = (v >> 8) & 0xFF

    def step(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        if self.mem[TORQUE]:
            goal, speed = self.get16(GOAL), self.get16(SPEED)
            d = goal - self._pos
            if speed == 0 or abs(d) <= speed * dt:
                self._pos = float(goal)
            else:
                self._pos += speed * dt * (1 if d > 0 else -1)
        self.set16(PRESENT, int(round(self._pos)))

    def push(self, pos):
        """손으로 민 것처럼 위치만 바꾼다 (토크가 꺼져 있을 때)"""
        self._pos = float(pos)
        self.set16(PRESENT, int(pos))


class FakeBus:
    """여러 서보가 한 줄로 이어진 버스. FakeSerial 이 이 객체를 쓴다"""

    def __init__(self, present=None, goal=None):
        present = present or {}
        goal = goal or {}
        self.servos = {sid: FakeServo(sid, present.get(sid, 2048), goal.get(sid)) for sid in range(1, 7)}
        self.log = []                       # (inst, id, addr, data)


class FakeSerial:
    bus = None                              # 시험마다 FakeSerial.bus = FakeBus(...) 로 바꾼다

    def __init__(self, port=None, baudrate=None, timeout=None, **_):
        self.port = port
        self.is_open = True
        self._rx = b""

    def reset_input_buffer(self):
        self._rx = b""

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def write(self, pkt):
        pkt = bytes(pkt)
        sid, inst, params = pkt[2], pkt[4], list(pkt[5:-1])
        s = self.bus.servos.get(sid)
        if s is None:
            return len(pkt)
        s.step()
        if inst == 0x01:                    # PING
            self._reply(sid, [])
        elif inst == 0x02:                  # READ addr, n
            a, n = params
            self._reply(sid, list(s.mem[a:a + n]))
        elif inst == 0x03:                  # WRITE addr, data...
            a, data = params[0], params[1:]
            s.mem[a:a + len(data)] = bytes(data)
            if a == PRESENT:
                s._pos = float(s.get16(PRESENT))
            if a == GOAL and not s.mem[TORQUE]:
                s.mem[TORQUE] = 1           # 실측: 토크가 꺼진 서보에 목표를 쓰면 토크가 켜진다
            self.bus.log.append(("write", sid, a, data))
            self._reply(sid, [])
        return len(pkt)

    def _reply(self, sid, data):
        body = [sid, len(data) + 2, 0, *data]
        self._rx += bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])

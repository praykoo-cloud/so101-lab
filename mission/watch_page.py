"""허브 1번 '대기 자세 저장' 화면: 차가 오는 쪽을 바라보는 **고정 자세(대기 자세)** 를 절대 위치로 저장하고 확인한다.

카메라는 필요 없고 팔만 연결돼 있으면 된다. 저장하는 곳은 data/poses.json 의 'watch' (설정 load_wait_pose).
상차 미션(mission.loading)은 전원을 켜면 이 자세로 먼저 가고, 차가 떠난 뒤에도 여기로 돌아온다. 4번 바구니 넣기 학습도 여기서 시작한다.

쓰는 순서 (화면에도 적혀 있다)
  1) '힘 빼기' (팔을 손으로 받치고!) → 팔을 손으로 옮겨 차가 오는 길이 보이게 한다 (카메라에 어떻게 보이는지는 2번 화면에서 확인)
  2) '지금 자세를 대기 자세로 저장'  (예전 저장값은 runtime/backup 에 복사해 둔다)
  3) '대기 자세로 가기' 로 다시 가 보며 확인

단어장
- 절대 위치 : 다른 값과 비교하지 않고, 모터가 실제로 있는 위치 그대로를 저장한 것 (같은 곳으로 매번 똑같이 돌아온다)
"""
from __future__ import annotations

import shutil
import threading
from datetime import datetime
from pathlib import Path

from .arm_io import ArmIO
from .config import RUNTIME_DIR, MissionConfig
from .motions import MotionError


class WatchDesk:
    def __init__(self, robot, cfg: MissionConfig, backup_dir: Path | str | None = None):
        self.robot, self.cfg = robot, cfg
        self.arm = ArmIO(robot, cfg)
        self.backup_dir = Path(backup_dir) if backup_dir else RUNTIME_DIR / "backup"
        self.torque_on: bool | None = None
        self.message = "준비됐어요."
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def pose_name(self) -> str:
        return self.cfg.load_wait_pose

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _need_idle(self):
        if self.busy():
            raise MotionError("팔이 지금 대기 자세로 가는 중이에요. 끝난 뒤에 다시 누르세요.")

    def state(self) -> dict:
        saved = self.robot.poses().get(self.pose_name)
        if self.busy():                                          # 이동 중에는 통신을 건드리지 않는다 (화면이 멈추지 않게)
            now = {}
        else:
            with self._lock:
                now = self.robot.angles()
        return {"name": self.pose_name, "busy": self.busy(), "torque": self.torque_on, "message": self.message,
                "saved": None if saved is None else {"time": saved.get("timestamp"),
                                                     "angles": {k: None if v is None else round(v, 1) for k, v in saved["angles"].items()}},
                "now": {k: None if v is None else round(v, 1) for k, v in now.items()}, "fake": not self.robot.real}

    # ── 버튼 ────────────────────────────────────────────────────────────
    def set_torque(self, on: bool) -> str:
        self._need_idle()
        with self._lock:
            self.robot.torque(bool(on))
        self.torque_on = bool(on)
        return "힘을 줬어요. 팔이 지금 자리에서 버팁니다." if on else "힘을 뺐어요. 팔을 손으로 받치고 차가 오는 쪽이 보이게 옮기세요."

    def save(self) -> str:
        self._need_idle()
        with self._lock:
            positions = self.robot.positions()
            if any(positions.get(j) is None for j in (1, 2, 3, 4, 5, 6)):
                raise MotionError("관절 위치를 다 읽지 못했어요. 전원과 케이블을 확인하세요.")
            from soarm.robot import POSES_FILE
            backup = None
            if Path(POSES_FILE).exists():                       # 덮어쓰기 전에 원본을 복사해 둔다
                self.backup_dir.mkdir(parents=True, exist_ok=True)
                backup = self.backup_dir / f"poses-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.json"
                shutil.copy2(POSES_FILE, backup)
            self.robot.save_pose(self.pose_name, positions=positions)
        self.message = f"'{self.pose_name}' 자세를 저장했어요."
        return self.message + (f" 예전 파일: {backup}" if backup else "")

    def go(self) -> str:
        self._need_idle()
        if self.pose_name not in self.robot.poses():
            raise MotionError(f"'{self.pose_name}' 자세가 아직 저장되지 않았어요. 먼저 저장하세요.")
        if self.arm.too_hot():
            raise MotionError("모터가 뜨거워요. 식힌 뒤에 다시 하세요.")
        with self._lock:
            self.robot.torque(True)                              # 지금 자리에서 잠근다 (튀지 않음)
        self.torque_on = True
        self.message = "대기 자세로 가는 중…"
        self._thread = threading.Thread(target=self._go, daemon=True)
        self._thread.start()
        return "대기 자세로 천천히 갑니다."

    def _go(self):
        try:
            with self._lock:
                ok = self.arm.go(self.pose_name)
            self.message = "대기 자세에 도착했어요." if ok else "대기 자세에 다 가지 못했어요. 걸린 곳이 없는지 보세요."
        except Exception as error:
            try:
                self.robot.stop()
            except Exception:
                pass
            self.message = f"이동 중 문제가 생겨 멈췄어요: {error}"

    def join(self, timeout: float = 20.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def handle(self, action: str, data: dict) -> str:
        if action == "torque":
            return self.set_torque(data.get("on") is True)
        if action == "save":
            return self.save()
        if action == "go":
            return self.go()
        raise MotionError("요청이 올바르지 않아요.")


BODY = """
<h1><span class="num g-prep">1</span>대기 자세 저장</h1>
<p class=sub>차가 오는 쪽을 바라보는 <b>고정 자세</b>를 절대 위치로 저장합니다. 전원을 켜면 팔이 항상 이 자세로 가고, 차 추적도 여기서 시작해요. 카메라는 필요 없습니다.</p>
<p class='msg warn' id=fake hidden>지금은 <b>화면 연습용 팔</b>이에요. 실제 팔은 움직이지 않습니다.</p>
<p class=msg id=out role=status aria-live=polite>준비됐어요.</p>

<h2>① 팔을 원하는 자세로</h2>
<div class=msg>
  <p><button id=t-off class=sec>힘 빼기 (손으로 받치고!)</button> <button id=t-on class=sec>힘 주기</button> <span id=torque class=small></span></p>
  <p class=small>힘을 빼고 팔을 손으로 옮겨, 그리퍼 카메라가 차가 오는 길을 바라보게 하세요. 카메라에 어떻게 보이는지는 <a href='/p/yolo' target=_blank>2번 카메라 · YOLO 확인</a>에서 볼 수 있어요.</p>
</div>

<h2>② 저장</h2>
<div class=msg>
  <p><button id=save>지금 자세를 대기 자세로 저장</button></p>
  <p class=small>예전에 저장한 값이 있으면 덮어씁니다. 덮어쓰기 전에 원본은 runtime/backup 폴더에 복사해 둬요.</p>
</div>

<h2>③ 확인</h2>
<div class=msg>
  <p><button id=go class=sec>대기 자세로 가기</button> <span class=small>팔이 움직여요. 주변을 비우세요.</span></p>
  <table><thead><tr><th>관절</th><th>저장된 값(도)</th><th>지금 값(도)</th></tr></thead><tbody id=rows></tbody></table>
  <p class=small id=saved></p>
</div>
"""

SCRIPT = """
<script>
const TOKEN='__TOKEN__';const $=id=>document.getElementById(id);let S=null;
const NAMES={1:'1 받침',2:'2 어깨',3:'3 팔꿈치',4:'4 손목 꺾기',5:'5 손목 돌리기',6:'6 그리퍼'};
async function post(action,data){
  const r=await fetch('/api/watch/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Hub-Token':TOKEN},body:JSON.stringify(data||{})});
  const d=await r.json().catch(()=>({message:'응답을 읽지 못했어요.'}));
  const o=$('out');o.textContent=d.message;o.className='msg '+(r.ok?'ok':'bad');await refresh();return r.ok}
function render(){
  $('fake').hidden=!S.fake;
  $('torque').textContent=S.torque===null?'':(S.torque?'지금: 힘을 주고 있음':'지금: 힘을 뺌 (손으로 받치세요)');
  for(const id of ['t-off','t-on','save','go'])$(id).disabled=S.busy;
  $('rows').innerHTML=[1,2,3,4,5,6].map(j=>{const a=S.saved?S.saved.angles[j]:null,b=S.now[j];
    return '<tr><td>'+NAMES[j]+'</td><td>'+(a==null?'-':a)+'</td><td>'+(b==null?'-':b)+'</td></tr>'}).join('');
  $('saved').textContent=S.saved?("'"+S.name+"' 저장 시각: "+S.saved.time):("'"+S.name+"' 자세가 아직 저장되지 않았어요.");
  if(S.busy)$('out').textContent=S.message;
}
async function refresh(){try{S=await (await fetch('/api/watch/state',{cache:'no-store'})).json();render()}catch(e){}}
$('t-off').onclick=()=>{if(confirm('힘을 빼면 팔이 떨어질 수 있어요. 팔을 손으로 받치고 있나요?'))post('torque',{on:false})};
$('t-on').onclick=()=>post('torque',{on:true});
$('save').onclick=()=>{if(confirm('지금 자세를 대기 자세로 저장할까요? (예전 값은 덮어쓰고, 원본은 backup 폴더에 복사해 둬요)'))post('save')};
$('go').onclick=()=>{if(confirm('팔이 대기 자세로 움직입니다. 주변을 비웠나요?'))post('go')};
refresh();setInterval(refresh,1000);
</script>
"""


def watch_body() -> str:
    return BODY


def watch_script(token: str) -> str:
    return SCRIPT.replace("__TOKEN__", token)

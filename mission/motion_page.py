"""허브 3번 '동작 티칭' 화면: 팔을 손으로 움직여 동작을 가르치고, 저장하고, 클릭해서 다시 재생해 본다.

실행:
    .venv\\Scripts\\python.exe -m mission.hub --start motion            (Windows, 실물 팔)
    .venv/bin/python -m mission.hub --start motion                      (Pi)
    .venv\\Scripts\\python.exe -m mission.hub --start motion --fake-arm  (화면 연습: 가짜 팔, 실제로는 아무것도 안 움직임)

쓰는 순서 (화면에도 적혀 있다)
  1) 동작을 고른다 (빨간공_집기 / 파란공_집기 / 바구니_넣기 또는 새 이름)
  2) '힘 빼기' → 팔을 손으로 옮긴다 → '지점 추가' 를 반복 (팔을 손으로 받치고 있을 것!)
  3) '새 회차로 저장' → 같은 동작을 여러 번 가르치면 1차 · 2차 … 로 쌓인다
  4) 회차의 '▶ 재생' 으로 다시 움직여 보고 '잘 됨 / 안 됨' 표시 → 성적이 좋은 회차를 '★ 이걸로 사용'

안전
- 재생은 느린 속도(설정의 '느린 속도')로 한다. '정지' 는 지금 가는 지점까지만 가고 멈춘다. 급하면 팔 전원 스위치를 끈다.
- 재생이 끝나면 팔은 그 자리에서 힘을 준 채 서 있다 (공중에서 힘을 빼면 떨어지므로). 힘을 뺄 때는 손으로 받친다.
- 모터 온도가 중단 기준을 넘으면 스스로 멈춘다.
"""
from __future__ import annotations

import threading
import time

from .arm_io import ArmIO
from .config import MissionConfig
from .motions import MAX_POINTS, MotionBook, MotionError, capture_point, play


class MotionDesk:
    """화면 뒤에서 일하는 책상: 가르치는 중인 지점(초안) · 재생 스레드 · 팔 사용 순서를 관리한다"""

    def __init__(self, robot, cfg: MissionConfig, book: MotionBook | None = None):
        self.robot = robot
        self.cfg = cfg
        self.arm = ArmIO(robot, cfg)
        self.book = book or MotionBook()
        self.draft: list[dict] = []
        self.torque_on: bool | None = None            # None = 아직 모름 (화면에서 버튼을 누르기 전)
        self.status = {"state": "idle", "message": "아직 재생하지 않았어요.", "name": None, "take": None,
                       "step": 0, "total": 0, "report": None}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._arm_lock = threading.Lock()             # 팔 통신은 한 번에 하나씩 (재생 스레드와 버튼이 섞이지 않게)

    # ── 상태 ────────────────────────────────────────────────────────────
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _need_idle(self):
        if self.busy():
            raise MotionError("지금 재생 중이에요. 끝나거나 '정지' 한 뒤에 다시 누르세요.")

    def state(self) -> dict:
        ref = self.book.data.get("car_ref")
        return {"motions": self.book.summary(), "draft": self.draft, "torque": self.torque_on, "busy": self.busy(),
                "status": dict(self.status), "car_ref": ref, "max_points": MAX_POINTS, "fake": not self.robot.real}

    # ── 가르치기 ────────────────────────────────────────────────────────
    def set_torque(self, on: bool) -> str:
        self._need_idle()
        with self._arm_lock:
            self.robot.torque(bool(on))
        self.torque_on = bool(on)
        return "힘을 줬어요. 팔이 지금 자리에서 버팁니다." if on else "힘을 뺐어요. 팔을 손으로 받치고 원하는 자세로 옮기세요."

    def add_point(self) -> str:
        self._need_idle()
        if len(self.draft) >= MAX_POINTS:
            raise MotionError(f"지점은 {MAX_POINTS}개까지예요.")
        with self._arm_lock:
            self.draft.append(capture_point(self.robot))
        return f"{len(self.draft)}번 지점을 담았어요."

    def undo_point(self) -> str:
        if not self.draft:
            raise MotionError("담아 둔 지점이 없어요.")
        self.draft.pop()
        return f"마지막 지점을 뺐어요. 남은 지점 {len(self.draft)}개."

    def clear_draft(self) -> str:
        self.draft = []
        return "담아 둔 지점을 모두 비웠어요."

    def save_take(self, name: str, label: str = "") -> str:
        entry = self.book.motion(name)
        ref = self.book.car_ref_pan() if entry.get("relative") else None
        tid = self.book.add_take(name, self.draft, label, ref_pan=ref)
        count = len(self.draft)
        self.draft = []
        note = ""
        if entry.get("relative") and ref is None:
            note = " (주의: '기준 차 위치'를 아직 저장하지 않았어요. 차 위치 보정 없이 그대로 재생됩니다)"
        return f"'{name}' 의 {self.book.take(name, tid)['label']} 로 저장했어요 (지점 {count}개).{note}"

    def set_car_ref(self) -> str:
        self._need_idle()
        with self._arm_lock:
            pan = self.arm.pan_angle()
        if pan is None:
            raise MotionError("받침(1번 관절) 각도를 읽지 못했어요.")
        self.book.set_car_ref(pan)
        return f"지금 받침 각도({pan:.1f}도)를 '기준 차 위치'로 저장했어요. 이제 '바구니_넣기' 를 이 차 위치에서 가르치세요."

    # ── 재생 ────────────────────────────────────────────────────────────
    def start_play(self, name: str, tid: str | None = None, points: list[dict] | None = None) -> str:
        """저장된 회차(tid) 또는 지금 담아 둔 초안(points)을 재생"""
        self._need_idle()
        if points is None:
            points = self.book.take(name, tid)["points"]
        if not points:
            raise MotionError("재생할 지점이 없어요.")
        with self._arm_lock:
            if self.arm.too_hot():
                raise MotionError("모터가 뜨거워요. 식힌 뒤에 다시 하세요.")
            self.robot.torque(True)                    # 지금 자리에서 잠근다 (튀지 않음)
        self.torque_on = True
        self._stop.clear()
        self.status = {"state": "running", "message": "움직이는 중…", "name": name, "take": tid,
                       "step": 0, "total": len(points), "report": None}
        self._thread = threading.Thread(target=self._run, args=(name, tid, points), daemon=True)
        self._thread.start()
        return "재생을 시작했어요. 멈추려면 '정지'."

    def _run(self, name, tid, points):
        def on_step(i, n):
            self.status.update(step=i + 1, total=n, message=f"{i + 1} / {n} 번 지점으로 가는 중…")
        try:
            with self._arm_lock:
                report = play(self.robot, points, speed=self.cfg.speed_slow, stop=self._stop, on_step=on_step,
                              too_hot=self.arm.too_hot)
            if tid is not None:
                self.book.add_run(name, tid, report)
            if report["stopped"]:
                msg = f"멈췄어요 ({report['why']}). 팔은 그 자리에서 힘을 주고 있어요."
            elif report["reached"]:
                msg = f"끝까지 갔어요. 가장 많이 벗어난 정도 {report['max_err']}도. 잘 됐는지 표시해 주세요."
            else:
                msg = f"끝까지 갔지만 목표에서 {report['max_err']}도 벗어난 지점이 있어요 (걸리거나 무거웠을 수 있어요)."
            self.status.update(state="done", message=msg, report=report)
        except Exception as error:                     # 통신 끊김 · 토크 꺼짐 등 → 그 자리에서 멈춘다
            try:
                self.robot.stop()
            except Exception:
                pass
            self.status.update(state="error", message=f"재생 중 문제가 생겨 멈췄어요: {error}", report=None)

    def stop(self) -> str:
        if not self.busy():
            raise MotionError("지금 재생 중이 아니에요.")
        self._stop.set()
        return "정지 요청을 보냈어요. 지금 가는 지점까지만 가고 멈춰요."

    def join(self, timeout: float = 20.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def go_pose(self, pose: str) -> str:
        """저장된 자세(허브 1번에서 저장한 것)로 천천히 간다. 재생 전 시작 자리로 옮길 때 쓴다"""
        self._need_idle()
        with self._arm_lock:
            if pose not in self.robot.poses():
                raise MotionError(f"자세 '{pose}' 가 아직 저장되지 않았어요 (허브 1번 대기 자세 저장).")
            self.robot.torque(True)
            self.torque_on = True
            ok = self.arm.go(pose)
        return f"'{pose}' 자세로 갔어요." if ok else f"'{pose}' 자세에 다 가지 못했어요. 걸린 곳이 없는지 보세요."

    # ── 화면 버튼 → 일 ──────────────────────────────────────────────────
    def handle(self, action: str, data: dict) -> str:
        name, tid = data.get("name"), data.get("take")
        if action == "torque":
            return self.set_torque(data.get("on") is True)
        if action == "point-add":
            return self.add_point()
        if action == "point-undo":
            return self.undo_point()
        if action == "draft-clear":
            return self.clear_draft()
        if action == "draft-play":
            return self.start_play("(저장 전 초안)", None, list(self.draft))
        if action == "save":
            return self.save_take(name, data.get("label", ""))
        if action == "create":
            return f"동작 '{self.book.create(name, data.get('relative') is True)}' 칸을 만들었어요."
        if action == "rename":
            return f"동작 이름을 '{self.book.rename(name, data.get('new'))}' 로 바꿨어요."
        if action == "relative":
            self.book.set_relative(name, data.get("on") is True)
            return "차 위치에 맞추는 동작으로 표시했어요." if data.get("on") is True else "고정 동작으로 표시했어요."
        if action == "delete":
            self._need_idle()
            saved = self.book.delete_motion(name)
            return f"동작 '{name}' 을 지웠어요." + (f" 지우기 전 파일: {saved}" if saved else "")
        if action == "take-rename":
            self.book.rename_take(name, tid, data.get("label"))
            return "회차 이름을 바꿨어요."
        if action == "take-delete":
            self._need_idle()
            saved = self.book.delete_take(name, tid)
            return "회차를 지웠어요." + (f" 지우기 전 파일: {saved}" if saved else "")
        if action == "choose":
            self.book.choose(name, tid)
            return f"'{name}' 은 이제 {self.book.take(name, tid)['label']} 를 사용해요."
        if action == "play":
            return self.start_play(name, tid)
        if action == "mark":
            self.book.mark_run(name, tid, data.get("ok") is True)
            return "기록했어요."
        if action == "stop":
            return self.stop()
        if action == "car-ref":
            return self.set_car_ref()
        if action == "goto":
            return self.go_pose(str(data.get("pose", "")))
        raise MotionError("요청이 올바르지 않아요.")


# ── 화면 ────────────────────────────────────────────────────────────────
BODY = """
<h1><span class="num g-prep">3</span>동작 티칭</h1>
<p class=sub>팔을 손으로 움직여 동작을 가르치고, 저장한 동작을 눌러서 다시 움직여 봅니다. 같은 동작을 여러 번 가르쳐 두고 가장 안정적인 회차를 고르세요.</p>
<p class='msg warn' id=fake hidden>지금은 <b>가짜 팔</b>(화면 연습용)이에요. 실제 팔은 움직이지 않습니다.</p>
<p class=msg id=out role=status aria-live=polite>준비됐어요.</p>

<h2>① 가르치기</h2>
<div class=msg>
  <p><label>가르칠 동작 <select id=target></select></label>
     <span class=small id=target-help></span></p>
  <p><button id=t-off class=sec>힘 빼기 (손으로 받치고!)</button>
     <button id=t-on class=sec>힘 주기</button>
     <span id=torque class=small></span></p>
  <p><button id=p-add>＋ 지점 추가 (지금 팔 모양 담기)</button>
     <button id=p-undo class=sec>마지막 지점 빼기</button>
     <button id=p-clear class=sec>모두 비우기</button></p>
  <table><thead><tr><th>순서</th><th>받침</th><th>어깨</th><th>팔꿈치</th><th>손목 꺾기</th><th>손목 돌리기</th><th>그리퍼 %</th></tr></thead>
  <tbody id=draft></tbody></table>
  <p style='margin-top:12px'><label>회차 이름 (비우면 1차, 2차 …) <input type=text id=label maxlength=30 style='max-width:240px'></label></p>
  <p><button id=d-play class=sec>▶ 저장 전에 한 번 재생</button> <button id=save>새 회차로 저장</button></p>
</div>

<h2>② 재생 상태</h2>
<div class=msg><b id=st-title>대기</b> <span id=st-msg></span>
  <p><button id=stop class=sec disabled>■ 정지</button>
     <button id=go-watch class=sec>대기 자세(watch)로 가기</button></p>
  <p class=small>정지는 지금 가는 지점까지만 가고 멈춥니다. 급하면 팔 전원 스위치를 끄세요.</p></div>

<h2>③ 저장된 동작</h2>
<div id=list></div>

<h2>④ 새 동작 칸 만들기</h2>
<div class=msg><p><label>이름 <input type=text id=new-name maxlength=30 style='max-width:240px'></label>
  <label><input type=checkbox id=new-rel> 차 위치에 맞추는 동작</label> <button id=create class=sec>만들기</button></p></div>

<h2>⑤ 기준 차 위치</h2>
<div class=msg><p id=ref-text></p>
  <p class=small>차를 '기준 정차 위치'에 세워 두고, 팔을 대기 자세에서 받침만 돌려 차 위 상자가 화면 가운데에 오게 한 뒤 누르세요 (허브 2번 화면으로 확인).
  미션은 받침이 이 각도에 왔을 때 차에게 정지 신호를 보내고, '차 위치에 맞추는 동작'은 실제 정차 위치와 이 각도의 차이만큼 받침을 더 돌려 재생합니다.</p>
  <p><button id=ref class=sec>지금 받침 각도를 기준으로 저장</button></p></div>
"""

SCRIPT = """
<script>
const TOKEN='__TOKEN__';const $=id=>document.getElementById(id);
let S=null;
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
async function post(action,data){
  const r=await fetch('/api/motion/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Hub-Token':TOKEN},body:JSON.stringify(data||{})});
  const d=await r.json().catch(()=>({message:'응답을 읽지 못했어요.'}));
  const o=$('out');o.textContent=d.message;o.className='msg '+(r.ok?'ok':'bad');
  await refresh();return r.ok}
function fmtStats(s){if(!s.plays)return '아직 재생 안 함';
  return '재생 '+s.plays+'회 · 잘 됨 '+s.ok+' · 안 됨 '+s.bad+' · 평균 오차 '+(s.mean_err==null?'-':s.mean_err+'도')+' · 최대 '+(s.worst_err==null?'-':s.worst_err+'도')}
function render(){
  $('fake').hidden=!S.fake;
  const sel=$('target');const cur=sel.value;
  sel.innerHTML=S.motions.map(m=>'<option value="'+esc(m.name)+'">'+esc(m.name)+(m.relative?' (차 위치에 맞춤)':'')+'</option>').join('');
  if(S.motions.some(m=>m.name===cur))sel.value=cur;
  const tm=S.motions.find(m=>m.name===sel.value);$('target-help').textContent=tm?tm.help:'';
  $('torque').textContent=S.torque===null?'':(S.torque?'지금: 힘을 주고 있음':'지금: 힘을 뺌 (손으로 받치세요)');
  $('draft').innerHTML=S.draft.length?S.draft.map((p,i)=>'<tr><td>'+(i+1)+'</td>'+[1,2,3,4,5].map(j=>'<td>'+p.angles[j].toFixed(1)+'°</td>').join('')+'<td>'+(p.grip==null?'-':p.grip)+'</td></tr>').join(''):'<tr><td colspan=7 class=small>아직 담은 지점이 없어요. 힘을 빼고 팔을 옮긴 뒤 ＋ 지점 추가를 누르세요.</td></tr>';
  const st=S.status;$('st-title').textContent={idle:'대기',running:'재생 중',done:'끝',error:'문제 발생'}[st.state]+(st.name?' · '+st.name:'');
  $('st-msg').textContent=st.message;$('stop').disabled=!S.busy;
  for(const id of ['t-off','t-on','p-add','d-play','save','go-watch','ref'])$(id).disabled=S.busy;
  $('ref-text').innerHTML=S.car_ref?'저장된 기준: 받침 <b>'+S.car_ref.pan+'도</b> ('+esc(S.car_ref.t)+')':'<span class=warn>아직 저장하지 않았어요.</span>';
  $('list').innerHTML=S.motions.map(m=>{
    const rows=m.takes.length?m.takes.map(t=>{
      const d='data-name="'+esc(m.name)+'" data-take="'+t.id+'"';
      const mark=t.stats.plays&&!t.last_marked?'<button class=sec data-a=mark-ok '+d+'>잘 됨</button> <button class=sec data-a=mark-bad '+d+'>안 됨</button>':(t.last_marked?'<span class=small>마지막 재생: '+(t.last_ok?'잘 됨':'안 됨')+'</span>':'');
      return '<tr><td><b>'+esc(t.label)+'</b>'+(t.chosen?' <span class=ok>★ 사용 중</span>':'')+(t.recommended?' <span class=warn>추천</span>':'')+'<br><span class=small>지점 '+t.points+'개 · '+esc(t.created)+(m.relative?' · 기준 받침 '+(t.ref_pan==null?'없음':t.ref_pan+'도'):'')+'</span></td>'
        +'<td>'+fmtStats(t.stats)+'<br>'+mark+'</td>'
        +'<td><button data-a=play '+d+(S.busy?' disabled':'')+'>▶ 재생</button> '
        +(t.chosen?'':'<button class=sec data-a=choose '+d+'>★ 이걸로 사용</button> ')
        +'<button class=sec data-a=take-rename '+d+'>이름</button> <button class=sec data-a=take-delete '+d+'>지우기</button></td></tr>'}).join('')
      :'<tr><td colspan=3 class=small>아직 가르친 회차가 없어요. 위 ①에서 이 동작을 고르고 가르치세요.</td></tr>';
    const dn='data-name="'+esc(m.name)+'"';
    return '<h3 style="margin:18px 0 6px">'+esc(m.name)+' <span class=small>'+(m.relative?'차 위치에 맞추는 동작':'고정 동작')+(m.help?' · '+esc(m.help):'')+'</span></h3>'
      +'<table><thead><tr><th>회차</th><th>성적표</th><th>할 일</th></tr></thead><tbody>'+rows+'</tbody></table>'
      +'<p class=small><button class=sec data-a=rename '+dn+'>동작 이름 바꾸기</button> <button class=sec data-a=relative '+dn+' data-on="'+(m.relative?'0':'1')+'">'+(m.relative?'고정 동작으로 바꾸기':'차 위치에 맞추는 동작으로 바꾸기')+'</button> <button class=sec data-a=delete '+dn+'>동작 통째로 지우기</button></p>'}).join('');
}
async function refresh(){try{S=await (await fetch('/api/motion/state',{cache:'no-store'})).json();render()}catch(e){}}
$('target').onchange=render;
$('t-off').onclick=()=>{if(confirm('힘을 빼면 팔이 떨어질 수 있어요. 팔을 손으로 받치고 있나요?'))post('torque',{on:false})};
$('t-on').onclick=()=>post('torque',{on:true});
$('p-add').onclick=()=>post('point-add');$('p-undo').onclick=()=>post('point-undo');
$('p-clear').onclick=()=>{if(confirm('담아 둔 지점을 모두 비울까요?'))post('draft-clear')};
$('d-play').onclick=()=>{if(confirm('담아 둔 지점을 순서대로 움직여 봅니다. 주변을 비웠나요?'))post('draft-play')};
$('save').onclick=async()=>{if(await post('save',{name:$('target').value,label:$('label').value}))$('label').value=''};
$('stop').onclick=()=>post('stop');
$('go-watch').onclick=()=>{if(confirm('대기 자세(watch)로 천천히 움직입니다. 주변을 비웠나요?'))post('goto',{pose:'watch'})};
$('create').onclick=async()=>{if(await post('create',{name:$('new-name').value,relative:$('new-rel').checked})){$('target').value=$('new-name').value;$('new-name').value='';render()}};
$('ref').onclick=()=>{if(confirm('지금 받침 각도를 기준 차 위치로 저장할까요? (예전 기준은 덮어씁니다)'))post('car-ref')};
$('list').onclick=e=>{const b=e.target.closest('button[data-a]');if(!b)return;const a=b.dataset.a,name=b.dataset.name,take=b.dataset.take;
  if(a==='play'){if(confirm("'"+name+"' 을 재생합니다. 팔이 움직이니 주변을 비우세요. 시작 자리가 첫 지점과 멀면 크게 움직입니다."))post('play',{name,take})}
  else if(a==='choose')post('choose',{name,take});
  else if(a==='mark-ok')post('mark',{name,take,ok:true});
  else if(a==='mark-bad')post('mark',{name,take,ok:false});
  else if(a==='take-rename'){const v=prompt('회차 이름');if(v)post('take-rename',{name,take,label:v})}
  else if(a==='take-delete'){if(confirm('이 회차를 지울까요? (지우기 전 파일은 backup 폴더에 복사해 둡니다)'))post('take-delete',{name,take})}
  else if(a==='rename'){const v=prompt('새 동작 이름',name);if(v&&v!==name)post('rename',{name,new:v})}
  else if(a==='relative')post('relative',{name,on:b.dataset.on==='1'});
  else if(a==='delete'){if(confirm("동작 '"+name+"' 과 그 안의 회차를 모두 지울까요? (지우기 전 파일은 backup 폴더에 복사해 둡니다)"))post('delete',{name})}};
refresh();setInterval(()=>{if(S&&S.busy||S&&S.status.state==='running')refresh()},700);
</script>
"""


def motion_body() -> str:
    return BODY


def motion_script(token: str) -> str:
    return SCRIPT.replace("__TOKEN__", token)

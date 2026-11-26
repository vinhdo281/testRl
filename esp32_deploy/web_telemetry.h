#pragma once
// ============================================================================
// WIFI ACCESS POINT + TRANG WEB LOCAL XEM THONG SO MPU6050 / GIU GOC / DONG CO
//   - ESP32 tu phat WiFi (SSID/PASS ben duoi), mo trinh duyet: http://192.168.4.1
//   - Web server chay o task rieng tren core 0, vong dieu khien 50 Hz o core 1
//     khong bi anh huong. Chi doc (khong gui lenh dieu khien xuong xe).
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#define WEB_AP_SSID  "XeCanBang"
#define WEB_AP_PASS  "12345678"   // >= 8 ky tu, dat "" de mo (khong mat khau)
#define WEB_AP_CHAN  6

// state: 0 = khoi dong/test motor, 1 = calib MPU, 2 = dang can bang, 3 = an toan (cat motor), 4 = pause
struct WebTelemetry {
  float pitch, pitchAcc, pitchRate, yawRate;  // deg, deg, deg/s, deg/s
  float ax, ay, az;                           // g
  float gx, gy, gz;                           // deg/s (da tru offset)
  float wL, wR, vFwd;                         // rad/s, rad/s, m/s
  float actL, actR;                           // -1..1 (lenh PWM chuan hoa)
  long  encL, encR;
  float pitchZero;                            // deg (offset zero / calib)
  float trim;                                 // deg (trim luu NVS)
  float dtMs;
  uint8_t state;
  uint8_t i2cFail;
  int8_t  cfg[8];                             // truc X?, dau: pitch, yaw, encL, encR, motL, motR, mirrorEnc
};

static WebTelemetry g_tel = {};
static portMUX_TYPE g_telMux = portMUX_INITIALIZER_UNLOCKED;
static WebServer g_server(80);

// ---- Hang doi lenh tu web -> vong dieu khien (web task push, loop() pop) ----
enum WebCmdType : uint8_t { WCMD_NONE = 0, WCMD_TRIM, WCMD_SET_OFFSET, WCMD_ZERO, WCMD_RESET_TRIM, WCMD_CALIB, WCMD_CFG_AXIS, WCMD_CFG_FLIP, WCMD_PAUSE, WCMD_RESUME, WCMD_ENC_RESET, WCMD_AUTO_TRIM };
struct WebCmd { uint8_t type; float val; };
static WebCmd g_cmdQ[8];
static volatile uint8_t g_cmdHead = 0, g_cmdTail = 0;

static bool webPushCommand(uint8_t type, float val) {
  bool ok = false;
  portENTER_CRITICAL(&g_telMux);
  uint8_t next = (g_cmdHead + 1) & 7;
  if (next != g_cmdTail) { g_cmdQ[g_cmdHead] = {type, val}; g_cmdHead = next; ok = true; }
  portEXIT_CRITICAL(&g_telMux);
  return ok;
}

static bool webPopCommand(WebCmd &c) {
  bool ok = false;
  portENTER_CRITICAL(&g_telMux);
  if (g_cmdTail != g_cmdHead) { c = g_cmdQ[g_cmdTail]; g_cmdTail = (g_cmdTail + 1) & 7; ok = true; }
  portEXIT_CRITICAL(&g_telMux);
  return ok;
}

// ---- Dieu khien tay dong co (chi khi PAUSE). Co deadman: tu tat sau MAN_TIMEOUT_MS neu ngung nhan lenh ----
#define MAN_TIMEOUT_MS 400
static float g_manL = 0.0f, g_manR = 0.0f;
static uint32_t g_manUntil = 0;
static bool g_manActive = false;

static void webSetManual(float l, float r) {
  portENTER_CRITICAL(&g_telMux);
  g_manL = l; g_manR = r;
  g_manUntil = millis() + MAN_TIMEOUT_MS;
  g_manActive = (l != 0.0f || r != 0.0f);
  portEXIT_CRITICAL(&g_telMux);
}

// Tra ve false (va l=r=0) neu khong co lenh tay hoac da het han
static bool webGetManual(float &l, float &r) {
  bool ok = false;
  portENTER_CRITICAL(&g_telMux);
  if (g_manActive && (int32_t)(millis() - g_manUntil) < 0) { l = g_manL; r = g_manR; ok = true; }
  else { l = 0.0f; r = 0.0f; g_manActive = false; }
  portEXIT_CRITICAL(&g_telMux);
  return ok;
}

static void webClearCommands() {
  portENTER_CRITICAL(&g_telMux);
  g_cmdTail = g_cmdHead;
  portEXIT_CRITICAL(&g_telMux);
}

static void webPublish(const WebTelemetry &t) {
  portENTER_CRITICAL(&g_telMux);
  g_tel = t;
  portEXIT_CRITICAL(&g_telMux);
}

static void webSetState(uint8_t s) {
  portENTER_CRITICAL(&g_telMux);
  g_tel.state = s;
  portEXIT_CRITICAL(&g_telMux);
}

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Xe can bang RL</title>
<style>
:root{--bg:#0f1419;--card:#1a212b;--tx:#e6edf3;--mu:#8b98a5;--ac:#4cc2ff;--ok:#3fb950;--wr:#f0883e;--er:#f85149}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:14px system-ui,sans-serif;padding:12px;max-width:980px;margin:auto}
h1{font-size:18px;margin:0 0 10px;display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.badge{font-size:12px;padding:3px 10px;border-radius:99px;background:#30363d}
.b2{background:var(--ok);color:#04260f}.b1,.b0{background:var(--wr);color:#2b1300}.b3{background:var(--er);color:#fff}.b4{background:#d29922;color:#2b1d00}
#pb{padding:6px 16px;font-weight:600;border-color:#d29922;color:#d29922}#pb.rs{background:var(--ok);border-color:var(--ok);color:#04260f}
.dot{width:9px;height:9px;border-radius:50%;background:var(--er);display:inline-block}.dot.on{background:var(--ok)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:10px;margin-bottom:10px}
.card{background:var(--card);border-radius:10px;padding:12px}
.card h2{margin:0 0 8px;font-size:12px;color:var(--mu);text-transform:uppercase;letter-spacing:.05em;font-weight:600}
.big{font-size:34px;font-weight:600;font-variant-numeric:tabular-nums}
.u{font-size:13px;color:var(--mu);margin-left:4px;font-weight:400}
.row{display:flex;justify-content:space-between;padding:2px 0;font-variant-numeric:tabular-nums}
.row span:first-child{color:var(--mu)}
.bar{position:relative;height:12px;background:#0d1117;border-radius:6px;margin:4px 0 8px;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;background:var(--ac)}
.bar:after{content:"";position:absolute;left:50%;top:0;bottom:0;width:1px;background:#586069}
canvas{width:100%;height:150px;display:block;background:#0d1117;border-radius:8px}
.lg{font-size:12px;color:var(--mu);margin-top:6px}.lg b{display:inline-block;width:10px;height:10px;border-radius:2px;margin:0 4px 0 10px}
svg{width:100%;height:150px}
.btns{display:flex;gap:6px;flex-wrap:wrap;margin:6px 0}
button,input{background:#262e3a;color:var(--tx);border:1px solid #3a4452;border-radius:6px;padding:7px 11px;font:inherit}
button:active{background:#33404f}button.w{border-color:var(--wr);color:var(--wr)}button.g{color:var(--mu)}
input{width:90px}.hint{font-size:12px;color:var(--mu)}
</style></head><body>
<h1>Xe can bang RL <span id="st" class="badge">...</span> <button id="pb" onclick="togglePause()">&#10074;&#10074; Pause</button>
<span><span id="dot" class="dot"></span> <span id="hz" style="color:var(--mu);font-size:12px"></span></span></h1>

<div class="grid">
 <div class="card"><h2>Goc nghieng (pitch)</h2>
  <div class="big"><span id="pitch">0</span><span class="u">&deg;</span></div>
  <svg viewBox="-100 -80 200 100"><line x1="-95" y1="10" x2="95" y2="10" stroke="#30363d"/>
   <g id="rob"><rect x="-9" y="-62" width="18" height="62" rx="4" fill="#4cc2ff"/><circle r="10" cy="0" fill="#e6edf3"/></g></svg>
 </div>
 <div class="card"><h2>MPU6050</h2>
  <div class="row"><span>Pitch accel</span><span><b id="pacc">0</b>&deg;</span></div>
  <div class="row"><span>Pitch rate</span><span><b id="prate">0</b>&deg;/s</span></div>
  <div class="row"><span>Yaw rate</span><span><b id="yrate">0</b>&deg;/s</span></div>
  <div class="row"><span>Zero offset</span><span><b id="pz">0</b>&deg;</span></div>
  <div class="row"><span>I2C loi</span><span><b id="i2c">0</b></span></div>
  <div class="row"><span>Accel x/y/z (g)</span><span><b id="acc">-</b></span></div>
  <div class="row"><span>Gyro x/y/z (&deg;/s)</span><span><b id="gyr">-</b></span></div>
 </div>
 <div class="card"><h2>Dong co</h2>
  <div class="row"><span>Trai (PWM)</span><span><b id="aL">0</b>%</span></div>
  <div class="bar"><i id="bL"></i></div>
  <div class="row"><span>Phai (PWM)</span><span><b id="aR">0</b>%</span></div>
  <div class="bar"><i id="bR"></i></div>
  <div class="row"><span>Banh trai</span><span><b id="wL">0</b> rad/s</span></div>
  <div class="row"><span>Banh phai</span><span><b id="wR">0</b> rad/s</span></div>
  <div class="row"><span>Encoder T/P</span><span><b id="enc">0 / 0</b></span></div>
  <div class="row"><span>Van toc tien</span><span><b id="v">0</b> m/s</span></div>
 </div>
</div>

<div class="card" style="margin-bottom:10px"><h2>Hieu chinh</h2>
 <div class="grid" style="margin:0">
  <div>
   <div class="row"><span>Trim (diem can bang)</span><span><b id="trim">0</b>&deg;</span></div>
   <div class="btns"><button onclick="cmd('trim',-1)">-1</button><button onclick="cmd('trim',-0.5)">-0.5</button><button onclick="cmd('trim',-0.1)">-0.1</button>
    <button onclick="cmd('trim',0.1)">+0.1</button><button onclick="cmd('trim',0.5)">+0.5</button><button onclick="cmd('trim',1)">+1</button></div>
   <div class="btns"><button class="g" onclick="cmd('trimreset')">Reset trim</button><button class="w" onclick="cmd('autotrim')">Auto-trim (act = 0)</button></div>
   <div class="hint">Auto-trim: bu do lech co san cua mang RL (xe dung yen ma motor van bi day 1 phia). Pause va dat xe dung thang o diem can bang (da calib) truoc khi bam.</div>
   <div class="hint">Trim + = dich diem can bang ve phia truoc. Duoc luu vao bo nho, giu sau khi khoi dong lai.</div>
  </div>
  <div>
   <div class="row"><span>Offset zero</span><span><b id="off">0</b>&deg;</span></div>
   <div class="btns"><input id="offv" type="number" step="0.1" placeholder="do"><button onclick="setOff()">Dat offset</button></div>
   <div class="btns"><button class="g" onclick="if(confirm('Xe dang dung thang o diem can bang?'))cmd('zero')">Zero tai vi tri hien tai</button></div>
   <div class="hint">Zero: coi goc hien tai la 0&deg; (khong luu, mat khi khoi dong lai).</div>
  </div>
  <div>
   <div class="row"><span>Calib gyro + zero</span><span></span></div>
   <div class="btns"><button class="w" onclick="if(confirm('Se TAT motor 5 giay. Giu xe DUNG YEN o diem can bang. Tiep tuc?'))cmd('calib')">Calib lai (5 s)</button></div>
   <div class="hint">Tat motor, do lai offset gyro va zero goc. Giu xe dung yen luc calib.</div>
  </div>
 </div>
 <div id="msg" class="hint" style="margin-top:8px">&nbsp;</div>
</div>

<div class="card" style="margin-bottom:10px"><h2>Cau hinh chieu (luu vao bo nho)</h2>
 <div class="grid" style="margin:0">
  <div>
   <div class="row"><span>Truc pitch cua MPU6050</span><span><b id="cAxis">-</b></span></div>
   <div class="btns"><button onclick="if(confirm('Doi truc se calib lai 5 s (tat motor). Tiep tuc?'))cmd('axis')">Doi truc X &harr; Y</button></div>
   <div class="row"><span>Dau pitch (+ = nghieng tien)</span><span><b id="cPs">-</b></span></div>
   <div class="btns"><button onclick="if(confirm('Dao dau pitch se calib lai 5 s. Tiep tuc?'))cmd('flip',0)">Dao dau pitch</button></div>
   <div class="row"><span>Dau yaw</span><span><b id="cYs">-</b></span></div>
   <div class="btns"><button onclick="cmd('flip',1)">Dao dau yaw</button></div>
  </div>
  <div>
   <div class="row"><span>Dong co trai</span><span><b id="cMl">-</b></span></div>
   <div class="btns"><button onclick="cmd('flip',4)">Dao chieu</button></div>
   <div class="row"><span>Dong co phai</span><span><b id="cMr">-</b></span></div>
   <div class="btns"><button onclick="cmd('flip',5)">Dao chieu</button></div>
  </div>
  <div>
   <div class="row"><span>Encoder trai</span><span><b id="cEl">-</b></span></div>
   <div class="btns"><button onclick="cmd('flip',2)">Dao dau encoder</button></div>
   <div class="row"><span>Encoder phai</span><span><b id="cEr">-</b></span></div>
   <div class="btns"><button onclick="cmd('flip',3)">Dao dau encoder</button></div>
   <div class="row"><span>Mirror wR = wL (encoder phai hong)</span><span><b id="cMir">-</b></span></div>
   <div class="btns"><button class="w" onclick="cmd('flip',6)">Bat / tat mirror</button></div>
   <div class="hint">Dung muc "Test dong co" ben duoi: banh phai quay TIEN va encoder tang (+). Neu banh quay lui thi dao chieu dong co; neu banh tien ma encoder am thi dao dau encoder.</div>
  </div>
 </div>
</div>

<div class="card" style="margin-bottom:10px"><h2>Test dong co (giu nut de quay, tha ra la dung)</h2>
 <div class="hint" id="mtHint">Bam <b>Pause</b> truoc, kenh banh len khoi mat dat. Dong co chi quay khi dang giu nut.</div>
 <div class="row" style="margin-top:8px"><span>Cong suat</span><span><b id="pwv">30</b>%</span></div>
 <input id="pw" type="range" min="10" max="100" step="5" value="30" style="width:100%" oninput="$('pwv').textContent=this.value">
 <div class="grid" style="margin:8px 0 0">
  <div><div class="hint">Banh TRAI</div><div class="btns"><button class="mt" data-l="-1" data-r="0">&#9664; Lui</button><button class="mt" data-l="1" data-r="0">Tien &#9654;</button></div></div>
  <div><div class="hint">Banh PHAI</div><div class="btns"><button class="mt" data-l="0" data-r="-1">&#9664; Lui</button><button class="mt" data-l="0" data-r="1">Tien &#9654;</button></div></div>
  <div><div class="hint">CA HAI</div><div class="btns"><button class="mt" data-l="-1" data-r="-1">&#9664; Lui</button><button class="mt" data-l="1" data-r="1">Tien &#9654;</button></div></div>
 </div>
 <div class="row" style="margin-top:8px"><span>Encoder T / P (da nhan dau)</span><span><b id="tEnc">0 / 0</b></span></div>
 <div class="row"><span>Toc do banh T / P</span><span><b id="tW">0 / 0</b> rad/s</span></div>
 <div class="btns"><button class="g" onclick="cmd('encreset')">Reset encoder ve 0</button></div>
</div>

<div class="card" style="margin-bottom:10px"><h2>Goc nghieng theo thoi gian (30 s)</h2><canvas id="c1"></canvas>
 <div class="lg"><b style="background:#4cc2ff"></b>pitch loc<b style="background:#8b98a5"></b>pitch accel</div></div>
<div class="card"><h2>Lenh dong co theo thoi gian (30 s)</h2><canvas id="c2"></canvas>
 <div class="lg"><b style="background:#3fb950"></b>trai<b style="background:#f0883e"></b>phai &nbsp; vong dieu khien: <span id="dt">-</span> ms</div></div>

<script>
const $=id=>document.getElementById(id), N=300;
const H={p:[],a:[],l:[],r:[]};
const S=['KHOI DONG','CALIB MPU','DANG CAN BANG','AN TOAN (CAT MOTOR)','TAM DUNG (MOTOR TAT)'];
let paused=false;
function togglePause(){cmd(paused?'resume':'pause');}
function push(k,v){H[k].push(v);if(H[k].length>N)H[k].shift();}
function plot(cv,series,lo,hi){
 const dpr=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
 if(cv.width!==w*dpr){cv.width=w*dpr;cv.height=h*dpr;}
 const c=cv.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
 const y=v=>h-(v-lo)/(hi-lo)*h;
 c.strokeStyle='#30363d';c.fillStyle='#586069';c.font='10px sans-serif';c.lineWidth=1;
 for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4;c.beginPath();c.moveTo(0,y(v));c.lineTo(w,y(v));c.stroke();c.fillText(v.toFixed(0),3,y(v)-2);}
 c.lineWidth=1.6;
 for(const [d,col] of series){c.strokeStyle=col;c.beginPath();
  d.forEach((v,i)=>{const x=i/(N-1)*w;i?c.lineTo(x,y(v)):c.moveTo(x,y(v));});c.stroke();}
}
function msg(t){$('msg').textContent=t;}
async function cmd(c,v){
 try{const r=await fetch('/cmd?c='+c+(v===undefined?'':'&v='+v),{method:'POST'});
  msg(r.ok?'Da gui lenh: '+c+(v===undefined?'':' '+v):(r.status===409?'Phai bam Pause truoc':'Lenh bi tu choi ('+r.status+')'));}
 catch(e){msg('Mat ket noi');}
}
function setOff(){const v=parseFloat($('offv').value);if(isNaN(v)){msg('Nhap so do hop le');return;}cmd('offset',v);}
const sg=v=>v>0?'+1 (thuan)':'-1 (dao)';
function showCfg(c){$('cAxis').textContent=c[0]?'X (gx, ay)':'Y (gy, ax)';
 $('cPs').textContent=sg(c[1]);$('cYs').textContent=sg(c[2]);$('cEl').textContent=sg(c[3]);
 $('cEr').textContent=sg(c[4]);$('cMir').textContent=c[7]?'BAT (tam thoi)':'tat';$('cMl').textContent=sg(c[5]);$('cMr').textContent=sg(c[6]);}
let manTimer=null;
async function mot(l,r){try{const x=await fetch('/mot?l='+l+'&r='+r,{method:'POST'});if(x.status===409)msg('Phai bam Pause truoc khi test dong co');}catch(e){msg('Mat ket noi');}}
function manStop(){if(manTimer){clearInterval(manTimer);manTimer=null;mot(0,0);}}
document.querySelectorAll('.mt').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();manStop();const p=$('pw').value/100,l=b.dataset.l*p,r=b.dataset.r*p;
  mot(l,r);manTimer=setInterval(()=>mot(l,r),150);});
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,manStop));
 b.addEventListener('contextmenu',e=>e.preventDefault());
});
window.addEventListener('blur',manStop);
function bar(el,v){const m=Math.min(Math.abs(v),100)/2;el.style.width=m+'%';el.style.left=v<0?(50-m)+'%':'50%';}
const f=(v,n=1)=>Number(v).toFixed(n);
let last=performance.now(),hz=0;
function render(d){
 const s=$('st');s.textContent=S[d.state];s.className='badge b'+d.state;
 paused=d.state===4;const pb=$('pb');pb.innerHTML=paused?'&#9654; Tiep tuc':'&#10074;&#10074; Pause';pb.className=paused?'rs':'';
 $('pitch').textContent=f(d.pitch,2);
 $('rob').setAttribute('transform','rotate('+Math.max(-80,Math.min(80,d.pitch))+')');
 $('pacc').textContent=f(d.pitchAcc,2);$('prate').textContent=f(d.pitchRate);$('yrate').textContent=f(d.yawRate);
 $('pz').textContent=f(d.pitchZero,2);$('off').textContent=f(d.pitchZero,2);$('trim').textContent=f(d.trim,2);showCfg(d.cfg);
 $('tEnc').textContent=d.encL+' / '+d.encR;$('tW').textContent=f(d.wL)+' / '+f(d.wR);$('i2c').textContent=d.i2c;
 $('acc').textContent=f(d.ax,2)+' / '+f(d.ay,2)+' / '+f(d.az,2);
 $('gyr').textContent=f(d.gx)+' / '+f(d.gy)+' / '+f(d.gz);
 $('aL').textContent=f(d.actL*100,0);$('aR').textContent=f(d.actR*100,0);
 bar($('bL'),d.actL*100);bar($('bR'),d.actR*100);
 $('wL').textContent=f(d.wL);$('wR').textContent=f(d.wR);
 $('enc').textContent=d.encL+' / '+d.encR;$('v').textContent=f(d.v,2);$('dt').textContent=f(d.dt,1);
 push('p',d.pitch);push('a',d.pitchAcc);push('l',d.actL*100);push('r',d.actR*100);
 plot($('c1'),[[H.a,'#8b98a5'],[H.p,'#4cc2ff']],-60,60);
 plot($('c2'),[[H.l,'#3fb950'],[H.r,'#f0883e']],-100,100);
 const n=performance.now();hz=hz*.8+(1000/(n-last))*.2;last=n;$('hz').textContent=f(hz,0)+' Hz';
}
async function tick(){
 try{const r=await fetch('/data',{cache:'no-store'});render(await r.json());$('dot').className='dot on';}
 catch(e){$('dot').className='dot';}
 setTimeout(tick,80);
}
tick();
</script></body></html>)HTML";

static float webFinite(float v) { return isfinite(v) ? v : 0.0f; }

static void webHandleData() {
  WebTelemetry t;
  portENTER_CRITICAL(&g_telMux);
  t = g_tel;
  portEXIT_CRITICAL(&g_telMux);

  char buf[760];
  snprintf(buf, sizeof(buf),
    "{\"state\":%u,\"pitch\":%.2f,\"pitchAcc\":%.2f,\"pitchRate\":%.1f,\"yawRate\":%.1f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
    "\"wL\":%.2f,\"wR\":%.2f,\"v\":%.3f,\"actL\":%.3f,\"actR\":%.3f,"
    "\"cfg\":[%d,%d,%d,%d,%d,%d,%d,%d],\"encL\":%ld,\"encR\":%ld,\"pitchZero\":%.2f,\"trim\":%.2f,\"dt\":%.2f,\"i2c\":%u}",
    (unsigned)t.state, webFinite(t.pitch), webFinite(t.pitchAcc), webFinite(t.pitchRate), webFinite(t.yawRate),
    webFinite(t.ax), webFinite(t.ay), webFinite(t.az), webFinite(t.gx), webFinite(t.gy), webFinite(t.gz),
    webFinite(t.wL), webFinite(t.wR), webFinite(t.vFwd), webFinite(t.actL), webFinite(t.actR),
    t.cfg[0], t.cfg[1], t.cfg[2], t.cfg[3], t.cfg[4], t.cfg[5], t.cfg[6], t.cfg[7],
    t.encL, t.encR, webFinite(t.pitchZero), webFinite(t.trim), webFinite(t.dtMs), (unsigned)t.i2cFail);
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(200, "application/json", buf);
}

static void webHandleCmd() {
  String c = g_server.arg("c");
  float v = g_server.arg("v").toFloat();
  uint8_t type = WCMD_NONE;
  if      (c == "trim")      { type = WCMD_TRIM;        if (!isfinite(v) || fabsf(v) > 5.0f)  type = WCMD_NONE; }
  else if (c == "offset")    { type = WCMD_SET_OFFSET;  if (!isfinite(v) || fabsf(v) > 30.0f) type = WCMD_NONE; }
  else if (c == "zero")      type = WCMD_ZERO;
  else if (c == "trimreset") type = WCMD_RESET_TRIM;
  else if (c == "calib")     type = WCMD_CALIB;
  else if (c == "pause")     type = WCMD_PAUSE;
  else if (c == "resume")    type = WCMD_RESUME;
  else if (c == "axis")      type = WCMD_CFG_AXIS;
  else if (c == "flip")      { type = WCMD_CFG_FLIP; if (v < 0 || v > 6) type = WCMD_NONE; }
  else if (c == "encreset")  type = WCMD_ENC_RESET;
  else if (c == "autotrim")  {
    type = WCMD_AUTO_TRIM;
    bool paused;
    portENTER_CRITICAL(&g_telMux);
    paused = (g_tel.state == 4);
    portEXIT_CRITICAL(&g_telMux);
    if (!paused) { g_server.send(409, "application/json", "{\"ok\":0}"); return; }
  }
  if (type == WCMD_NONE) { g_server.send(400, "application/json", "{\"ok\":0}"); return; }
  if (!webPushCommand(type, v)) { g_server.send(503, "application/json", "{\"ok\":0}"); return; }
  g_server.send(200, "application/json", "{\"ok\":1}");
}

static void webHandleMotor() {
  float l = g_server.arg("l").toFloat();
  float r = g_server.arg("r").toFloat();
  bool paused;
  portENTER_CRITICAL(&g_telMux);
  paused = (g_tel.state == 4);
  portEXIT_CRITICAL(&g_telMux);
  if (!isfinite(l) || !isfinite(r) || fabsf(l) > 1.0f || fabsf(r) > 1.0f) { g_server.send(400, "application/json", "{\"ok\":0}"); return; }
  if (!paused && (l != 0.0f || r != 0.0f)) { g_server.send(409, "application/json", "{\"ok\":0}"); return; }
  webSetManual(l, r);
  g_server.send(200, "application/json", "{\"ok\":1}");
}

static void webServerTask(void *) {
  for (;;) {
    g_server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void webBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ok = (WEB_AP_PASS[0] == '\0') ? WiFi.softAP(WEB_AP_SSID, nullptr, WEB_AP_CHAN)
                                      : WiFi.softAP(WEB_AP_SSID, WEB_AP_PASS, WEB_AP_CHAN);
  Serial.print("WiFi AP '"); Serial.print(WEB_AP_SSID);
  Serial.print("' "); Serial.print(ok ? "OK" : "LOI");
  Serial.print(" -> http://"); Serial.println(WiFi.softAPIP());

  g_server.on("/", HTTP_GET, []() {
    g_server.sendHeader("Cache-Control", "no-store");
    g_server.send_P(200, "text/html; charset=utf-8", WEB_PAGE);
  });
  g_server.on("/data", HTTP_GET, webHandleData);
  g_server.on("/cmd", HTTP_POST, webHandleCmd);
  g_server.on("/mot", HTTP_POST, webHandleMotor);
  g_server.onNotFound([]() { g_server.send(404, "text/plain", "404"); });
  g_server.begin();

  // Core 0: tach khoi vong dieu khien (loop() chay o core 1)
  xTaskCreatePinnedToCore(webServerTask, "web", 6144, nullptr, 1, nullptr, 0);
}

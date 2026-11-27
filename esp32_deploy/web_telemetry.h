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
  uint8_t why;                                // ly do cat motor khi state = 3 (xem controller.h: Why)
  uint8_t i2cFail;
  float wLs, wRs;                             // rad/s, toc do banh theo mo hinh motor cua mo phong
  float wRr;                                  // rad/s, banh phai tu encoder phai (khong mirror)
  int8_t  cfg[8];                             // truc X?, dau: pitch, yaw, encL, encR, motL, motR, mirrorEnc
};

static WebTelemetry g_tel = {};
static portMUX_TYPE g_telMux = portMUX_INITIALIZER_UNLOCKED;
static WebServer g_server(80);

// ---- Hang doi lenh tu web -> vong dieu khien (web task push, loop() pop) ----
enum WebCmdType : uint8_t { WCMD_NONE = 0, WCMD_TRIM, WCMD_SET_OFFSET, WCMD_ZERO, WCMD_RESET_TRIM, WCMD_CALIB, WCMD_CFG_AXIS, WCMD_CFG_FLIP, WCMD_PAUSE, WCMD_RESUME, WCMD_ENC_RESET, WCMD_AUTO_TRIM, WCMD_REBOOT };
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
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b0f14">
<title>Xe cân bằng RL</title>
<style>
:root{--bg:#0b0f14;--sf:#131a23;--sf2:#1b2430;--ln:#253040;--tx:#e7edf4;--mu:#8795a7;--ac:#4cc2ff;--acbg:rgba(76,194,255,.14);
--ok:#3fb950;--okbg:rgba(63,185,80,.16);--wr:#f0a33e;--wrbg:rgba(240,163,62,.16);--er:#f85149;--erbg:rgba(248,81,73,.16);--pa:#e3b341;--pabg:rgba(227,179,65,.16);
--c1:#4cc2ff;--c2:#8795a7;--cl:#3fb950;--cr:#f0883e;--grid:#1f2935}
@media(prefers-color-scheme:light){:root{--bg:#f3f5f8;--sf:#fff;--sf2:#eef1f5;--ln:#dde3ea;--tx:#16202b;--mu:#5b6878;--ac:#0a7fd0;--acbg:rgba(10,127,208,.1);
--ok:#1a7f37;--okbg:rgba(26,127,55,.12);--wr:#b35900;--wrbg:rgba(179,89,0,.12);--er:#cf222e;--erbg:rgba(207,34,46,.1);--pa:#9a6700;--pabg:rgba(154,103,0,.12);
--c1:#0a7fd0;--c2:#9aa5b3;--cl:#1a7f37;--cr:#d4620b;--grid:#e8ecf1}}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;font-variant-numeric:tabular-nums}
button,input{font:inherit;color:inherit}
button{background:var(--sf2);border:1px solid var(--ln);border-radius:9px;padding:8px 13px;cursor:pointer;font-weight:500;touch-action:manipulation;user-select:none;-webkit-user-select:none}
button:hover{border-color:var(--mu)}button:active{transform:translateY(1px)}
button.pri{background:var(--ac);border-color:var(--ac);color:#fff}
button.warn{border-color:var(--wr);color:var(--wr)}button.dng{border-color:var(--er);color:var(--er)}
button.ghost{background:none}
button:disabled{opacity:.4;cursor:not-allowed;transform:none}
input[type=number]{background:var(--bg);border:1px solid var(--ln);border-radius:9px;padding:8px 10px;width:100px}
.wrap{max-width:1100px;margin:auto;padding:0 14px}
/* header */
.top{position:sticky;top:0;z-index:20;background:var(--bg);border-bottom:1px solid var(--ln)}
.hd{display:flex;align-items:center;gap:10px;flex-wrap:wrap;padding:10px 0}
.brand{display:flex;align-items:center;gap:10px;margin-right:auto}
.logo{width:34px;height:34px;border-radius:10px;background:var(--acbg);color:var(--ac);display:grid;place-items:center}
.brand b{display:block;font-size:15px;line-height:1.2}.brand small{color:var(--mu);font-size:12px}
.conn{display:flex;align-items:center;gap:7px;color:var(--mu);font-size:12px;min-width:58px}
.dot{width:8px;height:8px;border-radius:50%;background:var(--er);box-shadow:0 0 0 3px var(--erbg)}
.dot.on{background:var(--ok);box-shadow:0 0 0 3px var(--okbg)}
.badge{font-size:12px;font-weight:700;padding:5px 11px;border-radius:99px;background:var(--sf2);color:var(--mu);letter-spacing:.03em;white-space:nowrap}
.b0,.b1{background:var(--wrbg);color:var(--wr)}.b2{background:var(--okbg);color:var(--ok)}.b3{background:var(--erbg);color:var(--er)}.b4{background:var(--pabg);color:var(--pa)}
#pb{font-weight:700;min-width:124px;border-color:var(--pa);color:var(--pa);background:var(--pabg)}
#pb.rs{background:var(--ok);border-color:var(--ok);color:#fff}
#pb,#rb{white-space:nowrap}
@media(max-width:440px){.rbt{display:none}#rb{padding:8px 12px;font-size:16px;line-height:1.1}#pb{padding-left:8px;padding-right:8px}.badge{padding:5px 9px}}
.tabs{display:flex;gap:2px}
.tabs button{background:none;border:0;border-bottom:2px solid transparent;border-radius:0;color:var(--mu);font-weight:600;padding:9px 12px;display:flex;align-items:center;gap:7px}
.tabs button.on{color:var(--tx);border-bottom-color:var(--ac)}
.tabs svg,.logo svg{width:18px;height:18px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.lost{display:none;background:var(--erbg);color:var(--er);text-align:center;font-size:13px;font-weight:600;padding:6px}
.lost.on{display:block}
/* layout */
main{padding:14px 0 28px}
.tab{display:none}.tab.on{display:block}
.g2{display:grid;gap:12px;grid-template-columns:minmax(0,1fr) minmax(0,1fr);margin-bottom:12px}
.g3{display:grid;gap:12px;grid-template-columns:repeat(3,minmax(0,1fr));margin-bottom:12px}
.stack{display:grid;gap:12px;align-content:start}
.card{background:var(--sf);border:1px solid var(--ln);border-radius:14px;padding:14px 16px}
.card h2{margin:0 0 12px;font-size:12px;font-weight:700;color:var(--mu);text-transform:uppercase;letter-spacing:.06em;display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap}
.hint{font-size:12.5px;color:var(--mu);margin:8px 0 0}
.hint b{color:var(--tx)}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.u{font-style:normal;color:var(--mu);font-size:.5em;font-weight:500;margin-left:3px}
/* gauge */
.gz{display:flex;align-items:baseline;justify-content:space-between;gap:10px}
.big{font-size:46px;font-weight:700;line-height:1;letter-spacing:-.02em}
.dir{font-size:13px;color:var(--mu);text-align:right}
#gauge{width:100%;height:auto;display:block;margin-top:4px;max-height:260px}
#gauge text{fill:var(--mu);font-size:9px;text-anchor:middle}
.kv{display:flex;justify-content:space-between;gap:10px;padding:6px 0;border-top:1px solid var(--ln)}
.kv:first-child{border-top:0}
.kv span{color:var(--mu)}.kv b{font-weight:600}
/* tiles */
.tiles{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.tile{background:var(--sf);border:1px solid var(--ln);border-radius:14px;padding:12px 14px}
.tile span{display:block;color:var(--mu);font-size:12px;margin-bottom:2px}
.tile b{font-size:24px;font-weight:700}
.tile .u{font-size:13px}
/* motors */
.mrow+.mrow{margin-top:14px}
.mh{display:flex;justify-content:space-between;align-items:baseline}
.mh span{font-weight:600}.mh b{font-size:20px}
.bar{position:relative;height:10px;background:var(--sf2);border-radius:99px;margin:6px 0;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;width:0;border-radius:99px}
.bar:after{content:"";position:absolute;left:50%;top:-2px;bottom:-2px;width:2px;margin-left:-1px;background:var(--mu);opacity:.6}
.ms{display:flex;justify-content:space-between;font-size:12.5px;color:var(--mu)}
.ms b{color:var(--tx);font-weight:600}
/* xyz table */
.xyz{width:100%;border-collapse:collapse}
.xyz th{font-size:11px;color:var(--mu);font-weight:600;text-align:right;padding:0 0 6px}
.xyz th:first-child{text-align:left}
.xyz td{padding:7px 0;border-top:1px solid var(--ln);text-align:right;font-weight:600}
.xyz td:first-child{text-align:left;color:var(--mu);font-weight:400}
.bad{color:var(--er)!important}
/* charts */
canvas{width:100%;height:190px;display:block}
.lg{display:flex;gap:14px;font-size:12px;color:var(--mu);font-weight:500;text-transform:none;letter-spacing:0}
.lg i{display:inline-block;width:10px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
.lg b{color:var(--tx);margin-left:4px}
.lg i.ds{background:repeating-linear-gradient(90deg,var(--k) 0 3px,transparent 3px 5px)}
/* calibrate */
.val{font-size:34px;font-weight:700;line-height:1.1}
.step{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin-top:12px}
.step button{padding:9px 0}
/* config */
.li{display:flex;align-items:center;gap:10px;padding:10px 0;border-top:1px solid var(--ln)}
.li:first-of-type{border-top:0;padding-top:0}
.li div{flex:1;min-width:0}.li small{display:block;color:var(--mu);font-size:12px}
.chip{font-size:12px;font-weight:700;padding:3px 9px;border-radius:99px;background:var(--sf2);color:var(--mu);white-space:nowrap}
.chip.inv{background:var(--acbg);color:var(--ac)}.chip.on{background:var(--wrbg);color:var(--wr)}
/* motor test */
.lock{display:flex;align-items:center;gap:12px;background:var(--pabg);color:var(--pa);border-radius:12px;padding:10px 14px;margin-bottom:12px;font-weight:600}
.lock span{flex:1}
.lock.off{display:none}
.pw{display:flex;align-items:center;gap:14px;white-space:nowrap}
.pw input{flex:1;accent-color:var(--ac)}
.pw b{font-size:22px;min-width:64px;text-align:right}
.pad{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin-top:14px}
.pad h3{margin:0 0 8px;font-size:12px;color:var(--mu);text-transform:uppercase;letter-spacing:.06em;text-align:center}
.mt{display:block;width:100%;padding:16px 0;font-size:15px;font-weight:700;touch-action:none}
.mt+.mt{margin-top:8px}
.mt.hold{background:var(--ac);border-color:var(--ac);color:#fff}
/* toast + modal */
#toast{position:fixed;left:50%;bottom:22px;transform:translate(-50%,20px);opacity:0;pointer-events:none;background:var(--tx);color:var(--bg);padding:9px 16px;border-radius:10px;font-weight:600;font-size:13px;transition:.2s;z-index:40;max-width:calc(100% - 28px);text-align:center}
#toast.on{opacity:1;transform:translate(-50%,0)}
#toast.e{background:var(--er);color:#fff}
#modal{position:fixed;inset:0;background:rgba(0,0,0,.55);display:none;align-items:center;justify-content:center;z-index:50;padding:16px}
#modal.on{display:flex}
#modal .card{max-width:400px;width:100%}
#modal h3{margin:0 0 6px;font-size:17px}
#modal p{margin:0;color:var(--mu)}
#modal .btns{justify-content:flex-end;margin-top:18px}
@media(max-width:860px){.g3{grid-template-columns:minmax(0,1fr)}}
@media(max-width:700px){
 .g2{grid-template-columns:minmax(0,1fr)}
 .hd{gap:8px}
 .brand{flex:1 0 calc(100% - 76px)}
 #pb{min-width:0;flex:1}
 .tabs{position:fixed;left:0;right:0;bottom:0;background:var(--sf);border-top:1px solid var(--ln);padding:5px 6px calc(5px + env(safe-area-inset-bottom));justify-content:space-around;z-index:30}
 .tabs button{flex:1;flex-direction:column;gap:2px;font-size:11px;padding:6px 2px;border:0;border-radius:10px}
 .tabs button.on{background:var(--acbg);color:var(--ac)}
 main{padding-bottom:96px}
 #toast{bottom:84px}
 .big{font-size:40px}
 .pad{gap:8px}.mt{padding:20px 0}
}
</style></head><body>
<header class="top"><div class="wrap">
 <div class="hd">
  <div class="brand"><span class="logo"><svg viewBox="0 0 24 24"><rect x="9" y="2" width="6" height="13" rx="2"/><circle cx="12" cy="18" r="4"/></svg></span>
   <div><b>Xe cân bằng RL</b><small>ESP32 · 192.168.4.1</small></div></div>
  <div class="conn"><span id="dot" class="dot"></span><span id="hz">--</span></div>
  <span id="st" class="badge">…</span>
  <button id="pb" onclick="togglePause()">&#10074;&#10074; Tạm dừng</button>
  <button id="rb" class="dng" title="Khởi động lại ESP32" onclick="reboot()">&#8635;<span class="rbt"> Reset</span></button>
 </div>
 <nav class="tabs">
  <button data-t="mon" class="on"><svg viewBox="0 0 24 24"><path d="M3 12h4l3-8 4 16 3-8h4"/></svg>Giám sát</button>
  <button data-t="cal"><svg viewBox="0 0 24 24"><path d="M4 7h9M17 7h3M4 17h3M11 17h9"/><circle cx="15" cy="7" r="2"/><circle cx="9" cy="17" r="2"/></svg>Hiệu chỉnh</button>
  <button data-t="cfg"><svg viewBox="0 0 24 24"><path d="M7 20V4M3 8l4-4 4 4M17 4v16M13 16l4 4 4-4"/></svg>Cấu hình</button>
  <button data-t="mot"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2"/><path d="M12 4v6M12 14v6M4 12h6M14 12h6"/></svg>Test motor</button>
 </nav>
</div><div id="lost" class="lost">Mất kết nối với xe — đang thử lại…</div></header>

<main class="wrap">
<!-- ================= GIÁM SÁT ================= -->
<section id="t-mon" class="tab on">
 <div class="g2">
  <div class="card">
   <h2>Góc nghiêng (pitch)<span id="pz2" style="text-transform:none;letter-spacing:0;font-weight:500"></span></h2>
   <div class="gz"><div class="big"><span id="pitch">0.00</span><i class="u">°</i></div><div class="dir" id="pdir">—</div></div>
   <svg id="gauge" viewBox="-120 -118 240 150">
    <g id="garc"></g>
    <line x1="-112" y1="16" x2="112" y2="16" stroke="var(--ln)" stroke-width="2"/>
    <line id="gacc" x1="0" y1="0" x2="0" y2="-104" stroke="var(--c2)" stroke-width="1.5" stroke-dasharray="3 3"/>
    <g id="rob"><rect x="-10" y="-88" width="20" height="86" rx="6" fill="var(--ac)"/><rect x="-6" y="-80" width="12" height="18" rx="3" fill="var(--bg)" opacity=".35"/>
     <circle r="15" fill="var(--sf2)" stroke="var(--tx)" stroke-width="3"/><circle r="3" fill="var(--tx)"/></g>
   </svg>
   <div class="hint">Nét đứt = góc từ gia tốc kế. Vùng xanh ±5°, cắt motor khi vượt ±45°.</div>
  </div>
  <div class="stack">
   <div class="tiles">
    <div class="tile"><span>Tốc độ góc pitch</span><b id="prate">0</b><i class="u">°/s</i></div>
    <div class="tile"><span>Tốc độ quay yaw</span><b id="yrate">0</b><i class="u">°/s</i></div>
    <div class="tile"><span>Vận tốc tiến</span><b id="v">0</b><i class="u">m/s</i></div>
    <div class="tile"><span>Chu kỳ điều khiển</span><b id="dt">-</b><i class="u">ms</i></div>
   </div>
   <div class="card"><h2>Động cơ</h2>
    <div class="mrow"><div class="mh"><span>Trái</span><b><span id="aL">0</span><i class="u">%</i></b></div>
     <div class="bar"><i id="bL" style="background:var(--cl)"></i></div>
     <div class="ms"><span>ω <b id="wL">0</b> rad/s</span><span>encoder <b id="eL">0</b></span></div></div>
    <div class="mrow"><div class="mh"><span>Phải</span><b><span id="aR">0</span><i class="u">%</i></b></div>
     <div class="bar"><i id="bR" style="background:var(--cr)"></i></div>
     <div class="ms"><span>ω <b id="wR">0</b> rad/s</span><span>encoder <b id="eR">0</b></span></div></div>
   </div>
  </div>
 </div>
 <div class="g2">
  <div class="card"><h2>Góc nghiêng · 30 s<span class="lg"><span><i style="background:var(--c1)"></i>lọc<b id="lp">0</b>°</span><span><i style="background:var(--c2)"></i>accel<b id="la">0</b>°</span></span></h2><canvas id="c1"></canvas></div>
  <div class="card"><h2>Lệnh động cơ · 30 s<span class="lg"><span><i style="background:var(--cl)"></i>trái<b id="ll">0</b>%</span><span><i style="background:var(--cr)"></i>phải<b id="lr">0</b>%</span></span></h2><canvas id="c2"></canvas></div>
 </div>
 <div class="g2">
  <div class="card"><h2>Bánh trái · rad/s<span class="lg"><span><i style="background:var(--cl)"></i>thực tế<b class="vwl">0</b></span><span><i class="ds" style="--k:var(--cl)"></i>mô phỏng<b class="vwls">0</b></span></span></h2><canvas id="c3"></canvas></div>
  <div class="card"><h2>Bánh phải · rad/s<span class="lg"><span><i style="background:var(--cr)"></i>thực tế<b class="vwr">0</b></span><span><i class="ds" style="--k:var(--cr)"></i>mô phỏng<b class="vwrs">0</b></span></span></h2><canvas id="c4"></canvas></div>
 </div>
 <p class="hint"><b>Mô phỏng</b> = tốc độ bánh mà mô hình motor dùng lúc train (MuJoCo) sẽ đạt với <b>cùng lệnh PWM</b>, khi bánh quay tự do. So đúng nhất ở tab <b>Test motor</b> khi kê bánh lên: thực tế thấp hơn nhiều → motor/nguồn yếu hơn mô phỏng; ngược dấu → sai chiều motor/encoder. Khi xe đứng trên đất, bánh chịu tải nên thực tế thấp hơn là bình thường. Bánh phải luôn vẽ encoder phải thật (không mirror).</p>
 <div class="g2">
  <div class="card"><h2>MPU6050 · thô</h2>
   <table class="xyz"><tr><th></th><th>X</th><th>Y</th><th>Z</th></tr>
    <tr><td>Gia tốc (g)</td><td id="ax">-</td><td id="ay">-</td><td id="az">-</td></tr>
    <tr><td>Con quay (°/s)</td><td id="gx">-</td><td id="gy">-</td><td id="gz">-</td></tr></table>
  </div>
  <div class="card"><h2>Ước lượng</h2>
   <div class="kv"><span>Pitch từ gia tốc kế</span><b><span id="pacc">0</span>°</b></div>
   <div class="kv"><span>Offset zero</span><b><span id="pz">0</span>°</b></div>
   <div class="kv"><span>Trim</span><b><span id="trim2">0</span>°</b></div>
   <div class="kv"><span>Lỗi I2C</span><b id="i2c">0</b></div>
  </div>
 </div>
</section>

<!-- ================= HIỆU CHỈNH ================= -->
<section id="t-cal" class="tab">
 <div class="g3">
  <div class="card"><h2>Trim · điểm cân bằng</h2>
   <div class="val"><span id="trim">0.00</span><i class="u">°</i></div>
   <div class="step"><button onclick="cmd('trim',-1)">−1</button><button onclick="cmd('trim',-0.5)">−0.5</button><button onclick="cmd('trim',-0.1)">−0.1</button>
    <button onclick="cmd('trim',0.1)">+0.1</button><button onclick="cmd('trim',0.5)">+0.5</button><button onclick="cmd('trim',1)">+1</button></div>
   <div class="btns"><button class="ghost" onclick="cmd('trimreset')">Đặt lại trim</button><button class="warn" onclick="autoTrim()">Auto-trim (act = 0)</button></div>
   <p class="hint">Trim + dịch điểm cân bằng về phía trước. Được lưu vào bộ nhớ, giữ sau khi khởi động lại.</p>
   <p class="hint"><b>Auto-trim:</b> bù độ lệch sẵn có của mạng RL (xe đứng yên mà motor vẫn bị đẩy về một phía). Cần <b>Tạm dừng</b> và đặt xe đứng thẳng ở điểm cân bằng trước khi bấm.</p>
  </div>
  <div class="card"><h2>Offset zero</h2>
   <div class="val"><span id="off">0.00</span><i class="u">°</i></div>
   <div class="btns"><input id="offv" type="number" step="0.1" placeholder="độ" inputmode="decimal"><button class="pri" onclick="setOff()">Đặt offset</button></div>
   <div class="btns"><button onclick="ask('Zero tại vị trí hiện tại?','Xe phải đang đứng thẳng ở điểm cân bằng.','Zero',()=>cmd('zero'))">Zero tại vị trí hiện tại</button></div>
   <p class="hint">Zero: coi góc hiện tại là 0°. Offset và Zero đều được lưu vào bộ nhớ, giữ sau khi khởi động lại.</p>
  </div>
  <div class="card"><h2>Calib gyro + zero</h2>
   <p class="hint" style="margin-top:0">Tắt motor 5 giây. Giữ xe <b>đứng yên</b>. Bias con quay đo được ở mọi tư thế; điểm 0 chỉ lấy lại (và lưu) khi xe đứng thẳng trong ±25°, nếu không sẽ giữ điểm 0 đã lưu.</p>
   <div class="btns"><button class="dng" onclick="ask('Calib lại MPU6050?','Motor sẽ TẮT trong 5 giây. Giữ xe đứng yên ở điểm cân bằng.','Calib (5 s)',()=>cmd('calib'))">Calib lại (5 s)</button></div>
  </div>
 </div>
</section>

<!-- ================= CẤU HÌNH ================= -->
<section id="t-cfg" class="tab">
 <div class="g3">
  <div class="card"><h2>Cảm biến MPU6050</h2>
   <div class="li"><div><b>Trục pitch</b><small>Đổi trục sẽ calib lại 5 s</small></div><span class="chip" id="cAxis">-</span>
    <button onclick="ask('Đổi trục pitch X ↔ Y?','Sẽ calib lại 5 s (tắt motor).','Đổi trục',()=>cmd('axis'))">Đổi</button></div>
   <div class="li"><div><b>Dấu pitch</b><small>+ = nghiêng tới · calib lại 5 s</small></div><span class="chip" id="cPs">-</span>
    <button onclick="ask('Đảo dấu pitch?','Sẽ calib lại 5 s (tắt motor).','Đảo dấu',()=>cmd('flip',0))">Đảo</button></div>
   <div class="li"><div><b>Dấu yaw</b></div><span class="chip" id="cYs">-</span><button onclick="cmd('flip',1)">Đảo</button></div>
  </div>
  <div class="card"><h2>Động cơ</h2>
   <div class="li"><div><b>Động cơ trái</b></div><span class="chip" id="cMl">-</span><button onclick="cmd('flip',4)">Đảo chiều</button></div>
   <div class="li"><div><b>Động cơ phải</b></div><span class="chip" id="cMr">-</span><button onclick="cmd('flip',5)">Đảo chiều</button></div>
   <p class="hint">Dùng tab <b>Test motor</b>: bánh phải quay TỚI và encoder tăng (+). Bánh quay lùi → đảo chiều động cơ; bánh tới mà encoder âm → đảo dấu encoder.</p>
  </div>
  <div class="card"><h2>Encoder</h2>
   <div class="li"><div><b>Encoder trái</b></div><span class="chip" id="cEl">-</span><button onclick="cmd('flip',2)">Đảo dấu</button></div>
   <div class="li"><div><b>Encoder phải</b></div><span class="chip" id="cEr">-</span><button onclick="cmd('flip',3)">Đảo dấu</button></div>
   <div class="li"><div><b>Mirror wR = wL</b><small>Tạm dùng khi encoder phải hỏng</small></div><span class="chip" id="cMir">-</span><button class="warn" onclick="cmd('flip',6)">Bật/tắt</button></div>
  </div>
 </div>
 <p class="hint">Mọi thay đổi ở đây được lưu vào bộ nhớ (NVS) của ESP32.</p>
</section>

<!-- ================= TEST ĐỘNG CƠ ================= -->
<section id="t-mot" class="tab">
 <div id="lock" class="lock"><span>Phải tạm dừng trước khi test. Kê bánh lên khỏi mặt đất.</span><button class="warn" onclick="cmd('pause')">Tạm dừng</button></div>
 <div class="card">
  <h2>Giữ nút để quay · thả ra là dừng</h2>
  <div class="pw"><span style="color:var(--mu)">Công suất</span><input id="pw" type="range" min="10" max="100" step="5" value="30" oninput="$('pwv').textContent=this.value"><b><span id="pwv">30</span><i class="u">%</i></b></div>
  <div class="pad">
   <div><h3>Bánh trái</h3><button class="mt" data-l="1" data-r="0">▲ Tới</button><button class="mt" data-l="-1" data-r="0">▼ Lùi</button></div>
   <div><h3>Cả hai</h3><button class="mt" data-l="1" data-r="1">▲ Tới</button><button class="mt" data-l="-1" data-r="-1">▼ Lùi</button></div>
   <div><h3>Bánh phải</h3><button class="mt" data-l="0" data-r="1">▲ Tới</button><button class="mt" data-l="0" data-r="-1">▼ Lùi</button></div>
  </div>
 </div>
 <div class="g2" style="margin-top:12px">
  <div class="card"><h2>Bánh trái</h2>
   <div class="kv"><span>Encoder (đã nhân dấu)</span><b id="tEl">0</b></div>
   <div class="kv"><span>Tốc độ thực tế / mô phỏng</span><b><span id="tWl">0</span> / <span class="vwls">0</span> rad/s</b></div>
   <canvas id="c5"></canvas></div>
  <div class="card"><h2>Bánh phải</h2>
   <div class="kv"><span>Encoder (đã nhân dấu)</span><b id="tEr">0</b></div>
   <div class="kv"><span>Tốc độ thực tế / mô phỏng</span><b><span id="tWr">0</span> / <span class="vwrs">0</span> rad/s</b></div>
   <canvas id="c6"></canvas></div>
 </div>
 <p class="hint">Nét liền = encoder thật, nét đứt = mô hình motor của mô phỏng với cùng lệnh. Ở 100%, mô phỏng đạt ≈ 12,7 rad/s.</p>
 <div class="btns"><button class="ghost" onclick="cmd('encreset')">Đặt encoder về 0</button></div>
</section>
</main>

<div id="toast"></div>
<div id="modal"><div class="card"><h3 id="mT"></h3><p id="mP"></p>
 <div class="btns"><button class="ghost" onclick="closeAsk()">Huỷ</button><button class="pri" id="mOk"></button></div></div></div>

<script>
const $=id=>document.getElementById(id);
const S=['KHỞI ĐỘNG','CALIB MPU','ĐANG CÂN BẰNG','AN TOÀN · CẮT MOTOR','TẠM DỪNG'];
const W=['CẮT MOTOR','CHỜ DỰNG THẲNG','ĐÃ NGÃ','LỖI MPU6050','BÁNH QUAY TỰ DO'];
const WL=['Cắt motor','Chờ dựng thẳng: đưa xe về gần thẳng (< 8°) và giữ yên 0,3 s','Nghiêng quá 45° → cắt motor','Không đọc được MPU6050 → cắt motor','Bánh quay tự do / đẩy 1 chiều liên tục 0,8 s → cắt motor (xe bị nhấc lên?). Đặt xe thẳng, bánh đứng yên để chạy lại'];
const SPAN=30000,H={t:[],p:[],a:[],l:[],r:[],wl:[],wls:[],wr:[],wrs:[]};
let paused=false,fails=0;
const f=(v,n=1)=>Number(v).toFixed(n);
const css=k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim();

/* ---- tabs ---- */
document.querySelectorAll('.tabs button').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('.tabs button').forEach(x=>x.classList.toggle('on',x===b));
 document.querySelectorAll('.tab').forEach(s=>s.classList.toggle('on',s.id==='t-'+b.dataset.t));
 manStop();scrollTo(0,0);
});

/* ---- toast / modal ---- */
let tT;
function msg(t,err){const e=$('toast');e.textContent=t;e.className='on'+(err?' e':'');clearTimeout(tT);tT=setTimeout(()=>e.className=err?'e':'',2200);}
function ask(t,p,ok,fn){$('mT').textContent=t;$('mP').textContent=p;const b=$('mOk');b.textContent=ok;b.onclick=()=>{closeAsk();fn();};$('modal').className='on';}
function closeAsk(){$('modal').className='';}
$('modal').onclick=e=>{if(e.target.id==='modal')closeAsk();};

/* ---- commands ---- */
const NAME={trim:'Trim',trimreset:'Đặt lại trim',autotrim:'Auto-trim',offset:'Offset',zero:'Zero',calib:'Calib',pause:'Tạm dừng',resume:'Tiếp tục',axis:'Đổi trục',flip:'Đảo cấu hình',encreset:'Reset encoder'};
async function cmd(c,v){
 try{const r=await fetch('/cmd?c='+c+(v===undefined?'':'&v='+v),{method:'POST'});
  if(r.ok)msg('✓ '+(NAME[c]||c)+(v===undefined||c==='flip'?'':' '+(v>0&&c==='trim'?'+':'')+v));
  else msg(r.status===409?'Phải tạm dừng trước':'Lệnh bị từ chối ('+r.status+')',1);}
 catch(e){msg('Mất kết nối',1);}
}
function togglePause(){cmd(paused?'resume':'pause');}
function reboot(){ask('Khởi động lại ESP32?','Giống bấm nút EN trên mạch. Hai bánh sẽ tự quay thử khoảng 1 giây, sau đó calib 5 giây: dựng xe thẳng đứng và giữ yên đến khi đèn LED sáng đứng. Trim và cấu hình vẫn giữ nguyên. WiFi sẽ ngắt khoảng 10 giây; nếu trang không tự kết nối lại, hãy vào lại WiFi XeCanBang.','Khởi động lại',async()=>{
 try{const r=await fetch('/cmd?c=reboot',{method:'POST'});
  if(r.ok){msg('Đang khởi động lại…');fails=3;online(false);}else msg('Lệnh bị từ chối ('+r.status+')',1);}
 catch(e){msg('Mất kết nối',1);}});}
function autoTrim(){if(!paused){msg('Phải tạm dừng trước',1);return;}
 ask('Auto-trim?','Xe phải đang đứng thẳng ở điểm cân bằng (đã calib). Trim mới sẽ được lưu.','Auto-trim',()=>cmd('autotrim'));}
function setOff(){const v=parseFloat($('offv').value);if(isNaN(v)||Math.abs(v)>30){msg('Nhập số độ trong khoảng ±30',1);return;}cmd('offset',v);}

/* ---- motor test (deadman: firmware tự tắt sau 400 ms nếu không nhận lệnh) ---- */
let manTimer=null,manBtn=null;
async function mot(l,r){try{const x=await fetch('/mot?l='+l+'&r='+r,{method:'POST'});if(x.status===409){manStop();msg('Phải tạm dừng trước khi test động cơ',1);}}catch(e){}}
function manStop(){if(manTimer){clearInterval(manTimer);manTimer=null;mot(0,0);}if(manBtn){manBtn.classList.remove('hold');manBtn=null;}}
document.querySelectorAll('.mt').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();manStop();
  if(!paused){msg('Phải tạm dừng trước khi test động cơ',1);return;}
  const p=$('pw').value/100,l=b.dataset.l*p,r=b.dataset.r*p;
  manBtn=b;b.classList.add('hold');mot(l,r);manTimer=setInterval(()=>mot(l,r),150);});
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,manStop));
 b.addEventListener('contextmenu',e=>e.preventDefault());
});
addEventListener('blur',manStop);
document.addEventListener('visibilitychange',()=>{if(document.hidden)manStop();});

/* ---- gauge ---- */
const R=100,pt=(a,r)=>[(r*Math.sin(a*Math.PI/180)).toFixed(1),(-r*Math.cos(a*Math.PI/180)).toFixed(1)];
function arc(a0,a1,col,w){const[x0,y0]=pt(a0,R),[x1,y1]=pt(a1,R);
 return '<path d="M'+x0+' '+y0+'A'+R+' '+R+' 0 0 1 '+x1+' '+y1+'" stroke="'+col+'" stroke-width="'+w+'" fill="none"/>';}
(function(){let s=arc(-60,-45,'var(--er)',6)+arc(-45,-5,'var(--wr)',6)+arc(-5,5,'var(--ok)',6)+arc(5,45,'var(--wr)',6)+arc(45,60,'var(--er)',6);
 for(let a=-60;a<=60;a+=5){const m=a%15===0,[x0,y0]=pt(a,R+5),[x1,y1]=pt(a,R+(m?12:9));
  s+='<line x1="'+x0+'" y1="'+y0+'" x2="'+x1+'" y2="'+y1+'" stroke="var(--mu)" stroke-width="'+(m?1.5:.8)+'"/>';
  if(m&&a%30===0&&a){const[tx,ty]=pt(a,R-12);s+='<text x="'+tx+'" y="'+(+ty+3)+'">'+(a>0?'+':'')+a+'°</text>';}}
 $('garc').innerHTML=s;})();

/* ---- charts ---- */
function plot(cv,series,lo,hi,now,unit){
 const dpr=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;if(!w)return;
 if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);}
 const c=cv.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
 const L=34,B=16,pw=w-L,ph=h-B-4,y=v=>4+(hi-v)/(hi-lo)*ph,x=t=>L+pw-(now-t)/SPAN*pw;
 c.font='10px system-ui,sans-serif';c.lineWidth=1;c.textAlign='right';c.textBaseline='middle';
 const gc=css('--grid'),mc=css('--mu');
 for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,yy=Math.round(y(v))+.5;c.strokeStyle=v===0?mc:gc;c.globalAlpha=v===0?.5:1;
  c.beginPath();c.moveTo(L,yy);c.lineTo(w,yy);c.stroke();c.globalAlpha=1;c.fillStyle=mc;c.fillText(f(v,0)+unit,L-5,yy);}
 c.textAlign='center';c.textBaseline='alphabetic';
 for(let s=0;s<=30;s+=10){const xx=L+pw-s/30*pw;c.fillText(s?'-'+s+'s':'bây giờ',Math.min(Math.max(xx,L+14),w-20),h-3);}
 c.save();c.beginPath();c.rect(L,0,pw,h-B);c.clip();c.lineWidth=1.8;c.lineJoin='round';
 for(const[d,col,ds]of series){c.strokeStyle=css(col);c.setLineDash(ds?[6,4]:[]);c.beginPath();
  for(let i=0;i<d.length;i++){const xx=x(H.t[i]),yy=y(Math.max(lo,Math.min(hi,d[i])));i?c.lineTo(xx,yy):c.moveTo(xx,yy);}c.stroke();}
 c.setLineDash([]);c.restore();
}
function push(d,now){H.t.push(now);H.p.push(d.pitch);H.a.push(d.pitchAcc);H.l.push(d.actL*100);H.r.push(d.actR*100);
 H.wl.push(d.wL);H.wls.push(d.wLs);H.wr.push(d.wRr);H.wrs.push(d.wRs);
 while(H.t.length&&now-H.t[0]>SPAN+500)for(const k in H)H[k].shift();}
function range(){let m=0;for(const v of H.p)m=Math.max(m,Math.abs(v));for(const v of H.a)m=Math.max(m,Math.abs(v));
 return Math.min(90,Math.max(10,Math.ceil(m/10)*10));}

/* ---- render ---- */
function bar(el,v){const m=Math.min(Math.abs(v),100)/2;el.style.width=m+'%';el.style.left=v<0?(50-m)+'%':'50%';}
const sg=v=>v>0?'+1 thuận':'−1 đảo';
function chip(id,txt,cls){const e=$(id);e.textContent=txt;e.className='chip'+(cls?' '+cls:'');}
function showCfg(c){chip('cAxis',c[0]?'X (gx, ay)':'Y (gy, ax)');
 chip('cPs',sg(c[1]),c[1]<0&&'inv');chip('cYs',sg(c[2]),c[2]<0&&'inv');chip('cEl',sg(c[3]),c[3]<0&&'inv');chip('cEr',sg(c[4]),c[4]<0&&'inv');
 chip('cMl',sg(c[5]),c[5]<0&&'inv');chip('cMr',sg(c[6]),c[6]<0&&'inv');chip('cMir',c[7]?'BẬT (tạm)':'tắt',c[7]&&'on');}
let last=performance.now(),hz=0;
function render(d){
 const s=$('st'),cut=d.state===3;s.textContent=cut?(W[d.why]||W[0]):(S[d.state]||'?');s.className='badge b'+(cut&&d.why===1?1:d.state);s.title=cut?(WL[d.why]||''):'';
 paused=d.state===4;const pb=$('pb');pb.innerHTML=paused?'&#9654; Tiếp tục':'&#10074;&#10074; Tạm dừng';pb.className=paused?'rs':'';
 $('lock').className='lock'+(paused?' off':'');if(!paused&&manTimer)manStop();
 const p=d.pitch,ap=Math.abs(p);
 $('pitch').textContent=f(p,2);$('pitch').style.color=ap<5?'':ap<45?css('--wr'):css('--er');
 $('pdir').textContent=ap<0.5?'thẳng đứng':(p>0?'nghiêng tới':'nghiêng lùi');
 $('rob').setAttribute('transform','rotate('+Math.max(-80,Math.min(80,p))+')');
 $('gacc').setAttribute('transform','rotate('+Math.max(-80,Math.min(80,d.pitchAcc))+')');
 $('prate').textContent=f(d.pitchRate);$('yrate').textContent=f(d.yawRate);$('v').textContent=f(d.v,2);$('dt').textContent=f(d.dt,1);
 $('aL').textContent=f(d.actL*100,0);$('aR').textContent=f(d.actR*100,0);bar($('bL'),d.actL*100);bar($('bR'),d.actR*100);
 $('wL').textContent=f(d.wL);$('wR').textContent=f(d.wR);$('eL').textContent=d.encL;$('eR').textContent=d.encR;
 ['ax','ay','az'].forEach(k=>$(k).textContent=f(d[k],2));['gx','gy','gz'].forEach(k=>$(k).textContent=f(d[k]));
 $('pacc').textContent=f(d.pitchAcc,2);$('pz').textContent=f(d.pitchZero,2);$('off').textContent=f(d.pitchZero,2);
 $('trim').textContent=f(d.trim,2);$('trim2').textContent=f(d.trim,2);
 $('pz2').textContent='zero '+f(d.pitchZero,2)+'° · trim '+f(d.trim,2)+'°';
 $('i2c').textContent=d.i2c;$('i2c').className=d.i2c>0?'bad':'';
 showCfg(d.cfg);
 $('tEl').textContent=d.encL;$('tEr').textContent=d.encR;$('tWl').textContent=f(d.wL);$('tWr').textContent=f(d.wRr);
 const now=performance.now();push(d,now);
 $('lp').textContent=f(p,1);$('la').textContent=f(d.pitchAcc,1);$('ll').textContent=f(d.actL*100,0);$('lr').textContent=f(d.actR*100,0);
 for(const[k,v]of[['vwl',d.wL],['vwls',d.wLs],['vwr',d.wRr],['vwrs',d.wRs]])document.querySelectorAll('.'+k).forEach(e=>e.textContent=f(v));
 const wsL=[[H.wls,'--cl',1],[H.wl,'--cl']],wsR=[[H.wrs,'--cr',1],[H.wr,'--cr']];
 if($('t-mon').classList.contains('on')){const g=range();
  plot($('c1'),[[H.a,'--c2'],[H.p,'--c1']],-g,g,now,'°');plot($('c2'),[[H.l,'--cl'],[H.r,'--cr']],-100,100,now,'');
  plot($('c3'),wsL,-16,16,now,'');plot($('c4'),wsR,-16,16,now,'');}
 if($('t-mot').classList.contains('on')){plot($('c5'),wsL,-16,16,now,'');plot($('c6'),wsR,-16,16,now,'');}
 hz=hz*.8+(1000/Math.max(1,now-last))*.2;last=now;$('hz').textContent=f(hz,0)+' Hz';
}
function online(ok){fails=ok?0:fails+1;$('dot').className=ok?'dot on':'dot';$('lost').className=fails>=3?'lost on':'lost';if(!ok)$('hz').textContent='--';}
async function tick(){
 const ac=new AbortController(),to=setTimeout(()=>ac.abort(),1500);
 try{const r=await fetch('/data',{cache:'no-store',signal:ac.signal});render(await r.json());online(true);}
 catch(e){online(false);}
 clearTimeout(to);setTimeout(tick,80);
}
tick();
</script></body></html>)HTML";

static float webFinite(float v) { return isfinite(v) ? v : 0.0f; }

static void webHandleData() {
  WebTelemetry t;
  portENTER_CRITICAL(&g_telMux);
  t = g_tel;
  portEXIT_CRITICAL(&g_telMux);

  char buf[900];
  snprintf(buf, sizeof(buf),
    "{\"state\":%u,\"why\":%u,\"pitch\":%.2f,\"pitchAcc\":%.2f,\"pitchRate\":%.1f,\"yawRate\":%.1f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
    "\"wL\":%.2f,\"wR\":%.2f,\"wLs\":%.2f,\"wRs\":%.2f,\"wRr\":%.2f,\"v\":%.3f,\"actL\":%.3f,\"actR\":%.3f,"
    "\"cfg\":[%d,%d,%d,%d,%d,%d,%d,%d],\"encL\":%ld,\"encR\":%ld,\"pitchZero\":%.2f,\"trim\":%.2f,\"dt\":%.2f,\"i2c\":%u}",
    (unsigned)t.state, (unsigned)t.why, webFinite(t.pitch), webFinite(t.pitchAcc), webFinite(t.pitchRate), webFinite(t.yawRate),
    webFinite(t.ax), webFinite(t.ay), webFinite(t.az), webFinite(t.gx), webFinite(t.gy), webFinite(t.gz),
    webFinite(t.wL), webFinite(t.wR), webFinite(t.wLs), webFinite(t.wRs), webFinite(t.wRr),
    webFinite(t.vFwd), webFinite(t.actL), webFinite(t.actR),
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
  else if (c == "reboot")    type = WCMD_REBOOT;
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

#!/usr/bin/env python3
"""Processor-in-the-Loop: MuJoCo (PC) <-> ESP32 chay mang RL qua USB-Serial.

  PC : mo phong vat ly, tinh obs, ap dung action
  ESP: nhan obs (RAW, da nhan OBS_SCALES) -> rl_policy_predict() -> tra action

Dung:  python3 hil_sim.py --headless --seconds 40      (khong can man hinh)
       python3 hil_sim.py                              (mo cua so 3D, dieu khien bang phim)
Giao thuc: xem esp32_hil/src/main.cpp
"""
import argparse, math, select, struct, sys, termios, time, tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
import serial
from serial.tools import list_ports

from envs.balance_car_env import SelfBalancingCarEnv
from policy_np import NpPolicy

SCALES = SelfBalancingCarEnv.OBS_SCALES


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class EspLink:
    REP_LEN = 2 + 1 + 8 + 4 + 2

    def __init__(self, port, baud=921600):
        self.seq = 0
        self.retries = 0
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.01
        self.ser.dtr = False   # tranh reset ESP khi mo cong
        self.ser.rts = False
        self.ser.open()
        self.buf = bytearray()
        if not self._probe():
            print("[HIL] ESP khong tra loi -> reset bang RTS va doi READY...")
            self.ser.rts = True; time.sleep(0.1); self.ser.rts = False
            t0 = time.time(); txt = b""
            while time.time() - t0 < 4 and b"READY" not in txt:
                txt += self.ser.read(256)
            print("[HIL] boot:", txt.decode(errors="ignore").strip().splitlines()[-1:] or "(khong thay)")
            if not self._probe():
                raise RuntimeError("ESP32 khong tra loi frame. Da nap esp32_hil chua? Toc do 921600?")

    def _probe(self):
        self.ser.reset_input_buffer()
        for _ in range(3):
            try:
                self.query(np.zeros(12, np.float32), timeout=0.4, retries=0)
                return True
            except TimeoutError:
                pass
        return False

    def query(self, raw_obs, timeout=0.1, retries=3):
        """Gui obs, doi action. Mang khong co trang thai nen gui lai la an toan."""
        self.seq = (self.seq + 1) & 0xFF
        payload = bytes([self.seq]) + np.asarray(raw_obs, "<f4").tobytes()
        frame = b"\xA5\x5A" + payload + struct.pack("<H", crc16(payload))
        for attempt in range(retries + 1):
            self.ser.write(frame)
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < timeout:
                self.buf += self.ser.read(self.ser.in_waiting or 1)
                r = self._parse()
                if r is not None:
                    return r + (time.perf_counter() - t0,)
            self.retries += 1
        raise TimeoutError("ESP khong tra loi")

    def _parse(self):
        b = self.buf
        while True:
            i = b.find(b"\x5A\xA5")
            if i < 0:
                del b[:max(0, len(b) - 1)]
                return None
            if i:
                del b[:i]
            if len(b) < self.REP_LEN:
                return None
            body = bytes(b[2:15])   # seq(1) + action(8) + infer_us(4)
            if crc16(body) == struct.unpack("<H", b[15:17])[0] and body[0] == self.seq:
                act = np.frombuffer(body[1:9], "<f4").copy()
                us = struct.unpack("<I", body[9:13])[0]
                del b[:self.REP_LEN]
                return act, us
            del b[:1]   # header gia -> resync

    def close(self):
        self.ser.close()


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x10C4 or "USB" in p.device:
            return p.device
    raise SystemExit("Khong thay cong serial ESP32")


class XHold:
    """Doc trang thai phim DANG GIU (X11) -> nha phim la xe dung. Phim: mui ten / WASD / IJKL."""
    NAMES = {"fwd": ["Up", "w", "i"], "back": ["Down", "s", "k"],
             "left": ["Left", "a", "j"], "right": ["Right", "d", "l"]}

    def __init__(self):
        from Xlib import display, XK
        self.d = display.Display()
        self.codes = {k: [self.d.keysym_to_keycode(XK.string_to_keysym(n)) for n in v]
                      for k, v in self.NAMES.items()}

    def read(self, v=0.40, w=1.5):
        km = self.d.query_keymap()
        down = lambda k: any(km[c // 8] & (1 << (c % 8)) for c in self.codes[k])
        return v * (down("fwd") - down("back")), w * (down("left") - down("right"))


class Stats:
    def __init__(self):
        self.rtt, self.us, self.diff = [], [], 0.0
        self.falls = self.steps = 0

    def report(self, link):
        r = np.array(self.rtt) * 1e3
        print(f"\n=== KET QUA HIL ===\nbuoc: {self.steps}  nga: {self.falls}  gui lai frame: {link.retries}")
        print(f"|action ESP - numpy| max: {self.diff:.2e}")
        print(f"round-trip ms  p50={np.percentile(r,50):.2f}  p99={np.percentile(r,99):.2f}  max={r.max():.2f}  (chu ky dieu khien 20 ms)")
        print(f"suy luan tren ESP (us) p50={np.percentile(self.us,50):.0f}  max={max(self.us)}")


def step_once(env, obs, link, ref, st, rt_check=True):
    act, us, rtt = link.query(obs * SCALES)
    if rt_check:
        st.diff = max(st.diff, float(np.abs(act - ref.predict_raw(obs * SCALES)).max()))
    st.rtt.append(rtt); st.us.append(us); st.steps += 1
    return env.step(act)


def run_headless(link, seconds, cmds, push):
    ref, st = NpPolicy(), Stats()
    env = SelfBalancingCarEnv(domain_rand=False, command_mode="manual", max_episode_steps=10**6)
    obs, _ = env.reset(seed=1)
    n = int(seconds / env.dt)
    seg = max(1, n // len(cmds))
    t_start = time.time()
    for i in range(n):
        if i % seg == 0:
            env.set_command(*cmds[min(i // seg, len(cmds) - 1)])
            print(f"[t={i*env.dt:5.1f}s] cmd v={env._target_v:+.2f} yaw={env._target_yaw_rate:+.2f}")
        if push and i % 250 == 125:
            env.apply_external_push(2.0 * (1 if (i // 250) % 2 == 0 else -1), 0)
        obs, _, term, _, info = step_once(env, obs, link, ref, st)
        if term:
            st.falls += 1
            print(f"  !! nga o buoc {i} pitch={info['pitch_deg']:.1f} -> reset")
            obs, _ = env.reset(seed=100 + i); env.set_command(0, 0)
    print(f"(chay {time.time()-t_start:.1f}s thoi gian that cho {seconds}s mo phong)")
    st.report(link)
    return st


def run_viewer(link):
    import mujoco.viewer
    ref, st = NpPolicy(), Stats()
    env = SelfBalancingCarEnv(domain_rand=False, command_mode="manual", max_episode_steps=10**6)
    obs, _ = env.reset(seed=1)
    cmd = {"v": 0.0, "w": 0.0, "push": 0.0, "sign": 1.0}

    try:
        hold = XHold()
    except Exception as e:
        hold = None
        print(f"[HIL] Khong doc duoc phim giu ({e}) -> dung che do bam-la-giu, X de dung")

    def key(ch):
        ch = ch.upper()
        if hold and ch != "P":   # di chuyen do XHold lo (giu phim = chay, nha = dung)
            return
        if ch in ("W", "I", "UP"): cmd["v"], cmd["w"] = 0.40, 0.0
        elif ch in ("S", "K", "DOWN"): cmd["v"], cmd["w"] = -0.40, 0.0
        elif ch in ("A", "J", "LEFT"): cmd["w"] = 1.5
        elif ch in ("D", "L", "RIGHT"): cmd["w"] = -1.5
        elif ch in ("X", "0", " "): cmd["v"], cmd["w"] = 0.0, 0.0
        elif ch == "P": cmd["push"] = 2.0 * cmd["sign"]; cmd["sign"] *= -1
        else: return
        print(f"[CMD] v={cmd['v']:+.2f} yaw={cmd['w']:+.2f}")

    gl = {265: "UP", 264: "DOWN", 263: "LEFT", 262: "RIGHT", **{ord(c): c for c in "IKJLXP0 "}}
    is_tty = sys.stdin.isatty()
    old = termios.tcgetattr(sys.stdin) if is_tty else None
    if is_tty: tty.setcbreak(sys.stdin.fileno())
    print("GIU mui ten / WASD / IJKL de di chuyen (nha ra la xe dung), P = day thu. Dong cua so de thoat.")
    try:
        with mujoco.viewer.launch_passive(env.model, env.data, key_callback=lambda k: key(gl[k]) if k in gl else None) as v:
            px, py = float(env.data.qpos[0]), float(env.data.qpos[1])
            t_log = time.time()
            while v.is_running():
                t0 = time.perf_counter()
                if is_tty:
                    while select.select([sys.stdin], [], [], 0)[0]:
                        c = sys.stdin.read(1)
                        if c == "\x1b":
                            c += sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ""
                            c = {"\x1b[A": "UP", "\x1b[B": "DOWN", "\x1b[D": "LEFT", "\x1b[C": "RIGHT"}.get(c, "")
                        key(c) if c else None
                if hold:
                    nv, nw = hold.read()
                    if (nv, nw) != (cmd["v"], cmd["w"]):
                        print(f"[CMD] v={nv:+.2f} yaw={nw:+.2f}")
                    cmd["v"], cmd["w"] = nv, nw
                env.set_command(cmd["v"], cmd["w"])
                if cmd["push"]:
                    env.apply_external_push(cmd["push"], 0); cmd["push"] = 0
                obs, _, term, _, info = step_once(env, obs, link, ref, st)
                x, y = float(env.data.qpos[0]), float(env.data.qpos[1])
                v.cam.lookat[0] += x - px; v.cam.lookat[1] += y - py; px, py = x, y
                v.sync()
                if term:
                    st.falls += 1; print("Xe nga -> reset")
                    obs, _ = env.reset(seed=int(time.time()) % 10000); env.set_command(0, 0)
                    px, py = float(env.data.qpos[0]), float(env.data.qpos[1])
                if time.time() - t_log > 2:
                    t_log = time.time()
                    print(f"pitch={info['pitch_deg']:+5.1f}  v={info['v_fwd']:+.2f}  rtt={np.mean(st.rtt[-50:])*1e3:.1f}ms  ESP={np.mean(st.us[-50:]):.0f}us")
                el = time.perf_counter() - t0
                if el < env.dt: time.sleep(env.dt - el)
    finally:
        if old: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    st.report(link)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float, default=40)
    ap.add_argument("--no-push", action="store_true")
    a = ap.parse_args()
    link = EspLink(a.port or find_port(), a.baud)
    print(f"[HIL] ESP32 san sang tren {link.ser.port}")
    try:
        if a.headless:
            demo = [(0, 0), (0.4, 0), (0, 0), (0.32, 1.2), (-0.38, 0), (0, -1.5), (0, 0)]
            run_headless(link, a.seconds, demo, not a.no_push)
        else:
            run_viewer(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()

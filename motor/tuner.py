#!/usr/bin/env python3
"""
Turntable motor tuner for the Pico firmware in motor.c.

    pip install pyserial matplotlib numpy
    python tuner.py                         # auto-detects the Pico
    python tuner.py --port /dev/cu.usbmodem1101

Close any other serial monitor (VS Code, screen, etc.) first; only one program
can hold the port.

Workflow:
  1. Run sweep   -> duty->RPM curve, deadband, local gain K, feedforward duty
                    (fills the FF box; press Apply to send it)
  2. Run step    -> time constant tau and K at the operating point,
                    suggested KP/KI (fills the boxes; press Apply)
  3. PI on       -> watch the live plot, adjust, Apply, repeat
Every test result is also saved as a CSV in the current directory.
"""

import argparse
import csv
import queue
import sys
import threading
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox
import serial
from serial.tools import list_ports

PICO_VID = 0x2E8A
WINDOW_S = 20.0
COLUMNS = ["t_s", "mode", "duty", "rpm", "target", "kp", "ki", "ff"]
# row tuple: (t_s, mode, duty, rpm, target, kp, ki, ff)


# ---------------------------------------------------------------------------
# Serial link
# ---------------------------------------------------------------------------

def find_port():
    for p in list_ports.comports():
        if p.vid == PICO_VID:
            return p.device
    return None


class Link:
    """Background reader that stores telemetry rows; thread-safe writer."""

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.write_lock = threading.Lock()
        self.rows_lock = threading.Lock()
        self.rows = []
        self.messages = queue.Queue()
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.messages.put(f"serial error: {e}")
                self.running = False
                return
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line.startswith("D,"):
                parts = line.split(",")
                if len(parts) != 9:
                    continue
                try:
                    row = (float(parts[1]) / 1000.0, parts[2],
                           *(float(x) for x in parts[3:]))
                except ValueError:
                    continue
                with self.rows_lock:
                    self.rows.append(row)
            elif line:
                self.messages.put(line)

    def send(self, cmd):
        with self.write_lock:
            self.ser.write((cmd + "\n").encode())

    def recent(self, n):
        with self.rows_lock:
            return self.rows[-n:]

    def all_rows(self):
        with self.rows_lock:
            return list(self.rows)

    def latest_t(self):
        with self.rows_lock:
            return self.rows[-1][0] if self.rows else None

    def rows_since(self, t0):
        with self.rows_lock:
            i = len(self.rows)
            while i > 0 and self.rows[i - 1][0] >= t0:
                i -= 1
            return self.rows[i:]

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Tests (run in a worker thread; results go back through a queue)
# ---------------------------------------------------------------------------

def run_sweep(link, start, stop, step, settle, avg_s, out, abort):
    points = []
    for d in np.arange(start, stop + step / 2, step):
        link.send(f"duty {d:.2f}")
        out.put(("status", f"sweep: duty {d:.1f}% ..."))
        if abort.wait(settle):
            break
        t_a = link.latest_t()
        if abort.wait(avg_s):
            break
        if t_a is None:
            out.put(("status", "sweep: no telemetry from the Pico"))
            break
        rpms = [r[3] for r in link.rows_since(t_a) if abs(r[2] - d) < 0.01]
        if rpms:
            points.append((float(d), float(np.mean(rpms)), float(np.std(rpms))))
    link.send("stop")
    out.put(("sweep", points) if not abort.is_set() else ("status", "sweep aborted"))


def run_step(link, d0, d1, settle, out, abort):
    link.send(f"duty {d0:.2f}")
    out.put(("status", f"step: settling at {d0:.1f}% ..."))
    if abort.wait(settle):
        link.send("stop")
        out.put(("status", "step aborted"))
        return
    t_pre = link.latest_t()
    link.send(f"duty {d1:.2f}")
    out.put(("status", f"step: {d0:.1f}% -> {d1:.1f}%, recording ..."))
    if abort.wait(2 * settle):
        link.send("stop")
        out.put(("status", "step aborted"))
        return
    link.send("stop")
    if t_pre is None:
        out.put(("status", "step: no telemetry from the Pico"))
        return
    out.put(("step", (d0, d1, link.rows_since(t_pre - 1.0))))


# ---------------------------------------------------------------------------
# Analysis (pure functions)
# ---------------------------------------------------------------------------

def analyze_sweep(points, target):
    d = np.array([p[0] for p in points])
    r = np.array([p[1] for p in points])
    moving = r > 0.5
    if moving.sum() < 2:
        raise ValueError("motor barely moved during the sweep")
    dm, rm = d[moving], r[moving]
    deadband = float(dm[0])
    k_all, b_all = np.polyfit(dm, rm, 1)

    # local fit around the target speed
    idx = np.argsort(np.abs(rm - target))[:4]
    if len(np.unique(dm[idx])) < 2:
        raise ValueError("not enough points near the target speed")
    k_loc, b_loc = np.polyfit(dm[idx], rm[idx], 1)
    ff = (target - b_loc) / k_loc
    in_range = rm.min() <= target <= rm.max()
    return dict(d=d, r=r, deadband=deadband, k_all=k_all, b_all=b_all,
                k_loc=k_loc, ff=ff, in_range=in_range)


def analyze_step(d0, d1, rows, lam_ratio):
    t = np.array([row[0] for row in rows])
    y = np.array([row[3] for row in rows])
    duty = np.array([row[2] for row in rows])

    at_new = np.abs(duty - d1) < 0.01
    if not at_new.any():
        raise ValueError("never saw the new duty in the telemetry")
    i0 = int(np.argmax(at_new))
    t0 = t[i0]

    pre = y[(t < t0) & (t >= t0 - 0.5)]
    if len(pre) == 0:
        raise ValueError("no data before the step")
    y0 = pre.mean()

    tp, yp = t[i0:], y[i0:]
    y1 = yp[tp >= tp[-1] - 0.25 * (tp[-1] - t0)].mean()
    dy = y1 - y0
    if abs(dy) < 0.5:
        raise ValueError(f"step changed speed by only {dy:.2f} RPM")

    ys = np.convolve(yp, np.ones(3) / 3, mode="same")
    frac = (ys - y0) / dy
    crossed = frac >= 0.632
    if not crossed.any():
        raise ValueError("never reached 63% of the final value - increase Settle")
    tau = float(tp[int(np.argmax(crossed))] - t0)
    if tau <= 0:
        raise ValueError("tau came out as zero - sample period too slow for this motor?")

    K = dy / (d1 - d0)                       # RPM per % duty
    lam = lam_ratio * tau                    # desired closed-loop time constant
    kp = tau / (K * lam)                     # % per RPM
    ki = 1.0 / (K * lam)                     # % per (RPM*s)
    return dict(t=t, y=y, t0=t0, y0=y0, y1=y1, tau=tau, K=K, lam=lam, kp=kp, ki=ki)


def save_csv(prefix, header, rows):
    name = f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return name


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class App:
    def __init__(self, link):
        self.link = link
        self.results = queue.Queue()
        self.abort = threading.Event()
        self.worker = None
        self.synced = False
        self.widgets = []

        self.fig = plt.figure("Turntable tuner", figsize=(13, 8))
        self.ax_rpm = self.fig.add_axes([0.06, 0.55, 0.62, 0.40])
        self.ax_duty = self.fig.add_axes([0.06, 0.10, 0.62, 0.38], sharex=self.ax_rpm)
        (self.l_rpm,) = self.ax_rpm.plot([], [], lw=1.2, label="RPM")
        (self.l_tgt,) = self.ax_rpm.plot([], [], "--", lw=1, label="target (PI)")
        (self.l_duty,) = self.ax_duty.plot([], [], lw=1.2, color="tab:orange")
        self.ax_rpm.set_ylabel("RPM")
        self.ax_rpm.legend(loc="upper right")
        self.ax_rpm.grid(alpha=0.3)
        self.ax_duty.set_ylabel("duty %")
        self.ax_duty.set_xlabel("time (s)")
        self.ax_duty.set_ylim(-2, 102)
        self.ax_duty.grid(alpha=0.3)
        self.stats = self.ax_rpm.text(0.01, 0.97, "", transform=self.ax_rpm.transAxes,
                                      va="top", family="monospace", fontsize=9)
        self.status = self.fig.text(0.06, 0.02, "connected", fontsize=9)

        self.boxes = {}
        self._box("target", "Target RPM", f"{100 / 3:.3f}", 0.92)
        self._box("kp", "KP", "0.5", 0.87)
        self._box("ki", "KI", "0.15", 0.82)
        self._box("ff", "FF duty %", "67", 0.77)
        self._button("Apply gains", self.on_apply, [0.74, 0.715, 0.23, 0.045])
        self._button("PI on", lambda e: self.link.send("pi"), [0.74, 0.655, 0.11, 0.045])
        self._button("Stop", self.on_stop, [0.86, 0.655, 0.11, 0.045])
        self._box("duty", "Duty %", "60", 0.59)
        self._button("Set duty (open loop)", self.on_duty, [0.74, 0.535, 0.23, 0.045])
        self._box("sweep", "Sweep a,b,step", "0,100,5", 0.445)
        self._box("settle", "Settle s", "3", 0.395)
        self._button("Run sweep", self.on_sweep, [0.74, 0.34, 0.23, 0.045])
        self._box("step", "Step from,to", "55,70", 0.28)
        self._box("lam", "lambda / tau", "0.5", 0.23)
        self._button("Run step test", self.on_step, [0.74, 0.175, 0.23, 0.045])
        self._button("Save log CSV", self.on_save, [0.74, 0.10, 0.23, 0.045])

        self.anim = FuncAnimation(self.fig, self.update, interval=100,
                                  cache_frame_data=False)

    # --- widget helpers -----------------------------------------------------
    def _box(self, key, label, initial, y):
        ax = self.fig.add_axes([0.84, y, 0.13, 0.04])
        tb = TextBox(ax, label + " ", initial=initial)
        self.boxes[key] = tb
        self.widgets.append(tb)

    def _button(self, label, cb, rect):
        b = Button(self.fig.add_axes(rect), label)
        b.on_clicked(cb)
        self.widgets.append(b)

    def val(self, key):
        return [float(x) for x in self.boxes[key].text.split(",")]

    def set_status(self, msg):
        self.status.set_text(msg)
        print(msg)

    def busy(self):
        if self.worker and self.worker.is_alive():
            self.set_status("a test is already running (Stop aborts it)")
            return True
        return False

    def start_worker(self, fn, *args):
        self.abort.clear()
        self.worker = threading.Thread(target=fn, args=(*args, self.results, self.abort),
                                       daemon=True)
        self.worker.start()

    # --- callbacks ------------------------------------------------------------
    def on_apply(self, _):
        try:
            for key in ("target", "kp", "ki", "ff"):
                self.link.send(f"{key} {self.val(key)[0]:.5f}")
        except ValueError:
            self.set_status("could not parse one of the gain boxes")

    def on_stop(self, _):
        self.abort.set()
        self.link.send("stop")

    def on_duty(self, _):
        try:
            self.link.send(f"duty {self.val('duty')[0]:.2f}")
        except ValueError:
            self.set_status("bad duty value")

    def on_sweep(self, _):
        if self.busy():
            return
        try:
            a, b, s = self.val("sweep")
            settle = self.val("settle")[0]
        except ValueError:
            self.set_status("sweep box should look like 30,100,5")
            return
        self.start_worker(run_sweep, self.link, a, b, s, settle, 1.0)

    def on_step(self, _):
        if self.busy():
            return
        try:
            d0, d1 = self.val("step")
            settle = self.val("settle")[0]
        except ValueError:
            self.set_status("step box should look like 55,70")
            return
        self.start_worker(run_step, self.link, d0, d1, settle)

    def on_save(self, _):
        name = save_csv("log", COLUMNS, self.link.all_rows())
        self.set_status(f"saved {name}")

    # --- results --------------------------------------------------------------
    def show_sweep(self, points):
        name = save_csv("sweep", ["duty", "rpm_mean", "rpm_std"], points)
        target = self.val("target")[0]
        try:
            res = analyze_sweep(points, target)
        except ValueError as e:
            self.set_status(f"sweep: {e} (data in {name})")
            return
        f, ax = plt.subplots(num=f"Sweep {datetime.now():%H:%M:%S}")
        ax.plot(res["d"], res["r"], "o", label="measured")
        xs = np.linspace(res["deadband"], res["d"].max(), 50)
        ax.plot(xs, res["k_all"] * xs + res["b_all"], "-", alpha=0.6, label="linear fit")
        ax.axhline(target, ls="--", color="gray")
        ax.axvline(res["ff"], ls=":", color="gray")
        ax.set_xlabel("duty %")
        ax.set_ylabel("RPM")
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_title(f"deadband ~{res['deadband']:.0f}%   K near target = {res['k_loc']:.3f} RPM/%   "
                     f"FF = {res['ff']:.1f}%", fontsize=10)
        f.show()
        self.boxes["ff"].set_val(f"{res['ff']:.2f}")
        warn = "" if res["in_range"] else "  (target outside measured range - extrapolated)"
        self.set_status(f"sweep done, FF {res['ff']:.2f}% filled in - press Apply{warn}  [{name}]")

    def show_step(self, payload):
        d0, d1, rows = payload
        name = save_csv("step", COLUMNS, rows)
        try:
            res = analyze_step(d0, d1, rows, self.val("lam")[0])
        except ValueError as e:
            self.set_status(f"step: {e} (data in {name})")
            return
        f, ax = plt.subplots(num=f"Step {datetime.now():%H:%M:%S}")
        ax.plot(res["t"] - res["t0"], res["y"], lw=1, label="measured")
        tm = np.linspace(0, res["t"][-1] - res["t0"], 200)
        ax.plot(tm, res["y0"] + (res["y1"] - res["y0"]) * (1 - np.exp(-tm / res["tau"])),
                "--", label="first-order model")
        ax.axvline(res["tau"], ls=":", color="gray")
        ax.set_xlabel("time since step (s)")
        ax.set_ylabel("RPM")
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_title(f"K = {res['K']:.3f} RPM/%   tau = {res['tau'] * 1000:.0f} ms   "
                     f"lambda = {res['lam'] * 1000:.0f} ms  ->  KP {res['kp']:.3f}, KI {res['ki']:.3f}",
                     fontsize=10)
        f.show()
        self.boxes["kp"].set_val(f"{res['kp']:.4f}")
        self.boxes["ki"].set_val(f"{res['ki']:.4f}")
        self.set_status(f"step done, KP/KI filled in - press Apply, then PI on  [{name}]")

    # --- animation tick -------------------------------------------------------
    def update(self, _):
        while not self.link.messages.empty():
            self.set_status(self.link.messages.get())
        while not self.results.empty():
            kind, data = self.results.get()
            if kind == "status":
                self.set_status(data)
            elif kind == "sweep":
                self.show_sweep(data)
            elif kind == "step":
                self.show_step(data)

        rows = self.link.recent(int(WINDOW_S * 60))
        if not rows:
            return []

        if not self.synced:   # pick up the firmware's current settings once
            _, _, _, _, tgt, kp, ki, ff = rows[-1]
            for key, v in (("target", tgt), ("kp", kp), ("ki", ki), ("ff", ff)):
                self.boxes[key].set_val(f"{v:g}")
            self.synced = True

        t = np.array([r[0] for r in rows])
        mode = np.array([r[1] for r in rows])
        duty = np.array([r[2] for r in rows])
        rpm = np.array([r[3] for r in rows])
        tgt = np.where(mode == "P", np.array([r[4] for r in rows]), np.nan)
        now = t[-1]
        m = t >= now - WINDOW_S

        self.l_rpm.set_data(t[m], rpm[m])
        self.l_tgt.set_data(t[m], tgt[m])
        self.l_duty.set_data(t[m], duty[m])
        self.ax_rpm.set_xlim(now - WINDOW_S, now)
        self.ax_rpm.relim()
        self.ax_rpm.autoscale_view(scalex=False)

        last = rpm[t >= now - 2.0]
        mean, sd = last.mean(), last.std()
        text = f"last 2 s: mean {mean:7.3f} RPM   sd {sd:6.3f}"
        if mean > 0.5:
            text += f" ({100 * sd / mean:.2f}%)"
        if mode[-1] == "P":
            text += f"   error {rows[-1][4] - mean:+.3f}"
        text += f"\nmode {'PI' if mode[-1] == 'P' else 'open loop'}   duty {duty[-1]:.2f}%"
        self.stats.set_text(text)
        return []


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect the Pico)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No Pico found. Plug it in, close other serial monitors, or pass --port.")
    print(f"connecting to {port}")
    link = Link(port)
    App(link)
    try:
        plt.show()
    finally:
        try:
            link.send("stop")
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    main()
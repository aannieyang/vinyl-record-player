"""Live oscilloscope view of the stylus waveform captured by audio-proc.

Usage: python scope.py [serial-port]
Close the VS Code serial monitor first -- only one program can hold the port.

Top: waveform in millivolts with the 1.65 V bias removed, so a few-mV stylus signal is visible.
Bottom: spectrum. Press n with the stylus lifted (or the record stopped) to save the noise floor;
after that the stats show signal-to-noise ratio and the floor is drawn in grey for comparison.
"""
import glob
import sys

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

VREF = 3.3
ADC_MAX = 4096
LSB_MV = VREF / ADC_MAX * 1000  # one ADC step is ~0.8 mV
SMOOTHING = 0.1                 # weight of each new frame in the running stats and spectrum

port = sys.argv[1] if len(sys.argv) > 1 else (glob.glob("/dev/cu.usbmodem*") or [None])[0]
if port is None:
    sys.exit("No Pico found (looked for /dev/cu.usbmodem*). Pass the port as an argument.")
ser = serial.Serial(port, timeout=1)

fig, (wave_ax, spec_ax) = plt.subplots(2, 1, figsize=(9, 7))
(wave_line,) = wave_ax.plot([])
wave_ax.set_xlabel("time (ms)")
wave_ax.set_ylabel("mV (bias removed)")
wave_ax.grid(True)
stats_text = wave_ax.text(0.01, 0.97, "", transform=wave_ax.transAxes, va="top",
                          family="monospace", bbox=dict(facecolor="white", alpha=0.8))

(spec_line,) = spec_ax.semilogx([], [], label="signal")
(noise_line,) = spec_ax.semilogx([], [], color="grey", label="noise floor (press n)")
spec_ax.set_xlabel("frequency (Hz)")
spec_ax.set_ylabel("dB re 1 mV")
spec_ax.legend(loc="upper right")
spec_ax.grid(True, which="both")
fig.tight_layout()

# running averages, so the numbers are readable instead of flickering every frame
state = {"rms": None, "pp": None, "power": None, "freqs": None,
         "noise_rms": None, "ylim": 2 * LSB_MV}


def smooth(old, new):
    return new if old is None else old + SMOOTHING * (new - old)


def read_capture():
    """Return (rate, samples) from the newest complete capture line, or None."""
    latest = None
    while True:
        text = ser.readline().decode(errors="ignore").strip()
        if text.startswith("W:"):
            latest = text
        if latest and ser.in_waiting == 0:
            break
        if not text and latest is None:
            return None
    try:
        _, rate, data = latest.split(":", 2)
        samples = np.array(data.split(","), dtype=float) * VREF / ADC_MAX
    except ValueError:
        return None  # partial line, e.g. right after connecting
    return int(rate), samples


def trigger(signal):
    """Index of the first rising crossing of the mean, so the wave holds still between frames."""
    mean = signal.mean()
    half = len(signal) // 2
    crossings = np.where((signal[:half - 1] < mean) & (signal[1:half] >= mean))[0]
    return crossings[0] if len(crossings) else 0


def to_db(power):
    return 10 * np.log10(power + 1e-12)


def on_key(event):
    if event.key == "n" and state["rms"] is not None:
        state["noise_rms"] = state["rms"]
        noise_line.set_data(state["freqs"][1:], to_db(state["power"][1:]))


def update(_):
    capture = read_capture()
    if capture is None:
        return wave_line, spec_line, noise_line, stats_text
    rate, samples = capture
    mv = (samples - samples.mean()) * 1000

    # waveform: half the capture starting at the trigger, y-axis follows the signal and decays slowly
    start = trigger(mv)
    window = len(mv) // 2
    t = np.arange(window) / rate * 1000
    wave_line.set_data(t, mv[start:start + window])
    wave_ax.set_xlim(0, t[-1])
    state["ylim"] = max(np.abs(mv).max() * 1.2, 2 * LSB_MV, state["ylim"] * 0.95)
    wave_ax.set_ylim(-state["ylim"], state["ylim"])

    # stats
    state["rms"] = smooth(state["rms"], np.sqrt(np.mean(mv ** 2)))
    state["pp"] = smooth(state["pp"], mv.max() - mv.min())
    lines = [
        f"p-p  {state['pp']:6.2f} mV  ({state['pp'] / LSB_MV:5.1f} ADC steps)",
        f"RMS  {state['rms']:6.2f} mV  ({state['rms'] / LSB_MV:5.1f} ADC steps)",
    ]
    if state["noise_rms"]:
        snr = 20 * np.log10(state["rms"] / state["noise_rms"])
        lines.append(f"SNR  {snr:6.1f} dB  (noise {state['noise_rms']:.2f} mV RMS)")
    else:
        lines.append("SNR  lift the stylus and press n")
    stats_text.set_text("\n".join(lines))

    # spectrum: Hann-windowed FFT, amplitude in mV, power averaged across frames
    w = np.hanning(len(mv))
    amplitude = np.abs(np.fft.rfft(mv * w)) / w.sum() * 2
    state["freqs"] = np.fft.rfftfreq(len(mv), 1 / rate)
    state["power"] = smooth(state["power"], amplitude ** 2)
    spec_line.set_data(state["freqs"][1:], to_db(state["power"][1:]))  # skip the DC bin
    spec_ax.set_xlim(state["freqs"][1], rate / 2)
    spec_ax.relim()
    spec_ax.autoscale_view(scalex=False)

    return wave_line, spec_line, noise_line, stats_text


fig.canvas.mpl_connect("key_press_event", on_key)
anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()

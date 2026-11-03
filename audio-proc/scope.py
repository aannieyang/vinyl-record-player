"""Live oscilloscope view of the stylus waveform captured by audio-proc.

Usage: python scope.py [serial-port]
Close the VS Code serial monitor first -- only one program can hold the port.
"""
import glob
import sys

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

VREF = 3.3
ADC_MAX = 4096

port = sys.argv[1] if len(sys.argv) > 1 else (glob.glob("/dev/cu.usbmodem*") or [None])[0]
if port is None:
    sys.exit("No Pico found (looked for /dev/cu.usbmodem*). Pass the port as an argument.")
ser = serial.Serial(port, timeout=1)

fig, ax = plt.subplots()
(line,) = ax.plot([])
ax.set_xlabel("time (ms)")
ax.set_ylabel("volts")
ax.set_ylim(0, VREF)
ax.grid(True)


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


def update(_):
    capture = read_capture()
    if capture is None:
        return (line,)
    rate, samples = capture
    start = trigger(samples)
    window = len(samples) // 2
    t = np.arange(window) / rate * 1000
    line.set_data(t, samples[start:start + window])
    ax.set_xlim(0, t[-1])
    return (line,)


anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.show()

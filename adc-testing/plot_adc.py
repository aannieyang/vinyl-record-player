import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

if len(sys.argv) != 2:
    print("Usage: python plot_adc.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.2)

FS = 46875
N = 256

t = np.arange(N) / FS * 1000

fig, ax = plt.subplots()
line, = ax.plot(t, np.zeros(N))

ax.set_ylim(-0.4, 0.4)
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage (V)")
ax.set_title("PCM1808 Live Audio")
ax.grid(True)

def update(frame):
    try:
        raw = ser.readline().decode().strip()
        samples = np.array([int(x) for x in raw.split(",")])

        if len(samples) == N:
            voltage = samples * 1.5 / 8388608
            line.set_ydata(voltage)

    except ValueError:
        pass

    return line,

ani = FuncAnimation(
    fig,
    update,
    interval=30,
    blit=True,
    cache_frame_data=False
)

plt.show()
ser.close()

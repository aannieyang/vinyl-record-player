
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import math
from collections import deque

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class MotorGUI:

    def __init__(self, root):
        self.root = root
        self.root.title("Record Player Motor Controller")
        self.root.geometry("1100x750")

        # Serial communication
        self.ser = None
        self.running = False
        self.reader_thread = None
        self.data_queue = queue.Queue()

        # Plot data
        self.times = deque(maxlen=6000)
        self.rpms = deque(maxlen=6000)
        self.targets = deque(maxlen=6000)
        self.first_timestamp = None

        # GUI variables
        self.kp_var = tk.StringVar(value="0.5")
        self.ki_var = tk.StringVar(value="0.0")

        self.base_rpm_var = tk.StringVar(value="33.333")
        self.step_size_var = tk.StringVar(value="2.0")
        self.target_rpm_var = tk.StringVar(value="33.333")

        self.rpm_display = tk.StringVar(value="--- RPM")
        self.target_display = tk.StringVar(value="--- RPM")
        self.pwm_display = tk.StringVar(value="--- %")
        self.kp_display = tk.StringVar(value="---")
        self.ki_display = tk.StringVar(value="---")

        self.status_var = tk.StringVar(value="Disconnected")

        self.port_var = tk.StringVar()

        self.build_gui()
        self.refresh_ports()

        self.root.after(100, self.update_gui)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # --------------------------------------------------
    # GUI layout
    # --------------------------------------------------

    def build_gui(self):

        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # Connection controls
        connection = ttk.LabelFrame(
            main, text="USB Connection", padding=10
        )
        connection.pack(fill=tk.X, pady=5)

        ttk.Label(connection, text="Serial port:").pack(side=tk.LEFT)

        self.port_box = ttk.Combobox(
            connection,
            textvariable=self.port_var,
            width=35,
            state="readonly"
        )
        self.port_box.pack(side=tk.LEFT, padx=5)

        ttk.Button(
            connection,
            text="Refresh",
            command=self.refresh_ports
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            connection,
            text="Connect",
            command=self.connect
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            connection,
            text="Disconnect",
            command=self.disconnect
        ).pack(side=tk.LEFT, padx=5)

        ttk.Label(
            connection,
            textvariable=self.status_var
        ).pack(side=tk.RIGHT, padx=10)

        # Main control panel
        controls = ttk.LabelFrame(
            main, text="Motor Controller Settings", padding=10
        )
        controls.pack(fill=tk.X, pady=5)

        # Gain controls
        gains = ttk.Frame(controls)
        gains.pack(fill=tk.X, pady=5)

        ttk.Label(gains, text="Proportional gain KP:").grid(
            row=0, column=0, sticky="w", padx=5, pady=5
        )

        ttk.Entry(
            gains, textvariable=self.kp_var, width=12
        ).grid(row=0, column=1, padx=5)

        ttk.Label(gains, text="Integral gain KI:").grid(
            row=0, column=2, sticky="w", padx=5
        )

        ttk.Entry(
            gains, textvariable=self.ki_var, width=12
        ).grid(row=0, column=3, padx=5)

        ttk.Button(
            gains,
            text="Apply Gains",
            command=self.apply_gains
        ).grid(row=0, column=4, padx=15)

        # Setpoint controls
        setpoints = ttk.Frame(controls)
        setpoints.pack(fill=tk.X, pady=10)

        ttk.Label(
            setpoints, text="Base setpoint (RPM):"
        ).grid(row=0, column=0, padx=5, pady=5)

        ttk.Entry(
            setpoints,
            textvariable=self.base_rpm_var,
            width=12
        ).grid(row=0, column=1, padx=5)

        ttk.Button(
            setpoints,
            text="Set Base",
            command=self.set_base
        ).grid(row=0, column=2, padx=5)

        ttk.Label(
            setpoints, text="Step size (RPM):"
        ).grid(row=0, column=3, padx=5)

        ttk.Entry(
            setpoints,
            textvariable=self.step_size_var,
            width=12
        ).grid(row=0, column=4, padx=5)

        ttk.Label(
            setpoints, text="Target RPM:"
        ).grid(row=1, column=0, padx=5, pady=10)

        ttk.Entry(
            setpoints,
            textvariable=self.target_rpm_var,
            width=12
        ).grid(row=1, column=1, padx=5)

        ttk.Button(
            setpoints,
            text="Apply Target",
            command=self.apply_target
        ).grid(row=1, column=2, padx=5)

        ttk.Button(
            setpoints,
            text="Step Below",
            command=lambda: self.step_target(-1)
        ).grid(row=1, column=3, padx=5)

        ttk.Button(
            setpoints,
            text="Step Above",
            command=lambda: self.step_target(1)
        ).grid(row=1, column=4, padx=5)

        ttk.Button(
            setpoints,
            text="Reset to Base",
            command=self.reset_target
        ).grid(row=1, column=5, padx=10)

        # Live readouts
        readouts = ttk.LabelFrame(
            main, text="Live Motor Measurements", padding=10
        )
        readouts.pack(fill=tk.X, pady=5)

        self.add_readout(readouts, "Measured RPM", self.rpm_display, 0)
        self.add_readout(readouts, "Target RPM", self.target_display, 1)
        self.add_readout(readouts, "PWM Duty", self.pwm_display, 2)
        self.add_readout(readouts, "KP", self.kp_display, 3)
        self.add_readout(readouts, "KI", self.ki_display, 4)

        # RPM graph
        graph_frame = ttk.LabelFrame(
            main, text="Motor Speed vs Time", padding=5
        )
        graph_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.figure = Figure(figsize=(9, 4.5), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("RPM")
        self.ax.set_title("Motor Speed Response")
        self.ax.grid(True)

        self.rpm_line, = self.ax.plot(
            [], [], label="Measured RPM"
        )

        self.target_line, = self.ax.plot(
            [], [], linestyle="--", label="Target RPM"
        )

        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(
            self.figure, master=graph_frame
        )

        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH, expand=True
        )

    # --------------------------------------------------
    # Readout helper
    # --------------------------------------------------

    def add_readout(self, parent, label, variable, column):

        frame = ttk.Frame(parent, padding=10)
        frame.grid(row=0, column=column, padx=10)

        ttk.Label(
            frame, text=label
        ).pack()

        ttk.Label(
            frame,
            textvariable=variable,
            font=("Arial", 16, "bold")
        ).pack()

    # --------------------------------------------------
    # Serial port selection
    # --------------------------------------------------

    def refresh_ports(self):

        ports = serial.tools.list_ports.comports()
        port_names = [port.device for port in ports]

        self.port_box["values"] = port_names

        if port_names:
            if self.port_var.get() not in port_names:
                self.port_var.set(port_names[0])
        else:
            self.port_var.set("")

    # --------------------------------------------------
    # Connect to Pico
    # --------------------------------------------------

    def connect(self):

        if self.ser is not None:
            return

        port = self.port_var.get()

        if not port:
            messagebox.showerror(
                "Connection Error",
                "Select a serial port first."
            )
            return

        try:
            self.ser = serial.Serial(
                port,
                115200,
                timeout=0.2
            )

            self.running = True

            # Start serial-reading thread.
            self.reader_thread = threading.Thread(
                target=self.serial_reader,
                daemon=True
            )
            self.reader_thread.start()

            self.status_var.set("Connected to " + port)

            # Clear old plot data.
            self.times.clear()
            self.rpms.clear()
            self.targets.clear()
            self.first_timestamp = None

            # Send current GUI settings.
            self.root.after(500, self.apply_all)

        except Exception as e:
            self.ser = None
            messagebox.showerror(
                "Connection Error", str(e)
            )

    # --------------------------------------------------
    # Disconnect
    # --------------------------------------------------

    def disconnect(self):

        self.running = False

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.status_var.set("Disconnected")

    # --------------------------------------------------
    # Background serial reader
    # --------------------------------------------------

    def serial_reader(self):

        connection = self.ser

        while self.running and connection is not None:

            try:
                raw = connection.readline()

                if raw:
                    line = raw.decode(
                        "utf-8", errors="ignore"
                    ).strip()

                    if line:
                        self.data_queue.put(line)

            except Exception as e:
                self.data_queue.put("ERROR," + str(e))
                break

    # --------------------------------------------------
    # Send command to Pico
    # --------------------------------------------------

    def send_command(self, command):

        if self.ser is None:
            self.status_var.set("Not connected")
            return False

        try:
            self.ser.write(
                (command + "\n").encode("utf-8")
            )
            return True

        except Exception as e:
            self.status_var.set("Serial error: " + str(e))
            return False

    # --------------------------------------------------
    # Numeric input validation
    # --------------------------------------------------

    def read_number(self, variable, label, minimum=None):

        try:
            value = float(variable.get())

            if not math.isfinite(value):
                raise ValueError

            if minimum is not None and value < minimum:
                raise ValueError

            return value

        except ValueError:
            messagebox.showerror(
                "Invalid Input",
                f"{label} must be a valid number"
                + (f" greater than or equal to {minimum}."
                   if minimum is not None else ".")
            )
            return None

    # --------------------------------------------------
    # Apply controller gains
    # --------------------------------------------------

    def apply_gains(self):

        kp = self.read_number(
            self.kp_var, "KP", 0.0
        )

        ki = self.read_number(
            self.ki_var, "KI", 0.0
        )

        if kp is None or ki is None:
            return

        self.send_command(f"KP {kp:.10g}")
        self.send_command(f"KI {ki:.10g}")

    # --------------------------------------------------
    # Apply all settings after connecting
    # --------------------------------------------------

    def apply_all(self):

        self.apply_gains()
        self.apply_target()

    # --------------------------------------------------
    # Set base setpoint
    # --------------------------------------------------

    def set_base(self):

        base = self.read_number(
            self.base_rpm_var, "Base setpoint", 0.0
        )

        if base is None or base > 300:
            messagebox.showerror(
                "Invalid Input",
                "Base setpoint must be between 0 and 300 RPM."
            )
            return

        self.target_rpm_var.set(str(base))
        self.send_command(f"SETPOINT {base:.10g}")

    # --------------------------------------------------
    # Set target directly
    # --------------------------------------------------

    def apply_target(self):

        target = self.read_number(
            self.target_rpm_var, "Target RPM", 0.0
        )

        if target is None or target > 300:
            messagebox.showerror(
                "Invalid Input",
                "Target RPM must be between 0 and 300."
            )
            return

        self.send_command(f"SETPOINT {target:.10g}")

    # --------------------------------------------------
    # Step above or below base setpoint
    # --------------------------------------------------

    def step_target(self, direction):

        base = self.read_number(
            self.base_rpm_var, "Base setpoint", 0.0
        )

        step = self.read_number(
            self.step_size_var, "Step size", 0.0
        )

        if base is None or step is None:
            return

        target = base + direction * step

        if target < 0 or target > 300:
            messagebox.showerror(
                "Invalid Step",
                "The resulting target must be between 0 and 300 RPM."
            )
            return

        self.target_rpm_var.set(f"{target:.6g}")
        self.send_command(f"SETPOINT {target:.10g}")

    # --------------------------------------------------
    # Reset to base setpoint
    # --------------------------------------------------

    def reset_target(self):

        base = self.read_number(
            self.base_rpm_var, "Base setpoint", 0.0
        )

        if base is None:
            return

        self.target_rpm_var.set(f"{base:.6g}")
        self.send_command(f"SETPOINT {base:.10g}")

    # --------------------------------------------------
    # Process incoming telemetry
    # --------------------------------------------------

    def process_line(self, line):

        if line.startswith("ERROR,"):
            self.status_var.set(line[6:])
            return

        if not line.startswith("DATA,"):
            return

        try:
            parts = line.split(",")

            if len(parts) != 7:
                return

            timestamp = float(parts[1])
            rpm = float(parts[2])
            target = float(parts[3])
            pwm = float(parts[4])
            kp = float(parts[5])
            ki = float(parts[6])

            if self.first_timestamp is None:
                self.first_timestamp = timestamp

            plot_time = timestamp - self.first_timestamp

            self.times.append(plot_time)
            self.rpms.append(rpm)
            self.targets.append(target)

            self.rpm_display.set(f"{rpm:.2f} RPM")
            self.target_display.set(f"{target:.2f} RPM")
            self.pwm_display.set(f"{pwm:.1f} %")
            self.kp_display.set(f"{kp:.4f}")
            self.ki_display.set(f"{ki:.4f}")

        except (ValueError, IndexError):
            return

    # --------------------------------------------------
    # Update graph and GUI
    # --------------------------------------------------

    def update_gui(self):

        # Drain all queued serial messages.
        while True:
            try:
                line = self.data_queue.get_nowait()
                self.process_line(line)
            except queue.Empty:
                break

        # Update graph.
        if len(self.times) > 0:

            self.rpm_line.set_data(
                list(self.times),
                list(self.rpms)
            )

            self.target_line.set_data(
                list(self.times),
                list(self.targets)
            )

            self.ax.relim()
            self.ax.autoscale_view()

            self.canvas.draw_idle()

        self.root.after(100, self.update_gui)

    # --------------------------------------------------
    # Close application
    # --------------------------------------------------

    def close(self):

        self.disconnect()
        self.root.destroy()


# ------------------------------------------------------
# Run application
# ------------------------------------------------------

if __name__ == "__main__":

    root = tk.Tk()
    app = MotorGUI(root)

    root.mainloop()

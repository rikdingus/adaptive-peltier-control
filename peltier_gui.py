#!/usr/bin/env python3
"""
Adaptive Junior V2 — Desktop Control App (Tkinter + matplotlib)

A windowed GUI alternative to the terminal UI in peltier_control.py. It reuses
the same serial controller, simulator, polling thread and CSV logger, and adds
a live embedded temperature plot plus point-and-click controls.

Requirements:
    pip install pyserial matplotlib --break-system-packages
    # Tkinter ships with CPython; on Debian/Ubuntu: sudo apt install python3-tk

Usage:
    python3 peltier_gui.py [--port /dev/ttyUSB0]
    python3 peltier_gui.py --simulate          # no hardware
    python3 peltier_gui.py --logfile run.csv    # choose log path
    python3 peltier_gui.py --no-log
"""

import argparse
import sys
import threading
import time
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk, messagebox

import serial

import peltier_control as pc
from peltier_control import (
    CONTROL_MODES, OUTPUT_MODES, State, CsvLogger, ControllerError,
    JuniorController, SimulatedController, poll_loop, export_png,
    REG_CTRL_MODE, REG_OUTPUT_MODE, REG_SETPOINT, REG_P, REG_I, REG_D,
    REG_CTRL_SENSOR, REG_SENSOR_EN,
)


SENSOR_CHOICES = {"TC1 — hot (sensor E)": 4, "TC2 — cold (sensor F)": 5}


def status_text(status):
    if status is None:
        return "—", "gray"
    flags = []
    if status & (1 << 0):  flags.append("SHUTDOWN")
    if status & (1 << 1):  flags.append("RELAY")
    if status & (1 << 6):  flags.append("HEAT/COOL")
    if status & (1 << 11): flags.append("TUNE OK")
    if status & (1 << 12): flags.append("TUNE FAIL")
    if status & (1 << 13): flags.append("FAULT")
    if not flags:
        return "OK", "#2e7d32"
    bad = status & ((1 << 0) | (1 << 12) | (1 << 13))
    return " ".join(flags), ("#c62828" if bad else "#f9a825")


class PeltierApp:
    def __init__(self, root: tk.Tk, ctrl, state: State, stop_event, logger):
        self.root = root
        self.ctrl = ctrl
        self.state = state
        self.stop_event = stop_event
        self.logger = logger

        root.title("Adaptive Junior V2 — Peltier Control")
        root.geometry("1080x680")
        root.minsize(900, 560)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.readout_vars: dict[str, tk.StringVar] = {}

        outer = ttk.Frame(root, padding=8)
        outer.pack(fill="both", expand=True)
        left = ttk.Frame(outer)
        left.pack(side="left", fill="y", padx=(0, 8))
        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True)

        self._build_readouts(left)
        self._build_controls(left)
        self._build_plot(right)
        self._build_statusbar(root)

        self._refresh_ui()  # kick off the 1 Hz UI loop

    # ── UI construction ──────────────────────────────────────────────
    def _build_readouts(self, parent):
        box = ttk.LabelFrame(parent, text="Live readings", padding=8)
        box.pack(fill="x")
        rows = [
            ("TC1 — hot side", "tc1"), ("TC2 — cold side", "tc2"),
            ("ΔT (hot − cold)", "dt"), ("NTC A (spare)", "ntc_a"),
            ("NTC B (spare)", "ntc_b"), ("PWM output", "pwm"),
            ("TEC voltage", "tec_v"), ("TEC current", "tec_i"),
            ("Supply voltage", "supply"), ("Status", "status"),
        ]
        for i, (label, key) in enumerate(rows):
            ttk.Label(box, text=label).grid(row=i, column=0, sticky="w", pady=1)
            var = tk.StringVar(value="—")
            self.readout_vars[key] = var
            lbl = ttk.Label(box, textvariable=var, font=("TkDefaultFont", 10, "bold"))
            lbl.grid(row=i, column=1, sticky="e", padx=(16, 0))
            if key == "status":
                self.status_label = lbl
        box.columnconfigure(1, weight=1)

    def _build_controls(self, parent):
        box = ttk.LabelFrame(parent, text="Controls", padding=8)
        box.pack(fill="x", pady=(8, 0))

        # Setpoint
        ttk.Label(box, text="Setpoint (°C)").grid(row=0, column=0, sticky="w")
        self.setpoint_entry = ttk.Entry(box, width=10)
        self.setpoint_entry.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(box, text="Set", command=self.apply_setpoint).grid(row=0, column=2)

        # Control mode
        ttk.Label(box, text="Control mode").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.mode_var = tk.StringVar()
        self.mode_combo = ttk.Combobox(box, textvariable=self.mode_var, state="readonly",
                                       width=14, values=list(CONTROL_MODES.values()))
        self.mode_combo.grid(row=1, column=1, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(box, text="Apply", command=self.apply_mode).grid(row=1, column=2, pady=(6, 0))

        # Output mode
        ttk.Label(box, text="Output mode").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.out_var = tk.StringVar()
        self.out_combo = ttk.Combobox(box, textvariable=self.out_var, state="readonly",
                                      width=14, values=list(OUTPUT_MODES.values()))
        self.out_combo.grid(row=2, column=1, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(box, text="Apply", command=self.apply_output_mode).grid(row=2, column=2, pady=(6, 0))

        # Control sensor
        ttk.Label(box, text="Control sensor").grid(row=3, column=0, sticky="w", pady=(6, 0))
        self.sensor_var = tk.StringVar()
        self.sensor_combo = ttk.Combobox(box, textvariable=self.sensor_var, state="readonly",
                                         width=14, values=list(SENSOR_CHOICES.keys()))
        self.sensor_combo.grid(row=3, column=1, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(box, text="Apply", command=self.apply_sensor).grid(row=3, column=2, pady=(6, 0))

        # PID gains
        ttk.Label(box, text="P / I / D gains").grid(row=4, column=0, sticky="w", pady=(6, 0))
        gains = ttk.Frame(box)
        gains.grid(row=4, column=1, columnspan=2, sticky="ew", pady=(6, 0))
        self.p_entry = ttk.Entry(gains, width=6); self.p_entry.pack(side="left", padx=1)
        self.i_entry = ttk.Entry(gains, width=6); self.i_entry.pack(side="left", padx=1)
        self.d_entry = ttk.Entry(gains, width=6); self.d_entry.pack(side="left", padx=1)
        ttk.Button(gains, text="Set", command=self.apply_gains).pack(side="left", padx=2)

        # Action buttons
        actions = ttk.Frame(box)
        actions.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        self.run_btn = tk.Button(actions, text="▶ RUN", bg="#2e7d32", fg="white",
                                 activebackground="#1b5e20", width=8, command=self.do_run)
        self.run_btn.pack(side="left", padx=2)
        self.stop_btn = tk.Button(actions, text="■ STOP", bg="#c62828", fg="white",
                                  activebackground="#8e0000", width=8, command=self.do_stop)
        self.stop_btn.pack(side="left", padx=2)
        ttk.Button(actions, text="Enable TC1+TC2", command=self.enable_sensors).pack(side="left", padx=2)

        ttk.Button(box, text="Export graph PNG", command=self.export_graph).grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        box.columnconfigure(1, weight=1)

    def _build_plot(self, parent):
        self.fig = Figure(figsize=(7, 5), dpi=100)
        self.ax = self.fig.add_subplot(211)
        self.ax_dt = self.fig.add_subplot(212, sharex=self.ax)
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.grid(True, alpha=0.3)
        self.ax_dt.set_ylabel("ΔT (°C)")
        self.ax_dt.set_xlabel("Elapsed (s)")
        self.ax_dt.grid(True, alpha=0.3)
        (self.line_tc1,) = self.ax.plot([], [], color="tab:red", label="TC1 hot")
        (self.line_tc2,) = self.ax.plot([], [], color="tab:blue", label="TC2 cold")
        (self.line_sp,) = self.ax.plot([], [], color="tab:green", linestyle="--",
                                       linewidth=1, label="setpoint")
        (self.line_dt,) = self.ax_dt.plot([], [], color="tab:purple", label="ΔT")
        self.ax.legend(loc="upper right", fontsize=8)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_statusbar(self, root):
        self.statusbar = tk.StringVar(value="Starting…")
        bar = ttk.Frame(root, relief="sunken", padding=(6, 2))
        bar.pack(side="bottom", fill="x")
        ttk.Label(bar, textvariable=self.statusbar).pack(side="left")

    # ── Control actions ──────────────────────────────────────────────
    def _write(self, reg, value, desc):
        try:
            self.ctrl.write_reg(reg, value)
            self.flash(f"{desc} → {value}")
        except ControllerError as e:
            self.flash(f"{desc} failed: {e}", error=True)

    def _read_float_entry(self, entry):
        try:
            return float(entry.get())
        except (ValueError, TypeError):
            return None

    def apply_setpoint(self):
        v = self._read_float_entry(self.setpoint_entry)
        if v is None:
            self.flash("Invalid setpoint", error=True)
        else:
            self._write(REG_SETPOINT, v, "Setpoint")

    def apply_mode(self):
        name = self.mode_var.get()
        for num, label in CONTROL_MODES.items():
            if label == name:
                self._write(REG_CTRL_MODE, num, "Mode")
                return

    def apply_output_mode(self):
        name = self.out_var.get()
        for num, label in OUTPUT_MODES.items():
            if label == name:
                self._write(REG_OUTPUT_MODE, num, "Output mode")
                return

    def apply_sensor(self):
        num = SENSOR_CHOICES.get(self.sensor_var.get())
        if num is not None:
            self._write(REG_CTRL_SENSOR, num, "Control sensor")

    def apply_gains(self):
        for entry, reg, name in ((self.p_entry, REG_P, "P"),
                                 (self.i_entry, REG_I, "I"),
                                 (self.d_entry, REG_D, "D")):
            txt = entry.get().strip()
            if txt == "":
                continue
            v = self._read_float_entry(entry)
            if v is None:
                self.flash(f"Invalid {name} gain", error=True)
            else:
                self._write(reg, v, f"{name} gain")

    def enable_sensors(self):
        self._write(REG_SENSOR_EN, 48, "Sensor enable")

    def do_run(self):
        try:
            self.ctrl.run(); self.flash("RUN")
        except ControllerError as e:
            self.flash(f"RUN failed: {e}", error=True)

    def do_stop(self):
        try:
            self.ctrl.stop(); self.flash("STOP")
        except ControllerError as e:
            self.flash(f"STOP failed: {e}", error=True)

    def export_graph(self):
        if self.logger is None:
            self.flash("No CSV log active to plot from", error=True)
            return
        try:
            png = export_png(self.logger.path)
            self.flash(f"Saved {png}")
            messagebox.showinfo("Graph exported", f"Saved chart to:\n{png}")
        except (ControllerError, OSError) as e:
            self.flash(f"Export failed: {e}", error=True)

    def flash(self, msg, error=False):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.statusbar.set(f"[{stamp}] {msg}")

    # ── Periodic UI refresh ──────────────────────────────────────────
    def _refresh_ui(self):
        s = self.state.snapshot()
        rv = self.readout_vars

        def t(v):
            return f"{v:+.1f} °C" if v is not None else "—"

        rv["tc1"].set(t(s.tc1_temp))
        rv["tc2"].set(t(s.tc2_temp))
        if s.tc1_temp is not None and s.tc2_temp is not None:
            rv["dt"].set(f"{s.tc1_temp - s.tc2_temp:+.1f} °C")
        else:
            rv["dt"].set("—")
        rv["ntc_a"].set(t(s.ntc_a))
        rv["ntc_b"].set(t(s.ntc_b))
        rv["pwm"].set(f"{s.pwm_out} %" if s.pwm_out is not None else "—")
        rv["tec_v"].set(f"{s.bridge_v:.2f} V" if s.bridge_v is not None else "—")
        rv["tec_i"].set(f"{s.bridge_i:.2f} A" if s.bridge_i is not None else "—")
        rv["supply"].set(f"{s.supply_v:.2f} V" if s.supply_v is not None else "—")
        txt, color = status_text(s.status)
        rv["status"].set(txt)
        self.status_label.configure(foreground=color)

        # Sync combo boxes to the board's reported state (only if user not editing)
        if s.ctrl_mode in CONTROL_MODES and not self.mode_combo.focus_get() == self.mode_combo:
            self.mode_var.set(CONTROL_MODES[s.ctrl_mode])
        if s.out_mode in OUTPUT_MODES and self.out_var.get() == "":
            self.out_var.set(OUTPUT_MODES[s.out_mode])
        if self.sensor_var.get() == "":
            for label, num in SENSOR_CHOICES.items():
                if num == s.ctrl_sensor:
                    self.sensor_var.set(label)
        if self.setpoint_entry.get() == "" and s.setpoint is not None:
            self.setpoint_entry.insert(0, f"{s.setpoint:.1f}")

        self._update_plot(s)

        conn = "ERROR: " + s.error if s.error else "connected"
        log = f"  •  log: {self.logger.path} ({self.logger.rows} rows)" if self.logger else "  •  logging off"
        cur = self.statusbar.get()
        if cur.endswith("Starting…") or "  •  " in cur or cur == "Starting…":
            self.statusbar.set(f"Updated {s.last_update}  •  {conn}{log}")

        if not self.stop_event.is_set():
            self.root.after(1000, self._refresh_ui)

    def _update_plot(self, s):
        hist = [h for h in s.history if h[1] is not None or h[2] is not None]
        if len(hist) < 2:
            return
        t = [h[0] for h in hist]
        tc1 = [h[1] for h in hist]
        tc2 = [h[2] for h in hist]
        sp = [h[3] for h in hist]
        dt = [(a - b) if (a is not None and b is not None) else None
              for a, b in zip(tc1, tc2)]
        self.line_tc1.set_data(t, tc1)
        self.line_tc2.set_data(t, tc2)
        self.line_sp.set_data(t, sp)
        self.line_dt.set_data(t, dt)
        for ax in (self.ax, self.ax_dt):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    # ── Shutdown ─────────────────────────────────────────────────────
    def on_close(self):
        self.stop_event.set()
        time.sleep(0.05)
        if self.logger:
            self.logger.close()
        try:
            self.ctrl.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Adaptive Junior V2 — desktop control app")
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect)")
    parser.add_argument("--simulate", action="store_true", help="Use the simulated board")
    parser.add_argument("--logfile", default=None, help="CSV log path")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV logging")
    args = parser.parse_args()

    if args.simulate:
        ctrl = SimulatedController()
    else:
        port = args.port or pc.find_port()
        try:
            ctrl = JuniorController(port)
        except serial.SerialException as e:
            root = tk.Tk(); root.withdraw()
            messagebox.showerror(
                "Connection failed",
                f"Cannot open {port}:\n{e}\n\nRun with --simulate to use a fake board.")
            sys.exit(1)

    logger = None
    if not args.no_log:
        path = args.logfile or f"peltier_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        try:
            logger = CsvLogger(path)
        except OSError as e:
            print(f"Could not open log file {path}: {e}", file=sys.stderr)

    state = State()
    stop_event = threading.Event()
    threading.Thread(target=poll_loop, args=(ctrl, state, stop_event, logger),
                     daemon=True).start()

    root = tk.Tk()
    PeltierApp(root, ctrl, state, stop_event, logger)
    root.mainloop()


if __name__ == "__main__":
    main()

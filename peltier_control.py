#!/usr/bin/env python3
"""
Adaptive Junior V2 — Linux Terminal Control UI
Controls TEC1-12706 via ADJ-48-450-UR-V2 over USB serial.

Requirements:
    pip install pyserial rich --break-system-packages
    # optional, for graphs:
    pip install plotext matplotlib --break-system-packages

Usage:
    python3 peltier_control.py [--port /dev/ttyUSB0]
    python3 peltier_control.py --simulate          # no hardware
    python3 peltier_control.py --logfile run.csv   # choose log path
    python3 peltier_control.py --no-graph          # disable live graph

All sensor channels are logged to a timestamped CSV every second. Press 'g'
in the menu to export a PNG chart, or call export_png(csv_path) directly.
"""

import argparse
import csv
import os
import sys
import time
import threading
from collections import deque
from datetime import datetime

import serial

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.text import Text
    from rich import box
except ImportError:
    print("Missing dependency. Install with:")
    print("  pip install pyserial rich --break-system-packages")
    sys.exit(1)


# ── Serial helpers ────────────────────────────────────────────────────────────

BAUD = 115200
TIMEOUT = 1.0

CONTROL_MODES = {0: "OFF", 1: "Manual", 2: "Thermostat", 3: "PID", 4: "Auto-Tune"}
OUTPUT_MODES   = {0: "+ve only", 1: "−ve only", 2: "Bidirectional", 3: "TRIAC"}

# Registers
REG_STATUS       = 1
REG_CTRL_MODE    = 2
REG_OUTPUT_MODE  = 3
REG_SETPOINT     = 4
REG_P            = 5
REG_I            = 6
REG_D            = 7
REG_SENSOR_EN    = 106
REG_TC1_TEMP     = 107   # Sensor E — K-type TC1
REG_TC2_TEMP     = 108   # Sensor F — K-type TC2
REG_CTRL_SENSOR  = 109
REG_BRIDGE_V     = 78
REG_BRIDGE_I     = 80
REG_PWM_OUT      = 82
REG_SUPPLY_V     = 83
REG_SENS_A_TEMP  = 65    # NTC sensor A (spare)
REG_SENS_B_TEMP  = 66    # NTC sensor B (spare)


class ControllerError(Exception):
    """Raised when the board returns an error or no response."""


class JuniorController:
    def __init__(self, port: str):
        self.port = port
        self.ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        time.sleep(0.5)  # let the board settle after USB enumeration
        self.ser.reset_input_buffer()  # drop boot-time chatter
        self._lock = threading.Lock()

    def _cmd(self, cmd: str) -> str:
        """Send a command and return the response line.

        Raises ControllerError on timeout (empty line) or an ERR reply.
        """
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\r\n").encode())
            resp = self.ser.readline().decode(errors="replace").strip()
        if resp == "":
            raise ControllerError(f"no response to {cmd!r} (timeout)")
        if resp.upper().startswith("ERR"):
            raise ControllerError(f"{cmd!r} → {resp}")
        return resp

    def identify(self) -> str:
        try:
            return self._cmd("$ID")
        except ControllerError as e:
            return f"<no ID: {e}>"

    def read_reg(self, n: int) -> str:
        resp = self._cmd(f"$REG {n}")
        # response format: "REG n=value"
        if "=" in resp:
            return resp.split("=", 1)[1].strip()
        return resp

    def write_reg(self, n: int, value) -> str:
        resp = self._cmd(f"$REG {n}={value}")
        if "=" in resp:
            return resp.split("=", 1)[1].strip()
        return resp

    def run(self) -> str:
        return self._cmd("$RUN")

    def stop(self) -> str:
        return self._cmd("$STOP")

    def read_float(self, n: int) -> float | None:
        try:
            return float(self.read_reg(n))
        except (ValueError, TypeError, ControllerError):
            return None

    def read_int(self, n: int) -> int | None:
        try:
            return int(float(self.read_reg(n)))
        except (ValueError, TypeError, ControllerError):
            return None

    def close(self):
        self.ser.close()


class SimulatedController:
    """In-memory fake of the board for testing the UI without hardware.

    Runs a crude thermal model so temperatures, PWM and current respond to
    the setpoint and control mode.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.regs: dict[int, float] = {
            REG_STATUS: 0, REG_CTRL_MODE: 0, REG_OUTPUT_MODE: 2,
            REG_SETPOINT: 20.0, REG_P: 4.0, REG_I: 0.2, REG_D: 1.0,
            REG_SENSOR_EN: 48, REG_CTRL_SENSOR: 4,
            REG_TC1_TEMP: 22.0, REG_TC2_TEMP: 22.0,
            REG_SENS_A_TEMP: 22.0, REG_SENS_B_TEMP: 22.0,
            REG_BRIDGE_V: 0.0, REG_BRIDGE_I: 0.0, REG_PWM_OUT: 0,
            REG_SUPPLY_V: 12.0,
        }
        self.running = False
        self._last = time.monotonic()

    def _step(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        sp = self.regs[REG_SETPOINT]
        ctrl_is_tc1 = self.regs[REG_CTRL_SENSOR] == 4
        ctrl_reg = REG_TC1_TEMP if ctrl_is_tc1 else REG_TC2_TEMP
        other_reg = REG_TC2_TEMP if ctrl_is_tc1 else REG_TC1_TEMP
        if self.running and self.regs[REG_CTRL_MODE] in (1, 2, 3, 4):
            err = sp - self.regs[ctrl_reg]
            drive = max(-1.0, min(1.0, err * 0.15))  # signed effort on control side
            self.regs[REG_PWM_OUT] = int(abs(drive) * 100)
            self.regs[REG_BRIDGE_V] = abs(drive) * self.regs[REG_SUPPLY_V]
            self.regs[REG_BRIDGE_I] = abs(drive) * 6.0
            # control side moves toward setpoint; the other side is the heat dump
            self.regs[ctrl_reg]  += drive * 8.0 * dt
            self.regs[other_reg] += (-drive * 4.0 + (22.0 - self.regs[other_reg]) * 0.1) * dt
            # spare NTCs loosely track the two sides (heatsink probes)
            self.regs[REG_SENS_A_TEMP] += (self.regs[REG_TC1_TEMP] - self.regs[REG_SENS_A_TEMP]) * 0.3 * dt
            self.regs[REG_SENS_B_TEMP] += (self.regs[REG_TC2_TEMP] - self.regs[REG_SENS_B_TEMP]) * 0.3 * dt
        else:
            self.regs[REG_PWM_OUT] = 0
            self.regs[REG_BRIDGE_V] = 0.0
            self.regs[REG_BRIDGE_I] = 0.0
            self.regs[REG_TC1_TEMP] += (22.0 - self.regs[REG_TC1_TEMP]) * 0.2 * dt
            self.regs[REG_TC2_TEMP] += (22.0 - self.regs[REG_TC2_TEMP]) * 0.2 * dt
            self.regs[REG_SENS_A_TEMP] += (22.0 - self.regs[REG_SENS_A_TEMP]) * 0.2 * dt
            self.regs[REG_SENS_B_TEMP] += (22.0 - self.regs[REG_SENS_B_TEMP]) * 0.2 * dt

    def identify(self) -> str:
        return "ID=Junior V2 (SIMULATED)"

    def read_reg(self, n: int) -> str:
        with self._lock:
            self._step()
            return str(self.regs.get(n, 0))

    def write_reg(self, n: int, value) -> str:
        with self._lock:
            self.regs[n] = value
            return str(value)

    def run(self) -> str:
        self.running = True
        return "OK RUN"

    def stop(self) -> str:
        self.running = False
        return "OK STOP"

    def read_float(self, n: int) -> float | None:
        try:
            return float(self.read_reg(n))
        except (ValueError, TypeError):
            return None

    def read_int(self, n: int) -> int | None:
        try:
            return int(float(self.read_reg(n)))
        except (ValueError, TypeError):
            return None

    def close(self):
        pass


# ── Live state ────────────────────────────────────────────────────────────────

HISTORY_LEN = 600   # ~10 min of 1 Hz samples kept for the live graph


class State:
    def __init__(self):
        self.tc1_temp: float | None = None      # hot side
        self.tc2_temp: float | None = None      # cold side
        self.ntc_a:    float | None = None      # spare NTC sensor A
        self.ntc_b:    float | None = None      # spare NTC sensor B
        self.bridge_v: float | None = None
        self.bridge_i: float | None = None
        self.pwm_out:  int   | None = None
        self.supply_v: float | None = None
        self.ctrl_mode: int  | None = None
        self.out_mode:  int  | None = None
        self.setpoint:  float| None = None
        self.p_gain:    float| None = None
        self.i_gain:    float| None = None
        self.d_gain:    float| None = None
        self.ctrl_sensor: int| None = None
        self.status:    int  | None = None
        self.last_update: str = "—"
        self.error: str = ""
        # rolling (elapsed_s, tc1, tc2, setpoint) samples for the live graph
        self.history: deque = deque(maxlen=HISTORY_LEN)
        self.lock = threading.Lock()

    def snapshot(self) -> "State":
        """Return a consistent copy so the UI never renders a torn frame."""
        with self.lock:
            copy = State()
            for k, v in self.__dict__.items():
                if k == "lock":
                    continue
                if k == "history":
                    copy.history = list(v)   # detach from the live deque
                else:
                    copy.__dict__[k] = v
            return copy


CSV_COLUMNS = [
    "iso_time", "elapsed_s", "tc1_hot_C", "tc2_cold_C", "delta_T_C",
    "ntc_a_C", "ntc_b_C", "setpoint_C", "pwm_pct", "tec_voltage_V",
    "tec_current_A", "supply_V", "ctrl_mode", "ctrl_sensor",
]


class CsvLogger:
    """Appends one timestamped row of every readable channel per poll."""

    def __init__(self, path: str):
        self.path = path
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        self._fh = open(path, "a", newline="")
        self._writer = csv.writer(self._fh)
        if new_file:
            self._writer.writerow(CSV_COLUMNS)
            self._fh.flush()
        self.rows = 0

    def log(self, elapsed: float, s: "State"):
        delta = (s.tc1_temp - s.tc2_temp) if (s.tc1_temp is not None and s.tc2_temp is not None) else None
        self._writer.writerow([
            datetime.now().isoformat(timespec="seconds"), f"{elapsed:.1f}",
            s.tc1_temp, s.tc2_temp, delta, s.ntc_a, s.ntc_b, s.setpoint,
            s.pwm_out, s.bridge_v, s.bridge_i, s.supply_v,
            s.ctrl_mode, s.ctrl_sensor,
        ])
        self._fh.flush()
        self.rows += 1

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass


def poll_loop(ctrl, state: State, stop_event: threading.Event, logger: "CsvLogger | None" = None):
    """Background thread: refresh live readings every second."""
    start = time.monotonic()
    while not stop_event.is_set():
        try:
            readings = dict(
                tc1_temp    = ctrl.read_float(REG_TC1_TEMP),
                tc2_temp    = ctrl.read_float(REG_TC2_TEMP),
                ntc_a       = ctrl.read_float(REG_SENS_A_TEMP),
                ntc_b       = ctrl.read_float(REG_SENS_B_TEMP),
                bridge_v    = ctrl.read_float(REG_BRIDGE_V),
                bridge_i    = ctrl.read_float(REG_BRIDGE_I),
                pwm_out     = ctrl.read_int(REG_PWM_OUT),
                supply_v    = ctrl.read_float(REG_SUPPLY_V),
                ctrl_mode   = ctrl.read_int(REG_CTRL_MODE),
                out_mode    = ctrl.read_int(REG_OUTPUT_MODE),
                setpoint    = ctrl.read_float(REG_SETPOINT),
                p_gain      = ctrl.read_float(REG_P),
                i_gain      = ctrl.read_float(REG_I),
                d_gain      = ctrl.read_float(REG_D),
                ctrl_sensor = ctrl.read_int(REG_CTRL_SENSOR),
                status      = ctrl.read_int(REG_STATUS),
            )
            elapsed = time.monotonic() - start
            with state.lock:
                state.__dict__.update(readings)
                state.history.append((elapsed, state.tc1_temp, state.tc2_temp, state.setpoint))
                state.last_update = datetime.now().strftime("%H:%M:%S")
                state.error = ""
            if logger:
                logger.log(elapsed, state.snapshot())
        except Exception as e:
            with state.lock:
                state.error = str(e)
        stop_event.wait(1.0)


# ── Display helpers ───────────────────────────────────────────────────────────

def fmt_temp(val: float | None) -> Text:
    if val is None:
        return Text("—", style="dim")
    color = "cyan" if val < 10 else "green" if val < 40 else "yellow" if val < 60 else "red"
    return Text(f"{val:+.1f} °C", style=f"bold {color}")

def fmt_float(val, unit="", fmt=".2f") -> str:
    return f"{val:{fmt}} {unit}".strip() if val is not None else "—"

def status_flags(status: int | None) -> str:
    if status is None:
        return "—"
    flags = []
    if status & (1 << 0):  flags.append("[red]SHUTDOWN[/]")
    if status & (1 << 1):  flags.append("[yellow]RELAY[/]")
    if status & (1 << 6):  flags.append("[cyan]HEAT/COOL[/]")
    if status & (1 << 11): flags.append("[green]TUNE OK[/]")
    if status & (1 << 12): flags.append("[red]TUNE FAIL[/]")
    if status & (1 << 13): flags.append("[red]FAULT[/]")
    return " ".join(flags) if flags else "[green]OK[/]"


def build_display(state: State) -> Panel:
    console = Console()

    # ── Temperature panel ────────────────────────────────
    temp_table = Table(box=box.SIMPLE, show_header=True, header_style="bold white")
    temp_table.add_column("Sensor",   style="bold", width=20)
    temp_table.add_column("Temp",     justify="right", width=12)
    temp_table.add_column("Role",     width=16)

    ctrl_s = state.ctrl_sensor
    tc1_role = "[green]◀ CONTROL[/]" if ctrl_s == 4 else "hot side"
    tc2_role = "[green]◀ CONTROL[/]" if ctrl_s == 5 else "cold side"

    temp_table.add_row("TC1 — Hot side",  fmt_temp(state.tc1_temp),  tc1_role)
    temp_table.add_row("TC2 — Cold side", fmt_temp(state.tc2_temp),  tc2_role)
    if state.tc1_temp is not None and state.tc2_temp is not None:
        delta = state.tc1_temp - state.tc2_temp
        temp_table.add_row("ΔT (hot − cold)",
                           Text(f"{delta:+.1f} °C", style="bold magenta"), "")
    if state.ntc_a is not None:
        temp_table.add_row("NTC A — spare", fmt_temp(state.ntc_a), "[dim]aux[/]")
    if state.ntc_b is not None:
        temp_table.add_row("NTC B — spare", fmt_temp(state.ntc_b), "[dim]aux[/]")

    # ── Control panel ────────────────────────────────────
    ctrl_mode_str = CONTROL_MODES.get(state.ctrl_mode, "?") if state.ctrl_mode is not None else "—"
    out_mode_str  = OUTPUT_MODES.get(state.out_mode, "?")  if state.out_mode  is not None else "—"

    ctrl_table = Table(box=box.SIMPLE, show_header=False)
    ctrl_table.add_column("Key",   style="dim", width=18)
    ctrl_table.add_column("Value", style="bold")

    ctrl_table.add_row("Control mode",  f"[cyan]{ctrl_mode_str}[/]")
    ctrl_table.add_row("Output mode",   out_mode_str)
    ctrl_table.add_row("Setpoint",      fmt_float(state.setpoint, "°C", "+.1f"))
    ctrl_table.add_row("P / I / D",
        f"{fmt_float(state.p_gain)} / {fmt_float(state.i_gain)} / {fmt_float(state.d_gain)}")

    # ── Bridge panel ─────────────────────────────────────
    bridge_table = Table(box=box.SIMPLE, show_header=False)
    bridge_table.add_column("Key",   style="dim", width=18)
    bridge_table.add_column("Value", style="bold")

    bridge_table.add_row("PWM output",   fmt_float(state.pwm_out, "%", ".0f"))
    bridge_table.add_row("TEC voltage",  fmt_float(state.bridge_v, "V"))
    bridge_table.add_row("TEC current",  fmt_float(state.bridge_i, "A"))
    bridge_table.add_row("Supply V",     fmt_float(state.supply_v, "V"))
    bridge_table.add_row("Status",       status_flags(state.status))

    panels = Table.grid(expand=True)
    panels.add_column(ratio=1)
    panels.add_column(ratio=1)
    panels.add_column(ratio=1)
    panels.add_row(
        Panel(temp_table,   title="[bold]Temperatures[/]",  border_style="blue"),
        Panel(ctrl_table,   title="[bold]Control[/]",       border_style="green"),
        Panel(bridge_table, title="[bold]Bridge / TEC[/]",  border_style="yellow"),
    )

    combined = Table.grid(expand=True)
    combined.add_column()
    combined.add_row(panels)
    if SHOW_GRAPH:
        graph = render_live_graph(state.history)
        if graph is not None:
            combined.add_row(Panel(graph, title="[bold]Temperature history[/]",
                                   border_style="magenta"))

    footer = f"[dim]Updated: {state.last_update}   Port: {state.error if state.error else 'OK'}[/]"
    return Panel(combined, title="[bold white]Adaptive Junior V2 — Peltier Control[/]",
                 subtitle=footer, border_style="bright_blue")


# ── Graphing ──────────────────────────────────────────────────────────────────

SHOW_GRAPH = True   # toggled off by --no-graph or if plotext is unavailable


def render_live_graph(history) -> Text | None:
    """Render a plotext line graph of TC1/TC2/setpoint to a rich Text block."""
    pts = [h for h in history if h[1] is not None or h[2] is not None]
    if len(pts) < 2:
        return Text("collecting data…", style="dim")
    try:
        import plotext as plt
    except ImportError:
        return Text("install plotext for live graphs:  pip install plotext", style="yellow")

    t   = [p[0] for p in pts]
    tc1 = [p[1] if p[1] is not None else float("nan") for p in pts]
    tc2 = [p[2] if p[2] is not None else float("nan") for p in pts]
    sp  = [p[3] if p[3] is not None else float("nan") for p in pts]

    width = max(60, min(Console().width - 6, 160))
    plt.clf()
    plt.plotsize(width, 16)
    plt.theme("dark")
    plt.plot(t, tc1, label="TC1 hot",  color="red")
    plt.plot(t, tc2, label="TC2 cold", color="cyan")
    if any(s == s for s in sp):  # at least one non-NaN
        plt.plot(t, sp, label="setpoint", color="green")
    plt.xlabel("seconds")
    plt.ylabel("°C")
    out = plt.build()
    return Text.from_ansi(out)


def export_png(csv_path: str, out_path: str | None = None) -> str:
    """Render a publication-quality PNG from a logged CSV. Returns the path."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise ControllerError("matplotlib not installed:  pip install matplotlib")

    t, tc1, tc2, dT, sp = [], [], [], [], []
    with open(csv_path, newline="") as fh:
        for row in csv.DictReader(fh):
            def num(key):
                v = row.get(key, "")
                try:
                    return float(v)
                except (ValueError, TypeError):
                    return float("nan")
            t.append(num("elapsed_s"))
            tc1.append(num("tc1_hot_C"))
            tc2.append(num("tc2_cold_C"))
            dT.append(num("delta_T_C"))
            sp.append(num("setpoint_C"))
    if len(t) < 2:
        raise ControllerError(f"not enough samples in {csv_path} to plot")

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    ax1.plot(t, tc1, label="TC1 hot side",  color="tab:red")
    ax1.plot(t, tc2, label="TC2 cold side", color="tab:blue")
    ax1.plot(t, sp,  label="setpoint", color="tab:green", linestyle="--", linewidth=1)
    ax1.set_ylabel("Temperature (°C)")
    ax1.set_title("Adaptive Junior V2 — Peltier temperature log")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, dT, label="ΔT (hot − cold)", color="tab:purple")
    ax2.set_ylabel("ΔT (°C)")
    ax2.set_xlabel("Elapsed time (s)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


# ── Menu / interactive control ────────────────────────────────────────────────

MENU = """
[bold white]Commands[/]
  [cyan]m[/]  Set control [cyan]m[/]ode        [cyan]s[/]  Set [cyan]s[/]etpoint (°C)
  [cyan]p[/]  Set [cyan]P[/] gain              [cyan]i[/]  Set [cyan]I[/] gain
  [cyan]d[/]  Set [cyan]D[/] gain              [cyan]o[/]  Set [cyan]o[/]utput mode
  [cyan]c[/]  Set [cyan]c[/]ontrol sensor      [cyan]e[/]  [cyan]E[/]nable sensors (TC1+TC2)
  [cyan]r[/]  [cyan]R[/]UN output              [cyan]x[/]  Stop (SHUTDOWN)
  [cyan]g[/]  Export [cyan]g[/]raph PNG         [cyan]q[/]  [cyan]Q[/]uit
"""

def prompt(console: Console, msg: str) -> str:
    return console.input(f"[green]>[/] {msg}: ").strip()

def set_mode(ctrl: JuniorController, console: Console):
    console.print("[bold]Modes:[/] 0=OFF  1=Manual  2=Thermostat  3=PID  4=Auto-Tune")
    v = prompt(console, "Enter mode number")
    if v.isdigit() and int(v) in CONTROL_MODES:
        result = ctrl.write_reg(REG_CTRL_MODE, int(v))
        console.print(f"  → {result}")
    else:
        console.print("[red]Invalid mode[/]")

def set_setpoint(ctrl: JuniorController, console: Console, state: State):
    current = f"{state.setpoint:+.1f}" if state.setpoint is not None else "?"
    v = prompt(console, f"Setpoint °C (current: {current})")
    try:
        result = ctrl.write_reg(REG_SETPOINT, float(v))
        console.print(f"  → {result}")
    except ValueError:
        console.print("[red]Invalid value[/]")

def set_gain(ctrl: JuniorController, console: Console, reg: int, name: str, current):
    cur = f"{current:.4f}" if current is not None else "?"
    v = prompt(console, f"{name} gain (current: {cur})")
    try:
        result = ctrl.write_reg(reg, float(v))
        console.print(f"  → {result}")
    except ValueError:
        console.print("[red]Invalid value[/]")

def set_output_mode(ctrl: JuniorController, console: Console):
    console.print("[bold]Output modes:[/] 0=+ve only  1=−ve only  2=Bidirectional  3=TRIAC")
    console.print("[yellow]Note: can only be changed when control mode = OFF[/]")
    v = prompt(console, "Enter output mode number")
    if v.isdigit() and int(v) in OUTPUT_MODES:
        result = ctrl.write_reg(REG_OUTPUT_MODE, int(v))
        console.print(f"  → {result}")
    else:
        console.print("[red]Invalid mode[/]")

def set_ctrl_sensor(ctrl: JuniorController, console: Console):
    console.print("[bold]Sensors:[/] 4=TC1 (hot side, sensor E)  5=TC2 (cold side, sensor F)")
    v = prompt(console, "Enter sensor number")
    if v in ("4", "5"):
        result = ctrl.write_reg(REG_CTRL_SENSOR, int(v))
        console.print(f"  → {result}")
    else:
        console.print("[red]Enter 4 or 5[/]")

def enable_sensors(ctrl: JuniorController, console: Console):
    """Enable TC1 (bit 4) and TC2 (bit 5) → value 48 (0b110000)."""
    result = ctrl.write_reg(REG_SENSOR_EN, 48)
    console.print(f"  Sensor enable register → {result}  (TC1 + TC2 enabled)")


# ── Main ──────────────────────────────────────────────────────────────────────

def find_port() -> str:
    """Try common Linux USB-serial ports."""
    import glob
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return "/dev/ttyUSB0"


def main():
    parser = argparse.ArgumentParser(description="Adaptive Junior V2 — Linux control UI")
    parser.add_argument("--port", default=None,
                        help="Serial port (default: auto-detect /dev/ttyACM0 or /dev/ttyUSB0)")
    parser.add_argument("--simulate", action="store_true",
                        help="Run against an in-memory simulated board (no hardware needed)")
    parser.add_argument("--logfile", default=None,
                        help="CSV log path (default: peltier_log_<timestamp>.csv)")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV logging")
    parser.add_argument("--no-graph", action="store_true",
                        help="Disable the live in-terminal temperature graph")
    args = parser.parse_args()

    global SHOW_GRAPH
    SHOW_GRAPH = not args.no_graph

    console = Console()
    console.print(f"\n[bold]Adaptive Junior V2 — Peltier Control UI[/]")

    if args.simulate:
        console.print("[yellow]Running in SIMULATION mode — no hardware.[/]\n")
        ctrl = SimulatedController()
    else:
        port = args.port or find_port()
        console.print(f"Connecting to [cyan]{port}[/] at 115200 baud…\n")
        try:
            ctrl = JuniorController(port)
        except serial.SerialException as e:
            console.print(f"[red]Cannot open {port}: {e}[/]")
            console.print("Check the port with:  ls /dev/tty{USB,ACM}*")
            console.print("Or run without hardware:  python3 peltier_control.py --simulate")
            sys.exit(1)

    ident = ctrl.identify()
    console.print(f"[green]Connected:[/] {ident}")

    logger = None
    if not args.no_log:
        log_path = args.logfile or f"peltier_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        try:
            logger = CsvLogger(log_path)
            console.print(f"[green]Logging to:[/] {os.path.abspath(log_path)}")
        except OSError as e:
            console.print(f"[red]Could not open log file {log_path}: {e}[/]")
    console.print()

    state = State()
    stop_event = threading.Event()
    poll_thread = threading.Thread(target=poll_loop, args=(ctrl, state, stop_event, logger), daemon=True)
    poll_thread.start()

    # Give the first poll time to complete
    time.sleep(1.5)

    def action(label_ok, label, fn):
        try:
            console.print(f"  {label_ok} {fn()}")
        except ControllerError as e:
            console.print(f"  [red]{label} failed:[/] {e}")

    try:
        while True:
            console.clear()
            console.print(build_display(state.snapshot()))
            console.print(MENU)
            try:
                key = console.input("[bold green]Command[/]: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if key == "q":
                break
            elif key == "m":
                set_mode(ctrl, console)
            elif key == "s":
                set_setpoint(ctrl, console, state.snapshot())
            elif key == "p":
                set_gain(ctrl, console, REG_P, "P", state.p_gain)
            elif key == "i":
                set_gain(ctrl, console, REG_I, "I", state.i_gain)
            elif key == "d":
                set_gain(ctrl, console, REG_D, "D", state.d_gain)
            elif key == "o":
                set_output_mode(ctrl, console)
            elif key == "c":
                set_ctrl_sensor(ctrl, console)
            elif key == "e":
                enable_sensors(ctrl, console)
            elif key == "r":
                action("[green]RUN →[/]", "RUN", ctrl.run)
            elif key == "x":
                action("[red]STOP →[/]", "STOP", ctrl.stop)
            elif key == "g":
                if logger is None:
                    console.print("[yellow]No CSV log is active to plot from.[/]")
                else:
                    try:
                        png = export_png(logger.path)
                        console.print(f"  [green]Saved graph →[/] {os.path.abspath(png)}")
                    except ControllerError as e:
                        console.print(f"  [red]Graph export failed:[/] {e}")
            elif key == "":
                continue  # just refresh
            else:
                console.print("[dim]Unknown command[/]")

            console.input("[dim]Press Enter to continue…[/]")

    finally:
        stop_event.set()
        poll_thread.join(timeout=2)
        if logger:
            logger.close()
            console.print(f"[dim]Wrote {logger.rows} rows to {logger.path}[/]")
        ctrl.close()
        console.print("[dim]Connection closed.[/]")


if __name__ == "__main__":
    main()

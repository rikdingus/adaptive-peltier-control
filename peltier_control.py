#!/usr/bin/env python3
"""
Adaptive Junior V2 — Linux Terminal Control UI
Controls TEC1-12706 via ADJ-48-450-UR-V2 over USB serial.

Requirements:
    pip install pyserial rich --break-system-packages

Usage:
    python3 peltier_control.py [--port /dev/ttyUSB0]
"""

import argparse
import sys
import time
import threading
import serial
from datetime import datetime

try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.text import Text
    from rich import box
    from rich.columns import Columns
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


class JuniorController:
    def __init__(self, port: str):
        self.port = port
        self.ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        time.sleep(0.5)  # let the board settle after USB enumeration
        self._lock = threading.Lock()

    def _cmd(self, cmd: str) -> str:
        """Send a command and return the response line."""
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\r\n").encode())
            resp = self.ser.readline().decode(errors="replace").strip()
            return resp

    def identify(self) -> str:
        return self._cmd("$ID")

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
        self.ser.close()


# ── Live state ────────────────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.tc1_temp: float | None = None      # hot side
        self.tc2_temp: float | None = None      # cold side
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


def poll_loop(ctrl: JuniorController, state: State, stop_event: threading.Event):
    """Background thread: refresh live readings every second."""
    while not stop_event.is_set():
        try:
            state.tc1_temp    = ctrl.read_float(REG_TC1_TEMP)
            state.tc2_temp    = ctrl.read_float(REG_TC2_TEMP)
            state.bridge_v    = ctrl.read_float(REG_BRIDGE_V)
            state.bridge_i    = ctrl.read_float(REG_BRIDGE_I)
            state.pwm_out     = ctrl.read_int(REG_PWM_OUT)
            state.supply_v    = ctrl.read_float(REG_SUPPLY_V)
            state.ctrl_mode   = ctrl.read_int(REG_CTRL_MODE)
            state.out_mode    = ctrl.read_int(REG_OUTPUT_MODE)
            state.setpoint    = ctrl.read_float(REG_SETPOINT)
            state.p_gain      = ctrl.read_float(REG_P)
            state.i_gain      = ctrl.read_float(REG_I)
            state.d_gain      = ctrl.read_float(REG_D)
            state.ctrl_sensor = ctrl.read_int(REG_CTRL_SENSOR)
            state.status      = ctrl.read_int(REG_STATUS)
            state.last_update = datetime.now().strftime("%H:%M:%S")
            state.error = ""
        except Exception as e:
            state.error = str(e)
        time.sleep(1.0)


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

    bridge_table.add_row("PWM output",   fmt_float(state.pwm_out, "%"))
    bridge_table.add_row("TEC voltage",  fmt_float(state.bridge_v, "V"))
    bridge_table.add_row("TEC current",  fmt_float(state.bridge_i, "A"))
    bridge_table.add_row("Supply V",     fmt_float(state.supply_v, "V"))
    bridge_table.add_row("Status",       status_flags(state.status))

    combined = Table.grid(expand=True)
    combined.add_column(ratio=1)
    combined.add_column(ratio=1)
    combined.add_column(ratio=1)
    combined.add_row(
        Panel(temp_table,   title="[bold]Temperatures[/]",  border_style="blue"),
        Panel(ctrl_table,   title="[bold]Control[/]",       border_style="green"),
        Panel(bridge_table, title="[bold]Bridge / TEC[/]",  border_style="yellow"),
    )

    footer = f"[dim]Updated: {state.last_update}   Port: {state.error if state.error else 'OK'}[/]"
    return Panel(combined, title="[bold white]Adaptive Junior V2 — Peltier Control[/]",
                 subtitle=footer, border_style="bright_blue")


# ── Menu / interactive control ────────────────────────────────────────────────

MENU = """
[bold white]Commands[/]
  [cyan]m[/]  Set control [cyan]m[/]ode        [cyan]s[/]  Set [cyan]s[/]etpoint (°C)
  [cyan]p[/]  Set [cyan]P[/] gain              [cyan]i[/]  Set [cyan]I[/] gain
  [cyan]d[/]  Set [cyan]D[/] gain              [cyan]o[/]  Set [cyan]o[/]utput mode
  [cyan]c[/]  Set [cyan]c[/]ontrol sensor      [cyan]e[/]  [cyan]E[/]nable sensors (TC1+TC2)
  [cyan]r[/]  [cyan]R[/]UN output              [cyan]x[/]  Stop (SHUTDOWN)
  [cyan]q[/]  [cyan]Q[/]uit
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
    args = parser.parse_args()

    port = args.port or find_port()
    console = Console()

    console.print(f"\n[bold]Adaptive Junior V2 — Peltier Control UI[/]")
    console.print(f"Connecting to [cyan]{port}[/] at 115200 baud…\n")

    try:
        ctrl = JuniorController(port)
    except serial.SerialException as e:
        console.print(f"[red]Cannot open {port}: {e}[/]")
        console.print("Check the port with:  ls /dev/tty{USB,ACM}*")
        sys.exit(1)

    ident = ctrl.identify()
    console.print(f"[green]Connected:[/] {ident}\n")

    state = State()
    stop_event = threading.Event()
    poll_thread = threading.Thread(target=poll_loop, args=(ctrl, state, stop_event), daemon=True)
    poll_thread.start()

    # Give the first poll time to complete
    time.sleep(1.5)

    try:
        with Live(build_display(state), console=console, refresh_per_second=1,
                  screen=False, transient=False) as live:

            while True:
                # Refresh display
                live.update(build_display(state))

                # Print menu and get input
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
                    set_setpoint(ctrl, console, state)
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
                    resp = ctrl._cmd("$RUN")
                    console.print(f"  [green]RUN →[/] {resp}")
                elif key == "x":
                    resp = ctrl._cmd("$STOP")
                    console.print(f"  [red]STOP →[/] {resp}")
                else:
                    console.print("[dim]Unknown command[/]")

                time.sleep(0.2)
                live.update(build_display(state))

    finally:
        stop_event.set()
        poll_thread.join(timeout=2)
        ctrl.close()
        console.print("\n[dim]Connection closed.[/]")


if __name__ == "__main__":
    main()

# Adaptive Junior V2 — Peltier Control Software

Software to monitor and control a **TEC1‑12706** Peltier module through an
**ADJ‑48‑450‑UR‑V2 (Adaptive Junior V2)** controller over USB serial.

Two front‑ends are included, sharing the same control core:

| File | Interface | Best for |
|------|-----------|----------|
| `peltier_control.py` | Terminal UI (text dashboard + menu) | SSH sessions, headless boxes, quick checks |
| `peltier_gui.py` | Desktop window (Tkinter + live plots) | A workstation with a display |

Both stream every sensor to a timestamped **CSV log** and can render
**graphs** — live while running and as exported PNG charts.

For wiring, the serial protocol, and controller first‑time setup, see
[`project_walkthrough.md`](project_walkthrough.md). This document covers the
**software**.

---

## 1. Requirements

- **Python 3.10+**
- A connected Adaptive Junior V2 board, *or* use `--simulate` for a built‑in
  fake board (no hardware needed).

### Install dependencies

```bash
# Core (always needed)
pip install pyserial rich

# Graphs (optional but recommended)
pip install plotext      # live in‑terminal graph
pip install matplotlib   # PNG export + desktop‑app plots
```

On a system‑managed Python (Debian/Ubuntu/Raspberry Pi OS) add
`--break-system-packages`:

```bash
pip install pyserial rich plotext matplotlib --break-system-packages
```

**For the desktop app** you also need Tkinter (ships with most CPython builds).
On Debian/Ubuntu, if missing:

```bash
sudo apt install python3-tk
```

> The graph libraries are **optional**. If `plotext` is absent the terminal UI
> shows a one‑line install hint instead of the live graph; if `matplotlib` is
> absent, PNG export reports a clear error. Everything else keeps working.

---

## 2. Connecting the board

1. Wire up power, the TEC, both thermocouples, and a hot‑side fan (see the
   walkthrough). Fit heatsinks — **never dry‑run the TEC**.
2. Power on the board, then plug USB into the PC.
3. Find the serial port:
   ```bash
   ls /dev/tty{USB,ACM}*
   ```
   It is usually `/dev/ttyACM0` or `/dev/ttyUSB0`.
4. If you get a permission error, add yourself to the `dialout` group and log
   back in:
   ```bash
   sudo usermod -aG dialout $USER
   ```

The software auto‑detects the port; pass `--port` only to override.

---

## 3. Terminal UI — `peltier_control.py`

### Run it

```bash
python3 peltier_control.py                 # auto‑detect port, log + live graph on
python3 peltier_control.py --port /dev/ttyACM0
python3 peltier_control.py --simulate      # no hardware — try it right now
```

### What you see

A live dashboard refreshes on every action:

- **Temperatures** — TC1 (hot), TC2 (cold), ΔT, and the two spare NTC sensors.
  The active control sensor is marked `◀ CONTROL`.
- **Control** — control mode, output mode, setpoint, and P/I/D gains.
- **Bridge / TEC** — PWM %, TEC voltage/current, supply voltage, status flags.
- **Temperature history** — a live scrolling graph of TC1, TC2, and the
  setpoint (needs `plotext`).

### Command menu

Type a key and press Enter:

| Key | Action |
|-----|--------|
| `m` | Set control mode (0=OFF, 1=Manual, 2=Thermostat, 3=PID, 4=Auto‑Tune) |
| `s` | Set setpoint (°C) |
| `p` / `i` / `d` | Set P / I / D gain |
| `o` | Set output mode (0=+ve, 1=−ve, 2=Bidirectional, 3=TRIAC) |
| `c` | Set control sensor (4=TC1 hot, 5=TC2 cold) |
| `e` | Enable thermocouples TC1 + TC2 (writes register 106 = 48) |
| `r` | **RUN** output drive |
| `x` | **STOP** (shutdown) |
| `g` | Export a PNG graph from the current CSV log |
| `q` | Quit |

Press **Enter on an empty command** to just refresh the dashboard.

### Command‑line options

| Flag | Description |
|------|-------------|
| `--port PATH` | Serial port (default: auto‑detect) |
| `--simulate` | Use the built‑in simulated board |
| `--logfile PATH` | CSV log path (default: `peltier_log_<timestamp>.csv`) |
| `--no-log` | Disable CSV logging |
| `--no-graph` | Disable the live in‑terminal graph |

---

## 4. Desktop App — `peltier_gui.py`

A windowed front‑end with the same capabilities plus an embedded, always‑on
live plot.

### Run it

```bash
python3 peltier_gui.py                 # auto‑detect port
python3 peltier_gui.py --simulate      # no hardware
python3 peltier_gui.py --port /dev/ttyACM0 --logfile run.csv
```

### Layout

- **Live readings** (left) — all temperatures, ΔT, PWM, TEC voltage/current,
  supply voltage, and a colour‑coded status line.
- **Controls** (left) — setpoint field, control‑mode / output‑mode /
  control‑sensor dropdowns, P/I/D fields, green **RUN** and red **STOP**
  buttons, *Enable TC1+TC2*, and *Export graph PNG*.
- **Plots** (right) — a real‑time temperature chart (TC1, TC2, setpoint) with a
  ΔT subplot below, updated every second.

The dropdowns and setpoint field auto‑populate from the board's reported state
on startup. The status bar at the bottom shows the latest action and log path.

### Command‑line options

| Flag | Description |
|------|-------------|
| `--port PATH` | Serial port (default: auto‑detect) |
| `--simulate` | Use the built‑in simulated board |
| `--logfile PATH` | CSV log path |
| `--no-log` | Disable CSV logging |

---

## 5. Logging

Whenever logging is enabled (the default), one row per second is appended to a
CSV file and flushed immediately, so a crash or unplug loses at most the last
sample.

**Default filename:** `peltier_log_YYYYMMDD_HHMMSS.csv` in the working
directory. Override with `--logfile`.

### CSV columns

| Column | Meaning |
|--------|---------|
| `iso_time` | Wall‑clock timestamp (ISO‑8601, seconds) |
| `elapsed_s` | Seconds since the session started |
| `tc1_hot_C` | TC1 hot‑side temperature (°C) |
| `tc2_cold_C` | TC2 cold‑side temperature (°C) |
| `delta_T_C` | TC1 − TC2 (°C) |
| `ntc_a_C`, `ntc_b_C` | Spare NTC sensors A and B (°C) |
| `setpoint_C` | Target temperature (°C) |
| `pwm_pct` | PWM output (%) |
| `tec_voltage_V` | Bridge/TEC voltage (V) |
| `tec_current_A` | Bridge/TEC current (A) |
| `supply_V` | Supply voltage (V) |
| `ctrl_mode` | Control mode (numeric) |
| `ctrl_sensor` | Control sensor (4=TC1, 5=TC2) |

Open the CSV in Excel, LibreOffice, or pandas for further analysis:

```python
import pandas as pd
df = pd.read_csv("peltier_log_20260602_120000.csv")
df.plot(x="elapsed_s", y=["tc1_hot_C", "tc2_cold_C", "setpoint_C"])
```

---

## 6. Graphs

### Live graphs
- **Terminal UI:** a `plotext` chart is drawn in the dashboard.
- **Desktop app:** an embedded matplotlib chart updates every second.

### Exported PNG charts
Produce a publication‑quality chart (temperature plot + ΔT subplot) from any
CSV log:

- **Terminal UI:** press `g`.
- **Desktop app:** click **Export graph PNG**.
- **Programmatically / after the fact:**
  ```python
  from peltier_control import export_png
  export_png("peltier_log_20260602_120000.csv")          # writes ...png alongside
  export_png("run.csv", "run_chart.png")                  # explicit output path
  ```

The PNG is saved next to the source CSV by default.

---

## 7. Simulation mode

Both front‑ends accept `--simulate`, which swaps the real serial controller for
an in‑memory board running a crude thermal model. The setpoint, control mode,
and control‑sensor selection all drive realistic temperature, PWM, and current
responses. Use it to:

- Try the software with no hardware attached.
- Demo or screenshot the interface.
- Develop and test changes safely.

```bash
python3 peltier_control.py --simulate
python3 peltier_gui.py --simulate
```

---

## 8. Typical workflow

1. Power on the board and connect USB.
2. Launch a front‑end (`peltier_control.py` or `peltier_gui.py`).
3. Confirm both thermocouples read sensible temperatures.
4. If first run: enable sensors (`e` / *Enable TC1+TC2*) and set the control
   sensor (`c` / dropdown).
5. Set the output mode to **Bidirectional** (requires control mode = OFF).
6. Set a setpoint near room temperature, switch to **PID**, and press **RUN**.
7. Optionally run **Auto‑Tune** (mode 4) to derive P/I/D, then return to PID.
8. Watch the live graph converge; the CSV captures the whole run.
9. Press `g` / **Export graph PNG** for a shareable chart.

---

## 9. Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Cannot open /dev/ttyACM0: Permission denied` | Add user to `dialout` group, log out/in |
| Port not found | Check `ls /dev/tty{USB,ACM}*`; power the board on **before** launching; pass `--port` |
| Temperatures read `—` | Enable the thermocouples (`e` / *Enable TC1+TC2*) and check wiring polarity |
| `no response … (timeout)` in status | Wrong port, board powered off, or cable issue |
| Live terminal graph missing | `pip install plotext` |
| PNG export fails | `pip install matplotlib` |
| Desktop app won't start (`No module named tkinter`) | `sudo apt install python3-tk` |
| Garbled box characters in the terminal | Use a UTF‑8 terminal (set `LANG`/`LC_ALL` to a UTF‑8 locale) |

---

## 10. Safety

The software can command full drive. Always observe the hardware safety
checklist in [`project_walkthrough.md`](project_walkthrough.md):

- Supply voltage ≤ 12 V.
- Heatsink and fan fitted on the hot side **before** running.
- Thermal paste between the TEC and heatsinks.
- High‑temp / low‑temp / over‑current alarms configured on the controller.
- **Never run the TEC without a thermal load on both sides.**

---

## Files

| File | Purpose |
|------|---------|
| `peltier_control.py` | Terminal control UI + shared control core, logging, graphing |
| `peltier_gui.py` | Desktop (Tkinter) control app |
| `project_walkthrough.md` | Hardware wiring, serial protocol, controller setup |
| `README.md` | This document |

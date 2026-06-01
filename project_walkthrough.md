# Adaptive Junior V2 + TEC1-12706 — Project Walkthrough

**Controller:** ADJ-48-450-UR-V2 (Adaptive Junior V2)  
**Peltier module:** TEC1-12706 (12 V, 6 A, 60 W, 40×40 mm)  
**Temperature sensing:** 2× K-type thermocouples (TC1 = hot side, TC2 = cold side)  
**PC interface:** USB-C → Linux PC (`/dev/ttyUSB0` or `/dev/ttyACM0`), 115200 baud, 8N1

---

## 1. System Overview

```
[12 V DC PSU] ──────────────────────┐
                                     ▼
                          ┌──────────────────────┐
                          │  ADJ-48-450-UR-V2     │
[TC1 — hot side] ──TC1──▶│  J2 pins 3/4          │
[TC2 — cold side] ─TC2──▶│  J2 pins 1/2          │
                          │                        │
[Fan (hot side)] ─────── │  J3 Fan 1              │
[Fan (cold side)] ──────  │  J3 Fan 2 (optional)  │
                          │                        │
[Linux PC USB-C] ──────── │  J9 USB               │
                          └───────────┬────────────┘
                                      │ TE OUT+ / TE OUT-
                                      ▼
                             [TEC1-12706 Peltier]
```

The controller handles all PWM driving of the Peltier. The Linux PC communicates over USB serial to read temperatures, change the setpoint, and switch control modes.

---

## 2. Hardware — TEC1-12706

| Parameter         | Value        |
|-------------------|--------------|
| Supply voltage    | 12 V DC      |
| Max current       | 6 A          |
| Max power         | 60 W         |
| Module size       | 40 × 40 mm   |
| ΔT max (no load)  | ~68 °C       |
| Polarity          | Red = +, Black = − |

**Important:** The controller's PWM output voltage closely follows its input supply voltage. Use a 12 V supply so the Peltier never sees more than its rated voltage. Keep output wires short (< 50 cm).

---

## 3. Wiring

### 3.1 Power (J4)

| J4 Pin | Connection          |
|--------|---------------------|
| 1      | +12 V DC supply     |
| 2      | GND (supply −)      |

### 3.2 TEC Output (J5)

| J5 Pin | Connection           |
|--------|----------------------|
| 1      | TEC1-12706 red (+)   |
| 2      | TEC1-12706 black (−) |

For bidirectional (heat and cool) operation, the controller reverses polarity internally — wire it this way and select **Bidirectional** output mode.

### 3.3 Thermocouples (J2)

The board has two dedicated K-type thermocouple inputs. Polarity matters — yellow (positive) and red (negative) for standard K-type.

| J2 Pin | Signal   | Connect to                    |
|--------|----------|-------------------------------|
| 4      | TC1+     | TC1 hot side — yellow wire    |
| 3      | TC1−     | TC1 hot side — red wire       |
| 2      | TC2+     | TC2 cold side — yellow wire   |
| 1      | TC2−     | TC2 cold side — red wire      |

> **Note:** TC1 maps to sensor E (register 107), TC2 maps to sensor F (register 108) in the serial protocol.

### 3.4 Fan(s) (J3)

Connect a heatsink fan to J3 Fan 1 (pins 1–4). A second fan for the cold side is optional. If powering fans directly from the supply, fit jumper P12 (and P13 for fan 2).

| J3 Pin | Signal            |
|--------|-------------------|
| 1      | Fan 1 GND         |
| 2      | Fan 1 PWR (+)     |
| 3      | Fan 1 TACH (3/4-wire only) |
| 4      | Fan 1 PWM (4-wire only)    |

### 3.5 USB (J9)

Connect a USB-C cable between the board and your Linux PC. Power on the board **before** opening the serial port on the PC.

On Linux the device will appear as `/dev/ttyUSB0` or `/dev/ttyACM0`. Check with:
```bash
ls /dev/tty{USB,ACM}*
dmesg | tail -20
```

Add your user to the `dialout` group if you get permission errors:
```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## 4. First-Time Setup (Controller Side)

1. **Wire up** power, TEC, both thermocouples, and at least one fan. Do not power on yet.
2. **Connect USB** to the Linux PC.
3. **Power on** the board (LED should be green).
4. **Enable thermocouples** via serial (or the official HMI on a Windows machine for first config):
   - Enable TC1 (sensor E) and TC2 (sensor F): write register 106 = `48` (binary `110000`, bits 4 and 5 set).
   - Set TC1 as the control sensor: write register 109 = `4` (sensor E).
5. **Set output mode to Bidirectional**: write register 3 = `2`. (Only changeable when control mode = OFF.)
6. **Configure a fan**: write register 39 = `1` (2-wire), register 40 = `1` (PWM mode), register 41 = `80` (80% PWM demand).
7. **Set temperature alarms** to protect the TEC (e.g., hot side high alarm at 65 °C, cold side low alarm at −10 °C).

---

## 5. Control Modes

### 5.1 OFF (mode 0)
Output drive disabled. Use this to safely change output mode or clear a shutdown alarm.

### 5.2 Manual (mode 1)
Open-loop. Setpoint is 0–100% of full output voltage, no sensor feedback. Useful for verifying TEC polarity and direction.
- 0% = zero output
- 50% = zero output (bidirectional mode null point)
- 100% = full positive (cooling)
- Set setpoint to 0–49% for heating, 51–100% for cooling in bidirectional mode.

### 5.3 PID (mode 3) — Recommended
Closed-loop control using the selected control sensor (TC1 or TC2) to reach and hold the setpoint temperature.

Key registers:
| Register | Description             | Typical starting value |
|----------|-------------------------|------------------------|
| 4        | Setpoint (°C)           | 20.0                   |
| 5        | Proportional gain (P)   | Use auto-tune first    |
| 6        | Integral gain (I)       | Use auto-tune first    |
| 7        | Differential gain (D)   | Use auto-tune first    |
| 109      | Control sensor          | 4 = TC1 (hot side)     |

### 5.4 Auto-Tune (mode 4)
The board drives the TEC through full positive and negative cycles and derives P, I, D values automatically. Takes 5–10 minutes. The system must be able to safely handle maximum drive during this process.

---

## 6. Serial Protocol Reference

**Connection settings:** 115200 baud, 8 data bits, 1 stop bit, no parity.

Commands start with `$` and end with `\r\n`.

| Command              | Example                   | Description                        |
|----------------------|---------------------------|------------------------------------|
| `$ID`                | → `ID=Junior V2...`       | Get device identity                |
| `$REG n`             | `$REG 107` → TC1 temp     | Read register n                    |
| `$REG n=x`           | `$REG 4=25.0`             | Write value x to register n        |
| `$RUN`               | —                         | Start output drive                 |
| `$STOP`              | —                         | Stop output drive (shutdown)       |

### Key Read-Only Registers (live data)

| Register | Description              | Unit   |
|----------|--------------------------|--------|
| 107      | TC1 temperature (hot)    | °C     |
| 108      | TC2 temperature (cold)   | °C     |
| 78       | Bridge (TEC) voltage     | V      |
| 80       | Bridge (TEC) current     | A      |
| 82       | PWM output value         | %      |
| 83       | Supply voltage           | V      |
| 1        | Status register (bitmask)| —      |

### Key Writable Registers

| Register | Description              | Values                               |
|----------|--------------------------|--------------------------------------|
| 2        | Control mode             | 0=OFF, 1=Manual, 3=PID, 4=AutoTune  |
| 3        | Output drive mode        | 0=+only, 1=−only, 2=Bidirectional   |
| 4        | Setpoint                 | −50 to +250 °C                       |
| 5        | P gain                   | float                                |
| 6        | I gain                   | float                                |
| 7        | D gain                   | float                                |
| 106      | Sensor enable bitmask    | 48 = TC1+TC2 enabled                 |
| 109      | Control sensor           | 4=TC1 (sensor E), 5=TC2 (sensor F)  |

---

## 7. Linux Control — Quick Start

Install dependency:
```bash
pip install pyserial rich --break-system-packages
```

Run the control UI:
```bash
python3 peltier_control.py
```

The UI polls both thermocouples every second and lets you:
- Switch control mode (OFF / Manual / PID)
- Set target temperature or manual PWM %
- Adjust P, I, D gains
- View live bridge voltage, current, and PWM output

---

## 8. Safety Checklist

- [ ] Supply voltage ≤ 12 V (matches TEC1-12706 rating)
- [ ] Output wires to TEC < 50 cm
- [ ] Hot-side heatsink and fan fitted before running
- [ ] High-temp alarm set for hot side (≤ 65 °C)
- [ ] Low-temp alarm set for cold side (≥ −10 °C)
- [ ] Over-current alarm enabled on bridge
- [ ] Never run TEC without thermal load on both sides (dry-running causes damage)
- [ ] Thermal paste applied between TEC and heatsinks

---

## 9. Typical Workflow

1. Wire everything up (power off).
2. Power on board, connect USB to Linux PC.
3. Run `peltier_control.py`.
4. Verify both thermocouple temperatures read correctly.
5. Set mode to **Manual**, set 50% (neutral) — confirm no movement, then 60% — confirm one side cools.
6. Set mode to **OFF**, set mode to **PID**, set a setpoint near room temperature.
7. Run **Auto-Tune** to get initial P/I/D values.
8. Switch to **PID**, set your target setpoint.
9. Monitor both TC temps in the live display.

<div align="center">

# 🔧 TMotorAPI

**High-level control API for CubeMars T-Motor (AK series)**

[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](#-license)
[![CAN](https://img.shields.io/badge/CAN-socketcan%20%7C%20gs__usb-orange.svg)](#-motorconfig)
[![Based on](https://img.shields.io/badge/based%20on-TMotorCANControl-lightgrey.svg)](https://github.com/neurobionics/TMotorCANControl)

🇺🇸 English · [🇰🇷 한국어](README-KR.md)

</div>

---

## 📖 Overview

A thin wrapper over [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) (Neurobionics Lab), adding **non-blocking control** and a **soft limit**.

| | Feature |
|---|------|
| ⚙️ | MIT-mode position / velocity / torque / full-state control |
| ⚡ | Non-blocking design — commands return instantly, actual bus I/O runs in `update()` |
| 🛡️ | Soft limit — damping ramps up near the bounds for a smooth stop |
| 🖥️ | Linux (socketcan) / Windows (gs_usb) backend support |

---

## 📦 Installation

```bash
pip install git+https://github.com/KR70004526/TMotorAPI.git
```

- **Python** >= 3.8
- **Auto-installed deps** — `python-can`, `gs_usb`, `pyusb`, `libusb-package`, `TMotorCANControl` (gs_usb-enabled fork)
- **Windows** — requires a gs_usb-compatible adapter (e.g. CANable)

---

## 🚀 Quick start

```python
from TMotorAPI import Motor, MotorConfig
import time, math

cfg = MotorConfig(
    motorType="AK70-10",
    motorId=1,
    canBackend="gs_usb",   # Linux: "socketcan"
    canChannel=0,          # Linux: "can0"
    bitrate=1_000_000,
    maxTemperature=80,
    autoInit=False,        # False on Windows (no ip link needed)
)

with Motor(config=cfg) as motor:      # enable on enter, disable on exit
    motor.zero_position()             # set current position to 0
    while True:
        motor.update()                # receive state + send command
        motor.set_torque(1.0)         # e.g. 1 Nm torque
        time.sleep(0.002)             # 500 Hz loop
```

> ⚠️ **Control loop** — every cycle, issue a `set_*()` command then call `update()`. Only then does the actual CAN transfer happen.

---

## ⚙️ MotorConfig

| Field | Default | Description |
|------|--------|------|
| `motorType` | `'AK70-10'` | Motor model (`AK80-64`, `AK80-9`, …) |
| `motorId` | `1` | CAN ID (0–127) |
| `bitrate` | `1000000` | CAN bitrate |
| `maxTemperature` | `50.0` | Safe MOSFET temperature limit (°C) |
| `defaultKp` / `defaultKd` | `10.0` / `0.5` | Default position / velocity gains |
| `canBackend` | `'socketcan'` | `socketcan` (Linux) or `gs_usb` (Windows) |
| `canChannel` | `'can0'` | `'can0'` string for socketcan, `0` int for gs_usb |
| `autoInit` | `True` | Auto bring-up of the CAN interface at startup (Linux only) |

> Backend and channel type are validated on construction — socketcan needs a `'canN'` string, gs_usb needs an integer.

---

## 🎮 Control methods (non-blocking)

| Method | Description |
|--------|------|
| `set_position(pos, kp, kd, feedTor)` | Position control |
| `set_velocity(vel, kd)` | Velocity control |
| `set_torque(tor)` | Torque control |
| `set_fullState(pos, vel, kp, kd, feedTor)` | MIT full-state: `tau = kp·(pos−p) + kd·(vel−v) + feedTor` |
| `set_position_smooth(pos, ...)` | Position control with soft limit applied |
| `stop()` | Emergency stop (torque 0) |
| `zero_position()` | Set current position to 0 (encoder reset, ~1 s EEPROM save) |
| `update()` | Send command + receive state (call every loop) |

> Units — position rad, velocity rad/s, torque Nm

### 📊 State readouts (properties)

`position` · `velocity` · `torque` · `temperature` · `current` (q-axis current, A) · `acceleration` · `error` (0 = OK) · `control_mode`

> The dict returned by `update()` carries the same values in one shot.

---

## 🛡️ Soft limit

Damping (kd) ramps up automatically near the bounds, stopping like a soft wall.

```python
motor.set_soft_limit(-1.57, 1.57)     # -90° ~ +90°
while True:
    motor.set_position_smooth(target) # clamped if outside the limit
    motor.update()
    time.sleep(0.01)
```

> `set_soft_limit(min, max, soft_zone=0.2, base_kd=2.0, max_kd=20.0)` — inside `soft_zone`, kd grows linearly from `base_kd` to `max_kd`.

---

## 📈 Trajectory generation (TrajectoryGenerator)

Static helpers that compute `(position, velocity)` toward a target over time.

```python
from TMotorAPI import TrajectoryGenerator as Traj
pos, vel = Traj.minimum_jerk(start, end, t, duration)
```

| Method | Type |
|--------|------|
| `minimum_jerk` | 5th-order polynomial (minimum jerk) |
| `cubic` | 3rd-order polynomial |
| `linear` | Linear interpolation |

---

## 📁 Example

`demos/TEMPLATE.ipynb` — a real-world template with a 500 Hz control loop, status printing on a separate thread, Windows 1 ms timer resolution, and precise sleep.

---

## 📄 License

MIT

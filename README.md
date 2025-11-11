# TMotor Control API v2.0

Professional control library for AK-series TMotors (MIT CAN protocol)

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🌟 Features

- **4 Control Modes**: Trajectory, Velocity, Torque, and Impedance control
- **Simple API**: Easy-to-use high-level interface
- **Context Manager**: Automatic power management
- **Multi-Motor Support**: Synchronized control of multiple motors
- **Power Monitoring**: Track motor uptime and connection status
- **Type Hints**: Full type annotation for better IDE support
- **Comprehensive Logging**: Detailed operation logs

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Control Modes](#control-modes)
- [API Reference](#api-reference)
- [Examples](#examples)
- [FAQ](#faq)
- [Troubleshooting](#troubleshooting)

## 🚀 Installation

### Prerequisites

```bash
# Install TMotorCANControl library
pip install TMotorCANControl

# Setup CAN interface (one-time)
sudo apt-get install can-utils
```

### Setup Sudo Permission (Recommended)

```bash
sudo visudo
# Add this line:
your_username ALL=(ALL) NOPASSWD: /sbin/ip
```

### Install This Library

```bash
# Just copy tmotor_control_final.py to your project
cp tmotor_control_final.py your_project/
```

## ⚡ Quick Start

### Basic Usage

```python
from tmotor_control_final import Motor

# Create and use motor with context manager (recommended)
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Motor is powered ON here
    motor.track_trajectory(1.57)  # Move to 1.57 rad
    # Motor is automatically powered OFF when exiting
```

### Understanding Power Management

**IMPORTANT**: The motor power works in two stages:

1. **Object Creation** (Connection, Power OFF)
```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)
# TMotorManager object created
# CAN connection established
# Motor power is still OFF (motor won't move)
```

2. **Enable/With Block** (Power ON)
```python
with motor as m:  # __enter__() called → enable() → Power ON
    # Motor is powered ON now
    m.track_trajectory(1.57)
    # Power remains ON throughout the with block
# __exit__() called → disable() → Power OFF
```

### Manual Control

```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)

motor.enable()  # Power ON - Motor can move now
print(f"Power status: {motor.is_power_on()}")  # True

motor.track_trajectory(1.57)

motor.disable()  # Power OFF
print(f"Power status: {motor.is_power_on()}")  # False
```

## 🎯 Control Modes

### Overview

| Mode | Function | Use Case |
|------|----------|----------|
| **1. Trajectory** | `track_trajectory()` | Position control, smooth motion |
| **2. Velocity** | `set_velocity()` | Constant velocity rotation |
| **3. Torque** | `set_torque()` | Force control, gravity compensation |
| **4. Impedance** | `send_command()` | Low-level full control (expert) |

### Mode 1: Trajectory Control

Position control with automatic trajectory generation.

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Immediate move (step position)
    motor.track_trajectory(1.57)
    
    # Smooth trajectory (2 seconds)
    motor.track_trajectory(1.57, duration=2.0)
    
    # Adjust stiffness
    motor.track_trajectory(1.57, kp=50, kd=2.0)  # Stiff
    motor.track_trajectory(1.57, kp=5, kd=0.3)   # Compliant
```

**Parameters:**
- `position`: Target position (rad)
- `kp`: Position gain (Nm/rad) - Higher = stiffer
- `kd`: Velocity gain (Nm/(rad/s)) - Higher = more damped
- `duration`: Motion time (s) - 0 for immediate, >0 for trajectory
- `trajectory_type`: 'minimum_jerk', 'cubic', 'linear'

### Mode 2: Velocity Control

Constant velocity rotation (no feedforward torque).

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Rotate at 2 rad/s
    motor.set_velocity(2.0)
    time.sleep(10)
    
    # Stop
    motor.set_velocity(0.0)
```

### Mode 3: Torque Control

Pure torque control without position/velocity feedback.

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Operating torque
    motor.set_torque(3.5)
    
    # Remove torque (free motion)
    motor.set_torque(0.0)
```

### Mode 4: Impedance Control (Low-level)

Full manual control of all parameters (expert mode).

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Position + FF torque
    motor.send_command(
        position=motor.position,
        kp=0, kd=5.0,
        torque=gravity_comp
    )
    
    # MPC controller integration
    for _ in range(1000):
        p, v, kp, kd, tau = mpc.compute()
        motor.send_command(p, v, kp, kd, tau)
        time.sleep(0.01)
```

## 📚 API Reference

### Motor Class

#### Initialization

```python
Motor(motor_type='AK80-64', motor_id=1, can_interface='can0', 
      auto_init=False, config=None)
```

**Parameters:**
- `motor_type`: Motor model ('AK80-64', 'AK80-9', etc.)
- `motor_id`: CAN ID (1-32)
- `can_interface`: CAN interface name ('can0', 'can1', etc.)
- `auto_init`: If True, calls initialize() automatically
- `config`: MotorConfig object for advanced settings

#### Power Management

```python
motor.enable()   # Power ON
motor.disable()  # Power OFF
motor.is_power_on()  # Check power status
motor.get_uptime()   # Get time since power on
```

#### Control Methods

```python
# Trajectory control
motor.track_trajectory(position, kp=10, kd=0.5, duration=0.0, 
                      trajectory_type='minimum_jerk')

# Velocity control
motor.set_velocity(velocity, kd=5.0)

# Torque control
motor.set_torque(torque)

# Low-level control
motor.send_command(position, velocity, kp, kd, torque=0.0)
```

#### Utility Methods

```python
motor.update()          # Read motor state
motor.zero_position()   # Set current position as zero
motor.check_connection()  # Check if motor is responding
```

#### Properties

```python
motor.position       # Current position (rad)
motor.velocity       # Current velocity (rad/s)
motor.torque         # Current torque (Nm)
motor.temperature    # Current temperature (°C)
motor.is_enabled     # Power status
```

### MotorGroup Class

```python
# Create motor group
motors = MotorGroup([
    ('AK80-64', 1),
    ('AK80-64', 2),
    ('AK80-9', 3)
])

# Use with context manager
with motors:
    # Synchronized motion
    motors.track_all_trajectory([1.57, 0.0, -1.57], duration=2.0)
    
    # Individual control
    motors[0].set_velocity(2.0)
    motors[1].track_trajectory(1.0)
```

## 💡 Examples

### Example 1: Basic Position Control

```python
from tmotor_control_final import Motor

with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Move to 1.57 rad
    motor.track_trajectory(1.57)
    time.sleep(2)
    
    # Return to 0
    motor.track_trajectory(0.0)
```

### Example 2: Smooth Trajectory

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 5 second smooth motion
    motor.track_trajectory(3.14, duration=5.0)
```

### Example 3: Velocity Control

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Rotate for 10 seconds
    motor.set_velocity(2.0)
    time.sleep(10)
    motor.set_velocity(0.0)
```

### Example 4: Multiple Motors

```python
from tmotor_control_final import MotorGroup

motors = MotorGroup([
    ('AK80-64', 1),
    ('AK80-64', 2),
    ('AK80-9', 3)
])

with motors:
    # Move all together
    motors.track_all_trajectory([1.57, 0.0, -1.57], duration=2.0)
    
    # Check positions
    print(f"Positions: {motors.get_positions()}")
```

### Example 5: Power Status Monitoring

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    print(f"Power ON: {motor.is_power_on()}")  # True
    print(f"Uptime: {motor.get_uptime():.2f}s")
    
    motor.track_trajectory(1.57)
    
    print(f"Still ON: {motor.is_power_on()}")  # True
    print(f"Uptime: {motor.get_uptime():.2f}s")

# After with block
print(f"Power OFF: {motor.is_power_on()}")  # False
```

### Example 6: MPC Integration

```python
from your_mpc_library import MPCController

mpc = MPCController()

with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    for _ in range(1000):
        # Compute optimal control
        p, v, kp, kd, tau = mpc.compute(
            current_state=motor.position,
            target_state=1.57
        )
        
        # Apply control
        motor.send_command(p, v, kp, kd, tau)
        
        time.sleep(0.01)
```

## ❓ FAQ

### Q1: When is the motor actually powered on?

**A:** The motor is powered on when `enable()` is called or when entering a `with` block:

```python
motor = Motor(..., auto_init=True)  # Connected, but power OFF
motor.enable()  # NOW power is ON
```

```python
with Motor(...) as motor:  # Power ON here
    pass  # Power OFF here
```

### Q2: How long does the power stay on?

**A:** The power stays on until:
1. `disable()` is called, or
2. The `with` block exits

```python
with motor:
    # Power ON throughout this entire block
    motor.track_trajectory(1.57)
    time.sleep(10)
    motor.set_velocity(2.0)
    time.sleep(20)
    # Power still ON here
# Power OFF when exiting
```

### Q3: What's the difference between `auto_init=True` and `auto_init=False`?

**A:**
- `auto_init=True`: Creates TMotorManager object immediately (connects to motor, power OFF)
- `auto_init=False`: TMotorManager created on first `enable()` call

Both result in power OFF initially. Power only turns ON with `enable()` or `with` block.

### Q4: Can I use `enable()` and `with` together?

**A:** Not recommended! This will call `enable()` twice:

```python
# ❌ Don't do this
motor = Motor(...)
motor.enable()  # First enable
with motor:     # Second enable (bad!)
    pass
```

**✅ Choose one:**
```python
# Method 1: Manual
motor.enable()
motor.track_trajectory(1.57)
motor.disable()

# Method 2: Context manager (recommended)
with motor:
    motor.track_trajectory(1.57)
```

### Q5: How do I check if the motor is still powered on?

**A:** Use these methods:
```python
motor.is_power_on()      # True if powered on
motor.get_uptime()       # Seconds since power on
motor.check_connection() # Check if responding
```

## 🔧 Troubleshooting

### Motor not responding

```python
# Check connection
if not motor.check_connection():
    print("Motor not responding!")
    motor.disable()
    time.sleep(1)
    motor.enable()
```

### Excessive vibration

```python
# Increase damping (kd)
motor.track_trajectory(1.57, kp=10, kd=2.0)  # Higher kd
```

### Too compliant

```python
# Increase stiffness (kp)
motor.track_trajectory(1.57, kp=50, kd=2.0)  # Higher kp
```

### CAN interface not found

```bash
# Check interface
ip link show can0

# Bring up manually
sudo ip link set can0 up type can bitrate 1000000
```

### Permission denied

```bash
# Setup sudo permission (see Installation section)
sudo visudo
# Add: your_username ALL=(ALL) NOPASSWD: /sbin/ip
```

## 📝 Gain Tuning Guide

| Purpose | Kp | Kd | Characteristics |
|---------|----|----|-----------------|
| Precision control | 50 | 2.0 | Stiff, accurate |
| General control | 10 | 0.5 | Default, balanced |
| Safe interaction | 5 | 0.3 | Compliant, soft |
| Velocity control | 0 | 5.0 | Position control OFF |
| Torque control | 0 | 0 | All feedback OFF |

**Tuning tips:**
- Vibration → Increase Kd
- Too soft → Increase Kp
- Overshoot → Increase Kd
- Slow response → Increase Kp

## 🎓 Control Theory

All control modes use the same impedance control equation:

```
τ = Kp × (pos_target - pos_actual) + 
    Kd × (vel_target - vel_actual) + 
    τ_feedforward
```

Each mode is just a different combination of these 5 parameters:

| Mode | pos_target | vel_target | Kp | Kd | τ_ff |
|------|------------|------------|----|----|------|
| Position | target | 0 | ✓ | ✓ | 0 |
| Velocity | current | target | 0 | ✓ | 0 |
| Torque | 0 | 0 | 0 | 0 | ✓ |
| Impedance | ✓ | ✓ | ✓ | ✓ | ✓ |

## 📄 License

MIT License - See LICENSE file for details

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📧 Contact

For issues and questions, please open an issue on GitHub.

## 🙏 Acknowledgments

Based on [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) by Neurobionics Lab

---

**Happy Controlling! 🚀**

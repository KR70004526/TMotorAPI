# TMotor Control API

A high-level Python library for controlling AK-series T-Motors using the MIT CAN protocol.

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🌟 Features

- **Simple & Intuitive API**: Easy-to-use high-level interface built on TMotorCANControl
- **4 Control Modes**: Trajectory, Velocity, Torque, and Impedance control
- **Context Manager Support**: Automatic power management with Python's `with` statement
- **Multi-Motor Control**: Synchronize multiple motors with `MotorGroup`
- **Type Hints**: Full type annotations for better IDE support
- **Detailed Logging**: Comprehensive operation logs for debugging
- **Power Monitoring**: Track motor uptime and connection status
- **Auto CAN Setup**: Automatic CAN interface initialization (optional)

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Control Modes](#control-modes)
- [API Reference](#api-reference)
- [Examples](#examples)
- [FAQ](#faq)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)

## 🚀 Installation

### Prerequisites

```bash
# Install TMotorCANControl library
pip install TMotorCANControl

# Install CAN utilities (Linux)
sudo apt-get install can-utils
```

### Setup Sudo Permissions (Recommended)

To allow automatic CAN interface setup without password prompts:

```bash
sudo visudo
# Add this line:
your_username ALL=(ALL) NOPASSWD: /sbin/ip
```

### Install TMotorAPI

```bash
# Clone the repository
git clone https://github.com/KR70004526/TMotorAPI.git
cd TMotorAPI

# Copy to your project
cp src/TMotorAPI.py your_project/
```

## ⚡ Quick Start

### Basic Usage

```python
from TMotorAPI import Motor

# Create and use motor with context manager (recommended)
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # Motor is powered on inside this block
    motor.track_trajectory(1.57)  # Move to 1.57 rad
    # Motor is automatically powered off when exiting
```

### Understanding Power Management

**Important**: Motor power operates in 2 stages:

1. **Object Creation** (Connection, Power OFF)
```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)
# TMotorManager object created
# CAN connection established
# Motor power is still OFF (motor won't move yet)
```

2. **Enable/With Block** (Power ON)
```python
with motor as m:  # __enter__() called → enable() → Power ON
    # Motor is now powered and ready to move
    m.track_trajectory(1.57)
    # Power remains on throughout the with block
# __exit__() called → disable() → Power OFF
```

### Manual Control

```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)

motor.enable()  # Power ON - motor can now move
print(f"Power status: {motor.is_power_on()}")  # True

motor.track_trajectory(1.57)
motor.set_velocity(2.0)

motor.disable()  # Power OFF
print(f"Power status: {motor.is_power_on()}")  # False
```

## 🎯 Control Modes

### Overview

| Mode | Function | Use Case |
|------|----------|----------|
| **1. Trajectory** | `track_trajectory()` | Position control, smooth motion |
| **2. Velocity** | `set_velocity()` | Constant speed rotation |
| **3. Torque** | `set_torque()` | Force control, gravity compensation |
| **4. Impedance** | `set_impedance()` | Compliant interaction, stiffness control |

### Mode Details

#### 1. Trajectory Control
Position tracking with velocity control.

```python
motor.track_trajectory(
    position=1.57,      # Target position (rad)
    velocity=2.0,       # Target velocity (rad/s)
    kp=50.0,           # Position gain (Nm/rad)
    kd=2.0             # Velocity gain (Nm/(rad/s))
)

# Check if target reached
if motor.is_at_target(tolerance=0.01):  # Within 0.01 rad
    print("Target reached!")
```

**When to use**: Precise positioning tasks, trajectory following

#### 2. Velocity Control
Direct velocity command.

```python
motor.set_velocity(
    velocity=3.0,      # Target velocity (rad/s)
    torque_limit=5.0   # Maximum torque (Nm)
)
```

**When to use**: Continuous rotation, speed-based tasks

#### 3. Torque Control
Direct torque/force control.

```python
motor.set_torque(
    torque=2.5         # Desired torque (Nm)
)
```

**When to use**: Force control, gravity compensation, haptics

#### 4. Impedance Control
Virtual spring-damper system.

```python
motor.set_impedance(
    position=0.0,      # Equilibrium position (rad)
    kp=30.0,          # Stiffness (Nm/rad)
    kd=1.0            # Damping (Nm/(rad/s))
)
```

**When to use**: Human-robot interaction, compliant manipulation

## 📚 API Reference

### Motor Class

#### Constructor

```python
Motor(
    motor_type: str,           # Motor model ('AK80-64', 'AK80-9', 'AK80-10', 'AK70-10')
    motor_id: int = 1,         # CAN ID (1-32)
    can_interface: str = 'can0',  # CAN interface name
    auto_init: bool = True,    # Auto initialize CAN interface
    bitrate: int = 1000000,    # CAN bitrate (default: 1Mbps)
    max_temperature: float = 50.0  # Max safe temperature (°C)
)
```

#### Configuration Object

```python
from TMotorAPI import MotorConfig

config = MotorConfig(
    motorType='AK80-64',
    motorId=2,
    canInterface='can0',
    bitrate=1000000,
    autoInit=True,
    maxTemperature=50.0
)

motor = Motor(config=config)
```

#### Control Methods

```python
# Trajectory control
motor.track_trajectory(position, velocity=0.0, kp=None, kd=None)

# Velocity control
motor.set_velocity(velocity, torque_limit=None)

# Torque control
motor.set_torque(torque)

# Impedance control
motor.set_impedance(position=0.0, kp=None, kd=None)

# Zero position
motor.set_zero_position()
```

#### State Methods

```python
# Get current state
state = motor.update()  # Returns dict with position, velocity, torque, temperature

# Access cached state
pos = motor.position
vel = motor.velocity
temp = motor.temperature

# Check status
motor.is_power_on()
motor.is_at_target(tolerance=0.01)
motor.get_uptime()  # Time since enable() was called
```

#### Power Management

```python
motor.enable()   # Power on
motor.disable()  # Power off

# Context manager (automatic power management)
with motor:
    # Motor powered on
    pass
# Motor powered off
```

### MotorGroup Class

Control multiple motors synchronously.

```python
from TMotorAPI import MotorGroup

# Create motor group
group = MotorGroup([
    Motor('AK80-64', motor_id=1),
    Motor('AK80-64', motor_id=2),
    Motor('AK80-9', motor_id=3)
])

# Enable all motors
with group:
    # Set trajectory for all motors
    group.track_trajectory_all([0.0, 1.57, 3.14])
    
    # Set different velocities
    group.set_velocity_all([1.0, 2.0, 1.5])
    
    # Update all motors
    states = group.update_all()
    print(states[0]['position'])  # First motor position
```

## 💡 Examples

### Example 1: Simple Position Control

```python
from TMotorAPI import Motor
import time

with Motor('AK80-64', motor_id=1, auto_init=True) as motor:
    # Move to 90 degrees
    motor.track_trajectory(1.57)  # π/2 rad
    time.sleep(2)
    
    # Move to -90 degrees
    motor.track_trajectory(-1.57)
    time.sleep(2)
    
    # Return to zero
    motor.track_trajectory(0.0)
```

### Example 2: Velocity Control with Monitoring

```python
from TMotorAPI import Motor
import time

motor = Motor('AK80-9', motor_id=2, auto_init=True)
motor.enable()

try:
    # Rotate at 3 rad/s for 5 seconds
    motor.set_velocity(3.0)
    
    for _ in range(50):  # 5 seconds at 10Hz
        state = motor.update()
        print(f"Velocity: {state['velocity']:.2f} rad/s, "
              f"Temperature: {state['temperature']:.1f}°C")
        time.sleep(0.1)
        
finally:
    motor.disable()
```

### Example 3: Multi-Motor Synchronized Control

```python
from TMotorAPI import MotorGroup, Motor
import numpy as np
import time

# Create 3 motors
motors = [
    Motor('AK80-64', motor_id=1),
    Motor('AK80-64', motor_id=2),
    Motor('AK80-64', motor_id=3)
]

group = MotorGroup(motors)

with group:
    # Sinusoidal trajectory
    t = 0
    while t < 10:  # 10 seconds
        positions = [
            np.sin(2 * np.pi * 0.5 * t),      # Motor 1: 0.5 Hz
            np.sin(2 * np.pi * 0.5 * t + np.pi/3),  # Motor 2: phase shifted
            np.sin(2 * np.pi * 0.5 * t + 2*np.pi/3) # Motor 3: phase shifted
        ]
        
        group.track_trajectory_all(positions)
        time.sleep(0.01)  # 100 Hz control loop
        t += 0.01
```

### Example 4: Impedance Control for Compliance

```python
from TMotorAPI import Motor
import time

with Motor('AK70-10', motor_id=1, auto_init=True) as motor:
    # Set up soft virtual spring at position 0
    motor.set_impedance(
        position=0.0,
        kp=10.0,   # Low stiffness = soft spring
        kd=0.5     # Low damping = less resistance
    )
    
    # Let user move the motor manually
    print("Try moving the motor manually...")
    for _ in range(100):
        state = motor.update()
        print(f"Position: {state['position']:.3f} rad, "
              f"Torque: {state['torque']:.3f} Nm")
        time.sleep(0.1)
```

### Example 5: Temperature Monitoring

```python
from TMotorAPI import Motor
import time

motor = Motor('AK80-64', motor_id=1, auto_init=True, max_temperature=45.0)

with motor:
    motor.set_velocity(5.0)  # High speed
    
    while True:
        state = motor.update()
        temp = state['temperature']
        
        print(f"Temperature: {temp:.1f}°C")
        
        # Automatic safety check (built-in)
        if temp > motor._config.maxTemperature:
            print("WARNING: Temperature limit exceeded!")
            break
            
        time.sleep(1.0)
```

## ❓ FAQ

### Q: What motors are supported?

**A:** Currently supports AK-series motors with MIT CAN protocol:
- AK80-64 (high torque)
- AK80-9 (balanced)
- AK80-10 (high speed)
- AK70-10 (compact)

### Q: Can I control multiple motors on different CAN interfaces?

**A:** Yes! Specify different `can_interface` for each motor:

```python
motor1 = Motor('AK80-64', motor_id=1, can_interface='can0')
motor2 = Motor('AK80-9', motor_id=1, can_interface='can1')
```

### Q: Do I need to call `update()` regularly?

**A:** Yes, for continuous control. The control methods send commands, but `update()` reads motor feedback:

```python
while running:
    motor.track_trajectory(target_pos)
    state = motor.update()  # Get latest feedback
    time.sleep(0.01)  # 100 Hz recommended
```

### Q: How do I handle motor errors?

**A:** The API includes error handling and logging:

```python
try:
    with motor:
        motor.set_velocity(10.0)
        state = motor.update()
        
        if not state:  # Empty dict means error
            print("Communication error!")
            
except Exception as e:
    print(f"Error: {e}")
finally:
    motor.disable()  # Always safe to call
```

### Q: What's the difference between `track_trajectory()` and `set_impedance()`?

**A:**
- `track_trajectory()`: Active position tracking (stiff, precise)
- `set_impedance()`: Passive compliance (soft, interactive)

Use trajectory for precise positioning, impedance for safe interaction.

## 🔧 Troubleshooting

### CAN Interface Not Found

```bash
# Check if interface exists
ip link show can0

# If not, add Device Tree Overlay (Raspberry Pi)
sudo nano /boot/config.txt
# Add: dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25

# Reboot
sudo reboot
```

### Motor Not Responding

1. **Check power**: Verify motor power supply (24-48V depending on model)
2. **Check CAN bus**: Ensure proper termination resistors (120Ω at each end)
3. **Check ID**: Verify motor CAN ID matches code
4. **Check enable**: Make sure `enable()` was called or using `with` statement

```python
# Debug mode
logging.basicConfig(level=logging.DEBUG)
motor = Motor('AK80-64', motor_id=1)
```

### Permission Denied

```bash
# Add user to CAN group
sudo usermod -a -G dialout $USER

# Or run with sudo (not recommended)
sudo python3 your_script.py
```

### High Temperature Warning

- Reduce load or duty cycle
- Improve cooling (add heatsink/fan)
- Lower `max_temperature` threshold for earlier warning

## 🏗️ Architecture

```
User Application
       ↓
   TMotorAPI (This library)
       ↓ wraps
TMotorCANControl (Low-level)
       ↓
   CAN Bus
       ↓
   T-Motor
```

**Design Philosophy:**
- **TMotorCANControl**: Direct CAN protocol implementation (low-level)
- **TMotorAPI**: High-level abstractions and safety features (user-friendly)

## 📝 License

MIT License - See [LICENSE](LICENSE) file for details

## 🙏 Acknowledgments

This library is built on [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) by the Neurobionics Lab.

Special thanks to:
- [Neurobionics Lab](https://github.com/neurobionics) for the underlying CAN control library
- MIT for the open CAN protocol specification
- T-Motor for excellent motor hardware

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/KR70004526/TMotorAPI/issues)
- **Documentation**: [Wiki](https://github.com/KR70004526/TMotorAPI/wiki)
- **Base Library**: [TMotorCANControl Docs](https://github.com/neurobionics/TMotorCANControl)

---

**Happy Controlling! 🚀**

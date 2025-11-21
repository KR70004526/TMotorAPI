# TMotorAPI v4.3

A high-level Python library for controlling AK-series T-Motors using the MIT CAN protocol.

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-4.3-green.svg)](https://github.com/KR70004526/TMotorAPI)

## 🆕 What's New in v4.3

### Settling Time Logic for Step Commands
- **Robust Position Control**: Position must stay within tolerance for N consecutive cycles
- **Drift Prevention**: Prevents premature return when using feedforward torque
- **Automatic Settling Detection**: Monitors stability before confirming arrival
- **Enhanced Logging**: Real-time settling progress updates

### Key Improvements
```python
# Before v4.3: Simple tolerance check (could return prematurely)
motor.set_position(1.57, duration=0.0)  # Returns immediately when within tolerance

# v4.3: Settling time verification (more robust)
motor.set_position(1.57, duration=0.0, feedTor=2.0)  
# → Position must be stable for 0.1s (configurable) before returning
# → Prevents drift from feedforward torque
```

## 🌟 Features

- **4 Control Modes**: Position, Velocity, Torque, and Advanced Impedance control
- **Settling Time Logic**: Robust step command behavior with stability verification (NEW in v4.3)
- **Feedforward Torque**: Gravity compensation support for position control (NEW in v4.3)
- **Context Manager Support**: Automatic power management with Python's `with` statement
- **Type Hints**: Full type annotations for better IDE support
- **Detailed Logging**: Comprehensive operation logs with settling progress tracking
- **Auto CAN Setup**: Automatic CAN interface initialization (optional)
- **Trajectory Planning**: Minimum jerk, cubic, and linear trajectory generators

## 📋 Table of Contents

- [What's New in v4.3](#-whats-new-in-v43)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Control Modes](#-control-modes)
- [Advanced Features](#-advanced-features)
- [Configuration](#-configuration)
- [API Reference](#-api-reference)
- [Examples](#-examples)
- [Troubleshooting](#-troubleshooting)

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

### Basic Usage with Context Manager

```python
from TMotorAPI import Motor

# Create and use motor with context manager (recommended)
with Motor('AK80-64', motorId=2, autoInit=True) as motor:
    # Motor is powered on inside this block
    motor.set_position(1.57)  # Move to 1.57 rad with settling verification
    # Motor is automatically powered off when exiting
```

### Understanding Power Management

**Important**: Motor power operates in 2 stages:

#### 1. Object Creation (Connection, Power OFF)
```python
motor = Motor('AK80-64', motorId=2, autoInit=True)
# TMotorManager object created
# CAN connection established
# Motor power is still OFF (motor won't move yet)
```

#### 2. Enable/With Block (Power ON)
```python
with motor as m:  # __enter__() called → enable() → Power ON
    # Motor is now powered and ready to move
    m.set_position(1.57)
    # Power remains on throughout the with block
# __exit__() called → disable() → Power OFF
```

### Manual Power Control

```python
motor = Motor('AK80-64', motorId=2, autoInit=True)

motor.enable()  # Power ON - motor can now move
print(f"Power status: {motor.is_power_on()}")  # True

motor.set_position(1.57)

motor.disable()  # Power OFF
print(f"Power status: {motor.is_power_on()}")  # False
```

## 🎯 Control Modes

### Overview

| Mode | Function | Use Case |
|------|----------|----------|
| **Position** | `set_position()` | Position control with trajectory planning |
| **Velocity** | `set_velocity()` | Constant speed rotation |
| **Torque** | `set_torque()` | Force control, gravity compensation |

### 1. Position Control (Enhanced in v4.3)

Position tracking with trajectory planning and settling verification.

#### Step Command (duration ≤ 0.02s)
```python
# Simple step command with settling verification
motor.set_position(
    targetPos=1.57,      # Target position (rad)
    duration=0.0,        # Step command (no trajectory)
    kp=10.0,             # Position gain (Nm/rad)
    kd=2.0,              # Velocity gain (Nm/(rad/s))
    feedTor=0.0          # Feedforward torque (Nm) - NEW!
)
```

**Settling Behavior:**
```
Step: 0.000 → 1.570 rad
  Settling: 0.100s (10 cycles @ 100Hz)
    Settling: 10% (1/10)
    Settling: 20% (2/10)
    ...
    Settling: 100% (10/10)
  ✓ Reached and STABLE in 0.15s
    Final position: 1.570 rad
    Final error: 0.0023 rad
```

#### Trajectory Command (duration > 0.02s)
```python
# Smooth trajectory with minimum jerk
motor.set_position(
    targetPos=1.57,          # Target position (rad)
    duration=2.0,            # Motion duration (s)
    kp=10.0,                 # Position gain (Nm/rad)
    kd=2.0,                  # Velocity gain (Nm/(rad/s))
    feedTor=0.0,             # Feedforward torque (Nm)
    trajectoryType='minimum_jerk'  # 'minimum_jerk', 'cubic', 'linear'
)
```

**Trajectory Types:**
- `'minimum_jerk'`: Smooth 5th-order polynomial (default, best for human interaction)
- `'cubic'`: 3rd-order polynomial (faster, less smooth)
- `'linear'`: Linear interpolation (fastest, least smooth)

#### Feedforward Torque (Gravity Compensation)
```python
# Example: Compensate for gravity on a horizontal arm
import numpy as np

# Calculate gravity torque: τ = m * g * L * cos(θ)
mass = 2.0  # kg
g = 9.81    # m/s²
length = 0.3  # m

def gravity_torque(angle):
    return mass * g * length * np.cos(angle)

# Move with gravity compensation
target_angle = 1.57
motor.set_position(
    targetPos=target_angle,
    duration=0.0,
    feedTor=gravity_torque(target_angle)  # Compensate gravity
)
```

**When to use:** 
- **Step command**: Quick positioning with stability verification
- **Trajectory**: Smooth motion, predictable velocity profile
- **Feedforward**: Gravity compensation, load compensation

### 2. Velocity Control

Direct velocity command.

```python
motor.set_velocity(
    targetVel=3.0,      # Target velocity (rad/s)
    kd=5.0,             # Velocity gain (Nm/(rad/s))
    duration=2.0        # Motion duration (s), 0 for continuous
)
```

**When to use**: Continuous rotation, speed-based tasks

### 3. Torque Control

Direct torque/force control.

```python
motor.set_torque(
    targetTor=2.5,        # Desired torque (Nm)
    duration=2.0          # Motion duration (s), 0 for single command
)
```

**When to use**: Force control, compliance, haptics

## 🔬 Advanced Features

### Settling Time Configuration

Control how strictly the motor verifies position stability:

```python
from TMotorAPI import MotorConfig

config = MotorConfig(
    motorType='AK80-64',
    motorId=2,
    stepTimeout=5.0,        # Max time for step command (s)
    stepTolerance=0.05,     # Position tolerance (rad, ±2.87°)
    stepSettlingTime=0.1    # Required stability duration (s)
)

motor = Motor(config=config)
```

**Settling Time Guidelines:**
- `0.0s`: No settling check (returns immediately when within tolerance)
- `0.05s - 0.1s`: Light verification (5-10 cycles @ 100Hz)
- `0.2s - 0.5s`: Strong verification (20-50 cycles)
- `> 0.5s`: Very strict verification (rare, high-precision applications)

### Trajectory Planning

Built-in trajectory generators for smooth motion:

```python
# Minimum jerk (5th order) - smoothest, best for human interaction
motor.set_position(1.57, duration=2.0, trajectoryType='minimum_jerk')

# Cubic (3rd order) - faster, less smooth
motor.set_position(1.57, duration=2.0, trajectoryType='cubic')

# Linear - fastest, no acceleration smoothing
motor.set_position(1.57, duration=2.0, trajectoryType='linear')
```

### Zeroing Position

Set current position as zero reference:

```python
motor.zero_position()
# Current position is now considered 0.0 rad
```

### State Monitoring

```python
# Get current state (triggers update)
state = motor.update()
print(f"Position: {state['position']:.3f} rad")
print(f"Velocity: {state['velocity']:.3f} rad/s")
print(f"Torque: {state['torque']:.3f} Nm")
print(f"Temperature: {state['temperature']:.1f} °C")

# Access cached state (no communication)
pos = motor.position
vel = motor.velocity
temp = motor.temperature

# Check motor status
is_on = motor.is_power_on()
uptime = motor.get_uptime()  # Seconds since enable()
connected = motor.check_connection()
```

## ⚙️ Configuration

### MotorConfig Class

Complete configuration object for motor parameters:

```python
from TMotorAPI import MotorConfig

config = MotorConfig(
    # Motor identification
    motorType='AK80-64',        # 'AK80-64', 'AK80-9', 'AK70-10'
    motorId=2,                  # CAN ID (0-127)
    
    # CAN setup (for CANInterface.setup_interface only)
    canInterface='can0',        # 'can0', 'can1', etc.
    bitrate=1000000,            # CAN bitrate (default: 1Mbps)
    autoInit=True,              # Auto setup CAN interface
    
    # Safety
    maxTemperature=50.0,        # Max MOSFET temperature (°C)
    
    # Default control gains
    defaultKp=10.0,             # Position gain (Nm/rad)
    defaultKd=0.5,              # Velocity gain (Nm/(rad/s))
    
    # Step command parameters (NEW in v4.3)
    stepTimeout=5.0,            # Max time for step command (s)
    stepTolerance=0.05,         # Position tolerance (rad)
    stepSettlingTime=0.1        # Required stability duration (s)
)

motor = Motor(config=config)
```

### Parameter Explanation

#### Motor Identification
- **motorType**: Motor model string (must match physical motor)
- **motorId**: CAN ID configured on the motor (0-127)

#### CAN Setup
- **canInterface**: Only used for `CANInterface.setup_interface()`
- **bitrate**: CAN bus speed (default 1Mbps for T-Motors)
- **autoInit**: If `True`, automatically runs CAN interface setup

**Note**: `TMotorManager_mit_can` automatically detects CAN interface after setup.

#### Safety
- **maxTemperature**: Triggers warnings when exceeded (doesn't stop motor)

#### Control Gains
- **defaultKp**: Used when `kp=None` in `set_position()`
- **defaultKd**: Used when `kd=None` in `set_position()` and `set_velocity()`

#### Step Command Tuning (v4.3)
- **stepTimeout**: Prevents infinite waiting if motor can't reach target
- **stepTolerance**: Acceptable position error (±0.05 rad ≈ ±2.87°)
- **stepSettlingTime**: How long position must be stable before confirming

## 📚 API Reference

### Motor Class

#### Constructor

```python
Motor(
    motorType: Optional[str] = None,        # Motor model
    motorId: Optional[int] = None,          # CAN ID (0-127)
    canInterface: Optional[str] = None,     # CAN interface name
    bitrate: Optional[int] = None,          # CAN bitrate
    autoInit: Optional[bool] = None,        # Auto initialize CAN
    maxTemperature: Optional[float] = None, # Max safe temperature (°C)
    config: Optional[MotorConfig] = None,   # Or use config object
    **kwargs
)
```

**Three Ways to Create Motor:**

```python
# Method 1: Direct parameters
motor = Motor('AK80-64', motorId=2, autoInit=True)

# Method 2: Config object
config = MotorConfig(motorType='AK80-64', motorId=2)
motor = Motor(config=config)

# Method 3: Mix both (parameters override config)
config = MotorConfig(motorType='AK80-64', motorId=2)
motor = Motor(config=config, maxTemperature=60.0)  # Override temp
```

#### Control Methods

| Method | Parameters | Description |
|--------|-----------|-------------|
| `set_position()` | `targetPos, duration=0.0, kp=None, kd=None, feedTor=0.0, trajectoryType='minimum_jerk'` | Position control with trajectory planning |
| `set_velocity()` | `targetVel, kd=None, duration=0.0` | Velocity control |
| `set_torque()` | `targetTor, duration=0.0` | Torque control |
| `zero_position()` | - | Set current position as zero |

#### State Methods

```python
# Get current state (triggers CAN communication)
state = motor.update()
# Returns: {'position': float, 'velocity': float, 'torque': float, 'temperature': float}

# Access cached state (no CAN communication)
pos = motor.position        # rad
vel = motor.velocity        # rad/s
tor = motor.torque          # Nm
temp = motor.temperature    # °C

# Check status
motor.is_power_on()         # Returns True/False
motor.get_uptime()          # Seconds since enable()
motor.check_connection()    # Test CAN communication
```

#### Power Management

```python
motor.enable()   # Power on (required before any commands)
motor.disable()  # Power off

# Context manager (automatic power management)
with motor:
    # Motor powered on automatically
    motor.set_position(1.57)
# Motor powered off automatically
```

### CANInterface Class

Manual CAN interface setup (optional, done automatically with `autoInit=True`):

```python
from TMotorAPI import CANInterface

# Setup CAN interface
CANInterface.setup_interface(
    canInterface='can0',
    bitrate=1000000
)

# Or use config
CANInterface.setup_interface(config=motor_config)
```

### TrajectoryGenerator Class

Low-level trajectory planning utilities:

```python
from TMotorAPI import TrajectoryGenerator

# Minimum jerk trajectory
pos, vel = TrajectoryGenerator.minimum_jerk(
    startPos=0.0,
    endPos=1.57,
    currentTime=0.5,
    totalDuration=2.0
)

# Cubic trajectory
pos, vel = TrajectoryGenerator.cubic(
    startPos=0.0,
    endPos=1.57,
    currentTime=0.5,
    totalDuration=2.0
)

# Linear interpolation
pos, vel = TrajectoryGenerator.linear(
    startPos=0.0,
    endPos=1.57,
    currentTime=0.5,
    totalDuration=2.0
)
```

## 💡 Examples

### Example 1: Simple Position Control

```python
from TMotorAPI import Motor

with Motor('AK80-64', motorId=1, autoInit=True) as motor:
    # Quick step command
    motor.set_position(1.57)  # π/2 rad
    
    # Smooth trajectory
    motor.set_position(-1.57, duration=2.0)  # Move back smoothly
    
    # Return to zero
    motor.set_position(0.0, duration=1.5)
```

### Example 2: Gravity Compensation

```python
from TMotorAPI import Motor, MotorConfig
import numpy as np

# Configure motor
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    stepSettlingTime=0.2,  # Stricter settling for gravity comp
    stepTolerance=0.03     # Tighter tolerance
)

# System parameters
mass = 2.0      # kg
g = 9.81        # m/s²
length = 0.3    # m

def gravity_torque(angle):
    """Calculate gravity compensation torque"""
    return mass * g * length * np.cos(angle)

with Motor(config=config) as motor:
    # Zero at horizontal position
    motor.zero_position()
    
    # Move to various angles with gravity compensation
    angles = [0.0, 0.5, 1.0, 1.57, 2.0]
    
    for angle in angles:
        print(f"Moving to {np.degrees(angle):.1f}°...")
        motor.set_position(
            targetPos=angle,
            duration=0.0,
            feedTor=gravity_torque(angle)
        )
        print(f"  Position stable at {motor.position:.3f} rad")
```

### Example 3: Velocity Sweep

```python
from TMotorAPI import Motor
import time

with Motor('AK80-9', motorId=2, autoInit=True) as motor:
    # Velocity sweep test
    velocities = [1.0, 2.0, 3.0, 2.0, 1.0, 0.0]
    
    for vel in velocities:
        print(f"Setting velocity: {vel} rad/s")
        motor.set_velocity(vel, duration=1.0)
        time.sleep(0.5)  # Brief pause between changes
```

### Example 4: Torque Control with Monitoring

```python
from TMotorAPI import Motor
import time

config = MotorConfig(
    motorType='AK70-10',
    motorId=1,
    maxTemperature=45.0  # Lower threshold for safety
)

with Motor(config=config) as motor:
    target_torque = 2.0  # Nm
    duration = 5.0       # seconds
    
    print(f"Applying {target_torque} Nm for {duration}s...")
    
    start_time = time.time()
    motor.set_torque(target_torque, duration=0.0)  # Single command
    
    # Monitor while applying torque
    while time.time() - start_time < duration:
        state = motor.update()
        print(f"Pos: {state['position']:.3f} rad, "
              f"Vel: {state['velocity']:.3f} rad/s, "
              f"Torque: {state['torque']:.3f} Nm, "
              f"Temp: {state['temperature']:.1f} °C")
        time.sleep(0.1)
    
    # Stop torque
    motor.set_torque(0.0)
```

### Example 5: Custom Settling Parameters

```python
from TMotorAPI import Motor, MotorConfig

# High-precision application
precise_config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    stepTolerance=0.01,      # ±0.57° tolerance
    stepSettlingTime=0.5,    # 0.5s stability required
    stepTimeout=10.0         # Allow longer settling time
)

# Fast application (relaxed settling)
fast_config = MotorConfig(
    motorType='AK80-64',
    motorId=2,
    stepTolerance=0.1,       # ±5.73° tolerance
    stepSettlingTime=0.05,   # 50ms stability
    stepTimeout=3.0
)

with Motor(config=precise_config) as motor1, \
     Motor(config=fast_config) as motor2:
    
    # Motor 1: High precision, takes longer
    motor1.set_position(1.57)
    
    # Motor 2: Fast response, less strict
    motor2.set_position(1.57)
```

### Example 6: Trajectory Comparison

```python
from TMotorAPI import Motor
import time

with Motor('AK80-64', motorId=1, autoInit=True) as motor:
    target = 1.57
    duration = 2.0
    
    # Test all trajectory types
    trajectories = ['minimum_jerk', 'cubic', 'linear']
    
    for traj_type in trajectories:
        print(f"\nTesting {traj_type} trajectory...")
        
        motor.set_position(0.0, duration=1.0)  # Reset
        time.sleep(0.5)
        
        motor.set_position(
            targetPos=target,
            duration=duration,
            trajectoryType=traj_type
        )
        time.sleep(0.5)
```

## 🔧 Troubleshooting

### CAN Interface Not Found

```bash
# Check if interface exists
ip link show can0

# If not found on Raspberry Pi, add Device Tree Overlay
sudo nano /boot/firmware/config.txt
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
import logging
logging.basicConfig(level=logging.DEBUG)

motor = Motor('AK80-64', motorId=1, autoInit=True)
motor.enable()

# Test connection
if motor.check_connection():
    print("✓ Motor connected")
else:
    print("✗ Motor not responding")
```

### Position Not Settling (v4.3)

If step commands timeout without settling:

```python
# Increase settling time or tolerance
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    stepTolerance=0.1,        # Relax tolerance
    stepSettlingTime=0.05,    # Reduce required stability time
    stepTimeout=10.0          # Allow more time
)

motor = Motor(config=config)
```

Or disable settling check:
```python
config.stepSettlingTime = 0.0  # No settling verification
```

### Drift with Feedforward Torque

If position drifts when using feedforward:

```python
# Increase settling time to verify stability
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    stepSettlingTime=0.2,  # Longer verification
    stepTolerance=0.03     # Tighter tolerance
)

# Verify feedforward torque calculation
def gravity_torque(angle):
    # Double-check your physics!
    return mass * g * length * np.cos(angle)
```

### Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Setup sudo permissions for CAN commands
sudo visudo
# Add: your_username ALL=(ALL) NOPASSWD: /sbin/ip

# Logout and login for changes to take effect
```

### High Temperature Warning

```python
# Lower temperature threshold
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    maxTemperature=45.0  # Lower warning threshold
)

# Monitor temperature during operation
state = motor.update()
if state['temperature'] > 50:
    print("⚠ High temperature!")
    motor.disable()
```

### CAN Bus Errors

```bash
# Check CAN bus status
ip -details -statistics link show can0

# Look for errors
# RX-ERR and TX-ERR should be 0

# Reset CAN interface if needed
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

# Check for bus-off state
# If bus-off, check wiring and termination resistors
```

### Settling Counter Keeps Resetting

```
Settling: 10% (1/10)
Settling: 20% (2/10)
⚠ Drift detected! Resetting settling counter (3→0)
Settling: 10% (1/10)
...
```

**Possible causes:**
1. **External disturbances**: Vibration, load changes
2. **Incorrect feedforward**: Check torque calculations
3. **Low control gains**: Increase `kp` and `kd`
4. **Too strict tolerance**: Increase `stepTolerance`

**Solutions:**
```python
# Solution 1: Increase control gains
motor.set_position(1.57, kp=20.0, kd=2.0)

# Solution 2: Relax tolerance
config.stepTolerance = 0.1

# Solution 3: Reduce settling requirement
config.stepSettlingTime = 0.05
```

## 🏗️ Architecture

```
User Application
       ↓
   TMotorAPI v4.3 (High-level wrapper)
       ↓ uses
TMotorCANControl (Low-level CAN driver)
       ↓
   SocketCAN
       ↓
   CAN Hardware (MCP2515, etc.)
       ↓
   T-Motor (AK Series)
```

**Design Philosophy:**
- **TMotorCANControl**: Direct MIT CAN protocol implementation (low-level)
- **TMotorAPI**: High-level abstractions with safety features (user-friendly)
- **v4.3**: Enhanced robustness with settling verification and feedforward support

## 🎓 Understanding Settling Time Logic

### Why Settling Time?

**Problem without settling time:**
```python
# Simple tolerance check
while abs(position - target) > tolerance:
    send_command(target)
    
# Returns as soon as within tolerance
# But position might still be moving!
# With feedforward torque, position can drift after "reaching" target
```

**Solution with settling time (v4.3):**
```python
# Must maintain tolerance for N consecutive cycles
settling_counter = 0
while settling_counter < required_cycles:
    if abs(position - target) < tolerance:
        settling_counter += 1
    else:
        settling_counter = 0  # Reset if drift detected
    send_command(target)
    
# Position is truly stable!
```

### Settling Time Calculation

```
settling_time = 0.1s (default)
control_frequency = 100 Hz
required_cycles = settling_time * control_frequency = 10 cycles

Position must stay within tolerance for 10 consecutive cycles.
```

### Tuning Guidelines

| Application | Tolerance | Settling Time | Notes |
|------------|-----------|---------------|-------|
| High-speed | 0.1 rad | 0.05s | Fast, less strict |
| General | 0.05 rad | 0.1s | Balanced (default) |
| Precision | 0.03 rad | 0.2s | Tight, well-verified |
| Ultra-precision | 0.01 rad | 0.5s | Very strict, slow |

## 📊 Performance Characteristics

### Control Loop Timing
- **Frequency**: 100 Hz (10ms period)
- **Step command threshold**: 0.02s
- **Default settling**: 0.1s (10 cycles)

### Typical Step Response Time
```
Without feedforward:   0.15 - 0.30s
With feedforward:      0.20 - 0.40s (includes settling verification)
Trajectory (2s):       2.0 - 2.1s
```

## 📝 License

MIT License - See [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

This library is built on [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) by the Neurobionics Lab.

**Special thanks to:**
- [Neurobionics Lab](https://github.com/neurobionics) for the underlying CAN control library
- MIT for the open CAN protocol specification
- T-Motor for excellent motor hardware

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/KR70004526/TMotorAPI/issues)
- **Base Library**: [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl)
- **Documentation**: This README and code comments

## 🔄 Version History

### v4.3 (Current)
- ✨ Added settling time logic for step commands
- ✨ Added feedforward torque support in `set_position()`
- 🔧 Renamed `track_trajectory()` → `set_position()`
- 🐛 Fixed drift issues with feedforward torque
- 📝 Enhanced logging with settling progress
- 🎯 More robust position control

### v4.2 (Previous)
- Basic trajectory control
- Simple tolerance checking
- Context manager support

---

**Happy Controlling! 🚀**

*Now with robust settling verification!*

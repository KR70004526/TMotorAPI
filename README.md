# TMotorAPI v5.0

A high-level Python library for controlling AK-series T-Motors using the MIT CAN protocol.

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-5.0-green.svg)](https://github.com/KR70004526/TMotorAPI)

## 🆕 What's New in v5.0

### Non-Blocking Control Design 🚀
Complete redesign for **real-time control applications** with non-blocking commands.

**Major Changes from v4.3:**

#### ✅ Removed Duration Parameter
```python
# v4.3 (Blocking)
motor.set_position(1.57, duration=2.0)  # Blocks for 2 seconds
motor.set_torque(5.0, duration=1.0)     # Blocks for 1 second

# v5.0 (Non-blocking) ⭐
motor.set_position(1.57)  # Returns immediately
motor.set_torque(5.0)     # Returns immediately
# User controls timing with update() loop
```

#### ✅ Simplified Control Flow
```python
# v5.0: User-controlled loop
while running:
    motor.set_torque(calculate_torque())  # Set command
    motor.update()                         # Send & receive
    time.sleep(0.01)                       # 100 Hz control
```

#### ✅ Removed Settling Time Logic
- No automatic settling verification
- User implements custom settling if needed
- Removed: `stepTimeout`, `stepTolerance`, `stepSettlingTime`

#### ✅ Fixed zero_position() Bug
```python
# v4.3: Could cause unwanted movement after zeroing
motor.zero_position()  # Bug: Motor might move

# v5.0: Safe zeroing with position command reset
motor.zero_position()  # Safe: Motor stays in place
```

#### ✅ Added Emergency Stop
```python
motor.stop()  # New method: Immediately set torque to 0
```

---

## 🌟 Key Features

- **Non-Blocking Control**: All commands return immediately for real-time applications
- **4 Control Modes**: Position, Velocity, Torque, and Impedance control
- **Context Manager**: Automatic power management with `with` statement
- **Type Hints**: Full type annotations for better IDE support
- **Detailed Logging**: Comprehensive operation logs
- **Auto CAN Setup**: Automatic CAN interface initialization (optional)
- **Emergency Stop**: Safe motor stop with `stop()` method
- **Safe Zeroing**: zero_position() prevents unwanted movement

---

## 📋 Table of Contents

- [What's New in v5.0](#-whats-new-in-v50)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Control Modes](#-control-modes)
- [Non-Blocking Design](#-non-blocking-design)
- [Advanced Features](#-advanced-features)
- [Configuration](#-configuration)
- [API Reference](#-api-reference)
- [Examples](#-examples)
- [Migration from v4.3](#-migration-from-v43)
- [Troubleshooting](#-troubleshooting)

---

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

# Or install using pip (editable mode)
python3 -m pip install -e . --break-system-packages
# Editable mode: Changes to code take effect immediately
```

---

## ⚡ Quick Start

### Basic Non-Blocking Control

```python
from TMotorAPI import Motor
import time
import signal

# Signal handler for clean exit
running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

# Create and use motor with context manager
with Motor('AK80-64', motorId=2, autoInit=True) as motor:
    print("Motor enabled!")
    
    # Non-blocking control loop
    while running:
        # Set control command
        motor.set_position(1.57)  # Target position
        
        # Send command and receive state
        motor.update()
        
        # Print current state
        print(f"Position: {motor.position:.3f} rad, "
              f"Velocity: {motor.velocity:.3f} rad/s, "
              f"Torque: {motor.torque:.3f} Nm")
        
        # Control loop timing (100 Hz)
        time.sleep(0.01)
    
print("Motor disabled!")
```

### Understanding Power Management

**Important**: Motor power operates in 2 stages:

#### 1. Object Creation (Connection, Power OFF)
```python
motor = Motor('AK80-64', motorId=2, autoInit=True)
# ✅ TMotorManager object created
# ✅ CAN connection established
# ⚠️ Motor power is still OFF (motor won't move yet)
```

#### 2. Enable/With Block (Power ON)
```python
with motor:  # __enter__() → enable() → Power ON
    # ✅ Motor is now powered and ready to move
    motor.set_position(1.57)
    motor.update()  # Actually sends command
    # Power remains on throughout the with block
# __exit__() → disable() → Power OFF
```

### Manual Power Control

```python
motor = Motor('AK80-64', motorId=2, autoInit=True)

motor.enable()  # Power ON
print(f"Power: {motor.is_power_on()}")  # True

while running:
    motor.set_position(1.57)
    motor.update()
    time.sleep(0.01)

motor.disable()  # Power OFF
print(f"Power: {motor.is_power_on()}")  # False
```

---

## 🎯 Control Modes

### Overview

| Mode | Function | Use Case |
|------|----------|----------|
| **Position** | `set_position()` | Position tracking with PD control |
| **Velocity** | `set_velocity()` | Speed control |
| **Torque** | `set_torque()` | Force control, gravity compensation |

**All modes are non-blocking in v5.0!**

---

### 1. Position Control

Position tracking with PD gains and optional feedforward torque.

```python
motor.set_position(
    targetPos=1.57,      # Target position (rad)
    kp=10.0,             # Position gain (Nm/rad), optional
    kd=2.0,              # Velocity gain (Nm/(rad/s)), optional
    feedTor=0.0          # Feedforward torque (Nm), optional
)
```

**Parameters:**
- `targetPos`: Desired position in radians
- `kp`: Position gain (uses `defaultKp` if None)
- `kd`: Velocity gain (uses `defaultKd` if None)
- `feedTor`: Feedforward torque for gravity/load compensation

**Usage Pattern:**
```python
while running:
    motor.set_position(target_angle, kp=10, kd=2)
    motor.update()
    time.sleep(0.01)  # 100 Hz control loop
```

**When to use:**
- Position tracking with custom control loop
- Real-time position updates based on sensor data
- Integrating with external trajectory planners

---

### 2. Velocity Control

Direct velocity command with damping gain.

```python
motor.set_velocity(
    targetVel=3.0,      # Target velocity (rad/s)
    kd=5.0              # Velocity gain (Nm/(rad/s)), optional
)
```

**Parameters:**
- `targetVel`: Desired velocity in rad/s
- `kd`: Velocity gain (uses `defaultKd` if None)

**Usage Pattern:**
```python
while running:
    motor.set_velocity(target_speed, kd=5)
    motor.update()
    time.sleep(0.01)
```

**When to use:**
- Continuous rotation applications
- Speed-based control
- Wheel/joint velocity control

---

### 3. Torque Control

Direct torque/force control.

```python
motor.set_torque(
    targetTor=2.5       # Desired torque (Nm)
)
```

**Parameters:**
- `targetTor`: Desired torque in Nm

**Usage Pattern:**
```python
while running:
    torque = calculate_torque()  # Your control algorithm
    motor.set_torque(torque)
    motor.update()
    time.sleep(0.01)
```

**When to use:**
- Force control applications
- Gravity compensation
- Impedance control
- Haptic feedback
- Compliant manipulation

---

## 🔄 Non-Blocking Design

### Core Concept

**v5.0 uses a non-blocking design where the user controls the timing:**

```python
# The Control Loop Pattern
while running:
    # 1. Calculate/update command
    target = calculate_target()
    
    # 2. Set command (non-blocking, returns immediately)
    motor.set_position(target)
    
    # 3. Send command & receive state (CAN communication)
    motor.update()
    
    # 4. Use current state for next iteration
    current_pos = motor.position
    current_vel = motor.velocity
    
    # 5. Control loop timing
    time.sleep(0.01)  # 100 Hz
```

### Key Differences from v4.3

| Aspect | v4.3 (Blocking) | v5.0 (Non-blocking) |
|--------|-----------------|---------------------|
| **Command** | Blocks until complete | Returns immediately |
| **Duration** | `duration=2.0` | No duration parameter |
| **Timing** | Handled by library | User controls timing |
| **Flexibility** | Limited | High flexibility |
| **Real-time** | Difficult | Easy |

### Advantages of Non-Blocking Design

#### 1. Real-Time Control
```python
# React to sensor data immediately
while running:
    sensor_data = read_sensor()
    
    if sensor_data > threshold:
        motor.set_torque(0)  # Immediate response
    else:
        motor.set_torque(5.0)
    
    motor.update()
    time.sleep(0.001)  # 1000 Hz possible
```

#### 2. Multi-Motor Coordination
```python
# Control multiple motors synchronously
with Motor('AK80-64', motorId=1) as motor1, \
     Motor('AK80-64', motorId=2) as motor2:
    
    while running:
        # Set commands for both motors
        motor1.set_position(angle1)
        motor2.set_position(angle2)
        
        # Update both at the same time
        motor1.update()
        motor2.update()
        
        time.sleep(0.01)
```

#### 3. Custom Control Algorithms
```python
# Implement your own settling logic
def wait_until_settled(motor, target, tolerance=0.05, timeout=5.0):
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        motor.set_position(target)
        motor.update()
        
        if abs(motor.position - target) < tolerance:
            return True
        
        time.sleep(0.01)
    
    return False

# Use it
if wait_until_settled(motor, 1.57):
    print("Position reached!")
```

#### 4. Integration with External Systems
```python
# Integrate with ROS, game loops, etc.
def ros_control_loop():
    rate = rospy.Rate(100)  # 100 Hz
    
    while not rospy.is_shutdown():
        # Get command from ROS
        target = get_ros_command()
        
        # Send to motor
        motor.set_position(target)
        motor.update()
        
        # Publish state back to ROS
        publish_motor_state(motor.position, motor.velocity)
        
        rate.sleep()
```

---

## 🔬 Advanced Features

### Emergency Stop

Immediately set torque to zero for safety:

```python
# In emergency situations
motor.stop()  # Sets torque to 0 and updates

# Equivalent to:
motor.set_torque(0.0)
motor.update()
```

**Use cases:**
- Safety stop button
- Collision detection
- Over-temperature protection
- Error conditions

### Zeroing Position

Set current position as zero reference (safe in v5.0):

```python
motor.zero_position()
# ✅ Current position is now 0.0 rad
# ✅ Motor does NOT move
# ✅ Position command is reset to 0
# ✅ Takes ~1 second (EEPROM save)
```

**What happens:**
1. Sends zero position command to controller
2. Waits 1 second for EEPROM save
3. Sets position command to 0 (prevents movement)
4. Updates motor state

**When to use:**
- Initial calibration
- Setting home position
- Resetting encoder after mechanical adjustment

### Gravity Compensation

Compensate for gravity in position control:

```python
import numpy as np

# System parameters
mass = 2.0      # kg
g = 9.81        # m/s²
length = 0.3    # m

def gravity_torque(angle):
    """Calculate gravity compensation torque"""
    return mass * g * length * np.cos(angle)

# Use in control loop
while running:
    target_angle = 1.57  # 90 degrees
    
    motor.set_position(
        targetPos=target_angle,
        kp=10.0,
        kd=2.0,
        feedTor=gravity_torque(target_angle)
    )
    motor.update()
    time.sleep(0.01)
```

### State Monitoring

Access motor state in real-time:

```python
# Update and get all state
state = motor.update()
print(f"Position: {state['position']:.3f} rad")
print(f"Velocity: {state['velocity']:.3f} rad/s")
print(f"Torque: {state['torque']:.3f} Nm")
print(f"Temperature: {state['temperature']:.1f} °C")

# Or access properties (uses last updated values)
pos = motor.position        # rad
vel = motor.velocity        # rad/s
tor = motor.torque          # Nm
temp = motor.temperature    # °C

# Check motor status
is_on = motor.is_power_on()
uptime = motor.get_uptime()  # Seconds since enable()
connected = motor.check_connection()
```

### Custom Control Loops

Implement advanced control algorithms:

```python
# Example: Simple settling logic
def move_with_settling(motor, target, tolerance=0.05, 
                       stable_count=10):
    """
    Move to target and wait until position is stable
    
    Args:
        motor: Motor instance
        target: Target position (rad)
        tolerance: Position tolerance (rad)
        stable_count: Required consecutive stable readings
    """
    count = 0
    
    while count < stable_count:
        motor.set_position(target)
        motor.update()
        
        if abs(motor.position - target) < tolerance:
            count += 1
        else:
            count = 0  # Reset if position drifts
        
        time.sleep(0.01)
    
    print(f"Position stable at {motor.position:.3f} rad")

# Use it
move_with_settling(motor, 1.57, tolerance=0.03, stable_count=20)
```

---

## ⚙️ Configuration

### MotorConfig Class

Complete configuration for motor parameters:

```python
from TMotorAPI import MotorConfig

config = MotorConfig(
    # ==================== Motor Identification ====================
    motorType='AK80-64',        # 'AK80-64', 'AK80-9', 'AK70-10'
    motorId=2,                  # CAN ID (0-127)
    
    # ==================== CAN Setup ====================
    canInterface='can0',        # 'can0', 'can1', etc.
    bitrate=1000000,            # CAN bitrate (default: 1 Mbps)
    autoInit=True,              # Auto setup CAN interface
    
    # ==================== Safety ====================
    maxTemperature=50.0,        # Max MOSFET temperature (°C)
    
    # ==================== Default Control Gains ====================
    defaultKp=10.0,             # Position gain (Nm/rad)
    defaultKd=0.5,              # Velocity gain (Nm/(rad/s))
)

motor = Motor(config=config)
```

### Parameter Details

#### Motor Identification
- **motorType**: Motor model string
  - Must match physical motor
  - Examples: `'AK80-64'`, `'AK80-9'`, `'AK70-10'`
- **motorId**: CAN ID (0-127)
  - Configured on the motor hardware
  - Must be unique on the CAN bus

#### CAN Setup
- **canInterface**: Linux CAN interface name
  - Only used by `CANInterface.setup_interface()`
  - Common: `'can0'`, `'can1'`
- **bitrate**: CAN bus speed
  - Default: 1000000 (1 Mbps) for T-Motors
  - Must match all devices on bus
- **autoInit**: Automatic CAN interface setup
  - `True`: Runs setup automatically
  - `False`: Manual setup required

**Note**: `TMotorManager_mit_can` automatically detects CAN interface after setup.

#### Safety
- **maxTemperature**: Temperature warning threshold (°C)
  - Logs warning when exceeded
  - Does NOT automatically stop motor
  - User should monitor and take action

#### Default Control Gains
- **defaultKp**: Default position gain (Nm/rad)
  - Used when `kp=None` in `set_position()`
  - Higher = stiffer position control
  - Typical: 5.0 - 20.0
- **defaultKd**: Default velocity gain (Nm/(rad/s))
  - Used when `kd=None` in `set_position()` and `set_velocity()`
  - Higher = more damping
  - Typical: 0.5 - 5.0

### Three Ways to Create Motor

```python
# Method 1: Direct parameters (simple)
motor = Motor('AK80-64', motorId=2, autoInit=True)

# Method 2: Config object (recommended for complex setups)
config = MotorConfig(
    motorType='AK80-64',
    motorId=2,
    maxTemperature=60.0,
    defaultKp=15.0,
    defaultKd=2.0
)
motor = Motor(config=config)

# Method 3: Mix both (parameters override config)
config = MotorConfig(motorType='AK80-64', motorId=2)
motor = Motor(config=config, maxTemperature=70.0)  # Override
```

---

## 📚 API Reference

### Motor Class

#### Constructor

```python
Motor(
    motorType: Optional[str] = None,
    motorId: Optional[int] = None,
    canInterface: Optional[str] = None,
    bitrate: Optional[int] = None,
    autoInit: Optional[bool] = None,
    maxTemperature: Optional[float] = None,
    config: Optional[MotorConfig] = None,
    **kwargs
)
```

#### Control Methods

All methods are **non-blocking** and return immediately.

| Method | Parameters | Description |
|--------|-----------|-------------|
| `set_position()` | `targetPos, kp=None, kd=None, feedTor=0.0` | Set position command |
| `set_velocity()` | `targetVel, kd=None` | Set velocity command |
| `set_torque()` | `targetTor` | Set torque command |
| `stop()` | - | Emergency stop (torque = 0) |
| `zero_position()` | - | Set current position as zero |

#### State Methods

```python
# Send command and receive state (CAN communication)
state = motor.update()
# Returns: {'position': float, 'velocity': float, 
#           'torque': float, 'temperature': float}

# Access cached state (no CAN communication)
pos = motor.position        # rad
vel = motor.velocity        # rad/s
tor = motor.torque          # Nm
temp = motor.temperature    # °C

# Status checks
motor.is_power_on()         # True/False
motor.get_uptime()          # Seconds since enable()
motor.check_connection()    # Test CAN communication
```

#### Power Management

```python
motor.enable()   # Power ON (required before commands)
motor.disable()  # Power OFF

# Context manager (automatic power management)
with motor:
    # Motor powered on
    motor.set_position(1.57)
    motor.update()
# Motor powered off
```

---

### CANInterface Class

Manual CAN interface setup (optional, automatic with `autoInit=True`):

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

---

### TrajectoryGenerator Class (Utility)

Low-level trajectory planning utilities (for custom implementations):

```python
from TMotorAPI import TrajectoryGenerator

# Minimum jerk trajectory (5th order)
pos, vel = TrajectoryGenerator.minimum_jerk(
    startPos=0.0,
    endPos=1.57,
    currentTime=0.5,
    totalDuration=2.0
)

# Cubic trajectory (3rd order)
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

**Note**: These are utilities for implementing custom trajectory following in your control loop.

---

## 💡 Examples

### Example 1: Simple Position Control

```python
from TMotorAPI import Motor
import time
import signal

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

with Motor('AK80-64', motorId=1, autoInit=True) as motor:
    print("Moving to 90 degrees...")
    
    target = 1.57  # π/2 rad
    
    while running:
        motor.set_position(target, kp=10, kd=2)
        motor.update()
        
        print(f"Position: {motor.position:.3f} rad, "
              f"Error: {abs(motor.position - target):.4f} rad")
        
        time.sleep(0.01)  # 100 Hz
```

### Example 2: Gravity Compensation

```python
from TMotorAPI import Motor, MotorConfig
import numpy as np
import time
import signal

# System parameters
mass = 2.0      # kg
g = 9.81        # m/s²
length = 0.3    # m (center of mass distance)

def gravity_torque(angle):
    """Calculate gravity compensation torque"""
    return mass * g * length * np.cos(angle)

# Configure motor
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    defaultKp=15.0,
    defaultKd=2.0
)

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

with Motor(config=config) as motor:
    # Zero at horizontal position
    print("Zeroing position...")
    motor.zero_position()
    
    # Move to various angles with gravity compensation
    angles = [0.0, 0.5, 1.0, 1.57, 2.0]  # radians
    
    for target_angle in angles:
        print(f"\nMoving to {np.degrees(target_angle):.1f}°...")
        
        # Move for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0 and running:
            motor.set_position(
                targetPos=target_angle,
                kp=15.0,
                kd=2.0,
                feedTor=gravity_torque(target_angle)
            )
            motor.update()
            
            print(f"  Pos: {motor.position:.3f} rad, "
                  f"Torque: {motor.torque:.2f} Nm, "
                  f"Temp: {motor.temperature:.1f}°C")
            
            time.sleep(0.01)
        
        if not running:
            break
    
    print("\nDone!")
```

### Example 3: Velocity Sweep

```python
from TMotorAPI import Motor
import time
import signal

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

with Motor('AK80-9', motorId=2, autoInit=True) as motor:
    velocities = [1.0, 2.0, 3.0, 2.0, 1.0, 0.0]  # rad/s
    
    for target_vel in velocities:
        print(f"\nSetting velocity: {target_vel} rad/s")
        
        # Hold velocity for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0 and running:
            motor.set_velocity(target_vel, kd=5.0)
            motor.update()
            
            print(f"  Vel: {motor.velocity:.3f} rad/s, "
                  f"Pos: {motor.position:.3f} rad")
            
            time.sleep(0.01)
        
        if not running:
            break
    
    # Stop
    motor.set_velocity(0.0)
    motor.update()
```

### Example 4: Torque Control with Safety Monitoring

```python
from TMotorAPI import Motor, MotorConfig
import time
import signal

# Configure with lower temperature threshold
config = MotorConfig(
    motorType='AK70-10',
    motorId=1,
    maxTemperature=45.0
)

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

with Motor(config=config) as motor:
    target_torque = 2.0  # Nm
    duration = 5.0       # seconds
    
    print(f"Applying {target_torque} Nm for {duration}s...")
    
    start_time = time.time()
    
    while time.time() - start_time < duration and running:
        # Apply torque
        motor.set_torque(target_torque)
        motor.update()
        
        # Monitor state
        print(f"Pos: {motor.position:.3f} rad, "
              f"Vel: {motor.velocity:.3f} rad/s, "
              f"Torque: {motor.torque:.3f} Nm, "
              f"Temp: {motor.temperature:.1f}°C")
        
        # Safety check
        if motor.temperature > config.maxTemperature:
            print("⚠ Temperature too high! Stopping...")
            motor.stop()
            break
        
        time.sleep(0.1)
    
    # Stop torque
    motor.stop()
    print("Torque stopped")
```

### Example 5: Custom Settling Logic

```python
from TMotorAPI import Motor
import time
import signal

def move_and_settle(motor, target, tolerance=0.05, stable_cycles=10):
    """
    Move to target and wait until position is stable
    
    Args:
        motor: Motor instance
        target: Target position (rad)
        tolerance: Position tolerance (rad)
        stable_cycles: Required consecutive stable readings
    
    Returns:
        True if settled, False if interrupted
    """
    count = 0
    running = True
    
    def stop_handler(s, f):
        nonlocal running
        running = False
    
    signal.signal(signal.SIGINT, stop_handler)
    
    print(f"Moving to {target:.3f} rad...")
    
    while count < stable_cycles and running:
        motor.set_position(target, kp=10, kd=2)
        motor.update()
        
        error = abs(motor.position - target)
        
        if error < tolerance:
            count += 1
            print(f"  Settling: {count}/{stable_cycles} "
                  f"(error: {error:.4f} rad)")
        else:
            if count > 0:
                print(f"  Drift detected! Resetting counter "
                      f"({count}→0)")
            count = 0
        
        time.sleep(0.01)
    
    if running:
        print(f"✓ Position stable at {motor.position:.3f} rad")
        return True
    else:
        print("✗ Interrupted")
        return False

# Use it
with Motor('AK80-64', motorId=1, autoInit=True) as motor:
    if move_and_settle(motor, 1.57, tolerance=0.03, stable_cycles=20):
        print("Ready for next operation")
```

### Example 6: Multi-Motor Synchronization

```python
from TMotorAPI import Motor
import time
import signal

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

# Create two motors
with Motor('AK80-64', motorId=1, canInterface='can0') as motor1, \
     Motor('AK80-64', motorId=2, canInterface='can0') as motor2:
    
    print("Synchronizing two motors...")
    
    # Synchronous sine wave motion
    import numpy as np
    
    t = 0.0
    dt = 0.01  # 100 Hz
    
    while running:
        # Calculate synchronized positions
        angle1 = np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz sine
        angle2 = np.cos(2 * np.pi * 0.5 * t)  # 0.5 Hz cosine
        
        # Set commands for both motors
        motor1.set_position(angle1, kp=10, kd=2)
        motor2.set_position(angle2, kp=10, kd=2)
        
        # Update both motors
        motor1.update()
        motor2.update()
        
        print(f"Motor1: {motor1.position:.3f} rad, "
              f"Motor2: {motor2.position:.3f} rad")
        
        time.sleep(dt)
        t += dt
    
    print("Stopped")
```

### Example 7: Implementing Custom Trajectory

```python
from TMotorAPI import Motor, TrajectoryGenerator
import time
import signal

running = True
signal.signal(signal.SIGINT, lambda s,f: globals().update(running=False))

with Motor('AK80-64', motorId=1, autoInit=True) as motor:
    # Trajectory parameters
    start_pos = 0.0
    end_pos = 1.57
    duration = 2.0
    
    print(f"Following trajectory: {start_pos} → {end_pos} rad "
          f"in {duration}s")
    
    # Execute trajectory
    start_time = time.time()
    
    while running:
        t = time.time() - start_time
        
        if t > duration:
            break
        
        # Calculate trajectory point
        target_pos, target_vel = TrajectoryGenerator.minimum_jerk(
            startPos=start_pos,
            endPos=end_pos,
            currentTime=t,
            totalDuration=duration
        )
        
        # Send command
        motor.set_position(target_pos, kp=10, kd=2)
        motor.update()
        
        print(f"t={t:.2f}s: Target={target_pos:.3f}, "
              f"Actual={motor.position:.3f}, "
              f"Error={abs(motor.position - target_pos):.4f}")
        
        time.sleep(0.01)
    
    print(f"✓ Trajectory complete!")
    print(f"Final position: {motor.position:.3f} rad")
```

---

## 🔄 Migration from v4.3

### Key Changes

| Feature | v4.3 | v5.0 |
|---------|------|------|
| **Control** | Blocking | Non-blocking |
| **Duration** | `duration=2.0` | Removed |
| **Settling** | Automatic | User implements |
| **Timing** | Library controlled | User controlled |
| **update()** | Called internally | User calls explicitly |

### Migration Steps

#### 1. Remove Duration Parameters

**Before (v4.3):**
```python
motor.set_position(1.57, duration=2.0)  # Blocks for 2s
motor.set_velocity(3.0, duration=1.0)   # Blocks for 1s
motor.set_torque(5.0, duration=0.5)     # Blocks for 0.5s
```

**After (v5.0):**
```python
# Implement your own control loop
while running:
    motor.set_position(1.57)
    motor.update()
    time.sleep(0.01)  # You control timing
```

#### 2. Add Control Loop

**Before (v4.3):**
```python
with motor:
    motor.set_position(1.57, duration=2.0)
    motor.set_position(0.0, duration=2.0)
    # Motor automatically handles timing
```

**After (v5.0):**
```python
with motor:
    # Move to 1.57
    for _ in range(200):  # 2 seconds at 100 Hz
        motor.set_position(1.57)
        motor.update()
        time.sleep(0.01)
    
    # Move to 0.0
    for _ in range(200):
        motor.set_position(0.0)
        motor.update()
        time.sleep(0.01)
```

#### 3. Implement Custom Settling (if needed)

**Before (v4.3):**
```python
# Automatic settling
motor.set_position(1.57, duration=0.0)  # Waits until settled
```

**After (v5.0):**
```python
# Implement your own settling
def wait_settled(motor, target, tolerance=0.05, cycles=10):
    count = 0
    while count < cycles:
        motor.set_position(target)
        motor.update()
        
        if abs(motor.position - target) < tolerance:
            count += 1
        else:
            count = 0
        
        time.sleep(0.01)

wait_settled(motor, 1.57)
```

#### 4. Remove Settling Config

**Before (v4.3):**
```python
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    stepTimeout=5.0,        # Removed in v5.0
    stepTolerance=0.05,     # Removed in v5.0
    stepSettlingTime=0.1    # Removed in v5.0
)
```

**After (v5.0):**
```python
config = MotorConfig(
    motorType='AK80-64',
    motorId=1,
    # Only these parameters remain
    maxTemperature=50.0,
    defaultKp=10.0,
    defaultKd=0.5
)
```

### Why the Change?

**v4.3 Problems:**
- ❌ Blocking calls limit flexibility
- ❌ Hard to implement custom control
- ❌ Difficult multi-motor coordination
- ❌ Can't react to sensors in real-time

**v5.0 Advantages:**
- ✅ Full control over timing
- ✅ Easy custom algorithms
- ✅ Simple multi-motor sync
- ✅ Real-time sensor integration
- ✅ Cleaner code structure

---

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

**Checklist:**
1. ✅ Power supply (24-48V depending on model)
2. ✅ CAN bus termination (120Ω at each end)
3. ✅ Correct motor CAN ID
4. ✅ Motor enabled (`enable()` or `with` statement)
5. ✅ Calling `update()` in loop

**Debug:**
```python
import logging
logging.basicConfig(level=logging.DEBUG)

motor = Motor('AK80-64', motorId=1, autoInit=True)
motor.enable()

# Test connection
if motor.check_connection():
    print("✓ Motor connected")
else:
    print("✗ Motor not responding")
    print("Check power, CAN bus, and motor ID")
```

### update() Not Called

**Problem:**
```python
# Wrong: Command set but never sent
motor.set_position(1.57)
# Motor won't move!
```

**Solution:**
```python
# Correct: Always call update() after command
motor.set_position(1.57)
motor.update()  # This actually sends the command
```

### Position Not Reaching Target

In v5.0, there's no automatic settling. Implement your own:

```python
def wait_for_position(motor, target, tolerance=0.05, timeout=5.0):
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        motor.set_position(target)
        motor.update()
        
        if abs(motor.position - target) < tolerance:
            return True
        
        time.sleep(0.01)
    
    return False

# Use it
if wait_for_position(motor, 1.57):
    print("Position reached!")
else:
    print("Timeout!")
```

### Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Setup sudo permissions for CAN
sudo visudo
# Add: your_username ALL=(ALL) NOPASSWD: /sbin/ip

# Logout and login
```

### High Temperature Warning

```python
# Monitor temperature in control loop
while running:
    motor.set_torque(5.0)
    motor.update()
    
    if motor.temperature > 55.0:
        print("⚠ High temperature! Stopping...")
        motor.stop()
        break
    
    time.sleep(0.01)
```

### CAN Bus Errors

```bash
# Check CAN status
ip -details -statistics link show can0

# Look for errors (RX-ERR and TX-ERR should be 0)

# Reset CAN if needed
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

# Check wiring and termination if errors persist
```

### zero_position() Causes Movement

This was a bug in v4.3, **fixed in v5.0**!

```python
# v5.0: Safe zeroing
motor.zero_position()
# ✓ Position command is reset to 0
# ✓ Motor stays in place
```

---

## 🏗️ Architecture

```
User Application
       ↓
   TMotorAPI v5.0 (Non-blocking wrapper)
       ↓ uses
TMotorCANControl (Low-level CAN driver)
       ↓
   SocketCAN (Linux kernel)
       ↓
   CAN Hardware (MCP2515, etc.)
       ↓
   T-Motor (AK Series)
```

**Design Philosophy:**
- **TMotorCANControl**: Direct MIT CAN protocol (low-level)
- **TMotorAPI v5.0**: Non-blocking, real-time control (high-level)
- **User Application**: Full control over timing and logic

---

## 📊 Performance Characteristics

### Control Loop Timing

**Recommended frequencies:**
- **100 Hz (10ms)**: General purpose, good balance
- **200 Hz (5ms)**: High performance applications
- **500 Hz (2ms)**: Research, high-speed control
- **1000 Hz (1ms)**: Maximum performance (requires fast CPU)

**Example:**
```python
# 100 Hz control
while running:
    motor.set_position(target)
    motor.update()
    time.sleep(0.01)  # 10ms

# 500 Hz control
while running:
    motor.set_torque(torque)
    motor.update()
    time.sleep(0.002)  # 2ms
```

### Typical Response Time

| Control Mode | Response Time |
|--------------|---------------|
| Torque | 10-20 ms |
| Velocity | 50-100 ms |
| Position (PD) | 100-300 ms |

*Response times depend on control gains and system dynamics*

---

## 📝 License

MIT License - See [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

This library is built on [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) by the Neurobionics Lab.

**Special thanks to:**
- [Neurobionics Lab](https://github.com/neurobionics) for TMotorCANControl
- MIT for the open CAN protocol specification
- T-Motor for excellent motor hardware

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/KR70004526/TMotorAPI/issues)
- **Base Library**: [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl)
- **Documentation**: This README

---

## 🔄 Version History

### v5.0 (Current - Major Release)
- 🚀 **Non-blocking control design**
- ✂️ Removed `duration` parameter from all control methods
- ✂️ Removed automatic settling time logic
- ✂️ Removed `stepTimeout`, `stepTolerance`, `stepSettlingTime` parameters
- 🐛 Fixed `zero_position()` causing unwanted movement
- ✨ Added `stop()` method for emergency stop
- 🎯 Simplified API for real-time applications
- 📝 User now controls timing with `update()` loop

### v4.3 (Previous)
- Settling time logic for step commands
- Feedforward torque support
- Blocking control with duration parameter
- Automatic position settling verification

### v4.2
- Basic trajectory control
- Context manager support
- Simple tolerance checking

---

**Happy Controlling! 🚀**

*Now with non-blocking design for real-time control!*

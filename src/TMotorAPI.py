# NOTE: Some non-ASCII text may remain; manual pass recommended.
"""
TMotor Control API v2.1 - Professional Motor Control Library
Based on TMotorCANControl by Neurobionics Lab

Improvements (v2.1):
- Removed unused imports (contextmanager, threading)
- Security hardening (prevent command injection)
- MotorGroup → MotorGroupEnhanced (clear individual/group control separation)
- Added per-motor control API
- Added motor naming
- Added partial group control

Control Modes:
1. Trajectory Control (Position) - Position/trajectory following
2. Velocity Control - Constant velocity rotation
3. Torque Control - Direct torque command
4. Impedance Control - Low-level full control

Author: Custom TMotor Team
License: MIT
"""

import os
import sys
import time
import subprocess
import logging
import re
from typing import Optional, Dict, List, Tuple, Callable, Union
from dataclasses import dataclass

try:
    from TMotorCANControl.TMotorManager_mit_can import TMotorManager_mit_can
    from TMotorCANControl import motor_constants as mc
except ImportError:
    print("Error: TMotorCANControl not installed. Run: pip install TMotorCANControl")
    sys.exit(1)


# ==================== Constants ====================
CONTROL_LOOP_FREQUENCY = 100  # Hz
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_FREQUENCY  # 0.01 seconds
STEP_COMMAND_THRESHOLD = 0.02  # 20ms
ZERO_POSITION_SETTLE_TIME = 0.5  # seconds (reduced from 1.5)
CAN_INTERFACE_PATTERN = re.compile(r'^can\d+$')  # Security: validate interface names


# ==================== Configuration Class ====================
@dataclass
class MotorConfig:
    """
    Motor configuration class

    Stores motor parameters and control gains

    Attributes:
        motor_type: Motor model (e.g., 'AK80-64', 'AK80-9')
        motor_id: CAN ID (1-32)
        can_interface: CAN interface name (e.g., 'can0')
        bitrate: CAN bitrate (default: 1000000)
        auto_enable: Auto enable on initialization
        max_temperature: Maximum safe temperature (°C)
        timeout: Communication timeout (seconds)
        default_kp: Default position gain (Nm/rad)
        default_kd: Default velocity gain (Nm/(rad/s))
        default_torque_limit: Default torque limit (Nm)
    """
    motor_type: str = 'AK80-64'
    motor_id: int = 1
    can_interface: str = 'can0'
    bitrate: int = 1000000
    auto_enable: bool = True
    max_temperature: float = 50.0
    timeout: float = 0.5

    # Default control gains
    default_kp: float = 10.0
    default_kd: float = 0.5
    default_torque_limit: float = 12.0

    def __post_init__(self):
        """Validate configuration after initialization"""
        if not 1 <= self.motor_id <= 32:
            raise ValueError(f"motor_id must be between 1 and 32, got {self.motor_id}")
        if not CAN_INTERFACE_PATTERN.match(self.can_interface):
            raise ValueError(f"Invalid CAN interface name: {self.can_interface}")
        if self.bitrate <= 0:
            raise ValueError(f"bitrate must be positive, got {self.bitrate}")


# ==================== CAN Interface Management ====================
class CANInterface:
    """
    CAN interface automatic setup and management

    Provides methods to configure and manage CAN bus interface
    """

    @staticmethod
    def check_sudo_permission() -> bool:
        """
        Check if sudo permission is available

        Returns:
            bool: True if sudo permission is available
        """
        try:
            result = subprocess.run(
                ['sudo', '-n', 'true'],
                capture_output=True,
                timeout=1
            )
            return result.returncode == 0
        except (subprocess.SubprocessError, OSError, TimeoutError) as e:
            logging.debug(f"Sudo permission check failed: {e}")
            return False

    @staticmethod
    def is_interface_up(interface: str) -> bool:
        """
        Check if CAN interface is up

        Args:
            interface: CAN interface name (e.g., 'can0')

        Returns:
            bool: True if interface is up and running
        """
        # Security: Validate interface name
        if not CAN_INTERFACE_PATTERN.match(interface):
            raise ValueError(f"Invalid CAN interface name: {interface}")

        try:
            result = subprocess.run(
                ['ip', 'link', 'show', interface],
                capture_output=True,
                text=True,
                timeout=2
            )
            return 'state UP' in result.stdout
        except (subprocess.SubprocessError, OSError, TimeoutError) as e:
            logging.debug(f"Failed to check interface status: {e}")
            return False

    @staticmethod
    def setup_can_interface(
        interface: Optional[str] = None,
        bitrate: Optional[int] = None,
        config: Optional[MotorConfig] = None
    ) -> bool:
        """
        Automatically setup CAN interface

        Args:
            interface: CAN interface name (overrides config)
            bitrate: CAN bitrate (overrides config)
            config: MotorConfig object

        Returns:
            bool: True if setup successful

        Examples:
            # Using config object
            config = MotorConfig(can_interface='can0', bitrate=1000000)
            CANInterface.setup_can_interface(config=config)

            # Using individual parameters
            CANInterface.setup_can_interface('can0', 1000000)
        """
        # Priority: individual parameters > config > defaults
        if config:
            _interface = interface if interface is not None else config.can_interface
            _bitrate = bitrate if bitrate is not None else config.bitrate
        else:
            _interface = interface or 'can0'
            _bitrate = bitrate or 1000000

        # Security: Validate interface name
        if not CAN_INTERFACE_PATTERN.match(_interface):
            raise ValueError(f"Invalid CAN interface name: {_interface}")

        try:
            # Check if already up
            if CANInterface.is_interface_up(_interface):
                logging.info(f"{_interface} is already up")
                return True

            # Check sudo permission
            if not CANInterface.check_sudo_permission():
                logging.warning("No sudo permission. Trying without sudo...")
                logging.warning("If this fails, run: sudo visudo")
                logging.warning("Add line: your_username ALL=(ALL) NOPASSWD: /sbin/ip")

            # Bring interface down
            subprocess.run(
                ['sudo', 'ip', 'link', 'set', _interface, 'down'],
                capture_output=True,
                timeout=2,
                check=False  # Don't fail if already down
            )

            # Configure CAN interface
            subprocess.run(
                ['sudo', 'ip', 'link', 'set', _interface, 'type', 'can',
                 'bitrate', str(_bitrate)],
                capture_output=True,
                timeout=2,
                check=True
            )

            # Bring interface up
            subprocess.run(
                ['sudo', 'ip', 'link', 'set', _interface, 'up'],
                capture_output=True,
                timeout=2,
                check=True
            )

            logging.info(f"Successfully set up {_interface} at {_bitrate} bps")
            return True

        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to setup CAN interface: {e}")
            if e.stderr:
                logging.error(f"Error output: {e.stderr.decode()}")
            return False
        except (subprocess.SubprocessError, OSError, ValueError) as e:
            logging.error(f"Error setting up CAN interface: {e}")
            return False

    @staticmethod
    def teardown_can_interface(interface: str = 'can0'):
        """
        Bring down CAN interface

        Args:
            interface: CAN interface name
        """
        # Security: Validate interface name
        if not CAN_INTERFACE_PATTERN.match(interface):
            raise ValueError(f"Invalid CAN interface name: {interface}")

        try:
            subprocess.run(
                ['sudo', 'ip', 'link', 'set', interface, 'down'],
                capture_output=True,
                timeout=2,
                check=True
            )
            logging.info(f"Disabled {interface}")
        except subprocess.CalledProcessError as e:
            logging.warning(f"Could not disable {interface}: {e}")
        except (subprocess.SubprocessError, OSError, ValueError) as e:
            logging.warning(f"Error disabling {interface}: {e}")


# ==================== Trajectory Generator ====================
class TrajectoryGenerator:
    """
    Trajectory generation algorithms

    All methods return (position, velocity) tuple
    """

    @staticmethod
    def minimum_jerk(start: float, end: float, t: float, duration: float) -> tuple:
        """
        Minimum jerk trajectory (5th order polynomial)

        Generates smooth trajectory with continuous acceleration.
        Most commonly used in robot control.

        Args:
            start: Start position (rad)
            end: End position (rad)
            t: Current time (s)
            duration: Total duration (s)

        Returns:
            tuple: (position, velocity) in (rad, rad/s)
        """
        if t <= 0:
            return start, 0.0
        if t >= duration:
            return end, 0.0

        s = t / duration  # normalized time [0, 1]

        # 5th order polynomial: 10s³ - 15s⁴ + 6s⁵
        position = start + (end - start) * (10*s**3 - 15*s**4 + 6*s**5)

        # Velocity (derivative of position)
        velocity = (end - start) / duration * (30*s**2 - 60*s**3 + 30*s**4)

        return position, velocity

    @staticmethod
    def linear(start: float, end: float, t: float, duration: float) -> tuple:
        """
        Linear interpolation trajectory (1st order)

        Simplest but has discontinuous acceleration.

        Args:
            start: Start position (rad)
            end: End position (rad)
            t: Current time (s)
            duration: Total duration (s)

        Returns:
            tuple: (position, velocity) in (rad, rad/s)
        """
        if t <= 0:
            return start, 0.0
        if t >= duration:
            return end, 0.0

        s = t / duration
        position = start + (end - start) * s
        velocity = (end - start) / duration

        return position, velocity

    @staticmethod
    def cubic(start: float, end: float, t: float, duration: float) -> tuple:
        """
        Cubic polynomial trajectory (3rd order)

        Velocity is continuous but acceleration is not.

        Args:
            start: Start position (rad)
            end: End position (rad)
            t: Current time (s)
            duration: Total duration (s)

        Returns:
            tuple: (position, velocity) in (rad, rad/s)
        """
        if t <= 0:
            return start, 0.0
        if t >= duration:
            return end, 0.0

        s = t / duration

        # Cubic polynomial: 3s² - 2s³
        position = start + (end - start) * (3*s**2 - 2*s**3)
        velocity = (end - start) / duration * (6*s - 6*s**2)

        return position, velocity

    @staticmethod
    def trapezoidal(start: float, end: float, t: float, duration: float,
                   v_max: Optional[float] = None) -> tuple:
        """
        Trapezoidal velocity profile

        Constant acceleration → constant velocity → constant deceleration
        Useful when velocity limit exists.

        Args:
            start: Start position (rad)
            end: End position (rad)
            t: Current time (s)
            duration: Total duration (s)
            v_max: Maximum velocity (rad/s), auto-calculated if None

        Returns:
            tuple: (position, velocity) in (rad, rad/s)
        """
        if t <= 0:
            return start, 0.0
        if t >= duration:
            return end, 0.0

        distance = abs(end - start)
        direction = 1.0 if end > start else -1.0

        # Calculate max velocity (acceleration/deceleration time = duration/4)
        if v_max is None:
            v_max = 2.0 * distance / duration

        t_accel = duration * 0.25  # Acceleration time (25%)
        t_const = duration * 0.50  # Constant velocity time (50%)
        t_decel = duration * 0.75  # Deceleration start time (75%)

        if t < t_accel:
            # Acceleration phase
            s = t / t_accel
            position = start + direction * 0.5 * v_max * t * s
            velocity = direction * v_max * s
        elif t < t_decel:
            # Constant velocity phase
            position = start + direction * (0.125 * v_max * duration + v_max * (t - t_accel))
            velocity = direction * v_max
        else:
            # Deceleration phase
            t_remain = duration - t
            s = t_remain / (duration - t_decel)
            position = end - direction * 0.5 * v_max * t_remain * s
            velocity = direction * v_max * s

        return position, velocity


# ==================== Single Motor Control Class ====================
class Motor:
    """
    High-level API for single motor control

    Provides 4 control modes:
    1. Trajectory Control (Position) - track_trajectory()
    2. Velocity Control - set_velocity()
    3. Torque Control - set_torque()
    4. Impedance Control (Low-level) - send_command()

    The motor power is managed through enable()/disable() or context manager.

    Examples:
        # Basic usage
        motor = Motor('AK80-64', motor_id=2, auto_init=True)
        motor.enable()  # Power ON
        motor.track_trajectory(position=1.57)
        motor.disable()  # Power OFF

        # Using context manager (recommended)
        with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
            motor.track_trajectory(1.57)
    """

    def __init__(self, motor_type: str = 'AK80-64', motor_id: int = 1,
                 can_interface: str = 'can0', auto_init: bool = False,
                 config: Optional[MotorConfig] = None):
        """
        Initialize motor object

        This creates the motor object but does NOT power it on.
        Call enable() or use context manager to power on.

        Args:
            motor_type: Motor model (e.g., 'AK80-64', 'AK80-9')
            motor_id: CAN ID (1-32)
            can_interface: CAN interface name
            auto_init: If True, calls initialize() automatically
            config: Additional configuration (optional)

        Note:
            auto_init=True: Creates TMotorManager object (connects, power OFF)
            auto_init=False: TMotorManager created on first enable()
        """
        # Initialize configuration
        if config is None:
            self.config = MotorConfig(
                motor_type=motor_type,
                motor_id=motor_id,
                can_interface=can_interface
            )
        else:
            self.config = config

        # Setup logging
        self._setup_logging()

        # Internal state variables
        self._manager: Optional[TMotorManager_mit_can] = None  # TMotorManager object
        self._is_enabled = False  # Power status flag
        self._last_position = 0.0
        self._last_velocity = 0.0
        self._last_torque = 0.0
        self._last_temperature = 0.0

        # Power monitoring
        self._power_on_time = 0.0  # Timestamp when power turned ON
        self._last_update_time = 0.0  # Last communication time

        # Auto initialization (creates TMotorManager, power still OFF)
        if auto_init:
            self.initialize()

        logging.info(f"Motor object created: {motor_type} (ID: {motor_id})")

    def _setup_logging(self):
        """Setup logging configuration"""
        logging.basicConfig(
            level=logging.INFO,
            format='[%(levelname)s] %(message)s'
        )

    def initialize(self) -> bool:
        """
        Setup CAN interface and connect to motor

        This method:
        1. Sets up CAN interface (bitrate, etc.)
        2. Creates TMotorManager object
        3. Establishes CAN connection

        The motor power is still OFF after this!
        Call enable() to power on the motor.

        Returns:
            bool: True if successful

        Note:
            Called automatically if auto_init=True in __init__
            Called automatically on first enable() if not initialized
        """
        try:
            # Setup CAN interface
            if not CANInterface.setup_can_interface(config=self.config):
                logging.error("Failed to setup CAN interface")
                return False

            # Create TMotorManager object (connects to motor, power OFF)
            self._manager = TMotorManager_mit_can(
                motor_type=self.config.motor_type,
                motor_ID=self.config.motor_id,
                socket_name=self.config.can_interface,
                max_temp=self.config.max_temperature
            )

            logging.info(f"Motor {self.config.motor_type} (ID: {self.config.motor_id}) "
                        f"connected on {self.config.can_interface} @ {self.config.bitrate} bps")
            logging.info("Motor is connected but power is OFF. Call enable() to power on.")
            return True

        except Exception as e:
            logging.error(f"Failed to initialize motor: {e}")
            import traceback
            traceback.print_exc()
            # Cleanup on failure
            self._manager = None
            return False

    def enable(self) -> bool:
        """
        Enable motor (Power ON)

        This method powers on the motor by calling TMotorManager.__enter__()
        After this call, the motor is ready for control.

        Power remains ON until disable() is called or context manager exits.

        Returns:
            bool: True if successful

        Examples:
            motor = Motor('AK80-64', motor_id=2, auto_init=True)
            motor.enable()  # Power ON
            motor.track_trajectory(1.57)
            motor.disable()  # Power OFF

        Note:
            This is automatically called when using context manager:
            with Motor(...) as motor:  # enable() called here
                pass
        """
        # Initialize if not already done
        if self._manager is None:
            logging.info("TMotorManager not initialized. Calling initialize()...")
            if not self.initialize():
                return False

        try:
            # Power ON: Call TMotorManager.__enter__()
            # This sends CAN message to power on the motor
            self._manager.__enter__()

            # Update state
            self._is_enabled = True
            self._power_on_time = time.time()

            logging.info("=" * 60)
            logging.info("Motor ENABLED (Powered ON)")
            logging.info("=" * 60)
            logging.info("The motor is now powered on and will remain on until:")
            logging.info("  1. disable() is called, or")
            logging.info("  2. Context manager exits")
            logging.info("=" * 60)

            # Initial state update
            self.update()

            return True
        except Exception as e:
            logging.error(f"Failed to enable motor: {e}")
            import traceback
            traceback.print_exc()
            self._is_enabled = False
            return False

    def disable(self):
        """
        Disable motor (Power OFF)

        This method powers off the motor by calling TMotorManager.__exit__()
        The motor will stop holding position and can move freely.

        Note:
            This is automatically called when using context manager:
            with Motor(...) as motor:
                pass  # disable() called here when exiting
        """
        if self._manager and self._is_enabled:
            try:
                # Calculate uptime
                uptime = time.time() - self._power_on_time

                # Power OFF: Call TMotorManager.__exit__()
                # This sends CAN message to power off the motor
                self._manager.__exit__(None, None, None)

                # Update state
                self._is_enabled = False

                logging.info("=" * 60)
                logging.info("Motor DISABLED (Powered OFF)")
                logging.info(f"Total powered-on time: {uptime:.2f} seconds")
                logging.info("=" * 60)
            except Exception as e:
                logging.error(f"Error disabling motor: {e}")
                import traceback
                traceback.print_exc()

    def update(self) -> Dict[str, float]:
        """
        Update and read motor state

        Reads current position, velocity, torque, and temperature from motor.

        Returns:
            dict: Current state {'position', 'velocity', 'torque', 'temperature'}

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        try:
            # Update motor state via CAN communication
            self._manager.update()

            # Save state
            self._last_position = self._manager.position
            self._last_velocity = self._manager.velocity
            self._last_torque = self._manager.torque
            self._last_temperature = self._manager.get_temperature_celsius()
            self._last_update_time = time.time()

            return {
                'position': self._last_position,
                'velocity': self._last_velocity,
                'torque': self._last_torque,
                'temperature': self._last_temperature
            }
        except Exception as e:
            logging.error(f"Failed to update motor state: {e}")
            raise  # Re-raise to make failure explicit

    def is_power_on(self) -> bool:
        """
        Check if motor power is on

        Returns:
            bool: True if motor is powered on
        """
        return self._is_enabled

    def get_uptime(self) -> float:
        """
        Get motor uptime (time since power on)

        Returns:
            float: Uptime in seconds, 0 if not powered on
        """
        if self._is_enabled:
            return time.time() - self._power_on_time
        return 0.0

    def check_connection(self) -> bool:
        """
        Check if motor is still connected and responding

        Returns:
            bool: True if motor is responding
        """
        if not self._is_enabled:
            return False

        try:
            self.update()
            return True
        except Exception as e:
            logging.error(f"Connection check failed: {e}")
            return False

    # ==================== Control Mode 1: Trajectory Control ====================

    def track_trajectory(self,
                        position: float,
                        kp: Optional[float] = None,
                        kd: Optional[float] = None,
                        duration: float = 0.0,
                        trajectory_type: str = 'minimum_jerk') -> None:
        """
        Trajectory control (Position control)

        Unified position/trajectory control interface.
        Behavior changes based on duration parameter:
        - duration = 0: Step position (immediate move, velocity=0)
        - duration > 0: Trajectory following (smooth motion)

        Args:
            position: Target position (rad)
            kp: Position gain (Nm/rad), None for default
            kd: Velocity gain (Nm/(rad/s)), None for default
            duration: Motion duration (seconds)
                - 0: Immediate move (step position)
                - >0: Smooth trajectory generation
            trajectory_type: Trajectory type (only for duration>0)
                - 'minimum_jerk': 5th order polynomial (smoothest) [default]
                - 'cubic': 3rd order polynomial
                - 'linear': Linear interpolation
                - 'trapezoidal': Trapezoidal velocity profile

        Examples:
            # Immediate move (step position)
            motor.track_trajectory(1.57)
            motor.track_trajectory(1.57, kp=10, kd=0.5)

            # Smooth trajectory (2 seconds)
            motor.track_trajectory(1.57, duration=2.0)
            motor.track_trajectory(1.57, kp=10, kd=0.5, duration=2.0)

            # Fast linear motion
            motor.track_trajectory(3.14, duration=1.0, trajectory_type='linear')

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        # Use default gains if not specified
        if kp is None:
            kp = self.config.default_kp
        if kd is None:
            kd = self.config.default_kd

        try:
            # Set impedance gains
            self._manager.set_impedance_gains_real_unit(K=kp, B=kd)

            if duration <= STEP_COMMAND_THRESHOLD:
                # ============ Step Position Mode ============
                # Immediate move (velocity = 0)
                # Note: Direct assignment to TMotorManager properties
                self._manager.position = position
                self._manager.velocity = 0.0
                self._manager.torque = 0.0
                self.update()
                logging.info(f"Step position: {self._last_position:.3f} → {position:.3f} rad")
                return

            # ============ Trajectory Following Mode ============
            # Select trajectory generation function
            if trajectory_type == 'minimum_jerk':
                traj_func = TrajectoryGenerator.minimum_jerk
            elif trajectory_type == 'cubic':
                traj_func = TrajectoryGenerator.cubic
            elif trajectory_type == 'linear':
                traj_func = TrajectoryGenerator.linear
            elif trajectory_type == 'trapezoidal':
                traj_func = TrajectoryGenerator.trapezoidal
            else:
                raise ValueError(f"Unknown trajectory type: {trajectory_type}")

            # Track trajectory
            start_pos = self._last_position
            t0 = time.time()

            logging.info(f"Trajectory start: {start_pos:.3f} → {position:.3f} rad "
                        f"({duration:.2f}s, {trajectory_type})")

            while True:
                t = time.time() - t0
                if t >= duration:
                    break

                # Calculate trajectory point
                p, v = traj_func(start_pos, position, t, duration)

                # Send command (position + velocity)
                self._manager.position = p
                self._manager.velocity = v
                self._manager.torque = 0.0
                self.update()

                time.sleep(CONTROL_LOOP_PERIOD)

            # Final stabilization
            self._manager.position = position
            self._manager.velocity = 0.0
            self._manager.torque = 0.0
            self.update()

            logging.info(f"Trajectory complete: {start_pos:.3f} → {position:.3f} rad "
                        f"({duration:.2f}s)")

        except Exception as e:
            logging.error(f"Failed to execute trajectory: {e}")
            import traceback
            traceback.print_exc()
            raise  # Re-raise for explicit error handling

    # ==================== Control Mode 2: Velocity Control ====================

    def set_velocity(self, velocity: float, kd: Optional[float] = None) -> None:
        """
        Velocity control

        Controls motor velocity without feedforward torque.
        For velocity control with FF torque, use send_command().

        Internal operation:
        - position = current position (fixed)
        - velocity = target velocity
        - Kp = 0 (position control OFF)
        - Kd = kd (velocity tracking gain)
        - FF torque = 0

        Args:
            velocity: Target velocity (rad/s)
            kd: Velocity gain (Nm/(rad/s)), None for default

        Examples:
            # Rotate at 2 rad/s
            motor.set_velocity(2.0)

            # Strong velocity tracking
            motor.set_velocity(2.0, kd=10.0)

            # Stop
            motor.set_velocity(0.0)

        Note:
            For velocity control with FF torque (e.g., gravity compensation):
            motor.send_command(
                position=motor.position,
                velocity=2.0,
                kp=0, kd=5.0,
                torque=gravity_compensation
            )

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        # Use default gain if not specified
        if kd is None:
            kd = self.config.default_kd

        try:
            # Set impedance gains (Kp=0, Kd=kd)
            self._manager.set_impedance_gains_real_unit(K=0.0, B=kd)

            # Velocity control
            self._manager.position = self._last_position  # Fix at current position
            self._manager.velocity = velocity
            self._manager.torque = 0.0  # No FF torque
            self.update()

            logging.debug(f"Velocity set: {velocity:.3f} rad/s")

        except Exception as e:
            logging.error(f"Failed to set velocity: {e}")
            import traceback
            traceback.print_exc()
            raise

    # ==================== Control Mode 3: Torque Control ====================

    def set_torque(self, torque: float) -> None:
        """
        Torque control

        Pure feedforward torque control.
        Position/velocity control is OFF (Kp=0, Kd=0).

        Args:
            torque: Target torque (Nm)

        Examples:
            # Gravity compensation
            motor.set_torque(3.5)

            # Apply constant torque
            motor.set_torque(5.0)

            # Remove torque (free motion)
            motor.set_torque(0.0)

        Note:
            - Motor can move freely with external forces
            - No position/velocity feedback
            - Useful for force control or gravity compensation

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        try:
            # Turn off impedance gains (Kp=0, Kd=0)
            self._manager.set_impedance_gains_real_unit(K=0.0, B=0.0)

            # Torque control
            self._manager.position = 0.0
            self._manager.velocity = 0.0
            self._manager.torque = torque
            self.update()

            logging.debug(f"Torque set: {torque:.3f} Nm")

        except Exception as e:
            logging.error(f"Failed to set torque: {e}")
            import traceback
            traceback.print_exc()
            raise

    # ==================== Control Mode 4: Impedance Control (Low-level) ====================

    def send_command(self,
                    position: float,
                    velocity: float,
                    kp: float,
                    kd: float,
                    torque: float = 0.0) -> None:
        """
        Impedance control (Low-level full control)

        Direct control of all parameters.
        This is the low-level interface for experts.
        Use this for integration with external controllers (MPC, LQR, etc.).

        Control equation:
        τ = Kp × (pos_target - pos_actual) + Kd × (vel_target - vel_actual) + τ_ff

        Args:
            position: Target position (rad)
            velocity: Target velocity (rad/s)
            kp: Position gain (Nm/rad)
            kd: Velocity gain (Nm/(rad/s))
            torque: Feedforward torque (Nm)

        Examples:
            # Velocity + FF torque (gravity compensation)
            motor.send_command(
                position=motor.position,
                velocity=2.0,
                kp=0.0, kd=5.0,
                torque=gravity_compensation
            )

            # MPC controller integration
            for _ in range(1000):
                p, v, kp, kd, tau = mpc.compute()
                motor.send_command(p, v, kp, kd, tau)
                time.sleep(0.01)

            # Soft force control
            motor.send_command(
                position=target_pos,
                velocity=0.0,
                kp=5.0,   # Low stiffness
                kd=0.3,   # Low damping
                torque=desired_force
            )

        Warning:
            This is a low-level interface.
            For typical use cases, use track_trajectory(), set_velocity(), or set_torque().

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        try:
            # Set impedance gains
            self._manager.set_impedance_gains_real_unit(K=kp, B=kd)

            # Send command
            self._manager.position = position
            self._manager.velocity = velocity
            self._manager.torque = torque
            self.update()

            logging.debug(f"Command sent: pos={position:.3f}, vel={velocity:.3f}, "
                         f"kp={kp:.1f}, kd={kd:.2f}, tau={torque:.2f}")

        except Exception as e:
            logging.error(f"Failed to send command: {e}")
            import traceback
            traceback.print_exc()
            raise

    # ==================== Utility Functions ====================

    def zero_position(self) -> None:
        """
        Set current position as zero

        This changes the encoder zero reference point.
        The motor's physical position doesn't change.

        Raises:
            RuntimeError: If motor is not enabled
        """
        if not self._is_enabled or self._manager is None:
            raise RuntimeError("Motor not enabled. Call enable() first.")

        try:
            self._manager.set_zero_position()
            logging.info("Zero position set")
            time.sleep(ZERO_POSITION_SETTLE_TIME)  # Wait for setting to take effect
            self.update()
        except Exception as e:
            logging.error(f"Failed to set zero position: {e}")
            import traceback
            traceback.print_exc()
            raise

    # ==================== Context Manager ====================

    def __enter__(self):
        """
        Context manager entry

        Automatically calls enable() to power on the motor.

        Returns:
            Motor: self

        Note:
            This is called when entering 'with' block:
            with Motor(...) as motor:  # __enter__() called here
                pass
        """
        if not self.enable():
            raise RuntimeError("Failed to enable motor in context manager")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit

        Automatically calls disable() to power off the motor.

        Note:
            This is called when exiting 'with' block:
            with Motor(...) as motor:
                pass  # __exit__() called here
        """
        self.disable()  # Power OFF

    # ==================== Properties ====================

    @property
    def position(self) -> float:
        """Current position (radians)"""
        return self._last_position

    @property
    def velocity(self) -> float:
        """Current velocity (rad/s)"""
        return self._last_velocity

    @property
    def torque(self) -> float:
        """Current torque (Nm)"""
        return self._last_torque

    @property
    def temperature(self) -> float:
        """Current temperature (°C)"""
        return self._last_temperature

    @property
    def is_enabled(self) -> bool:
        """Motor power status"""
        return self._is_enabled


# ==================== Enhanced Multi-Motor Control Class ====================
class MotorGroup:
    """
    Enhanced class for controlling multiple motors

    Improvements (v2.1):
    - Clear support for both individual and group control
    - Support assigning names to motors
    - Supports partial group control
    - Optional enable/disable support

    Key features:
    1. Synchronized control of all motors
    2. Individual motor control (by index or name)
    3. Partial group control
    4. Optional enable/disable

    Examples:
        # Basic usage
        motors = MotorGroup([
            ('AK80-64', 1, 'shoulder'),
            ('AK80-64', 2, 'elbow'),
            ('AK80-9', 3, 'wrist')
        ])

        with motors:
            # 1. Control all motors simultaneously
            motors.move_all([1.57, 0.0, -1.57], duration=2.0)

            # 2. Individual motor control (by index)
            motors.move_motor(0, position=1.57, duration=1.0)
            motors.set_motor_velocity(1, velocity=2.0)
            motors.set_motor_torque(2, torque=3.0)

            # 3. Individual motor control (by name)
            motors.move_motor_by_name('shoulder', position=1.57)
            motors.set_motor_velocity_by_name('elbow', velocity=2.0)

            # 4. Partial group control
            motors.move_selected([0, 1], [1.57, 0.0], duration=2.0)

            # 5. Direct access (using Motor API)
            motors[0].track_trajectory(1.57)
            motors['shoulder'].set_velocity(2.0)
    """

    def __init__(self,
                 motor_configs: List[Union[Tuple[str, int], Tuple[str, int, str]]],
                 can_interface: str = 'can0',
                 bitrate: int = 1000000):
        """
        Initialize multiple motors

        Args:
            motor_configs: list of motor configurations
                - (motor_type, motor_id) or
                - (motor_type, motor_id, name)
            can_interface: CAN interface name
            bitrate: CAN bitrate

        Examples:
            # Without names
            motors = MotorGroup([
                ('AK80-64', 1),
                ('AK80-64', 2)
            ])

            # With names (recommended)
            motors = MotorGroup([
                ('AK80-64', 1, 'shoulder'),
                ('AK80-64', 2, 'elbow'),
                ('AK80-9', 3, 'wrist')
            ])
        """
        self.motors: List[Motor] = []
        self.motor_names: Dict[str, int] = {}  # name -> index mapping
        self._initialized_count = 0

        try:
            # Setup CAN interface
            first_config = MotorConfig(
                can_interface=can_interface,
                bitrate=bitrate
            )
            CANInterface.setup_can_interface(config=first_config)

            # Create motor objects
            for i, config in enumerate(motor_configs):
                # Extract name depending on tuple length
                if len(config) == 2:
                    motor_type, motor_id = config
                    name = f"motor_{i}"  # default name
                elif len(config) == 3:
                    motor_type, motor_id, name = config
                else:
                    raise ValueError(f"Invalid config format: {config}")

                # Create motor
                motor = Motor(
                    motor_type=motor_type,
                    motor_id=motor_id,
                    can_interface=can_interface,
                    auto_init=False
                )
                motor.config.bitrate = bitrate

                if motor.initialize():
                    self.motors.append(motor)
                    self.motor_names[name] = i
                    self._initialized_count += 1
                    logging.info(f"Motor '{name}' (ID: {motor_id}) initialized")
                else:
                    logging.warning(f"Failed to initialize motor '{name}' (ID: {motor_id})")

            if self._initialized_count == 0:
                raise RuntimeError("Failed to initialize any motors")

            logging.info(f"MotorGroup initialized with {len(self.motors)} motors")
            logging.info(f"Motor names: {list(self.motor_names.keys())}")

        except Exception as e:
            logging.error(f"MotorGroup initialization failed: {e}")
            self._cleanup_motors()
            raise

    def _cleanup_motors(self):
        """Cleanup motors on initialization failure"""
        for motor in self.motors:
            try:
                if motor._is_enabled:
                    motor.disable()
            except Exception as e:
                logging.warning(f"Error cleaning up motor: {e}")
        self.motors.clear()
        self.motor_names.clear()

    # ==================== Enable/Disable ====================

    def enable_all(self) -> None:
        """Enable all motors"""
        logging.info(f"Enabling all {len(self.motors)} motors...")
        failed_motors = []

        for i, motor in enumerate(self.motors):
            name = self._get_name_by_index(i)
            logging.info(f"Enabling motor '{name}' ({i+1}/{len(self.motors)})...")
            if not motor.enable():
                failed_motors.append(name)

        if failed_motors:
            logging.warning(f"Failed to enable motors: {failed_motors}")
        else:
            logging.info("All motors enabled")

    def disable_all(self) -> None:
        """Disable all motors"""
        logging.info(f"Disabling all {len(self.motors)} motors...")
        for i, motor in enumerate(self.motors):
            name = self._get_name_by_index(i)
            logging.info(f"Disabling motor '{name}' ({i+1}/{len(self.motors)})...")
            motor.disable()
        logging.info("All motors disabled")

    def enable_motor(self, index: int) -> bool:
        """Enable a specific motor (by index)"""
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Enabling motor '{name}' (index {index})...")
        return self.motors[index].enable()

    def disable_motor(self, index: int) -> None:
        """Disable a specific motor (by index)"""
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Disabling motor '{name}' (index {index})...")
        self.motors[index].disable()

    def enable_motor_by_name(self, name: str) -> bool:
        """Enable a specific motor (by name)"""
        index = self._get_index_by_name(name)
        return self.enable_motor(index)

    def disable_motor_by_name(self, name: str) -> None:
        """Disable a specific motor (by name)"""
        index = self._get_index_by_name(name)
        self.disable_motor(index)

    # ==================== Group control (synchronized) ====================

    def move_all(self,
                 positions: List[float],
                 kp: Optional[float] = None,
                 kd: Optional[float] = None,
                 duration: float = 0.0,
                 trajectory_type: str = 'minimum_jerk') -> None:
        """
        Move all motors simultaneously (synchronized trajectory)

        Args:
            positions: list of target positions per selected motors
            kp: position gain
            kd: velocity gain
            duration: motion time (0 for immediate move)
            trajectory_type: trajectory type

        Examples:
            # Immediate move
            motors.move_all([1.57, 0.0, -1.57])

            # Smooth motion over 2 seconds
            motors.move_all([1.57, 0.0, -1.57], duration=2.0)
        """
        if len(positions) != len(self.motors):
            raise ValueError(f"Position list length ({len(positions)}) must match "
                           f"number of motors ({len(self.motors)})")

        if duration > STEP_COMMAND_THRESHOLD:
            # Track synchronized trajectory
            start_time = time.time()
            start_positions = [motor.position for motor in self.motors]

            # Choose trajectory function
            if trajectory_type == 'minimum_jerk':
                traj_func = TrajectoryGenerator.minimum_jerk
            elif trajectory_type == 'cubic':
                traj_func = TrajectoryGenerator.cubic
            elif trajectory_type == 'linear':
                traj_func = TrajectoryGenerator.linear
            elif trajectory_type == 'trapezoidal':
                traj_func = TrajectoryGenerator.trapezoidal
            else:
                raise ValueError(f"Unknown trajectory type: {trajectory_type}")

            # Set default gains
            if kp is None:
                kp = self.motors[0].config.default_kp
            if kd is None:
                kd = self.motors[0].config.default_kd

            logging.info(f"Synchronized trajectory for all motors: {duration:.2f}s")

            # Trajectory tracking loop
            while True:
                t = time.time() - start_time
                if t >= duration:
                    break

                # Compute and send trajectory for each motor
                for motor, start_pos, target_pos in zip(self.motors, start_positions, positions):
                    p, v = traj_func(start_pos, target_pos, t, duration)
                    motor.send_command(p, v, kp, kd, 0.0)

                time.sleep(CONTROL_LOOP_PERIOD)

            # Final stabilization
            for motor, pos in zip(self.motors, positions):
                motor.send_command(pos, 0.0, kp, kd, 0.0)

            logging.info(f"All motors trajectory complete ({duration:.2f}s)")
        else:
            # Immediate move
            for motor, pos in zip(self.motors, positions):
                motor.track_trajectory(pos, kp, kd, duration=0)

    def set_all_velocity(self,
                        velocities: List[float],
                        kd: Optional[float] = None) -> None:
        """
        Set velocity for all motors

        Args:
            velocities: list of target velocities per motor
            kd: velocity gain

        Examples:
            motors.set_all_velocity([2.0, 1.0, 0.5])
        """
        if len(velocities) != len(self.motors):
            raise ValueError(f"Velocity list length ({len(velocities)}) must match "
                           f"number of motors ({len(self.motors)})")

        for motor, vel in zip(self.motors, velocities):
            motor.set_velocity(vel, kd)

    def set_all_torque(self,
                      torques: List[float]) -> None:
        """
        모든 모터의 토크 설정

        Args:
            torques: list of target torques per motor

        Examples:
            motors.set_all_torque([2.0, 1.5, 1.0])
        """
        if len(torques) != len(self.motors):
            raise ValueError(f"Torque list length ({len(torques)}) must match "
                           f"number of motors ({len(self.motors)})")

        for motor, torque in zip(self.motors, torques):
            motor.set_torque(torque)

    def stop_all(self) -> None:
        """Stop all motors (set velocity to 0)"""
        logging.info("Stopping all motors...")
        for motor in self.motors:
            motor.set_velocity(0.0)
        logging.info("All motors stopped")

    # ==================== Individual Motor Control (by index) ====================

    def move_motor(self,
                   index: int,
                   position: float,
                   kp: Optional[float] = None,
                   kd: Optional[float] = None,
                   duration: float = 0.0,
                   trajectory_type: str = 'minimum_jerk') -> None:
        """
        Move a specific motor (by index)

        Args:
            index: motor index
            position: target position
            kp: position gain
            kd: velocity gain
            duration: motion time
            trajectory_type: trajectory type

        Examples:
            # Immediate move
            motors.move_motor(0, position=1.57)

            # Smooth motion over 2 seconds
            motors.move_motor(0, position=1.57, duration=2.0)
        """
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Moving motor '{name}' (index {index}) to {position:.3f} rad")
        self.motors[index].track_trajectory(position, kp, kd, duration, trajectory_type)

    def set_motor_velocity(self,
                          index: int,
                          velocity: float,
                          kd: Optional[float] = None) -> None:
        """
        Set velocity for a specific motor (by index)

        Args:
            index: motor index
            velocity: target velocity
            kd: velocity gain

        Examples:
            motors.set_motor_velocity(0, velocity=2.0)
        """
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Setting motor '{name}' (index {index}) velocity to {velocity:.3f} rad/s")
        self.motors[index].set_velocity(velocity, kd)

    def set_motor_torque(self,
                        index: int,
                        torque: float) -> None:
        """
        Set torque for a specific motor (by index)

        Args:
            index: motor index
            torque: target torque

        Examples:
            motors.set_motor_torque(0, torque=3.0)
        """
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Setting motor '{name}' (index {index}) torque to {torque:.3f} Nm")
        self.motors[index].set_torque(torque)

    def stop_motor(self, index: int) -> None:
        """Stop a specific motor (by index)"""
        self._validate_index(index)
        name = self._get_name_by_index(index)
        logging.info(f"Stopping motor '{name}' (index {index})")
        self.motors[index].set_velocity(0.0)

    # ==================== Individual motor control (by name) ====================

    def move_motor_by_name(self,
                          name: str,
                          position: float,
                          kp: Optional[float] = None,
                          kd: Optional[float] = None,
                          duration: float = 0.0,
                          trajectory_type: str = 'minimum_jerk') -> None:
        """
        Move a specific motor (by name)

        Examples:
            motors.move_motor_by_name('shoulder', position=1.57, duration=2.0)
        """
        index = self._get_index_by_name(name)
        self.move_motor(index, position, kp, kd, duration, trajectory_type)

    def set_motor_velocity_by_name(self,
                                   name: str,
                                   velocity: float,
                                   kd: Optional[float] = None) -> None:
        """
        Set velocity for a specific motor (by name)

        Examples:
            motors.set_motor_velocity_by_name('elbow', velocity=2.0)
        """
        index = self._get_index_by_name(name)
        self.set_motor_velocity(index, velocity, kd)

    def set_motor_torque_by_name(self,
                                name: str,
                                torque: float) -> None:
        """
        Set torque for a specific motor (by name)

        Examples:
            motors.set_motor_torque_by_name('wrist', torque=3.0)
        """
        index = self._get_index_by_name(name)
        self.set_motor_torque(index, torque)

    def stop_motor_by_name(self, name: str) -> None:
        """Stop a specific motor (by name)"""
        index = self._get_index_by_name(name)
        self.stop_motor(index)

    # ==================== Partial group control ====================

    def move_selected(self,
                     indices: List[int],
                     positions: List[float],
                     kp: Optional[float] = None,
                     kd: Optional[float] = None,
                     duration: float = 0.0,
                     trajectory_type: str = 'minimum_jerk') -> None:
        """
        Move only selected motors

        Args:
            indices: list of motor indices
            positions: list of target positions per selected motors
            kp: position gain
            kd: velocity gain
            duration: motion time
            trajectory_type: trajectory type

        Examples:
            # Move only motors 0 and 1
            motors.move_selected([0, 1], [1.57, 0.0], duration=2.0)
        """
        if len(indices) != len(positions):
            raise ValueError("indices and positions must have the same length")

        for idx in indices:
            self._validate_index(idx)

        selected_motors = [self.motors[i] for i in indices]

        if duration > STEP_COMMAND_THRESHOLD:
            # Synchronized trajectory
            start_time = time.time()
            start_positions = [motor.position for motor in selected_motors]

            if trajectory_type == 'minimum_jerk':
                traj_func = TrajectoryGenerator.minimum_jerk
            elif trajectory_type == 'cubic':
                traj_func = TrajectoryGenerator.cubic
            elif trajectory_type == 'linear':
                traj_func = TrajectoryGenerator.linear
            elif trajectory_type == 'trapezoidal':
                traj_func = TrajectoryGenerator.trapezoidal
            else:
                raise ValueError(f"Unknown trajectory type: {trajectory_type}")

            if kp is None:
                kp = selected_motors[0].config.default_kp
            if kd is None:
                kd = selected_motors[0].config.default_kd

            logging.info(f"Synchronized trajectory for selected motors {indices}: {duration:.2f}s")

            while True:
                t = time.time() - start_time
                if t >= duration:
                    break

                for motor, start_pos, target_pos in zip(selected_motors, start_positions, positions):
                    p, v = traj_func(start_pos, target_pos, t, duration)
                    motor.send_command(p, v, kp, kd, 0.0)

                time.sleep(CONTROL_LOOP_PERIOD)

            for motor, pos in zip(selected_motors, positions):
                motor.send_command(pos, 0.0, kp, kd, 0.0)

            logging.info(f"Selected motors trajectory complete")
        else:
            # Immediate move
            for motor, pos in zip(selected_motors, positions):
                motor.track_trajectory(pos, kp, kd, duration=0)

    def move_selected_by_names(self,
                              names: List[str],
                              positions: List[float],
                              kp: Optional[float] = None,
                              kd: Optional[float] = None,
                              duration: float = 0.0,
                              trajectory_type: str = 'minimum_jerk') -> None:
        """
        Move only selected motors (이름으로)

        Examples:
            motors.move_selected_by_names(
                ['shoulder', 'elbow'],
                [1.57, 0.0],
                duration=2.0
            )
        """
        indices = [self._get_index_by_name(name) for name in names]
        self.move_selected(indices, positions, kp, kd, duration, trajectory_type)

    # ==================== Status queries ====================

    def update_all(self) -> List[Dict[str, float]]:
        """Update all motor states"""
        results = []
        for motor in self.motors:
            try:
                results.append(motor.update())
            except Exception as e:
                logging.error(f"Failed to update motor: {e}")
                results.append({})
        return results

    def get_positions(self) -> List[float]:
        """Current positions for all motors"""
        return [motor.position for motor in self.motors]

    def get_velocities(self) -> List[float]:
        """Current velocities for all motors"""
        return [motor.velocity for motor in self.motors]

    def get_torques(self) -> List[float]:
        """Current torques for all motors"""
        return [motor.torque for motor in self.motors]

    def get_temperatures(self) -> List[float]:
        """Current temperatures for all motors"""
        return [motor.temperature for motor in self.motors]

    def get_motor_position(self, index: int) -> float:
        """Current position of a specific motor (by index)"""
        self._validate_index(index)
        return self.motors[index].position

    def get_motor_position_by_name(self, name: str) -> float:
        """Current position of a specific motor (by name)"""
        index = self._get_index_by_name(name)
        return self.motors[index].position

    def get_status(self) -> Dict[str, Dict]:
        """Status of all motors"""
        status = {}
        for name, index in self.motor_names.items():
            motor = self.motors[index]
            status[name] = {
                'index': index,
                'is_enabled': motor.is_enabled,
                'position': motor.position,
                'velocity': motor.velocity,
                'torque': motor.torque,
                'temperature': motor.temperature
            }
        return status

    def print_status(self):
        """Print motor status"""
        print("\n" + "="*60)
        print("Motor Group Status")
        print("="*60)
        for name, index in self.motor_names.items():
            motor = self.motors[index]
            status = "ON" if motor.is_enabled else "OFF"
            print(f"[{index}] {name:12s} | Status: {status:3s} | "
                  f"Pos: {motor.position:6.3f} rad | "
                  f"Vel: {motor.velocity:6.3f} rad/s | "
                  f"Torque: {motor.torque:5.2f} Nm | "
                  f"Temp: {motor.temperature:5.1f}°C")
        print("="*60 + "\n")

    def zero_all_positions(self) -> None:
        """Set zero position for all motors"""
        for motor in self.motors:
            motor.zero_position()

    # ==================== Helper methods ====================

    def _validate_index(self, index: int):
        """Validate index"""
        if not 0 <= index < len(self.motors):
            raise IndexError(f"Motor index {index} out of range [0, {len(self.motors)-1}]")

    def _get_index_by_name(self, name: str) -> int:
        """Find index by name"""
        if name not in self.motor_names:
            available = list(self.motor_names.keys())
            raise KeyError(f"Motor '{name}' not found. Available motors: {available}")
        return self.motor_names[name]

    def _get_name_by_index(self, index: int) -> str:
        """Find name by index"""
        for name, idx in self.motor_names.items():
            if idx == index:
                return name
        return f"motor_{index}"

    def get_motor_names(self) -> List[str]:
        """List all motor names"""
        return list(self.motor_names.keys())

    # ==================== Context Manager ====================

    def __enter__(self):
        """Context manager enter - enable all motors"""
        self.enable_all()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disable all motors"""
        self.disable_all()

    # ==================== Special methods ====================

    def __len__(self):
        """Return number of motors"""
        return len(self.motors)

    def __getitem__(self, key: Union[int, str]):
        """
        Access motor by index or name

        Examples:
            motor = motors[0]           # Access by index
            motor = motors['shoulder']  # Access by name
        """
        if isinstance(key, int):
            self._validate_index(key)
            return self.motors[key]
        elif isinstance(key, str):
            index = self._get_index_by_name(key)
            return self.motors[index]
        else:
            raise TypeError(f"Invalid key type: {type(key)}")

    def __repr__(self):
        """String representation"""
        return f"MotorGroup({len(self.motors)} motors: {list(self.motor_names.keys())})"


# ==================== Sudo Permission Setup Helper ====================
def setup_sudo_permissions():
    """
    Print instructions for setting up sudo permissions

    Allows CAN interface control without password prompt.
    """
    username = os.getenv('USER', 'pi')

    print("\n" + "="*60)
    print("CAN Interface Sudo Permission Setup")
    print("="*60)
    print("\nRun the following commands:\n")
    print("1. Edit sudoers file:")
    print("   sudo visudo\n")
    print("2. Add this line at the end:")
    print(f"   {username} ALL=(ALL) NOPASSWD: /sbin/ip\n")
    print("3. Save and exit with Ctrl+X\n")
    print("Or safer method:")
    print("   sudo nano /etc/sudoers.d/can-interface")
    print(f"   {username} ALL=(ALL) NOPASSWD: /sbin/ip\n")
    print("="*60 + "\n")

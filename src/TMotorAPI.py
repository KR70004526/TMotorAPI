"""
TMotor Control API v5.1 - Non-blocking Control with Soft Limit
Based on TMotorCANControl by Neurobionics Lab

Changes from v4.3:
- Removed duration parameter from set_* methods (non-blocking design)
- Removed unused variables (stepTimeout, stepTolerance, stepSettlingTime)
- Fixed zero_position() causing unwanted movement
- Added stop() method for emergency stop
- Added soft limit feature with smooth stopping (v5.0)
- Auto-register bundled libusb-1.0 DLL for Windows gs_usb backend (v5.1)

Author: TMotor Control Team
License: MIT
"""

import os
import sys
import time
import subprocess
import logging
import re
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
import numpy as np

# ==================== libusb backend bootstrap (Windows / gs_usb) ====================
# gs_usb (gs_usb/gs_usb.py) talks to the adapter through pyusb's default libusb1
# backend: usb.backend.libusb1.get_backend(), which locates libusb-1.0.dll via
# find_library() -> PATH. The libusb-package wheel ships that DLL but does NOT put
# it on PATH, so on a fresh env get_backend() returns None and GsUsb.scan() finds
# nothing (silently, no error). Register the bundled DLL directory here at import
# time so gs_usb works out of the box without any manual DLL copying.
try:
    import libusb_package
    _libusb_dir = os.path.dirname(str(libusb_package.get_library_path()))
    if _libusb_dir:
        if _libusb_dir not in os.environ.get("PATH", "").split(os.pathsep):
            os.environ["PATH"] = _libusb_dir + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(_libusb_dir)
            except (OSError, ValueError):
                pass
except Exception:
    # libusb-package absent (e.g. Linux/socketcan) -> nothing to register
    pass

try:
    from TMotorCANControl.mit_can import TMotorManager_mit_can
except ImportError:
    print("Error: TMotorCANControl not installed")
    print("Run: pip install TMotorCANControl")
    sys.exit(1)


# ==================== Constants ====================
ZERO_POSITION_SETTLE_TIME = 1.0  # Wait time after zeroing (EEPROM save)
CAN_INTERFACE_PATTERN = re.compile(r'^can\d+$')

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# ==================== Configuration ====================
@dataclass
class MotorConfig:
    """
    Motor configuration
    
    Attributes:
        motorType: Motor model (e.g., 'AK80-64', 'AK80-9', 'AK70-10')
        motorId: CAN ID (0-127)
        canInterface: CAN interface name (e.g., 'can0', 'can1')
        bitrate: CAN bitrate (default: 1000000)
        autoInit: Auto initialize CAN interface
        maxTemperature: Maximum safe MOSFET temperature (°C)
        defaultKp: Default position gain (Nm/rad)
        defaultKd: Default velocity gain (Nm/(rad/s))
    """
    motorType: str = 'AK70-10'
    motorId: int = 1
    canInterface: str = 'can0'
    bitrate: int = 1000000
    autoInit: bool = True
    maxTemperature: float = 50.0
    
    # Default control gains
    defaultKp: float = 10.0
    defaultKd: float = 0.5

    # CAN Interface Backend
    canBackend: str = "socketcan"   # Linux -> socketcan | Window -> gs_usb
    canChannel: object = "can0"     # socketcan -> can0 | Window -> 0

    def __post_init__(self):
        if not 0 <= self.motorId <= 127:
            raise ValueError(f"motorId must be 0-127, got {self.motorId}")

        # backend별 channel 타입 검증
        if self.canBackend in ("socketcan", "socketcan_native"):
            if not isinstance(self.canChannel, str) or not CAN_INTERFACE_PATTERN.match(self.canChannel):
                raise ValueError(f"SocketCAN channel must be like 'can0', got {self.canChannel}")
        elif self.canBackend == "gs_usb":
            if not isinstance(self.canChannel, int):
                raise ValueError(f"gs_usb channel must be int (e.g., 0), got {self.canChannel}")
        else:
            raise ValueError(f"Unsupported canBackend: {self.canBackend}")


# ==================== CAN Interface ====================
class CANInterface:
    @staticmethod
    def setup_interface(canInterface: Optional[str] = None,
                        bitrate: Optional[int] = None,
                        config: Optional[MotorConfig] = None) -> bool:
        """Setup CAN interface using system commands"""

        if config is not None:
            _interface = canInterface if canInterface is not None else config.canInterface
            _bitrate = bitrate if bitrate is not None else config.bitrate
        else:
            _interface = canInterface if canInterface is not None else 'can0'
            _bitrate = bitrate if bitrate is not None else 1000000

        # ✅ OS/백엔드 가드 추가: Windows에서는 ip link 자체가 의미 없음
        if os.name == "nt":
            logging.info(f"Skipping CAN bring-up on Windows (interface={_interface})")
            return True

        # ✅ socketcan 이름이 아니면(예: gs_usb) ip link 스킵
        #    (현재 코드 구조상 _interface는 'can0' 같은 채널 문자열이어야 함)
        if not CAN_INTERFACE_PATTERN.match(_interface):
            # 원래는 ValueError였지만, "전체 유지"가 목적이면 여기서 스킵이 더 실용적
            # 그래도 명시적으로 막고 싶으면 기존 ValueError 유지해도 됨.
            logging.info(f"Skipping CAN bring-up for non-socketcan interface name: {_interface}")
            return True

        try:
            subprocess.run(['sudo', 'ip', 'link', 'set', _interface, 'down'],
                           check=True, capture_output=True)
            subprocess.run(['sudo', 'ip', 'link', 'set', _interface,
                            'type', 'can', 'bitrate', str(_bitrate)],
                           check=True, capture_output=True)
            subprocess.run(['sudo', 'ip', 'link', 'set', _interface, 'up'],
                           check=True, capture_output=True)

            logging.info(f"✓ CAN {_interface} ready (bitrate: {_bitrate})")
            return True
        except subprocess.CalledProcessError as e:
            err = getattr(e, "stderr", b"")
            logging.error(f"Failed to setup CAN: {err.decode(errors='ignore')}")
            return False


# ==================== Trajectory Generator ====================
class TrajectoryGenerator:
    """Trajectory generation utilities"""
    
    @staticmethod
    def minimum_jerk(startPos: float, endPos: float,
                    currentTime: float, totalDuration: float) -> Tuple[float, float]:
        """Minimum jerk trajectory (5th order polynomial)"""
        if currentTime >= totalDuration:
            return endPos, 0.0
        
        tau = currentTime / totalDuration
        posDelta = endPos - startPos
        
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        sDot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / totalDuration
        
        return startPos + posDelta * s, posDelta * sDot
    
    @staticmethod
    def cubic(startPos: float, endPos: float,
             currentTime: float, totalDuration: float) -> Tuple[float, float]:
        """Cubic polynomial trajectory"""
        if currentTime >= totalDuration:
            return endPos, 0.0
        
        tau = currentTime / totalDuration
        posDelta = endPos - startPos
        
        s = -2 * tau**3 + 3 * tau**2
        sDot = (-6 * tau**2 + 6 * tau) / totalDuration
        
        return startPos + posDelta * s, posDelta * sDot
    
    @staticmethod
    def linear(startPos: float, endPos: float,
              currentTime: float, totalDuration: float) -> Tuple[float, float]:
        """Linear interpolation"""
        if currentTime >= totalDuration:
            return endPos, 0.0
        
        posDelta = endPos - startPos
        velocity = posDelta / totalDuration
        return startPos + velocity * currentTime, velocity


# ==================== Motor Class ====================
class Motor:
    """
    High-level motor control API (Non-blocking design with Soft Limit)
    
    Usage:
        # Basic usage
        with Motor('AK70-10', motorId=1) as motor:
            while running:
                motor.set_position(target)
                motor.update()
                time.sleep(0.01)
        
        # With soft limit
        with Motor('AK70-10', motorId=1) as motor:
            motor.set_soft_limit(-1.57, 1.57)
            while running:
                motor.set_position_smooth(target)
                motor.update()
                time.sleep(0.01)
    """
    
    def __init__(self,
                 motorType: Optional[str] = None,
                 motorId: Optional[int] = None,
                 canInterface: Optional[str] = None,
                 bitrate: Optional[int] = None,
                 autoInit: Optional[bool] = None,
                 maxTemperature: Optional[float] = None,
                 config: Optional[MotorConfig] = None,
                 **kwargs):
        """Initialize Motor object"""
        
        # Handle config parameter
        if config is not None:
            self.config = config
        else:
            params = {}
            if motorType is not None:
                params['motorType'] = motorType
            if motorId is not None:
                params['motorId'] = motorId
            if canInterface is not None:
                params['canInterface'] = canInterface
            if bitrate is not None:
                params['bitrate'] = bitrate
            if autoInit is not None:
                params['autoInit'] = autoInit
            if maxTemperature is not None:
                params['maxTemperature'] = maxTemperature
            
            params.update(kwargs)
            self.config = MotorConfig(**params)
        
        # Internal state
        self._manager: Optional[TMotorManager_mit_can] = None
        self._isEnabled = False
        self._powerOnTime = 0.0
        
        # Last known state
        self._lastPosition = 0.0
        self._lastVelocity = 0.0
        self._lastTorque = 0.0
        self._lastTemperature = 0.0
        self._lastCurrent = 0.0          # q-axis current (A)
        self._lastAcceleration = 0.0     # output acceleration (rad/s^2)
        self._lastError = 0              # motor error code
        self._lastControlMode = "IDLE"   # _TMotorManState name

        # Soft limit parameters (internal only)
        self._softLimitMin: Optional[float] = None
        self._softLimitMax: Optional[float] = None
        self._softZone: float = 0.2
        self._baseKd: float = 2.0
        self._maxKd: float = 20.0
        
        # Setup CAN interface
        if self.config.autoInit:
            CANInterface.setup_interface(config=self.config)
        
        # Initialize TMotorManager
        try:
            self._manager = TMotorManager_mit_can(
                motor_type=self.config.motorType,
                motor_ID=self.config.motorId,
                max_mosfett_temp=int(self.config.maxTemperature),
                can_interface=self.config.canBackend,      # 핵심
                can_channel=self.config.canChannel,        # 핵심
                can_bitrate=self.config.bitrate,
                can_bring_up=False,
            )
            logging.info(f"✓ Motor connected: {self.config.motorType} ID={self.config.motorId}")
        except Exception as e:
            logging.error(f"Failed to connect motor: {e}")
            raise
    
    def __enter__(self):
        self.enable()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disable()
        return False
    
    # ==================== Properties ====================
    @property
    def position(self) -> float:
        """Current position (rad)"""
        return self._lastPosition
    
    @property
    def velocity(self) -> float:
        """Current velocity (rad/s)"""
        return self._lastVelocity
    
    @property
    def torque(self) -> float:
        """Current torque (Nm)"""
        return self._lastTorque
    
    @property
    def temperature(self) -> float:
        """Current MOSFET temperature (°C)"""
        return self._lastTemperature

    @property
    def current(self) -> float:
        """Current q-axis current (A)"""
        return self._lastCurrent

    @property
    def acceleration(self) -> float:
        """Current output acceleration (rad/s^2)"""
        return self._lastAcceleration

    @property
    def error(self) -> int:
        """Latest motor error code (0 = OK)"""
        return self._lastError

    @property
    def control_mode(self) -> str:
        """Current control state (IDLE/IMPEDANCE/CURRENT/SPEED/FULL_STATE)"""
        return self._lastControlMode

    # ==================== Power Control ====================
    def enable(self) -> None:
        """Enable motor (Power ON)"""
        if self._isEnabled:
            logging.warning("Motor already enabled")
            return
        
        if self._manager is None:
            raise RuntimeError("Motor manager not initialized")
        
        self._manager.__enter__()
        self._isEnabled = True
        self._powerOnTime = time.time()
        
        time.sleep(0.1)
        self.update()
        
        logging.info("=" * 50)
        logging.info("Motor ENABLED")
        logging.info("=" * 50)
    
    def disable(self) -> None:
        """Disable motor (Power OFF)"""
        if self._manager and self._isEnabled:
            uptime = time.time() - self._powerOnTime
            self._manager.__exit__(None, None, None)
            self._isEnabled = False
            
            logging.info("=" * 50)
            logging.info(f"Motor DISABLED (uptime: {uptime:.1f}s)")
            logging.info("=" * 50)
    
    def is_power_on(self) -> bool:
        """Check if motor is powered on"""
        return self._isEnabled
    
    def get_uptime(self) -> float:
        """Get motor uptime in seconds"""
        if self._isEnabled:
            return time.time() - self._powerOnTime
        return 0.0
    
    # ==================== State Update ====================
    def update(self) -> Dict[str, float]:
        """Send command and receive motor state"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        self._manager.update()
        
        self._lastPosition = self._manager.position
        self._lastVelocity = self._manager.velocity
        self._lastTorque = self._manager.torque
        self._lastTemperature = self._manager.get_temperature_celsius()
        self._lastCurrent = self._manager.get_current_qaxis_amps()
        self._lastAcceleration = self._manager.get_output_acceleration_radians_per_second_squared()
        self._lastError = self._manager.get_motor_error_code()
        self._lastControlMode = self._manager._control_state.name

        return {
            'position': self._lastPosition,
            'velocity': self._lastVelocity,
            'torque': self._lastTorque,
            'temperature': self._lastTemperature,
            'current': self._lastCurrent,
            'acceleration': self._lastAcceleration,
            'error': self._lastError,
            'control_mode': self._lastControlMode,
        }
    
    # ==================== Soft Limit Methods ====================
    def set_soft_limit(self, 
                      min_pos: float, 
                      max_pos: float,
                      soft_zone: float = 0.2,
                      base_kd: float = 2.0,
                      max_kd: float = 20.0) -> None:
        """
        Set soft limit range
        
        Args:
            min_pos: Minimum position limit (rad)
            max_pos: Maximum position limit (rad)
            soft_zone: Damping increase zone near limits (rad)
            base_kd: Base damping in normal zone
            max_kd: Maximum damping at limits (bumper effect)
        
        Example:
            motor.set_soft_limit(-1.57, 1.57)  # -90° to +90°
        """
        self._softLimitMin = min_pos
        self._softLimitMax = max_pos
        self._softZone = soft_zone
        self._baseKd = base_kd
        self._maxKd = max_kd
        
        logging.info(f"Soft limit set: {min_pos:.2f} ~ {max_pos:.2f} rad, "
                    f"zone: {soft_zone:.2f} rad")
    
    def set_position_smooth(self,
                           targetPos: float,
                           kp: Optional[float] = None,
                           feedTor: float = 0.0,
                           min_pos: Optional[float] = None,
                           max_pos: Optional[float] = None,
                           soft_zone: Optional[float] = None,
                           base_kd: Optional[float] = None,
                           max_kd: Optional[float] = None) -> None:
        """
        Set position command with soft limit and smooth stopping
        
        Damping automatically increases near limits for smooth deceleration.
        
        Args:
            targetPos: Target position (rad)
            kp: Position gain (Nm/rad), None to use default
            feedTor: Feedforward torque (Nm)
            min_pos: Minimum position (rad), None to use configured value
            max_pos: Maximum position (rad), None to use configured value
            soft_zone: Damping increase zone (rad), None to use configured value
            base_kd: Base damping, None to use configured value
            max_kd: Maximum damping at limits, None to use configured value
        
        Usage:
            # Method 1: Pre-configure limits
            motor.set_soft_limit(-1.57, 1.57)
            motor.set_position_smooth(target)
            motor.update()
            
            # Method 2: Specify limits each time
            motor.set_position_smooth(target, min_pos=-1.57, max_pos=1.57)
            motor.update()
        
        Raises:
            ValueError: If soft limits are not configured
        """
        # Determine parameters
        min_pos = min_pos if min_pos is not None else self._softLimitMin
        max_pos = max_pos if max_pos is not None else self._softLimitMax
        soft_zone = soft_zone if soft_zone is not None else self._softZone
        base_kd = base_kd if base_kd is not None else self._baseKd
        max_kd = max_kd if max_kd is not None else self._maxKd
        kp = kp if kp is not None else self.config.defaultKp
        
        # Check if soft limits are configured
        if min_pos is None or max_pos is None:
            raise ValueError(
                "Soft limits not configured. "
                "Call set_soft_limit() first or specify min_pos/max_pos."
            )
        
        # Clamp target position to limits
        safe_pos = max(min_pos, min(max_pos, targetPos))
        
        # Get current position
        current = self.position
        
        # Calculate damping based on proximity to limits
        kd = base_kd
        
        # Near upper limit
        if current > max_pos - soft_zone:
            progress = (current - (max_pos - soft_zone)) / soft_zone
            kd = base_kd + (max_kd - base_kd) * min(progress, 1.0)
        
        # Near lower limit
        elif current < min_pos + soft_zone:
            progress = ((min_pos + soft_zone) - current) / soft_zone
            kd = base_kd + (max_kd - base_kd) * min(progress, 1.0)
        
        # Send position command with adjusted damping
        self.set_position(safe_pos, kp=kp, kd=kd, feedTor=feedTor)
    
    # ==================== Control Commands ====================
    def set_position(self,
                     targetPos: float,
                     kp: Optional[float] = None,
                     kd: Optional[float] = None,
                     feedTor: float = 0.0) -> None:
        """
        Set position command (non-blocking)
        
        Args:
            targetPos: Target position (rad)
            kp: Position gain (Nm/rad)
            kd: Velocity gain (Nm/(rad/s))
            feedTor: Feedforward torque (Nm)
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        if kp is None:
            kp = self.config.defaultKp
        if kd is None:
            kd = self.config.defaultKd
        
        self._manager.set_impedance_gains_real_unit_full_state_feedback(K=kp, B=kd)
        self._manager.position = targetPos
        self._manager.torque = feedTor
    
    def set_velocity(self,
                     targetVel: float,
                     kd: Optional[float] = None) -> None:
        """
        Set velocity command (non-blocking)
        
        Args:
            targetVel: Target velocity (rad/s)
            kd: Velocity gain
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        if kd is None:
            kd = self.config.defaultKd
        
        self._manager.set_speed_gains(kd=kd)
        self._manager.velocity = targetVel

    def set_fullState(self,
                      targetPos: float = 0.0,
                      targetVel: float = 0.0,
                      kp: float = 0.0,
                      kd: Optional[float] = None,
                      feedTor: float = 0.0) -> None:
        """
        Full-state MIT command (non-blocking).

        Sends position + velocity feedback with feedforward torque in ONE command:
            tau = kp*(targetPos - p) + kd*(targetVel - v) + feedTor
        Superset of set_position (no velocity term) and set_velocity (no
        feedforward). With kp=0 this is velocity damping toward targetVel plus a
        feedforward torque -- e.g. an ergometer resistance (feedTor) that also
        regulates cadence (kd, targetVel), computed in motor firmware.

        Args:
            targetPos: Target position (rad). Ignored when kp=0.
            targetVel: Target velocity (rad/s).
            kp: Position gain / stiffness (Nm/rad). 0 = no position hold.
            kd: Velocity gain / damping (Nm/(rad/s)). None -> config.defaultKd.
            feedTor: Feedforward torque (Nm).

        Note: kp/kd/targetVel/targetPos are range-checked by the driver per motor
        type (e.g. AK80-64: Kd 0..5, V +-8 rad/s). Out-of-range raises.
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        if kd is None:
            kd = self.config.defaultKd
        self._manager.set_impedance_gains_real_unit_full_state_feedback(K=kp, B=kd)
        self._manager.position = targetPos
        self._manager.velocity = targetVel
        self._manager.torque = feedTor

    def set_torque(self, targetTor: float) -> None:
        """
        Set torque command (non-blocking)
        
        Args:
            targetTor: Target torque (Nm)
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        self._manager.set_current_gains()
        self._manager.torque = targetTor
    
    def stop(self) -> None:
        """Emergency stop - set torque to zero immediately"""
        if not self._isEnabled or self._manager is None:
            return
        
        self._manager.set_current_gains()
        self._manager.torque = 0.0
        self.update()
        logging.info("⚠ Motor STOPPED")
    
    # ==================== Utility ====================
    def zero_position(self) -> None:
        """
        Set current position as zero (encoder reset)
        
        Note: 
            - Motor does NOT move, only redefines current position as 0
            - Takes ~1 second for EEPROM save
            - After zeroing, position command is set to 0 to prevent unwanted movement
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        logging.info("Zeroing encoder...")
        
        self._manager.set_zero_position()
        time.sleep(ZERO_POSITION_SETTLE_TIME)
        self._manager.set_impedance_gains_real_unit_full_state_feedback(
            K=self.config.defaultKp, 
            B=self.config.defaultKd
        )
        self._manager.position = 0.0
        self._manager.torque = 0.0
        
        # Update state
        self.update()
        
        logging.info(f"✓ Encoder zeroed (position: {self._lastPosition:.4f} rad)")
    
    def check_connection(self) -> bool:
        """Check if motor is responding"""
        if not self._isEnabled:
            return False
        try:
            self.update()
            return True
        except:
            return False

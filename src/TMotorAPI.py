"""
TMotor Control API v5.0 - Non-blocking Control
Based on TMotorCANControl by Neurobionics Lab

Changes from v4.3:
- Removed duration parameter from set_* methods (non-blocking design)
- Removed unused variables (stepTimeout, stepTolerance, stepSettlingTime)
- Fixed zero_position() causing unwanted movement
- Added stop() method for emergency stop

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

try:
    from TMotorCANControl.mit_can import TMotorManager_mit_can
except ImportError:
    print("Error: TMotorCANControl not installed")
    print("Run: pip install TMotorCANControl")
    sys.exit(1)


# ==================== Constants ====================
ZERO_POSITION_SETTLE_TIME = 1.0  # 영점 설정 후 대기 시간 (EEPROM 저장)
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

    def __post_init__(self):
        """Validate configuration"""
        if not 0 <= self.motorId <= 127:
            raise ValueError(f"motorId must be 0-127, got {self.motorId}")
        if not CAN_INTERFACE_PATTERN.match(self.canInterface):
            raise ValueError(f"Invalid CAN interface: {self.canInterface}")


# ==================== CAN Interface ====================
class CANInterface:
    """CAN interface setup utility"""
    
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

        if not CAN_INTERFACE_PATTERN.match(_interface):
            raise ValueError(f"Invalid CAN interface: {_interface}")
        
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
    High-level motor control API (Non-blocking design)
    
    Usage:
        with Motor('AK70-10', motorId=1) as motor:
            while running:
                motor.set_position(target)
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
        
        # Setup CAN interface
        if self.config.autoInit:
            CANInterface.setup_interface(config=self.config)
        
        # Initialize TMotorManager
        try:
            self._manager = TMotorManager_mit_can(
                motor_type=self.config.motorType,
                motor_ID=self.config.motorId,
                max_mosfett_temp=int(self.config.maxTemperature)
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
        return self._lastPosition
    
    @property
    def velocity(self) -> float:
        return self._lastVelocity
    
    @property
    def torque(self) -> float:
        return self._lastTorque
    
    @property
    def temperature(self) -> float:
        return self._lastTemperature
    
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
        return self._isEnabled
    
    def get_uptime(self) -> float:
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
        
        return {
            'position': self._lastPosition,
            'velocity': self._lastVelocity,
            'torque': self._lastTorque,
            'temperature': self._lastTemperature
        }
    
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
        
        # 4. 상태 업데이트
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

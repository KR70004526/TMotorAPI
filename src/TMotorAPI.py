"""
TMotor Control API v4.2 - CORRECT TMotorManager_mit_can Signature
Based on TMotorCANControl by Neurobionics Lab

Critical Fixes (v4.2):
- FIXED: TMotorManager_mit_can has NO CAN_bus parameter!
- CORRECT signature: __init__(motor_type, motor_ID, max_mosfett_temp, CSV_file, log_vars)
- CAN interface is set up separately via CANInterface.setup_interface()
- Motor(config=config) works perfectly
- All control methods properly set control mode

Actual TMotorManager_mit_can.__init__:
    def __init__(self, 
                 motor_type='AK80-9', 
                 motor_ID=1, 
                 max_mosfett_temp=50, 
                 CSV_file=None, 
                 log_vars=LOG_VARIABLES)

Author: TMotor Control Team
License: MIT
"""

import os
import sys
import time
import subprocess
import logging
import re
from typing import Optional, Dict, List, Tuple, Union
from dataclasses import dataclass
import numpy as np
import threading

try:
    from TMotorCANControl.mit_can import TMotorManager_mit_can
except ImportError:
    print("Error: TMotorCANControl not installed")
    print("Run: pip install TMotorCANControl")
    sys.exit(1)


# ==================== Constants ====================
CONTROL_LOOP_FREQUENCY = 100  # Hz
CONTROL_LOOP_PERIOD = 1.0 / CONTROL_LOOP_FREQUENCY
STEP_COMMAND_THRESHOLD = 0.02
ZERO_POSITION_SETTLE_TIME = 0.5
CAN_INTERFACE_PATTERN = re.compile(r'^can\d+$')

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# ==================== Configuration ====================
@dataclass
class MotorConfig:
    """
    Motor configuration with camelCase naming
    
    Attributes:
        motorType: Motor model (e.g., 'AK80-64', 'AK80-9', 'AK80-10', 'AK70-10')
        motorId: CAN ID (1-32)
        canInterface: CAN interface name for setup (e.g., 'can0')
        bitrate: CAN bitrate for setup (default: 1000000)
        autoInit: Auto initialize CAN interface
        maxTemperature: Maximum safe MOSFET temperature (°C)
        defaultKp: Default position gain (Nm/rad)
        defaultKd: Default velocity gain (Nm/(rad/s))
        defaultTorqueLimit: Default torque limit (Nm)
    
    Note:
        canInterface is only used for CANInterface.setup_interface()
        TMotorManager_mit_can automatically detects CAN interface
    """
    motorType: str = 'AK70-10'
    motorId: int = 1
    canInterface: str = 'can0'  # Only for setup, not passed to TMotorManager
    bitrate: int = 1000000
    autoInit: bool = True
    maxTemperature: float = 50.0  # Maps to max_mosfett_temp
    
    # Default control gains
    defaultKp: float = 10.0
    defaultKd: float = 0.5
    
    # Step command parameters
    stepTimeout: float = 5.0        # Maximum time for step command
    stepTolerance: float = 0.05     # Position tolerance for "reached"

    def __post_init__(self):
        """Validate configuration"""
        if not 0 <= self.motorId <= 127:
            raise ValueError(f"motorId must be 0-127, got {self.motorId}")
        if not CAN_INTERFACE_PATTERN.match(self.canInterface):
            raise ValueError(f"Invalid CAN interface: {self.canInterface}")


# ==================== CAN Interface ====================
class CANInterface:
    """CAN interface automatic setup"""
    
    @staticmethod
    def setup_interface(canInterface: Optional[str] = None, 
                        bitrate: Optional[int]= None,
                        config: Optional[MotorConfig] = None) -> bool:
        """
        Setup CAN interface using system commands
        
        This is separate from TMotorManager_mit_can initialization.
        TMotorManager automatically detects the CAN interface after setup.
        """

        if config is not None:
            if canInterface is None:
                _interface = config.canInterface
            if bitrate is None:
                _bitrate = config.bitrate
        else:
            if canInterface is None:
                _interface = 'can0'
            if bitrate is None:
                _bitrate = 1000000

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
            # stderr 디코딩해 주면 디버깅 좋음
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
    High-level motor control API
    
    Naming Convention:
    - Variables: camelCase
    - Functions: snake_case
    
    TMotorManager_mit_can Usage:
        manager = TMotorManager_mit_can(
            motor_type='AK80-9',
            motor_ID=1,
            max_mosfett_temp=50  # Only these 3 parameters!
        )
    
    Example:
        # Method 1: Direct parameters
        motor = Motor('AK80-9', motorId=2, autoInit=True)
        
        # Method 2: Config object
        config = MotorConfig(motorType='AK80-9', motorId=2, maxTemperature=80)
        motor = Motor(config=config)
        
        # Method 3: Context manager
        with Motor('AK80-9', motorId=2) as motor:
            motor.track_trajectory(1.57, 2.0)
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
        """
        Initialize Motor object
        
        Args:
            motorType: Motor model
            motorId: CAN ID (1-32)
            canInterface: CAN interface for setup only
            bitrate: CAN bitrate for setup
            autoInit: Auto setup CAN
            maxTemperature: Max MOSFET temperature (°C)
            config: MotorConfig object
            **kwargs: Additional params
        """
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
        self._controlModeSet = False
        
        # Last known state
        self._lastPosition = 0.0
        self._lastVelocity = 0.0
        self._lastTorque = 0.0
        self._lastTemperature = 0.0
        
        # Setup CAN interface (separate from TMotorManager)
        if self.config.autoInit:
            CANInterface.setup_interface(config=self.config)
        
        # Initialize TMotorManager with CORRECT signature
        try:
            self._manager = TMotorManager_mit_can(
                motor_type=self.config.motorType,
                motor_ID=self.config.motorId,
                max_mosfett_temp=int(self.config.maxTemperature)
                # NO CAN_bus parameter!
                # TMotorManager automatically detects CAN interface
            )
            logging.info(f"✓ Motor connected: {self.config.motorType} ID={self.config.motorId}")
            logging.info(f"  Max MOSFET temp: {self.config.maxTemperature}°C")
            logging.info(f"  Power: OFF (call enable() or use 'with')")
        except Exception as e:
            logging.error(f"Failed to connect motor: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def __enter__(self):
        self.enable()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disable()
        return False
    
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
    
    def enable(self) -> None:
        """Enable motor (Power ON)"""
        if self._isEnabled:
            logging.warning("Motor already enabled")
            return
        
        if self._manager is None:
            raise RuntimeError("Motor manager not initialized")
        
        try:
            self._manager.__enter__()
            self._isEnabled = True
            self._powerOnTime = time.time()
            
            time.sleep(0.1)
            self.update()
            
            logging.info("=" * 60)
            logging.info("Motor ENABLED (Powered ON)")
            logging.info("=" * 60)
        except Exception as e:
            logging.error(f"Failed to enable motor: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def disable(self) -> None:
        """Disable motor (Power OFF)"""
        if self._manager and self._isEnabled:
            try:
                uptime = time.time() - self._powerOnTime
                self._manager.__exit__(None, None, None)
                self._isEnabled = False
                self._controlModeSet = False
                
                logging.info("=" * 60)
                logging.info("Motor DISABLED (Powered OFF)")
                logging.info(f"Total powered-on time: {uptime:.2f} seconds")
                logging.info("=" * 60)
            except Exception as e:
                logging.error(f"Error disabling motor: {e}")
    
    def update(self) -> Dict[str, float]:
        """Update motor state"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        try:
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
        except Exception as e:
            logging.debug(f"Update warning: {e}")
            return {
                'position': self._lastPosition,
                'velocity': self._lastVelocity,
                'torque': self._lastTorque,
                'temperature': self._lastTemperature
            }
    
    def is_power_on(self) -> bool:
        return self._isEnabled
    
    def get_uptime(self) -> float:
        if self._isEnabled:
            return time.time() - self._powerOnTime
        return 0.0
    
    def check_connection(self) -> bool:
        if not self._isEnabled:
            return False
        try:
            self.update()
            return True
        except:
            return False
    
    def track_trajectory(self,
                        targetPos: float,
                        duration: float = 0.0,
                        kp: Optional[float] = None,
                        kd: Optional[float] = None,
                        trajectoryType: str = 'minimum_jerk') -> None:
        """
        Trajectory control
        
        Args:
            targetPos: Target position (rad)
            duration: Motion duration (seconds) [2ND POSITIONAL!]
            kp: Position gain
            kd: Velocity gain
            trajectoryType: 'minimum_jerk', 'cubic', 'linear'
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        if kp is None:
            kp = self.config.defaultKp
        if kd is None:
            kd = self.config.defaultKd
        
        try:
            # Set control mode
            self._manager.set_impedance_gains_real_unit(K=kp, B=kd)
            self._controlModeSet = True
            
            if duration <= STEP_COMMAND_THRESHOLD:
                logging.info(f"Step: {self._lastPosition:.3f} → {targetPos:.3f} rad")
                
                startTime = time.time()
                timeout = self.config.stepTimeout
                tolerance = self.config.stepTolerance
                
                # 계속 명령 전송! (FIX!)
                while time.time() - startTime < timeout:
                    self._manager.position = targetPos
                    self.update()
                    
                    # 목표 도달 확인
                    error = abs(self._lastPosition - targetPos)
                    if error < tolerance:
                        elapsed = time.time() - startTime
                        logging.info(f"  ✓ Reached in {elapsed:.2f}s: {self._lastPosition:.3f} rad")
                        logging.info(f"    Error: {error:.4f} rad")
                        return
                    
                    time.sleep(CONTROL_LOOP_PERIOD)
                
                # Timeout
                error = abs(self._lastPosition - targetPos)
                logging.warning(f"  ⚠ Timeout ({timeout}s)!")
                logging.warning(f"    Target: {targetPos:.3f}, Current: {self._lastPosition:.3f}")
                logging.warning(f"    Error: {error:.3f} rad")
                return
            
            # Trajectory
            if trajectoryType == 'minimum_jerk':
                trajFunc = TrajectoryGenerator.minimum_jerk
            elif trajectoryType == 'cubic':
                trajFunc = TrajectoryGenerator.cubic
            elif trajectoryType == 'linear':
                trajFunc = TrajectoryGenerator.linear
            else:
                raise ValueError(f"Unknown trajectory: {trajectoryType}")
            
            startPos = self._lastPosition
            t0 = time.time()
            
            logging.info(f"Traj: {startPos:.3f} → {targetPos:.3f} rad ({duration:.2f}s)")
            
            while True:
                t = time.time() - t0
                if t >= duration:
                    break
                
                p, v = trajFunc(startPos, targetPos, t, duration)
                
                self._manager.position = p
                self.update()
                
                time.sleep(CONTROL_LOOP_PERIOD)
            
            # Final
            self._manager.position = targetPos
            self.update()
            
            logging.info(f"  ✓ Complete: {self._lastPosition:.3f} rad")
            
        except Exception as e:
            logging.error(f"Trajectory failed: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def set_velocity(self,
                    targetVel: float,
                    kd: Optional[float] = None,
                    duration: float = 0.0) -> None:
        """Velocity control"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        if kd is None:
            kd = self.config.defaultKd
        
        try:
            logging.info(f"Velocity: {targetVel:.3f} rad/s")
            
            # Set control mode BEFORE command
            self._manager.set_speed_gains(kd=kd)
            self._controlModeSet = True
            
            if duration <= 0:
                self._manager.velocity = targetVel
                self.update()
            else:
                t0 = time.time()
                while time.time() - t0 < duration:
                    self._manager.velocity = targetVel
                    self.update()
                    time.sleep(CONTROL_LOOP_PERIOD)
                
                self._manager.velocity = 0.0
                self.update()
                logging.info("  ✓ Complete")
                
        except Exception as e:
            logging.error(f"Velocity failed: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def set_torque(self,
                  targetTorque: float,
                  duration: float = 0.0) -> None:
        """Torque control"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        try:
            logging.info(f"Torque: {targetTorque:.3f} Nm")
            
            # Set control mode
            self._manager.set_current_gains()
            self._controlModeSet = True
            
            if duration <= 0:
                self._manager.torque = targetTorque
                self.update()
            else:
                t0 = time.time()
                while time.time() - t0 < duration:
                    self._manager.torque = targetTorque
                    self.update()
                    time.sleep(CONTROL_LOOP_PERIOD)
                
                self._manager.torque = 0.0
                self.update()
                logging.info("  ✓ Complete")
                
        except Exception as e:
            logging.error(f"Torque failed: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def send_command(self,
                    targetPos: float,
                    kp: float,
                    kd: float,
                    feedforwardTorque: float = 0.0) -> None:
        """Low-level impedance control"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        try:
            self._manager.set_impedance_gains_real_unit_full_state_feedback(K=kp, B=kd)
            self._controlModeSet = True
            
            self._manager.position = targetPos
            self._manager.torque = feedforwardTorque
            
            self.update()
            
        except Exception as e:
            logging.error(f"Command failed: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def zero_position(self) -> None:
        """Zero position"""
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")
        
        try:
            logging.info("Zeroing...")
            self._manager.set_zero_position()
            time.sleep(ZERO_POSITION_SETTLE_TIME)
            self.update()
            logging.info(f"  ✓ Zeroed at {self._lastPosition:.3f} rad")
        except Exception as e:
            logging.error(f"Zero failed: {e}")
            raise

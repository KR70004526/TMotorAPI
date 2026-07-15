"""
TMotor Control API v5.1 - Non-blocking Control with Soft Limit + AK driver v3 support

Changes from v5.0:
- Added MotorConfig.driverVersion ("v2" | "v3"); default "v2" (기존 동작 100% 유지)
- Embedded AK driver v3 MIT protocol driver (TMotorManager_mit_can_v3) into this single file
- zero_position() safety gate (영점 실패 시 홀드 스킵 -> 폭주 방지)

driverVersion 을 건드리지 않으면 자동으로 "v2"(클래식 MIT) = 기존과 동일.
v3 프로토콜(CubeMars AK driver v3.x)은 driverVersion="v3" 를 명시했을 때만 활성화된다.

Based on TMotorCANControl by Neurobionics Lab
License: MIT
"""

import os
import sys
import time
import subprocess
import logging
import re
import warnings
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from math import pi
from enum import Enum
import numpy as np
import can

# v2 (classic MIT) 매니저 - driverVersion="v2"(기본) 일 때 사용
try:
    from TMotorCANControl.mit_can import TMotorManager_mit_can
except ImportError:
    print("Error: TMotorCANControl not installed")
    print("Run: pip install TMotorCANControl")
    sys.exit(1)


# ============================================================================
#  AK driver v3.x MIT driver (embedded)  --  driverVersion="v3" 일 때만 사용
#  근거: ak_drive_v3_manual.pdf V3.2.0
#    송신(MIT)   §4.2  : 확장프레임 arb=(8<<8)|id, 순서 Kp·Kd·Pos·Spd·T, 토크=출력 N·m
#    수신(피드백)§4.3.1: 서보형 arb=(0x29<<8)|id, pos int16/10° / spd int16*10 ERPM /
#                        cur int16/100 A / temp int8 / err
#    영점        §4.1.6: Set Origin arb=(5<<8)|id, data=[0|1]   (MIT 0xFE 아님)
#    종료        §4.1.8: Motor Disable arb=(15<<8)|id, DLC=0     (MIT FF..FD 아님)
# ============================================================================
def _v3_mk(v_max, t_max, kt, gear, pole_pairs=21):
    """v3 모터 파라미터. Position±12.56 / Kp 0-500 / Kd 0-5 는 전 모델 공통(매뉴얼 §4.2)."""
    return {'P_min': -12.56, 'P_max': 12.56,
            'V_min': -float(v_max), 'V_max': float(v_max),
            'T_min': -float(t_max), 'T_max': float(t_max),
            'Kp_min': 0.0, 'Kp_max': 500.0, 'Kd_min': 0.0, 'Kd_max': 5.0,
            'Kt': kt, 'GEAR_RATIO': float(gear), 'POLE_PAIRS': pole_pairs}


# 매뉴얼 §4.2 p42 파라미터 표 전체(모델 -> V_max, T_max, Kt, gear).
#   AK10-9 만 하드웨어 실측 검증됨. 나머지는 매뉴얼 공식 스펙값(미검증, 그러나 공식).
#   Position ±12.56 / Kp 0-500 / Kd 0-5 는 전 모델 공통(펌웨어 인코더 풀스케일).
#   POLE_PAIRS 는 속도(rad/s) 표시에만 영향 -> 정밀 필요시 모델별 조정.
MIT_Params_v3 = {
    'ERROR_CODES': {
        0: 'No Error', 1: 'Over temperature', 2: 'Over current',
        3: 'Over voltage', 4: 'Under voltage', 5: 'Encoder fault',
        6: 'MOSFET over-temperature', 7: 'Motor lock-up',
    },
    'AK10-9':   _v3_mk(28,  54,  1.3137, 9),
    'AK60-6':   _v3_mk(60,  12,  0.5994, 6),
    'AK60-39':  _v3_mk(10,  80,  3.4616, 39),
    'AK70-9':   _v3_mk(30,  32,  1.0621, 9),
    'AK80-8':   _v3_mk(38,  32,  1.0569, 8),
    'AK80-9':   _v3_mk(65,  18,  0.5701, 9),
    'AKE60-8':  _v3_mk(40,  15,  0.7382, 8),
    'AKE80-8':  _v3_mk(20,  35,  1.7909, 8),
    'AKE90-8':  _v3_mk(20,  150, 1.8265, 8),
    'AKH70-16': _v3_mk(13,  110, 2.8334, 16),
    'AKH70-48': _v3_mk(5,   280, 8.8123, 48),
}

_V3_CONTROL_MODE_ID = 8      # MIT 송신 Control Mode ID.  §4.2
_V3_FEEDBACK_FUNC_ID = 0x29  # 서보형 피드백 Function ID.  §4.3.1
_V3_SET_ORIGIN_MODE_ID = 5   # Set Origin(영점).          §4.1.6
_V3_DISABLE_MODE_ID = 15     # Motor Disable.             §4.1.8


def _v3_float_to_uint(x, x_min, x_max, bits):
    """실수 -> uint(bits). x_max 에서 오버플로 안 나게 상한 클램프."""
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    scaled = int((x - x_min) * ((1 << bits) / span))
    max_int = (1 << bits) - 1
    return scaled if scaled < max_int else max_int


class _V3State(Enum):
    IDLE = 0
    CURRENT = 2
    FULL_STATE = 3
    SPEED = 4


class _MotorListenerV3(can.Listener):
    """서보형(0x29) 피드백 프레임만 골라 모터 상태 갱신."""
    def __init__(self, motor):
        self.motor = motor

    def on_message_received(self, msg):
        m = self.motor
        arb = msg.arbitration_id
        if m._raw_dump_left > 0:
            is_echo = (arb == m._tx_arb) or (getattr(msg, "is_rx", True) is False)
            tag = "ECHO" if is_echo else "RX  "
            data = " ".join(f"{b:02X}" for b in msg.data)
            ext = "EXT" if msg.is_extended_id else "STD"
            print(f"[RAW {tag}] {ext} id=0x{arb:X} dlc={msg.dlc} data=[{data}]")
            m._raw_dump_left -= 1
        if msg.is_extended_id and (arb >> 8) == _V3_FEEDBACK_FUNC_ID and (arb & 0xFF) == m.ID:
            m._parse_reply(bytes(msg.data))


class TMotorManager_mit_can_v3:
    """CubeMars AK driver v3.x MIT 매니저. TMotorManager_mit_can(v2)과 동일 인터페이스."""
    def __init__(self, motor_type='AK10-9', motor_ID=1, max_mosfett_temp=80,
                 can_interface="gs_usb", can_channel=0,
                 can_bitrate=1_000_000, can_bring_up=False,
                 raw_dump=0):
        if motor_type not in MIT_Params_v3:
            raise KeyError(f"{motor_type} not in MIT_Params_v3 (v3). 매뉴얼 §4.2 값으로 추가 필요.")
        self.type = motor_type
        self.ID = motor_ID
        self.P = MIT_Params_v3[motor_type]
        self.max_temp = max_mosfett_temp
        self._tx_arb = (_V3_CONTROL_MODE_ID << 8) | self.ID
        self._raw_dump_left = raw_dump

        self._cmd = dict(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tor=0.0)
        self._st = dict(pos=0.0, pos_deg=0.0, vel=0.0, erpm=0.0,
                        tor=0.0, cur=0.0, temp=0.0, err=0)
        self._control_state = _V3State.IDLE
        self._last_vel = 0.0
        self._last_t = time.time()
        self._accel = 0.0
        self._entered = False
        self._got_any_reply = False

        if can_bring_up and can_interface in ("socketcan", "socketcan_native") \
                and os.name != "nt" and isinstance(can_channel, str):
            subprocess.run(["sudo", "ip", "link", "set", can_channel, "down"], check=False)
            subprocess.run(["sudo", "ip", "link", "set", can_channel, "up",
                            "type", "can", "bitrate", str(can_bitrate)], check=False)

        self._bus = can.Bus(interface=can_interface, channel=can_channel, bitrate=can_bitrate)
        self._notifier = can.Notifier(self._bus, [_MotorListenerV3(self)])
        print(f"[v3] CAN open: interface={can_interface} channel={can_channel} bitrate={can_bitrate}")

    def __enter__(self):
        print(f"[v3] enter: {self.type} ID={self.ID} tx_arb=0x{self._tx_arb:X}")
        self._enter_mode()
        for _ in range(10):
            self._send_mit(0, 0, 0, 0, 0)
            time.sleep(0.02)
        time.sleep(0.1)
        self._entered = True
        if not self._got_any_reply:
            raise RuntimeError(
                "Device not connected (v3): 응답 프레임 0개. "
                "CAN ID/비트레이트/배선/120Ω 종단 확인.")
        print("[v3] connected")
        return self

    def __exit__(self, *a):
        try:
            # 부드러운 정지: 홀딩 토크 급제거하면 코깅/마찰로 "툭" 튐. 약한 댐핑 후 정식 disable.
            for _ in range(20):
                self._send_mit(0.0, 0.0, 0.0, 0.5, 0.0)
                time.sleep(0.005)
            self._disable()
        finally:
            try:
                self._notifier.stop()
            except Exception:
                pass
            try:
                self._bus.shutdown()
            except Exception:
                pass
            print("[v3] exit (bus closed)")
        return False

    def _enter_mode(self):
        # MIT 클래식 enter(FF..FC). v3 매뉴얼엔 없으나 이 펌웨어에서 활성화 트리거로 동작(실측).
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        try:
            self._bus.send(can.Message(arbitration_id=self._tx_arb, data=data, is_extended_id=True))
        except can.CanError:
            pass

    def _disable(self):
        """v3 정식 종료 = Motor Disable Mode(모드15, arb=0xF01, DLC=0). §4.1.8"""
        arb = (_V3_DISABLE_MODE_ID << 8) | self.ID
        try:
            self._bus.send(can.Message(arbitration_id=arb, data=[], is_extended_id=True))
            print(f"[v3] disable sent: arb=0x{arb:X} (dlc=0)")
        except can.CanError:
            warnings.warn("disable not sent", RuntimeWarning)

    def set_zero_position(self, permanent=False):
        """v3 영점 = 서보 Set Origin Mode(모드5, arb=0x501). §4.1.6. permanent: 임시/영구."""
        mode = 1 if permanent else 0
        arb = (_V3_SET_ORIGIN_MODE_ID << 8) | self.ID
        try:
            self._bus.send(can.Message(arbitration_id=arb, data=bytes([mode]), is_extended_id=True))
            print(f"[v3] set origin sent: arb=0x{arb:X} data=[{mode:02X}] "
                  f"({'permanent' if permanent else 'temporary'})")
        except can.CanError:
            warnings.warn("set origin not sent", RuntimeWarning)

    def _send_mit(self, pos, vel, kp, kd, tor):
        P = self.P
        kp_i = _v3_float_to_uint(kp,  P['Kp_min'], P['Kp_max'], 12)
        kd_i = _v3_float_to_uint(kd,  P['Kd_min'], P['Kd_max'], 12)
        p_i  = _v3_float_to_uint(pos, P['P_min'],  P['P_max'], 16)
        v_i  = _v3_float_to_uint(vel, P['V_min'],  P['V_max'], 12)
        t_i  = _v3_float_to_uint(tor, P['T_min'],  P['T_max'], 12)
        data = bytes([
            (kp_i >> 4) & 0xFF,
            (((kp_i & 0xF) << 4) | ((kd_i >> 8) & 0xF)),
            (kd_i & 0xFF),
            (p_i >> 8) & 0xFF,
            (p_i & 0xFF),
            (v_i >> 4) & 0xFF,
            (((v_i & 0xF) << 4) | ((t_i >> 8) & 0xF)),
            (t_i & 0xFF),
        ])
        try:
            self._bus.send(can.Message(arbitration_id=self._tx_arb, data=data, is_extended_id=True))
        except can.CanError:
            warnings.warn("MIT frame not sent", RuntimeWarning)

    def _send_command(self):
        c, s = self._cmd, self._control_state
        if s == _V3State.FULL_STATE:
            self._send_mit(c['pos'], c['vel'], c['kp'], c['kd'], c['tor'])
        elif s == _V3State.CURRENT:
            self._send_mit(0, 0, 0, 0, c['tor'])
        elif s == _V3State.SPEED:
            self._send_mit(0, c['vel'], 0, c['kd'], 0.0)
        else:  # IDLE
            self._send_mit(0, 0, 0, 0, 0)

    def _parse_reply(self, data):
        if len(data) < 8:
            return
        self._got_any_reply = True
        try:
            P = self.P
            pos_raw = int.from_bytes(data[0:2], 'big', signed=True)
            spd_raw = int.from_bytes(data[2:4], 'big', signed=True)
            cur_raw = int.from_bytes(data[4:6], 'big', signed=True)
            temp = data[6] if data[6] < 128 else data[6] - 256
            err = data[7]
            pos_deg = pos_raw / 10.0
            erpm = spd_raw * 10.0
            out_rpm = erpm / P['POLE_PAIRS'] / P['GEAR_RATIO']
            amps = cur_raw / 100.0
            self._st['pos'] = pos_deg * pi / 180.0
            self._st['pos_deg'] = pos_deg
            self._st['vel'] = out_rpm * 2.0 * pi / 60.0
            self._st['erpm'] = erpm
            self._st['cur'] = amps
            self._st['tor'] = amps * P['Kt']
            self._st['temp'] = temp
            self._st['err'] = err
            now = time.time()
            dt = now - self._last_t
            if dt > 1e-6:
                self._accel = (self._st['vel'] - self._last_vel) / dt
            self._last_vel = self._st['vel']
            self._last_t = now
        except Exception:
            pass

    def update(self):
        if not self._entered:
            raise RuntimeError("update() before enter")
        err = self._st['err']
        if err:
            name = MIT_Params_v3['ERROR_CODES'].get(err, f'code {err}')
            raise RuntimeError(f"Motor fault ({self.type} ID={self.ID}): {name}")
        if self._st['temp'] > self.max_temp:
            raise RuntimeError(f"Over temp {self._st['temp']}C (max {self.max_temp})")
        self._send_command()
        return dict(self._st)

    def get_temperature_celsius(self): return self._st['temp']
    def get_current_qaxis_amps(self): return self._st['cur']
    def get_output_acceleration_radians_per_second_squared(self): return self._accel
    def get_motor_error_code(self): return self._st['err']

    def set_impedance_gains_real_unit_full_state_feedback(self, kp=0, ki=0, K=0.0, B=0.0, ff=0):
        self._cmd['kp'] = K
        self._cmd['kd'] = B
        self._control_state = _V3State.FULL_STATE

    def set_speed_gains(self, kd=1.0):
        self._cmd['kd'] = kd
        self._control_state = _V3State.SPEED

    def set_current_gains(self, **kw):
        self._control_state = _V3State.CURRENT

    @property
    def position(self): return self._st['pos']
    @position.setter
    def position(self, v): self._cmd['pos'] = v

    @property
    def velocity(self): return self._st['vel']
    @velocity.setter
    def velocity(self, v): self._cmd['vel'] = v

    @property
    def torque(self): return self._st['tor']
    @torque.setter
    def torque(self, v): self._cmd['tor'] = v


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
        motorType: Motor model (e.g., 'AK80-64', 'AK80-9', 'AK70-10', 'AK10-9')
        motorId: CAN ID (0-127)
        canInterface: CAN interface name (e.g., 'can0', 'can1')
        bitrate: CAN bitrate (default: 1000000)
        autoInit: Auto initialize CAN interface
        maxTemperature: Maximum safe MOSFET temperature (°C)
        defaultKp: Default position gain (Nm/rad)
        defaultKd: Default velocity gain (Nm/(rad/s))
        driverVersion: "v2"(classic MIT, 기본) | "v3"(AK driver v3.x extended MIT)
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

    # Driver protocol version
    driverVersion: str = "v2"       # "v2"=classic MIT | "v3"=extended MIT (AK driver v3.x)

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

        # driver version 검증
        if self.driverVersion not in ("v2", "v3"):
            raise ValueError(f"driverVersion must be 'v2' or 'v3', got {self.driverVersion}")


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

        # ✅ OS/백엔드 가드: Windows에서는 ip link 자체가 의미 없음
        if os.name == "nt":
            logging.info(f"Skipping CAN bring-up on Windows (interface={_interface})")
            return True

        # ✅ socketcan 이름이 아니면(예: gs_usb) ip link 스킵
        if not CAN_INTERFACE_PATTERN.match(_interface):
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

    driverVersion="v3" 로 config 하면 v3(AK driver v3.x) 매니저가 선택된다.
    건드리지 않으면 v2(클래식 MIT) = 기존 동작.

    Usage:
        # v2 (기본)
        with Motor('AK70-10', motorId=1) as motor:
            while running:
                motor.set_position(target)
                motor.update()
                time.sleep(0.01)

        # v3 (AK driver v3.x)
        cfg = MotorConfig(motorType='AK10-9', driverVersion='v3',
                          canBackend='gs_usb', canChannel=0, autoInit=False)
        with Motor(config=cfg) as motor:
            motor.zero_position()
            motor.set_position(0.3)
            motor.update()
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
        self._manager = None
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
        self._lastControlMode = "IDLE"   # control state name

        # Soft limit parameters (internal only)
        self._softLimitMin: Optional[float] = None
        self._softLimitMax: Optional[float] = None
        self._softZone: float = 0.2
        self._baseKd: float = 2.0
        self._maxKd: float = 20.0

        # Setup CAN interface
        if self.config.autoInit:
            CANInterface.setup_interface(config=self.config)

        # Initialize TMotorManager (driverVersion 으로 v2/v3 매니저 선택)
        try:
            if self.config.driverVersion == "v3":
                _Manager = TMotorManager_mit_can_v3
            else:
                _Manager = TMotorManager_mit_can

            self._manager = _Manager(
                motor_type=self.config.motorType,
                motor_ID=self.config.motorId,
                max_mosfett_temp=int(self.config.maxTemperature),
                can_interface=self.config.canBackend,      # 핵심
                can_channel=self.config.canChannel,        # 핵심
                can_bitrate=self.config.bitrate,
                can_bring_up=False,
            )
            logging.info(f"✓ Motor connected: {self.config.motorType} "
                         f"ID={self.config.motorId} ({self.config.driverVersion})")
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
        """Current control state (IDLE/CURRENT/SPEED/FULL_STATE)"""
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
        Set position command with soft limit and smooth stopping.
        Damping automatically increases near limits for smooth deceleration.
        """
        min_pos = min_pos if min_pos is not None else self._softLimitMin
        max_pos = max_pos if max_pos is not None else self._softLimitMax
        soft_zone = soft_zone if soft_zone is not None else self._softZone
        base_kd = base_kd if base_kd is not None else self._baseKd
        max_kd = max_kd if max_kd is not None else self._maxKd
        kp = kp if kp is not None else self.config.defaultKp

        if min_pos is None or max_pos is None:
            raise ValueError(
                "Soft limits not configured. "
                "Call set_soft_limit() first or specify min_pos/max_pos."
            )

        safe_pos = max(min_pos, min(max_pos, targetPos))
        current = self.position
        kd = base_kd

        if current > max_pos - soft_zone:
            progress = (current - (max_pos - soft_zone)) / soft_zone
            kd = base_kd + (max_kd - base_kd) * min(progress, 1.0)
        elif current < min_pos + soft_zone:
            progress = ((min_pos + soft_zone) - current) / soft_zone
            kd = base_kd + (max_kd - base_kd) * min(progress, 1.0)

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
        Set current position as zero (encoder reset).

        Note:
            - v2: 클래식 MIT zero / v3: 서보 Set Origin (매니저가 알아서 처리)
            - 영점 후 안전게이트: 위치가 0 근처가 아니면 홀드 스킵(폭주 방지)
        """
        if not self._isEnabled or self._manager is None:
            raise RuntimeError("Motor not enabled")

        logging.info("Zeroing encoder...")

        self._manager.set_zero_position()
        time.sleep(ZERO_POSITION_SETTLE_TIME)

        # Refresh state (control state still IDLE -> sends zero command, safe)
        self.update()

        # Safety gate: 영점 후 위치가 0 근처가 아니면 홀드 안 함 (범위 밖 절대명령 -> 폭주 방지)
        if abs(self._manager.position) > 0.5:
            logging.error(
                f"Zeroing may have failed (pos={self._manager.position:.3f} rad). "
                f"Skipping position hold to avoid runaway."
            )
            return

        self._manager.set_impedance_gains_real_unit_full_state_feedback(
            K=self.config.defaultKp,
            B=self.config.defaultKd
        )
        self._manager.position = 0.0
        self._manager.torque = 0.0
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

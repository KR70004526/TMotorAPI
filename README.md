# TMotorAPI v5.0

A high-level Python library for controlling AK-series T-Motors using the MIT CAN protocol.  
Based on [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) by Neurobionics Lab.

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-5.0-green.svg)](https://github.com/KR70004526/TMotorAPI)

---

## What's New in v5.0

v4.3 대비 주요 변경사항:

- `set_position()`, `set_velocity()`, `set_torque()`에서 `duration` 파라미터 제거 (Non-blocking 설계)
- `stepTimeout`, `stepTolerance`, `stepSettlingTime` 등 미사용 변수 제거
- `zero_position()` 호출 시 의도치 않은 모터 움직임 버그 수정
- `stop()` 긴급 정지 메서드 추가
- Soft Limit 기능 추가 (`set_soft_limit()`, `set_position_smooth()`)

---

## Installation

```bash
git clone https://github.com/KR70004526/TMotorAPI.git
cd TMotorAPI
python -m pip install -e .
```

---

## Quick Start

```python
from TMotorAPI import Motor
import time
import signal

running = True
signal.signal(signal.SIGINT, lambda s, f: globals().update(running=False))

with Motor('AK70-10', motorId=1, autoInit=True) as motor:
    while running:
        motor.set_position(1.57)
        state = motor.update()
        print(f"Position: {state['position']:.3f} rad, "
              f"Velocity: {state['velocity']:.3f} rad/s, "
              f"Torque: {state['torque']:.3f} Nm, "
              f"Temperature: {state['temperature']:.1f} °C")
        time.sleep(0.01)
```

---

## Classes

### MotorConfig

모터 설정을 담는 dataclass.

```python
from TMotorAPI import MotorConfig

config = MotorConfig(
    motorType='AK70-10',       # 모터 모델 (기본값: 'AK70-10')
    motorId=1,                 # CAN ID, 0-127 (기본값: 1)
    canInterface='can0',       # CAN 인터페이스 이름 (기본값: 'can0')
    bitrate=1000000,           # CAN bitrate (기본값: 1000000)
    autoInit=True,             # CAN 인터페이스 자동 초기화 (기본값: True)
    maxTemperature=50.0,       # 최대 MOSFET 온도, °C (기본값: 50.0)
    defaultKp=10.0,            # 기본 위치 게인, Nm/rad (기본값: 10.0)
    defaultKd=0.5,             # 기본 속도 게인, Nm/(rad/s) (기본값: 0.5)
    canBackend='socketcan',    # CAN 백엔드 (기본값: 'socketcan')
    canChannel='can0',         # CAN 채널 (기본값: 'can0')
)
```

**`canBackend` — CAN 통신 백엔드 선택**

| 값 | OS | 설명 |
|---|---|---|
| `'socketcan'` | Linux | Linux 커널 SocketCAN 사용. `canChannel`은 `'can0'`, `'can1'` 등 문자열 |
| `'socketcan_native'` | Linux | SocketCAN native 모드. `canChannel` 형식은 `'socketcan'`과 동일 |
| `'gs_usb'` | Windows / Linux | USB-CAN 어댑터(CANable, canable pro 등) 사용. `canChannel`은 정수(`0`, `1` 등) |

Linux 환경에서는 일반적으로 `'socketcan'`을 사용하며, Windows 환경에서는 `'gs_usb'` 백엔드를 사용해야 한다.  
`gs_usb` 백엔드를 사용할 경우 `python-can`의 gs_usb 드라이버가 필요하며, `canChannel`은 USB 디바이스 인덱스(정수)를 지정한다.

```python
# Linux 예시
config = MotorConfig(canBackend='socketcan', canChannel='can0')

# Windows 예시 (USB-CAN 어댑터)
config = MotorConfig(canBackend='gs_usb', canChannel=0)
```

**`canInterface` — Linux SocketCAN 인터페이스 이름**

`canInterface`는 `CANInterface.setup_interface()`에서 Linux의 `ip link set` 명령을 통해 CAN 인터페이스를 초기화할 때 사용된다.  
이 파라미터는 **Linux SocketCAN 환경에서만 의미가 있으며**, Windows나 `gs_usb` 백엔드 사용 시에는 `ip link` 명령이 실행되지 않으므로 무시된다.  
값은 `'can0'`, `'can1'` 등 `can\d+` 패턴의 문자열이어야 하며, 패턴에 맞지 않는 이름은 CAN bring-up이 건너뛰어진다.

**Validation (`__post_init__`)**:
- `motorId`는 0~127 범위여야 함
- `canBackend`가 `'socketcan'` 또는 `'socketcan_native'`이면 `canChannel`은 `'can0'` 형식의 문자열이어야 함
- `canBackend`가 `'gs_usb'`이면 `canChannel`은 정수여야 함
- 지원하지 않는 `canBackend` 값은 `ValueError` 발생

---

### CANInterface

CAN 인터페이스를 시스템 명령으로 설정하는 정적 클래스.

```python
from TMotorAPI import CANInterface

CANInterface.setup_interface(canInterface='can0', bitrate=1000000)
# 또는
CANInterface.setup_interface(config=config)
```

**동작 방식**:
- Linux에서 `sudo ip link set` 명령을 통해 CAN 인터페이스를 down → bitrate 설정 → up 순서로 실행
- Windows(`os.name == "nt"`)에서는 CAN bring-up을 건너뛰고 `True` 반환
- `canInterface` 이름이 `can\d+` 패턴이 아닌 경우에도 건너뛰고 `True` 반환
- 실패 시 `False` 반환

---

### TrajectoryGenerator

궤적 생성 유틸리티 클래스. 모든 메서드는 `@staticmethod`이며 `(position, velocity)` 튜플을 반환한다.

```python
from TMotorAPI import TrajectoryGenerator
```

**`minimum_jerk(startPos, endPos, currentTime, totalDuration)`**  
5차 다항식 기반 최소 저크 궤적. `currentTime >= totalDuration`이면 `(endPos, 0.0)` 반환.

```python
pos, vel = TrajectoryGenerator.minimum_jerk(0.0, 1.57, 0.5, 2.0)
```

**`cubic(startPos, endPos, currentTime, totalDuration)`**  
3차 다항식 궤적. `currentTime >= totalDuration`이면 `(endPos, 0.0)` 반환.

```python
pos, vel = TrajectoryGenerator.cubic(0.0, 1.57, 0.5, 2.0)
```

**`linear(startPos, endPos, currentTime, totalDuration)`**  
선형 보간. `currentTime >= totalDuration`이면 `(endPos, 0.0)` 반환.

```python
pos, vel = TrajectoryGenerator.linear(0.0, 1.57, 0.5, 2.0)
```

---

### Motor

고수준 모터 제어 API. Non-blocking 설계.

#### 생성 방법

```python
# 방법 1: 직접 파라미터
motor = Motor('AK70-10', motorId=1, autoInit=True)

# 방법 2: MotorConfig 객체
config = MotorConfig(motorType='AK70-10', motorId=1)
motor = Motor(config=config)

# 방법 3: 혼합 (kwargs도 가능)
motor = Motor(motorType='AK70-10', motorId=1, maxTemperature=60.0)
```

생성 시 `autoInit=True`이면 `CANInterface.setup_interface()`가 자동 호출된다.  
내부적으로 `TMotorManager_mit_can`을 생성하며, `canBackend`와 `canChannel`을 전달한다.

#### Context Manager

```python
with Motor('AK70-10', motorId=1) as motor:
    # __enter__() → enable() 호출 (Power ON)
    motor.set_position(1.57)
    motor.update()
# __exit__() → disable() 호출 (Power OFF)
```

#### Properties (읽기 전용)

마지막 `update()` 호출 시 갱신된 캐시 값을 반환한다.

| Property | Type | Description |
|----------|------|-------------|
| `position` | `float` | 현재 위치 (rad) |
| `velocity` | `float` | 현재 속도 (rad/s) |
| `torque` | `float` | 현재 토크 (Nm) |
| `temperature` | `float` | 현재 MOSFET 온도 (°C) |

#### Power Control

**`enable()`** — 모터 Power ON. `__enter__`에서 호출된다. 이미 활성화 상태면 경고 로그만 출력. 활성화 후 0.1초 대기 후 `update()` 1회 실행.

**`disable()`** — 모터 Power OFF. `__exit__`에서 호출된다. 가동 시간(uptime)을 로그에 기록.

**`is_power_on()`** — `bool` 반환. 모터 활성화 여부.

**`get_uptime()`** — `float` 반환. `enable()` 이후 경과 시간(초). 비활성화 상태면 `0.0`.

#### State Update

**`update()`** — CAN 통신으로 명령 전송 및 상태 수신. `Dict[str, float]` 반환:

```python
state = motor.update()
# {'position': float, 'velocity': float, 'torque': float, 'temperature': float}
```

모터가 활성화되지 않았으면 `RuntimeError` 발생.

#### Control Commands

모든 제어 명령은 **non-blocking**이며, `update()`를 호출해야 실제 CAN 통신이 수행된다.  
모터가 활성화되지 않았으면 `RuntimeError` 발생.

**`set_position(targetPos, kp=None, kd=None, feedTor=0.0)`**  
위치 제어 명령. 내부적으로 `set_impedance_gains_real_unit_full_state_feedback(K=kp, B=kd)`를 호출한다.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `targetPos` | `float` | (필수) | 목표 위치 (rad) |
| `kp` | `float` or `None` | `config.defaultKp` | 위치 게인 (Nm/rad) |
| `kd` | `float` or `None` | `config.defaultKd` | 속도 게인 (Nm/(rad/s)) |
| `feedTor` | `float` | `0.0` | 피드포워드 토크 (Nm) |

**`set_velocity(targetVel, kd=None)`**  
속도 제어 명령. 내부적으로 `set_speed_gains(kd=kd)`를 호출한다.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `targetVel` | `float` | (필수) | 목표 속도 (rad/s) |
| `kd` | `float` or `None` | `config.defaultKd` | 속도 게인 |

**`set_torque(targetTor)`**  
토크 제어 명령. 내부적으로 `set_current_gains()`를 호출한다.

| Parameter | Type | Description |
|-----------|------|-------------|
| `targetTor` | `float` | 목표 토크 (Nm) |

**`stop()`**  
긴급 정지. `set_current_gains()` 호출 후 토크를 0으로 설정하고 `update()`까지 수행한다. 모터가 비활성화 상태면 아무 동작도 하지 않는다.

#### Soft Limit

소프트 리밋은 위치 한계 근처에서 댐핑(kd)을 자동으로 증가시켜 부드럽게 감속하는 기능이다.

**`set_soft_limit(min_pos, max_pos, soft_zone=0.2, base_kd=2.0, max_kd=20.0)`**  
소프트 리밋 범위를 설정한다.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_pos` | `float` | (필수) | 최소 위치 한계 (rad) |
| `max_pos` | `float` | (필수) | 최대 위치 한계 (rad) |
| `soft_zone` | `float` | `0.2` | 댐핑 증가 구간 (rad) |
| `base_kd` | `float` | `2.0` | 일반 구간 기본 댐핑 |
| `max_kd` | `float` | `20.0` | 한계 지점 최대 댐핑 |

**`set_position_smooth(targetPos, kp=None, feedTor=0.0, min_pos=None, max_pos=None, soft_zone=None, base_kd=None, max_kd=None)`**  
소프트 리밋을 적용한 위치 제어 명령.

동작 방식:
1. 목표 위치를 `[min_pos, max_pos]` 범위로 클램핑
2. 현재 위치가 `max_pos - soft_zone` 이상이면 상한 접근 → kd 증가
3. 현재 위치가 `min_pos + soft_zone` 이하이면 하한 접근 → kd 증가
4. 조정된 kd로 `set_position()` 호출

소프트 리밋이 설정되지 않은 상태에서 `min_pos`/`max_pos`를 명시하지 않으면 `ValueError` 발생.

사용 방법:

```python
# 방법 1: 미리 설정
motor.set_soft_limit(-1.57, 1.57)
while running:
    motor.set_position_smooth(target)
    motor.update()
    time.sleep(0.01)

# 방법 2: 호출 시마다 지정
while running:
    motor.set_position_smooth(target, min_pos=-1.57, max_pos=1.57)
    motor.update()
    time.sleep(0.01)
```

#### Utility

**`zero_position()`**  
현재 위치를 0으로 재설정 (엔코더 리셋). 모터가 움직이지 않으며, EEPROM 저장을 위해 약 1초 대기한다. 리셋 후 위치 명령을 0으로 설정하여 의도치 않은 움직임을 방지한다.

```python
motor.zero_position()
# 현재 위치가 0.0 rad로 재정의됨
```

**`check_connection()`**  
`update()`를 시도하여 모터 응답 여부를 `bool`로 반환. 비활성화 상태면 `False`.

---

## Non-Blocking Design Pattern

v5.0의 모든 제어 명령은 즉시 반환되며, 사용자가 `update()` 호출로 CAN 통신 타이밍을 직접 제어한다.

```python
while running:
    # 1. 명령 설정 (non-blocking)
    motor.set_position(target)
    
    # 2. CAN 통신 (명령 전송 + 상태 수신)
    motor.update()
    
    # 3. 상태 확인
    pos = motor.position
    vel = motor.velocity
    
    # 4. 루프 주기 제어
    time.sleep(0.01)  # 100 Hz
```

---

## Examples

### Position Control with Feedforward Torque

```python
import numpy as np

mass, g, length = 2.0, 9.81, 0.3

def gravity_torque(angle):
    return mass * g * length * np.cos(angle)

with Motor('AK70-10', motorId=1) as motor:
    target = 1.57
    while running:
        motor.set_position(target, kp=15.0, kd=2.0,
                           feedTor=gravity_torque(target))
        motor.update()
        time.sleep(0.01)
```

### Velocity Control

```python
with Motor('AK70-10', motorId=1) as motor:
    while running:
        motor.set_velocity(3.0, kd=5.0)
        motor.update()
        time.sleep(0.01)
```

### Torque Control with Temperature Monitoring

```python
with Motor('AK70-10', motorId=1, maxTemperature=45.0) as motor:
    while running:
        motor.set_torque(2.5)
        motor.update()
        
        if motor.temperature > 45.0:
            motor.stop()
            break
        
        time.sleep(0.01)
```

### Soft Limit Position Control

```python
with Motor('AK70-10', motorId=1) as motor:
    motor.set_soft_limit(-1.57, 1.57, soft_zone=0.2, base_kd=2.0, max_kd=20.0)
    
    while running:
        motor.set_position_smooth(target)
        motor.update()
        time.sleep(0.01)
```

### Trajectory Following

```python
from TMotorAPI import Motor, TrajectoryGenerator
import time

with Motor('AK70-10', motorId=1) as motor:
    start_pos, end_pos, duration = 0.0, 1.57, 2.0
    start_time = time.time()
    
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        
        target_pos, target_vel = TrajectoryGenerator.minimum_jerk(
            start_pos, end_pos, t, duration)
        
        motor.set_position(target_pos, kp=10.0, kd=2.0)
        motor.update()
        time.sleep(0.01)
```

### Multi-Motor Synchronization

```python
with Motor('AK70-10', motorId=1, canInterface='can0') as motor1, \
     Motor('AK70-10', motorId=2, canInterface='can0') as motor2:
    
    while running:
        motor1.set_position(angle1)
        motor2.set_position(angle2)
        motor1.update()
        motor2.update()
        time.sleep(0.01)
```

### Zeroing and Emergency Stop

```python
with Motor('AK70-10', motorId=1) as motor:
    # 현재 위치를 0으로 설정
    motor.zero_position()
    
    while running:
        motor.set_position(1.57)
        motor.update()
        
        # 긴급 상황 시
        if emergency:
            motor.stop()
            break
        
        time.sleep(0.01)
```

---

## Architecture

```
User Application
       ↓
   TMotorAPI v5.0 (Non-blocking wrapper)
       ↓
   TMotorCANControl (Low-level MIT CAN driver)
       ↓
   SocketCAN (Linux) / gs_usb (Windows)
       ↓
   CAN Hardware → T-Motor (AK Series)
```

---

## Dependencies

- [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) (`TMotorManager_mit_can`)
- `numpy`
- `can-utils` (Linux, CAN 인터페이스 설정 시)

---

## License

MIT License

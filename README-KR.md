# TMotor Control API v2.0

AK 시리즈 T모터 전문 제어 라이브러리 (MIT CAN 프로토콜)

[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 🌟 주요 기능

- **4가지 제어 모드**: Trajectory, Velocity, Torque, Impedance 제어
- **간단한 API**: 사용하기 쉬운 고수준 인터페이스
- **Context Manager**: 자동 전원 관리
- **다중 모터 지원**: 여러 모터의 동기화 제어
- **전원 모니터링**: 모터 가동 시간 및 연결 상태 추적
- **타입 힌트**: 완전한 타입 주석으로 IDE 지원 강화
- **상세한 로깅**: 자세한 작동 로그

## 📋 목차

- [설치](#설치)
- [빠른 시작](#빠른-시작)
- [제어 모드](#제어-모드)
- [API 레퍼런스](#api-레퍼런스)
- [예제](#예제)
- [FAQ](#faq)
- [문제 해결](#문제-해결)

## 🚀 설치

### 사전 요구사항

```bash
# TMotorCANControl 라이브러리 설치
pip install TMotorCANControl

# CAN 인터페이스 설정 (한 번만)
sudo apt-get install can-utils
```

### Sudo 권한 설정 (권장)

```bash
sudo visudo
# 다음 줄 추가:
your_username ALL=(ALL) NOPASSWD: /sbin/ip
```

### 라이브러리 설치

```bash
# tmotor_control_final.py를 프로젝트에 복사
cp tmotor_control_final.py your_project/
```

## ⚡ 빠른 시작

### 기본 사용법

```python
from tmotor_control_final import Motor

# Context manager를 사용한 모터 생성 및 사용 (권장)
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 이 안에서는 모터 전원이 켜져있습니다
    motor.track_trajectory(1.57)  # 1.57 rad로 이동
    # 빠져나오면 자동으로 전원이 꺼집니다
```

### 전원 관리 이해하기

**중요**: 모터 전원은 2단계로 작동합니다:

1. **객체 생성** (연결, 전원 OFF)
```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)
# TMotorManager 객체 생성됨
# CAN 연결 확립됨
# 모터 전원은 아직 OFF (모터가 움직이지 않음)
```

2. **Enable/With 블록** (전원 ON)
```python
with motor as m:  # __enter__() 호출 → enable() → 전원 ON
    # 이제 모터 전원이 켜짐
    m.track_trajectory(1.57)
    # with 블록 내내 전원이 유지됨
# __exit__() 호출 → disable() → 전원 OFF
```

### 수동 제어

```python
motor = Motor('AK80-64', motor_id=2, auto_init=True)

motor.enable()  # 전원 ON - 이제 모터가 움직일 수 있음
print(f"전원 상태: {motor.is_power_on()}")  # True

motor.track_trajectory(1.57)
motor.set_velocity(2.0)

motor.disable()  # 전원 OFF
print(f"전원 상태: {motor.is_power_on()}")  # False
```

## 🎯 제어 모드

### 개요

| 모드 | 함수 | 사용 목적 |
|------|------|----------|
| **1. Trajectory** | `track_trajectory()` | 위치 제어, 부드러운 동작 |
| **2. Velocity** | `set_velocity()` | 일정 속도 회전 |
| **3. Torque** | `set_torque()` | 힘 제어, 중력 보상 |
| **4. Impedance** | `send_command()` | 저수준 완전 제어 (전문가용) |

### 모드 1: Trajectory Control (궤적 제어)

자동 궤적 생성을 통한 위치 제어

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 즉시 이동 (step position)
    motor.track_trajectory(1.57)
    
    # 부드러운 궤적 (2초)
    motor.track_trajectory(1.57, duration=2.0)
    
    # 강성 조절
    motor.track_trajectory(1.57, kp=50, kd=2.0)  # 단단하게
    motor.track_trajectory(1.57, kp=5, kd=0.3)   # 유연하게
```

**파라미터:**
- `position`: 목표 위치 (rad)
- `kp`: 위치 게인 (Nm/rad) - 높을수록 단단함
- `kd`: 속도 게인 (Nm/(rad/s)) - 높을수록 댐핑 강함
- `duration`: 이동 시간 (s) - 0이면 즉시, >0이면 궤적
- `trajectory_type`: 'minimum_jerk', 'cubic', 'linear', 'trapezoidal'

### 모드 2: Velocity Control (속도 제어)

일정 속도 회전 (피드포워드 토크 없음)

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 2 rad/s로 회전
    motor.set_velocity(2.0)
    time.sleep(10)
    
    # 정지
    motor.set_velocity(0.0)
```

**속도 + FF 토크가 필요한 경우:**
```python
motor.send_command(
    position=motor.position,
    velocity=2.0,
    kp=0, kd=5.0,
    torque=gravity_compensation
)
```

### 모드 3: Torque Control (토크 제어)

위치/속도 피드백 없는 순수 토크 제어

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 중력 보상
    motor.set_torque(3.5)
    
    # 토크 제거 (자유 움직임)
    motor.set_torque(0.0)
```

### 모드 4: Impedance Control (임피던스 제어 - 저수준)

모든 파라미터의 완전 수동 제어 (전문가 모드)

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 속도 + FF 토크
    motor.send_command(
        position=motor.position,
        velocity=2.0,
        kp=0, kd=5.0,
        torque=gravity_comp
    )
    
    # MPC 제어기 통합
    for _ in range(1000):
        p, v, kp, kd, tau = mpc.compute()
        motor.send_command(p, v, kp, kd, tau)
        time.sleep(0.01)
```

## 📚 API 레퍼런스

### Motor 클래스

#### 초기화

```python
Motor(motor_type='AK80-64', motor_id=1, can_interface='can0', 
      auto_init=False, config=None)
```

**파라미터:**
- `motor_type`: 모터 모델 ('AK80-64', 'AK80-9' 등)
- `motor_id`: CAN ID (1-32)
- `can_interface`: CAN 인터페이스 이름 ('can0', 'can1' 등)
- `auto_init`: True면 initialize() 자동 호출
- `config`: 고급 설정용 MotorConfig 객체

#### 전원 관리

```python
motor.enable()   # 전원 ON
motor.disable()  # 전원 OFF
motor.is_power_on()  # 전원 상태 확인
motor.get_uptime()   # 전원 켜진 후 경과 시간
```

#### 제어 메서드

```python
# 궤적 제어
motor.track_trajectory(position, kp=10, kd=0.5, duration=0.0, 
                      trajectory_type='minimum_jerk')

# 속도 제어
motor.set_velocity(velocity, kd=5.0)

# 토크 제어
motor.set_torque(torque)

# 저수준 제어
motor.send_command(position, velocity, kp, kd, torque=0.0)
```

#### 유틸리티 메서드

```python
motor.update()          # 모터 상태 읽기
motor.zero_position()   # 현재 위치를 0으로 설정
motor.check_connection()  # 모터 응답 확인
```

#### 속성

```python
motor.position       # 현재 위치 (rad)
motor.velocity       # 현재 속도 (rad/s)
motor.torque         # 현재 토크 (Nm)
motor.temperature    # 현재 온도 (°C)
motor.is_enabled     # 전원 상태
```

### MotorGroup 클래스

```python
# 모터 그룹 생성
motors = MotorGroup([
    ('AK80-64', 1),
    ('AK80-64', 2),
    ('AK80-9', 3)
])

# Context manager 사용
with motors:
    # 동기화된 동작
    motors.track_all_trajectory([1.57, 0.0, -1.57], duration=2.0)
    
    # 개별 제어
    motors[0].set_velocity(2.0)
    motors[1].track_trajectory(1.0)
```

## 💡 예제

### 예제 1: 기본 위치 제어

```python
from tmotor_control_final import Motor

with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 1.57 rad로 이동
    motor.track_trajectory(1.57)
    time.sleep(2)
    
    # 0으로 복귀
    motor.track_trajectory(0.0)
```

### 예제 2: 부드러운 궤적

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 5초에 걸쳐 부드럽게 이동
    motor.track_trajectory(3.14, duration=5.0)
```

### 예제 3: 속도 제어

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    # 10초 동안 회전
    motor.set_velocity(2.0)
    time.sleep(10)
    motor.set_velocity(0.0)
```

### 예제 4: 다중 모터

```python
from tmotor_control_final import MotorGroup

motors = MotorGroup([
    ('AK80-64', 1),
    ('AK80-64', 2),
    ('AK80-9', 3)
])

with motors:
    # 모든 모터 동시 이동
    motors.track_all_trajectory([1.57, 0.0, -1.57], duration=2.0)
    
    # 위치 확인
    print(f"위치: {motors.get_positions()}")
```

### 예제 5: 전원 상태 모니터링

```python
with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    print(f"전원 ON: {motor.is_power_on()}")  # True
    print(f"가동 시간: {motor.get_uptime():.2f}s")
    
    motor.track_trajectory(1.57)
    
    print(f"여전히 ON: {motor.is_power_on()}")  # True
    print(f"가동 시간: {motor.get_uptime():.2f}s")

# with 블록 후
print(f"전원 OFF: {motor.is_power_on()}")  # False
```

### 예제 6: MPC 통합

```python
from your_mpc_library import MPCController

mpc = MPCController()

with Motor('AK80-64', motor_id=2, auto_init=True) as motor:
    for _ in range(1000):
        # 최적 제어 계산
        p, v, kp, kd, tau = mpc.compute(
            current_state=motor.position,
            target_state=1.57
        )
        
        # 제어 적용
        motor.send_command(p, v, kp, kd, tau)
        
        time.sleep(0.01)
```

## ❓ FAQ

### Q1: 모터 전원은 언제 실제로 켜지나요?

**A:** `enable()`이 호출되거나 `with` 블록에 진입할 때 켜집니다:

```python
motor = Motor(..., auto_init=True)  # 연결됨, 하지만 전원 OFF
motor.enable()  # 이제 전원이 ON
```

```python
with Motor(...) as motor:  # 여기서 전원 ON
    pass  # 여기서 전원 OFF
```

### Q2: 전원이 얼마나 오래 유지되나요?

**A:** 전원은 다음까지 유지됩니다:
1. `disable()`이 호출되거나
2. `with` 블록을 빠져나갈 때까지

```python
with motor:
    # 이 블록 전체에서 전원 ON
    motor.track_trajectory(1.57)
    time.sleep(10)
    motor.set_velocity(2.0)
    time.sleep(20)
    # 여기서도 여전히 전원 ON
# 빠져나올 때 전원 OFF
```

### Q3: `auto_init=True`와 `auto_init=False`의 차이는?

**A:**
- `auto_init=True`: TMotorManager 객체를 즉시 생성 (모터 연결, 전원 OFF)
- `auto_init=False`: 첫 `enable()` 호출 시 TMotorManager 생성

둘 다 처음에는 전원이 OFF입니다. 전원은 `enable()`이나 `with` 블록에서만 켜집니다.

### Q4: `enable()`과 `with`를 함께 사용할 수 있나요?

**A:** 권장하지 않습니다! `enable()`이 두 번 호출됩니다:

```python
# ❌ 이렇게 하지 마세요
motor = Motor(...)
motor.enable()  # 첫 번째 enable
with motor:     # 두 번째 enable (나쁨!)
    pass
```

**✅ 하나만 선택하세요:**
```python
# 방법 1: 수동
motor.enable()
motor.track_trajectory(1.57)
motor.disable()

# 방법 2: Context manager (권장)
with motor:
    motor.track_trajectory(1.57)
```

### Q5: 모터가 여전히 전원이 켜져있는지 어떻게 확인하나요?

**A:** 다음 메서드를 사용하세요:
```python
motor.is_power_on()      # 전원이 켜져있으면 True
motor.get_uptime()       # 전원 켜진 후 경과 시간 (초)
motor.check_connection() # 응답하는지 확인
```

## 🔧 문제 해결

### 모터가 응답하지 않음

```python
# 연결 확인
if not motor.check_connection():
    print("모터가 응답하지 않습니다!")
    motor.disable()
    time.sleep(1)
    motor.enable()
```

### 과도한 진동

```python
# 댐핑(kd) 증가
motor.track_trajectory(1.57, kp=10, kd=2.0)  # kd 높임
```

### 너무 유연함

```python
# 강성(kp) 증가
motor.track_trajectory(1.57, kp=50, kd=2.0)  # kp 높임
```

### CAN 인터페이스를 찾을 수 없음

```bash
# 인터페이스 확인
ip link show can0

# 수동으로 활성화
sudo ip link set can0 up type can bitrate 1000000
```

### 권한 거부됨

```bash
# sudo 권한 설정 (설치 섹션 참조)
sudo visudo
# 추가: your_username ALL=(ALL) NOPASSWD: /sbin/ip
```

## 📝 게인 튜닝 가이드

| 목적 | Kp | Kd | 특징 |
|------|----|----|------|
| 정밀 제어 | 50 | 2.0 | 단단함, 정확함 |
| 일반 제어 | 10 | 0.5 | 기본값, 균형 잡힘 |
| 안전한 상호작용 | 5 | 0.3 | 유연함, 부드러움 |
| 속도 제어 | 0 | 5.0 | 위치 제어 OFF |
| 토크 제어 | 0 | 0 | 모든 피드백 OFF |

**튜닝 팁:**
- 진동 → Kd 증가
- 너무 유연 → Kp 증가
- 오버슈트 → Kd 증가
- 느린 응답 → Kp 증가

## 🎓 제어 이론

모든 제어 모드는 같은 임피던스 제어 방정식을 사용합니다:

```
τ = Kp × (pos_target - pos_actual) + 
    Kd × (vel_target - vel_actual) + 
    τ_feedforward
```

각 모드는 이 5개 파라미터의 조합일 뿐입니다:

| 모드 | pos_target | vel_target | Kp | Kd | τ_ff |
|------|------------|------------|----|----|------|
| Position | target | 0 | ✓ | ✓ | 0 |
| Velocity | current | target | 0 | ✓ | 0 |
| Torque | 0 | 0 | 0 | 0 | ✓ |
| Impedance | ✓ | ✓ | ✓ | ✓ | ✓ |

## 📄 라이선스

MIT License - 자세한 내용은 LICENSE 파일 참조

## 🤝 기여

기여를 환영합니다! Pull Request를 자유롭게 제출해주세요.

## 📧 연락처

이슈와 질문은 GitHub에서 이슈를 열어주세요.

## 🙏 감사의 말

Neurobionics Lab의 [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl)을 기반으로 제작되었습니다.

---

**즐거운 제어 되세요! 🚀**

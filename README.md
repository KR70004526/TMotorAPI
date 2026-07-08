# TMotorAPI

CubeMars T-Motor(AK 시리즈)용 고수준 제어 API. [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl)(Neurobionics Lab) 위에 얹은 얇은 래퍼로, **논블로킹 제어**와 **소프트 리밋**을 제공한다.

- MIT 모드 기반 위치 / 속도 / 토크 / 풀스테이트 제어
- 논블로킹 설계 — 명령은 즉시 반환, 실제 통신은 `update()`에서 처리
- 소프트 리밋 — 한계 근처에서 댐핑을 자동으로 키워 부드럽게 정지
- Linux(socketcan) / Windows(gs_usb) 백엔드 지원

---

## 설치

```bash
pip install git+https://github.com/KR70004526/TMotorAPI.git
```

- Python >= 3.8
- 의존성: `python-can>=4.6`, `TMotorCANControl`(gs_usb 지원 포크가 자동 설치됨)
- Windows에서 gs_usb를 쓰려면 CANable 등 gs_usb 호환 어댑터 필요

---

## 빠른 시작

```python
from TMotorAPI import Motor, MotorConfig
import time, math

cfg = MotorConfig(
    motorType="AK70-10",
    motorId=1,
    canBackend="gs_usb",   # Linux: "socketcan"
    canChannel=0,          # Linux: "can0"
    bitrate=1_000_000,
    maxTemperature=80,
    autoInit=False,        # Windows는 False (ip link 불필요)
)

with Motor(config=cfg) as motor:      # enter 시 enable, exit 시 disable
    motor.zero_position()             # 현재 위치를 0으로
    while True:
        motor.update()                # 상태 수신 + 명령 송신
        motor.set_torque(1.0)         # 예: 1 Nm 토크
        time.sleep(0.002)             # 500 Hz 루프
```

> 핵심 루프: 매 주기마다 `set_*()`로 명령을 걸고 `update()`를 호출해야 실제로 CAN 통신이 일어난다.

---

## MotorConfig

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `motorType` | `'AK70-10'` | 모터 모델 (`AK80-64`, `AK80-9` 등) |
| `motorId` | `1` | CAN ID (0–127) |
| `bitrate` | `1000000` | CAN 비트레이트 |
| `maxTemperature` | `50.0` | MOSFET 안전 온도 상한(°C) |
| `defaultKp` / `defaultKd` | `10.0` / `0.5` | 기본 위치/속도 게인 |
| `canBackend` | `'socketcan'` | `socketcan`(Linux) 또는 `gs_usb`(Windows) |
| `canChannel` | `'can0'` | socketcan은 `'can0'` 문자열, gs_usb는 `0` 정수 |
| `autoInit` | `True` | 시작 시 CAN 인터페이스 자동 up (Linux 전용) |

백엔드와 채널 타입은 생성 시 검증된다 — socketcan은 `'canN'` 문자열, gs_usb는 정수만 허용.

---

## 제어 메서드 (논블로킹)

| 메서드 | 설명 |
|--------|------|
| `set_position(pos, kp, kd, feedTor)` | 위치 제어 |
| `set_velocity(vel, kd)` | 속도 제어 |
| `set_torque(tor)` | 토크 제어 |
| `set_fullState(pos, vel, kp, kd, feedTor)` | MIT 풀스테이트: `tau = kp·(pos−p) + kd·(vel−v) + feedTor` |
| `set_position_smooth(pos, ...)` | 소프트 리밋 적용 위치 제어 |
| `stop()` | 비상 정지 (토크 0) |
| `zero_position()` | 현재 위치를 0으로 (엔코더 리셋, EEPROM 저장 ~1초) |
| `update()` | 명령 송신 + 상태 수신 (매 루프 호출) |

각도 단위는 rad, 속도 rad/s, 토크 Nm.

## 상태 조회 (property)

`position`, `velocity`, `torque`, `temperature`, `current`(q축 전류 A), `acceleration`, `error`(0=정상), `control_mode`

`update()`가 반환하는 dict로도 동일 값을 한 번에 받을 수 있다.

---

## 소프트 리밋

한계 위치 근처에서 댐핑(kd)을 자동으로 키워 벽처럼 부드럽게 멈춘다.

```python
motor.set_soft_limit(-1.57, 1.57)     # -90° ~ +90°
while True:
    motor.set_position_smooth(target) # 리밋 밖이면 자동 클램프
    motor.update()
    time.sleep(0.01)
```

`set_soft_limit(min, max, soft_zone=0.2, base_kd=2.0, max_kd=20.0)` — `soft_zone` 안으로 들어오면 `base_kd`에서 `max_kd`까지 선형 증가.

---

## 궤적 생성 (TrajectoryGenerator)

목표까지의 `(위치, 속도)`를 시간에 따라 계산하는 정적 유틸.

```python
from TMotorAPI import TrajectoryGenerator as Traj
pos, vel = Traj.minimum_jerk(start, end, t, duration)
```

- `minimum_jerk` — 5차 다항식 (jerk 최소)
- `cubic` — 3차 다항식
- `linear` — 선형 보간

---

## 예제

`demos/TEMPLATE.ipynb` — 500 Hz 제어 루프, 별도 스레드 상태 출력, Windows 1ms 타이머 해상도, 정밀 슬립 포함 실전 템플릿.

---

## 라이선스

MIT

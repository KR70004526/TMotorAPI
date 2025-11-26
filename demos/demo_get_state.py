from TMotorAPI import Motor, MotorConfig
import time

motor.enable()
try:
    while True:
        state = motor.update()
        pos  = state['position']
        vel  = state['velocity']
        tor  = state['torque']
        temp = state['temperature']

        print(f"Current State --> Pos: {pos:.3f}, Vel: {vel:.3f}, "
              f"Tor: {tor:.3f}, Temp: {temp:.1f}")

        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n[INFO] Stopped by user (Ctrl+C)")
finally:
    motor.disable()

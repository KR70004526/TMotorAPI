from TMotorAPI import Motor, MotorConfig
import time

# Motor Config & Setup
cfg = MotorConfig(motorType='AK70-10', motorId=2, maxTemperature=90)
motor = Motor(config=cfg)

#%% Example 1 - Without Duration
motor.enable()
motor.set_velocity(targetVel=3, duration=0, kd=0.5)
time.sleep(1)
motor.disable()

# To use the with statement, write it as follows:
with Motor(config=cfg) as motor:
    motor.set_velocity(targetVel=3, duration=0, kd=0.5)
    time.sleep(1)

#%% Example 2 - With Duration
motor.enable()
motor.set_velocity(targetVel=3, duration=4, kd=0.5)
time.sleep(1)
motor.disable()

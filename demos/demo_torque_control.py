from TMotorAPI import Motor, MotorConfig
import time

# Motor Config & Setup
cfg = MotorConfig(motorType='AK70-10', motorId=2, maxTemperature=80)
motor = Motor(config=cfg)

#%% Example 1 - Torque Control Without Duration
motor.enable()
motor.set_torque(targetTor=1, duration=0)
time.sleep(3)
motor.disable()

#%% Example 2 - Torque Control With Duration
motor.enable()
motor.set_torque(targetTor=1, duration=3)
motor.disable()

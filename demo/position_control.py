""" Demo Code for Position Control """
from TMotorAPI import Motor, MotorConfig
import numpy as np
import time

# Define Motor Setup Config
cfg = MotorConfig(motorType='AK70-10', motorId=1, maxTemperature=80)

# Position Control -> 1. Manually Turn on 
motor = Motor(config=cfg)
motor.enable() # Turn on Motor
time.sleep(1)
motor.zero_position() # Set Enocder Zero
time.sleep(1)
motor.track_trajectory(targetPos=np.pi/2, duration=0, kp=5, kd=1) # duration=0 --> Immediately Move to Traget Position
time.sleep(1)
motor.disable() # Turn off Motor

# Position Control --> 2. Automatically Turn on
with Motor(config=cfg) as motor:
  motor.zero_position() # Set Enocder Zero
  time.sleep(1)
  motor.track_trajectory(targetPos=np.pi/2, duration=0, kp=5, kd=1) # duration=0 --> Immediately Move to Traget Position

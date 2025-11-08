""" Demo Code for Track Trajectory Control """
from TMotorAPI import Motor, MotorConfig
import numpy as np
import time

# Define Motor Setup Config
cfg = MotorConfig(motorType='AK70-10', motorId=1, maxTemperature=80)

# Position Control -> 1. Manually Turn on 
motor = Motor(config=cfg)
motor.enable() # Turn on Motor
motor.zero_position() # Set Enocder Zero
time.sleep(1)
motor.track_trajectory(targetPos=np.pi/2, duration=2, kp=5, kd=1, trajectoryType='linear') # If duration > 0 --> Track Trajectory Mode
time.sleep(1)                                                                              # Trajectory Type = 'minimum_jerk', 'cubic', 'linear'
motor.disable() # Turn off Motor                                                           # Velocity changes if you change the duration

# Position Control --> 2. Automatically Turn on
with Motor(config=cfg) as motor:
  motor.zero_position() # Set Enocder Zero
  time.sleep(1)
  motor.track_trajectory(targetPos=np.pi/2, duration=2, kp=5, kd=1, trajectoryType='linear')

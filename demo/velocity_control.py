""" Demo Code for Velocity Control """
from TMotorAPI import Motor, MotorConfig
import time

# Define Motor Setup Config
cfg = MotorConfig(motorType='AK70-10', motorId=1, maxTemperature=80)

# Velocity Control -> 1. Manually Turn on 
motor = Motor(config=cfg)
motor.enable() # Turn on Motor
motor.set_velocity(targetVel=3, kd=2, duration=0) # This will run until motor turn off
time.sleep(1)                                     # If you want to run for desired time, then change the duration
motor.disable() # Turn off Motor

# Velocity Control --> 2. Automatically Turn on
with Motor(config=cfg) as motor:
  motor.set_velocity(targetVel=3, kd=2, duration=0) 

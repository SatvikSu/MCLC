# run front right wheel motor (channel 0, port 2, encoder pin a is 37, encoder pin b is 38)

# USAGE: python motor_closed_loop_vel.py omega_deg_per_sec
# do python motor_closed_loop_vel.py 0 to stop motor

import smbus
import time
from motoron import MotoronI2C
import sys
import matplotlib.pyplot as plt
import Jetson.GPIO as GPIO
import math

reference_omega_deg_per_sec = int(sys.argv[1])
Kp = 8 # Kp going from omega (deg/sec) to motor speed command (from -800 to 800). For motor voltage = 12 V
Ki = 2 # Ki going from omega (deg/sec) * time (sec) to motor speed command (from -800 to 800). For motor voltage = 12 V

# setup
# make multiplexer and Motoron motor controller objects
mux = smbus.SMBus(7)
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
mux.write_byte(112, 0b1) # address 0x70, channel 7
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

# run motor
# motoron.set_speed(2, speed) # port 2

# calculate motor velocity, do some closed - loop control by controlling speed with a PI controller to affect motor velolcity
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.IN)
enc_a_change_count = 0
outer_revs_count = 0
old_outer_revs_count = 0
prev_enc_a = GPIO.input(37)
old_time = time.perf_counter()
print_time = time.perf_counter()
omega_error_integral = 0 # for Ki
while True:     
     enc_a = GPIO.input(37)
     if enc_a != prev_enc_a:
          enc_a_change_count += 1
          quad_counts = enc_a_change_count * 2
          inner_revs_count = quad_counts / 12
          outer_revs_count = inner_revs_count / 986.41
          prev_enc_a = enc_a
     curr_time = time.perf_counter()  
     if curr_time - old_time >= 0.01: # calculate velocity and update speed command 
          delta_t = curr_time - old_time
          omega_rev_per_sec = (outer_revs_count - old_outer_revs_count) / delta_t
          omega_deg_per_sec = omega_rev_per_sec * 360
          old_time = curr_time
          old_outer_revs_count = outer_revs_count
          # closed loop stuff
          omega_error = reference_omega_deg_per_sec - omega_deg_per_sec
          omega_error_integral += omega_error * delta_t
          speed = Kp * omega_error + Ki * omega_error_integral
          motoron.set_speed(2, int(speed)) # port 2
     if time.perf_counter() - print_time >= 0.25:
          print(f'Reference omega (deg/s): {reference_omega_deg_per_sec}, Actual omega (deg/s): {omega_deg_per_sec}, Speed set to: {speed}')
          print_time = time.perf_counter()

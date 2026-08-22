# run rear right wheel motor (channel 7, port 2, encoder pin a is 7, encoder pin b is 11)

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
# mux.write_byte(112, 0b10000000) # address 0x70, channel 7
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

# run motor
# motoron.set_speed(2, speed) # port 2

# calculate motor velocity, do some closed - loop control by controlling speed with a PI controller to affect motor velolcity
GPIO.setmode(GPIO.BOARD)
GPIO.setup(31, GPIO.IN)
GPIO.setup(37, GPIO.IN)

enc_a_change_count_w = 0
outer_revs_count_w = 0
old_outer_revs_count_w = 0
prev_enc_a_w = GPIO.input(37)
omega_error_integral_w = 0 # for Ki

enc_a_change_count_l = 0
outer_revs_count_l = 0
old_outer_revs_count_l = 0
prev_enc_a_l = GPIO.input(31)
omega_error_integral_l = 0 # for Ki

old_time = time.perf_counter()
print_time = time.perf_counter()

while True:     

     enc_a_w = GPIO.input(37)
     if enc_a_w != prev_enc_a_w:
          enc_a_change_count_w += 1
          quad_counts_w = enc_a_change_count_w * 2
          inner_revs_count_w = quad_counts_w / 12
          outer_revs_count_w = inner_revs_count_w / 986.41
          prev_enc_a_w = enc_a_w
          
     enc_a_l = GPIO.input(31)
     if enc_a_l != prev_enc_a_l:
          enc_a_change_count_l += 1
          quad_counts_l = enc_a_change_count_l * 2
          inner_revs_count_l = quad_counts_l / 12
          outer_revs_count_l = inner_revs_count_l / 986.41
          prev_enc_a_l = enc_a_l
          
     curr_time = time.perf_counter()  
     if curr_time - old_time >= 0.01: # calculate velocity and update speed command 
     
          delta_t = curr_time - old_time
          old_time = curr_time
          
          omega_rev_per_sec_w = (outer_revs_count_w - old_outer_revs_count_w) / delta_t
          omega_deg_per_sec_w = omega_rev_per_sec_w * 360
          old_outer_revs_count_w = outer_revs_count_w
          # closed loop stuff
          omega_error_w = reference_omega_deg_per_sec - omega_deg_per_sec_w
          omega_error_integral_w += omega_error_w * delta_t
          speed_w = Kp * omega_error_w + Ki * omega_error_integral_w
          mux.write_byte(112, 0b1)
          motoron.set_speed(2, int(speed_w)) 
          
          omega_rev_per_sec_l = (outer_revs_count_l - old_outer_revs_count_l) / delta_t
          omega_deg_per_sec_l = omega_rev_per_sec_l * 360
          old_outer_revs_count_l = outer_revs_count_l
          # closed loop stuff
          omega_error_l = reference_omega_deg_per_sec - omega_deg_per_sec_l
          omega_error_integral_l += omega_error_l * delta_t
          speed_l = Kp * omega_error_l + Ki * omega_error_integral_l
          mux.write_byte(112, 0b10)
          motoron.set_speed(2, int(speed_l)) 
          
     if time.perf_counter() - print_time >= 0.25:
          print(f'Reference omega (deg/s): {reference_omega_deg_per_sec}')
          print(f'Wheel omega (deg/s): {omega_deg_per_sec_w}')
          print(f'Leg omega (deg/s): {omega_deg_per_sec_l}')
          print_time = time.perf_counter()

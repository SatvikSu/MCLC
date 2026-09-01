# USAGE: python motor_closed_loop_pos.py <theta in degrees>
# run front right wheel motor (channel 0, port 2, encoder pin a is 37, encoder pin b is 38)

import smbus
import time
from motoron import MotoronI2C
import sys
import Jetson.GPIO as GPIO

reference_theta_deg = int(sys.argv[1])

# Constants
Kp_omega = 8 # Kp going from omega (deg/sec) to motor speed command (from -800 to 800). For motor voltage = 12 V
Ki_omega = 2 # Ki going from omega (deg/sec) * time (sec) to motor speed command (from -800 to 800). For motor voltage = 12 V
Kp_theta = 0.5 # Kp going from theta (deg) to omega (deg/sec)
omega_update_period = 0.01 # time in seconds to update omega in inner control loop
theta_update_period = 0.05 # time in seconds to update theta in outer control loop
print_period = 0.25 # time in seconds to print
deadband_deg = 0.5 # deaband, in degrees, to stop running control loop

# setup
# make multiplexer and Motoron motor controller objects
mux = smbus.SMBus(7)
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
mux.write_byte(112, 1) # address 0x70, channel 0
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.IN)

enc_a_count = 0
outer_deg_count = 0
prev_outer_deg_count = 0
enc_a = GPIO.input(37)
prev_enc_a = GPIO.input(37)
prev_time_omega = time.perf_counter()
prev_time_theta = time.perf_counter()
print_time = time.perf_counter()
omega_error_integral = 0 # for Ki_omega

theta_error = reference_theta_deg - 0
reference_omega_deg_per_sec = Kp_theta * theta_error

while True:     
     enc_a = GPIO.input(37)
     if enc_a != prev_enc_a:
          enc_a_count += 1
          outer_deg_count = enc_a_count * 2 / 12 / 986.41 * 360
          prev_enc_a = enc_a
     curr_time = time.perf_counter()  
     if curr_time - prev_time_omega >= omega_update_period: # calculate velocity and update speed command 
          # calculate velocity
          delta_t = curr_time - prev_time_omega
          omega_deg_per_sec = (outer_deg_count - prev_outer_deg_count) / delta_t
          prev_time_omega = curr_time
          prev_outer_deg_count = outer_deg_count
          # closed loop stuff
          omega_error = reference_omega_deg_per_sec - omega_deg_per_sec
          omega_error_integral += omega_error * delta_t
          speed = Kp_omega * omega_error + Ki_omega * omega_error_integral
          motoron.set_speed(2, int(speed)) # port 2
     if curr_time - prev_time_theta >= theta_update_period: # calculate position error and update velocity command
          # closed loop stuff
          theta_error = reference_theta_deg - outer_deg_count
          # deadband code
          if abs(theta_error) <= deadband_deg:
               print(f'Theta error (deg): {theta_error}, Deadband reached')
               motoron.set_speed(2, 0)
               break
          else:
               reference_omega_deg_per_sec = Kp_theta * theta_error  
     if time.perf_counter() - print_time >= print_period:
          print(f'Reference theta (deg): {reference_theta_deg}, Actual theta (deg): {outer_deg_count}, Velocity set to: {reference_omega_deg_per_sec} (deg/s)')
          print_time = time.perf_counter()
          

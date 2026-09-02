# run front right wheel motor for 10 revs (channel 0, port 2, encoder pin a is 37, encoder pin b is 38)

import smbus
import time
from motoron import MotoronI2C
import sys
import matplotlib.pyplot as plt
import Jetson.GPIO as GPIO
import math

# setup
# make multiplexer and Motoron motor controller objects
mux = smbus.SMBus(7)
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
mux.write_byte(112, 0b1) # address 0x70, channel 7
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

# calculate motor velocity, do some closed - loop control by controlling speed with a PI controller to affect motor velolcity
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.IN)
enc_a_change_count = 0
outer_revs_count = 0
prev_enc_a = GPIO.input(37)

motoron.set_speed(2, 400)
print('Going 10 revs')
while outer_revs_count < 10:     
     enc_a = GPIO.input(37)
     if enc_a != prev_enc_a:
          enc_a_change_count += 1
          quad_counts = enc_a_change_count * 2
          inner_revs_count = quad_counts / 12
          outer_revs_count = inner_revs_count / 986.41
          prev_enc_a = enc_a
motoron.set_speed(2, 0)
print('Went 10 revs')
GPIO.cleanup()

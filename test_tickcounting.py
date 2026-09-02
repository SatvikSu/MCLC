# run front right wheel motor for 10 revs (channel 0, port 2, encoder pin a is 37, encoder pin b is 38)

import smbus
import time
from motoron import MotoronI2C
import matplotlib.pyplot as plt
import Jetson.GPIO as GPIO
import threading

# setup
mux = smbus.SMBus(7)
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
mux.write_byte(112, 0b1) # address 0x70, channel 0
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.IN)
GPIO.setup(38, GPIO.IN)

outer_revs_count = 0

def main():
     x = threading.Thread(target=read_ticks, daemon=True)
     print('Created thread for reading enc ticks')
     x.start()
     print('Started thread')
     motoron.set_speed(2, 400)
     print('Going 10 revs')

     while abs(outer_revs_count) < 10:
          print(f'Revs: {outer_revs_count}')
          time.sleep(0.01)
          
     motoron.set_speed(2, 0)
     print('Went 10 revs')
     GPIO.cleanup()


def read_ticks():     
     prev_a = GPIO.input(37)
     while True:     
          a = GPIO.input(37)
          b = GPIO.input(38)
          delta = 0
          if a != prev_a:
               if a == 0:
                    if b == 1:
                         delta = -2
                    elif b == 0:
                         delta = 2
               elif a == 1:
                    if b == 0:
                         delta = -2
                    elif b == 1:
                         delta = 2
               global outer_revs_count
               outer_revs_count += delta / 12 / 986.41
               prev_a = a
     

if __name__ == '__main__':
     main()
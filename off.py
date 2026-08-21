# turn off L_FR motor
import smbus
from motoron import MotoronI2C
import Jetson.GPIO as GPIO

# make multiplexer and Motoron motor controller objects
mux = smbus.SMBus(7)
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
mux.write_byte(112, 0b10) # address 0x70, channel 1
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()


motoron.set_speed(2,0)

GPIO.setmode(GPIO.BOARD)
GPIO.remove_event_detect(37)
GPIO.remove_event_detect(38)
GPIO.cleanup()

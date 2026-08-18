# evental goal: make wheels spin at a certain speed while legs maintain a certain angle to wheel

# steps to get to that:
# 1) read encoder(s) with event detects intead of while True() and if(change) (aka do it properly)

import smbus
import time
import threading
from motoron import MotoronI2C
import sys
import Jetson.GPIO as GPIO

class QuadEncoder:
    # ------- Transition Table ---------
    # State format is: AB
    # Four possible states: 00, 01, 11, 10
    # The encoders change steps by the bit such that:
    # Forwards: 00 -> 01 -> 11 -> 10 ()
    # Backwards: 10 -> 11 -> 01 -> 00
    _TT = {
        (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
        (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
    }
    def __init__(self, pin_a, pin_b, sign=+1):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.sign = 1 if sign >= 0 else -1
        self._count = 0
        self._lock  = threading.Lock()
        self._last  = 0
        # Config GPIO as an input pin
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        a = GPIO.input(self.pin_a); b = GPIO.input(self.pin_b)
        self._last = (a << 1) | b # Bitwise left shift to capture current baseline edge
        # Adding event detection (interrupt) to detect edges for A and B
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._edge, bouncetime=0)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._edge, bouncetime=0)
    
    # Method to detect the new state of A and B channels
    def _edge(self, _):
        a = GPIO.input(self.pin_a); b = GPIO.input(self.pin_b)
        state = (a << 1) | b # Bitwise left shift to capture new state
        if state != self._last: 
            delta = self._TT.get((self._last, state), 0) # Compares old and new state with transition table
            if delta:
                with self._lock:
                    self._count += delta * self.sign
            self._last = state
    
    # Method to read the current tick count # CONVERT TO ROTATIONAL (rad or deg)
    def read(self):
        with self._lock:
            return self._count

## SETUP
# GPIO, multiplexer, and motor setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Allows to call GPIO pins by their physical location #'s
mux = smbus.SMBus(7)
mux.write_byte(112, 1) # address 0x70, channel 0
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()


## CODE
encoder = QuadEncoder(37, 38)
motoron.set_speed(2,-300)


print_time = time.perf_counter()
print_period = 0.25
start_time = time.perf_counter()
run_time = 5


while time.perf_counter() - start_time < run_time:
	if time.perf_counter() - print_time >= print_period:
		print(f'Ticks: {encoder.read()}')
		print_time = time.perf_counter()

# time.sleep(run_time)
print(f'Done. Ticks: {encoder.read()}')
# problem to solve: ticks seem to be working with the time.sleep() method,
# but when using the commented-out while loop, the ticks read are much smaller than expected. Why? TODO
motoron.set_speed(2,0)

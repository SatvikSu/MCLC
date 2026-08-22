# Goal: drive robot on ground with front two wheels' legs out
# Program should find min and max range of legs, then it should: 
#    1) maintain wheel motor at a certain angular velocity
#    2) maintain leg at a certain angle w.r.t. wheel 
# leg angle = leg motor angle - wheel motor angle

import smbus
import time
import threading
from motoron import MotoronI2C
import sys
import Jetson.GPIO as GPIO

reference_wheel_omega = 90 # deg / sec
reference_leg_theta = 90 # deg. should make this the midpoint after implementing max angle

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

############################################################################################################

def get_leg_bounds():

    # find leg min

    stall_deg = 2 # degrees: if motor moves less than this amt after time.sleep(time), it has stalled
    stall_time = 0.05 # seconds
    stall_min = False
    print('Finding leg min')
    motoron.set_speed(2, 250)
    while not stall_min:
        prev_counts = encoder.read()
        time.sleep(stall_time)
        counts = encoder.read()
        degs = (counts - prev_counts) / (12 * 986.41) * 360
        print(degs) # testing
        if abs(degs) < stall_deg:
            stall_min = True
            motoron.set_speed(2,0)
    encoder._count = 0
    print('Leg min determined, encoder count set to 0 here')
    print('Rotating motor other way a bit')
    # rotate the motors back a little bit
    motoron.set_speed(2, -250)
    time.sleep(2)
    motoron.set_speed(2,0)
    time.sleep(2)

    # find leg max
    print('Finding leg max')
    stall_max = False
    motoron.set_speed(2,-250)
    while not stall_max:
        prev_counts = encoder.read()
        time.sleep(stall_time)
        counts = encoder.read()
        degs = (counts - prev_counts) / (12 * 986.41) * 360
        print (abs(degs)) # testing
        if abs(degs) < stall_deg:
            stall_max = True
            motoron.set_speed(2,0)
    max_degs = encoder.read() / (12*986.41) * 360
    print(f'Leg max determined: {max_degs} degrees')
    print('Rotating motor other way a bit')
    motoron.set_speed(2,250)
    time.sleep(2)
    motoron.set_speed(0)
    print('Done')


## SETUP
# GPIO, multiplexer, and motor setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Allows to call GPIO pins by their physical location #'s
mux = smbus.SMBus(7)
mux.write_byte(112, 0b10) # address 0x70, channel 1 for L_FR
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

## CODE: stall detect
encoder = QuadEncoder(31, 32)
get_leg_bounds()
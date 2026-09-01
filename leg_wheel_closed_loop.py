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
from math import pi

reference_wheel_omega = 90 # deg / sec
reference_leg_theta = 90 # deg. should make this the midpoint after implementing max angle

############################################################################################################

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

class VelEMA: 
    def __init__(self, encoder, alpha=0.2):
        self.enc = encoder
        self.alpha = alpha # low alpha -> low filtering
        self.prev_rad = encoder.read() * (2*pi/12) / 986.41
        self.prev_t = time.perf_counter()
        self.ema = 0.0

    def update(self):
        now = time.perf_counter()
        theta  = self.enc.read() * (2*pi/12) / 986.41 # Gets current angular position
        dt  = max(1e-4, now - self.prev_t)
        rps = (theta - self.prev_rad) / dt # Radians per second (CONVERT TO COUNTS PER CM IN OUTPUT)
        self.ema = self.alpha * rps + (1 - self.alpha) * self.ema
        self.prev_rad, self.prev_t = theta, now
        return self.ema

############################################################################################################

def get_leg_bounds(encoder):

    # find leg min

    stall_deg = 2 # degrees: if motor moves less than this amt after time.sleep(time), it has stalled
    stall_time = 0.05 # seconds
    stall_min = False
    print('Finding leg min')
    motoron.set_speed(1, 250)
    while not stall_min:
        prev_counts = encoder.read()
        time.sleep(stall_time)
        counts = encoder.read()
        degs = (counts - prev_counts) / (12 * 986.41) * 360
        print(degs) # testing
        if abs(degs) < stall_deg:
            stall_min = True
            motoron.set_speed(1,0)
    L_FL_encoder._count = 0
    print('Leg min determined, encoder count set to 0 here')

    # find leg max
    print('Finding leg max')
    stall_max = False
    motoron.set_speed(1,-250)
    while not stall_max:
        prev_counts = encoder.read()
        time.sleep(stall_time)
        counts = encoder.read()
        degs = (counts - prev_counts) / (12 * 986.41) * 360
        print (abs(degs)) # testing
        if abs(degs) < stall_deg:
            stall_max = True
            motoron.set_speed(1,0)
    max_degs = encoder.read() / (12*986.41) * 360
    print(f'Leg max determined: {max_degs} degrees')
    print('Rotating motor other way a bit')
    motoron.set_speed(1,250)
    time.sleep(2)
    motoron.set_speed(1, 0)
    print('Done')

def set_speed_wfr(speed):
    mux.write_byte(112, 0b1)
    motoron.set_speed(1,speed)

def set_speed_lfl(speed):
    mux.write_byte(112, 0b10)
    motoron.set_speed(1,speed)

## SETUP
# GPIO, multiplexer, and motor setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Allows to call GPIO pins by their physical location #'s
mux = smbus.SMBus(7)
mux.write_byte(112, 0b10) # address 0x70, channel 1 for L_FL
motoron = MotoronI2C(bus=7, address=16) # address 0x10 
motoron.reinitialize()
motoron.clear_reset_flag()
motoron.disable_command_timeout()

## CODE: stall detect
L_FL_encoder = QuadEncoder(23, 24)
#W_FR_encoder = QuadEncoder(37, 38)
L_FL_VelEMA = VelEMA(L_FL_encoder)
#W_FR_VelEMA = VelEMA(W_FR_encoder)

get_leg_bounds(L_FL_encoder)
time.sleep(5)

## closed-loop control of both leg and wheel motors' velocities
# reference_omega_deg_per_sec = int(sys.argv[1])
#reference_omega_deg_per_sec = 90
#Kp = 8 # Kp going from omega (deg/sec) to motor speed command (from -800 to 800). For motor voltage = 12 V
#Ki = 2 # Ki going from omega (deg/sec) * time (sec) to motor speed command (from -800 to 800). For motor voltage = 12 V

#set_speed_wfr(300)
#set_speed_lfr(-300)        

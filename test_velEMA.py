#!/usr/bin/env python3
"""
motor_interface_node.py
- Gives speed cmds to the 8 motors 

PUB: motor/velocity
PUB: leg/angle
PUB: init/motor_ready
SUB: init/motor_init
SUB: motor/speed_cmd
"""

import time
import threading
from math import copysign
from math import pi
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, String, Int32MultiArray, Float32MultiArray

from motoron import MotoronI2C # Used for motor actuation
import Jetson.GPIO as GPIO
import smbus # System Management Bus

# Global I2C mux + Motoron config
MUX_ADDR       = 0x70 # MUX I2C address
MUX_BUS        = 7 # MUX I2C Bus number (/dev/i2c-7)
MOTORON_ADDR   = 0x10 # Encoder I2C address (all encoders use the same address)
RUN_TIMEOUTS   = 10.0  # Timeout for motor movement until ending the loop
VEL_FILTER_A   = 0.2

# ---- OLD PID CODE ------
# min_PWM        = 200 
# KP_POS_FAR     = 0.8 
# KP_POS_NEAR    = 2 # 1.2
# KI_POS         = 0
# KD_POS         = 0
# I_ERR_MAX      = 5000.0
# NEAR_THRESH    = 2000
# MIN_CMD        = 60
# DEADBAND       = 40
# HOLD_TIME      = 0.25
# STALL_WINDOW   = 0.5
# STALL_MIN_CMD  = 200

# Rotational movement
TICKS_PER_REV = 12 # 12 digital pulses per 360 revolution
GEAR_RATIO = 986.41

# GPIO + encoder mapping
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Allows to call GPIO pins by their physical location #'s

# (mux_channel, port) -> (pinA, pinB)
# 4 total motor controllers, 8 total controlled actuators
ENCODER_PINS_BY_CH_PORT = { # are the enc pins wrong?
    # Mux 7 (m1) -> Back Wheels
    (7, 1): (12, 13),  # back_wheel_1 (7, 1) -> (12, 13)
    (7, 2): (7, 11),   # back_wheel_2

    # Mux 6 (m2) -> Legs
    (6, 1): (15, 16),  # legs_1
    (6, 2): (21, 22),  # legs_2

    # Mux 1 (m3) -> Legs
    (1, 1): (23, 24),  # legs_3
    (1, 2): (31, 32),  # legs_4

    # Mux 0 (m4) -> Front Wheels
    (0, 1): (35, 36),  # front_wheel_1
    (0, 2): (37, 38),  # front_wheel_2
}


# Helper functions
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

# ------------- POSITION (TICKS) ------------------
# Decodes a standard two-channel quadrature encoder signal into direction and a tick count
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


# ---------- VELOCITY ---------------
# Class to estimate the current rotational velocity in encoder counts per second
# Uses an exponenial moving average (EMA) filter
class VelEMA: 
    def __init__(self, encoder, alpha=0.2):
        self.enc = encoder
        self.alpha = alpha # low alpha -> low filtering
        self.prev_rad = encoder.read() * (2*pi/TICKS_PER_REV) / GEAR_RATIO
        self.prev_t = time.perf_counter()
        self.ema = 0.0

    def update(self):
        now = time.perf_counter()
        theta  = self.enc.read() * (2*pi/TICKS_PER_REV) / GEAR_RATIO # Gets current angular position
        dt  = max(1e-4, now - self.prev_t)
        rps = (theta - self.prev_rad) / dt # Radians per second (CONVERT TO COUNTS PER CM IN OUTPUT)
        self.ema = self.alpha * rps + (1 - self.alpha) * self.ema
        self.prev_rad, self.prev_t = theta, now
        return self.ema

# Shared Motoron object; select mux channel before issuing commands
mux = smbus.SMBus(MUX_BUS)
def select_channel(ch):
    mux.write_byte(MUX_ADDR, 1 << ch)
    time.sleep(0.001)

motoron = MotoronI2C(bus=MUX_BUS, address=MOTORON_ADDR)

def init_motoron_on(ch):
    select_channel(ch)
    motoron.reinitialize()
    motoron.clear_reset_flag()
    motoron.disable_command_timeout()


# Axis is representative of ONE motor
class Axis:
    def __init__(self, name, mux_ch, port, enc_sign=+1, speed_max=600,vel_alpha=VEL_FILTER_A,
                 motor_sign=+1, leg_min = 0, leg_max = 0):
        self.name = name
        self.ch   = mux_ch
        self.port = port

        # Motor_sign flips the physical drive direction for a given command.
        # Pair it with enc_sign so a +command still increases pos (loop stays stable).
        self.motor_sign = 1 if motor_sign >= 0 else -1
        if (self.ch, self.port) not in ENCODER_PINS_BY_CH_PORT:
            raise RuntimeError(f"No encoder pins mapped for channel {self.ch}, port {self.port}")
        a, b = ENCODER_PINS_BY_CH_PORT[(self.ch, self.port)] # Finds the corresponding encoder pins by channel and port id
        self.enc = QuadEncoder(a, b, sign=enc_sign) # Gets encoder values
        self.vel = VelEMA(self.enc, alpha=vel_alpha) # Gets velocity

        self.speed_max = speed_max

        # ------ Leg calibration value --------
        self.min = leg_min
        self.max = leg_max

    # Sets the speed of the motor and direction
    def set_speed(self, u):
        select_channel(self.ch)
        u = u * self.motor_sign
        motoron.set_speed(self.port, int(clamp(u, -self.speed_max, self.speed_max)))

    # Returns the angular position of the motor in RADIANS
    def get_pos(self): 
        return self.enc.read() * (2*pi/TICKS_PER_REV) / GEAR_RATIO

    def get_vel(self):
        return self.vel.update()
    
    def set_bounds(self, retract, extend):
        self.min = retract
        self.max = extend

def main():
    W_FR_motor = Axis('W_FR', ch=0, port=2, enc_sign=-1, speed_max=800)
    W_FR_motor.set_speed(200)
    for _ in range(10):
        print(W_FR_motor.get_vel())
        time.sleep(0.5)

if __name__ == '__main__':
    main()

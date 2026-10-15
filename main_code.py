from simple_pid import PID
import time
import board
import busio
import rotaryio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_motorkit import MotorKit
from digitalio import DigitalInOut, Direction, Pull
from adafruit_debouncer import Button

#---Hardware setup
pin1 = DigitalInOut(board.D12)
pin1.direction = Direction.INPUT
pin1.pull = Pull.UP
button1 = Button(pin1)
start = False

pin2 = DigitalInOut(board.D11)
pin2.direction = Direction.INPUT
pin2.pull = Pull.UP
button2 = Button(pin2)

i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit(i2c=i2c)
ads = ADS.ADS1115(i2c)
encoder = rotaryio.IncrementalEncoder(board.D9, board.D10)

kit.motor2.throttle = 0
print("Code started")

#---Sensor settings
SENSOR_LEFT_PIN = ADS.P0
SENSOR_RIGHT_PIN = ADS.P1

ALPHA = 0.15
filtered_voltage_left = None
filtered_voltage_right = None

GAIN_VALUES = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]
GAIN_SETTINGS = [2/3, 1, 2, 4, 8, 16]

def apply_low_pass_filter(new_voltage, previous_filtered_voltage, alpha):
    if previous_filtered_voltage is None:
        return new_voltage
    return alpha * new_voltage + (1 - alpha) * previous_filtered_voltage

def auto_gain_read(channel):
    last_voltage = None
    for gain_index in range(len(GAIN_SETTINGS) - 1, -1, -1):
        ads.gain = GAIN_SETTINGS[gain_index]
        analog_in = AnalogIn(ads, channel)
        voltage = analog_in.voltage
        last_voltage = voltage
        max_voltage = GAIN_VALUES[gain_index]
        if abs(voltage) <= max_voltage * 0.9:
            return voltage
    return last_voltage

ads.gain = 1
time.sleep(1)

#---PID Controllers
pid_distance = PID(Kp=0.4, Ki=0.2, Kd=0.5, setpoint=26)
pid_distance.output_limits = (-5, 5)
pid_distance.sample_time = 0.005

pid_angle = PID(Kp=0.1, Ki=0, Kd=0, setpoint=0)
pid_angle.output_limits = (-1, 1)
pid_angle.sample_time = 0.001

#---Distance calibration constants
a1, b1, c1 = 10.999, -3.298, 0.264
a2, b2, c2 = 10.998, -1.239, 0.264
L = 50
EPS = 1e-3
MAX_PLAUSIBLE = 53

#--- Safe distance functions, no really small denominators
def safe_distance_left(v_left):
    if v_left is None:
        return None
    denom = v_left - c1
    if abs(denom) < EPS:
        return None
    d = abs((a1 / denom) - b1)
    return d if d < MAX_PLAUSIBLE else None

def safe_distance_right(v_right):
    if v_right is None:
        return None
    denom = v_right - c2
    if abs(denom) < EPS:
        return None
    d = abs((-a2 / denom) + b2 + L)
    return d if d < MAX_PLAUSIBLE else None

def final_distance_safe(v_left, v_right):
    l = safe_distance_left(v_left)
    r = safe_distance_right(v_right)

    if l is None and r is None:
        return None
    if l is None:
        return r
    if r is None:
        return l

#---Adding an extra offset to correct distance at the ends
    ave = (l + r) / 2
    if ave < 10:
        out = abs(r - 3)
    elif ave < 14:
        w = (ave - 10) / 4
        out = abs(w * ave + (1 - w) * (r - 3))
    elif ave > 40:
        out = abs(l + 3)
    elif ave > 36:
        w = (ave - 36) / 4
        out = abs(w * (l + 3) + (1 - w) * ave)
    else:
        out = abs(ave)

    return out if out < MAX_PLAUSIBLE else None

#---Motor throttle settings
min_throttle = 0.6
last_valid_distance = 26

#--- Main loop
while True:
    button1.update()
    button2.update()

    if button1.pressed:
        start = True
        start_time = time.monotonic()
        print("Logging gestart")

    if button2.pressed:
        start = False
        kit.motor2.throttle = 0
        print("Logging gestopt")

    if start:
        voltage_left = auto_gain_read(SENSOR_LEFT_PIN)
        voltage_right = auto_gain_read(SENSOR_RIGHT_PIN)

        filtered_voltage_left = apply_low_pass_filter(voltage_left, filtered_voltage_left, ALPHA)
        filtered_voltage_right = apply_low_pass_filter(voltage_right, filtered_voltage_right, ALPHA)

        final_afstand = final_distance_safe(filtered_voltage_left, filtered_voltage_right)

        #--- Use last valid measurement if measurement unvalid
        if final_afstand is None:
            final_afstand = last_valid_distance
        else:
            last_valid_distance = final_afstand

        # --- Cascade PID ---
        target_angle = pid_distance(final_afstand)
        pid_angle.setpoint = target_angle

        current_angle = encoder.position / 2.95
        motorspeed = pid_angle(current_angle)

        # --- Throttle mapping ---
        if abs(motorspeed) < 0.01:
            motorspeed_final = 0
        else:
            motorspeed_final = (min_throttle if motorspeed > 0 else -min_throttle) + (0.68 - min_throttle) * motorspeed

        kit.motor2.throttle = motorspeed_final

        # --- Debug output for Mu Plotter ---
        print(final_afstand)

    time.sleep(0.001)

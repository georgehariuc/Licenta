#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import threading
 
# All GPIO numbers below use BCM numbering.
 
# AlphaBot2 motor driver
LEFT_IN1, LEFT_IN2, LEFT_PWM = 12, 13, 6
RIGHT_IN1, RIGHT_IN2, RIGHT_PWM = 20, 21, 26
 
# Encoders: physical header pins converted to BCM
ENC = {
    "L": (10, 8),   # Physical pins 19, 24
    "R": (9, 11),   # Physical pins 21, 23
}
 
# Adjust signs so forward wheel rotation is positive.
ENC_SIGN = {"L": -1, "R": 1}
 
# Start slowly. Target is decoded encoder counts per second.
TARGET = 100.0
CONTROL_PERIOD = 0.05  # 50 ms
 
# Initial PI gains; these require tuning.
KP = 0.15
KI = 0.40
 
# Motor wiring direction: reverse these if necessary.
MOTOR_FORWARD = {"L": (1, 0), "R": (0, 1)}
 
lock = threading.Lock()
count = {"L": 0, "R": 0}
state = {}
integral = {"L": 0.0, "R": 0.0}
pwm_value = {"L": 0.0, "R": 0.0}
 
 
def read_ab(wheel):
    a, b = ENC[wheel]
    return (GPIO.input(a), GPIO.input(b))
 
 
# Quadrature decoder: +1/-1 depends on signal phase.
TRANSITIONS = {
    (0, 0, 0, 1): 1, (0, 1, 1, 1): 1,
    (1, 1, 1, 0): 1, (1, 0, 0, 0): 1,
    (0, 0, 1, 0): -1, (1, 0, 1, 1): -1,
    (1, 1, 0, 1): -1, (0, 1, 0, 0): -1,
}
 
 
def encoder_callback(wheel):
    new = read_ab(wheel)
    with lock:
        old = state[wheel]
        if new != old:
            delta = TRANSITIONS.get((*old, *new), 0)
            count[wheel] += delta * ENC_SIGN[wheel]
            state[wheel] = new
 
 
def set_motor(wheel, duty):
    if wheel == "L":
        in1, in2 = LEFT_IN1, LEFT_IN2
        pwm = left_pwm
    else:
        in1, in2 = RIGHT_IN1, RIGHT_IN2
        pwm = right_pwm
 
    duty = max(0.0, min(100.0, duty))
 
    if duty <= 0:
        pwm.ChangeDutyCycle(0)
        return
 
    forward = MOTOR_FORWARD[wheel]
    GPIO.output(in1, forward[0])
    GPIO.output(in2, forward[1])
    pwm.ChangeDutyCycle(duty)
 
 
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
 
    for pin in (
        LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2,
        LEFT_PWM, RIGHT_PWM
    ):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
 
    for wheel, (a, b) in ENC.items():
        GPIO.setup(a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        state[wheel] = read_ab(wheel)
 
    left_pwm = GPIO.PWM(LEFT_PWM, 1000)
    right_pwm = GPIO.PWM(RIGHT_PWM, 1000)
    left_pwm.start(0)
    right_pwm.start(0)
 
    # Register callbacks after initializing the initial states.
    for wheel, (a, b) in ENC.items():
        callback = lambda channel, w=wheel: encoder_callback(w)
        GPIO.add_event_detect(a, GPIO.BOTH, callback=callback)
        GPIO.add_event_detect(b, GPIO.BOTH, callback=callback)
 
    previous = {"L": 0, "R": 0}
    last = time.monotonic()
 
    print("Speed controller running. Ctrl+C to stop.")
    print("Target:", TARGET, "encoder counts/s")
 
    while True:
        time.sleep(CONTROL_PERIOD)
        now = time.monotonic()
        dt = now - last
        last = now
 
        with lock:
            current = count.copy()
 
        measured = {
            w: (current[w] - previous[w]) / dt
            for w in ("L", "R")
        }
        previous = current
 
        for w in ("L", "R"):
            error = TARGET - measured[w]
            candidate = integral[w] + error * dt
 
            output = KP * error + KI * candidate
 
            # Basic anti-windup at the PWM limits.
            if 0 < output < 100:
                integral[w] = candidate
 
            output = KP * error + KI * integral[w]
            output = max(0.0, min(100.0, output))
            pwm_value[w] = output
            set_motor(w, output)
 
        print(
            f"L: {measured['L']:7.1f} counts/s, "
            f"PWM {pwm_value['L']:5.1f}% | "
            f"R: {measured['R']:7.1f} counts/s, "
            f"PWM {pwm_value['R']:5.1f}%"
        )
 
except KeyboardInterrupt:
    print("\nStopping.")
 
finally:
    try:
        left_pwm.stop()
        right_pwm.stop()
    except (NameError, RuntimeError):
        pass
    GPIO.cleanup()

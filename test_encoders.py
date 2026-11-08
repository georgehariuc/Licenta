#!/usr/bin/env python3
 
from gpiozero import DigitalInputDevice
from time import sleep, monotonic
from signal import signal, SIGINT
from sys import exit
 
# BCM GPIO numbers corresponding to physical header pins
ENCODERS = {
    "Left":  {"A": 10,  "B": 8},
    "Right": {"A": 9, "B": 11},
}
 
# Quadrature transitions.
# Positive/negative sign indicates direction, depending on wiring.
TRANSITIONS = {
    (0, 0, 0, 1):  1,
    (0, 1, 1, 1):  1,
    (1, 1, 1, 0):  1,
    (1, 0, 0, 0):  1,
    (0, 0, 1, 0): -1,
    (1, 0, 1, 1): -1,
    (1, 1, 0, 1): -1,
    (0, 1, 0, 0): -1,
}
 
devices = {}
states = {}
counts = {"Left": 0, "Right": 0}
changes = {"Left": 0, "Right": 0}
invalid = {"Left": 0, "Right": 0}
 
 
def read_state(wheel):
    a = devices[wheel]["A"].value
    b = devices[wheel]["B"].value
    return (a, b)
 
 
def make_callback(wheel):
    def callback(_pin):
        new = read_state(wheel)
        old = states[wheel]
 
        if new == old:
            return
 
        changes[wheel] += 1
        key = (old[0], old[1], new[0], new[1])
 
        if key in TRANSITIONS:
            direction = TRANSITIONS[key]
            
            if wheel == "left":
                direction = -direction
            counts[wheel] += direction
        else:
            invalid[wheel] += 1
 
        states[wheel] = new
 
    return callback
 
 
def cleanup(_signal=None, _frame=None):
    print("\nStopping encoder test.")
    for wheel in devices:
        for channel in devices[wheel].values():
            channel.close()
    exit(0)
 
 
signal(SIGINT, cleanup)
 
try:
    # Initial setup: inputs only, with pull-ups enabled.
    for wheel, pins in ENCODERS.items():
        devices[wheel] = {
            "A": DigitalInputDevice(pins["A"], pull_up=True),
            "B": DigitalInputDevice(pins["B"], pull_up=True),
        }
        states[wheel] = read_state(wheel)
 
    # Listen to changes on either encoder channel.
    for wheel in ENCODERS:
        callback = make_callback(wheel)
        devices[wheel]["A"].when_activated = callback
        devices[wheel]["A"].when_deactivated = callback
        devices[wheel]["B"].when_activated = callback
        devices[wheel]["B"].when_deactivated = callback
 
    print("Encoder test running. Rotate each wheel by hand.")
    print("Ctrl+C to stop.\n")
    print("Wheel   A B   Signed transitions   Transitions/s   Invalid")
 
    last = monotonic()
    last_changes = {"Left": 0, "Right": 0}
 
    while True:
        sleep(1)
        now = monotonic()
        dt = now - last
 
        for wheel in ENCODERS:
            a, b = read_state(wheel)
            rate = (changes[wheel] - last_changes[wheel]) / dt
 
            print(
                f"{wheel:6}  {a} {b}   "
                f"{counts[wheel]:+10d}          "
                f"{rate:8.1f}       {invalid[wheel]}"
            )
 
            last_changes[wheel] = changes[wheel]
 
        print()
        last = now
 
except Exception as e:
    print(f"Error: {e}")
    cleanup()

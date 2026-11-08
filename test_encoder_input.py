from gpiozero import DigitalInputDevice
from signal import pause
 
# BCM GPIO numbers corresponding to physical header pins
pins = {
    "Left A (physical 24)": 10,
    "Left B (physical 19)": 8,
    "Right A (physical 21)": 9,
    "Right B (physical 23)": 11,
}
 
devices = {}
 
def show(name):
    def callback():
        print(f"Transition detected: {name}")
    return callback
 
for name, pin in pins.items():
    dev = DigitalInputDevice(pin, pull_up=True)
    devices[name] = dev
    dev.when_activated = show(name)
    dev.when_deactivated = show(name)
 
print("Rotate ONLY the left wheel slowly.")
print("Then rotate ONLY the right wheel.")
print("Press Ctrl+C to stop.")
 
pause()

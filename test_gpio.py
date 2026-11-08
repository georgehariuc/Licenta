import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    GPIO.add_event_detect(
        17,
        GPIO.BOTH,
        callback=lambda channel: print("GPIO 17 changed")
    )
    print("Edge detection works on GPIO 17.")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
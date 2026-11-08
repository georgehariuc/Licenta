import time
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from TRSensors import TRSensor

comanda_maxima = 30
thresh = 300        # sensor <= thresh  ->  line detected (1)
Kp = 8          # start low, then tune
Kd = 100
eroare_ant = 0

Ab = AlphaBot2()
Tr = TRSensor()

turn_speed = 10
min_turn_time = 0.15


def calibrare():
    # calibrarea senzorilor IR: robotul se roteste stanga-dreapta peste linie
    for i in range(0, 100):
        if i < 25 or i >= 75:
            Ab.right()
        else:
            Ab.left()
        Ab.setPWMA(30)
        Ab.setPWMB(30)
        Tr.calibrate()

    Ab.stop()
    print(Tr.calibratedMin)
    print(Tr.calibratedMax)


def normalization(vect, thresh):
    # returneaza o lista noua de 0/1 (1 = linie)
    return [0 if v <= thresh else 1 for v in vect]


def turn(direction, timeout=3):
    # roteste pana cand senzorul din mijloc gaseste linia (sau expira timeout-ul)
    start = time.time()
    while time.time() - start < timeout:
        _, raw = Tr.readLine()
        if normalization(raw, thresh)[2] == 1:
            break
        if direction == "left":
            Ab.setPWMA(0)
            Ab.setPWMB(20)
            print("left!!")
        else:
            Ab.setPWMA(20)
            Ab.setPWMB(0)
            print("right!!")


def PD(position):
    global eroare_ant

    eroare = position - 2000
    er_D = eroare - eroare_ant
    eroare_ant = eroare   # salvare eroare anterioara

    # Legea de reglare PD in forma paralela
    regulator = Kp * eroare + Kd * er_D

    # mecanism antiwindup
    regulator = max(-comanda_maxima, min(comanda_maxima, regulator))

    # calcul viteza tinand cont de semnul comenzii
    if regulator < 0:
        Ab.setPWMA(comanda_maxima + regulator)
        Ab.setPWMB(comanda_maxima)
    else:
        Ab.setPWMA(comanda_maxima)
        Ab.setPWMB(comanda_maxima - regulator)


def alg():
    Ab.stop()
    print("Line follow Example")
    time.sleep(0.3)

    calibrare()
    Ab.forward()
    orientation = "straight"

    while True:
        # o singura citire pe iteratie
        position, raw = Tr.readLine()
        Sensors = normalization(raw, thresh)
        print(Sensors)

        # verificare viraj
        if Sensors == [1, 1, 1, 0, 0] or Sensors == [1, 1, 0, 0, 0] or Sensors == [1, 1, 1, 1, 0]:
            orientation = "left"
        elif Sensors == [0, 0, 1, 1, 1] or Sensors == [0, 0, 0, 1, 1] or Sensors == [0, 1, 1, 1, 1]:
            orientation = "right"
        elif Sensors == [0, 0, 1, 0, 0] or Sensors == [0, 0, 1, 1, 0] or Sensors == [0, 1, 1, 0, 0] or Sensors == [0, 1, 1, 1, 0]:
            orientation = "straight"   # centrat pe linie

        # linia a disparut complet
        if sum(Sensors) == 0 and orientation != "straight":
            time.sleep(0.149)
            turn(orientation)
            orientation = "straight"
            continue

        PD(position)


if __name__ == '__main__':
    try:
        alg()
    except KeyboardInterrupt:
        pass
    finally:
        Ab.stop()
        GPIO.cleanup()

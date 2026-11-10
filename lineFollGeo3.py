import time
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from TRSensors import TRSensor

# ---------------- Parametri ----------------
comanda_maxima = 35     # viteza de baza la urmarirea liniei
thresh = 300            # senzor > thresh -> linie detectata (1), vezi normalization()
Kp = 8               # eroarea e in intervalul +-2000, deci Kp trebuie sa fie mic
Kd = 100
eroare_ant = 0

pivot_speed = 18 
slow_speed = 12       # viteza la rotirea pe loc (daca se blocheaza, creste; daca depaseste linia, scade)
min_turn_time = 0.15    # timp minim de rotire inainte sa accepte "linia gasita"
advance_time = 0.03     # cat merge drept ca axa rotilor sa ajunga la colt (de reglat)

Ab = AlphaBot2()
Tr = TRSensor()


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
    # returneaza o lista noua de 0/1 (1 = linie, adica valoare > thresh)
    return [0 if v <= thresh else 1 for v in vect]


def turn(direction, timeout=3):
    # rotire pe loc (roti in sensuri opuse) pana cand senzorul din mijloc gaseste linia
    global eroare_ant

    Ab.setPWMA(pivot_speed)
    Ab.setPWMB(pivot_speed)
    if direction == "left":
        Ab.left()       # daca se roteste invers, schimba left() cu right()
    else:
        Ab.right()

    start = time.time()
    slowed = False
    while time.time() - start < timeout:
        elapsed = time.time() - start
        
        if elapsed > min_turn_time and not slowed:
            Ab.setPWMA(slow_speed)
            Ab.setPWMB(slow_speed)
            slowed = True
            
        _, raw = Tr.readLine()
        s = normalization(raw, thresh)
        
        if elapsed > min_turn_time and (s[1] == 1 or s[2] == 1 or s[3] == 1):
            break
        time.sleep(0.003)
        
    Ab.stop()
    time.sleep(0.05)
    Ab.forward()        # restaureaza directia inainte pentru bucla PD
    eroare_ant = 0      # reset eroare derivativa


def PD(position):
    global eroare_ant

    eroare = position - 2000
    er_D = eroare - eroare_ant
    eroare_ant = eroare   # salvare eroare anterioara

    # Legea de reglare PD in forma paralela
    regulator = Kp * eroare + Kd * er_D

    # limitare comanda (antiwindup)
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
        # print(Sensors)   # decomenteaza doar pentru debug, incetineste bucla

        # retine spre ce parte se indreapta linia
        if Sensors[0] == 1 and Sensors[4] == 0:
            orientation = "left"
        elif Sensors[4] == 1 and Sensors[0] == 0:
            orientation = "right"
        elif Sensors[2] == 1 and Sensors[0] == 0 and Sensors[4] == 0:
            orientation = "straight"   # centrat pe linie

        # linia a disparut complet dupa un colt
        if sum(Sensors) == 0 and orientation != "straight":
            Ab.setPWMA(comanda_maxima)   # ambele roti egale = merge drept
            Ab.setPWMB(comanda_maxima)
            time.sleep(advance_time)
            Ab.stop()
            time.sleep(0.05)
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


from turtle import position

from AlphaBot2 import AlphaBot2
from TRSensors import TRSensor
import time
import matplotlib.pyplot as plt

comanda_maxima = 30
thresh = 300
# Foarte bune 
Kp = 8
Kd = 100

Ab = AlphaBot2()
Tr = TRSensor()

#calibrare
def calibrare():
    #calibrarea senzorilor IR
    for i in range(0,100):
        if(i<25 or i>= 75):
            Ab.right()
            Ab.setPWMA(30)
            Ab.setPWMB(30)
        else:
            Ab.left()
            Ab.setPWMA(30)
            Ab.setPWMB(30)
        Tr.calibrate()
    Ab.stop()
    print(Tr.calibratedMin)
    print(Tr.calibratedMax)

def normalization(vect, thresh):
    for element in range(len(vect)):
        if vect[element] <= thresh:
            vect[element] = 1
        else:
            vect[element] = 0

def turn_left():
    _,sensors = Tr.readLine()
    normalization(sensors,thresh)
    while sensors[2] != 1:
        Ab.setPWMA(10)
        Ab.setPWMB(0)
        _,sensors = Tr.readLine()
        normalization(sensors,thresh)
    
    orientation = "straight"
    print("stangaaa!!!!")
    

def turn_right():
    _,sensors = Tr.readLine()
    normalization(sensors,thresh)
    while sensors[2] != 1:
        Ab.setPWMA(0)
        Ab.setPWMB(10)
        _,sensors = Tr.readLine()
        normalization(sensors,thresh)
    
    orientation = "straight"
    print("dreaptaa!!!!")

def PD():
    position,Sensors = Tr.readLine()
    eroare_ant = 0

    eroare = position - 2000

    er_D = eroare - eroare_ant
    # Salvare pozitie anterioara
    eroare_ant = eroare
    # Legea de reglare PD in forma paralela       
    regulator = Kp*eroare + Kd*er_D

    #mecanism antiwindup 
    if (regulator > comanda_maxima):
        regulator = comanda_maxima
    if (regulator < - comanda_maxima):
        regulator = - comanda_maxima

    #print(position)
    #mecanism de calculare al vitezei tinand cont de semnul comenzii
    if (regulator < 0):
        Ab.setPWMA(comanda_maxima + regulator)
        Ab.setPWMB(comanda_maxima)
    else:
        Ab.setPWMA(comanda_maxima)
        Ab.setPWMB(comanda_maxima - regulator)


def alg():
    Ab.stop()
    print("Line follow Example")
    time.sleep(2)

    #functie calibrare
    calibrare()

   # Ab.forward()
    orientation = "straight"
    while True:
        #citire pozitie curenta senzori
        position,Sensors = Tr.readLine()
        
        normalization(Sensors, thresh)

        #verificare viraj
        if(Sensors[0] == 1 and Sensors[1] == 1 and Sensors[2] == 1 and Sensors[3] == 0 and Sensors[4] == 0):
            orientation = "right"
        elif(Sensors[0] == 1 and Sensors[1] == 1 and Sensors[2] == 0 and Sensors[3] == 0 and Sensors[4] == 0):
            orientation = "right"
        elif(Sensors[0] == 0 and Sensors[1] == 0 and Sensors[2] == 1 and Sensors[3] == 1 and Sensors[4] == 1):
            orientation = "left"
        elif(Sensors[0] == 0 and Sensors[1] == 0 and Sensors[2] == 0 and Sensors[3] == 1 and Sensors[4] == 1):
            orientation = "left"
       

        if(Sensors[0] != 1 and Sensors[1] != 1 and Sensors[2] != 1 and Sensors[3] != 1 and Sensors[4] != 1):
                    if orientation != "straight":
                        time.sleep(0.5)
                        if orientation == "left":
                           turn_left()
                        else:
                            turn_right()
        print(orientation)
                            
						#orientation = "straight"
       # PD()
 
if __name__=='__main__':
    
    try:
        alg()
    except KeyboardInterrupt:
        Ab.stop()
        GPIO.cleanup()
        

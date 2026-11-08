
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import sys
sys.path.append('..')
from AlphaBot2 import AlphaBot2
Ab = AlphaBot2()
Ab.forward()
comanda_maxima = 10

# Foarte bune
eroare_ant = 0
Kp = 8
Kd = 100
cx = 0
rawCapture = 0
camera = 0
mask = 0

# Initializare  camera prin setarea rezolutiei si frecventei

def initializare():
    camera = picamera.PiCamera()
    camera.resolution = (160, 120)
    camera.framerate = 20
    rawCapture = PiRGBArray(camera,size=(160, 120))
    time.sleep(0.1)

# Se capteaza continuu cadrele de la camera

def procesare():

    initializare()

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        # Se extrage imaginea din cadrul curent si se afiseaza 
        image = frame.array
        cv2.imshow('img',image)
        key = cv2.waitKey(1) & 0xFF

        # Conversia imaginii din BGR in GRAY
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Blurarea imagini utilizand un filtru Gaussian
        blur = cv2.GaussianBlur(gray,(5,5),0)
        # Binarizarea imaginii filtrate
        ret,thresh1 = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)
        # Se aplica operatia de erodare pentru a reduce zgomotul 
        mask = cv2.erode(thresh1, None, iterations=2)
        # Se aplica operatia de dilatare pentru a restaura partile erodate din imagine
        mask = cv2.dilate(mask, None, iterations=2)
        # Extragerea tuturor contururilor din imaginea binara
        contours, hierarchy = cv2.findContours(mask.copy(),1,cv2.CHAIN_APPROX_NONE)
        # Se determina cel mai mare contur
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            #functia returneaza un dictionar cu momentele calculate
            M = cv2.moments(c)

            # Se determina coordonata pe x a centrului 
            return int(M['m10']/M['m00'])

def alg():	
 # Robotul se misca in functie de pozitia camerei fata de centrul linie determinat anterior
        #Opreste robotul daca nu este detectata nici o linie   
        if(cx >250 or cx < 5):
            Ab.setPWMA(0)
            Ab.setPWMB(0)
        else:
            #calculeaza eroarea de la intrarea regulatorului
            eroare = cx - 80     
            er_D = eroare - eroare_ant
            eroare_ant = eroare
            #calculaza comanda utilizand un regulator in forma paralela
            regulator = Kp*eroare + Kd*er_D 
            print("Comanda regulator: ", regulator)
            print("Eroare: ", eroare)
            if (regulator > comanda_maxima):
                regulator = comanda_maxima
            if (regulator < - comanda_maxima):
                regulator = - comanda_maxima
            print("centru linie ",cx)
            if (regulator < 0):
                Ab.setPWMA(comanda_maxima + regulator)
                Ab.setPWMB(comanda_maxima)
            else:
                Ab.setPWMA(comanda_maxima)
                Ab.setPWMB(comanda_maxima - regulator)
            print("Comanda regulator: ", regulator)
    
def exit():
    rawCapture.truncate(0)
    cv2.imshow("Mask", mask)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        return
    camera.release()
    cv2.destroyAllWindows()
    GPIO.output(12, GPIO.LOW)
    GPIO.output(16, GPIO.LOW)
    GPIO.output(20, GPIO.LOW)
    GPIO.output(21, GPIO.LOW)
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        while True:
            procesare()
            alg()
    except KeyboardInterrupt:
        print("Programul a fost oprit de utilizator.")
    finally: 
        exit()
    

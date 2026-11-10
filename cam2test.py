import time
import cv2
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from AlphaBot2 import AlphaBot2   # daca modulul tau se numeste altfel (ex: initializare), schimba aici

# ---------------- Parametri ----------------
comanda_maxima = 20   # viteza de baza pe drum drept (0-100)
Kp = 0.15
Kd = 0.3
steer = -1             # pune -1 daca robotul vireaza DEPARTE de linie

# Incetinire in viraje
viteza_min = 12       # viteza minima in viraje
slow_error = 0.10     # cat scade viteza pentru fiecare pixel de eroare
slow_curve = 0.15     # cat scade viteza pentru fiecare pixel de diferenta intre banda departe si banda aproape

# Procesare imagine (imagine 160x120)
roi_top = 40          # se foloseste doar zona de jos a imaginii (de la randul 40)
split_row = 70        # randurile 40-70 = banda "departe", 70-120 = banda "aproape"
use_otsu = True       # prag automat; pune False ca sa folosesti 'prag'
prag = 100            # folosit doar daca use_otsu = False
contrast_min = 40     # diferenta minima intre pixelul cel mai inchis si cel mai deschis
min_area = 30         # contururi mai mici decat atat sunt zgomot

viteza_cautare = 10   # viteza de rotatie cand linia e pierduta
debug = False         # True = afiseaza valorile de gri si numarul de pixeli de linie
show = False           # False daca rulezi fara ecran (SSH)

Ab = AlphaBot2()


def init_camera():
    cam = Picamera2()
    cfg = cam.create_preview_configuration(main={"size": (160, 120), "format": "RGB888"})
    cam.configure(cfg)
    cam.start()
    time.sleep(0.5)
    return cam


def centroid(mask):
    # coordonata x a centrului celui mai mare contur, sau None
    contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"])


def find_line(image):
    # returneaza (cx_aproape, cx_departe, mask); fiecare cx poate fi None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    roi = blur[roi_top:, :]

    if debug:
        print("gray min/max/mean:", roi.min(), roi.max(), int(roi.mean()))

    # contrast prea mic = nu exista linie in imagine
    if int(roi.max()) - int(roi.min()) < contrast_min:
        return None, None, roi

    if use_otsu:
        _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, binary = cv2.threshold(roi, prag, 255, cv2.THRESH_BINARY_INV)

    mask = cv2.erode(binary, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    if debug:
        print("pixeli linie:", cv2.countNonZero(mask))

    split = split_row - roi_top
    cx_departe = centroid(mask[:split])
    cx_aproape = centroid(mask[split:])
    return cx_aproape, cx_departe, mask


def drive(eroare, eroare_ant, curba):
    er_D = eroare - eroare_ant

    # viteza scade cand eroarea e mare sau cand urmeaza o curba
    viteza = comanda_maxima - slow_error * abs(eroare) - slow_curve * abs(curba)
    viteza = max(viteza_min, viteza)

    regulator = steer * (Kp * eroare + Kd * er_D)
    regulator = max(-viteza, min(viteza, regulator))

    # in biblioteca: A = roata dreapta, B = roata stanga
    # eroare > 0 = linia e in dreapta -> incetinim roata dreapta (A)
    if regulator < 0:
        Ab.setPWMA(viteza)
        Ab.setPWMB(viteza + regulator)
    else:
        Ab.setPWMA(viteza - regulator)
        Ab.setPWMB(viteza)
    return regulator, viteza


def main():
    cam = init_camera()
    eroare_ant = None   # None = nu avem eroare anterioara (evita saltul derivativei)
    ultima_eroare = 0
    cautand = False
    Ab.forward()   # directia de mers trebuie setata, setPWM schimba doar viteza
	
    try:
        while True:
			
            image = cam.capture_array()
            cx_aproape, cx_departe, mask = find_line(image)

            # banda apropiata comanda directia; daca lipseste, folosim banda departata
            cx = cx_aproape if cx_aproape is not None else cx_departe

            if cx is None:
                # linia pierduta: se roteste in loc spre ultima parte unde a fost vazuta
                print("Linie pierduta")
                if ultima_eroare * steer > 0:
                    Ab.right()
                else:
                    Ab.left()
                Ab.setPWMA(viteza_cautare)
                Ab.setPWMB(viteza_cautare)
                cautand = True
                eroare_ant = None
            else:
                if cautand:
                    Ab.forward()   # linia a fost regasita, revenim la mers inainte
                    cautand = False

                eroare = cx - 80   # 80 = centrul imaginii de 160 px
                if eroare_ant is None:
                    eroare_ant = eroare   # primul cadru: derivata = 0

                # cat de mult se curbeaza linia in fata robotului
                if cx_aproape is not None and cx_departe is not None:
                    curba = cx_departe - cx_aproape
                else:
                    curba = 0

                regulator, viteza = drive(eroare, eroare_ant, curba)
                eroare_ant = eroare
                ultima_eroare = eroare
                print("aproape:", cx_aproape, "departe:", cx_departe,
                      "eroare:", eroare, "curba:", curba,
                      "regulator:", round(regulator, 1), "viteza:", round(viteza, 1))

            if show:
                cv2.imshow("img", image)
                cv2.imshow("Mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("Programul a fost oprit de utilizator.")
    finally:
        Ab.stop()
        cam.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()


if __name__ == "__main__":
    main()

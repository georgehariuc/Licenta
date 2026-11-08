import time
import cv2
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from AlphaBot2 import AlphaBot2   # daca modulul tau se numeste altfel (ex: initializare), schimba aici

# ---------------- Parametri ----------------
comanda_maxima = 20   # viteza de baza (0-100)
Kp = 8                # eroarea e in pixeli (+-80), deci castigurile sunt mici
Kd = 100
steer = 1             # pune -1 daca robotul vireaza DEPARTE de linie
prag = 100            # prag de binarizare (linie inchisa la culoare)
roi_top = 60          # se foloseste doar partea de jos a imaginii (randurile 60-120)
min_area = 30         # contururi mai mici decat atat sunt considerate zgomot
viteza_cautare = 15   # viteza de rotatie cand linia e pierduta
use_otsu = True       # prag automat; pune False ca sa folosesti 'prag' de mai sus
contrast_min = 40     # diferenta minima intre pixelul cel mai inchis si cel mai deschis
debug = True          # afiseaza valorile de gri si numarul de pixeli de linie
show = True           # False daca rulezi fara ecran (SSH)

Ab = AlphaBot2()


def init_camera():
    cam = Picamera2()
    cfg = cam.create_preview_configuration(main={"size": (160, 120), "format": "RGB888"})
    cam.configure(cfg)
    cam.start()
    time.sleep(0.5)
    return cam


def find_line(image):
    # returneaza (cx, mask); cx = None daca nu s-a gasit linia
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    roi = blur[roi_top:, :]

    if debug:
        print("gray min/max/mean:", roi.min(), roi.max(), int(roi.mean()))

    # contrast prea mic = nu exista linie in imagine (evita zgomot cu Otsu)
    if int(roi.max()) - int(roi.min()) < contrast_min:
        return None, roi

    if use_otsu:
        # pragul se alege automat, nu depinde de lumina
        _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, binary = cv2.threshold(roi, prag, 255, cv2.THRESH_BINARY_INV)

    mask = cv2.erode(binary, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    if debug:
        print("pixeli linie:", cv2.countNonZero(mask))

    contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None, mask

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask
    return int(M["m10"] / M["m00"]), mask


def drive(eroare, eroare_ant):
    er_D = eroare - eroare_ant
    regulator = steer * (Kp * eroare + Kd * er_D)
    regulator = max(-comanda_maxima, min(comanda_maxima, regulator))

    # in biblioteca: A = roata dreapta, B = roata stanga
    # eroare > 0 = linia e in dreapta -> incetinim roata dreapta (A)
    if regulator < 0:
        Ab.setPWMA(comanda_maxima)
        Ab.setPWMB(comanda_maxima + regulator)
    else:
        Ab.setPWMA(comanda_maxima - regulator)
        Ab.setPWMB(comanda_maxima)
    return regulator


def main():
    cam = init_camera()
    eroare_ant = 0
    ultima_eroare = 0
    cautand = False
    Ab.forward()   # directia de mers trebuie setata, setPWM schimba doar viteza

    try:
        while True:
            image = cam.capture_array()
            cx, mask = find_line(image)

            if cx is None:
                # linia pierduta: se roteste in loc spre ultima parte unde a fost vazuta
                print("Linie pierduta")
                if ultima_eroare * steer > 0:
                    Ab.setPWMA(0)
                    Ab.setPWMB(viteza_cautare)
                else:
                    Ab.setPWMA(viteza_cautare)
                    Ab.setPWMB(0)
                
                cautand = True
            else:
                if cautand:
                    Ab.forward()   # linia a fost regasita, revenim la mers inainte
                    cautand = False
                eroare = cx - 80   # 80 = centrul imaginii de 160 px
                regulator = drive(eroare, eroare_ant)
                eroare_ant = eroare
                ultima_eroare = eroare
                print("cx:", cx, "eroare:", eroare, "regulator:", round(regulator, 1))

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

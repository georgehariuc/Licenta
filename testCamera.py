from picamera2 import Picamera2
from AlphaBot2 import AlphaBot2
import cv2
import time

Ab = AlphaBot2()

# PID controller tuning values (adjust these based on testing)
Kp = 0.5 
Kd = 0.2
comanda_maxima = 100
eroare_ant = 0 

# Declare our global camera variable so we can use it anywhere
picam2 = None 

def initCamera():
    global picam2
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size" : (160, 120)})
    picam2.configure(config)
    picam2.start()

def video():
    global eroare_ant # Tell Python we want to modify the global variable
    
    # Wait a moment for the camera sensor to warm up and adjust to light
    time.sleep(3) 
    
    while True:
        try:
            # 1. Capture frame directly using Picamera2
            frame = picam2.capture_array()
            
            # 2. Convert RGB to BGR (OpenCV standard)
            image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # 3. Image Processing
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5,5), 0)
            ret, thresh1 = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
            
            mask = cv2.erode(thresh1, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
            
            cx = 0
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M['m00'] != 0: # Prevent divide-by-zero error if contour is tiny
                    cx = int(M['m10']/M['m00'])

            # Draw a circle on the calculated center for debugging
            cv2.circle(image, (cx, 60), 5, (0, 0, 255), -1)
            cv2.imshow('Robot View', image)

            # 4. Motor Control
            if (cx > 250 or cx < 5):
                # We lost the line (Note: max cx is 160 based on your resolution, so > 250 is never true)
                pass # Replace with: Ab.setPWMA(0); Ab.setPWMB(0)
            else:
                eroare = cx - 80     
                er_D = eroare - eroare_ant
                eroare_ant = eroare
                
                regulator = (Kp * eroare) + (Kd * er_D)
                
                if (regulator > comanda_maxima):
                    regulator = comanda_maxima
                if (regulator < -comanda_maxima):
                    regulator = -comanda_maxima
                    
                if (regulator < 0):
                    Ab.setPWMA(comanda_maxima + regulator); 
                    Ab.setPWMB(comanda_maxima)
                else:
                    Ab.setPWMA(comanda_maxima);
                    Ab.setPWMB(comanda_maxima - regulator)

            # Quit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Error: {e}") # Fixed string formatting
            break

if __name__ == "__main__":
    initCamera()
    video()
    
    # Safe cleanup after the loop breaks
    if picam2 is not None:
        picam2.stop()
    cv2.destroyAllWindows()

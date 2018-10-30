#first run: sudo modprobe bcm2835-v4l2

import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# cap.set(CAP_PROP_FRAME_WIDTH, 640)
# cap.set(CAP_PROP_FRAME_HEIGHT, 480)
<<<<<<< HEAD:openCV.py

WIDTH = 1280
HEIGHT = 960

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # size of gray 
    # len(gray) * len(gray[0])
    
    frameOut = frame.copy()
    frameOut = cv2.resize(frame, (WIDTH, HEIGHT))
    frameOut[:,int(WIDTH/2)] = [0,0,255]
    frameOut[int(HEIGHT/2), :] = [0,0,255]
    cv2.rectangle(frameOut, (int(WIDTH/4), int(HEIGHT/4)), (int(WIDTH/4*3), int(HEIGHT/4*3)), [0,255,0], 1, 8, 0)
    #frameOut = cv2.flip(frame, 0)

#    size, shape->zwaraca liczbe wierszy i kolumn shape(row, col) zwraca

    # Display the resulting frame
    cv2.imshow('frame',frameOut)
    #cv2.imshow('gray',gray)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# numpy indeksowanie itp. 
# jak narysować coś kolorowego na obrazie szarym i jak to obejść?
# wyszukiwanie rzeczy kolorowych na szarym obrazie
# wykrywanie rzeczy o określonym kolorze/kształcie
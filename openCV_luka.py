#first run: sudo modprobe bcm2835-v4l2

import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# cap.set(CAP_PROP_FRAME_WIDTH, 640)
# cap.set(CAP_PROP_FRAME_HEIGHT, 480)
basicWidth = 640
basicHeight = 480
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # size of gray 
    # len(gray) * len(gray[0])
    
    frameOut = frame.copy()
    frameOut = cv2.flip(frame, 0)
    # frameOut = cv2.resize(frame, (200, 160))

    frameOut[:, int(basicWidth / 2)] = [255, 0, 0]
    frameOut[int(basicHeight / 2), :] = [255, 0, 0]



#    size, shape->zwaraca liczbe wierszy i kolumn shape(row, col) zwraca

    # Display the resulting frame
    cv2.imshow('frame',frameOut)
    cv2.imshow('gray',gray)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


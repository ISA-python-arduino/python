import time 
from imutils.video import VideoStream
import serial
import numpy as np
import cv2
import random as rng
import math

usesPiCamera = False
cameraResolution = (640, 480)

vs = VideoStream(usePiCamera=usesPiCamera, resolution=cameraResolution, framerate=60).start()
time.sleep(2.0)

blueLower = (0, 100, 50)
blueUpper = (100, 255, 255)
colorTolerance = 10
roiSize = (6, 6)
Done = False
minPointsInCount = 15

while not Done:
    frame = vs.read()
    height, width = frame.shape[0:2]
    scaleFactor = 4
    newWidth, newHeight = width//scaleFactor, height//scaleFactor
    
    resizedColor = cv2.resize(frame, (newWidth, newHeight), interpolation=cv2.INTER_LINEAR)
    resizedColor_blurred = cv2.GaussianBlur(resizedColor, (5, 5), 0)
    resizedHSV = cv2.cvtColor(resizedColor_blurred, cv2.COLOR_BGR2HSV)
    roi = resizedHSV[newHeight//2 - roiSize[0]//2 : newHeight //2 + roiSize[0]//2, newWidth//2 - roiSize[1]//2 : newWidth//2 + roiSize[1]//2, :]

    blueLowerWithTolerance = (blueLower[0] - colorTolerance,) + blueLower[1:]
    blueUpperWithTolerance = (blueUpper[0] + colorTolerance,) + blueUpper[1:]

    # ret, threshold = cv2.threshold(h, 225, 255, cv2.THRESH_BINARY)

    mask = cv2.inRange(resizedHSV, blueLowerWithTolerance, blueUpperWithTolerance)
    cv2.erode(mask, None, iterations=5)
    cv2.dilate(mask, None, iterations=5)
    res = cv2.bitwise_and(resizedColor, resizedColor, mask = mask)

    filteredContours = []
    boundingBoxes = []
    (_,contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if contours: 
        for i, contour in enumerate(contours):
            if len(contour) >= minPointsInCount:
                contourArea = cv2.contourArea(contour)
                M = cv2.moments(contour)
                n02 = M['nu02']
                n20 = M['nu20']
                m01 = M['m01']
                m10 = M['m10']
                momentArea = M['m00']
                momentR = math.sqrt(momentArea / 3.141592)
                maxMoment = max(n02, n20)
                minMoment = min(n02, n20)
                if minMoment > 0 and momentArea > 0:
                    posX = m01 / momentArea
                    posY = m10 / momentArea
                    if (maxMoment / minMoment) > 0.5 and (maxMoment / minMoment) < 1.5:
                        contour2D = contour.reshape(-1,2)
                        x,y,w,h = cv2.boundingRect(contour)
                        boundingBoxes.append((x,y,w,h))
                        filteredContours.append(contour2D)

    upscaledColor = cv2.resize(resizedColor, (width, height), interpolation=cv2.INTER_NEAREST)
    xROI, yROI = width//2 - roiSize[1]//2 * scaleFactor, height//2 - roiSize[0]//2 * scaleFactor
    cv2.rectangle(upscaledColor, (xROI, yROI), (xROI + roiSize[0]*scaleFactor, yROI + roiSize[1]*scaleFactor), (0, 0, 0), thickness=3)

    for contour2D in filteredContours: 
        for (x, y) in contour2D:
            cv2.circle(upscaledColor, (x*scaleFactor, y*scaleFactor), 1, (255, 0, 0), 3)

    for boundingBox in boundingBoxes:
        x,y,w,h = boundingBox
        cv2.rectangle(resizedColor, (x, y), (x+w, y+h), (255, 255, 0), thickness=1)
        cv2.rectangle(upscaledColor, (x*scaleFactor, y*scaleFactor), ((x+w)*scaleFactor, (y+h)*scaleFactor), (255, 255, 0), thickness=2)

    cv2.imshow("Res", res)
    cv2.imshow("Mask", mask)
    cv2.imshow("Roi", roi)
    cv2.imshow("HSV", resizedHSV)
    cv2.imshow("UpscaledColor", upscaledColor)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        Done = True
    elif key == ord('a'):
        avg_h = 0
        avg_s = 0
        avg_v = 0
        i = 0
        for _, row in enumerate(roi):
            avg = np.average(row, 0)
            avg_h += avg[0]
            avg_s += avg[1]
            avg_v += avg[2]
            i+=1

        avg_h /= i
        avg_s /= i
        avg_v /= i
        print("HUE:{}, SAT:{}, VAL:{}".format(avg_h, avg_s, avg_v))
        blueLower = (max(0,avg_h), max(0, avg_s - 50), max(0,avg_v - 50))
        blueUpper = (min (255, avg_h), min(255, avg_s + 50), min(255, avg_v + 50))
    elif key == ord('w'):
        colorTolerance = min(colorTolerance + 1, 100)
        print("New color range: {}".format(colorTolerance))
    elif key == ord('s'):
        colorTolerance = max(colorTolerance - 1, 0)
        print("New color range: {}".format(colorTolerance))

cv2.destroyAllWindows()
vs.stop()
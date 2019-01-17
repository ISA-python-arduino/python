import time 
from imutils.video import VideoStream
import numpy as np
import cv2
import random as rng
import math
# import serial
withArduino = False

def translate(value, oldMin, oldMax, newMin=-100, newMax=100):
    oldRange = oldMax - oldMin
    newRange = newMax - newMin
    NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return int(NewValue)

def checkMinimumPointsInTorus(contour):
    (x,y),radius = cv2.minEnclosingCircle(contour)
    center = (int(x),int(y))
    radius = int(radius)
    pointsInTorus = 0
    for point in contour:
        lenght = math.sqrt((point[0][0] - x)*(point[0][0] - x) + (point[0][1] - y) * (point[0][1] - y))
        if lenght > radius * 0.9:
            pointsInTorus += 1
    percent = pointsInTorus/len(contour)
    print("Procent punkow w torusie: ", percent)
    if (percent > 0.6):
        return True
    return False


usesPiCamera = withArduino
if withArduino:
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=0.05)
sendStop = 0.3
stopMessage = time.process_time()
cameraResolution = (640, 480)
vs = VideoStream(usePiCamera=usesPiCamera, resolution=cameraResolution, framerate=60).start()
time.sleep(2.0)


blueLower = (20, 150, 100)
blueUpper = (33, 255, 255)

if not withArduino: 
    blueLower = (40, 100, 50)
    blueUpper = (80, 255, 255)

colorTolerance = 10
paused = False
roiSize = (6, 6)

while True:
    if not paused:
        frame = vs.read()
        if withArduino:
            frame = cv2.flip(frame, flipCode=-1)
        
        height, width = frame.shape[0:2]
        scaleFactor = 4
        newWidth, newHeight = width//scaleFactor, height//scaleFactor
        kernel = (9,9)
        resizedColor = cv2.resize(frame, (newWidth, newHeight), interpolation=cv2.INTER_LINEAR)
        resizedColor_blurred = cv2.GaussianBlur(resizedColor, kernel, 0)
        resizedHSV = cv2.cvtColor(resizedColor_blurred, cv2.COLOR_BGR2HSV)
        roi = resizedHSV[newHeight//2 - roiSize[0]//2 : newHeight //2 + roiSize[0]//2, newWidth//2 - roiSize[1]//2 : newWidth//2 + roiSize[1]//2, :]
        
        blueLowerWithTolerance = (blueLower[0] - colorTolerance,) + blueLower[1:]
        blueUpperWithTolerance = (blueUpper[0] + colorTolerance,) + blueUpper[1:]
		
        mask = cv2.inRange(resizedHSV, blueLowerWithTolerance, blueUpperWithTolerance)

        cv2.erode(mask, kernel, iterations=5)
        cv2.dilate(mask, kernel, iterations=5)
        cv2.dilate(mask, kernel, iterations=5)
        cv2.erode(mask, kernel, iterations=5)
        (_,contours, hierarchy) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        boundingBoxes = []
        biggestObject_BoundingBox = None
        biggestObjectMiddle = None
        filteredContours = []
        minPointsInCount = 15
        if contours:            
            for i, contour in enumerate(contours):
                if len(contour) >= minPointsInCount:
                    M = cv2.moments(contour)
                    n02 = M['nu02']
                    n20 = M['nu20']
                    m01 = M['m01'] 
                    m10 = M['m10']
                    momentArea = M['m00']
                    momentR = math.sqrt(momentArea / 3.141592)
                    maxMoment = max(n02, n20)
                    minMoment = min(n02, n20)
                    if momentR > 0 and momentArea > 0:
                        tempMoment = (maxMoment / minMoment) - 1
                        tempMoment = tempMoment * tempMoment
                        tempMoment = tempMoment / (1 - M['nu11']) * (1 - M['nu11'])
                        if  tempMoment > 0.0 and tempMoment < 0.05:
                            # print("tempMoment: ", tempMoment)
                            posX = m10 / momentArea
                            posY = m01 / momentArea
                            if posX > 0 and posY > 0 and checkMinimumPointsInTorus(contour):
                                x,y,w,h = cv2.boundingRect(contour)
                                boundingBoxes.append((x,y,w,h))
                                filteredContours.append(contour)
        else:
            pass

        upscaledColor = cv2.resize(resizedColor, (width, height), interpolation=cv2.INTER_NEAREST)
        xROI, yROI = width//2 - roiSize[1]//2 * scaleFactor, height//2 - roiSize[0]//2 * scaleFactor
        cv2.rectangle(upscaledColor, (xROI, yROI), (xROI + roiSize[0]*scaleFactor, yROI + roiSize[1]*scaleFactor), (0, 0, 0), thickness=3)
		
        if boundingBoxes:  
            largestContour = max(filteredContours, key=cv2.contourArea)
            biggestObject_BoundingBox = cv2.boundingRect(largestContour) 
            cv2.drawContours(upscaledColor, [largestContour * [scaleFactor, scaleFactor]], -1, (255,0,0), thickness=3)
            (x,y),radius = cv2.minEnclosingCircle(largestContour * [scaleFactor, scaleFactor])
            center = (int(x),int(y))
            radius = int(radius *0.9)
            cv2.circle(upscaledColor, center, radius, (0,255,255), thickness=1)
            cv2.circle(upscaledColor, center, radius, (0,0,255), thickness=1)
            
        packet = None
		
        if biggestObject_BoundingBox:
            x, y, w, h = biggestObject_BoundingBox
            biggestObjectMiddle = ((x+ w//2)*scaleFactor, (y + h//2)*scaleFactor)
            screenMiddle = width//2, height//2
            distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMiddle, screenMiddle))
            scaled = (translate(distanceVector[0], -width//2, width//2), translate(distanceVector[1], -height//2, height//2) )
            cv2.line(upscaledColor, screenMiddle, biggestObjectMiddle, (0, 0, 255))
            packet = '<packet, {}, {}>'.format(scaled[0], scaled[1])
            packetBytes = bytes(packet, 'utf-8')
            stopMessage = time.process_time()
            if withArduino:
                ser.write(packetBytes)
            else:
                cv2.circle(upscaledColor, biggestObjectMiddle, 2, (255, 0, 0), thickness=2)
                cv2.rectangle(resizedColor, (x, y), (x+w, y+h), (0, 0, 255), thickness=2)
            
        else:
            currentTime = time.process_time()
            if (currentTime - stopMessage)  > sendStop:
                packet = '<stop, {}, {}>'.format(int(0), int(0))
                packetBytes = bytes(packet, 'utf-8')
                stopMessage = currentTime
                if withArduino:
                    ser.write(packetBytes)
					
        if not withArduino:
            #if packet is not None:
            #   print(packet)
            cv2.imshow("video", upscaledColor)
            cv2.imshow("roi", roi)
            cv2.imshow("mask", mask)
			
        modTolerances = False
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
cv2.destroyAllWindows()
vs.stop()
import time 
from imutils.video import VideoStream
import numpy as np
import cv2
import random as rng
import math
import serial
ARDUINO_MODE = False
DEBUG = True
USE_PI_CAMERA = ARDUINO_MODE
CAMERA_RESOLUTION = (640, 480)
PERCENT_IN_TORUS_TO_ACCEPT = 0.6    # Określa jaki procent punktów musi się znajdować w torusie aby uznać kształt za okrąg
SEND_STOP_TIME = 1.0                # Czas po jakim wysyłana jest komenda stopu do Arduino
SCALE_FACTOR = 4
KERNEL = (9, 9)
MINIMUM_POINTS_IN_CONTOUR = 15      # Minimalna ilość punktów w konturze potrzebnych do podjęcia próby wykrywania okręgów
MINIMUM_MOMENT = 0.05               # Minimalna wyliczona wartość ze wzoru aby uznać kontur za okrąg
INNER_RADIUS_MULTIPLIER = 0.9       # Promień wewnętrznego okrąg w toruse

stopMessage = time.process_time()

vs = VideoStream(usePiCamera=USE_PI_CAMERA, resolution=CAMERA_RESOLUTION, framerate=60).start()

LOWER_COLOR = (20, 150, 100)
UPPER_COLOR = (33, 255, 255)
if not ARDUINO_MODE:
    LOWER_COLOR = (40, 100, 50)
    UPPER_COLOR = (80, 255, 255)

roiSize = (6, 6)

if ARDUINO_MODE:
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=0.05)

def debug(message):
    if DEBUG:
        print("Debug: ", message)

def translate(value, oldMin, oldMax, newMin=-100, newMax=100):
    oldRange = oldMax - oldMin
    newRange = newMax - newMin
    NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
    return int(NewValue)

def checkIfCircleMomentsMethod(contour):
    M = cv2.moments(contour)
    n02 = M['nu02']
    n20 = M['nu20']
    m01 = M['m01'] 
    m10 = M['m10']
    momentArea = M['m00']
    momentR = math.sqrt(momentArea / math.pi)
    maxMoment = max(n02, n20)
    minMoment = min(n02, n20)
    if momentR > 0 and momentArea > 0:
        tempMoment = (maxMoment / minMoment) - 1.0
        tempMoment = tempMoment * tempMoment
        tempMoment = tempMoment / (1.0 - M['nu11']) * (1.0 - M['nu11'])
        if  tempMoment > 0.0 and tempMoment < MINIMUM_MOMENT:
            debug('Moment: {}'.format(tempMoment))
            posX = m10 / momentArea
            posY = m01 / momentArea
            return (posX > 0 and posY > 0)
    return False

def checkMinimumPointsInTorus(contour):
    (x,y),radius = cv2.minEnclosingCircle(contour)
    center = (int(x),int(y))
    radius = int(radius)
    pointsInTorus = 0
    for point in contour:
        lenght = math.sqrt((point[0][0] - x)*(point[0][0] - x) + (point[0][1] - y) * (point[0][1] - y))
        if lenght > radius * INNER_RADIUS_MULTIPLIER:
            pointsInTorus += 1
    percent = pointsInTorus/len(contour)
    debug("Procent punkow w torusie: {}%".format(percent*100.0))
    if (percent > PERCENT_IN_TORUS_TO_ACCEPT):
        return True
    return False

def sendPacket(message, xCord, yCord):
    packet = '<{}, {}, {}>'.format(message, xCord, yCord)
    debug("Packet: {}".format(packet))
    packetBytes = bytes(packet, 'utf-8')
    if ARDUINO_MODE:
        ser.write(packetBytes)

def makeMask(image, lowerColor, upperColor):
    mask = cv2.inRange(image, LOWER_COLOR, UPPER_COLOR)
    cv2.erode(mask, KERNEL, iterations=5)
    cv2.dilate(mask, KERNEL, iterations=5)
    cv2.dilate(mask, KERNEL, iterations=5)
    cv2.erode(mask, KERNEL, iterations=5)
    return mask

def stopCommand(stopMessage):
    currentTime = time.process_time()
    timeDiff = currentTime - stopMessage
    if timeDiff > SEND_STOP_TIME:
        sendPacket('stop', int(0), int(0))
        debug('Process time difference: {}'.format(timeDiff))
        return currentTime
    return stopMessage

def roi(image):
    return image[newHeight//2 - roiSize[0]//2 : newHeight //2 + roiSize[0]//2, newWidth//2 - roiSize[1]//2 : newWidth//2 + roiSize[1]//2, :]

def quit(): 
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        return True
    return False

time.sleep(2.0)

while not quit():
    frame = vs.read()
    if ARDUINO_MODE:
        frame = cv2.flip(frame, flipCode=-1)
    
    height, width = frame.shape[0:2]
    newWidth, newHeight = width//SCALE_FACTOR, height//SCALE_FACTOR

    resizedColor = cv2.resize(frame, (newWidth, newHeight), interpolation=cv2.INTER_LINEAR)
    resizedColor_blurred = cv2.GaussianBlur(resizedColor, KERNEL, 0)
    resizedHSV = cv2.cvtColor(resizedColor_blurred, cv2.COLOR_BGR2HSV)
    
    
    mask = makeMask(resizedHSV, LOWER_COLOR, UPPER_COLOR)
    (_,contours, hierarchy) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    filteredContours = []
    if contours:            
        for i, contour in enumerate(contours):
            if len(contour) >= MINIMUM_POINTS_IN_CONTOUR:
                if checkIfCircleMomentsMethod(contour) and checkMinimumPointsInTorus(contour):
                    filteredContours.append(contour)
    else:
        pass

    upscaledColor = cv2.resize(resizedColor, (width, height), interpolation=cv2.INTER_NEAREST)
    biggestObject_BoundingBox = None

    if filteredContours: 
        largestContour = max(filteredContours, key=cv2.contourArea)
        biggestObject_BoundingBox = cv2.boundingRect(largestContour) 
        if not ARDUINO_MODE:
            cv2.drawContours(upscaledColor, [largestContour * [SCALE_FACTOR, SCALE_FACTOR]], -1, (255,0,0), thickness=3)
            (x,y),radius = cv2.minEnclosingCircle(largestContour * [SCALE_FACTOR, SCALE_FACTOR])
            center = (int(x),int(y))
            radius = int(radius*INNER_RADIUS_MULTIPLIER)
            cv2.circle(upscaledColor, center, radius, (0,255,255), thickness=1)
            cv2.circle(upscaledColor, center, radius, (0,0,255), thickness=1)
                
    if biggestObject_BoundingBox:
        x, y, w, h = biggestObject_BoundingBox
        biggestObjectMiddle = ((x+ w//2)*SCALE_FACTOR, (y + h//2)*SCALE_FACTOR)
        screenMiddle = width//2, height//2
        distanceVector = tuple(map(lambda x, y: x - y, biggestObjectMiddle, screenMiddle))
        scaled = (translate(distanceVector[0], -width//2, width//2), translate(distanceVector[1], -height//2, height//2) )
        cv2.line(upscaledColor, screenMiddle, biggestObjectMiddle, (0, 0, 255))

        sendPacket('packet', scaled[0], scaled[1])
        stopMessage = time.process_time()
        if not ARDUINO_MODE:
            cv2.circle(upscaledColor, biggestObjectMiddle, 2, (255, 0, 0), thickness=2)
            cv2.rectangle(resizedColor, (x, y), (x+w, y+h), (0, 0, 255), thickness=2)
    else:
        stopMessage = stopCommand(stopMessage)
                
    if not ARDUINO_MODE:
        cv2.imshow("video", upscaledColor)
        cv2.imshow("roi", roi(resizedHSV))
        cv2.imshow("mask", mask)
			
cv2.destroyAllWindows()
vs.stop()
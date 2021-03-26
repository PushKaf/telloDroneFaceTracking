import cv2
import numpy as np
from djitellopy import Tello
import time


#NOTE These params should work for most people, but if you're trying to do full body tracking you'll have to tweak it a little bit
w, h = 360, 240 
fbRange = [3600, 5600] #range where it'll get closer or get further
pid = [0.4, 0.4, 0] #proptional integral derrivitive
pError = 0
# startOrNah = 0  #0 to start; 1 to test. test -> wont take off, but other functions like face tracking will still run

drone = Tello()
drone.connect()

drone.for_back_velocity = 0
drone.left_right_velocity = 0
drone.up_down_velocity = 0
drone.yaw_velocity = 0
drone.speed = 0

print(drone.get_battery())
drone.streamoff()
drone.streamon()

drone.takeoff()
drone.send_rc_control(0, 0, 25, 0)
time.sleep(3.5)
drone.send_rc_control(0, 0, 0, 0)

#gets the drone footage resizes it and returns
def getDroneFootage(drone, w=360, h=240):
    frame = drone.get_frame_read().frame
    img = cv2.resize(frame, (w, h))

    return img

#uses open cv2 with the face cascade provided to recognize the face
def findFace(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

    faceListCenter = []
    faceListArea = []

    #used for drawing rectangle on the face and finding the center point
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0,255,0), 2)
        centerX = x + w // 2
        centerY = y + h // 2
        area = w * h
        cv2.circle(img, (centerX, centerY), 7, (0, 0,255), cv2.FILLED)
        faceListCenter.append([centerX, centerY])
        faceListArea.append(area)
        
    if len(faceListArea) != 0:
        i = faceListArea.index(max(faceListArea))
        return img, [faceListCenter[i], faceListArea[i]]
    else:
        return img, [[0, 0], 0]

#sends the commands to the drone to track your face
def trackFace(faceInfo, w, pid, pError):
    area = faceInfo[1]
    x, y = faceInfo[0]
    fb = 0

    error = x - w // 2 #w= width; finding center of face
    speed = pid[0] * error + pid[1] * (error - pError) #determining the yaw; it'll travel at: so it dosent overshoot
    speed = int(np.clip(speed, -100, 100)) #cliping the value so it dosent go over the predefined API limits

    if area > fbRange[0] and area < fbRange[1]:
        fb = 0 #forewards and backwards
    elif area > fbRange[1]: #If the drone is too close, get back
        fb = -25
    elif area < fbRange[0] and area != 0: #If the drone is too far get closer
        fb = 25

    if x == 0:
        speed = 0
        error = 0

    print(speed, fb)

    drone.send_rc_control(0, fb, 0, speed)
    return error

while True:
    # First Step | Get video:
    img = getDroneFootage(drone)
    # Second Step | Find the face:
    img, info = findFace(img)
    #Third Step | Track ET MUG:
    pError = trackFace(info, w, pid, pError)

    #shows what the drone is seeing w/ the face recog and opencv on
    cv2.imshow("Drone Video:", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        drone.land()
        break


#for emergencys if the land command dosent go through or is unsucessfull
emerLand = input("ENTER TO EMERGENCY LAND: ")
drone.emergency()


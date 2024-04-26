import pyOrbSlam
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
cap = cv2.VideoCapture("http://192.168.178.44:4747/video")



vocabPath = "/home/pyOrbSlam3/modules/ORB_SLAM3/Vocabulary/ORBvoc.txt"
settingsPath = "/dockerMount/slamSettings.yaml"

#db = pyOrbSlam.Debug()
#print(db.getPID())
for i in range(10):
    time.sleep(0.5)
    ret, frame = cap.read()
    if frame is not None:
        break
slam = pyOrbSlam.OrbSlam(vocabPath, settingsPath)

#image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
image = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
#slam.ActivateLocalizationMode()
pose = slam.process(image, 0.01)

print(pose)

startTime = time.time()

timeStamp = 0.00
frameNr = 0
while True:
    timeStamp = time.time()-startTime
    frameNr+=1
    ret, frame = cap.read()
    print(frameNr, timeStamp)
    if frame is None:
        break
    if len(frame) == 0:
        break
    #image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    image = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    if frameNr%10 == 1:
        print("FramePoints:")
        print(slam.getFramePoints())
    cv2.imshow("win",image)
    cv2.waitKey(1)
    pose = slam.process(image, timeStamp)
    print(pose)
    if (slam.GetTrackingState() > 2) :
        print(slam.GetTrackingState())
        #time.sleep(0.3)
time.sleep(1.)
slam.shutdown()
time.sleep(10.)
print("ok")
#ret, frame = cap.read()
#
#image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
#image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
#pose = slam.process(image, time)
#
map_ = slam.GetTrackedMapPoints()
#
print(map_)
print("OK")
map_ = slam.GetTrackedMapReferencePoints()
print(map_)
print("OK")
print(slam.getNrOfMaps())
print("OK")
print(slam.getKeyFramesOfMap())
print("OK")



print(time.time()-startTime)
cap.release()

import pyOrbSlam
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
cap = cv2.VideoCapture('SOME_TEST_VIDEO.mp4')

mappingStart = 0
mappingEnd = 2000

vocabPath = "PATH_TO_pyOrbSlam3/pyOrbSlam3/modules/ORB_SLAM3/Vocabulary/ORBvoc.txt"
settingsPath = "PATH_TO_EXAMPLE/test.yaml"

db = pyOrbSlam.Debug()
print(db.getPID())

slam = pyOrbSlam.OrbSlam(vocabPath, settingsPath)
ret, frame = cap.read()
#image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
image = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
#slam.ActivateLocalizationMode()
pose = slam.process(image, 0.01)

print(pose)

startTime = time.time()

timeStamp = 0.00
frameNr = 0
for i in range(max(0,mappingStart-200)):
    timeStamp += 0.05
    frameNr+=1
    ret, frame = cap.read()

while True:
    timeStamp += 0.05
    time.sleep(0.1)
    frameNr+=1
    #if frameNr == mappingStart-200:
    #    slam.DeactivateLocalizationMode()
    if frameNr%50 == 0:
        print(frameNr)
    ret, frame = cap.read()
    if frame is None:
        break
    if len(frame) == 0:
        break
    #image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    image = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    cv2.imshow("win",image)
    cv2.waitKey(4)
    pose = slam.process(image, timeStamp)
    print(pose)
    if frameNr%10 == 1:
        print("FramePoints:")
        print(slam.getFramePoints())
    if (slam.GetTrackingState() > 2) :
        print(slam.GetTrackingState())
        #time.sleep(0.3)
    if (frameNr > mappingEnd):
        break
time.sleep(1.)
slam.shutdown()
time.sleep(10.)
print(db.getPID())
#
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

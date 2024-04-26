import pyOrbSlam
import cv2
import numpy as np
import redis
import time
import json

r = redis.Redis(host='localhost', port=6379, db=0)
p = r.pubsub()
p.subscribe("GrayPinholeImageInput")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_getFrame")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_getMap")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_getOrbState")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_setLocalization")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_shutdown")
time.sleep(0.1)
p.get_message()

vocabPath = "/home/pyOrbslam3/modules/ORB_SLAM3/Vocabulary/ORBvoc.txt"
settingsPath = "/dockerMount/slamSettings.yaml"

#get camera settings:
width = 0
height = 0
with open(settingsPath,"r") as f:
    for line in f.readlines():
        if "Camera.width" in line:
            width = int(line.split(":")[1].strip())
        if "Camera.height" in line:
            height = int(line.split(":")[1].strip())

slam = pyOrbSlam.OrbSlam(vocabPath, settingsPath)


startTime = time.time()

while True:
    retDict = p.get_message()
    if retDict is not None:
        if retDict["channel"] == b"GrayPinholeImageInput":
            data =  np.frombuffer(retDict["data"], dtype=np.uint8)
            data = data.reshape(height,width)
            pose = slam.process(data, time.time()-startTime)
            r.publish("orbSlam_Pose", pose.tobytes())
        if retDict["channel"] == b"orbSlam_getFrame":
            framePoints = slam.getFramePoints()
            r.publish("orbSlam_Frame", framePoints.tobytes())
        if retDict["channel"] == b"orbSlam_getMap":
            map_ = slam.GetTrackedMapReferencePoints()
            r.publish("orbSlam_Map", map_.tobytes())
        if retDict["channel"] == b"orbSlam_getOrbState":
            r.publish("orbSlam_OrbState",slam.GetTrackingState())
        if retDict["channel"] == b"orbSlam_setLocalization":
            slam.ActivateLocalizationMode()
        if retDict["channel"] == b"orbSlam_shutdown":
            slam.shutdown()
            time.sleep(2)
            quit()

import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import redis
import requests

r = redis.Redis(host='localhost', port=6379, db=0)
p = r.pubsub()
p.subscribe("orbSlam_Pose")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_Frame")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_Map")
time.sleep(0.1)
p.get_message()
p.subscribe("orbSlam_OrbState")
time.sleep(0.1)
p.get_message()

ipOfCam = "192.168.1.141"

cap = cv2.VideoCapture("http://"+ipOfCam+":4747/video")

frameNr=0
r.set("orbSlam_newImageAvailable",0)
lastFrame = time.time()
while True:
    if time.time()-lastFrame < 0.1:
        continue
    ret, frame = cap.read()
    if frame is None:
        continue
    if len(frame) == 0:
        continue
    frameNr = frameNr+1
    #image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    image = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    #rectify image to pinholemap
    if r.get("orbSlam_newImageAvailable") == b'0':
        r.set("rawPinholeGrayImage",image.tobytes())
        r.set("orbSlam_newImageAvailable",1)
        r.publish("orbSlam_getFrame",0)
    if frameNr%10 == 1:
        requests.get("http://"+ipOfCam+":4747/cam/1/af")
    retDict = p.get_message()
    framePoints = None
    try:
        data =  np.frombuffer(r.get("orbSlam_Frame"), dtype=np.float32).reshape(-1,8)
        framePoints = data[:,:2]#print("FramePoints: ",data)
    except:
        pass
    try:
        data =  np.frombuffer(r.get("orbSlam_Pose"), dtype=np.float32).reshape(4,4)
        print("Pose: ",data)
    except:
        pass
    
    if retDict is not None:
        if retDict["channel"] == b"orbSlam_Pose":
            data =  np.frombuffer(retDict["data"], dtype=np.float32).reshape(4,4)
            print("Pose: ",data)
        if retDict["channel"] == b"orbSlam_Map":
            data =  np.frombuffer(retDict["data"], dtype=np.float32).reshape(-1,3)
            print("MapPoints: ",data)
        if retDict["channel"] == b"orbSlam_OrbState":
            data =  retDict["data"]
            print("State: ",data)

    image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
    if framePoints is not None:
        for coord in framePoints:
            x, y = coord
            image = cv2.circle(image, (int(x), int(y)), 3, (255,0, 0), -1)
    cv2.imshow("win",image)
    cv2.waitKey(1)
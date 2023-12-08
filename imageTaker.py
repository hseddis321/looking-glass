import cv2 as cv
import cv2.aruco as aruco
import socket 
import numpy as np
import time

index = 0
folder_path = "./calibration_images/"
postfix = "_cal_img.jpg"

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 7
CHARUCOBOARD_COLCOUNT = 5 
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
CHARUCO_BOARD = aruco.CharucoBoard(
        (CHARUCOBOARD_COLCOUNT,CHARUCOBOARD_ROWCOUNT),
        squareLength=0.04,
        markerLength=0.02,
        dictionary=ARUCO_DICT)

last_capture_time = time.time()
cap = cv.VideoCapture(0)
while True:
    ret, frame = cap.read()
    orig_frame = frame.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(
            image=gray,
            dictionary=ARUCO_DICT)
    img = aruco.drawDetectedMarkers(
                image=frame, 
                corners=corners)
    if(ids is not None):
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=CHARUCO_BOARD)
        if response > 20 and time.time() - 0.5 > last_capture_time:
            index += 1
            last_capture_time = time.time()
            cv.imwrite(folder_path + str(index) + postfix, orig_frame)
            print(str(index) + " capture")    
    cv.imshow("out", img)
    if cv.waitKey(1) == ord('q'):
        break
#!/usr/bin/env python3
import numpy as np
import cv2
import glob


# 2.
objp = np.zeros((54,3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

# 3.
objpoints = []  # 3d calibration points
imgpoints = []  # detected points in 2d plan

#img_paths = glob.glob("*.png")
cap = cv2.VideoCapture("output.avi")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
a = 0
# 4.
while cap.grab():
    if True:
        print(a)
        a+=1
        flag, frame = cap.retrieve()
        if not flag:
            continue
        else:
            img = frame
            #img = cv2.resize(frame, (int(360/2),int(288/2)), interpolation = cv2.INTER_AREA)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
            ret, corners = cv2.findChessboardCorners(img, (9, 6), None)
        
            if ret:
        
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                
        
                cv2.drawChessboardCorners(img, (9, 6), corners2, ret)
                cv2.imshow("img", img)
                cv2.waitKey(100)
                if input("Valide : ") == "y":
                    print("True")
                    objpoints.append(objp)
                    imgpoints.append(corners2)
                else:
                    print("False")
        

# 5.
print(len(imgpoints))
print(len(imgpoints[1]))
print(len(objpoints[1]))

# 6.
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# mtx: calibration matrix - fx, fy, cx, cy
# dist: distortion parameters
# rvecs: rotation for each view
# tvecs: translation for each view

# 7.

# 8.
np.savez('calib_parameters', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# 9.
img = cv2.imread('frame_3.png')
dst = cv2.undistort(img, mtx, dist, None)
cv2.imshow('before correction', img)
cv2.waitKey(0)
cv2.imshow('after correction', dst)
cv2.waitKey(0)

import cv2
import numpy as np

def traite(img):
	frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	frame_threshold = cv2.inRange(frameHSV, (300/2, 10, 10), (360/2, 256, 256));
	frame_threshold += cv2.inRange(frameHSV, (0/2, 10, 10), (30/2, 256, 256));
	cv2.imshow('HSV', frame_threshold)


cap = cv2.VideoCapture("output.avi")


while cap.grab():
	flag, frame = cap.retrieve()
	if not flag:
		continue
	else:
		cv2.imshow("img", frame)
		traite(frame)
		cv2.waitKey(10)

"""
frame = cv2.imread("frame_3.png")
traite(frame)
"""



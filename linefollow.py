from __future__ import print_function
import numpy as np
import cv2 as cv
import argparse
 
max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def onChange(x):
    print("ValueCount")
    pass

cap = cv.VideoCapture(1)
if not cap.isOpened():
 print("Cannot open camera")
 exit()
while True:
 # Capture frame-by-frame
 ret, frame = cap.read()
 
 # if frame is read correctly ret is True
 if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break
 
 cv.namedWindow(window_capture_name)
 cv.namedWindow(window_detection_name)

 cv.createTrackbar("Low H", 'Object Detection', 0, 180, onChange)


 # Our operations on the frame come here
 frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
 frame_threshold = cv.inRange(frame_HSV, (80, low_S, low_V), (100, high_S, high_V))
 # Display the resulting frame

 cv.imshow(window_capture_name, frame)
 cv.imshow(window_detection_name, frame_threshold)

 if cv.waitKey(1) == ord('q'):
    break
 
# When everything done, release the capture
cap.release()
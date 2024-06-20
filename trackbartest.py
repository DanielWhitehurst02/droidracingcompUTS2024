import numpy as np
import cv2 as cv


window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'


def nothing():
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

 cv.createTrackbar("Low H", window_capture_name, 0, 180, nothing)


 # Our operations on the frame come here
 frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
 frame_threshold = cv.inRange(frame_HSV, (80, 0, 0), (100, 255, 255))
 # Display the resulting frame

 cv.imshow(window_capture_name, frame)


 if cv.waitKey(1) == ord('q'):
    break
 
# When everything done, release the capture
cap.release()

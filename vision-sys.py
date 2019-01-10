## vision-sys.py 
# version 0.1

import cv2
import numpy as np

cap = cv2.VideoCapture(1)
cv2.namedWindow('filter')

def nothing(x):
    pass

cv2.createTrackbar('H_MAX','filter',255,255,nothing)
cv2.createTrackbar('H_MIN','filter',0,255,nothing)
cv2.createTrackbar('S_MAX','filter',255,255,nothing)
cv2.createTrackbar('S_MIN','filter',0,255,nothing)
cv2.createTrackbar('V_MAX','filter',255,255,nothing)
cv2.createTrackbar('V_MIN','filter',0,255,nothing)

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    H = cv2.getTrackbarPos('H_MAX','filter')
    S = cv2.getTrackbarPos('S_MAX','filter')
    V = cv2.getTrackbarPos('V_MAX','filter')
    h = cv2.getTrackbarPos('H_MIN','filter') 
    s = cv2.getTrackbarPos('S_MIN','filter')
    v = cv2.getTrackbarPos('V_MIN','filter')

    upper = np.array([H,S,V])
    lower = np.array([h,s,v])

    mask = cv2.inRange(hsv, lower, upper)

    res = cv2.bitwise_and(frame,frame, mask= mask)


    cv2.imshow('BRG Image', frame)
    cv2.imshow('HSV Image', hsv)
    cv2.imshow('Result', res)
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
import numpy as np
import cv2
import time
import copy

cap = cv2.VideoCapture(0)

count = 0
start = time.time()
cv2.namedWindow("Channels")
cv2.namedWindow("Main")

rg = 1.7
rb = 1.7
gr = 1.1
gb = 1.1

while(True):
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()

#    frame2 = copy.deepcopy(frame)
    frame2 = copy.copy(frame)
    b,g,r = cv2.split(frame2) 
    frame2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frame2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    frame2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]

    imgray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,127,255,127)
    _, contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame2, contours, -1, (255,0,0), 3)


    # Display the resulting frame
    cv2.imshow("Channels", frame)
    cv2.imshow("Main", frame2)

    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

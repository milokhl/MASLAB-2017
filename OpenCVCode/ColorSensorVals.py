import numpy as np
import cv2
import time
import copy

cap = cv2.VideoCapture(0)
#cap.set(3,100)
#cap.set(4,60)

count = 0
start = time.time()
cv2.namedWindow("Channels")
cv2.namedWindow("Main")

rg = 1.65 # red to green ratio
rb = 2.0
gr = 1.2
gb = 1.25
maxgr = 1.5

while(True):
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame2 = copy.copy(frame)

    b,g,r = cv2.split(frame2) 
    frame2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frame2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    frame2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow("Channels", frame)
    cv2.imshow("Main", frame2)
    
    #cv2.imshow('frame',frame2)
    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

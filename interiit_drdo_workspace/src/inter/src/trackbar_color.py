import cv2
import numpy as np

def empty(a):
    pass
cv2.namedWindow("t")
cv2.resizeWindow("t",640,240)
cv2.createTrackbar("hi","t",0,179,empty)
cv2.createTrackbar("ha","t",179,179,empty)
cv2.createTrackbar("si","t",0,255,empty)
cv2.createTrackbar("sa","t",255,255,empty)
cv2.createTrackbar("vi","t",0,255,empty)
cv2.createTrackbar("va","t",255,255,empty)


while True:
    img = cv2.imread("car1.png")
    #img = cv2.imread("b2.jpeg")
    #img = img[1400:1900,1700:2500,:]
    imghsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    hi = cv2.getTrackbarPos("hi","t")
    ha = cv2.getTrackbarPos("ha","t")
    si = cv2.getTrackbarPos("si","t")
    sa = cv2.getTrackbarPos("sa","t")
    vi = cv2.getTrackbarPos("vi","t")
    va = cv2.getTrackbarPos("va","t")

    lower = np.array([hi,si,vi])
    upper = np.array([ha,sa,va])
    mask = cv2.inRange(imghsv,lower,upper)
    imgr = cv2.bitwise_and(img,img,mask=mask)

    #cv2.imshow("img",imghsv)
    cv2.imshow("ball",imgr)
    cv2.imshow("m",mask)
    cv2.waitKey(1)
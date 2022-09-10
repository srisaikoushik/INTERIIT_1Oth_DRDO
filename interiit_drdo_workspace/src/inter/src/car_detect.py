import cv2
import numpy as np

img = cv2.imread("car2.png")
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#blue color hsv limits
lower = np.array([116,187,203])
upper = np.array([132,255,255])
mask = cv2.inRange(img_hsv,lower,upper)
#print(mask.shape)

edged = cv2.Canny(mask, 170, 255) #Determine edges of objects in an image
ret,thresh = cv2.threshold(edged,240,255,cv2.THRESH_BINARY)  
contours = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

center = contours[1][0]

print(center.shape)

if r>0:
    M = cv2.moments(center)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        # cv2.drawContours(img, [center], -1, (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
    cv2.imshow("img",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
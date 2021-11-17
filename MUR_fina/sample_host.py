import conLib as connect;
import cv2
import cv2 as cv
import numpy as np
session = connect.host(4219)
session.accept()



def nothing(self):
    pass
window_name = "track_img"
cv.namedWindow(window_name)

cv.createTrackbar('UH', window_name, 0, 255, nothing)
cv.setTrackbarPos('UH', window_name, 255)

cv.createTrackbar('US', window_name, 0, 255, nothing)
cv.setTrackbarPos('US', window_name, 255)

cv.createTrackbar('UV', window_name, 0, 255, nothing)
cv.setTrackbarPos('UV', window_name, 255)

# create trackbars for Lower HSV
cv.createTrackbar('LH', window_name, 0, 255, nothing)
cv.setTrackbarPos('LH', window_name, 0)

cv.createTrackbar('LS', window_name, 0, 255, nothing)
cv.setTrackbarPos('LS', window_name, 0)

cv.createTrackbar('LV', window_name, 0, 255, nothing)
cv.setTrackbarPos('LV', window_name, 0)
window_name = "track_img"
while True:
    ret, image = session.read()
    cv2.imshow("cap", image)
    uh = cv.getTrackbarPos('UH', window_name)
    us = cv.getTrackbarPos('US', window_name)
    uv = cv.getTrackbarPos('UV', window_name)
    upper_blue = np.array([uh, us, uv])
    # get current positions of Lower HSCV trackbars
    lh = cv.getTrackbarPos('LH', window_name)
    ls = cv.getTrackbarPos('LS', window_name)
    lv = cv.getTrackbarPos('LV', window_name)
    upper_hsv = np.array([uh, us, uv])
    lower_hsv = np.array([lh, ls, lv])
    print(upper_hsv, lower_hsv)
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    cv.imshow("HSV", mask)
    cv.waitKey(5)
    session.write((',0,,0' + ',\n').zfill(20).encode())
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()
session.close() 

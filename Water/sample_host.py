from conLib import host
import cv2 as cv
import numpy as np
import math


def start_client():
    import paramiko
    host = "10.3.141.1"
    user = 'pi'
    secret = 'raspberry'
    port = 22
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=host, username=user, password=secret, port=port)
    stdin, stdout, stderr = client.exec_command('python3 sample_client.py 192.168.0.14')
    client.close()


def constrain(x, a, b):
    if x < a:
        return a
    elif b < x:
        return b
    else:
        return x

mi = 0
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


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

cv.createTrackbar('Min', window_name, 0, 255, nothing)
cv.setTrackbarPos('Min', window_name, 0)
window_name = "track_img"

start_client()
cap = host(4221)
cap.accept()

hsv_min = np.array((0, 0, 255), np.uint8)
hsv_max = np.array((72, 51, 255), np.uint8)

color_blue = (255, 0, 0)
color_red = (0, 0, 128)
def rect(frame):
    
    global hsv_max, hsv_min, mi
    blur = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    hsv_mask = cv.inRange(hsv, hsv_min, hsv_max)
    #contours0, hierarchy = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    
    global mi
    img = frame.copy()
    #crop = cv.resize(frame[:40], (0, 0), fx=0.5, fy=0.5)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    ret, thresh_mask = cv.threshold(blur, mi, 255, cv.THRESH_BINARY_INV)
    thresh_mask = 255 - thresh_mask
    mask = np.zeros_like(thresh_mask)
    for m in range(len(mask)):
        for k in range(len(mask[0])):
            if thresh_mask[m][k] and hsv_mask[m][k]:
                mask[m][k] = 255
    contours0, hierarchy = cv.findContours(mask.copy(), 1, cv.CHAIN_APPROX_NONE)
    cv.imshow("HSV", hsv_mask)
    cv.imshow("thresh", thresh_mask)
    cv.imshow("mask", mask)


    for cnt in contours0:
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        center = (int(rect[0][0]), int(rect[0][1]))
        area = int(rect[1][0]*rect[1][1])

        edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
        edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

        usedEdge = edge1
        if cv.norm(edge2) > cv.norm(edge1):
            usedEdge = edge2

        reference = (1, 0)  # horizontal edge
        angle = 180.0/math.pi * \
            math.acos((reference[0]*usedEdge[0] + reference[1] *
                        usedEdge[1]) / (cv.norm(reference) * cv.norm(usedEdge)))

        if area > 500:
            cv.drawContours(img, [box], 0, color_blue, 2)
            cv.circle(img, center, 5, color_red, 2)
            cv.putText(img, "%d" % int(
                angle), (center[0]+20, center[1]-20), cv.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
        cv.imshow('result', img)

def tBar():
    uh = cv.getTrackbarPos('UH', window_name)
    us = cv.getTrackbarPos('US', window_name)
    uv = cv.getTrackbarPos('UV', window_name)
    lh = cv.getTrackbarPos('LH', window_name)
    ls = cv.getTrackbarPos('LS', window_name)
    lv = cv.getTrackbarPos('LV', window_name)
    mi = cv.getTrackbarPos('Min', window_name)
    hsv_max = np.array([uh, us, uv])
    hsv_min = np.array([lh, ls, lv])
    return hsv_min, hsv_max, mi
while True:
    ret, image = cap.read()
    cv.imshow("cap", image)
    hsv_min, hsv_max, mi = tBar()
    rect(image)
    if cv.waitKey(1) == ord('q'):
        break
cv.destroyAllWindows()
cap.close() 

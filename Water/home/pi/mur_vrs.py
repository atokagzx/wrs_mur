import numpy as np
import pymurapi
import time
import cv2 as cv
#[ 33 255 255] [ 16 183  79]
def constrain(v, min, max):
    if v < min:
        return min
    if v > max:
        return max
    return v
def rotate(val):
    if val >= 180:
        return val - 360
    elif val <= -180:
        return val + 360
    else:
           return val
class PID:
    def __init__(self, P, I, D):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = 0.0
        self.clear()

    def clear(self):
        #self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
        return self.output;
        
    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    def setSetPoint(self, set_point):
        self.SetPoint = set_point

class function():
    def __init__(self, mur):
        self.mur = mur
        from time import time
        self.time = time
        self.speed = 0
        self.angle = 0
        self.depth = 0
        self.res = (320, 240)
        self.depth_timer = self.time()
        self.angle_timer = self.time()
        self.pid_depth = PID(250, 0, 50)
        self.pid_angle = PID(0.9, 0, 0.25)
        self.pid_bottom_x = PID(0.33, 0.3, 0.3)
        self.pid_bottom_y = PID(0.3, 0.3, 0.3)
        self.pid_angle.setSetPoint(0)
        self.pid_bottom_x.setSetPoint(160)
        self.pid_bottom_y.setSetPoint(120)
    def keep_depth(self, target):
        if target is None:
            self.mur.set_motor_power(0, 0)
            self.mur.set_motor_power(3, 0)
        elif (self.time() - self.depth_timer) >= 0.2:
            target -= 0.18
            out = -int(self.pid_depth.update(self.mur.get_depth() + target))
            '''
            if out >= 0:
                out += 20
            else:
                out -= 20
            '''
            self.mur.set_motor_power(0, constrain(int(out), -100, 100))
            self.mur.set_motor_power(3, -constrain(int(out), -100, 100))
            self.depth_timer = self.time()
    def keep_angle(self, target, speed = None):
        if not speed is None:
            self.speed = speed
        if target is None:
            mur.set_motor_power(1, 0)
            mur.set_motor_power(2, 0)
        elif (self.time() - self.angle_timer) >= 0.2 and (not self.angle is None):
            angle_l = rotate(mur.get_yaw() - target)
            out = int(self.pid_angle.update(angle_l))
            #print(out)
            mur.set_motor_power(1, constrain(out + self.speed, -100, 100))
            mur.set_motor_power(2, -constrain(-out + self.speed, -100, 100))
            self.angle_timer = self.time()
    def set(self, angle = None, depth = None, speed = 0):
        offset_timer = self.time()
        self.speed = 0
        from time import sleep
        self.pid_bottom_x.clear()
        self.pid_bottom_y.clear()
        self.pid_angle.clear()
        while True:
            if not depth is None:
                if abs(self.mur.get_depth() + depth) >= 0.05:
                    offset_timer = self.time()
            if not angle is None:
                if abs(abs(self.mur.get_yaw()) - abs(angle)) >= 5:
                    offset_timer = self.time()
            if self.time() - offset_timer >= 0.6:
                break
            self.keep_angle(angle)
            self.keep_depth(depth)
            #print("setting")
            sleep(0.1)
        self.speed = speed
class vision():
    
    def __init__(self, cv):
        import numpy
        self.cv = cv
        self.np = numpy

    def obj(self, image, hsv):
        from math import pi
        np = self.np
        cv2 = self.cv
        cv = self.cv
        hsv_l = hsv[0]
        hsv_u = hsv[1]
        img = image.copy()
        img = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask_ = cv.inRange(img_hsv, hsv_l, hsv_u)

        contours, hierarchy = cv2.findContours(mask_, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        n = 0
        for i in contours:
            size = cv2.contourArea(i)
            rect = cv2.minAreaRect(i)
            if 100 < size < 10000:
                if len(i) < 5:
                    continue
                bEllipse = cv.fitEllipse(i)
                hull = cv.convexHull(i, True)
                hull = cv.approxPolyDP(hull, 15, True)
                if (not cv.isContourConvex(hull)):
                    continue
                #angle - bEllipse[2]
                
                if len(hull) == 4:
                    cv2.imshow('Mask', mask_)
                    cv2.waitKey(1)
                    return int(bEllipse[0][0]), int(bEllipse[0][1])
cap = cv.VideoCapture(0)
#_, frame = cap.read()
#cv.imshow("img.jpg", frame)
if __name__ == '__main__':
    line = np.array([np.array([16, 183, 79]), np.array([33, 255, 255])])
    log = open("//home//pi//logger.txt", "w")
    mur = pymurapi.mur_init()
    v = vision(cv)
    f = function(mur)

    f.set(10, -0.15, -50)
    timer = time.time()
    while time.time() - timer < 5 :
        print(mur.get_yaw())
    _, img = cap.read()
    log.close()
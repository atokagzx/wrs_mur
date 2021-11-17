import cv2
import numpy as np
from conLib import client

try:
    addr = sys.argv[1]
except:
    addr = "10.3.141.84"
frame_size = [336, 256]
cap = cv2.VideoCapture(0)

session = client(addr, 4221)
session.connect()
print(addr)
#time.sleep(1)

while True:
    ret, frame = cap.read()
    session.send(frame)
    

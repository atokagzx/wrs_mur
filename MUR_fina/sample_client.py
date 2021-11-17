import socket
import cv2
import numpy as np
import socket


class host:
    def __init__(self, port):
        self.sock = socket.socket()
        self.sock.bind(('', port))
        self.sock.listen(1)

    def accept(self):
        self.sock.settimeout(99999)
        self.conn, self.addr = self.sock.accept()
        self.conn_f = self.conn.makefile('rb')
        print("Connection library: ip: " + str(self.addr[0]))
        self.data = b' '
        self.sock.settimeout(2)

    def read(self):

        while True:
            try:
                self.data += self.conn_f.read(1024)
            except:
                print("Connection library: TimeoutError, reconnecting...")
                self.accept()
                return 0, np.zeros((1, 1))
            first = self.data.find(b'\xff\xd8')
            last = self.data.find(b'\xff\xd9')
            if first != -1 and last != -1:
                jpg = self.data[first:last + 2]
                self.data = self.data[last + 2:]
                return 1, cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def write(self, stroke):
        self.conn.send(stroke)

    def close(self):
        self.conn.close()


class client:
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.sock = socket.socket()
        self.sock.settimeout(None)

    def connect(self):
        try:
            self.sock.connect((self.addr, self.port))
            self.conn = self.sock.makefile('wb')
            self.sock.settimeout(5)
            return 1
        except:
            return 0

    def send(self, frame):
        stream = cv2.imencode('.jpg', frame)[1].tostring()
        self.conn.write(stream)

    def read(self):
        try:
            data = self.sock.recv(20).decode().lstrip('0')
            return data
        except:
            return None

import time
import sys
import cv2

try:
    addr = sys.argv[1]
except:
    addr = "10.3.141.84"
    
frame_size = [336, 256]
cap = cv2.VideoCapture(0)

session = client(addr, 4219)
while True:
    if session.connect():
        break
print(addr)
time.sleep(1)

while True:
    ret, frame = cap.read()
    session.send(frame)
    data = session.read()
    print(data)
    if not(data is None):
        print(data.encode())
    else:
        print(",0,,0,\n")
    

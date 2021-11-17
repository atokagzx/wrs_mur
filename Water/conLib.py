import cv2
import numpy as np
from socket import socket

class host:
    def __init__(self, port):
        self.sock = socket()
        self.sock.bind(('', port))
        self.sock.listen(1)
        self.sock.setblocking(0)
        self.data = b''

    def accept(self):
        self.sock.settimeout(None)
        self.conn, self.addr = self.sock.accept()
        self.rx = self.conn.makefile('rb')
        print("Connection library: ip: " + str(self.addr[0]))

    def read(self):
        while True:
            self.data += self.rx.readline()
            first = self.data.find(b'\xff\xd8')
            last = self.data.find(b'\xff\xd9')
            if first != -1 and last != -1:
                jpg = self.data[first:last + 2]
                self.data = self.data[last + 2:]
                return 1, cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def close(self):
        self.conn.close()

class client:
    def __init__(self, addr, port, ):
        self.addr = addr
        self.port = port
        self.sock = socket()
        self.sock.settimeout(30)
        self.data = b''

    def connect(self):
        self.sock.connect((self.addr, self.port))
        self.tx = self.sock.makefile('wb')
        self.sock.settimeout(None)
        self.sock.setblocking(1)
        
    def send(self, frame):
        stream = cv2.imencode('.jpg', frame)[1].tostring()
        self.tx.flush()
        self.tx.write(stream)

    def close(self):
        self.sock.close()
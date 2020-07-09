import os, sys
import socket
import numpy as np


class SMFCBase:
    def __init__(self, ip, port=9999):
        """
        Init the server with port and root directory
        """
        # create socket and bind it to IP and port
        self.websocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM, fileno=None)
        self.websocket.bind((ip, port))
        self.back_count = 0

    def __del__(self):
        self.websocket.close()
        print("Server stopped")

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        self.websocket.listen()
        while 1:
            conn, addr = self.websocket.accept()
            #print(f"Connection from address {addr}")
            # start a new thread for every connection
            self.handle_request(conn)

    def handle_request(self, request):
        """
        Handle requests from the given connection until timout occurs
        """
        # self.request is the TCP socket connected to the client
        self.data1 = np.int32(request.recv(32))
        dist1 = (self.data1 & (2**16-1))/10
        dist2 = (self.data1 >> 16)/10
        #self.data1_fl = float(self.data1.strip().decode())/10
        #self.data2 = self.request.recv(1024).strip().decode()
        ins = self.handle_locdata(dist1, dist2)
        request.send(ins)
        print(dist1, dist2)
        request.close()

    def handle_locdata(self, dat1, dat2):
        # 5 stop and go cycles -> ~180 deg turn
        if dat2 < 40:
            fak = 2
        else:
            fak = 1
        bc_1 = [b"2", b"2", b"0", b"0"] * fak
        bc_2 = [b"2", b"0"] * 5
        bc_lookup = [b"2"] * 10 + bc_1
        bc_max = len(bc_lookup)
        if dat1 < 30 or self.back_count > 0:
            if bc_max >= self.back_count > 0:
                self.back_count -= 1
                ins = bc_lookup[self.back_count]
            else:
                self.back_count = bc_max
                ins = b"0"
        else:
            ins = b"2"
        return ins


if __name__ == "__main__":
    port = 9999
    ip = "0.0.0.0"
    hs = SMFCBase(ip, port)
    hs.serve()

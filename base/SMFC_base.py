import time
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
        self.logfile = 0

    def __del__(self):
        self.websocket.close()
        if self.logfile:
            self.logfile.close()
        print("Server stopped")

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        self.websocket.listen()
        while 1:
            conn, addr = self.websocket.accept()
            self.handle_request(conn)

    def savelocdata(self, data1, data2, data3, ins):
        if not os.path.isfile("rb_datalog.csv"):
            with open("rb_datalog.csv", "w") as lf:
                lf.write("t;s1;s2;s3;ins;\\n")
        if not self.logfile:
            self.logfile = open("rb_datalog.csv", "a")
        self.logfile.write(";".join([str(x) for x in [time.time(), data1, data2, data3, ins, "\\n"]]))
        
            

    def handle_request(self, request):
        """
        Handle requests from the given connection until timout occurs
        """
        # self.request is the TCP socket connected to the client
        data_rcv = request.recv(48).hex()

        dist1 = (int(data_rcv[0:4], 16))/10
        dist2 = (int(data_rcv[4:8], 16))/10
        dist3 = (int(data_rcv[8:12], 16))/10
        ins = self.handle_locdata(dist1, dist2, dist3)
        #ins = b"2"
        request.send(ins)
        self.savelocdata(dist1, dist2, dist3, ins)
        print(dist1, dist2, dist3)
        request.close()

    def handle_locdata(self, dat1, dat2, dat3):
        # 5 stop and go cycles -> ~180 deg turn
        if dat2 < 40 or dat3 < 40:
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

import time
import os, sys
import socket
import numpy as np
import paho.mqtt.client as mqtt
import getch


class SMFCBase(mqtt.Client):
    def __init__(self, ip, port=9999):
        """
        Init the server with port and root directory
        """
        super().__init__(client_id="SMFC_base")
        self.connect("192.168.178.27", port=1883)
        self.loop_start()
        self.back_count = 0
        self.logfile = 0
        self.ins_list = []
        self.wasd = [0,0,0]

    def __del__(self):
        if self.logfile:
            self.logfile.close()
        print("Server stopped")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        self.subscribe("RR/sensors")

    def on_message(self, client, userdata, msg):
        print(f"{msg.topic}: {msg.payload}")
        ins = [int(msg.payload.decode()),0]
        self.ins_list.append(ins)

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        while 1:
            #key_in  = keyboard.read_key()
            key_in = getch.getch()
            k_ins = self.handle_keyin(key_in)
            self.ins_list.append(k_ins)
            if len(self.ins_list):
                for ins in self.ins_list:
                    self.publish("RR/driveIns", self.conv_ar_to_msg(ins))
                self.ins_list = []
            time.sleep(0.001)

    def handle_keyin(self, key):
        fwd, l, r = self.wasd
        print(key)
        r = 1024
        l = 1023
        if key == "w":
            fwd = 1
        elif key == "s" and fwd == 1:
            fwd = 0
        elif key == "s":
            fwd = -1
        elif key == "a":
            l = 0
            r = 1024
        elif key == "d":
            r = 0
            l = 1024
        self.wasd = [fwd, 0, 0]
        return [l*fwd, r*fwd]


    def savelocdata(self, data1, data2, data3, ins):
        if not os.path.isfile("rb_datalog.csv"):
            with open("rb_datalog.csv", "w") as lf:
                lf.write("t;s1;s2;s3;ins;\n")
        if not self.logfile:
            self.logfile = open("rb_datalog.csv", "a")
        self.logfile.write(";".join([str(x) for x in [time.time(), data1, data2, data3, int(ins), "\n"]]))
        
            
    def conv_ar_to_msg(self, ar):
        pwrL = (ar[0]+1024).to_bytes(2, "little")
        pwrR = (ar[1]+1024).to_bytes(2, "little")
        msg = pwrL+pwrR
        print(f"conv msg: {msg}")
        return msg

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

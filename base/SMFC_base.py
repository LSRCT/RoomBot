import time
import os, sys
import socket
import paho.mqtt.client as mqtt
import numpy as np

class SMFCBase(mqtt.Client):
    def __init__(self, ip, port=1883):
        """
        Init the server with port and root directory
        """
        super().__init__(client_id="SMFC_base")
        # connect to mqtt broker
        self.connect(ip, port=port)
        self.loop_start()
        self.back_count = 0
        self.ins_list = []
        # create a logfile if it doesnt exist
        if not os.path.isfile("rb_datalog.csv"):
            with open("rb_datalog.csv", "w") as lf:
                lf.write("t;s1;s2;s3;ins;\n")
        self.logfile = open("rb_datalog.csv", "a")

    def __del__(self):
        if self.logfile:
            self.logfile.close()
        print("Server stopped")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        self.subscribe("RR/sensors")

    def on_message(self, client, userdata, msg):
        #print(f"{msg.topic}: {msg.payload}")
        data_rcv = msg.payload.hex()
        dist1 = (int(data_rcv[0:4], 16))/100
        dist2 = (int(data_rcv[4:8], 16))/100
        dist3 = (int(data_rcv[8:12], 16))/100
        print(dist1, dist2, dist3)
        ins = self.handle_locdata(dist1, dist2, dist3)
        self.savelocdata(dist1, dist2, dist3, ins)

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        while 1:
            if len(self.ins_list):
                for ins in self.ins_list:
                    self.publish("RR/driveIns", ins)
                self.ins_list = []
            time.sleep(0.001)

    def savelocdata(self, data1, data2, data3, ins):
        self.logfile.write(";".join([str(x) for x in [time.time(), data1, data2, data3, int(ins), "\n"]]))
        self.logfile.flush()

    def conv_ar_to_msg(self, ar):
        pwrL = (ar[0]+1024).to_bytes(2, "little")
        pwrR = (ar[1]+1024).to_bytes(2, "little")
        msg = pwrL+pwrR
        print(f"conv msg: {msg}")
        return msg
    
    def handle_locdata(self, dat1, dat2, dat3):
        # 5 stop and go cycles -> ~180 deg turn
        if dat2 < 40 or dat3 < 40:
            fak = 2
        else:
            fak = 1
        bc_1 = [b"0", b"0", b"2", b"2"] * fak
        bc_2 = [b"0", b"2"] * 5
        bc_lookup = [b"0"] * 10 + bc_1

        bc_max = len(bc_lookup)
        if dat1 < 30 or self.back_count > 0:
            if bc_max >= self.back_count > 0:
                self.back_count -= 1
                ins = bc_lookup[self.back_count]
            else:
                self.back_count = bc_max
                ins = b"2"
        else:
            ins = b"0"
        self.ins_list.append(ins)
        return ins


if __name__ == "__main__":
    # mqtt broker port
    port = 1883
    # mqtt broker ip
    ip = "192.168.178.27"
    hs = SMFCBase(ip, port)
    hs.serve()

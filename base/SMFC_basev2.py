import time
import os, sys
import numpy as np
import paho.mqtt.client as mqtt
import getch


class SMFCBase(mqtt.Client):
    def __init__(self, ip, port=1883, manual=0):
        """
        Init the server with port and root directory
        :param ip: IP of the mqtt broker
        :param port: Port for mqtt broker
        :param manual: Manual controll on/off
        """
        super().__init__(client_id="SMFC_base")
        # connect to mqtt broker
        self.connect("192.168.178.27", port=port) 
        self.control_mode = manual 
        self.loop_start()
        self.back_count = 0
        self.ins_list = []
        self.wasd = [0, 0, 0, 0]

        # magnetometer offset
        self.mag_offset_X = -(154+214)/2
        self.mag_offset_Y = -(227+289)/2
        self.phi_recent = 0

        # create a logfile if it doesnt exist
        if not os.path.isfile("rb_datalog.csv"):
            with open("rb_datalog.csv", "w") as lf:
                lf.write("t;s1;s2;s3;phi;ins;\n")
        self.logfile = open("rb_datalog.csv", "a")

    def __del__(self):
        if self.logfile:
            self.logfile.close()
        print("Server stopped")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        self.subscribe("RR/sensors")
        self.subscribe("RR/magnetometer")

    def on_message(self, client, userdata, msg):
        if msg.topic == "RR/sensors": 
            self.on_message_sensors(msg)
        elif msg.topic == "RR/magnetometer":
            self.on_message_mag(msg)

    def on_message_sensors(self,msg):
        """
        Callback when distance sensor message is recieved
        Decode the recieved message to 3 distances
        """
        data_rcv = msg.payload.hex()
        dist1 = ((int(data_rcv[0:4], 16))/10)
        dist2 = ((int(data_rcv[4:8], 16))/10)
        dist3 = ((int(data_rcv[8:12], 16))/10)
        if dist1 > 2000:
            dist1 = 1
        if dist2 > 2000:
            dist2 = 1
        if dist3 > 2000:
            dist3 = 1
        if not self.control_mode:
            self.handle_locdata(dist1, dist2, dist3)
        print(dist1, dist2, dist3)
        #self.savelocdata(dist1, dist2, dist3, self.phi_recent, ins)

    def on_message_mag(self,msg):
        """
        Callback when magnetometer msg is recieved.
        Calculates angle from the raw data, accounting for offset and scaling
        """
        data_rcv = msg.payload.hex()
        magX = (int(data_rcv[0:4], 16)-200)/10
        magY = (int(data_rcv[4:8], 16)-200)/10

        # offset due to manufactoring
        magX += self.mag_offset_X
        magY += self.mag_offset_Y

        # Scale because we are not at the equator
        magX *= 30/33
        magY *= 30/31
        
        # get angle from complex
        phi = np.arctan2(magX, magY)*180/np.pi

        # make sure 0 deg is same as 0 deg on map
        phi += 20

        # -180:180 -> 0:360
        if phi < 0:
            phi += 360
        self.phi_recent = phi

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        while 1:
            if self.control_mode:
                key_in = getch.getch()
                k_ins = self.handle_keyin(key_in)
                print(key_in)
                self.ins_list.append(k_ins)
            if len(self.ins_list):
                for ins in self.ins_list:
                    self.publish("RR/driveIns", self.conv_ar_to_msg(ins))
                self.ins_list = []
            time.sleep(0.001)

    def handle_keyin(self, key):
        """
        Determine instruction from key input for manual controll
        """
        fwdL, fwdR, l, r = self.wasd
        print(key)
        r = 1024
        l = 1024
        if key == "w":
            fwdL = 1
            fwdR = 1
        elif key == "s" and fwdL == 1:
            fwdL = 0
            fwdR = 0
        elif key == "s":
            fwdR = -1
            fwdL = -1
        elif key == "a":
            l = 0
            r = 1024
        elif key == "d":
            r = 0
            l = 1024
        elif key == "q":
            fwdL = -1
            fwdR = 1
        elif key == "e":
            fwdL = 1
            fwdR = -1
        self.wasd = [fwdL, fwdR, 0, 0]
        return [l*fwdL, r*fwdR]


    def savelocdata(self, data1, data2, data3, phi, ins):
        """
        Save the sensor data to a csv file
        """
        self.logfile.write(";".join([str(x) for x in [time.time(), data1, data2, data3, phi, int(ins), "\n"]]))
        self.logfile.flush()
            
    def conv_ar_to_msg(self, ar):
        """
        Encode an instruction for the robot
        :param ar: Instruction to encode, [left, right]
        """
        pwrL = (ar[0]+1024).to_bytes(2, "little")
        pwrR = (ar[1]+1024).to_bytes(2, "little")
        msg = pwrL+pwrR
        return msg


    def handle_locdata(self, dat1, dat2, dat3):
        """
        React to new sensordata.
        Basically modify the instruction list
        """
        r = 1024
        l = 1024
        bc_1 = [[-1*l, -1*r]]
        bc_2 = [[-1*l, 1*r]]
        bc_lookup = bc_2 * 10 + bc_1 * 10
        bc_max = len(bc_lookup)
        if dat1 < 30 or self.back_count > 0:
            if bc_max >= self.back_count > 0:
                self.back_count -= 1
                ins = bc_lookup[self.back_count]
            else:
                self.back_count = bc_max
                ins = [-1*l, -1*r]
        else:
            ins = [1*l, 1*r]
        self.ins_list.append(ins)
        return ins


if __name__ == "__main__":
    # manual mode command line argument
    manual = 0
    if len(sys.argv) > 1:
        if sys.argv[1] == "manual":
            manual = 1
    # mqtt broker port
    port = 1883
    # mqtt broker ip
    ip = "192.168.178.27"
    hs = SMFCBase(ip, port, manual=manual)
    hs.serve()
